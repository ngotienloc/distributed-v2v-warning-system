/* task/task_display_tft.c — Hiển thị radar V2V và cảnh báo lên màn hình TFT 128×160.
 *
 * Chạy ở 10 Hz. Mỗi frame:
 *   1. Drain q_tft_collision → s_ci (danh sách ego + peers).
 *   2. Drain q_alert_tft     → s_alert (cảnh báo hiện tại).
 *   3. Vẽ tối ưu: chỉ vẽ đè phần động, xe và mặt đường đứng yên.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "drivers/tft/tft_driver.h"
#include "../config.h"
#include "fusion/geo_utils/geo_utils.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

static const char *TAG = "task_tft";

/* ── Hằng số hình học ADAS 3D Road ───────────────────────────────────── */
#define ADAS_HY       25          /* Điểm chân trời Y (Horizon Y) */
#define ADAS_BY       120         /* Điểm đáy đường Y (Bottom Y) */
#define ADAS_H_SPAN   95          /* Chiều cao vùng đường (120 - 25) */
#define ADAS_MAX_W    49          /* Bán độ rộng lớn nhất của đường ở đáy */

/* Phân vùng màn hình */
#define STATUSBAR_H   0           /* Bỏ thanh trạng thái phía trên */
#define FOOTER_Y      120
#define FOOTER_H      (CFG_TFT_HEIGHT - FOOTER_Y)  /* 40px */

/* ── Trạng thái nội bộ ───────────────────────────────────────────────── */
static collision_input_t s_ci              = {0};
static alert_result_t    s_alert           = {0};
static bool              s_alert_active     = false;
static uint32_t          s_alert_until      = 0;  /* hiển thị cảnh báo tối thiểu 2 giây */
static uint8_t           s_frame_cnt        = 0;  /* Biến đếm khung hình để tạo hoạt ảnh */

/* Trạng thái lưu vết render */
static bool              s_need_full_redraw = true;
static bool              s_last_alert_active = false;
static alert_level_t     s_last_alert_level = ALERT_LEVEL_NONE;
static bool              s_last_has_target  = false;
static int               s_last_ty          = -1;
static int               s_last_skew_x      = 0;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ── Drain queue, cập nhật state nội bộ ─────────────────────── */
static void refresh_state(void)
{
    /* Drain q_tft_collision — bản sao riêng cho TFT, tránh race với collision task */
    collision_input_t tmp_ci;
    while (xQueueReceive(q_tft_collision, &tmp_ci, 0) == pdTRUE)
        s_ci = tmp_ci;

    /* Chuyển đổi lat/lon của từng peer sang ENU (m) lấy ego làm gốc.
     * Không kiểm tra gps_valid: task_localization đã cập nhật lat/lon từ
     * DR 100 Hz nên ego và peer luôn có vị trí hợp lệ sau GPS fix đầu tiên.
     * Chỉ bỏ qua nếu lat/lon vẫn là (0,0) — tức chưa từng có GPS fix. */
    bool ego_has_pos = (s_ci.ego.lat != 0.0f || s_ci.ego.lon != 0.0f);
    if (ego_has_pos) {
        for (int i = 0; i < s_ci.n_peers && i < COLLISION_MAX_PEERS; i++) {
            if (s_ci.peers[i].lat == 0.0f && s_ci.peers[i].lon == 0.0f) continue;
            geo_latlon_to_enu(s_ci.ego.lat,       s_ci.ego.lon,
                              s_ci.peers[i].lat,  s_ci.peers[i].lon,
                              &s_ci.peers[i].x,   &s_ci.peers[i].y);
        }
    }

    /* Drain alerts — giữ cảnh báo hiển thị tối thiểu 2 giây */
    alert_result_t tmp_al;
    while (xQueueReceive(q_alert_tft, &tmp_al, 0) == pdTRUE) {
        if (tmp_al.level > ALERT_LEVEL_NONE) {
            s_alert = tmp_al;
            s_alert_active = true;
            s_alert_until  = now_ms() + 2000;
        }
    }

    if (s_alert_active && now_ms() > s_alert_until) {
        s_alert_active = false;
        s_alert.level = ALERT_LEVEL_NONE;
    }
}

/* ── Tìm kiếm xe trước gần nhất ──────────────────────────────────────── */
/* dx/dy từ geo_latlon_to_enu() là tọa độ ENU tuyệt đối (East-North).
 * Cần xoay về Body frame (trục dọc = hướng mũi xe) để kiểm tra
 * đúng "phía trước xe" bất kể xe đang quay về hướng địa lý nào. */
static int get_closest_front_peer(float *out_dist)
{
    int closest_idx = -1;
    float min_dist = 999.0f;

    /* Xoay ENU → Body frame theo heading ego.
     * heading = 0 → North ≡ ENU (không đổi gì).
     * heading = π/2 (East) → trục dọc xe khớp trục East của ENU. */
    float cos_h = cosf(s_ci.ego.heading);
    float sin_h = sinf(s_ci.ego.heading);

    for (int i = 0; i < s_ci.n_peers && i < COLLISION_MAX_PEERS; i++) {
        if (!s_ci.peers[i].gps_valid) continue;
        float dx = s_ci.peers[i].x;   /* East offset (m) */
        float dy = s_ci.peers[i].y;   /* North offset (m) */
        float dist = sqrtf(dx*dx + dy*dy);

        /* Chiếu sang body frame:
         *   dy_body > 0 → peer ở phía trước mũi xe
         *   dx_body     → lệch ngang (dương = bên phải xe) */
        float dx_body =  dx * cos_h - dy * sin_h;
        float dy_body =  dx * sin_h + dy * cos_h;

        /* Xe phía trước: thẳng phía trước (dy_body > 0) và cùng làn (|dx_body| < 15m) */
        if (dy_body > 0.0f && fabsf(dx_body) < 15.0f && dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    if (closest_idx != -1) {
        *out_dist = min_dist;
    }
    return closest_idx;
}

/* ── Vẽ bầu trời (Sky & Horizon Line) ────────────────────────────────── */
static void draw_sky_scenery(void)
{
    // Đường chân trời mờ màu xanh xám ngăn cách bầu trời
    tft_fill_rect(0, ADAS_HY - 1, CFG_TFT_WIDTH, 1, TFT_RGB(35, 45, 60));

    // Vẽ 4 ngôi sao nhỏ cố định trên trời đêm
    uint16_t star_color = TFT_RGB(150, 170, 200);
    tft_draw_pixel(15, 12, star_color);
    tft_draw_pixel(45, 8, star_color);
    tft_draw_pixel(85, 10, star_color);
    tft_draw_pixel(110, 6, star_color);
}

/* ── Vẽ mặt đường 3D (3D Road Surface) ──────────────────────────────── */
static void draw_road_surface(void)
{
    for (int y = ADAS_HY; y <= ADAS_BY; y++) {
        float t = (float)(y - ADAS_HY) / (float)ADAS_H_SPAN;
        int half_w = (int)(t * (float)ADAS_MAX_W);
        tft_fill_rect(64 - half_w, y, 2 * half_w + 1, 1, TFT_RGB(25, 28, 32));
    }
}

/* ── Vẽ vùng cảnh báo nguy hiểm dạng khối tĩnh (Warning Corridor) ───── */
static void draw_road_corridor(int ty)
{
    int y_start = ty > ADAS_HY ? ty : ADAS_HY;
    int y_end = 95; // Dừng ngay sát đầu xe Ego

    if (y_start >= y_end) return;

    if (s_alert_active) {
        uint16_t color = (s_alert.level == ALERT_LEVEL_CRITICAL) ? TFT_RGB(255, 30, 30) :
                         (s_alert.level == ALERT_LEVEL_WARNING)  ? TFT_RGB(255, 140, 0) :
                                                                   TFT_RGB(255, 230, 0);

        for (int y = y_start; y <= y_end; y++) {
            float t = (float)(y - ADAS_HY) / (float)ADAS_H_SPAN;
            int half_w = (int)(t * (float)ADAS_MAX_W);
            tft_fill_rect(64 - half_w, y, 2 * half_w + 1, 1, color);
        }
    }
}

/* ── Vẽ xe Ego Truck (Ego Vehicle) ──────────────────────────────────── */
static void draw_ego_car(int skew_x)
{
    // Bánh sau
    tft_fill_rect(48, 107, 4, 8, TFT_BLACK);
    tft_fill_rect(76, 107, 4, 8, TFT_BLACK);
    
    // Bánh trước (bẻ lái nhẹ bánh xe trước nếu cua)
    // Nếu skew_x > 0 (cua trái, thân nghiêng phải), ta bẻ bánh trước sang trái 1 pixel (steer_left = -1)
    // Nếu skew_x < 0 (cua phải, thân nghiêng trái), ta bẻ bánh trước sang phải 1 pixel (steer_left = 1)
    int steer_left = (skew_x > 0) ? -1 : ((skew_x < 0) ? 1 : 0);
    tft_fill_rect(50 + steer_left, 97, 3, 5, TFT_BLACK);
    tft_fill_rect(75 + steer_left, 97, 3, 5, TFT_BLACK);

    // Thân xe chính (Bumper/Chassis) dịch chuyển skew_x / 2
    int dx_mid = skew_x / 2;
    tft_fill_rect(50 + dx_mid, 102, 28, 11, TFT_RGB(65, 75, 85));
    // Cabin dịch chuyển skew_x
    tft_fill_rect(53 + skew_x, 94, 22, 8, TFT_RGB(85, 100, 115));
    // Kính sau dịch chuyển skew_x
    tft_fill_rect(56 + skew_x, 96, 16, 4, TFT_BLACK);
    // Biển số xe dịch chuyển skew_x / 2
    tft_fill_rect(60 + dx_mid, 109, 8, 3, TFT_YELLOW);

    // Đèn phanh hậu (Sáng đỏ rực khi có cảnh báo)
    if (s_alert_active) {
        tft_fill_rect(50 + dx_mid, 102, 5, 4, TFT_RGB(255, 0, 0));
        tft_fill_rect(73 + dx_mid, 102, 5, 4, TFT_RGB(255, 0, 0));
    } else {
        tft_fill_rect(51 + dx_mid, 103, 4, 3, TFT_RGB(150, 20, 20));
        tft_fill_rect(73 + dx_mid, 103, 4, 3, TFT_RGB(150, 20, 20));
    }
}

/* ── Vẽ xe mục tiêu phía trước (Target Car) ───────────────────────────── */
static void draw_target_car(int ty, uint16_t color)
{
    float scale = (float)(ty - ADAS_HY) / (float)ADAS_H_SPAN;
    if (scale < 0.0f) scale = 0.0f;
    if (scale > 1.0f) scale = 1.0f;

    int w = 6 + (int)(scale * 14.0f);
    int h = 4 + (int)(scale * 8.0f);

    // Bánh xe
    if (w >= 10) {
        int tw_w = w / 6;
        if (tw_w < 1) tw_w = 1;
        tft_fill_rect(64 - w/2 - tw_w, ty - h + 2, tw_w, h - 2, TFT_BLACK);
        tft_fill_rect(64 + w/2,         ty - h + 2, tw_w, h - 2, TFT_BLACK);
    }

    // Khung gầm
    tft_fill_rect(64 - w / 2, ty - h, w, h, color);
    
    // Cabin
    int cw = (w * 2) / 3;
    int ch = h / 2;
    if (ch < 2) ch = 2;
    tft_fill_rect(64 - cw / 2, ty - h - ch, cw, ch, TFT_RGB(35, 45, 55));
    
    // Đèn hậu
    if (w >= 8) {
        int lw = w / 5;
        if (lw < 1) lw = 1;
        tft_fill_rect(64 - w / 2 + 1, ty - h + 1, lw, 2, TFT_RGB(255, 40, 40));
        tft_fill_rect(64 + w / 2 - 1 - lw, ty - h + 1, lw, 2, TFT_RGB(255, 40, 40));
    }
}

/* ── Xóa và vẽ lại vạch đứt tâm đường động nhanh ──────────────────────── */
static void erase_and_redraw_center_lanes(int ty, bool has_target)
{
    // 1. Vẽ đè đường thẳng tâm bằng màu nền tương ứng để xóa vạch đứt cũ
    int y_warn_start = (s_alert_active && has_target) ? ty : 95;
    if (y_warn_start < ADAS_HY) y_warn_start = ADAS_HY;

    // Phân đoạn trên: xóa bằng màu đường xám
    tft_fill_rect(64, ADAS_HY, 1, y_warn_start - ADAS_HY, TFT_RGB(25, 28, 32));

    // Phân đoạn cảnh báo: xóa bằng màu cảnh báo hiện tại
    if (s_alert_active && has_target && y_warn_start < 95) {
        uint16_t warn_color = (s_alert.level == ALERT_LEVEL_CRITICAL) ? TFT_RGB(255, 30, 30) :
                              (s_alert.level == ALERT_LEVEL_WARNING)  ? TFT_RGB(255, 140, 0) :
                                                                        TFT_RGB(255, 230, 0);
        tft_fill_rect(64, y_warn_start, 1, 95 - y_warn_start + 1, warn_color);
    }

    // Phân đoạn dưới: xóa bằng màu đường xám
    tft_fill_rect(64, 96, 1, ADAS_BY - 96 + 1, TFT_RGB(25, 28, 32));

    // 2. Vẽ các vạch đứt tâm mới di chuyển (Road motion)
    for (int i = 0; i < 3; i++) {
        float u = (float)((s_frame_cnt * 3 + i * 20) % 60) / 60.0f;
        int dy1 = ADAS_HY + (int)(u * u * (float)ADAS_H_SPAN);
        float u_next = u + 0.06f;
        if (u_next > 1.0f) u_next = 1.0f;
        int dy2 = ADAS_HY + (int)(u_next * u_next * (float)ADAS_H_SPAN);

        if (dy1 < ADAS_BY) {
            int y_end = dy2 > ADAS_BY ? ADAS_BY : dy2;
            if (y_end >= dy1) {
                tft_fill_rect(64, dy1, 1, y_end - dy1 + 1, TFT_GRAY);
            }
        }
    }
}

/* ── Vẽ footer trạng thái & cảnh báo tối ước không nháy ─────────────────── */
static void draw_footer(void)
{
    static bool s_footer_bg_drawn = false;
    static alert_level_t s_footer_bg_level = ALERT_LEVEL_NONE;
    static alert_type_t s_footer_bg_type = (alert_type_t)-1;

    // Chỉ vẽ lại nền footer khi trạng thái cảnh báo thay đổi (tránh chớp nháy vùng chữ tĩnh)
    bool need_bg_redraw = s_need_full_redraw || !s_footer_bg_drawn ||
                          (s_alert_active && (s_alert.level != s_footer_bg_level || s_alert.type != s_footer_bg_type)) ||
                          (!s_alert_active && s_footer_bg_level != ALERT_LEVEL_NONE);

    if (need_bg_redraw) {
        if (s_alert_active) {
            uint16_t bg = (s_alert.level == ALERT_LEVEL_CRITICAL) ? TFT_BG_CRIT :
                          (s_alert.level == ALERT_LEVEL_WARNING)  ? TFT_BG_WARN :
                                                                    TFT_BG_INFO;
            tft_fill_rect(0, FOOTER_Y, CFG_TFT_WIDTH, FOOTER_H, bg);
            tft_draw_line(0, FOOTER_Y, CFG_TFT_WIDTH - 1, FOOTER_Y, TFT_WHITE);

            const char *name = (s_alert.type == ALERT_TYPE_EBBL) ? "PHANH GAP!" :
                               (s_alert.type == ALERT_TYPE_TTC)  ? "VA CHAM!"   :
                                                                   "CAUTION";
            int name_len = strlen(name);
            int name_x = (CFG_TFT_WIDTH - name_len * 12 + 2) / 2;
            if (name_x < 0) name_x = 4;
            tft_draw_str(name_x, FOOTER_Y + 4, name, TFT_WHITE, bg, 2);

            s_footer_bg_level = s_alert.level;
            s_footer_bg_type = s_alert.type;
        } else {
            tft_fill_rect(0, FOOTER_Y, CFG_TFT_WIDTH, FOOTER_H, TFT_BG_IDLE);
            tft_draw_line(0, FOOTER_Y, CFG_TFT_WIDTH - 1, FOOTER_Y, TFT_RGB(30, 80, 50));
            tft_draw_str(31, FOOTER_Y + 6, "ADAS ACTIVE", TFT_RGB(0, 220, 100), TFT_BG_IDLE, 1);

            s_footer_bg_level = ALERT_LEVEL_NONE;
            s_footer_bg_type = (alert_type_t)-1;
        }
        s_footer_bg_drawn = true;
    }

    // Luôn vẽ đè các nội dung động (sử dụng chế độ vẽ đè chữ có màu nền để không nhấp nháy)
    if (s_alert_active) {
        uint16_t bg = (s_alert.level == ALERT_LEVEL_CRITICAL) ? TFT_BG_CRIT :
                      (s_alert.level == ALERT_LEVEL_WARNING)  ? TFT_BG_WARN :
                                                                TFT_BG_INFO;
        char mbuf[24];
        snprintf(mbuf, sizeof(mbuf), "TTC %.1fs  Dist %3.0fm", s_alert.ttc_s, s_alert.dist_m);
        int m_len = strlen(mbuf);
        int m_x = (CFG_TFT_WIDTH - m_len * 6 + 1) / 2;
        if (m_x < 0) m_x = 4;
        tft_draw_str(m_x, FOOTER_Y + 24, mbuf, TFT_WHITE, bg, 1);
    } else {
        // GPS: màu Xanh lá nếu có gps nhận được, màu Đỏ nếu không nhận được
        uint16_t gps_col = s_ci.ego.gps_valid ? TFT_RGB(0, 220, 100) : TFT_RGB(255, 50, 50);
        tft_draw_str(8, FOOTER_Y + 21, "GPS", gps_col, TFT_BG_IDLE, 1);

        char n_buf[10];
        snprintf(n_buf, sizeof(n_buf), "N:%2d", s_ci.n_peers);
        tft_draw_str(CFG_TFT_WIDTH - 36, FOOTER_Y + 21, n_buf, TFT_DOT_SAFE, TFT_BG_IDLE, 1);
    }
}

/* ── Vẽ một frame hoàn chỉnh theo lớp (Layer-based rendering) ────────── */
static void render_frame(void)
{
    static bool s_first_frame = true;
    if (s_first_frame) {
        tft_clear(TFT_BG_IDLE);
        s_first_frame = false;
        s_need_full_redraw = true;
    }

    s_frame_cnt++;

    // Tìm xe mục tiêu phía trước gần nhất
    float dist_m = 999.0f;
    int closest_idx = get_closest_front_peer(&dist_m);
    bool has_target = (closest_idx != -1);

    // Ánh xạ động khoảng cách thực tế nhận từ xe trước (dist_m) sang trục Y màn hình (ty)
    int ty = 65;
    if (has_target) {
        float t = dist_m / CFG_EBBL_MAX_DIST_M;
        if (t > 1.0f) t = 1.0f;
        if (t < 0.0f) t = 0.0f;
        ty = ADAS_HY + (int)((1.0f - t) * (95.0f - (float)ADAS_HY));
    }

    // Tính toán độ nghiêng thân xe (body roll) dựa trên vận tốc góc quay (gyro_z) từ IMU
    float yaw_rate = s_ci.ego.gyro_z;
    if (fabsf(yaw_rate) < 0.05f) {
        yaw_rate = 0.0f;
    }
    int skew_x = (int)(yaw_rate * 6.0f);
    if (skew_x > 3) skew_x = 3;
    if (skew_x < -3) skew_x = -3;

    // Phát hiện thay đổi trạng thái để vẽ lại toàn bộ nền khi xe xuất hiện/biến mất hoặc alert đổi màu hoặc khoảng cách xe trước thay đổi hoặc độ nghiêng xe thay đổi
    if (s_alert_active != s_last_alert_active || 
        s_alert.level != s_last_alert_level ||
        has_target != s_last_has_target ||
        (has_target && ty != s_last_ty) ||
        skew_x != s_last_skew_x) 
    {
        s_need_full_redraw = true;
        s_last_alert_active = s_alert_active;
        s_last_alert_level = s_alert.level;
        s_last_has_target = has_target;
        s_last_ty = ty;
        s_last_skew_x = skew_x;
    }

    bool need_full_redraw_saved = s_need_full_redraw;

    if (s_need_full_redraw) {
        // Lớp 0: Bầu trời phía trên chân trời & sao
        draw_sky_scenery();

        // Lớp 1: Mặt đường 3D
        draw_road_surface();

        // Lớp 2: Vùng nguy hiểm cảnh báo (corridor) - Chỉ vẽ khi phát hiện có xe trước
        if (has_target) {
            draw_road_corridor(ty);
        }

        // Lớp 3: Đường biên và các vạch viền
        tft_draw_line(64, ADAS_HY, 64 - ADAS_MAX_W, ADAS_BY, TFT_RGB(0, 180, 220));
        tft_draw_line(64, ADAS_HY, 64 + ADAS_MAX_W, ADAS_BY, TFT_RGB(0, 180, 220));

        // Lớp 4: Xe trước di chuyển vị trí động (Chỉ vẽ khi có xe thực tế nhận từ V2V)
        if (has_target) {
            uint16_t car_color = TFT_RGB(160, 170, 180);
            if (s_alert_active) {
                car_color = (s_alert.level == ALERT_LEVEL_CRITICAL) ? TFT_RGB(255, 30, 30) :
                            (s_alert.level == ALERT_LEVEL_WARNING)  ? TFT_RGB(255, 140, 0) :
                                                                      TFT_RGB(255, 230, 0);
            }
            draw_target_car(ty, car_color);
        }

        // Lớp 5: Xe của mình (Ego Car) cố định vị trí đứng yên ở đáy đường
        draw_ego_car(skew_x);

        // Lớp 6: Cung đồng hồ đo sườn trang trí
        tft_draw_line(12, 50, 4, 70, TFT_RGB(30, 50, 75));
        tft_draw_line(4, 70, 4, 90, TFT_RGB(30, 50, 75));
        tft_draw_line(4, 90, 12, 110, TFT_RGB(30, 50, 75));
        tft_draw_line(115, 50, 123, 70, TFT_RGB(30, 50, 75));
        tft_draw_line(123, 70, 123, 90, TFT_RGB(30, 50, 75));
        tft_draw_line(123, 90, 115, 110, TFT_RGB(30, 50, 75));
        tft_draw_str(10, 90, "HDG", TFT_GRAY, TFT_BG_IDLE, 1);
        tft_draw_str(106, 88, "km/h", TFT_GRAY, TFT_BG_IDLE, 1);

        s_need_full_redraw = false;
    }

    // --- Cập nhật động cực nhanh (Fast Update Path) ---

    // 1. Xóa và vẽ lại làn đường đứt quãng ở tâm đường (Không gây quét hình)
    erase_and_redraw_center_lanes(ty, has_target);

    // 2. Cập nhật tốc độ xe (chỉ vẽ đè số khi thay đổi)
    static int s_last_speed = -1;
    int speed_kmh = (int)(s_ci.ego.velocity * 3.6f);
    if (speed_kmh != s_last_speed) {
        char spd_str[4];
        snprintf(spd_str, sizeof(spd_str), "%3d", speed_kmh);
        tft_draw_str(86, 70, spd_str, TFT_WHITE, TFT_BG_IDLE, 2);
        s_last_speed = speed_kmh;
    }

    // 3. Cập nhật góc hướng xe HDG (chỉ vẽ đè số và ký tự hướng khi thay đổi)
    static int s_last_hdg = -1;
    int hdg_deg = (int)(s_ci.ego.heading * 57.29578f);
    hdg_deg = (hdg_deg % 360 + 360) % 360;

    if (hdg_deg != s_last_hdg || need_full_redraw_saved) {
        const char *comp = "N ";
        if (hdg_deg >= 338 || hdg_deg < 23)        comp = "N ";
        else if (hdg_deg >= 23  && hdg_deg < 68)   comp = "NE";
        else if (hdg_deg >= 68  && hdg_deg < 113)  comp = "E ";
        else if (hdg_deg >= 113 && hdg_deg < 158)  comp = "SE";
        else if (hdg_deg >= 158 && hdg_deg < 203)  comp = "S ";
        else if (hdg_deg >= 203 && hdg_deg < 248)  comp = "SW";
        else if (hdg_deg >= 248 && hdg_deg < 293)  comp = "W ";
        else                                       comp = "NW";

        // Vẽ chữ hướng la bàn (Scale 2, màu cam/vàng)
        tft_draw_str(7, 70, comp, TFT_RGB(255, 160, 0), TFT_BG_IDLE, 2);

        // Vẽ góc hướng dạng số (Scale 1) ở góc trên cung trái
        char hdg_str[5];
        snprintf(hdg_str, sizeof(hdg_str), "%3d", hdg_deg);
        tft_draw_str(4, 54, hdg_str, TFT_GRAY, TFT_BG_IDLE, 1);

        s_last_hdg = hdg_deg;
    }

    // 4. Cập nhật trạng thái chỉ báo GPS & Peer & Cảnh báo động ở Footer
    static bool s_last_footer_alert_active = false;
    static alert_level_t s_last_footer_alert_level = ALERT_LEVEL_NONE;
    static bool s_last_gps_valid = false;
    static int s_last_peers = -1;

    if (need_full_redraw_saved ||
        s_alert_active ||
        (s_alert_active != s_last_footer_alert_active) ||
        (s_alert.level != s_last_footer_alert_level) ||
        (s_ci.ego.gps_valid != s_last_gps_valid) ||
        (s_ci.n_peers != s_last_peers))
    {
        draw_footer();
        s_last_footer_alert_active = s_alert_active;
        s_last_footer_alert_level = s_alert.level;
        s_last_gps_valid = s_ci.ego.gps_valid;
        s_last_peers = s_ci.n_peers;
    }
}

/* ── Màn hình boot splash ────────────────────────────────────────────── */
static void render_boot(void)
{
    tft_clear(TFT_BG_IDLE);
    tft_draw_str( 8, 30,  "V2V",     TFT_ACCENT, TFT_BG_IDLE, 3);
    tft_draw_str( 8, 60,  "WARNING", TFT_ACCENT, TFT_BG_IDLE, 2);
    tft_draw_str( 8, 90,  "WARMING UP...", TFT_GRAY,   TFT_BG_IDLE, 1);
    /* Warmup delay thực xảy ra trong mpu_calibrate() (IMU task).
     * TFT chỉ hiển thị splash ngắn rồi tiếp tục render bình thường. */
    vTaskDelay(pdMS_TO_TICKS(500));
}

/* ── Task entry ──────────────────────────────────────────────────────── */
void task_display_tft(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    if (tft_init() != ESP_OK) {
        ESP_LOGE(TAG, "TFT init failed — task exits");
        vTaskDelete(NULL);
        return;
    }
    render_boot();

    ESP_LOGI(TAG, "started — q_tft_collision+q_alert_tft -> TFT (10 Hz)");

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CFG_PERIOD_TFT_MS));

        refresh_state();
        render_frame();
    }
}

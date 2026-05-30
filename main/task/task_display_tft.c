/* task/task_display_tft.c — Hiển thị radar V2V và cảnh báo lên màn hình TFT 128×160.
 *
 * Chạy ở 10 Hz. Mỗi frame:
 *   1. Drain q_tft_collision → s_ci (danh sách ego + peers).
 *   2. Drain q_alert_tft     → s_alert (cảnh báo hiện tại).
 *   3. Vẽ: radar grid → peers → ego → status bar → footer cảnh báo.
 *
 * Bố cục màn hình:
 *   [0..7px]   Status bar: GPS indicator | Speed | Peer count
 *   [8..119px] Vùng radar: North=up, tỷ lệ 150m/56px
 *   [120..159px] Footer: cảnh báo (màu đỏ/vàng/xanh) hoặc heading + "V2V READY" */
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

/* ── Hằng số hình học radar ──────────────────────────────────────────── */
#define RADAR_CX      64          /* tâm radar X (px) */
#define RADAR_CY      64          /* tâm radar Y (px) */
#define RADAR_R       56          /* bán kính radar (px) */
#define RADAR_SCALE_M 150.0f      /* 150m ↔ RADAR_R px */

/* Chuyển ENU (m) → tọa độ màn hình (North = lên = -Y màn hình) */
#define ENUx_TO_SCR(px) ((int)(RADAR_CX + (px) * RADAR_R / RADAR_SCALE_M))
#define ENUy_TO_SCR(py) ((int)(RADAR_CY - (py) * RADAR_R / RADAR_SCALE_M))

/* Phân vùng màn hình */
#define STATUSBAR_H   8
#define FOOTER_Y      120
#define FOOTER_H      (CFG_TFT_HEIGHT - FOOTER_Y)  /* 40px */

/* ── Trạng thái nội bộ ───────────────────────────────────────────────── */
static collision_input_t s_ci          = {0};
static alert_result_t    s_alert       = {0};
static bool              s_alert_active = false;
static uint32_t          s_alert_until  = 0;  /* hiển thị cảnh báo tối thiểu 2 giây */

extern volatile uint32_t g_gps_rx_count;
extern volatile uint32_t g_gps_raw_bytes;

/* [TEST 2.4] Lưu lịch sử 5 lần DR test gần nhất */
#define MAX_DR_HISTORY 5
static dr_test_result_t  s_dr_history[MAX_DR_HISTORY] = {0};

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
     * ego.lat/lon có thể = 0 khi GPS chưa có fix — các peer sẽ nằm ở tâm
     * nhưng được render_frame() bỏ qua nếu !gps_valid. */
    if (s_ci.ego.gps_valid) {
        for (int i = 0; i < s_ci.n_peers && i < COLLISION_MAX_PEERS; i++) {
            if (!s_ci.peers[i].gps_valid) continue;
            geo_latlon_to_enu(s_ci.ego.lat,       s_ci.ego.lon,
                              s_ci.peers[i].lat,  s_ci.peers[i].lon,
                              &s_ci.peers[i].x,   &s_ci.peers[i].y);
        }
    }

    /* Drain alerts — giữ cảnh báo hiển thị tối thiểu 2 giây */
    alert_result_t tmp_al;
    while (xQueueReceive(q_alert_tft, &tmp_al, 0) == pdTRUE) {
        s_alert = tmp_al;
        if (tmp_al.level > ALERT_LEVEL_NONE) {
            s_alert_active = true;
            s_alert_until  = now_ms() + 2000;
        }
    }
    if (s_alert_active && now_ms() > s_alert_until)
        s_alert_active = false;

    /* [TEST 2.4] Drain kết quả DR — lưu vào lịch sử */
    dr_test_result_t tmp_dr;
    while (xQueueReceive(q_dr_result, &tmp_dr, 0) == pdTRUE) {
        /* Dịch các kết quả cũ xuống dưới */
        for (int i = MAX_DR_HISTORY - 1; i > 0; i--) {
            s_dr_history[i] = s_dr_history[i - 1];
        }
        /* Lưu kết quả mới lên đầu */
        s_dr_history[0] = tmp_dr;
    }
}


/* ── Màu chấm peer dựa theo mức cảnh báo ────────────────────────────── */
static uint16_t peer_color(int peer_idx)
{
    /* Peer là nguồn gây cảnh báo → tô màu theo mức */
    if (s_alert_active &&
        memcmp(s_ci.peers[peer_idx].id, s_alert.peer_id, 4) == 0) {
        return (s_alert.level == ALERT_LEVEL_CRITICAL) ? TFT_DOT_CRIT :
               (s_alert.level == ALERT_LEVEL_WARNING)  ? TFT_DOT_WARN :
                                                          TFT_DOT_INFO;
    }
    return TFT_DOT_SAFE;  /* an toàn → xanh lá */
}

/* Vẽ xe ego: hình tròn xanh + mũi tên hướng đi */
static void draw_ego(float heading)
{
    tft_fill_circle(RADAR_CX, RADAR_CY, 5, TFT_DOT_SELF);

    /* Mũi tên 12px theo hướng heading (CW từ North) */
    int ax = RADAR_CX + (int)(12.0f * sinf(heading));
    int ay = RADAR_CY - (int)(12.0f * cosf(heading));
    tft_draw_line(RADAR_CX, RADAR_CY, ax, ay, TFT_WHITE);
}

/* Vẽ một xe peer: hình tròn + tick hướng */
static void draw_peer(const vehicle_state_t *peer, uint16_t color)
{
    int sx = ENUx_TO_SCR(peer->x);
    int sy = ENUy_TO_SCR(peer->y);

    /* Bỏ qua nếu ngoài vùng radar */
    if (sx < 0 || sx >= CFG_TFT_WIDTH) return;
    if (sy < STATUSBAR_H || sy >= FOOTER_Y) return;

    tft_fill_circle(sx, sy, 4, color);

    int tx = sx + (int)(8.0f * sinf(peer->heading));
    int ty = sy - (int)(8.0f * cosf(peer->heading));
    tft_draw_line(sx, sy, tx, ty, TFT_WHITE);
}

/* Vẽ lưới radar: chữ thập + nhãn "N" */
static void draw_radar_grid(void)
{
    tft_draw_line(RADAR_CX, STATUSBAR_H,
                  RADAR_CX, FOOTER_Y - 1, TFT_RGB(25, 40, 55));
    tft_draw_line(0, RADAR_CY, CFG_TFT_WIDTH - 1,
                  RADAR_CY, TFT_RGB(25, 40, 55));
    tft_draw_char(RADAR_CX - 2, STATUSBAR_H + 1, 'N',
                  TFT_GRAY, TFT_BG_IDLE, 1);
}

/* Vẽ status bar (8px trên cùng): GPS LED | tốc độ | số peer */
static void draw_statusbar(const vehicle_state_t *ego, int n_peers)
{
    tft_fill_rect(0, 0, CFG_TFT_WIDTH, STATUSBAR_H, TFT_RGB(15, 25, 45));

    /* Chỉ báo GPS: xanh = có fix, đỏ = mất GPS */
    uint16_t gps_col = ego->gps_valid ? TFT_RGB(0, 200, 80) :
                                        TFT_RGB(200, 50, 50);
    tft_fill_rect(1, 1, 5, 6, gps_col);

    char buf[24];
    snprintf(buf, sizeof(buf), "%3dkm/h", (int)(ego->velocity * 3.6f));
    tft_draw_str(8, 1, buf, TFT_CYAN, TFT_RGB(15, 25, 45), 1);

    /* Hiển thị số byte thô (B) và số NMEA hợp lệ (Rx) */
    snprintf(buf, sizeof(buf), "B:%lu R:%lu", 
             (unsigned long)(g_gps_raw_bytes % 10000), 
             (unsigned long)(g_gps_rx_count % 1000));
    tft_draw_str(55, 1, buf, TFT_WHITE, TFT_RGB(15, 25, 45), 1);

    snprintf(buf, sizeof(buf), "N:%d", n_peers);
    tft_draw_str(CFG_TFT_WIDTH - 24, 1, buf,
                 TFT_DOT_SAFE, TFT_RGB(15, 25, 45), 1);
}

/* Vẽ footer (40px dưới cùng): cảnh báo, kết quả DR test, hoặc heading + trạng thái */
static void draw_footer(void)
{
    if (s_alert_active) {
        /* Nền màu theo mức cảnh báo */
        uint16_t bg = (s_alert.level == ALERT_LEVEL_CRITICAL) ? TFT_BG_CRIT :
                      (s_alert.level == ALERT_LEVEL_WARNING)  ? TFT_BG_WARN :
                                                                TFT_BG_INFO;
        tft_fill_rect(0, FOOTER_Y, CFG_TFT_WIDTH, FOOTER_H, bg);

        const char *name = (s_alert.type == ALERT_TYPE_EBBL) ? "PHANH GAP!" :
                           (s_alert.type == ALERT_TYPE_TTC)  ? "VA CHAM!"   :
                                                               "CAUTION";
        tft_draw_str(4, FOOTER_Y + 2, name, TFT_WHITE, bg, 2);

        char mbuf[20];
        snprintf(mbuf, sizeof(mbuf), "TTC %.1fs %.0fm",
                 s_alert.ttc_s, s_alert.dist_m);
        tft_draw_str(4, FOOTER_Y + 20, mbuf, TFT_WHITE, bg, 1);

    } else {
        /* Idle: hiển thị heading và trạng thái hệ thống */
        tft_fill_rect(0, FOOTER_Y, CFG_TFT_WIDTH, FOOTER_H, TFT_BG_IDLE);
        char buf[20];
        float hdg = s_ci.ego.heading * 57.2957795f;
        if (hdg < 0) hdg += 360.0f;
        snprintf(buf, sizeof(buf), "HDG %.0f deg", hdg);
        tft_draw_str(4, FOOTER_Y + 4,  buf, TFT_ACCENT, TFT_BG_IDLE, 1);
        tft_draw_str(4, FOOTER_Y + 16, "V2V READY", TFT_DOT_SAFE, TFT_BG_IDLE, 1);
    }
}

/* ── Vẽ bảng kết quả DR ─────────────────────────────────────────────── */
static void draw_dr_table(void)
{
    int y = STATUSBAR_H + 4;
    tft_draw_str(4, y, "DR TEST CASES", TFT_CYAN, TFT_BG_IDLE, 1);
    y += 12;
    tft_draw_str(4, y, "L |T(s)|DR(m)|kmh", TFT_ACCENT, TFT_BG_IDLE, 1);
    tft_draw_line(0, y + 10, CFG_TFT_WIDTH, y + 10, TFT_RGB(25, 40, 55));
    y += 14;

    for (int i = 0; i < MAX_DR_HISTORY; i++) {
        if (s_dr_history[i].run_num > 0) {
            char buf[32];
            /* Hiển thị: Lần | Thời gian | Quãng đường | Vận tốc */
            snprintf(buf, sizeof(buf), "%-2d|%-4.1f|%-5.1f|%3.0f", 
                     s_dr_history[i].run_num, 
                     s_dr_history[i].blackout_s, 
                     s_dr_history[i].dr_dist_m,
                     s_dr_history[i].avg_vel_kmh);
            tft_draw_str(4, y, buf, TFT_WHITE, TFT_BG_IDLE, 1);
        } else {
            tft_draw_str(4, y, "- | -  |  -  | - ", TFT_GRAY, TFT_BG_IDLE, 1);
        }
        y += 16;
    }
}

/* ── Vẽ một frame hoàn chỉnh ────────────────────────────────────────── */
static void render_frame(void)
{
    /* Xóa vùng radar (tránh full-clear → nhanh hơn) */
    tft_fill_rect(0, STATUSBAR_H,
                  CFG_TFT_WIDTH, FOOTER_Y - STATUSBAR_H,
                  TFT_BG_IDLE);

    /* Vẽ bảng lịch sử DR thay cho radar */
    draw_dr_table();
    vTaskDelay(1);

    draw_statusbar(&s_ci.ego, s_ci.n_peers);
    draw_footer();
    vTaskDelay(1);
}

/* ── Màn hình boot splash ────────────────────────────────────────────── */
static void render_boot(void)
{
    tft_clear(TFT_BG_IDLE);
    tft_draw_str( 8, 30,  "V2V",     TFT_ACCENT, TFT_BG_IDLE, 3);
    tft_draw_str( 8, 60,  "WARNING", TFT_ACCENT, TFT_BG_IDLE, 2);
    tft_draw_str( 8, 90,  "INIT...", TFT_GRAY,   TFT_BG_IDLE, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));
}

/* ── Task entry ──────────────────────────────────────────────────────── */
void task_display_tft(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

#ifndef V2V_HW_STUB
    if (tft_init() != ESP_OK) {
        ESP_LOGE(TAG, "TFT init failed — task exits");
        vTaskDelete(NULL);
        return;
    }
    render_boot();
#endif

    ESP_LOGI(TAG, "started — q_tft_collision+q_alert_tft -> TFT (10 Hz)");

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CFG_PERIOD_TFT_MS));

#ifndef V2V_HW_STUB
        refresh_state();
        render_frame();
#else
        /* Stub mode: log ra console thay vì vẽ màn hình */
        refresh_state();
        ESP_LOGD(TAG, "[TFT] ego v=%.1f hdg=%.0f | peers=%d | alert=%d",
                 s_ci.ego.velocity,
                 s_ci.ego.heading * 57.3f,
                 s_ci.n_peers,
                 s_alert.level);
#endif
    }
}

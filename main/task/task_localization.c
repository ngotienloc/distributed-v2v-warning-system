/* task/task_localization.c — Ước tính trạng thái xe (ego vehicle state).
 *
 * Nhận fusion_output_t từ task_fusion, quyết định nguồn vị trí/tốc độ:
 *   - GPS hợp lệ  → reset Dead Reckoning, dùng tốc độ GPS.
 *   - GPS mất     → tiếp tục tích phân DR, áp decay để tránh drift tích lũy.
 *
 * Phát vehicle_state_t → q_ego_state để task_v2v broadcast lên ESP-NOW. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "fusion/dead_reckoning/dead_reckoning.h"
#include "fusion/geo_utils/geo_utils.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include <string.h>
#include <math.h>

static const char *TAG = "task_localization";

/* Tốc độ decay velocity khi mất GPS (%/giây).
 * 0.3 = giảm 30%/s — đủ nhẹ để không báo động trong đường hầm ngắn. */
#define DR_DECAY_PER_S  0.3f

/* ── STATE MACHINE — Dead Reckoning Test 2.4 ──────────────────────────────────
 * IDLE → (GPS stale) → RECORDING → (5s elapsed) → DONE + push result
 *                                                       ↓
 *                                               (GPS phục hồi) → IDLE
 *
 * Sau 5 giây mất GPS: hiển thị ngay quãng đường DR tính được lên TFT.
 * Người dùng ghi lại số, đo quãng thực bằng thước cuộn để tính drift. */
typedef enum {
    DR_STATE_IDLE = 0,   /* đang có GPS, chờ mất sóng */
    DR_STATE_RECORDING,  /* đang ghi DR, chờ hết 5s */
    DR_STATE_DONE,       /* đã hiển thị kết quả, chờ GPS phục hồi để reset */
} dr_test_state_t;

#define DR_TEST_DURATION_MS  5000   /* 5 giây đo DR */

static dr_test_state_t s_dr_state    = DR_STATE_IDLE;
static int             s_run_num     = 0;      /* số thứ tự lần chạy (tự động tăng) */
static float           s_dr_x0       = 0.0f;   /* vị trí DR khi bắt đầu mất sóng (m) */
static float           s_dr_y0       = 0.0f;
static uint32_t        s_blk_start   = 0;      /* thời điểm bắt đầu mất GPS (ms) */
static bool            s_had_gps_ever = false;  /* true sau khi có GPS fix lần đầu tiên */

/* Gọi mỗi chu kỳ — trigger sau DR_TEST_DURATION_MS kể từ lúc mất GPS */
static void dr_test_update(bool gps_valid_now, const dr_state_t *dr)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);

    switch (s_dr_state) {

    case DR_STATE_IDLE:
        /* Ghi nhận lần đầu tiên có GPS — bắt buộc phải có trước khi đo */
        if (gps_valid_now) {
            s_had_gps_ever = true;
        }

        /* Chỉ bắt đầu đo khi đã từng có GPS (tránh trigger ngay lúc boot) */
        if (!gps_valid_now && s_had_gps_ever) {
            s_dr_x0     = dr->x;
            s_dr_y0     = dr->y;
            s_blk_start = now;
            s_dr_state  = DR_STATE_RECORDING;
            ESP_LOGI(TAG, "[DR-TEST] Blackout bat dau — DR baseline x=%.2f y=%.2f",
                     s_dr_x0, s_dr_y0);
        }
        break;

    case DR_STATE_RECORDING:
        if (gps_valid_now) {
            /* GPS phục hồi sớm hơn 5s → hủy lần đo này, reset về IDLE */
            ESP_LOGW(TAG, "[DR-TEST] GPS phục hồi sớm (%.1fs) — bỏ lần đo này",
                     (float)(now - s_blk_start) * 0.001f);
            s_dr_state = DR_STATE_IDLE;
            break;
        }

        /* Chờ đủ 5 giây → tính và push kết quả ngay */
        if ((now - s_blk_start) >= DR_TEST_DURATION_MS) {
            float blackout_s = (float)(now - s_blk_start) * 0.001f;

            /* Quãng đường DR tính theo vector ENU (m) */
            float ddx     = dr->x - s_dr_x0;
            float ddy     = dr->y - s_dr_y0;
            float dr_dist = sqrtf(ddx * ddx + ddy * ddy);

            float avg_vel_kmh = (dr_dist / blackout_s) * 3.6f;

            s_run_num++;
            dr_test_result_t res = {
                .run_num    = s_run_num,
                .dr_dist_m  = dr_dist,
                .gps_dist_m = 0.0f,   /* đo thủ công bằng thước — điền vào bảng */
                .drift_m    = 0.0f,   /* tính sau khi có số thước đo */
                .drift_pct  = 0.0f,
                .blackout_s = blackout_s,
                .avg_vel_kmh = avg_vel_kmh,
            };
            xQueueSend(q_dr_result, &res, 0);

            ESP_LOGI(TAG,
                     "[DR-TEST] Lan %d: DR=%.2fm trong %.1fs (%.1fkm/h)",
                     s_run_num, dr_dist, blackout_s, avg_vel_kmh);

            s_dr_state = DR_STATE_DONE;
        }
        break;

    case DR_STATE_DONE:
        /* Cho GPS phuc hoi tro lai moi cho phep do lan tiep theo */
        if (gps_valid_now) {
            ESP_LOGI(TAG, "[DR-TEST] GPS phuc hoi — san sang lan do tiep theo");
            s_dr_state = DR_STATE_IDLE;
        }
        break;
    }
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* Lấy 4 byte giữa MAC WiFi làm ID xe duy nhất */
static void get_self_id(uint8_t id[4])
{
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    memcpy(id, mac + 2, 4);
}


void task_localization(void *arg)
{
    dr_state_t      dr;
    vehicle_state_t ego;
    fusion_output_t fused;

    dr_init(&dr);
    memset(&ego, 0, sizeof(ego));
    get_self_id(ego.id);

    while (1) {
        /* Chờ fusion output tối đa 25 ms; nếu timeout → dùng dt mặc định */
        if (xQueueReceive(q_fusion_out, &fused, pdMS_TO_TICKS(25)) != pdTRUE) {
            memset(&fused, 0, sizeof(fused));
            fused.dt = 0.01f;
        }

        float dt = fused.dt;
        if (dt <= 0.001f || dt > 0.05f) dt = 0.01f;

        /* ── 1. GPS hợp lệ: reset DR về vị trí GPS ──────────────────── */
        if (fused.gps_updated && fused.gps.valid) {
            float fix_age_s = (float)(now_ms() - fused.gps.timestamp_ms) * 0.001f;

            /* Bù trễ pipeline: ngoại suy vị trí GPS theo fix_age */
            dr_reset_from_gps(&dr,
                               0.0f, 0.0f,
                               fused.gps.speed,
                               fused.gps.course,
                               fix_age_s);

            ego.lat         = fused.gps.latitude;
            ego.lon         = fused.gps.longitude;
            ego.gps_valid   = true;
            ego.local_ts_ms = fused.gps.timestamp_ms;

            ESP_LOGD(TAG, "GPS reset: lat=%.6f lon=%.6f spd=%.1fkm/h age=%.0fms",
                     ego.lat, ego.lon, fused.gps.speed * 3.6f, fix_age_s * 1000.0f);
        }

        /* ── 2. Kiểm tra GPS stale: đánh dấu mất GPS sau CFG_GPS_STALE_MS ── */
        if (ego.gps_valid) {
            uint32_t gps_age_ms = now_ms() - ego.local_ts_ms;
            if (gps_age_ms > CFG_GPS_STALE_MS) {
                ego.gps_valid = false;
                ESP_LOGW(TAG, "GPS stale (%lums) — switching to DR velocity",
                         (unsigned long)gps_age_ms);
            }
        }

        /* ── 3. Dead Reckoning chạy mỗi chu kỳ (bất kể GPS) ────────── */
        dr_update(&dr,
                  fused.accel_x_lin,
                  fused.accel_y_lin,
                  fused.orient.heading,
                  dt);

        /* ── 4. Chọn nguồn tốc độ ────────────────────────────────────
         * GPS hợp lệ  → tốc độ GPS (chính xác).
         * GPS mất     → tốc độ DR + decay để giảm drift. */
        if (ego.gps_valid) {
            ego.velocity = fused.gps_updated ? fused.gps.speed : ego.velocity;
        } else {
            dr_apply_velocity_decay(&dr, DR_DECAY_PER_S, dt);
            ego.velocity = sqrtf(dr.vx * dr.vx + dr.vy * dr.vy);
        }

        ego.x            = dr.x;
        ego.y            = dr.y;
        ego.heading      = fused.orient.heading;
        ego.accel_x_lin  = fused.accel_x_lin;
        ego.gyro_z       = 0.0f;
        ego.update_ts_ms = now_ms();

        /* ── 5. [TEST 2.4] Theo dõi GPS loss → sau 5s push kết quả DR ── */
        dr_test_update(ego.gps_valid, &dr);

        /* Xả queue cũ, gửi snapshot mới nhất */
        vehicle_state_t stale;
        while (xQueueReceive(q_ego_state, &stale, 0) == pdTRUE) {}
        xQueueSend(q_ego_state, &ego, 0);

        ESP_LOGV(TAG, "EGO x=%.2f y=%.2f v=%.2f hdg=%.1f° gps=%d",
                 ego.x, ego.y, ego.velocity, ego.heading * 57.3f, ego.gps_valid);
    }
}

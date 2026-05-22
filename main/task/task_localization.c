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

        /* Xả queue cũ, gửi snapshot mới nhất */
        vehicle_state_t stale;
        while (xQueueReceive(q_ego_state, &stale, 0) == pdTRUE) {}
        xQueueSend(q_ego_state, &ego, 0);

        ESP_LOGV(TAG, "EGO x=%.2f y=%.2f v=%.2f hdg=%.1f° gps=%d",
                 ego.x, ego.y, ego.velocity, ego.heading * 57.3f, ego.gps_valid);
    }
}

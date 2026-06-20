/* task/task_localization.c — Ước tính trạng thái xe (ego vehicle state).
 *
 * Nhận fusion_output_t từ task_fusion, quyết định nguồn vị trí/tốc độ:
 *   - GPS hợp lệ  → reset Dead Reckoning, lưu gốc GPS tham chiếu.
 *   - Mọi chu kỳ → cung cấp lat/lon động bằng ENU→LatLon từ DR (100 Hz)
 *     — khắc phục hiện tượng giật vị trí ở phía xe nhận V2V.
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

/* ── IMU snapshot ring buffer ────────────────────────────────────────────
 * Lưu tối đa IMU_BUF_SIZE mẫu fusion gần nhất để tái tích phân khi GPS fix.
 * 16 × 10ms = 160ms — đủ bù pipeline latency thực tế (thường < 100ms).    */
#define IMU_BUF_SIZE  16

typedef struct {
    float    ax_lin;
    float    ay_lin;
    float    heading;
    float    dt;
    uint32_t ts_ms;
} imu_snap_t;

static imu_snap_t s_imu_buf[IMU_BUF_SIZE];
static int        s_imu_head  = 0;   /* index ghi tiếp theo (vòng tròn) */
static int        s_imu_count = 0;   /* số phần tử hợp lệ hiện có (0..IMU_BUF_SIZE) */

/* ── Gốc tọa độ GPS tham chiếu cho chuyển đổi ENU → LatLon ─────────────
 * Được cập nhật mỗi khi nhận được GPS fix hợp lệ.                        */
static float s_ref_lat   = 0.0f;
static float s_ref_lon   = 0.0f;
static bool  s_ref_valid = false;

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

        /* ── 1. GPS hợp lệ: reset DR + tái tích phân IMU buffer ──────── */
        bool gps_reset_this_tick = false;
        if (fused.gps_updated && fused.gps.valid) {
            uint32_t gps_ts = fused.gps.timestamp_ms;

            /* Reset DR về (0,0) tại đúng thời điểm GPS fix — không ngoại suy.
             * fix_age = 0.0f vì ta sẽ bù bằng re-integration thực tế bên dưới. */
            dr_reset_from_gps(&dr,
                               0.0f, 0.0f,
                               fused.gps.speed,
                               fused.gps.course,
                               0.0f);

            /* Tái tích phân các mẫu IMU xảy ra SAU thời điểm GPS fix.
             * Duyệt buffer từ cũ → mới; bỏ qua sample hiện tại (ts_ms == now_ms)
             * vì nó sẽ được xử lý ở bước dr_update bên dưới — nhưng ta skip
             * bước đó bằng cờ gps_reset_this_tick để tránh double-integrate. */
            for (int i = 0; i < s_imu_count; i++) {
                int idx = (s_imu_head - s_imu_count + i + IMU_BUF_SIZE) % IMU_BUF_SIZE;
                if (s_imu_buf[idx].ts_ms > gps_ts) {
                    dr_update(&dr,
                              s_imu_buf[idx].ax_lin,
                              s_imu_buf[idx].ay_lin,
                              s_imu_buf[idx].heading,
                              s_imu_buf[idx].dt);
                }
            }

            ego.lat         = fused.gps.latitude;
            ego.lon         = fused.gps.longitude;
            ego.gps_valid   = true;
            ego.local_ts_ms = gps_ts;
            gps_reset_this_tick = true;

            /* Cập nhật gốc tham chiếu: mỗi lần có GPS fix mới,
             * DR (x,y) vừa được reset về (0,0) tại vị trí này. */
            s_ref_lat   = fused.gps.latitude;
            s_ref_lon   = fused.gps.longitude;
            s_ref_valid = true;

            ESP_LOGD(TAG, "GPS reset+reint: lat=%.6f lon=%.6f spd=%.1fkm/h ts=%lums",
                     ego.lat, ego.lon, fused.gps.speed * 3.6f, (unsigned long)gps_ts);
        }

        /* ── 2. Kiểm tra GPS stale: đánh dấu mất GPS sau CFG_GPS_STALE_MS ── */
        if (ego.gps_valid && !CFG_VIRTUAL_GPS_ENABLE) {
            uint32_t gps_age_ms = now_ms() - ego.local_ts_ms;
            if (gps_age_ms > CFG_GPS_STALE_MS) {
                ego.gps_valid = false;
                ESP_LOGW(TAG, "GPS stale (%lums) — switching to DR velocity",
                         (unsigned long)gps_age_ms);
            }
        }

        /* ── 3. Dead Reckoning chạy mỗi chu kỳ (bất kể GPS) ────────── */
        /* Bỏ qua nếu vừa re-integrate từ GPS fix để tránh double-integrate
         * sample hiện tại (sample này đã được tái tích phân trong bước 1). */
        if (!gps_reset_this_tick) {
            dr_update(&dr,
                      fused.accel_x_lin,
                      fused.accel_y_lin,
                      fused.orient.heading,
                      dt);
        }

        /* Push snapshot vào ring buffer (sau dr_update để ts khớp với trạng thái DR) */
        s_imu_buf[s_imu_head] = (imu_snap_t){
            .ax_lin  = fused.accel_x_lin,
            .ay_lin  = fused.accel_y_lin,
            .heading = fused.orient.heading,
            .dt      = dt,
            .ts_ms   = now_ms(),
        };
        s_imu_head = (s_imu_head + 1) % IMU_BUF_SIZE;
        if (s_imu_count < IMU_BUF_SIZE) s_imu_count++;

#if CFG_VIRTUAL_GPS_ENABLE
        /* Trong chế độ test trong nhà, dùng vận tốc tích phân từ DR và không tự động giảm tốc khi đặt phẳng */
        ego.velocity = sqrtf(dr.vx * dr.vx + dr.vy * dr.vy);
        ego.gps_valid = true; /* Cưỡng bức GPS luôn valid cho UI/V2V */
#else
        if (ego.gps_valid) {
            ego.velocity = fused.gps_updated ? fused.gps.speed : ego.velocity;
        } else {
            dr_apply_velocity_decay(&dr, DR_DECAY_PER_S, dt);
            ego.velocity = sqrtf(dr.vx * dr.vx + dr.vy * dr.vy);
        }
#endif

        ego.x            = dr.x;
        ego.y            = dr.y;
        ego.heading      = fused.orient.heading;
        ego.accel_x_lin  = fused.accel_x_lin;
        ego.gyro_z       = fused.gyro_z;
        ego.update_ts_ms = now_ms();

        /* ── 5. Chuyển đổi ENU → LatLon tần số cao (100 Hz) ──────────────
         * Dùng gốc GPS tham chiếu gần nhất để ánh xạ dr.x/dr.y sang lat/lon
         * trơn tru — khắc phục hiện tượng giật vị trí ở phía xe nhận V2V. */
        if (s_ref_valid) {
            geo_enu_to_latlon(s_ref_lat, s_ref_lon,
                              dr.x, dr.y,
                              &ego.lat, &ego.lon);
        }

        /* Xả queue cũ, gửi snapshot mới nhất */
        vehicle_state_t stale;
        while (xQueueReceive(q_ego_state, &stale, 0) == pdTRUE) {}
        xQueueSend(q_ego_state, &ego, 0);

        ESP_LOGV(TAG, "EGO x=%.2f y=%.2f lat=%.6f lon=%.6f v=%.2f hdg=%.1f° gps=%d ref=%d",
                 ego.x, ego.y, ego.lat, ego.lon,
                 ego.velocity, ego.heading * 57.3f,
                 ego.gps_valid, s_ref_valid);
    }
}

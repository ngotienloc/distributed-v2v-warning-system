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

#if ENABLE_TEST_MODE
#include "driver/gpio.h"
#include "v2v/neighbor_table.h"
#endif

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
#if ENABLE_TEST_MODE
    vehicle_state_t ego;
    memset(&ego, 0, sizeof(ego));
    get_self_id(ego.id);

    // Cấu hình nút BOOT (GPIO 0)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Test mode running: Role=%s (1=Front, 0=Rear)", 
             TEST_ROLE_FRONT ? "FRONT" : "REAR");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Chạy chu kỳ 100ms (10 Hz)

        #if TEST_ROLE_FRONT
            // Xe phía trước (Front)
            static float local_rear_lat = 21.000000f;
            static float relative_offset = 100.0f; // Khoảng cách ban đầu 100m
            static float v_front = 30.0f;         // Tốc độ ban đầu 30 m/s (~108 km/h)

            // Đồng bộ hóa vĩ độ xe sau nếu nhận được gói tin từ xe sau
            vehicle_state_t peers[5];
            int n = ntable_get_all(peers, 5);
            if (n > 0) {
                // Giả định xe nhận được đầu tiên là xe sau (vì hệ thống chỉ có 2 xe)
                local_rear_lat = peers[0].lat;
            } else {
                // Tự động tịnh tiến xe sau nếu chưa kết nối
                local_rear_lat += (30.0f * 0.10f) / 110540.0f;
            }

            bool pressed = (gpio_get_level(GPIO_NUM_0) == 0);
            if (pressed) {
                // Phanh gấp: giảm tốc độ với gia tốc -8.0 m/s2, tối thiểu 2.0 m/s
                v_front -= 8.0f * 0.10f;
                if (v_front < 2.0f) v_front = 2.0f;
                ego.accel_x_lin = -8.0f;
                xEventGroupSetBits(g_ebbl_evt, EBBL_BRAKE_BIT);
                ESP_LOGW(TAG, "BOOT button pressed - Front vehicle Braking! speed=%.1f km/h, dist=%.1f m", v_front * 3.6f, relative_offset);
            } else {
                // Nhả nút: tăng tốc trở lại 30 m/s với gia tốc 4.0 m/s2
                v_front += 4.0f * 0.10f;
                if (v_front > 30.0f) v_front = 30.0f;
                ego.accel_x_lin = 0.0f;

                // Nếu đã tăng tốc về bình thường, hồi phục khoảng cách về 100m
                if (v_front >= 30.0f) {
                    relative_offset += 10.0f * 0.10f; // Hồi phục với tốc độ 10 m/s
                    if (relative_offset > 100.0f) relative_offset = 100.0f;
                }
            }

            // Tích phân khoảng cách tương đối dựa trên độ lệch tốc độ
            relative_offset += (v_front - 30.0f) * 0.10f;
            if (relative_offset < 5.0f) relative_offset = 5.0f; // Giới hạn khoảng cách tối thiểu 5m

            ego.velocity = v_front;
            ego.lat = local_rear_lat + relative_offset / 110540.0f;
            ego.lon = 105.000000f;
        #else
            // Xe phía sau (Rear)
            static float local_rear_lat = 21.000000f;
            local_rear_lat += (30.0f * 0.10f) / 110540.0f;

            ego.velocity = 30.0f;          // Tốc độ 30 m/s (~108 km/h)
            ego.accel_x_lin = 0.0f;
            ego.lat = local_rear_lat;
            ego.lon = 105.000000f;
        #endif

        ego.heading      = 0.0f;           // Di chuyển thẳng về hướng Bắc
        ego.gps_valid    = true;
        ego.local_ts_ms  = now_ms();
        ego.update_ts_ms = now_ms();
        ego.x            = 0.0f;
        ego.y            = 0.0f;
        ego.gyro_z       = 0.0f;

        // Xả queue cũ, gửi trạng thái ego mới nhất
        vehicle_state_t stale;
        while (xQueueReceive(q_ego_state, &stale, 0) == pdTRUE) {}
        xQueueSend(q_ego_state, &ego, 0);

        ESP_LOGD(TAG, "EGO: x=%.2f y=%.2f v=%.2f lat=%.6f lon=%.6f gps=%d",
                 ego.x, ego.y, ego.velocity, ego.lat, ego.lon, ego.gps_valid);
    }
#else
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

            ESP_LOGD(TAG, "GPS reset+reint: lat=%.6f lon=%.6f spd=%.1fkm/h ts=%lums",
                     ego.lat, ego.lon, fused.gps.speed * 3.6f, (unsigned long)gps_ts);
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
#endif
}

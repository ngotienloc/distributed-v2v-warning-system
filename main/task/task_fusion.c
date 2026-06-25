/* task/task_fusion.c — Tổng hợp IMU + GPS thành fusion_output_t.
 *
 * Chạy ở 100 Hz (theo nhịp q_imu):
 *   1. Đọc IMU từ q_imu → cập nhật Complementary Filter (pitch/roll/heading).
 *   2. Phát hiện phanh gấp (ego xe) → set EBBL_BRAKE_BIT nếu xác nhận.
 *   3. Drain q_gps → fuse heading GPS vào CF.
 *   4. Gửi fusion_output_t (luôn kèm GPS fix cuối cùng) → q_fusion_out. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "fusion/complementary_filter/com_fil.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG = "task_fusion";

/* Cache GPS fix hợp lệ cuối cùng — gửi kèm mỗi chu kỳ để task_localization
 * luôn có vị trí tham chiếu dù GPS không có fix mới trong chu kỳ này. */
static gps_data_t s_last_valid_gps = {0};

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void task_fusion(void *arg)
{
    imu_filter_state_t cf;
    imu_filter_init(&cf);

    imu_data_t      imu;
    gps_data_t      gps;
    fusion_output_t out;
    uint32_t        brake_cooldown = 0;  /* thời điểm cuối kích hoạt EBBL (ms) */

    while (1) {
        /* ── 1. Đọc IMU (chờ tối đa 20 ms) ─────────────────────────── */
        if (xQueueReceive(q_imu, &imu, pdMS_TO_TICKS(20)) != pdTRUE) {
            /* Timeout: dùng mẫu zero + dt mặc định để CF không bị stall */
            memset(&imu, 0, sizeof(imu));
            imu.dt = 0.01f;
        }

        if (imu.dt <= 0.001f || imu.dt > 0.05f) imu.dt = 0.01f;

        /* ── 1.1. Phát hiện cử chỉ lắc thiết bị để chuyển chế độ test ──── */
        if (g_dr_test.mode == 0) {
            static float s_prev_ax = 0.0f;
            static float s_prev_ay = 0.0f;
            static float s_prev_az = 0.0f;
            static int s_shake_count = 0;
            static uint32_t s_last_shake_time = 0;

            float dx = imu.accel_x - s_prev_ax;
            float dy = imu.accel_y - s_prev_ay;
            float dz = imu.accel_z - s_prev_az;

            s_prev_ax = imu.accel_x;
            s_prev_ay = imu.accel_y;
            s_prev_az = imu.accel_z;

            float diff_mag = sqrtf(dx * dx + dy * dy + dz * dz);
            uint32_t now = now_ms();

            // Nếu biến thiên gia tốc lớn (> 5.0 m/s2 trong 10ms), tăng đếm
            if (diff_mag > 5.0f) {
                s_shake_count++;
                s_last_shake_time = now;
            }

            // Nếu quá 500ms không có cú lắc nào mới, reset đếm
            if (now - s_last_shake_time > 500) {
                s_shake_count = 0;
            }

            // Nếu đếm đủ 15 lần vượt ngưỡng trong vòng 500ms -> Xác nhận lắc
            if (s_shake_count >= 15) {
                s_shake_count = 0;
                ESP_LOGW(TAG, "Device shake detected! Switching to DR Test mode...");
                
                g_dr_test.mode = 1;
                memset(g_dr_test.result_count, 0, sizeof(g_dr_test.result_count));
                memset(g_dr_test.results, 0, sizeof(g_dr_test.results));
                g_dr_test.outage_active = false;
                g_dr_test.waiting_first_fix = false;
                g_dr_test.last_drift_valid = false;
                g_dr_test.last_test_end_ms = now;
                g_dr_test.trigger_double_beep = true; // Kêu bíp kép xác nhận chuyển màn
            }
        }

        /* ── 2. Cập nhật Complementary Filter ───────────────────────── */
        imu_filter_update(&cf, &imu, imu.dt);

        /* ── 3. Phát hiện phanh gấp (sau warmup để ổn định CF) ─────── */
        if (cf.tick >= CFG_CF_WARMUP_TICKS) {
            uint32_t now = now_ms();
            if (imu_filter_is_braking(&cf) &&
                (now - brake_cooldown) >= CFG_EBBL_COOLDOWN_MS) {
                brake_cooldown = now;
                xEventGroupSetBits(g_ebbl_evt, EBBL_BRAKE_BIT);
                ESP_LOGW(TAG, "Hard brake accel_x_lin=%.2f m/s² — EBBL burst triggered", cf.accel_x_lin);
            }
        }

        /* ── 4. Drain tất cả GPS fix mới trong chu kỳ này ───────────── */
        bool gps_updated = false;
        while (xQueueReceive(q_gps, &gps, 0) == pdTRUE) {
            if (!gps.valid) continue;
            if (g_dr_test.outage_active) {
                continue; /* Giả lập mất GPS: bỏ qua không cập nhật vào Complementary Filter */
            }
            imu_filter_fuse_gps_heading(&cf, gps.course, gps.speed);
            s_last_valid_gps = gps;  /* lưu lại fix cuối */
            gps_updated = true;
        }

        /* ── 5. Đóng gói và gửi kết quả → q_fusion_out ─────────────── */
        out.orient.roll    = cf.roll;
        out.orient.pitch   = cf.pitch;
        out.orient.heading = cf.heading;
        out.accel_x_lin    = cf.accel_x_lin;
        out.accel_y_lin    = cf.accel_y_lin;
        out.dt             = imu.dt;
        out.gps            = s_last_valid_gps;  /* luôn gửi fix cuối cùng */
        out.gps_updated    = gps_updated;
        out.gyro_z         = imu.gyro_z;

        /* Xả queue cũ trước khi gửi để task_localization nhận dữ liệu mới nhất */
        fusion_output_t stale;
        while (xQueueReceive(q_fusion_out, &stale, 0) == pdTRUE)  {}
        xQueueSend(q_fusion_out, &out, 0);

        ESP_LOGV(TAG, "CF: roll=%.1f° pitch=%.1f° hdg=%.1f° gps_upd=%d",
            out.orient.roll    * 57.3f,
            out.orient.pitch   * 57.3f,
            out.orient.heading * 57.3f,
            (int)gps_updated);
    }
}

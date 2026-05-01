#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "fusion/complementary_filter/com_fil.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "task_fusion";

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// Sensor Fusion 
void task_fusion(void *arg)
{
    imu_filter_state_t cf;
    imu_filter_init(&cf);

    imu_data_t      imu;
    gps_data_t      gps;
    fusion_output_t out;
    uint32_t        brake_cooldown = 0;

    while(1){
        if (xQueueReceive(q_imu, &imu, pdMS_TO_TICKS(20)) != pdTRUE) {
            memset(&imu, 0, sizeof(imu));
            imu.dt = 0.01f;
        }

        if (imu.dt <= 0.001f || imu.dt > 0.05f) imu.dt = 0.01f;

        // Complementary Filter
        imu_filter_update(&cf, &imu, imu.dt);


        // hard brank detection (my car)
        if (cf.tick >= CFG_CF_WARMUP_TICKS) {
        uint32_t now = now_ms();
        if (imu_filter_detect_brake(&cf) &&
            (now - brake_cooldown) >= CFG_EBBL_COOLDOWN_MS) {
            brake_cooldown = now;
            xEventGroupSetBits(g_ebbl_evt, EBBL_BRAKE_BIT);
            ESP_LOGW(TAG,"Hard brake accel_x_lin=%.2f m/s² — EBBL burst triggered", cf.accel_x_lin);
            }
        }
        bool gps_updated = false;
        gps_data_t latest_gps;
        memset(&latest_gps, 0, sizeof(latest_gps));

        while (xQueueReceive(q_gps, &gps, 0) == pdTRUE) {
            if (!gps.valid) continue;
            imu_filter_fuse_gps_heading(&cf, gps.course, gps.speed);
            latest_gps  = gps;
            gps_updated = true;
        }

        out.orient.roll    = cf.roll;
        out.orient.pitch   = cf.pitch;
        out.orient.heading = cf.heading;
        out.accel_x_lin    = cf.accel_x_lin;
        out.accel_y_lin    = cf.accel_y_lin;
        out.dt             = imu.dt;
        out.gps            = latest_gps;
        out.gps_updated    = gps_updated;

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


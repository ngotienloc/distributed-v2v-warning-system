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

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

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

        if (xQueueReceive(q_fusion_out, &fused, pdMS_TO_TICKS(25)) != pdTRUE) {
            memset(&fused, 0, sizeof(fused));
            fused.dt = 0.01f;
        }

        float dt = fused.dt;
        if (dt <= 0.001f || dt > 0.05f) dt = 0.01f;

        if (fused.gps_updated && fused.gps.valid) {
            float fix_age_s = (float)(now_ms() - fused.gps.timestamp_ms) * 0.001f;

            dr_reset_from_gps(&dr,
                               0.0f, 0.0f,
                               fused.gps.speed,
                               fused.gps.course,
                               fix_age_s);

            /* Update ego GPS fields */
            ego.lat          = fused.gps.latitude;
            ego.lon          = fused.gps.longitude;
            ego.gps_valid    = true;
            ego.nmea_time_ms = fused.gps.nmea_time_ms;
            ego.local_ts_ms  = fused.gps.timestamp_ms;
            ego.velocity     = fused.gps.speed;

            ESP_LOGD(TAG, "GPS reset: lat=%.6f lon=%.6f spd=%.1fkm/h age=%.0fms",
                     ego.lat, ego.lon, ego.velocity * 3.6f, fix_age_s * 1000.0f);
        }

        //dead reckoning
        dr_update(&dr,
                  fused.accel_x_lin,
                  fused.accel_y_lin,
                  fused.orient.heading,
                  dt);

        ego.x            = dr.x;
        ego.y            = dr.y;
        ego.heading      = fused.orient.heading;
        ego.accel_x_lin  = fused.accel_x_lin;
        ego.gyro_z       = 0.0f;  
        ego.update_ts_ms = now_ms();

        if (!ego.gps_valid) {
            ego.velocity = sqrtf(dr.vx * dr.vx + dr.vy * dr.vy);
        }
        
        vehicle_state_t stale;
        while (xQueueReceive(q_ego_state, &stale, 0) == pdTRUE) { /* drain */ }
        xQueueSend(q_ego_state, &ego, 0);

        ESP_LOGV(TAG, "EGO x=%.2f y=%.2f v=%.2f hdg=%.1f°",
                 ego.x, ego.y, ego.velocity, ego.heading * 57.3f);
    }
}


#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "drivers/gps/gps.h"

static const char *TAG = "main";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static void on_gps_fix(const gps_fix_t *fix, void *ctx)
{
    (void)ctx;
    if (!fix || !fix->valid) return;
    ESP_LOGI(TAG,
             "GPS fix lat=%.6f lon=%.6f spd=%.2f m/s hdg=%.1f deg ts=%u nmea_ms=%u",
             fix->lat, fix->lon, fix->speed_ms,
             fix->heading_rad * 180.0f / (float)M_PI,
             (unsigned)fix->timestamp_ms, (unsigned)fix->nmea_time_ms);
}

static void gps_status_task(void *arg)
{
    (void)arg;
    while (1) {
        ESP_LOGI(TAG, "GPS has_fix=%s age_ms=%u",
                 gps_has_fix() ? "yes" : "no",
                 (unsigned)gps_fix_age_ms());
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    gps_register_cb(on_gps_fix, NULL);
    esp_err_t err = gps_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gps_init failed: %s", esp_err_to_name(err));
        return;
    }

    xTaskCreatePinnedToCore(gps_status_task, "gps_status",
                            2048, NULL, 1, NULL, tskNO_AFFINITY);
    ESP_LOGI(TAG, "GPS test started");
}

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "drivers/gps/gps.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "types.h"
#include <string.h>

static const char *TAG = "task_gps";

/* ── GPS driver callback ──────────────────────────────────────── */
static void on_fix(const gps_fix_t *fix, void *ctx)
{
    (void)ctx;
    if (!fix || !fix->valid) return;

    /* Convert driver type -> queue message type */
    gps_data_t msg = {
        .latitude     = fix->lat,
        .longitude    = fix->lon,
        .speed        = fix->speed_ms,
        .course       = fix->heading_rad,
        .valid        = fix->valid,
        .timestamp_ms = fix->timestamp_ms,
        .nmea_time_ms = fix->nmea_time_ms,
    };

    if (xQueueSend(q_gps, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "q_gps full — GPS fix dropped");
    }
}

void task_gps(void *arg)
{
    gps_register_cb(on_fix, NULL);
    ESP_LOGI(TAG, "GPS callback registered -> q_gps");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}



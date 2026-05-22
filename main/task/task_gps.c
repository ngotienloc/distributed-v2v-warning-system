/* task/task_gps.c — Cầu nối giữa GPS driver và pipeline.
 *
 * GPS driver gọi callback on_fix() khi có NMEA RMC hợp lệ.
 * Task này chỉ đăng ký callback rồi ngủ — mọi xử lý do driver đảm nhiệm. */
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

/* ── Callback từ GPS driver — chạy trong parse_task của driver ──────── */
static void on_fix(const gps_fix_t *fix, void *ctx)
{
    (void)ctx;
    if (!fix || !fix->valid) return;

    /* Chuyển kiểu driver (gps_fix_t) → kiểu queue (gps_data_t) */
    gps_data_t msg = {
        .latitude     = fix->lat,
        .longitude    = fix->lon,
        .speed        = fix->speed_ms,
        .course       = fix->heading_rad,
        .valid        = fix->valid,
        .timestamp_ms = fix->timestamp_ms,
    };

    if (xQueueSend(q_gps, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "q_gps full — GPS fix dropped");
    }
}

/* ── Task entry — chỉ đăng ký callback, không tự đọc GPS ───────────── */
void task_gps(void *arg)
{
    gps_register_cb(on_fix, NULL);
    ESP_LOGI(TAG, "GPS callback registered -> q_gps");

    /* Task tồn tại để giữ callback đăng ký; ngủ vĩnh viễn */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

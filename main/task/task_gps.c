/* task/task_gps.c — Cầu nối giữa GPS driver và pipeline.
 *
 * Task này "own" toàn bộ vòng đời GPS:
 *   1. Đăng ký callback on_fix() TRƯỚC
 *   2. Gọi gps_init() → driver tạo uart_reader_task và parse_task
 *   3. Loop ngủ — on_fix() chạy trong parse_task của driver khi có RMC fix.
 *
 * Thứ tự register → init đảm bảo callback luôn sẵn sàng trước khi driver
 * bắt đầu nhận dữ liệu NMEA, tránh bỏ lỡ GPS fix đầu tiên. */
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

/* ── Task entry ─────────────────────────────────────────────────────── */
void task_gps(void *arg)
{
    /* Bước 1: đăng ký callback TRƯỚC khi driver tạo parse task */
    gps_register_cb(on_fix, NULL);
    ESP_LOGI(TAG, "GPS callback registered -> q_gps");

    /* Bước 2: khởi tạo driver (tạo uart_reader_task + parse_task) */
    ESP_ERROR_CHECK(gps_init());

    /* Bước 3: loop — on_fix() tự được gọi bởi parse_task của driver */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

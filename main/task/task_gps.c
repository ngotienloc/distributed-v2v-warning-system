/* task/task_gps.c — Cầu nối giữa GPS driver và pipeline.
 *
 * GPS driver gọi callback on_fix() khi có NMEA RMC hợp lệ.
 *
 * QUAN TRỌNG — thứ tự khởi tạo đúng trong main.c:
 *   1. gps_task_register_cb()   ← đăng ký callback TRƯỚC
 *   2. gps_init()               ← driver bắt đầu parse NMEA
 *   3. xTaskCreate(task_gps)    ← task chỉ giữ ngữ cảnh, không đăng ký lại
 *
 * Lý do: gps_init() tạo uart_reader_task và parse_task ngay lập tức.
 * Nếu callback đăng ký sau gps_init() (như trước đây), các GPS fix đến
 * trong khoảng thời gian calibrate IMU (~3 giây) sẽ bị driver bỏ qua. */
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

/* ── Đăng ký callback — phải gọi trong main.c TRƯỚC gps_init() ─────────
 * Tách ra khỏi task_gps() để tránh race condition: nếu callback được
 * đăng ký sau gps_init(), các GPS fix đến trong khoảng thời gian calibrate
 * IMU (~3 giây) sẽ bị driver bỏ qua vì s_cb == NULL. */
void gps_task_register_cb(void)
{
    gps_register_cb(on_fix, NULL);
    ESP_LOGI(TAG, "GPS on_fix callback registered -> q_gps");
}

/* ── Task entry — task tồn tại để giữ ngữ cảnh; callback đã đăng ký ── */
void task_gps(void *arg)
{
    ESP_LOGI(TAG, "GPS task alive (callback registered before gps_init)");

    /* Task tồn tại để giữ callback đăng ký; ngủ vĩnh viễn */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

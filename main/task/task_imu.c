/* task/task_imu.c — Đọc IMU (MPU6050) ở 100 Hz và gửi vào q_imu.
 *
 * Dùng vTaskDelayUntil để đảm bảo chu kỳ cố định 10 ms.
 * Nếu đọc thất bại → bỏ qua cycle (không gửi dữ liệu rác). */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "drivers/imu/mpu6050.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "task_imu";

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void task_imu(void *arg)
{
    imu_data_t imu;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   last_ms   = now_ms();

    ESP_LOGI(TAG, "started — 100 Hz sensor read -> q_imu");

    while (1) {
        /* Chờ đủ 10 ms (chu kỳ cố định, bù trừ thời gian xử lý) */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CFG_PERIOD_IMU_MS));

        uint32_t now = now_ms();
        float    dt  = (now - last_ms) * 0.001f;
        last_ms = now;

        /* Kẹp dt vào khoảng hợp lệ (tránh tích phân sai khi scheduler trễ) */
        if (dt <= 0.001f || dt > 0.05f) dt = 0.01f;

        if (mpu_read(&imu) != ESP_OK) {
            ESP_LOGW(TAG, "mpu_read() failed — skipping cycle");
            continue;
        }

        imu.dt = dt;
        /* Gửi không chờ (timeout=0): nếu queue đầy → bỏ mẫu cũ */
        if (xQueueSend(q_imu, &imu, 0) != pdTRUE) {
            ESP_LOGV(TAG, "q_imu full — sample dropped");
        }
    }
}

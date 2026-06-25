#include "task_button.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "config.h"
#include "types.h"
#include "drivers/buzzer/buzzer.h"

static const char *TAG = "task_button";

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void trigger_beep(int duration_ms)
{
    buzzer_set_level(1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    buzzer_set_level(0);
}

static void trigger_double_beep(void)
{
    trigger_beep(80);
    vTaskDelay(pdMS_TO_TICKS(80));
    trigger_beep(80);
}

void task_button(void *arg)
{
    ESP_LOGI(TAG, "BOOT button task started on GPIO %d", CFG_BOOT_PIN);

    /* Cấu hình chân BOOT (GPIO0) làm Input với điện trở kéo lên */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CFG_BOOT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    int press_ticks = 0;
    bool long_press_triggered = false;

    while (1) {
        /* Đọc trạng thái nút nhấn (0 = nhấn, 1 = nhả) */
        if (gpio_get_level(CFG_BOOT_PIN) == 0) {
            press_ticks++;
            /* Nhấn giữ >= 1.5 giây (75 ticks * 20ms = 1500ms) */
            if (press_ticks >= 75 && !long_press_triggered) {
                long_press_triggered = true;
                
                /* Chuyển đổi vòng tròn chế độ: 0 -> 1 -> 2 -> 3 -> 4 -> 0 */
                g_dr_test.mode = (g_dr_test.mode + 1) % 5;
                g_dr_test.outage_active = false;
                g_dr_test.waiting_first_fix = false;
                g_dr_test.trigger_double_beep = false;
                g_dr_test.trigger_start_beep = false;
                g_dr_test.last_drift_valid = false;
                memset(g_dr_test.result_count, 0, sizeof(g_dr_test.result_count));
                memset(g_dr_test.results, 0, sizeof(g_dr_test.results));

                ESP_LOGI(TAG, "Mode switched to %d (0: ADAS, 1: 5s, 2: 10s, 3: 20s, 4: 30s)", g_dr_test.mode);
                trigger_beep(100);
            }
        } else {
            /* Nút nhả: nhấn ngắn ở chế độ tự động không làm gì */
            press_ticks = 0;
            long_press_triggered = false;
        }

        /* Kiểm tra nếu thời gian ngắt kết nối đã trôi qua */
        if (g_dr_test.outage_active) {
            uint32_t elapsed = now_ms() - g_dr_test.outage_start_ms;
            if (elapsed >= g_dr_test.outage_duration_ms) {
                g_dr_test.outage_active = false;
                g_dr_test.waiting_first_fix = true;
                
                ESP_LOGI(TAG, "GPS Outage simulation ended. Waiting for first GPS fix...");
                trigger_beep(150);
            }
        }

        /* Kiểm tra yêu cầu bíp báo bắt đầu cuộc đo */
        if (g_dr_test.trigger_start_beep) {
            g_dr_test.trigger_start_beep = false;
            trigger_beep(100);
        }

        /* Kiểm tra yêu cầu bíp báo hoàn thành cuộc đo */
        if (g_dr_test.trigger_double_beep) {
            g_dr_test.trigger_double_beep = false;
            trigger_double_beep();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "drivers/buzzer/buzzer.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

static const char *TAG = "task_buzzer";

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void task_buzzer(void *arg)
{
    ESP_LOGI(TAG, "started — q_alert_buzzer -> Buzzer");

    buzzer_init();
    buzzer_beep_boot();

    alert_result_t s_alert = {0};
    bool s_alert_active = false;
    uint32_t s_alert_until = 0;

    bool buzzer_on = false;
    uint32_t next_toggle_ms = 0;
    alert_level_t last_beep_level = ALERT_LEVEL_NONE;

    while (1) {
        // Drain alerts - keep warning for at least 1.5 seconds so beep is complete and noticeable
        alert_result_t tmp_al;
        while (xQueueReceive(q_alert_buzzer, &tmp_al, 0) == pdTRUE) {
            // Only overwrite if it's a higher level or if the current alert has expired/none
            if (tmp_al.level > s_alert.level || !s_alert_active) {
                s_alert = tmp_al;
            }
            if (tmp_al.level > ALERT_LEVEL_NONE) {
                s_alert_active = true;
                s_alert_until = now_ms() + 1500; // Keep alert active for 1.5 seconds minimum
                
                // Nhận được cảnh báo phanh khẩn cấp từ xe lân cận -> kéo chân test lên HIGH
                if (tmp_al.level == ALERT_LEVEL_CRITICAL) {
                    gpio_set_level(CFG_LATENCY_TEST_PIN, 1);
                }
            }
        }

        uint32_t cur_time = now_ms();

        // Expire active warning if timeout reached
        if (s_alert_active && cur_time > s_alert_until) {
            s_alert_active = false;
            s_alert.level = ALERT_LEVEL_NONE;
            s_alert.type = ALERT_TYPE_NONE;
            gpio_set_level(CFG_LATENCY_TEST_PIN, 0); // Trở về mức LOW khi kết thúc cảnh báo
        }

        // If level changes, reset toggling immediately
        if (s_alert.level != last_beep_level) {
            last_beep_level = s_alert.level;
            buzzer_on = false;
            buzzer_set_level(0);
            next_toggle_ms = cur_time; // force toggle on next check
        }

        // Determine beep pattern based on alert level
        uint32_t beep_on_ms = 0;
        uint32_t beep_off_ms = 0;

        switch (s_alert.level) {
            case ALERT_LEVEL_CRITICAL:
                // Beep very fast: 80ms ON, 80ms OFF
                beep_on_ms = 80;
                beep_off_ms = 80;
                break;
            case ALERT_LEVEL_WARNING:
                // Beep medium: 150ms ON, 250ms OFF
                beep_on_ms = 150;
                beep_off_ms = 250;
                break;
            case ALERT_LEVEL_INFO:
                // Beep slow: 200ms ON, 800ms OFF
                beep_on_ms = 200;
                beep_off_ms = 800;
                break;
            case ALERT_LEVEL_NONE:
            default:
                // No beep
                beep_on_ms = 0;
                beep_off_ms = 0;
                break;
        }

        if (beep_on_ms > 0 && beep_off_ms > 0) {
            if (cur_time >= next_toggle_ms) {
                buzzer_on = !buzzer_on;
                buzzer_set_level(buzzer_on ? 1 : 0);
                next_toggle_ms = cur_time + (buzzer_on ? beep_on_ms : beep_off_ms);
            }
        } else {
            if (buzzer_on) {
                buzzer_on = false;
                buzzer_set_level(0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // run loop every 20ms
    }
}

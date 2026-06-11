#include "drivers/buzzer/buzzer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "buzzer";

esp_err_t buzzer_init(void)
{
    gpio_reset_pin(CFG_BUZZER_PIN);
    gpio_set_direction(CFG_BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(CFG_BUZZER_PIN, 0);
    ESP_LOGI(TAG, "Buzzer initialized on GPIO %d", CFG_BUZZER_PIN);
    return ESP_OK;
}

void buzzer_set_level(int level)
{
    gpio_set_level(CFG_BUZZER_PIN, level);
}

void buzzer_beep_boot(void)
{
    ESP_LOGI(TAG, "Playing startup sound...");
    // 2 tiếng bíp ngắn
    buzzer_set_level(1);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_set_level(0);
    vTaskDelay(pdMS_TO_TICKS(100));
    buzzer_set_level(1);
    vTaskDelay(pdMS_TO_TICKS(150));
    buzzer_set_level(0);
}

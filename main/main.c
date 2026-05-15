#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "config.h"
#include "task/app_queues.h"
#include "drivers/gps/gps.h"
#include "drivers/imu/mpu6050.h"
#include "v2v/espnow_comm.h"
#include "v2v/neighbor_table.h"

static const char *TAG = "main";

/* ── Forward declarations of task entry points ─────────────────── */
void task_imu         (void *arg);
void task_gps         (void *arg);
void task_fusion      (void *arg);
void task_localization(void *arg);
void task_v2v         (void *arg);
void task_collision   (void *arg);
void task_display_tft (void *arg);   /* TFT visualization */

void app_main(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════════");
    ESP_LOGI(TAG, "  V2V Collision Warning System — ESP32-S3");
    ESP_LOGI(TAG, "  Queue-based pipeline architecture");
    ESP_LOGI(TAG, "═══════════════════════════════════════════");



    /* ── 1. NVS ─────────────────────────────────────────────────── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── 2. Event loop (before WiFi / ESP-NOW) ───────────────────── */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* ── 3. Queue + EventGroup init ─────────────────────────────── */

    ESP_ERROR_CHECK(app_queues_init());

    /* ── 4. Hardware drivers ─────────────────────────────────────── */
    ESP_LOGI(TAG, "Initializing hardware drivers...");
    ESP_ERROR_CHECK(mpu_init());
    ESP_ERROR_CHECK(mpu_calibrate());  /* ~3 s stationary calibration */
    ESP_ERROR_CHECK(gps_init());

    /* ── 5. V2V communication layer ─────────────────────────────── */
    ESP_ERROR_CHECK(espnow_init(q_v2v_rx));

    /* ── 6. Neighbor table (used internally by task_v2v only) ────── */
    ntable_init();

    /* ── 7. Create tasks in pipeline order ──────────────────────── *
     * Order does not matter for correctness (queues decouple timing)
     * but aligns with the pipeline for clarity.                     */
    ESP_LOGI(TAG, "Creating task pipeline...");

    xTaskCreatePinnedToCore(task_imu,
                             "imu",
                             CFG_STACK_IMU,
                             NULL, CFG_PRIO_IMU, NULL,
                             CFG_CORE_IMU);

    xTaskCreatePinnedToCore(task_gps,
                             "gps",
                             CFG_STACK_GPS,
                             NULL, CFG_PRIO_GPS, NULL,
                        7     CFG_CORE_GPS);

    xTaskCreatePinnedToCore(task_fusion,
                             "fusion",
                             CFG_STACK_FUSION,
                             NULL, CFG_PRIO_FUSION, NULL,
                             CFG_CORE_FUSION);

    xTaskCreatePinnedToCore(task_localization,       /* NEW */
                             "localize",
                             CFG_STACK_LOCALIZATION,
                             NULL, CFG_PRIO_LOCALIZATION, NULL,
                             CFG_CORE_LOCALIZATION);

    xTaskCreatePinnedToCore(task_v2v,
                             "v2v",
                             CFG_STACK_V2V,
                             NULL, CFG_PRIO_V2V, NULL,
                             CFG_CORE_V2V);

    xTaskCreatePinnedToCore(task_collision,
                             "collision",
                             CFG_STACK_COLLISION,
                             NULL, CFG_PRIO_COLLISION, NULL,
                             CFG_CORE_COLLISION);

    xTaskCreatePinnedToCore(task_display_tft,
                             "display_tft",
                             CFG_STACK_DISPLAY_TFT,
                             NULL, CFG_PRIO_DISPLAY_TFT, NULL,
                             CFG_CORE_DISPLAY_TFT);

    ESP_LOGI(TAG, "All 7 tasks created — FreeRTOS scheduler running.");
    /* app_main returns; FreeRTOS scheduler takes control. */

}

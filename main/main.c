/* main.c — Điểm khởi động hệ thống V2V
 *
 * Luồng khởi tạo:
 *   NVS → Event Loop → Queues/EventGroup → Hardware (IMU) → ESP-NOW → Tasks
 *   GPS được khởi tạo bởi task_gps (register callback trước, rồi gps_init).
 *
 * Pipeline FreeRTOS (IMU → Fusion → Localization → V2V → Collision → TFT):
 *   task_imu  →(q_imu)→  task_fusion  →(q_fusion_out)→  task_localization
 *   →(q_ego_state)→  task_v2v  →(q_collision_in)→  task_collision  →(q_alert_tft)→  task_display_tft
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "config.h"
#include "task/app_queues.h"
#include "drivers/imu/mpu6050.h"
#include "v2v/espnow_comm.h"
#include "v2v/neighbor_table.h"
#include "types.h"
#include "task/task_button.h"

dr_test_state_t g_dr_test = {0};

static const char *TAG = "main";

/* ── Khai báo trước các hàm task entry ────────────────────────────────── */
void task_imu         (void *arg);
void task_gps         (void *arg);
void task_fusion      (void *arg);
void task_localization(void *arg);
void task_v2v         (void *arg);
void task_collision   (void *arg);
void task_display_tft (void *arg);
void task_buzzer      (void *arg);

void app_main(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════════");
    ESP_LOGI(TAG, "  V2V Collision Warning System — ESP32-S3");
    ESP_LOGI(TAG, "  Queue-based pipeline architecture");
    ESP_LOGI(TAG, "═══════════════════════════════════════════");

    /* ── 1. Khởi tạo NVS (lưu calibration IMU) ───────────────────────── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── 2. Event loop (cần trước khi init WiFi / ESP-NOW) ────────────── */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* ── 3. Tạo tất cả queue + EventGroup ─────────────────────────────── */
    ESP_ERROR_CHECK(app_queues_init());

    /* ── 4. Khởi tạo phần cứng ────────────────────────────────────────── */
    ESP_LOGI(TAG, "Initializing hardware drivers...");
    ESP_ERROR_CHECK(mpu_init());
    ESP_ERROR_CHECK(mpu_calibrate());  /* warmup ~10 s + calib ~5 s — xe phải đứng yên */
    /* GPS được khởi tạo bởi task_gps: register callback → gps_init() */

    /* ── 5. Khởi tạo lớp V2V (ESP-NOW broadcast) ──────────────────────── */
    ESP_ERROR_CHECK(espnow_init(q_v2v_rx));

    /* ── 6. Khởi tạo bảng xe lân cận ─────────────────────────────────── */
    ntable_init();

    /* ── 7. Tạo 7 task theo thứ tự pipeline ──────────────────────────── */
    ESP_LOGI(TAG, "Creating task pipeline...");

    xTaskCreatePinnedToCore(task_imu,
                             "imu",
                             CFG_STACK_IMU,
                             NULL, CFG_PRIO_IMU, NULL,
                             CFG_CORE_IMU);
    vTaskDelay(pdMS_TO_TICKS(50));

    xTaskCreatePinnedToCore(task_gps,
                             "gps",
                             CFG_STACK_GPS,
                             NULL, CFG_PRIO_GPS, NULL,
                             CFG_CORE_GPS);
    vTaskDelay(pdMS_TO_TICKS(50));

    xTaskCreatePinnedToCore(task_fusion,
                             "fusion",
                             CFG_STACK_FUSION,
                             NULL, CFG_PRIO_FUSION, NULL,
                             CFG_CORE_FUSION);
    vTaskDelay(pdMS_TO_TICKS(50));

    xTaskCreatePinnedToCore(task_localization,
                             "localize",
                             CFG_STACK_LOCALIZATION,
                             NULL, CFG_PRIO_LOCALIZATION, NULL,
                             CFG_CORE_LOCALIZATION);
    vTaskDelay(pdMS_TO_TICKS(50));

    xTaskCreatePinnedToCore(task_v2v,
                             "v2v",
                             CFG_STACK_V2V,
                             NULL, CFG_PRIO_V2V, NULL,
                             CFG_CORE_V2V);
    vTaskDelay(pdMS_TO_TICKS(50));

    xTaskCreatePinnedToCore(task_collision,
                             "collision",
                             CFG_STACK_COLLISION,
                             NULL, CFG_PRIO_COLLISION, NULL,
                             CFG_CORE_COLLISION);
    vTaskDelay(pdMS_TO_TICKS(50));

    xTaskCreatePinnedToCore(task_display_tft,
                             "display_tft",
                             CFG_STACK_DISPLAY_TFT,
                             NULL, CFG_PRIO_DISPLAY_TFT, NULL,
                             CFG_CORE_DISPLAY_TFT);
    vTaskDelay(pdMS_TO_TICKS(50));

    xTaskCreatePinnedToCore(task_buzzer,
                             "buzzer",
                             CFG_STACK_BUZZER,
                             NULL, CFG_PRIO_BUZZER, NULL,
                             CFG_CORE_BUZZER);
    vTaskDelay(pdMS_TO_TICKS(50));

    xTaskCreatePinnedToCore(task_button,
                             "button",
                             CFG_STACK_BUTTON,
                             NULL, CFG_PRIO_BUTTON, NULL,
                             CFG_CORE_BUTTON);

    ESP_LOGI(TAG, "All 9 tasks created — FreeRTOS scheduler running.");
    /* app_main trả về; FreeRTOS scheduler tiếp quản. */
}

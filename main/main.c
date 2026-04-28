#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drivers/imu/mpu6050.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "types.h"

static const char *TAG = "mpu_test";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    if (mpu_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed");
        return;
    }

    ESP_LOGI(TAG, "MPU6050 test started");

    while (1) {
        imu_data_t imu;
        if (mpu_read(&imu) == ESP_OK) {
            ESP_LOGI(TAG,
                     "accel(m/s^2)=%.3f,%.3f,%.3f gyro(rad/s)=%.3f,%.3f,%.3f",
                     imu.accel_x, imu.accel_y, imu.accel_z,
                     imu.gyro_x, imu.gyro_y, imu.gyro_z);
        } else {
            ESP_LOGW(TAG, "mpu_read() failed");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

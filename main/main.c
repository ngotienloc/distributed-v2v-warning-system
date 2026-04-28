#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "config.h"
#include "drivers/imu/mpu6050.h"
#include "complementary_filter/com_fil.h"
#include "math_utils.h"

static const char *TAG = "main";

static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    init_nvs();

    ESP_ERROR_CHECK(mpu_init());
    ESP_ERROR_CHECK(mpu_calibrate());

    imu_filter_state_t filter;
    imu_filter_init(&filter);

    int64_t t0 = esp_timer_get_time();
    int64_t last = t0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CFG_PERIOD_IMU_MS));

        int64_t now = esp_timer_get_time();
        float dt = (now - last) * 1e-6f;
        last = now;
        if (dt <= 0.001f || dt > 0.05f) dt = 0.01f;

        imu_data_t imu;
        if (mpu_read(&imu) != ESP_OK) {
            ESP_LOGW(TAG, "mpu_read() failed");
            continue;
        }

        imu.dt = dt;
        imu_filter_update(&filter, &imu, dt);

        float t = (now - t0) * 1e-6f;
        float pitch_deg = RAD2DEG(filter.pitch);
        float roll_deg = RAD2DEG(filter.roll);

        printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
               t,
               imu.accel_x, imu.accel_y, imu.accel_z,
               imu.gyro_x, imu.gyro_y, imu.gyro_z,
               pitch_deg, roll_deg);
    }
}

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "i2c_bus.h"
#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "mpu6050";

#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CFG     0x1B
#define REG_ACCEL_CFG    0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_WHO_AM_I     0x75

#define WHO_GENUINE     0x68
#define WHO_CLONE_70    0x70
#define WHO_CLONE_72    0x72
#define WHO_VALID(x)    ((x)==WHO_GENUINE || (x)==WHO_CLONE_70 || (x)==WHO_CLONE_72)

#define ACCEL_SCALE  CFG_ACCEL_SCALE   
#define GYRO_SCALE   CFG_GYRO_SCALE   

static void         *s_dev  = NULL; 
static mpu_calib_t   s_cal  = {0};

static inline int16_t to16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((hi << 8) | lo);
}

bool mpu_present(void)
{
    uint8_t who = 0xFF;
    if (i2c_bus_read_reg(s_dev, REG_WHO_AM_I, &who) != ESP_OK) {
        ESP_LOGE(TAG, "I2C read WHO_AM_I failed - check wiring SDA=%d SCL=%d",
                 CFG_IMU_I2C_SDA, CFG_IMU_I2C_SCL);
        return false;
    }
    bool ok = WHO_VALID(who);
    ESP_LOGI(TAG, "WHO_AM_I=0x%02X -> %s%s",
             who,
             ok ? "OK" : "UNKNOWN",
             who == WHO_GENUINE ? " (genuine)" : " (clone)");
    return ok;
}

esp_err_t mpu_init(void)
{
    i2c_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_bus_init(CFG_IMU_I2C_PORT,
                                  CFG_IMU_I2C_SDA, CFG_IMU_I2C_SCL,
                                  CFG_IMU_I2C_FREQ_HZ, &bus));
    ESP_ERROR_CHECK(i2c_bus_add_device(bus, CFG_IMU_I2C_ADDR,
                                        CFG_IMU_I2C_FREQ_HZ, &s_dev));

    if (!mpu_present()) {
        ESP_LOGE(TAG, "MPU6050 not detected - aborting init");
        return ESP_FAIL;
    }

    // Wake up: clear SLEEP bit 
    ESP_ERROR_CHECK(i2c_bus_write_reg(s_dev, REG_PWR_MGMT_1, 0x00));
    vTaskDelay(pdMS_TO_TICKS(100));

    //Sample rate = 100 Hz
    ESP_ERROR_CHECK(i2c_bus_write_reg(s_dev, REG_SMPLRT_DIV, 9));

    //DLPF bandwidth ~44 Hz
    ESP_ERROR_CHECK(i2c_bus_write_reg(s_dev, REG_CONFIG, 0x03));

    // Gyro ±500°
    ESP_ERROR_CHECK(i2c_bus_write_reg(s_dev, REG_GYRO_CFG, 1 << 3));

    // Accel ±8
    ESP_ERROR_CHECK(i2c_bus_write_reg(s_dev, REG_ACCEL_CFG, 2 << 3));

    //Load calibration from NVS 
    nvs_handle_t nvs;
    if (nvs_open("mpu_calib", NVS_READONLY, &nvs) == ESP_OK) {
        size_t sz = sizeof(mpu_calib_t);
        if (nvs_get_blob(nvs, "cal", &s_cal, &sz) == ESP_OK)
            ESP_LOGI(TAG, "Calibration loaded from NVS");
        nvs_close(nvs);
    } else {
        ESP_LOGW(TAG, "No NVS calibration found - using zero offsets");
    }

    ESP_LOGI(TAG, "MPU6050 init OK");
    return ESP_OK;
}

esp_err_t mpu_calibrate(void)
{
    const int n = CFG_IMU_CALIB_S * 100;  //100hz sample 
    ESP_LOGI(TAG, "Calibrating %d s - keep vehicle stationary...", CFG_IMU_CALIB_S);

    int32_t sax = 0, say = 0, saz = 0;
    int32_t sgx = 0, sgy = 0, sgz = 0;

    for (int i = 0; i < n; i++) {
        uint8_t buf[14];
        ESP_ERROR_CHECK(i2c_bus_read_burst(s_dev, REG_ACCEL_XOUT_H, buf, 14));
        sax += to16(buf[0],  buf[1]);
        say += to16(buf[2],  buf[3]);
        saz += to16(buf[4],  buf[5]);
        sgx += to16(buf[8],  buf[9]);
        sgy += to16(buf[10], buf[11]);
        sgz += to16(buf[12], buf[13]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_cal.ax = (int16_t)(sax / n);
    s_cal.ay = (int16_t)(say / n);
    // Remove 1g from az (gravity when flat after callib) 
    s_cal.az = (int16_t)(saz / n - (int32_t)(9.81f / ACCEL_SCALE));
    s_cal.gx = (int16_t)(sgx / n);
    s_cal.gy = (int16_t)(sgy / n);
    s_cal.gz = (int16_t)(sgz / n);

    
    nvs_handle_t nvs;
    if (nvs_open("mpu_calib", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_blob(nvs, "cal", &s_cal, sizeof(mpu_calib_t));
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Calib done: ax=%d ay=%d az=%d gx=%d gy=%d gz=%d",
             s_cal.ax, s_cal.ay, s_cal.az,
             s_cal.gx, s_cal.gy, s_cal.gz);
    return ESP_OK;
}

esp_err_t mpu_read(imu_data_t *out)
{
    uint8_t buf[14];
    esp_err_t ret = i2c_bus_read_burst(s_dev, REG_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK) return ret;

    out->accel_x = (to16(buf[0],  buf[1])  - s_cal.ax) * ACCEL_SCALE;
    out->accel_y = (to16(buf[2],  buf[3])  - s_cal.ay) * ACCEL_SCALE;
    out->accel_z = (to16(buf[4],  buf[5])  - s_cal.az) * ACCEL_SCALE;
    out->gyro_x  = (to16(buf[8],  buf[9])  - s_cal.gx) * GYRO_SCALE;
    out->gyro_y  = (to16(buf[10], buf[11]) - s_cal.gy) * GYRO_SCALE;
    out->gyro_z  = (to16(buf[12], buf[13]) - s_cal.gz) * GYRO_SCALE;
    return ESP_OK;
}

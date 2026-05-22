/* drivers/imu/mpu6050.h — Public API của driver MPU6050. */
#pragma once
#include "types.h"
#include "esp_err.h"
#include <stdbool.h>

/* Lưu offset hiệu chỉnh (raw ADC) — đọc/ghi NVS để bền qua reboot */
typedef struct {
    int16_t ax, ay, az;  /* accel offset (raw) */
    int16_t gx, gy, gz;  /* gyro offset  (raw) */
} mpu_calib_t;

/* Khởi tạo I2C bus, kiểm tra WHO_AM_I, đặt range ±8g/±500°s, load NVS calib */
esp_err_t mpu_init(void);

/* Hiệu chỉnh tĩnh: lấy trung bình CFG_IMU_CALIB_S giây → lưu NVS
 * Yêu cầu: xe đứng yên, mặt phẳng ngang */
esp_err_t mpu_calibrate(void);

/* Đọc một mẫu accel+gyro, áp offset calib, trả về imu_data_t (đơn vị SI) */
esp_err_t mpu_read(imu_data_t *out);

/* Kiểm tra WHO_AM_I — true nếu là MPU6050 gốc hoặc clone phổ biến */
bool mpu_present(void);

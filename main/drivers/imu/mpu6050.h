#pragma once
#include "types.h"
#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} mpu_calib_t;

esp_err_t mpu_init(void);

esp_err_t mpu_calibrate(void);

esp_err_t mpu_read(imu_data_t *out);

bool mpu_present(void);

#pragma once 
#include "types.h"
#include <stdbool.h> 

typedef struct {
    float pitch; 
    float roll; 
    float heading; 
    float accel_x_lin;
    float accel_y_lin; 
    uint32_t tick; 
} imu_filter_state_t; 

void imu_filter_init(imu_filter_state_t *s);

void imu_filter_update(imu_filter_state_t *s, const imu_data_t   *imu, float dt);

void imu_filter_fuse_gps_heading(imu_filter_state_t *s, float gps_heading, float gps_speed);

bool imu_filter_detect_brake(const imu_filter_state_t *s);

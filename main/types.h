#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float accel_x;  
    float accel_y; 
    float accel_z; 
    float gyro_x;   
    float gyro_y;   
    float gyro_z;   
    float dt;       
} imu_data_t;

typedef struct {
    float latitude; 
    float longitude; 
    float speed; 
    float course; //heading
    bool  valid;  //checking 
    uint32_t timestamp_ms; 
    uint32_t nmea_time_ms; 
}gps_data_t; 



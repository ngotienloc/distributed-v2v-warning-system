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

typedef struct {
    float roll;     
    float pitch;   
    float heading; 
} orientation_t;

typedef struct {
    orientation_t orient;      //roll/pitch/heading
    float         accel_x_lin;  
    float         accel_y_lin;
    float         dt;          
    gps_data_t    gps;         
    bool          gps_updated;
} fusion_output_t;

typedef struct {
    /* Required fields */
    float    x;          
    float    y;           
    float    velocity;     
    float    heading;      

    /* Extended fields (needed by EBBL / IMA) */
    uint8_t  id[4];   
    float    lat;          
    float    lon;         
    float    accel_x_lin;  
    float    gyro_z;        
    bool     gps_valid;
    uint32_t nmea_time_ms; 
    uint32_t update_ts_ms; 
} vehicle_state_t;

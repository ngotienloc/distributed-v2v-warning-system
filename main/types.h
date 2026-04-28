#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float latitude; 
    float longitude; 
    float speed; 
    float course; //heading
    bool  valid;  //checking 
    uint32_t timestamp_ms; 
    uint32_t nmea_time_ms; 
}gps_data_t; 



#pragma once
#include <stdint.h>
#include <stdbool.h> 
#define NMEA_MAX_LEN 82

typedef struct {
    float lat; 
    float lon;
    float speed_ms;
    float heading_rad; 
    bool valid; 
    uint32_t timestamp_ms; 
    uint32_t nmea_time_ms;
}gps_fix_t; 

// Call back gps to task 
typedef void (*gps_fix_cb_t)(const gps_fix_t *fix, void *ctx);
void gps_register_cb(gps_fix_cb_t cb, void *ctx); 

esp_err_t gps_init();
bool gps_has_fix(); 
uint32_t gps_fix_age_ms();  // last time fix 





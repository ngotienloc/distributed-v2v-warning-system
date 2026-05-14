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

    /* Extended fields (needed by EBBL) */
    uint8_t  id[4];   
    float    lat;          
    float    lon;         
    float    accel_x_lin;  
    float    gyro_z;        
    bool     gps_valid;
    uint32_t nmea_time_ms; 
    uint32_t local_ts_ms;
    uint32_t update_ts_ms; 
} vehicle_state_t;


typedef enum {
    ALERT_TYPE_NONE = 0,
    ALERT_TYPE_EBBL = 1,
} alert_type_t;

typedef enum {
    ALERT_LEVEL_NONE     = 0,
    ALERT_LEVEL_INFO     = 1,
    ALERT_LEVEL_WARNING  = 2,
    ALERT_LEVEL_CRITICAL = 3,
} alert_level_t;

typedef struct {
    alert_type_t  type;
    alert_level_t level;
    float         ttc_s;    
    float         dist_m;   
    uint8_t       peer_id[4];
    int           n_peers;    
} alert_result_t;

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  id[4];
    float    lat;
    float    lon;
    float    speed;
    float    heading;
    float    accel_x_lin;
    float    gyro_z;
    uint8_t  alert_type;
    uint8_t  alert_level;
    uint32_t nmea_time_ms;
} v2v_packet_t;

#define COLLISION_MAX_PEERS  16
typedef struct {
    vehicle_state_t ego;
    vehicle_state_t peers[COLLISION_MAX_PEERS];
    int             n_peers;
} collision_input_t;

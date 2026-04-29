#pragma once
#include "driver/i2c.h"

// GPS 
#define CFG_GPS_UART_PORT       UART_NUM_1
#define CFG_GPS_UART_TX_PIN     17    
#define CFG_GPS_UART_RX_PIN     18    
#define CFG_GPS_UART_BAUD       9600
#define CFG_GPS_UART_BUF        512
#define CFG_GPS_SENTENCE_QLEN   4    

//IMU
#define CFG_IMU_I2C_PORT        I2C_NUM_0
#define CFG_IMU_I2C_SDA         38
#define CFG_IMU_I2C_SCL         39
#define CFG_IMU_I2C_FREQ_HZ     400000
#define CFG_IMU_I2C_ADDR        0x68    
#define CFG_IMU_CALIB_S         3   

//Prio_task
#define CFG_PRIO_IMU            6
#define CFG_PRIO_GPS            5

// Core
#define CFG_CORE_IMU             0
#define CFG_CORE_GPS             0

// Accel ±8g range
#define CFG_ACCEL_SCALE         (1.0f / 4096.0f * 9.81f)
// Gyro ±500°/s range
#define CFG_GYRO_SCALE          (1.0f / 65.5f * 3.14159265f / 180.0f)



#define CFG_PERIOD_IMU_MS       10   

//Complementary filter - IMU (pitch/roll)
#define CFG_CF_ALPHA_BASE       0.98f   
#define CFG_CF_ACCEL_ERR_SOFT   0.8f 
#define CFG_CF_ACCEL_ERR_HARD   1.5f    
#define CFG_CF_WARMUP_TICKS     50  

//Complementary filer - IMU+GPS (heading)
#define CFG_HDG_MIN_SPEED_MS    0.5f  
#define CFG_HDG_GPS_ALPHA       0.10f
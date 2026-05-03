#pragma once
#include "driver/i2c.h"

// Feature flags
// #define V2V_HW_STUB

// Test Mode switch
#define ENABLE_TEST_MODE        0

// UART stream for plotting
#define ENABLE_UART_PLOT        1

// GPS
#define CFG_GPS_UART_PORT       UART_NUM_1
#define CFG_GPS_UART_TX_PIN     17    
#define CFG_GPS_UART_RX_PIN     18    
#define CFG_GPS_UART_BAUD       9600
#define CFG_GPS_UART_BUF        512
#define CFG_GPS_SENTENCE_QLEN   4    

// IMU
#define CFG_IMU_I2C_PORT        I2C_NUM_0
#define CFG_IMU_I2C_SDA         38
#define CFG_IMU_I2C_SCL         39
#define CFG_IMU_I2C_FREQ_HZ     400000
#define CFG_IMU_I2C_ADDR        0x68    
#define CFG_IMU_CALIB_S         3   

// TFT ST7735
#define CFG_TFT_SPI_HOST        SPI2_HOST
#define CFG_TFT_MOSI_PIN        16
#define CFG_TFT_SCLK_PIN        15
#define CFG_TFT_CS_PIN          5
#define CFG_TFT_DC_PIN          2
#define CFG_TFT_RST_PIN         4
#define CFG_TFT_BL_PIN          6
#define CFG_TFT_SPI_SPEED_HZ    27000000
#define CFG_TFT_WIDTH           128
#define CFG_TFT_HEIGHT          160

// Stack sizes
#define CFG_STACK_IMU           3072
#define CFG_STACK_GPS           3072
#define CFG_STACK_FUSION        3072
#define CFG_STACK_LOCALIZATION  3584
#define CFG_STACK_V2V           4096
#define CFG_STACK_COLLISION     4096
#define CFG_STACK_DISPLAY_TFT   5120

// Prio_task
#define CFG_PRIO_IMU            6
#define CFG_PRIO_GPS            5
#define CFG_PRIO_FUSION         4
#define CFG_PRIO_LOCALIZATION   4
#define CFG_PRIO_V2V            3
#define CFG_PRIO_COLLISION      3
#define CFG_PRIO_DISPLAY_TFT    2

// Core
#define CFG_CORE_IMU            0
#define CFG_CORE_GPS            0
#define CFG_CORE_FUSION         0
#define CFG_CORE_LOCALIZATION   0
#define CFG_CORE_V2V            0
#define CFG_CORE_COLLISION      1
#define CFG_CORE_DISPLAY_TFT    1

// Period / timeout
#define CFG_PERIOD_IMU_MS       10   
#define CFG_PERIOD_FUSION_MS    10
#define CFG_PERIOD_V2V_MS       67
#define CFG_PERIOD_COLLISION_MS 100
#define CFG_PERIOD_TFT_MS       100

// Queue depths
#define CFG_QLEN_IMU            4
#define CFG_QLEN_GPS            2
#define CFG_QLEN_FUSION_OUT     2
#define CFG_QLEN_EGO_STATE      2
#define CFG_QLEN_V2V_RX         8
#define CFG_QLEN_COLLISION_IN   2
#define CFG_QLEN_ALERT          4

// V2V
#define CFG_ESPNOW_CHANNEL      1
#define CFG_ESPNOW_BCAST        {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}
#define CFG_PKT_STALE_MS        500  
#define CFG_PKT_MAGIC           0xB6 

// Accel ±8g range
#define CFG_ACCEL_SCALE         (1.0f / 4096.0f * 9.81f)
// Gyro ±500°/s range
#define CFG_GYRO_SCALE          (1.0f / 65.5f * 3.14159265f / 180.0f)

// Complementary filter - IMU (pitch/roll)
#define CFG_CF_ALPHA_BASE       0.98f   
#define CFG_CF_ACCEL_ERR_SOFT   0.8f 
#define CFG_CF_ACCEL_ERR_HARD   1.5f    
#define CFG_CF_WARMUP_TICKS     50  

// Complementary filter - IMU+GPS (heading)
#define CFG_HDG_GPS_ALPHA       0.10f
#define CFG_HDG_MIN_SPEED_MS    0.5f  

// Dead Reckoning
#define CFG_ZVU_SPEED_MS        0.3f

// Coordinate transportation 
#define CFG_M_PER_LAT_DEG       110540.0f
#define CFG_M_PER_LON_DEG_BASE  111320.0f

// EBBL
#define CFG_EBBL_BRAKE_MS2      (-2.5f)
#define CFG_EBBL_CONE_DEG       30.0f   
#define CFG_EBBL_MAX_DIST_M     150.0f 
#define CFG_EBBL_TTC_CRIT_S     2.0f   
#define CFG_EBBL_TTC_WARN_S     4.0f   
#define CFG_EBBL_TTC_INFO_S     6.0f 
#define CFG_EBBL_BURST_COUNT    3      
#define CFG_EBBL_BURST_MS       20      
#define CFG_EBBL_COOLDOWN_MS    500  

// IMA
#define CFG_IMA_RADIUS_M        80.0f  
#define CFG_IMA_ANGLE_MIN_DEG   45.0f  
#define CFG_IMA_ANGLE_MAX_DEG   135.0f  
#define CFG_IMA_DT_CRIT_S       1.0f   
#define CFG_IMA_DT_WARN_S       2.5f
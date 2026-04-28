#pragma once

// GPS 
#define CFG_GPS_UART_PORT       UART_NUM_1
#define CFG_GPS_UART_TX_PIN     17    
#define CFG_GPS_UART_RX_PIN     18    
#define CFG_GPS_UART_BAUD       9600
#define CFG_GPS_UART_BUF        512
#define CFG_GPS_SENTENCE_QLEN   4    

//Prio_task
#define CFG_PRIO_GPS            5

// Core
#define CFG_CORE_GPS             0
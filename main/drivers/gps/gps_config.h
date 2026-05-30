#pragma once

#include "driver/uart.h"

typedef struct {
    const char *name;
    uart_port_t uart_port;
    int tx_pin;
    int rx_pin;
    int baud_rate;
    int update_rate_hz;
} gps_module_config_t;

const gps_module_config_t *gps_get_active_config(void);

#pragma once
#include "esp_err.h"
#include <stdint.h>

typedef void *i2c_bus_handle_t; 

esp_err_t i2c_bus_init(int port, int sda_pin, int scl_pin,
                        uint32_t freq_hz, i2c_bus_handle_t *out);

esp_err_t i2c_bus_add_device(i2c_bus_handle_t bus, uint8_t addr,
                              uint32_t scl_speed_hz,
                              void **dev_out);


esp_err_t i2c_bus_write_reg(void *dev, uint8_t reg, uint8_t val);


esp_err_t i2c_bus_read_reg(void *dev, uint8_t reg, uint8_t *val);


esp_err_t i2c_bus_read_burst(void *dev, uint8_t reg,
                              uint8_t *buf, uint8_t len);

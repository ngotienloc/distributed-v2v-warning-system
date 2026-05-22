/* drivers/imu/i2c_bus.h — Lớp trừu tượng I2C dùng ESP-IDF i2c_master API. */
#pragma once
#include "esp_err.h"
#include <stdint.h>

/* Handle đại diện cho một I2C bus (thực chất là i2c_master_bus_handle_t) */
typedef void *i2c_bus_handle_t;

/* Khởi tạo I2C master bus trên port/pin/freq cho trước */
esp_err_t i2c_bus_init(int port, int sda_pin, int scl_pin,
                        uint32_t freq_hz, i2c_bus_handle_t *out);

/* Thêm một thiết bị I2C (7-bit addr) vào bus, trả về handle thiết bị */
esp_err_t i2c_bus_add_device(i2c_bus_handle_t bus, uint8_t addr,
                              uint32_t scl_speed_hz,
                              void **dev_out);

/* Ghi một byte vào thanh ghi reg */
esp_err_t i2c_bus_write_reg(void *dev, uint8_t reg, uint8_t val);

/* Đọc một byte từ thanh ghi reg */
esp_err_t i2c_bus_read_reg(void *dev, uint8_t reg, uint8_t *val);

/* Đọc liên tiếp len byte bắt đầu từ thanh ghi reg */
esp_err_t i2c_bus_read_burst(void *dev, uint8_t reg,
                              uint8_t *buf, uint8_t len);

#include "i2c_bus.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "i2c_bus";

#define I2C_TIMEOUT_MS 50

esp_err_t i2c_bus_init(int port, int sda_pin, int scl_pin,
                        uint32_t freq_hz, i2c_bus_handle_t *out)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port          = port,
        .sda_io_num        = sda_pin,
        .scl_io_num        = scl_pin,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t handle;
    esp_err_t ret = i2c_new_master_bus(&cfg, &handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bus init failed (port=%d): 0x%x", port, ret);
        return ret;
    }
    *out = (i2c_bus_handle_t)handle;
    ESP_LOGI(TAG, "I2C bus %d init OK (sda=%d scl=%d %lukHz)",
             port, sda_pin, scl_pin, (unsigned long)(freq_hz / 1000));
    return ESP_OK;
}

esp_err_t i2c_bus_add_device(i2c_bus_handle_t bus, uint8_t addr,
                              uint32_t scl_speed_hz, void **dev_out)
{
    i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = scl_speed_hz,
    };
    i2c_master_dev_handle_t dev;
    esp_err_t ret = i2c_master_bus_add_device(
        (i2c_master_bus_handle_t)bus, &dcfg, &dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Add device 0x%02X failed: 0x%x", addr, ret);
        return ret;
    }
    *dev_out = (void *)dev;
    return ESP_OK;
}

esp_err_t i2c_bus_write_reg(void *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit((i2c_master_dev_handle_t)dev,
                               buf, 2, I2C_TIMEOUT_MS);
}

esp_err_t i2c_bus_read_reg(void *dev, uint8_t reg, uint8_t *val)
{
    return i2c_bus_read_burst(dev, reg, val, 1);
}

esp_err_t i2c_bus_read_burst(void *dev, uint8_t reg,
                              uint8_t *buf, uint8_t len)
{
    return i2c_master_transmit_receive(
        (i2c_master_dev_handle_t)dev,
        &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

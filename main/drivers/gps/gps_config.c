#include "drivers/gps/gps_config.h"
#include "config.h"

static const gps_module_config_t kNeo6mConfig = {
    .name = "NEO-6M",
    .uart_port = CFG_GPS_NEO6M_UART_PORT,
    .tx_pin = CFG_GPS_NEO6M_UART_TX,
    .rx_pin = CFG_GPS_NEO6M_UART_RX,
    .baud_rate = CFG_GPS_NEO6M_BAUD,
    .update_rate_hz = CFG_GPS_NEO6M_UPDATE_RATE_HZ,
};

static const gps_module_config_t kNeo8mConfig = {
    .name = "NEO-8M",
    .uart_port = CFG_GPS_NEO8M_UART_PORT,
    .tx_pin = CFG_GPS_NEO8M_UART_TX,
    .rx_pin = CFG_GPS_NEO8M_UART_RX,
    .baud_rate = CFG_GPS_NEO8M_BAUD,
    .update_rate_hz = CFG_GPS_NEO8M_UPDATE_RATE_HZ,
};

const gps_module_config_t *gps_get_active_config(void)
{
#if CFG_GPS_MODULE == GPS_MODULE_NEO6M
    return &kNeo6mConfig;
#elif CFG_GPS_MODULE == GPS_MODULE_NEO8M
    return &kNeo8mConfig;
#else
#error "Unsupported CFG_GPS_MODULE"
#endif
}

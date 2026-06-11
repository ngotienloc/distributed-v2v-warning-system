#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tft_driver.h"
#include "config.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "tft";
static spi_device_handle_t s_spi = NULL;

/* ================================================================
 * SPI primitives
 * ================================================================ */

static inline void set_dc(int v) { gpio_set_level(CFG_TFT_DC_PIN, v); }
static inline void set_cs(int v) { gpio_set_level(CFG_TFT_CS_PIN, v); }

static void spi_tx(const uint8_t *buf, int len)
{
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = buf,
    };
    spi_device_polling_transmit(s_spi, &t);
}

static void tft_cmd(uint8_t cmd)
{
    set_dc(0); set_cs(0);
    spi_tx(&cmd, 1);
    set_cs(1);
}

static void tft_data_byte(uint8_t d)
{
    set_dc(1); set_cs(0);
    spi_tx(&d, 1);
    set_cs(1);
}

static void tft_data_buf(const uint8_t *buf, int len)
{
    set_dc(1); set_cs(0);
    spi_tx(buf, len);
    set_cs(1);
}

static void set_window(uint16_t x0, uint16_t y0,
                       uint16_t x1, uint16_t y1)
{
    uint8_t cx[4] = { x0 >> 8, x0 & 0xFF, x1 >> 8, x1 & 0xFF };
    uint8_t cy[4] = { y0 >> 8, y0 & 0xFF, y1 >> 8, y1 & 0xFF };
    tft_cmd(0x2A); tft_data_buf(cx, 4);
    tft_cmd(0x2B); tft_data_buf(cy, 4);
    tft_cmd(0x2C);   /* Memory write */
}

/* ================================================================
 * Public: init / clear
 * ================================================================ */

esp_err_t tft_init(void)
{
    /* GPIO setup */
    const int gpios[] = {
        CFG_TFT_DC_PIN, CFG_TFT_CS_PIN, CFG_TFT_RST_PIN, CFG_TFT_BL_PIN
    };
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(gpios[i]);
        gpio_set_direction(gpios[i], GPIO_MODE_OUTPUT);
    }
    gpio_set_level(CFG_TFT_CS_PIN, 1);
    gpio_set_level(CFG_TFT_BL_PIN, 1);   /* Backlight on */

    /* SPI bus */
    spi_bus_config_t buscfg = {
        .mosi_io_num    = CFG_TFT_MOSI_PIN,
        .miso_io_num    = -1,             /* ST7735 write-only */
        .sclk_io_num    = CFG_TFT_SCLK_PIN,
        .quadwp_io_num  = -1,
        .quadhd_io_num  = -1,
        .max_transfer_sz = CFG_TFT_WIDTH * CFG_TFT_HEIGHT * 2,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CFG_TFT_SPI_SPEED_HZ,
        .mode           = 0,
        .spics_io_num   = -1,     /* CS managed manually */
        .queue_size     = 7,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(CFG_TFT_SPI_HOST, &buscfg,
                                       SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(CFG_TFT_SPI_HOST, &devcfg, &s_spi));

    /* Hardware reset */
    gpio_set_level(CFG_TFT_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CFG_TFT_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    /* ST7735S init sequence (same as old project — verified working) */
    tft_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(150));  /* SWRESET */
    tft_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(500));  /* SLPOUT  */

    tft_cmd(0xB1);                                   /* FRMCTR1 */
    tft_data_byte(0x01); tft_data_byte(0x2C); tft_data_byte(0x2D);
    tft_cmd(0xB2);                                   /* FRMCTR2 */
    tft_data_byte(0x01); tft_data_byte(0x2C); tft_data_byte(0x2D);
    tft_cmd(0xB3);                                   /* FRMCTR3 */
    tft_data_byte(0x01); tft_data_byte(0x2C); tft_data_byte(0x2D);
    tft_data_byte(0x01); tft_data_byte(0x2C); tft_data_byte(0x2D);

    tft_cmd(0xB4); tft_data_byte(0x07);              /* INVCTR  */

    tft_cmd(0xC0);                                   /* PWCTR1  */
    tft_data_byte(0xA2); tft_data_byte(0x02); tft_data_byte(0x84);
    tft_cmd(0xC1); tft_data_byte(0xC5);              /* PWCTR2  */
    tft_cmd(0xC2); tft_data_byte(0x0A); tft_data_byte(0x00); /* PWCTR3 */
    tft_cmd(0xC3); tft_data_byte(0x8A); tft_data_byte(0x2A); /* PWCTR4 */
    tft_cmd(0xC4); tft_data_byte(0x8A); tft_data_byte(0xEE); /* PWCTR5 */
    tft_cmd(0xC5); tft_data_byte(0x0E);              /* VMCTR1  */

    tft_cmd(0x36); tft_data_byte(0xC8);  /* MADCTL: portrait 128×160, BGR */
    tft_cmd(0x3A); tft_data_byte(0x05);  /* COLMOD: RGB565 */

    /* Gamma */
    tft_cmd(0xE0);
    const uint8_t gp[] = {0x0F,0x1A,0x0F,0x18,0x2F,0x28,0x20,0x22,
                           0x1F,0x1B,0x23,0x37,0x00,0x07,0x02,0x10};
    tft_data_buf(gp, 16);
    tft_cmd(0xE1);
    const uint8_t gn[] = {0x0F,0x1B,0x0F,0x17,0x33,0x2C,0x29,0x2E,
                           0x30,0x30,0x39,0x3F,0x00,0x07,0x03,0x10};
    tft_data_buf(gn, 16);

    tft_cmd(0x13);                               /* NORON */
    tft_cmd(0x29); vTaskDelay(pdMS_TO_TICKS(100)); /* DISPON */

    tft_clear(TFT_BG_IDLE);
    ESP_LOGI(TAG, "ST7735 init OK (%dx%d @ %d MHz)",
             CFG_TFT_WIDTH, CFG_TFT_HEIGHT,
             CFG_TFT_SPI_SPEED_HZ / 1000000);
    return ESP_OK;
}

void tft_clear(uint16_t color)
{
    tft_fill_rect(0, 0, CFG_TFT_WIDTH, CFG_TFT_HEIGHT, color);
}

/* ================================================================
 * Public: fill_rect  (chunked, yields to watchdog)
 * ================================================================ */

void tft_fill_rect(uint16_t x, uint16_t y,
                   uint16_t w, uint16_t h, uint16_t color)
{
    if (w == 0 || h == 0) return;
    if (x + w > CFG_TFT_WIDTH)  w = CFG_TFT_WIDTH  - x;
    if (y + h > CFG_TFT_HEIGHT) h = CFG_TFT_HEIGHT - y;

    set_window(x, y, x + w - 1, y + h - 1);

#define CHUNK_PX 64
    uint8_t buf[CHUNK_PX * 2];
    uint8_t ch = color >> 8, cl = color & 0xFF;
    for (int i = 0; i < CHUNK_PX * 2; i += 2) { buf[i] = ch; buf[i+1] = cl; }

    uint32_t total = (uint32_t)w * h;
    uint32_t full  = total / CHUNK_PX;
    uint32_t rem   = total % CHUNK_PX;

    set_dc(1); set_cs(0);
    for (uint32_t i = 0; i < full; i++) {
        spi_tx(buf, CHUNK_PX * 2);
        if ((i & 63) == 0) vTaskDelay(0);  /* yield to watchdog */
    }
    if (rem) spi_tx(buf, rem * 2);
    set_cs(1);
}

/* ================================================================
 * Public: draw_pixel
 * ================================================================ */

void tft_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= CFG_TFT_WIDTH || y >= CFG_TFT_HEIGHT) return;
    tft_fill_rect(x, y, 1, 1, color);
}

/* ================================================================
 * Public: draw_line  (Bresenham)
 * ================================================================ */

void tft_draw_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    /* Guard: max 300 iterations to avoid blocking */
    for (int steps = 0; steps < 300; steps++) {
        tft_fill_rect((uint16_t)x0, (uint16_t)y0, 1, 1, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
        if ((steps & 15) == 0) vTaskDelay(0); /* Yield to watchdog */
    }
}

/* ================================================================
 * Public: draw_circle / fill_circle  (midpoint algorithm)
 * ================================================================ */

void tft_draw_circle(int cx, int cy, int r, uint16_t color)
{
    int x = 0, y = r, d = 1 - r;
    while (x <= y) {
        tft_draw_pixel(cx + x, cy + y, color);
        tft_draw_pixel(cx - x, cy + y, color);
        tft_draw_pixel(cx + x, cy - y, color);
        tft_draw_pixel(cx - x, cy - y, color);
        tft_draw_pixel(cx + y, cy + x, color);
        tft_draw_pixel(cx - y, cy + x, color);
        tft_draw_pixel(cx + y, cy - x, color);
        tft_draw_pixel(cx - y, cy - x, color);
        if (d < 0)   d += 2 * x + 3;
        else       { d += 2 * (x - y) + 5; y--; }
        x++;
    }
}

void tft_fill_circle(int cx, int cy, int r, uint16_t color)
{
    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)sqrtf((float)(r * r - dy * dy));
        int px = cx - dx, py = cy + dy;
        if (px < 0) px = 0;
        int pw = 2 * dx + 1;
        if (px + pw > CFG_TFT_WIDTH) pw = CFG_TFT_WIDTH - px;
        tft_fill_rect((uint16_t)px, (uint16_t)py, (uint16_t)pw, 1, color);
    }
}

/* ================================================================
 * Public: font 5×7 text
 * ================================================================ */

/* 96 printable ASCII chars (0x20–0x7F), each 5 columns of 7 bits */
static const uint8_t s_font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},
    {0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},
    {0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},
    {0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
    {0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},
    {0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},
    {0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},
    {0x04,0x02,0x01,0x02,0x04},{0x40,0x40,0x40,0x40,0x40},
    {0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},
    {0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},
    {0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},
    {0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},
    {0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x40,0x7C},
    {0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},
    {0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},
    {0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},
    {0x10,0x08,0x08,0x10,0x08},{0x00,0x00,0x00,0x00,0x00},
};

void tft_draw_char(uint16_t x, uint16_t y, char c,
                   uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (c < 32 || c > 127) c = '?';
    const uint8_t *g = s_font5x7[c - 32];

    /* Buffer for entire char (max scale=3 → 15×21 = 315 px = 630 B) */
    uint8_t cbuf[15 * 21 * 2];
    int idx = 0;

    for (int row = 0; row < 7; row++) {
        for (int sr = 0; sr < scale; sr++) {
            for (int col = 0; col < 5; col++) {
                uint16_t color = (g[col] & (1 << row)) ? fg : bg;
                uint8_t ch_byte = color >> 8, cl_byte = color & 0xFF;
                for (int sc = 0; sc < scale; sc++) {
                    if (idx < (int)sizeof(cbuf) - 1) {
                        cbuf[idx++] = ch_byte;
                        cbuf[idx++] = cl_byte;
                    }
                }
            }
        }
    }

    set_window(x, y, x + 5 * scale - 1, y + 7 * scale - 1);
    set_dc(1); set_cs(0);
    spi_tx(cbuf, idx);
    set_cs(1);
    vTaskDelay(0);  /* yield */
}

void tft_draw_str(uint16_t x, uint16_t y, const char *s,
                  uint16_t fg, uint16_t bg, uint8_t scale)
{
    while (*s) {
        tft_draw_char(x, y, *s++, fg, bg, scale);
        x += (uint16_t)(6 * scale);
    }
}

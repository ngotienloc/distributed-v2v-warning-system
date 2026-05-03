#pragma once
#include "esp_err.h"
#include <stdint.h>

/**
 * @file tft_driver.h
 * @brief ST7735 1.8" SPI TFT driver — low-level drawing primitives.
 *
 * Pin config: all from config.h (CFG_TFT_*).
 * This driver is stateless; no FreeRTOS calls except optional yield in
 * fill_rect. Safe to call from a single task.
 */

/* ──────────────────────────────────────────────────────────────
 * RGB565 color helpers
 * ────────────────────────────────────────────────────────────── */
#define TFT_RGB(r,g,b) \
    ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

/* Color palette */
#define TFT_BLACK       TFT_RGB(  0,   0,   0)
#define TFT_WHITE       TFT_RGB(255, 255, 255)
#define TFT_NAVY        TFT_RGB( 10,  18,  35)   /* dark background   */
#define TFT_GRAY        TFT_RGB(120, 120, 120)
#define TFT_CYAN        TFT_RGB(  0, 255, 255)
#define TFT_YELLOW      TFT_RGB(255, 255,   0)
#define TFT_ACCENT      TFT_RGB( 80, 160, 255)   /* highlight blue    */

/* Alert backgrounds */
#define TFT_BG_CRIT     TFT_RGB(180,  20,  20)
#define TFT_BG_WARN     TFT_RGB(200, 140,   0)
#define TFT_BG_INFO     TFT_RGB(  0, 120,  60)
#define TFT_BG_IDLE     TFT_RGB( 10,  18,  35)

/* Peer dot colors */
#define TFT_DOT_CRIT    TFT_RGB(255,  60,  60)
#define TFT_DOT_WARN    TFT_RGB(255, 200,   0)
#define TFT_DOT_INFO    TFT_RGB(  0, 200, 100)
#define TFT_DOT_SAFE    TFT_RGB( 80, 100, 140)
#define TFT_DOT_SELF    TFT_RGB( 80, 180, 255)

/* ──────────────────────────────────────────────────────────────
 * Init / clear
 * ────────────────────────────────────────────────────────────── */

/** @brief Initialize SPI bus + ST7735, turn on backlight. */
esp_err_t tft_init(void);

/** @brief Fill entire screen with one color. */
void tft_clear(uint16_t color);

/* ──────────────────────────────────────────────────────────────
 * Drawing primitives
 * ────────────────────────────────────────────────────────────── */

/** @brief Set one pixel. */
void tft_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

/** @brief Fill a rectangle. Chunked — yields to FreeRTOS every 64 rows. */
void tft_fill_rect(uint16_t x, uint16_t y,
                   uint16_t w, uint16_t h, uint16_t color);

/** @brief Bresenham line. */
void tft_draw_line(int x0, int y0, int x1, int y1, uint16_t color);

/** @brief Midpoint circle (unfilled ring, 1 px). */
void tft_draw_circle(int cx, int cy, int r, uint16_t color);

/** @brief Filled circle. */
void tft_fill_circle(int cx, int cy, int r, uint16_t color);

/** @brief Draw a single ASCII character (5×7 font, scalable). */
void tft_draw_char(uint16_t x, uint16_t y, char c,
                   uint16_t fg, uint16_t bg, uint8_t scale);

/** @brief Draw a null-terminated ASCII string. */
void tft_draw_str(uint16_t x, uint16_t y, const char *s,
                  uint16_t fg, uint16_t bg, uint8_t scale);

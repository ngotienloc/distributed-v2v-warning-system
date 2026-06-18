#pragma once

/* ═══════════════════════════════════════════════════════════════
 * config.h — Test 2.3: ESP-NOW Range Test (Symmetric / Both-TFT)
 * Cả 2 board đều dùng TFT ST7735. Flash cùng 1 firmware cho cả 2.
 * Bên nào nhấn BOOT trước → TX burst 500 gói, bên kia nhận + hiển thị.
 * ═══════════════════════════════════════════════════════════════ */

/* ── TFT ST7735 (SPI) ────────────────────────────────────────────────── */
#define CFG_TFT_SPI_HOST        SPI2_HOST
#define CFG_TFT_MOSI_PIN        35
#define CFG_TFT_SCLK_PIN        36
#define CFG_TFT_CS_PIN          20
#define CFG_TFT_DC_PIN          2
#define CFG_TFT_RST_PIN         21
#define CFG_TFT_BL_PIN          37
#define CFG_TFT_SPI_SPEED_HZ    27000000
#define CFG_TFT_WIDTH           128
#define CFG_TFT_HEIGHT          160

/* ── ESP-NOW ─────────────────────────────────────────────────────────── */
#define CFG_ESPNOW_CHANNEL      1

/* ── Nút nhấn để kích hoạt TX burst ──────────────────────────────────── */
/* Dùng BOOT button (GPIO0) — active LOW, có pull-up nội */
#define CFG_BTN_GPIO            0

/* ── Tham số test range ──────────────────────────────────────────────── */
/* Chuẩn V2V: broadcast 15 Hz → chu kỳ 67ms */
#define CFG_TEST_TOTAL_PKTS     500        /* số gói mỗi lần bấm nút      */
#define CFG_TEST_PKT_INTERVAL_MS 67        /* khoảng cách giữa gói (ms)   */
#define CFG_PKT_MAGIC           0xB6       /* magic byte nhận dạng gói    */

/* ── Stack / Priority ────────────────────────────────────────────────── */
#define CFG_STACK_TX            3072
#define CFG_STACK_RX            3072
#define CFG_STACK_TFT           4096

#define CFG_PRIO_TX             5
#define CFG_PRIO_RX             5
#define CFG_PRIO_TFT            3

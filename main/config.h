#pragma once

/* ═══════════════════════════════════════════════════════════════
 * CHỌN CHẾ ĐỘ HIỂN THỊ
 *   - Comment dòng dưới  → dùng TFT ST7735 (board có màn hình)
 *   - Uncomment dòng dưới → dùng UART log  (board không có TFT)
 * ════════════════════════════════════════════════════════════════ */
 #define V2V_NO_TFT   /* ← Bỏ dấu // nếu board không có TFT */

/* ═══════════════════════════════════════════════════════════════
 * config.h — Test 2.3: ESP-NOW Range Test
 * Chỉ giữ lại: TFT (ST7735), ESP-NOW, nút nhấn (BOOT/GPIO0).
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

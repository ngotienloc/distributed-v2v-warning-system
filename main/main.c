/* main.c — Test 2.3: ESP-NOW Range Test
 *
 * Pipeline:
 *   task_tx  : Chờ nút nhấn → gửi 500 gói V2V (67ms/gói)
 *   task_rx  : Nhận gói từ ESP-NOW callback → đếm, tính error rate
 *   task_tft : Hiển thị kết quả lên TFT (5 Hz refresh)
 *
 * KHÔNG dùng: GPS, IMU, Fusion, Localization, Collision.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "config.h"
#include "types.h"
#include "v2v/espnow_comm.h"
#ifndef V2V_NO_TFT
#  include "drivers/tft/tft_driver.h"
#endif
#include <string.h>
#include <math.h>
#include <stdio.h>

static const char *TAG = "main";

/* ── Queue RX: WiFi task → task_rx ───────────────────────────────────── */
static QueueHandle_t q_rx;

/* ── Shared stats (task_rx viết, task_tft đọc) ─────────────────────── */
/* Dùng critical section đơn giản (portENTER/EXIT_CRITICAL) vì nhỏ gọn */
typedef struct {
    uint32_t total_expected;   /* tổng gói TX phía kia đã gửi (lấy từ seq+1) */
    uint32_t total_received;   /* số gói nhận được thực tế */
    uint32_t last_seq;         /* seq của gói cuối cùng nhận */
    bool     session_done;     /* TX phía kia đã hoàn thành burst chưa */
} rx_stats_t;

static volatile rx_stats_t s_stats = {0};
static portMUX_TYPE        s_mux   = portMUX_INITIALIZER_UNLOCKED;

/* ── TX state (task_tx viết, task_tft đọc) ──────────────────────────── */
static volatile uint32_t s_tx_sent   = 0;   /* số gói đã gửi */
static volatile bool     s_tx_active = false;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ════════════════════════════════════════════════════════════════════════
 * task_tx — Chờ nút BOOT (GPIO0, active LOW) → gửi 500 gói
 * ════════════════════════════════════════════════════════════════════════ */
void task_tx(void *arg)
{
    /* Config GPIO0 làm input pull-up */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CFG_BTN_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    /* Lấy 4 byte cuối MAC làm ID */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    ESP_LOGI(TAG, "task_tx ready — press BOOT to start %d-packet burst",
             CFG_TEST_TOTAL_PKTS);

    while (1) {
        /* Chờ nút nhấn (low) */
        if (gpio_get_level(CFG_BTN_GPIO) != 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* Debounce */
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(CFG_BTN_GPIO) != 0) continue;

        ESP_LOGI(TAG, "Button pressed — starting TX burst (%d pkts, %d ms/pkt)",
                 CFG_TEST_TOTAL_PKTS, CFG_TEST_PKT_INTERVAL_MS);

        /* Reset bộ đếm TX */
        espnow_reset_tx_ok();
        s_tx_sent   = 0;
        s_tx_active = true;

        TickType_t tick = xTaskGetTickCount();

        for (uint32_t seq = 0; seq < CFG_TEST_TOTAL_PKTS; seq++) {
            v2v_packet_t pkt = {0};
            pkt.magic    = CFG_PKT_MAGIC;
            pkt.seq      = seq;
            pkt.tx_ts_ms = now_ms();
            memcpy(pkt.id, mac + 2, 4);    /* 4 byte cuối MAC */

            esp_err_t err = espnow_broadcast(&pkt);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "broadcast err seq=%"PRIu32" err=%d", seq, err);
            }

            s_tx_sent = seq + 1;

            /* Duy trì đúng chu kỳ 67ms */
            vTaskDelayUntil(&tick, pdMS_TO_TICKS(CFG_TEST_PKT_INTERVAL_MS));
        }

        s_tx_active = false;

        /* Chờ callback on_send xử lý hết (~200ms) */
        vTaskDelay(pdMS_TO_TICKS(300));

        uint32_t tx_ok = espnow_get_tx_ok();
        ESP_LOGI(TAG, "TX burst done: sent=%"PRIu32" tx_ok=%"PRIu32
                 " loss=%.1f%%",
                 (uint32_t)CFG_TEST_TOTAL_PKTS, tx_ok,
                 100.0f * (CFG_TEST_TOTAL_PKTS - tx_ok) / CFG_TEST_TOTAL_PKTS);

        /* Chờ nhả nút */
        while (gpio_get_level(CFG_BTN_GPIO) == 0)
            vTaskDelay(pdMS_TO_TICKS(20));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * task_rx — Nhận gói từ queue, cập nhật thống kê
 * ════════════════════════════════════════════════════════════════════════ */
void task_rx(void *arg)
{
    v2v_packet_t pkt;
    uint32_t prev_seq    = UINT32_MAX;
    bool     first_pkt   = true;

    ESP_LOGI(TAG, "task_rx ready — waiting for V2V packets");

    while (1) {
        if (xQueueReceive(q_rx, &pkt, portMAX_DELAY) != pdTRUE) continue;

        portENTER_CRITICAL(&s_mux);

        if (first_pkt || pkt.seq == 0) {
            /* Gói đầu tiên của burst mới — reset stats */
            s_stats.total_expected = 0;
            s_stats.total_received = 0;
            s_stats.session_done   = false;
            first_pkt = false;
        }

        s_stats.total_received++;
        s_stats.last_seq = pkt.seq;

        /* Ước tính tổng dự kiến = seq cao nhất + 1
         * (nếu mất gói cuối thì khi nhận xong sẽ cập nhật từ TX side) */
        if (pkt.seq + 1 > s_stats.total_expected)
            s_stats.total_expected = pkt.seq + 1;

        /* Đánh dấu hoàn thành khi nhận được gói cuối (seq = 499) */
        if (pkt.seq == CFG_TEST_TOTAL_PKTS - 1)
            s_stats.session_done = true;

        portEXIT_CRITICAL(&s_mux);

        (void)prev_seq;
        prev_seq = pkt.seq;
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * task_tft / task_uart — Hiển thị kết quả (5 Hz)
 *
 * Biên dịch với V2V_NO_TFT  → in ra UART (idf monitor)
 * Biên dịch bình thường     → vẽ lên TFT ST7735 128×160
 * ════════════════════════════════════════════════════════════════════════ */

/* ── Helper dùng chung: tính error rate ─────────────────────────────── */
static void calc_stats(rx_stats_t *snap,
                       uint32_t   *tx_sent_out,
                       bool       *tx_active_out,
                       uint32_t   *tx_ok_out,
                       float      *err_rate_out)
{
    *tx_sent_out   = s_tx_sent;
    *tx_active_out = s_tx_active;
    *tx_ok_out     = espnow_get_tx_ok();

    portENTER_CRITICAL(&s_mux);
    *snap = (rx_stats_t){ s_stats.total_expected,
                          s_stats.total_received,
                          s_stats.last_seq,
                          s_stats.session_done };
    portEXIT_CRITICAL(&s_mux);

    uint32_t expected = snap->total_expected > 0
                        ? snap->total_expected
                        : (uint32_t)CFG_TEST_TOTAL_PKTS;
    *err_rate_out = 0.0f;
    if (expected > 0 && snap->total_received <= expected)
        *err_rate_out = 100.0f * (expected - snap->total_received) / expected;
}

/* ════════════════════════════════════════════════
 * Chế độ UART (V2V_NO_TFT defined)
 * ════════════════════════════════════════════════ */
#ifdef V2V_NO_TFT

void task_tft(void *arg)   /* giữ tên task_tft để app_main dùng chung */
{
    ESP_LOGI(TAG, "[NO-TFT] UART output mode — 2 Hz refresh");
    ESP_LOGI(TAG, "Press BOOT button to start TX burst (%d pkts)",
             CFG_TEST_TOTAL_PKTS);

    static uint32_t s_prev_rx = UINT32_MAX;   /* tránh log trùng */
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500)); /* 2 Hz */

        rx_stats_t stats;
        uint32_t tx_sent, tx_ok;
        bool     tx_active;
        float    err_rate;
        calc_stats(&stats, &tx_sent, &tx_active, &tx_ok, &err_rate);

        /* ── TX đang gửi: log tiến độ mỗi 50 gói ────────────── */
        if (tx_active) {
            ESP_LOGI(TAG, "[TX] Sending %"PRIu32"/%d ...",
                     tx_sent, CFG_TEST_TOTAL_PKTS);
            continue;
        }

        /* ── TX vừa xong: log kết quả 1 lần ────────────────── */
        if (tx_sent > 0 && tx_ok != s_prev_rx) {
            float tx_loss = 0.0f;
            if (CFG_TEST_TOTAL_PKTS > 0)
                tx_loss = 100.0f * (int)(CFG_TEST_TOTAL_PKTS - tx_ok)
                          / CFG_TEST_TOTAL_PKTS;
            if (tx_loss < 0.0f) tx_loss = 0.0f;
            ESP_LOGI(TAG, "╔══════════════════════════════════╗");
            ESP_LOGI(TAG, "║  TX RESULT                       ║");
            ESP_LOGI(TAG, "║  Sent   : %4d pkts              ║", CFG_TEST_TOTAL_PKTS);
            ESP_LOGI(TAG, "║  TX OK  : %4"PRIu32" (ACK by radio) ║", tx_ok);
            ESP_LOGI(TAG, "║  TX loss: %5.1f%%                ║", tx_loss);
            ESP_LOGI(TAG, "╚══════════════════════════════════╝");
            s_prev_rx = tx_ok;
        }

        /* ── RX: log liên tục khi đang nhận hoặc khi done ─── */
        if (stats.total_received != s_prev_rx || stats.session_done) {
            uint32_t expected = stats.total_expected > 0
                                ? stats.total_expected
                                : (uint32_t)CFG_TEST_TOTAL_PKTS;
            ESP_LOGI(TAG, "[RX] recv=%"PRIu32"/%"PRIu32
                     "  err=%.1f%%  last_seq=%"PRIu32"%s",
                     stats.total_received, expected,
                     err_rate, stats.last_seq,
                     stats.session_done ? "  << DONE >>" : "");
        }
    }
}

/* ════════════════════════════════════════════════
 * Chế độ TFT (mặc định)
 * ════════════════════════════════════════════════ */
#else  /* !V2V_NO_TFT */

#define TFT_BG          TFT_RGB( 10,  18,  35)
#define TFT_ACCENT_COL  TFT_RGB( 80, 160, 255)
#define TFT_GREEN       TFT_RGB(  0, 200,  80)
#define TFT_RED_COL     TFT_RGB(220,  40,  40)
#define TFT_PANEL_LABEL TFT_RGB(255, 200,   0)
#define TFT_GRAY        TFT_RGB(120, 120, 120)
#define TFT_DIVIDER     TFT_RGB( 30,  50,  80)

void task_tft(void *arg)
{
    if (tft_init() != ESP_OK) {
        ESP_LOGE(TAG, "TFT init failed");
        vTaskDelete(NULL);
    }

    /* Boot splash */
    tft_clear(TFT_BG);
    tft_draw_str(4,  20, "ESP-NOW",    TFT_ACCENT_COL, TFT_BG, 2);
    tft_draw_str(4,  44, "RANGE",      TFT_ACCENT_COL, TFT_BG, 2);
    tft_draw_str(4,  68, "TEST",       TFT_ACCENT_COL, TFT_BG, 2);
    tft_draw_str(4, 100, "Press BOOT", TFT_GRAY,       TFT_BG, 1);
    tft_draw_str(4, 112, "to start TX",TFT_GRAY,       TFT_BG, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));

    TickType_t last_wake = xTaskGetTickCount();
    char buf[32];

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200)); /* 5 Hz */

        rx_stats_t stats;
        uint32_t tx_sent, tx_ok;
        bool     tx_active;
        float    err_rate;
        calc_stats(&stats, &tx_sent, &tx_active, &tx_ok, &err_rate);

        /* ── Header ──────────────────────────────────────────── */
        tft_fill_rect(0, 0, 128, 18, TFT_RGB(15, 25, 50));
        tft_draw_str(4, 4, "ESP-NOW RANGE TEST",
                     TFT_ACCENT_COL, TFT_RGB(15,25,50), 1);
        tft_fill_rect(0, 18, 128, 1, TFT_DIVIDER);

        /* ── TX Panel (y 19..78) ─────────────────────────────── */
        tft_fill_rect(0, 19, 128, 60, TFT_BG);
        tft_draw_str(4, 21, "[ TX ]", TFT_PANEL_LABEL, TFT_BG, 1);

        if (tx_active) {
            snprintf(buf, sizeof(buf), "Sending %3"PRIu32"/%d",
                     tx_sent, CFG_TEST_TOTAL_PKTS);
            tft_draw_str(4, 33, buf, TFT_WHITE, TFT_BG, 1);
            uint16_t fill = (uint16_t)(120 * tx_sent / CFG_TEST_TOTAL_PKTS);
            tft_fill_rect(4, 45, 120, 8, TFT_RGB(30,50,80));
            tft_fill_rect(4, 45, fill, 8, TFT_GREEN);
            tft_draw_str(4, 57, "TX OK: --", TFT_GRAY, TFT_BG, 1);
        } else if (tx_sent > 0) {
            snprintf(buf, sizeof(buf), "Done: %d pkts", CFG_TEST_TOTAL_PKTS);
            tft_draw_str(4, 33, buf, TFT_GREEN, TFT_BG, 1);
            tft_fill_rect(4, 45, 120, 8, TFT_GREEN);
            float tx_loss = 0.0f;
            if (CFG_TEST_TOTAL_PKTS > 0)
                tx_loss = 100.0f * (CFG_TEST_TOTAL_PKTS - (int)tx_ok)
                          / CFG_TEST_TOTAL_PKTS;
            if (tx_loss < 0.0f) tx_loss = 0.0f;
            snprintf(buf, sizeof(buf), "TXok:%"PRIu32" err:%.1f%%", tx_ok, tx_loss);
            tft_draw_str(4, 57, buf, TFT_ACCENT_COL, TFT_BG, 1);
        } else {
            tft_draw_str(4, 33, "Press BOOT btn", TFT_GRAY, TFT_BG, 1);
            tft_fill_rect(4, 45, 120, 8, TFT_RGB(30,50,80));
            tft_draw_str(4, 57, "Waiting...",     TFT_GRAY, TFT_BG, 1);
        }

        tft_fill_rect(0, 79, 128, 1, TFT_DIVIDER);

        /* ── RX Panel (y 80..159) ────────────────────────────── */
        tft_fill_rect(0, 80, 128, 80, TFT_BG);
        tft_draw_str(4, 82, "[ RX ]", TFT_PANEL_LABEL, TFT_BG, 1);

        uint32_t expected = stats.total_expected > 0
                            ? stats.total_expected
                            : (uint32_t)CFG_TEST_TOTAL_PKTS;
        snprintf(buf, sizeof(buf), "Recv: %"PRIu32"/%"PRIu32,
                 stats.total_received, expected);
        tft_draw_str(4, 94, buf, TFT_WHITE, TFT_BG, 1);

        uint16_t err_col = (err_rate < 5.0f)  ? TFT_GREEN :
                           (err_rate < 20.0f) ? TFT_YELLOW : TFT_RED_COL;
        snprintf(buf, sizeof(buf), "Err: %.1f%%", err_rate);
        tft_draw_str(4, 106, buf, err_col, TFT_BG, 2);

        uint16_t err_px = (uint16_t)(120.0f * (100.0f - err_rate) / 100.0f);
        tft_fill_rect(4, 128, 120, 8, TFT_RGB(30,50,80));
        tft_fill_rect(4, 128, err_px, 8, err_col);

        if (stats.session_done) {
            snprintf(buf, sizeof(buf), "Seq: %"PRIu32, stats.last_seq);
            tft_draw_str(4, 140, buf,    TFT_GRAY,  TFT_BG, 1);
            tft_draw_str(4, 150, "DONE!", TFT_GREEN, TFT_BG, 1);
        } else if (stats.total_received > 0) {
            tft_draw_str(4, 140, "Receiving...", TFT_ACCENT_COL, TFT_BG, 1);
        } else {
            tft_draw_str(4, 140, "No RX yet",   TFT_GRAY, TFT_BG, 1);
        }
    }
}

#endif  /* V2V_NO_TFT */

/* ════════════════════════════════════════════════════════════════════════
 * app_main
 * ════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "═════════════════════════════════");
    ESP_LOGI(TAG, " Test 2.3 — ESP-NOW Range Test");
    ESP_LOGI(TAG, "═════════════════════════════════");

    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Queue RX */
    q_rx = xQueueCreate(16, sizeof(v2v_packet_t));
    configASSERT(q_rx);

    /* ESP-NOW */
    ESP_ERROR_CHECK(espnow_init(q_rx));

    /* 3 Tasks */
    xTaskCreatePinnedToCore(task_tx,  "tx",  CFG_STACK_TX,  NULL, CFG_PRIO_TX,  NULL, 0);
    xTaskCreatePinnedToCore(task_rx,  "rx",  CFG_STACK_RX,  NULL, CFG_PRIO_RX,  NULL, 0);
#ifdef V2V_NO_TFT
    /* Không có TFT — task_tft in ra UART, ít stack hơn */
    xTaskCreatePinnedToCore(task_tft, "uart_disp", 3584, NULL, CFG_PRIO_TFT, NULL, 1);
    ESP_LOGI(TAG, "Mode: NO-TFT — results via UART log");
#else
    xTaskCreatePinnedToCore(task_tft, "tft",  CFG_STACK_TFT, NULL, CFG_PRIO_TFT, NULL, 1);
    ESP_LOGI(TAG, "Mode: TFT display enabled");
#endif

    ESP_LOGI(TAG, "3 tasks created — scheduler running");
}

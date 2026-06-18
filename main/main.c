/* main.c — Test 2.3: ESP-NOW Range Test (Symmetric / Both-TFT mode)
 *
 * ┌─────────────────────┐      ESP-NOW       ┌──────────────────────┐
 * │  Board A (TFT)      │ ─── 500 data pkts ─▶ │  Board B (TFT)       │
 * │                     │                       │                      │
 * │  Nhấn BOOT          │      ESP-NOW       │  Nhấn BOOT           │
 * │  → TX burst 500 gói │ ◀── 500 data pkts ── │  → TX burst 500 gói  │
 * │  → Hiển thị RX      │                       │  → Hiển thị RX       │
 * └─────────────────────┘                       └──────────────────────┘
 *
 * Hai board đều flash cùng 1 firmware (không cần #define riêng).
 * Bên nào nhấn BOOT trước: gửi TX burst 500 gói.
 * Bên kia: nhận và hiển thị stats trên màn TFT.
 * Cả hai đều có thể TX và RX xen kẽ tùy ai nhấn BOOT trước.
 *
 * Phân loại gói qua trường alert_type:
 *   PKT_TYPE_DATA (0x00) — gói dữ liệu range test (seq 0..499)
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
#include "drivers/tft/tft_driver.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <inttypes.h>

static const char *TAG = "main";

/* ── Phân loại gói ────────────────────────────────────────────────────── */
#define PKT_TYPE_DATA    0x00   /* gói dữ liệu range test */

/* ── Queue RX ────────────────────────────────────────────────────────── */
static QueueHandle_t q_rx;

/* ── RX stats (task_rx viết, task_tft đọc) ──────────────────────────── */
typedef struct {
    uint32_t total_expected;
    uint32_t total_received;
    uint32_t last_seq;
    bool     session_done;
    uint8_t  peer_id[4];        /* MAC 4 byte của board đang gửi */
} rx_stats_t;

static volatile rx_stats_t s_stats   = {0};
static portMUX_TYPE        s_mux     = portMUX_INITIALIZER_UNLOCKED;

/* ── TX state (task_tx viết, task_tft đọc) ──────────────────────────── */
static volatile uint32_t s_tx_sent   = 0;
static volatile bool     s_tx_active = false;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ── Helper: đọc MAC 4 byte cuối ────────────────────────────────────── */
static void get_mac_id(uint8_t id[4])
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    memcpy(id, mac + 2, 4);
}

/* ── Helper: cấu hình GPIO BOOT button (GPIO0, active LOW) ─────────── */
static void btn_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CFG_BTN_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

/* ════════════════════════════════════════════════════════════════════════
 * task_rx — Nhận gói PKT_TYPE_DATA từ queue ESP-NOW, cập nhật RX stats
 * ════════════════════════════════════════════════════════════════════════ */
void task_rx(void *arg)
{
    bool first_data = true;

    ESP_LOGI(TAG, "task_rx ready");

    while (1) {
        v2v_packet_t pkt;
        if (xQueueReceive(q_rx, &pkt, portMAX_DELAY) != pdTRUE) continue;

        /* Chỉ xử lý gói DATA */
        if (pkt.alert_type != PKT_TYPE_DATA) continue;

        portENTER_CRITICAL(&s_mux);

        /* Phát hiện session mới:
         *   - peer_id thay đổi (board khác gửi)
         *   - seq = 0 (burst mới từ cùng board)
         *   - first_data (khởi động lần đầu) */
        bool new_peer   = (memcmp((const void *)s_stats.peer_id, pkt.id, 4) != 0);
        bool new_session = first_data || new_peer || (pkt.seq == 0);

        if (new_session) {
            s_stats.total_expected = 0;
            s_stats.total_received = 0;
            s_stats.session_done   = false;
            memcpy((void *)s_stats.peer_id, pkt.id, 4);
            first_data = false;
        }

        s_stats.total_received++;
        s_stats.last_seq = pkt.seq;

        if (pkt.seq + 1 > s_stats.total_expected)
            s_stats.total_expected = pkt.seq + 1;

        if (pkt.seq == CFG_TEST_TOTAL_PKTS - 1)
            s_stats.session_done = true;

        portEXIT_CRITICAL(&s_mux);
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * task_tx — Chờ nút BOOT nhấn → TX burst 500 gói data
 * ════════════════════════════════════════════════════════════════════════ */
void task_tx(void *arg)
{
    btn_init();

    uint8_t my_id[4];
    get_mac_id(my_id);

    ESP_LOGI(TAG, "task_tx ready — press BOOT to start TX burst (%d pkts @ %d ms/pkt)",
             CFG_TEST_TOTAL_PKTS, CFG_TEST_PKT_INTERVAL_MS);

    while (1) {
        /* ── Chờ nút BOOT nhấn xuống (active LOW) ───────────── */
        if (gpio_get_level(CFG_BTN_GPIO) != 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        /* Debounce */
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(CFG_BTN_GPIO) != 0) continue;

        /* Không TX đè lên burst đang chạy */
        if (s_tx_active) {
            /* Chờ nhả nút rồi bỏ qua */
            while (gpio_get_level(CFG_BTN_GPIO) == 0)
                vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        ESP_LOGI(TAG, "BOOT pressed — TX burst START (%d pkts, %d ms/pkt)",
                 CFG_TEST_TOTAL_PKTS, CFG_TEST_PKT_INTERVAL_MS);

        /* Reset TX counters */
        espnow_reset_tx_ok();
        s_tx_sent  = 0;
        s_tx_active = true;

        TickType_t tick = xTaskGetTickCount();

        for (uint32_t seq = 0; seq < CFG_TEST_TOTAL_PKTS; seq++) {
            v2v_packet_t pkt = {0};
            pkt.magic      = CFG_PKT_MAGIC;
            pkt.alert_type = PKT_TYPE_DATA;
            pkt.seq        = seq;
            pkt.tx_ts_ms   = now_ms();
            memcpy(pkt.id, my_id, 4);

            esp_err_t err = espnow_broadcast(&pkt);
            if (err != ESP_OK)
                ESP_LOGW(TAG, "broadcast err seq=%" PRIu32 " err=%d", seq, err);

            s_tx_sent = seq + 1;

            vTaskDelayUntil(&tick, pdMS_TO_TICKS(CFG_TEST_PKT_INTERVAL_MS));
        }

        s_tx_active = false;

        /* Chờ callback on_send flush */
        vTaskDelay(pdMS_TO_TICKS(300));

        uint32_t tx_ok = espnow_get_tx_ok();
        float tx_loss  = 0.0f;
        if (CFG_TEST_TOTAL_PKTS > 0)
            tx_loss = 100.0f * (int)(CFG_TEST_TOTAL_PKTS - tx_ok) / CFG_TEST_TOTAL_PKTS;
        if (tx_loss < 0.0f) tx_loss = 0.0f;

        ESP_LOGI(TAG, "TX burst DONE: sent=%d  tx_ok=%" PRIu32 "  loss=%.1f%%",
                 CFG_TEST_TOTAL_PKTS, tx_ok, tx_loss);

        /* Chờ nhả nút + cooldown */
        while (gpio_get_level(CFG_BTN_GPIO) == 0)
            vTaskDelay(pdMS_TO_TICKS(20));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * Helper: snapshot stats cho task_tft
 * ════════════════════════════════════════════════════════════════════════ */
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
    *snap = (rx_stats_t){
        .total_expected = s_stats.total_expected,
        .total_received = s_stats.total_received,
        .last_seq       = s_stats.last_seq,
        .session_done   = s_stats.session_done,
    };
    memcpy(snap->peer_id, (const void *)s_stats.peer_id, 4);
    portEXIT_CRITICAL(&s_mux);

    uint32_t expected = snap->total_expected > 0
                        ? snap->total_expected
                        : (uint32_t)CFG_TEST_TOTAL_PKTS;
    *err_rate_out = 0.0f;
    if (expected > 0 && snap->total_received <= expected)
        *err_rate_out = 100.0f * (expected - snap->total_received) / expected;
}

/* ════════════════════════════════════════════════════════════════════════
 * task_tft — Hiển thị lên TFT (cả 2 board đều dùng hàm này)
 *
 * Layout 128×160:
 *   [0..17]   Header: "ESP-NOW RANGE TEST"
 *   [18..1]   Divider
 *   [19..78]  Panel TX: trạng thái TX của board này
 *   [79]      Divider
 *   [80..159] Panel RX: stats nhận từ board kia
 * ════════════════════════════════════════════════════════════════════════ */
#define TFT_BG          TFT_RGB( 10,  18,  35)
#define TFT_ACCENT_COL  TFT_RGB( 80, 160, 255)
#define TFT_GREEN       TFT_RGB(  0, 200,  80)
#define TFT_RED_COL     TFT_RGB(220,  40,  40)
#define TFT_PANEL_LABEL TFT_RGB(255, 200,   0)
#define TFT_GRAY        TFT_RGB(120, 120, 120)
#define TFT_DIVIDER     TFT_RGB( 30,  50,  80)
#define TFT_ORANGE      TFT_RGB(255, 140,   0)

void task_tft(void *arg)
{
    if (tft_init() != ESP_OK) {
        ESP_LOGE(TAG, "TFT init failed");
        vTaskDelete(NULL);
    }

    /* Boot splash */
    tft_clear(TFT_BG);
    tft_draw_str(4, 16, "ESP-NOW",    TFT_ACCENT_COL, TFT_BG, 2);
    tft_draw_str(4, 40, "RANGE TEST", TFT_ACCENT_COL, TFT_BG, 2);
    tft_draw_str(4, 72, "Press BOOT", TFT_GRAY,       TFT_BG, 1);
    tft_draw_str(4, 84, "to TX burst",TFT_GRAY,       TFT_BG, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));

    TickType_t last_wake = xTaskGetTickCount();
    char       buf[32];

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200)); /* 5 Hz */

        rx_stats_t stats;
        uint32_t tx_sent, tx_ok;
        bool     tx_active;
        float    err_rate;
        calc_stats(&stats, &tx_sent, &tx_active, &tx_ok, &err_rate);

        /* ── Header ────────────────────────────────────────────── */
        tft_fill_rect(0, 0, 128, 18, TFT_RGB(15, 25, 50));
        tft_draw_str(4, 4, "ESP-NOW RANGE TEST",
                     TFT_ACCENT_COL, TFT_RGB(15, 25, 50), 1);
        tft_fill_rect(0, 18, 128, 1, TFT_DIVIDER);

        /* ── Panel TX (y 19..78) ───────────────────────────────── */
        tft_fill_rect(0, 19, 128, 60, TFT_BG);
        tft_draw_str(4, 21, "[ TX (this board) ]", TFT_PANEL_LABEL, TFT_BG, 1);

        if (tx_active) {
            /* Đang TX burst */
            tft_draw_str(4, 33, "TX-ing...", TFT_ORANGE, TFT_BG, 1);
            snprintf(buf, sizeof(buf), "%"PRIu32"/%d pkts", tx_sent, CFG_TEST_TOTAL_PKTS);
            tft_draw_str(4, 45, buf, TFT_WHITE, TFT_BG, 1);

            /* Progress bar */
            uint16_t fill = (CFG_TEST_TOTAL_PKTS > 0)
                            ? (uint16_t)(120 * tx_sent / CFG_TEST_TOTAL_PKTS) : 0;
            tft_fill_rect(4, 60, 120, 8, TFT_RGB(30, 50, 80));
            tft_fill_rect(4, 60, fill, 8, TFT_ORANGE);
        } else if (tx_sent > 0) {
            /* Vừa xong TX burst */
            float loss = 0.0f;
            if (CFG_TEST_TOTAL_PKTS > 0)
                loss = 100.0f * (int)(CFG_TEST_TOTAL_PKTS - tx_ok)
                       / CFG_TEST_TOTAL_PKTS;
            if (loss < 0.0f) loss = 0.0f;

            tft_draw_str(4, 33, "TX DONE!", TFT_GREEN, TFT_BG, 1);
            snprintf(buf, sizeof(buf), "OK:%"PRIu32"/%d", tx_ok, CFG_TEST_TOTAL_PKTS);
            tft_draw_str(4, 45, buf, TFT_WHITE, TFT_BG, 1);
            snprintf(buf, sizeof(buf), "Loss:%.1f%%", loss);
            uint16_t lc = (loss < 5.0f) ? TFT_GREEN
                        : (loss < 20.0f) ? TFT_YELLOW : TFT_RED_COL;
            tft_draw_str(4, 57, buf, lc, TFT_BG, 1);
        } else {
            /* Chờ người dùng nhấn BOOT */
            tft_draw_str(4, 33, "Press BOOT", TFT_GRAY, TFT_BG, 1);
            tft_draw_str(4, 45, "to start TX", TFT_GRAY, TFT_BG, 1);
        }

        tft_fill_rect(0, 79, 128, 1, TFT_DIVIDER);

        /* ── Panel RX (y 80..159) ──────────────────────────────── */
        tft_fill_rect(0, 80, 128, 80, TFT_BG);
        tft_draw_str(4, 82, "[ RX (from peer) ]", TFT_PANEL_LABEL, TFT_BG, 1);

        /* Hiển thị peer ID nếu có */
        if (stats.total_received > 0 || stats.session_done) {
            snprintf(buf, sizeof(buf), "ID:%02X%02X%02X%02X",
                     stats.peer_id[0], stats.peer_id[1],
                     stats.peer_id[2], stats.peer_id[3]);
            tft_draw_str(4, 94, buf, TFT_ACCENT_COL, TFT_BG, 1);
        } else {
            tft_draw_str(4, 94, "Waiting for peer...", TFT_GRAY, TFT_BG, 1);
        }

        uint32_t expected = stats.total_expected > 0
                            ? stats.total_expected
                            : (uint32_t)CFG_TEST_TOTAL_PKTS;

        snprintf(buf, sizeof(buf), "Recv:%"PRIu32"/%"PRIu32,
                 stats.total_received, expected);
        tft_draw_str(4, 106, buf, TFT_WHITE, TFT_BG, 1);

        uint16_t err_col = (err_rate < 5.0f)  ? TFT_GREEN :
                           (err_rate < 20.0f) ? TFT_YELLOW : TFT_RED_COL;
        snprintf(buf, sizeof(buf), "Err:%.1f%%", err_rate);
        tft_draw_str(4, 118, buf, err_col, TFT_BG, 2);

        /* RX progress bar */
        uint16_t rx_px = (expected > 0)
                         ? (uint16_t)(120 * stats.total_received / expected) : 0;
        tft_fill_rect(4, 140, 120, 8, TFT_RGB(30, 50, 80));
        tft_fill_rect(4, 140, rx_px, 8, err_col);

        if (stats.session_done) {
            tft_draw_str(4, 150, "DONE!", TFT_GREEN, TFT_BG, 1);
        } else if (stats.total_received > 0) {
            tft_draw_str(4, 150, "RX-ing...", TFT_ACCENT_COL, TFT_BG, 1);
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * app_main
 * ════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, " Test 2.3 — ESP-NOW Range Test");
    ESP_LOGI(TAG, " Mode: SYMMETRIC (both boards have TFT)");
    ESP_LOGI(TAG, " Press BOOT on any board to TX burst");
    ESP_LOGI(TAG, "═══════════════════════════════════════");

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

    /* Tasks */
    xTaskCreatePinnedToCore(task_tx,  "tx",  CFG_STACK_TX,  NULL, CFG_PRIO_TX,  NULL, 0);
    xTaskCreatePinnedToCore(task_rx,  "rx",  CFG_STACK_RX,  NULL, CFG_PRIO_RX,  NULL, 0);
    xTaskCreatePinnedToCore(task_tft, "tft", CFG_STACK_TFT, NULL, CFG_PRIO_TFT, NULL, 1);

    ESP_LOGI(TAG, "3 tasks created — FreeRTOS running");
}

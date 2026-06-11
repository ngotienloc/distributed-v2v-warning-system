/* main.c — Test 2.3: ESP-NOW Range Test (Remote-Trigger mode)
 *
 * ┌─────────────────────┐      ESP-NOW       ┌──────────────────────┐
 * │  Board TFT          │ ─── TRIGGER pkt ──▶ │  Board No-TFT        │
 * │  (commander)        │                     │  (worker)            │
 * │                     │ ◀── 500 data pkts ─ │                      │
 * │  Nhấn BOOT          │                     │  Nhận TRIGGER        │
 * │  → gửi 1 TRIGGER    │                     │  → TX burst 500 gói  │
 * │  → hiển thị TFT     │                     │  → log UART          │
 * └─────────────────────┘                     └──────────────────────┘
 *
 * Phân loại gói qua trường alert_type:
 *   PKT_TYPE_DATA    (0x00) — gói dữ liệu range test (seq 0..499)
 *   PKT_TYPE_TRIGGER (0xA5) — lệnh kích hoạt TX burst từ TFT → No-TFT
 *
 * Biên dịch:
 *   Mặc định        → Board TFT  (commander): gửi TRIGGER, hiển thị RX
 *   #define V2V_NO_TFT → Board no-TFT (worker): chờ TRIGGER, TX burst
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
#include <inttypes.h>

static const char *TAG = "main";

/* ── Phân loại gói (dùng trường alert_type trong v2v_packet_t) ────────── */
#define PKT_TYPE_DATA    0x00   /* gói dữ liệu range test */
#define PKT_TYPE_TRIGGER 0xA5   /* lệnh kích hoạt TX burst */

/* ── Queue RX ────────────────────────────────────────────────────────── */
static QueueHandle_t q_rx;

/* ── Handle task_tx để task_rx gửi notify khi nhận TRIGGER ──────────── */
static TaskHandle_t  h_tx_task = NULL;

/* ── Shared RX stats (task_rx viết, task_tft đọc) ───────────────────── */
typedef struct {
    uint32_t total_expected;
    uint32_t total_received;
    uint32_t last_seq;
    bool     session_done;
} rx_stats_t;

static volatile rx_stats_t s_stats    = {0};
static portMUX_TYPE        s_mux      = portMUX_INITIALIZER_UNLOCKED;

/* ── TX state (task_tx viết, task_tft đọc) ──────────────────────────── */
static volatile uint32_t s_tx_sent   = 0;
static volatile bool     s_tx_active = false;

/* ── TX trigger flag (set bởi task_rx khi nhận TRIGGER pkt) ─────────── */
/* Dùng xTaskNotify thay vì flag để wake task_tx ngay lập tức            */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ── Helper: cấu hình GPIO BOOT button ─────────────────────────────── */
#ifndef V2V_NO_TFT
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
#endif

/* ── Helper: đọc MAC 4 byte cuối ────────────────────────────────────── */
static void get_mac_id(uint8_t id[4])
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    memcpy(id, mac + 2, 4);
}

/* ════════════════════════════════════════════════════════════════════════
 * task_rx — Nhận gói từ queue ESP-NOW
 *
 * Board TFT   : nhận PKT_TYPE_DATA  → cập nhật RX stats
 * Board No-TFT: nhận PKT_TYPE_TRIGGER → notify task_tx để bắt đầu burst
 *               nhận PKT_TYPE_DATA    → (bỏ qua, không cần đếm ở worker)
 * ════════════════════════════════════════════════════════════════════════ */
void task_rx(void *arg)
{
    v2v_packet_t pkt;
    bool first_data = true;

    ESP_LOGI(TAG, "task_rx ready");

    while (1) {
        if (xQueueReceive(q_rx, &pkt, portMAX_DELAY) != pdTRUE) continue;

        /* ── Gói TRIGGER: chỉ board No-TFT xử lý ───────────── */
        if (pkt.alert_type == PKT_TYPE_TRIGGER) {
#ifdef V2V_NO_TFT
            ESP_LOGI(TAG, "[RX] TRIGGER received from [%02X:%02X:%02X:%02X] — starting TX burst",
                     pkt.id[0], pkt.id[1], pkt.id[2], pkt.id[3]);
            /* Reset stats TX */
            espnow_reset_tx_ok();
            s_tx_sent  = 0;
            first_data = true;
            /* Wake task_tx */
            if (h_tx_task)
                xTaskNotify(h_tx_task, 0, eNoAction);
#else
            ESP_LOGI(TAG, "[RX] Ignoring TRIGGER (not worker board)");
#endif
            continue;
        }

        /* ── Gói DATA: chỉ board TFT (commander) đếm ────────── */
#ifndef V2V_NO_TFT
        portENTER_CRITICAL(&s_mux);

        if (first_data || pkt.seq == 0) {
            s_stats.total_expected = 0;
            s_stats.total_received = 0;
            s_stats.session_done   = false;
            first_data = false;
        }

        s_stats.total_received++;
        s_stats.last_seq = pkt.seq;

        if (pkt.seq + 1 > s_stats.total_expected)
            s_stats.total_expected = pkt.seq + 1;

        if (pkt.seq == CFG_TEST_TOTAL_PKTS - 1)
            s_stats.session_done = true;

        portEXIT_CRITICAL(&s_mux);
#endif
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * task_tx
 *
 * Board TFT   : Chờ nút BOOT → gửi 1 gói TRIGGER đến No-TFT board
 * Board No-TFT: Chờ xTaskNotify từ task_rx → TX burst 500 gói data
 * ════════════════════════════════════════════════════════════════════════ */
void task_tx(void *arg)
{
    /* Lưu handle để task_rx có thể notify */
    h_tx_task = xTaskGetCurrentTaskHandle();

    uint8_t my_id[4];
    get_mac_id(my_id);

#ifdef V2V_NO_TFT
    /* ── WORKER: chờ TRIGGER, rồi TX burst ───────────────────────────── */
    ESP_LOGI(TAG, "task_tx [WORKER] — waiting for TRIGGER from TFT board");

    while (1) {
        /* Chờ notify từ task_rx (block vô hạn) */
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

        ESP_LOGI(TAG, "TRIGGER received — TX burst START (%d pkts, %d ms/pkt)",
                 CFG_TEST_TOTAL_PKTS, CFG_TEST_PKT_INTERVAL_MS);

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
    }

#else
    /* ── COMMANDER: nhấn BOOT → gửi TRIGGER ─────────────────────────── */
    btn_init();

    ESP_LOGI(TAG, "task_tx [COMMANDER] — press BOOT to send TRIGGER to worker");

    while (1) {
        /* Chờ nút nhấn */
        if (gpio_get_level(CFG_BTN_GPIO) != 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        /* Debounce */
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(CFG_BTN_GPIO) != 0) continue;

        /* Reset RX stats trước khi gửi trigger */
        portENTER_CRITICAL(&s_mux);
        s_stats.total_expected = 0;
        s_stats.total_received = 0;
        s_stats.session_done   = false;
        portEXIT_CRITICAL(&s_mux);

        /* Gửi gói TRIGGER */
        v2v_packet_t trig = {0};
        trig.magic      = CFG_PKT_MAGIC;
        trig.alert_type = PKT_TYPE_TRIGGER;
        trig.seq        = 0;
        trig.tx_ts_ms   = now_ms();
        memcpy(trig.id, my_id, 4);

        esp_err_t err = espnow_broadcast(&trig);
        if (err == ESP_OK)
            ESP_LOGI(TAG, "TRIGGER sent → waiting for worker's burst...");
        else
            ESP_LOGW(TAG, "TRIGGER send failed (err=%d)", err);

        /* Chờ nhả nút */
        while (gpio_get_level(CFG_BTN_GPIO) == 0)
            vTaskDelay(pdMS_TO_TICKS(20));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
#endif
}

/* ════════════════════════════════════════════════════════════════════════
 * Helper chung: snapshot stats để display task dùng
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

/* ════════════════════════════════════════════════════════════════════════
 * task_tft — chế độ UART (board No-TFT / V2V_NO_TFT defined)
 * Chỉ log TX progress + kết quả sau burst
 * ════════════════════════════════════════════════════════════════════════ */
#ifdef V2V_NO_TFT

void task_tft(void *arg)
{
    ESP_LOGI(TAG, "[WORKER] Waiting for TRIGGER — UART log mode");

    uint32_t prev_tx_ok = UINT32_MAX;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500)); /* 2 Hz */

        rx_stats_t stats;
        uint32_t tx_sent, tx_ok;
        bool     tx_active;
        float    err_rate;
        calc_stats(&stats, &tx_sent, &tx_active, &tx_ok, &err_rate);

        if (tx_active) {
            ESP_LOGI(TAG, "[TX] %" PRIu32 "/%d", tx_sent, CFG_TEST_TOTAL_PKTS);
            continue;
        }

        if (tx_sent > 0 && tx_ok != prev_tx_ok) {
            float loss = 0.0f;
            if (CFG_TEST_TOTAL_PKTS > 0)
                loss = 100.0f * (int)(CFG_TEST_TOTAL_PKTS - tx_ok)
                       / CFG_TEST_TOTAL_PKTS;
            if (loss < 0.0f) loss = 0.0f;
            ESP_LOGI(TAG, "╔══════════════════════════════════╗");
            ESP_LOGI(TAG, "║  TX BURST DONE                   ║");
            ESP_LOGI(TAG, "║  Sent   : %4d pkts              ║", CFG_TEST_TOTAL_PKTS);
            ESP_LOGI(TAG, "║  TX OK  : %4" PRIu32 " (radio ACK)     ║", tx_ok);
            ESP_LOGI(TAG, "║  TX loss: %5.1f%%                ║", loss);
            ESP_LOGI(TAG, "╚══════════════════════════════════╝");
            prev_tx_ok = tx_ok;
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════
 * task_tft — chế độ TFT (board có màn hình / mặc định)
 * Hiển thị: nút BOOT để gửi trigger + RX stats từ worker
 * Layout 128×160:
 *   [0..17]  Header
 *   [18..78] Panel TRIGGER / trạng thái gửi
 *   [79]     Divider
 *   [80..159] Panel RX stats
 * ════════════════════════════════════════════════════════════════════════ */
#else  /* !V2V_NO_TFT */

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
    tft_draw_str(4,  16, "ESP-NOW",      TFT_ACCENT_COL, TFT_BG, 2);
    tft_draw_str(4,  40, "RANGE TEST",   TFT_ACCENT_COL, TFT_BG, 2);
    tft_draw_str(4,  72, "[COMMANDER]",  TFT_ORANGE,     TFT_BG, 1);
    tft_draw_str(4,  90, "Press BOOT",   TFT_GRAY,       TFT_BG, 1);
    tft_draw_str(4, 102, "to trigger",   TFT_GRAY,       TFT_BG, 1);
    tft_draw_str(4, 114, "worker board", TFT_GRAY,       TFT_BG, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));

    TickType_t last_wake  = xTaskGetTickCount();
    char       buf[32];
    uint32_t   trigger_ts = 0;  /* timestamp lần gửi trigger gần nhất */
    uint32_t   prev_rx    = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(200)); /* 5 Hz */

        rx_stats_t stats;
        uint32_t tx_sent, tx_ok;
        bool     tx_active;
        float    err_rate;
        calc_stats(&stats, &tx_sent, &tx_active, &tx_ok, &err_rate);

        /* Phát hiện lần trigger mới (task_tx gửi trigger → tx_sent vẫn 0,
         * nhưng ta dùng một flag riêng qua now_ms thay đổi) */
        bool triggered = (stats.total_expected > 0 || stats.total_received > 0);

        /* ── Header ──────────────────────────────────────────── */
        tft_fill_rect(0, 0, 128, 18, TFT_RGB(15, 25, 50));
        tft_draw_str(4, 4, "ESP-NOW RANGE TEST",
                     TFT_ACCENT_COL, TFT_RGB(15,25,50), 1);
        tft_fill_rect(0, 18, 128, 1, TFT_DIVIDER);

        /* ── Panel Trigger (y 19..78) ────────────────────────── */
        tft_fill_rect(0, 19, 128, 60, TFT_BG);
        tft_draw_str(4, 21, "[ TRIGGER ]", TFT_PANEL_LABEL, TFT_BG, 1);

        if (!triggered) {
            /* Chờ người dùng nhấn BOOT */
            tft_draw_str(4, 33, "Press BOOT btn",  TFT_GRAY,  TFT_BG, 1);
            tft_draw_str(4, 45, "to start worker", TFT_GRAY,  TFT_BG, 1);
            tft_draw_str(4, 57, "TX burst",        TFT_GRAY,  TFT_BG, 1);
        } else if (!stats.session_done) {
            /* Đã trigger — đang nhận */
            tft_draw_str(4, 33, "Trigger sent!",   TFT_GREEN,  TFT_BG, 1);
            tft_draw_str(4, 45, "Worker TX-ing...", TFT_ACCENT_COL, TFT_BG, 1);

            /* Mini progress bar dựa trên số gói đã nhận */
            uint32_t expected = stats.total_expected > 0
                                ? stats.total_expected
                                : (uint32_t)CFG_TEST_TOTAL_PKTS;
            uint16_t fill = (expected > 0)
                            ? (uint16_t)(120 * stats.total_received / expected)
                            : 0;
            tft_fill_rect(4, 57, 120, 8, TFT_RGB(30,50,80));
            tft_fill_rect(4, 57, fill, 8, TFT_GREEN);
        } else {
            /* Xong */
            tft_draw_str(4, 33, "Trigger: DONE",   TFT_GREEN, TFT_BG, 1);
            tft_draw_str(4, 45, "Press BOOT",      TFT_GRAY,  TFT_BG, 1);
            tft_draw_str(4, 57, "for next test",   TFT_GRAY,  TFT_BG, 1);
        }

        tft_fill_rect(0, 79, 128, 1, TFT_DIVIDER);

        /* ── Panel RX stats (y 80..159) ──────────────────────── */
        tft_fill_rect(0, 80, 128, 80, TFT_BG);
        tft_draw_str(4, 82, "[ RX from worker ]", TFT_PANEL_LABEL, TFT_BG, 1);

        uint32_t expected = stats.total_expected > 0
                            ? stats.total_expected
                            : (uint32_t)CFG_TEST_TOTAL_PKTS;

        snprintf(buf, sizeof(buf), "Recv: %" PRIu32 "/%" PRIu32,
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
            snprintf(buf, sizeof(buf), "last_seq:%" PRIu32, stats.last_seq);
            tft_draw_str(4, 140, buf,    TFT_GRAY,  TFT_BG, 1);
            tft_draw_str(4, 150, "DONE!", TFT_GREEN, TFT_BG, 1);
        } else if (stats.total_received > 0) {
            tft_draw_str(4, 140, "Receiving...", TFT_ACCENT_COL, TFT_BG, 1);
        } else {
            tft_draw_str(4, 140, "Waiting...",   TFT_GRAY, TFT_BG, 1);
        }

        prev_rx = stats.total_received;
        (void)prev_rx;
        (void)trigger_ts;
        (void)tx_sent;
        (void)tx_ok;
        (void)tx_active;
    }
}

#endif  /* V2V_NO_TFT */

/* ════════════════════════════════════════════════════════════════════════
 * app_main
 * ════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════");
    ESP_LOGI(TAG, " Test 2.3 — ESP-NOW Range Test");
#ifdef V2V_NO_TFT
    ESP_LOGI(TAG, " Role: WORKER (no-TFT) — awaits TRIGGER");
#else
    ESP_LOGI(TAG, " Role: COMMANDER (TFT) — sends TRIGGER");
#endif
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

    /* Tasks — task_tx tạo trước để h_tx_task được set trước task_rx chạy */
    xTaskCreatePinnedToCore(task_tx, "tx", CFG_STACK_TX, NULL, CFG_PRIO_TX, NULL, 0);
    /* Nhỏ delay để task_tx gọi xTaskGetCurrentTaskHandle() xong */
    vTaskDelay(pdMS_TO_TICKS(10));
    xTaskCreatePinnedToCore(task_rx, "rx", CFG_STACK_RX, NULL, CFG_PRIO_RX, NULL, 0);

#ifdef V2V_NO_TFT
    xTaskCreatePinnedToCore(task_tft, "uart_log", 3584, NULL, CFG_PRIO_TFT, NULL, 1);
#else
    xTaskCreatePinnedToCore(task_tft, "tft",      CFG_STACK_TFT, NULL, CFG_PRIO_TFT, NULL, 1);
#endif

    ESP_LOGI(TAG, "3 tasks created — FreeRTOS running");
}

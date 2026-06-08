/* v2v/espnow_comm.c — ESP-NOW transport cho test range ESP-NOW.
 *
 * on_recv() chạy trong WiFi task context (không phải ISR).
 * on_send() đếm số lần TX thành công → dùng để tính delivery rate TX-side.
 */
#include "freertos/FreeRTOS.h"
#include "espnow_comm.h"
#include "config.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_log.h"
#include <string.h>
#include <stdatomic.h>

static const char   *TAG    = "espnow";
static QueueHandle_t s_rx_q;
static const uint8_t s_bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

/* Bộ đếm TX OK — atomic để an toàn với callback từ WiFi task */
static atomic_uint s_tx_ok = 0;

/* ── Callback nhận ─────────────────────────────────────────────────────── */
static void on_recv(const esp_now_recv_info_t *info,
                    const uint8_t             *data,
                    int                        len)
{
    if (len < (int)sizeof(v2v_packet_t)) {
        ESP_LOGD(TAG, "Short pkt (%d B) dropped", len);
        return;
    }
    if (data[0] != CFG_PKT_MAGIC) {
        ESP_LOGD(TAG, "Bad magic 0x%02X dropped", data[0]);
        return;
    }

    v2v_packet_t pkt;
    memcpy(&pkt, data, sizeof(v2v_packet_t));

    if (xQueueSend(s_rx_q, &pkt, 0) != pdTRUE) {
        ESP_LOGD(TAG, "RX queue full — pkt dropped");
    }
}

/* ── Callback gửi — đếm TX success ──────────────────────────────────────── */
static void on_send(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
    (void)info;
    if (status == ESP_NOW_SEND_SUCCESS) {
        atomic_fetch_add(&s_tx_ok, 1);
    } else {
        ESP_LOGD(TAG, "TX failed (status=%d)", (int)status);
    }
}

/* ── Khởi tạo ────────────────────────────────────────────────────────────── */
esp_err_t espnow_init(QueueHandle_t rx_queue)
{
    s_rx_q = rx_queue;

    ESP_ERROR_CHECK(esp_netif_init());

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CFG_ESPNOW_CHANNEL,
                                          WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_recv));
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_send));

    /* Đăng ký peer broadcast */
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_bcast, 6);
    peer.channel = CFG_ESPNOW_CHANNEL;
    peer.ifidx   = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW init OK (ch=%d, pkt_size=%u B)",
             CFG_ESPNOW_CHANNEL, (unsigned)sizeof(v2v_packet_t));
    return ESP_OK;
}

/* ── Broadcast ────────────────────────────────────────────────────────────── */
esp_err_t espnow_broadcast(const v2v_packet_t *pkt)
{
    return esp_now_send(s_bcast,
                        (const uint8_t *)pkt,
                        sizeof(v2v_packet_t));
}

uint32_t espnow_get_tx_ok(void)
{
    return (uint32_t)atomic_load(&s_tx_ok);
}

void espnow_reset_tx_ok(void)
{
    atomic_store(&s_tx_ok, 0);
}

/* v2v/espnow_comm.c — Triển khai ESP-NOW cho V2V broadcast.
 *
 * on_recv() được gọi từ WiFi task (ISR-safe) → dùng xQueueSendFromISR.
 * on_send() chỉ log lỗi TX nếu DEBUG level.
 * Peer broadcast được đăng ký sẵn với địa chỉ FF:FF:FF:FF:FF:FF. */
#include "freertos/FreeRTOS.h"
#include "espnow_comm.h"
#include "config.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_log.h"
#include <string.h>

static const char   *TAG    = "espnow";
static QueueHandle_t s_rx_q;                           /* queue nhận gói tin V2V */
static const uint8_t s_bcast[6] = CFG_ESPNOW_BCAST;   /* FF:FF:FF:FF:FF:FF */

/* Callback nhận ESP-NOW (chạy trong WiFi task — ISR-safe) */
static void on_recv(const esp_now_recv_info_t *info,
                    const uint8_t             *data,
                    int                        len)
{
    /* Lọc gói quá ngắn hoặc sai magic byte */
    if (len < (int)sizeof(v2v_packet_t)) {
        ESP_LOGD(TAG, "Short packet (%d bytes), dropped", len);
        return;
    }
    if (data[0] != CFG_PKT_MAGIC) {
        ESP_LOGD(TAG, "Bad magic 0x%02X, dropped", data[0]);
        return;
    }

    v2v_packet_t pkt;
    memcpy(&pkt, data, sizeof(v2v_packet_t));

    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_rx_q, &pkt, &woken) != pdTRUE) {
        ESP_LOGD(TAG, "RX queue full - packet dropped");
    }

    #if CONFIG_LOG_DEFAULT_LEVEL >= 4   /* chỉ log RSSI khi DEBUG */
    if (info) {
        ESP_LOGD(TAG, "RX from " MACSTR " RSSI=%d len=%d",
                 MAC2STR(info->src_addr),
                 info->rx_ctrl ? info->rx_ctrl->rssi : 0,
                 len);
    }
    #endif

    if (woken) portYIELD_FROM_ISR();
}

/* Callback gửi ESP-NOW — chỉ log khi TX thất bại */
static void on_send(const wifi_tx_info_t *info, esp_now_send_status_t status)
{
    (void)info;
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "TX failed (status=%d)", (int)status);
    }
}

/* Khởi tạo WiFi STA + ESP-NOW + đăng ký peer broadcast */
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

    /* Đăng ký peer broadcast (FF:FF:FF:FF:FF:FF) */
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_bcast, 6);
    peer.channel = CFG_ESPNOW_CHANNEL;
    peer.ifidx   = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW init OK (ch=%d)", CFG_ESPNOW_CHANNEL);
    return ESP_OK;
}

/* Gửi broadcast gói tin V2V đến tất cả thiết bị trong vùng phủ */
esp_err_t espnow_broadcast(const v2v_packet_t *pkt)
{
    return esp_now_send(s_bcast,
                        (const uint8_t *)pkt,
                        sizeof(v2v_packet_t));
}

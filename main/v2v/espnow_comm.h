/* v2v/espnow_comm.h — API ESP-NOW cho test range. */
#pragma once
#include "types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

/* Khởi tạo WiFi STA + ESP-NOW.
 * rx_queue: queue nhận gói tin (do caller tạo, item = v2v_packet_t). */
esp_err_t espnow_init(QueueHandle_t rx_queue);

/* Gửi broadcast v2v_packet_t đến tất cả thiết bị trong kênh.
 * Trả về ESP_OK nếu esp_now_send() accept (không đảm bảo delivery). */
esp_err_t espnow_broadcast(const v2v_packet_t *pkt);

/* Lấy số lần TX thành công (callback on_send = SUCCESS) kể từ lần reset */
uint32_t  espnow_get_tx_ok(void);

/* Reset bộ đếm TX OK về 0 */
void      espnow_reset_tx_ok(void);

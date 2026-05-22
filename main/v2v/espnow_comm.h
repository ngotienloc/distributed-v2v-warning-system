/* v2v/espnow_comm.h — API truyền thông V2V qua ESP-NOW. */
#pragma once
#include "types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

/* Khởi tạo WiFi STA + ESP-NOW, đăng ký callback RX/TX.
 * rx_queue: queue nhận gói tin (gps_v2v_rx, cấp phát trước bởi app_queues). */
esp_err_t espnow_init(QueueHandle_t rx_queue);

/* Gửi broadcast v2v_packet_t đến tất cả thiết bị trong kênh */
esp_err_t espnow_broadcast(const v2v_packet_t *pkt);

#pragma once
#include "types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

esp_err_t espnow_init(QueueHandle_t rx_queue);

esp_err_t espnow_broadcast(const v2v_packet_t *pkt);


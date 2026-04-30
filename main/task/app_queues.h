#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "types.h"

extern QueueHandle_t q_imu;

extern QueueHandle_t q_gps;

extern EventGroupHandle_t g_ebbl_evt;

extern QueueHandle_t q_fusion_out;

esp_err_t app_queues_init(void);

#define EBBL_BRAKE_BIT   BIT0
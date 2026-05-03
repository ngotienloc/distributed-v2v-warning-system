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

extern QueueHandle_t q_ego_state;

extern QueueHandle_t q_v2v_rx;

extern QueueHandle_t q_tft_collision;

extern QueueHandle_t q_collision_in;

extern QueueHandle_t q_alert_tft;

esp_err_t app_queues_init(void);

#define EBBL_BRAKE_BIT   BIT0
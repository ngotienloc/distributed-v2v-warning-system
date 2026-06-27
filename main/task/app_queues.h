/* task/app_queues.h — Khai báo extern tất cả queue và event group dùng chung.
 *
 * Mỗi queue kết nối đúng một cặp producer → consumer trong pipeline:
 *   q_imu          : task_imu       → task_fusion
 *   q_gps          : task_gps       → task_fusion
 *   q_fusion_out   : task_fusion    → task_localization
 *   q_ego_state    : task_localize  → task_v2v
 *   q_v2v_rx       : ESP-NOW ISR    → task_v2v
 *   q_collision_in : task_v2v       → task_collision
 *   q_tft_collision: task_v2v       → task_display_tft
 *   q_alert_tft    : task_collision → task_display_tft
 *   q_alert_buzzer : task_collision → task_buzzer
 *   g_ebbl_evt     : task_fusion    → task_v2v (EBBL_BRAKE_BIT)
 */
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "types.h"

extern QueueHandle_t q_imu;
extern QueueHandle_t q_gps;
extern QueueHandle_t q_fusion_out;
extern QueueHandle_t q_ego_state;
extern QueueHandle_t q_v2v_rx;
extern QueueHandle_t q_tft_collision;
extern QueueHandle_t q_collision_in;
extern QueueHandle_t q_alert_tft;
extern QueueHandle_t q_alert_buzzer;
extern EventGroupHandle_t g_ebbl_evt;
extern EventGroupHandle_t g_sys_state_evt;

/* Bit báo hiệu phanh gấp — set bởi task_fusion, đọc bởi task_v2v */
#define EBBL_BRAKE_BIT   BIT0

/* Bit báo hiệu GPS đã bắt được sóng (fix valid) */
#define SYS_GPS_READY_BIT BIT0

esp_err_t app_queues_init(void);
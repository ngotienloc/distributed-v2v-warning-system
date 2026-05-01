#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "v2v/espnow_comm.h"
#include "v2v/packet.h"
#include "v2v/neighbor_table.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "task_v2v";

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static vehicle_state_t s_ego       = {0};
static bool            s_ego_valid = false;

static void refresh_ego(void)
{
    vehicle_state_t tmp;
    while (xQueueReceive(q_ego_state, &tmp, 0) == pdTRUE) {
        s_ego       = tmp;
        s_ego_valid = true;
    }
}


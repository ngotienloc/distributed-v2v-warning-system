#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "../v2v/espnow_comm.h"
#include "../v2v/packet.h"
#include "../v2v/neighbor_table.h"
#include "../config.h"
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


static void do_broadcast(alert_type_t type, alert_level_t level)
{

    if (!s_ego_valid) {
        ESP_LOGD(TAG, "skip TX: ego state chưa sẵn sàng");
        return;
    }
    v2v_packet_t pkt;
    packet_serialize(&s_ego, type, level, &pkt);
    espnow_broadcast(&pkt);
}


static void process_rx(void)
{
    v2v_packet_t    pkt;
    vehicle_state_t peer;

    while (xQueueReceive(q_v2v_rx, &pkt, 0) == pdTRUE) {
        if (!packet_is_valid(&pkt)) continue;
        if (packet_deserialize(&pkt, &peer)) {
            ntable_upsert(&peer);
            ESP_LOGD(TAG, "RX from [%02X%02X%02X%02X] spd=%.1fkm/h",
                     peer.id[0], peer.id[1], peer.id[2], peer.id[3],
                     peer.velocity * 3.6f);
        }
    }
}


static void forward_to_collision(void)
{
    if (!s_ego_valid) return;
    collision_input_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.ego     = s_ego;
    ci.n_peers = ntable_get_all(ci.peers, COLLISION_MAX_PEERS);

    collision_input_t stale;
    while (xQueueReceive(q_collision_in, &stale, 0) == pdTRUE) {}
    xQueueSend(q_collision_in, &ci, 0);

    /* Post identical copy to task_display_tft */
    while (xQueueReceive(q_tft_collision, &stale, 0) == pdTRUE) {}
    xQueueSend(q_tft_collision, &ci, 0);

    ESP_LOGD(TAG, "collision snapshot: %d peers forwarded", ci.n_peers);
}


void task_v2v(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "started — ~15 Hz broadcast + q_v2v_rx drain -> q_collision_in");

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CFG_PERIOD_V2V_MS));


        refresh_ego();

        EventBits_t bits = xEventGroupWaitBits(g_ebbl_evt,
                                                EBBL_BRAKE_BIT,
                                                pdTRUE,    /* auto-clear */
                                                pdFALSE,
                                                0);        /* non-blocking */
        if (bits & EBBL_BRAKE_BIT) {
            ESP_LOGW(TAG, "EBBL emergency burst x%d", CFG_EBBL_BURST_COUNT);
            for (int b = 0; b < CFG_EBBL_BURST_COUNT; b++) {
                do_broadcast(ALERT_TYPE_EBBL, ALERT_LEVEL_CRITICAL);
                if (b < CFG_EBBL_BURST_COUNT - 1)
                    vTaskDelay(pdMS_TO_TICKS(CFG_EBBL_BURST_MS));
            }
        } else {
            do_broadcast(ALERT_TYPE_NONE, ALERT_LEVEL_NONE);
        }


        ntable_evict_stale(now_ms());


        process_rx();
                
        forward_to_collision();
    }
}
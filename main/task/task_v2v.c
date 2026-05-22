/* task/task_v2v.c — Quản lý truyền thông V2V qua ESP-NOW.
 *
 * Chạy ~15 Hz (67 ms/cycle):
 *   1. Cập nhật trạng thái ego từ q_ego_state.
 *   2. Kiểm tra EBBL_BRAKE_BIT → burst 3 gói CRITICAL nếu phanh gấp.
 *   3. Broadcast gói bình thường (ALERT_NONE) nếu không có emergency.
 *   4. Xóa peer cũ khỏi bảng lân cận (stale eviction).
 *   5. Drain q_v2v_rx → cập nhật bảng neighbor.
 *   6. Gửi snapshot {ego + peers} → q_collision_in và q_tft_collision. */
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

/* Drain q_ego_state — giữ snapshot mới nhất */
static void refresh_ego(void)
{
    vehicle_state_t tmp;
    while (xQueueReceive(q_ego_state, &tmp, 0) == pdTRUE) {
        s_ego       = tmp;
        s_ego_valid = true;
    }
}

/* Serialize ego state → v2v_packet_t và gửi broadcast ESP-NOW */
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

/* Drain q_v2v_rx → deserialize → upsert vào bảng neighbor */
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

/* Tạo collision_input_t {ego + all peers} và gửi vào hai queue hạ nguồn */
static void forward_to_collision(void)
{
    if (!s_ego_valid) return;
    collision_input_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.ego     = s_ego;
    ci.n_peers = ntable_get_all(ci.peers, COLLISION_MAX_PEERS);

    /* Xả queue cũ trước khi gửi snapshot mới */
    collision_input_t stale;
    while (xQueueReceive(q_collision_in, &stale, 0) == pdTRUE) {}
    xQueueSend(q_collision_in, &ci, 0);

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

        /* Kiểm tra EBBL bit (non-blocking, auto-clear) */
        EventBits_t bits = xEventGroupWaitBits(g_ebbl_evt,
                                                EBBL_BRAKE_BIT,
                                                pdTRUE,   /* auto-clear sau khi đọc */
                                                pdFALSE,
                                                0);       /* không chờ */
        if (bits & EBBL_BRAKE_BIT) {
            /* Phanh gấp: phát burst 3 gói CRITICAL, cách nhau CFG_EBBL_BURST_MS */
            ESP_LOGW(TAG, "EBBL emergency burst x%d", CFG_EBBL_BURST_COUNT);
            for (int b = 0; b < CFG_EBBL_BURST_COUNT; b++) {
                do_broadcast(ALERT_TYPE_EBBL, ALERT_LEVEL_CRITICAL);
                if (b < CFG_EBBL_BURST_COUNT - 1)
                    vTaskDelay(pdMS_TO_TICKS(CFG_EBBL_BURST_MS));
            }
        } else {
            /* Bình thường: broadcast heartbeat không cảnh báo */
            do_broadcast(ALERT_TYPE_NONE, ALERT_LEVEL_NONE);
        }

        ntable_evict_stale(now_ms());  /* xóa peer không còn gửi gói */
        process_rx();
        forward_to_collision();
    }
}

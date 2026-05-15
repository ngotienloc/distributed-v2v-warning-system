#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "collision/ebbl/ebbl.h"
#include "fusion/geo_utils/geo_utils.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "task_collision";

static alert_result_t pick_worse(alert_result_t a, alert_result_t b)
{
    return (b.level > a.level) ? b : a;
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

void task_collision(void *arg)
{
    collision_input_t ci;

    ESP_LOGI(TAG, "started — q_collision_in -> EBBL -> q_alert_tft");

    while (1) {
        /* ── 1. Block on q_collision_in ────────────────────────── *
         * Wait up to CFG_PERIOD_COLLISION_MS (100ms).
         * If task_v2v is operating at ~15 Hz we get a snapshot ~every 67ms. */
        if (xQueueReceive(q_collision_in, &ci, pdMS_TO_TICKS(CFG_PERIOD_COLLISION_MS)) != pdTRUE) {
            continue;  /* timeout — no data, skip cycle */
        }

        /* ── 2. Fast path: no GPS fix or no peers ----------------- *
         * Still report n_peers so the display can show nearby cars. */
        if (!ci.ego.gps_valid || ci.n_peers == 0) {
            alert_result_t none = {0};
            none.n_peers = ci.n_peers;
            xQueueSend(q_alert_tft, &none, 0);
            continue;
        }

        /* ── 3. [Fix #1] esp_timer timestamp hiện tại — ms precision
         *     Dùng để tính age của peer packet.
         *     Cả cur_esp_ms và peer.update_ts_ms đều là esp_timer của EGO
         *     → không cần đồng bộ đồng hồ với xe peer.              */
        uint32_t cur_esp_ms = now_ms();

        /* ── 4. Evaluate each peer ─────────────────────────────── */
        alert_result_t best = {0};

        vehicle_state_t self = ci.ego;
        /* In the ENU frame used here, ego is the origin.
         * peer.x/y are computed via geo_latlon_to_enu(self.lat, self.lon, ...)
         * so they are already relative to (0,0). Force self.x/y = 0 to match. */
        self.x = 0.0f;
        self.y = 0.0f;

        for (int i = 0; i < ci.n_peers; i++) {
            vehicle_state_t peer = ci.peers[i];
            if (!peer.gps_valid) continue;

            /* 4a. Convert peer GPS → ENU relative to ego */
            float px, py;
            geo_latlon_to_enu(self.lat, self.lon,
                               peer.lat, peer.lon,
                               &px, &py);

            /* 4b. [Fix #1] Dead reckoning bù trễ mạng — esp_timer precision
             *     peer.update_ts_ms = esp_timer của EGO lúc nhận packet
             *                         (gán trong packet_deserialize).
             *     age_ms = thời gian packet nằm trong queue + xử lý (~70ms).
             *     Trước: dùng nmea_time_ms (GPS 1Hz) → sai đến ±1000ms = ±16m @60km/h
             *     Sau:   dùng esp_timer (1ms)        → sai ~70ms        = ~1.2m @60km/h */
            uint32_t age_ms = cur_esp_ms - peer.update_ts_ms;
            if (age_ms > 0 && age_ms <= CFG_PKT_STALE_MS) {
                float dt_net = age_ms * 0.001f;
                px += peer.velocity * sinf(peer.heading) * dt_net;
                py += peer.velocity * cosf(peer.heading) * dt_net;
            }

            peer.x = px;
            peer.y = py;

            /* 4c. EBBL evaluation */
            alert_result_t r_ebbl = ebbl_eval(&self, &peer);
            best = pick_worse(best, r_ebbl);

            if (r_ebbl.level) {
                ESP_LOGI(TAG,
                         "Peer [%02X%02X%02X%02X] EBBL_lvl=%d dist=%.0fm",
                         peer.id[0], peer.id[1], peer.id[2], peer.id[3],
                         r_ebbl.level, r_ebbl.dist_m);
            }
        }

        best.n_peers = ci.n_peers;
        xQueueSend(q_alert_tft, &best, 0);
    }
}


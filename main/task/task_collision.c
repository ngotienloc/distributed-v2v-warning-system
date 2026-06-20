/* task/task_collision.c — Đánh giá nguy cơ va chạm mỗi 100 ms.
 *
 * Nhận collision_input_t từ q_collision_in (do task_v2v gửi ~15 Hz).
 * Với mỗi peer (bất kể gps_valid — lat/lon đã được task_localization cập nhật
 * bằng DR 100 Hz nên luôn có giá trị hợp lệ):
 *   1. Chuyển tọa độ lat/lon → ENU (m) lấy ego làm gốc.
 *   2. Bù trễ mạng bằng dead reckoning đơn giản (dùng esp_timer).
 *   3. Gọi ebbl_eval() → alert_result_t.
 * Gửi cảnh báo nặng nhất → q_alert_tft. */
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

/* Trả về cảnh báo có mức độ nghiêm trọng hơn trong hai kết quả */
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
        /* ── 1. Chờ dữ liệu từ task_v2v (timeout 100 ms) ────────────── */
        if (xQueueReceive(q_collision_in, &ci, pdMS_TO_TICKS(CFG_PERIOD_COLLISION_MS)) != pdTRUE) {
            continue;  /* timeout, không có dữ liệu */
        }

        /* ── 2. Fast path: không có peer hoặc chưa có gốc tọa độ ──────── */
        /* Không còn check gps_valid của ego: task_localization đã cập nhật
         * ego.lat/lon từ DR 100 Hz ngay cả khi GPS stale.                   */
        bool ego_has_pos = (ci.ego.lat != 0.0f || ci.ego.lon != 0.0f);
        if (!ego_has_pos || ci.n_peers == 0) {
            alert_result_t none = {0};
            none.n_peers = ci.n_peers;
            xQueueSend(q_alert_tft, &none, 0);
            xQueueSend(q_alert_buzzer, &none, 0);
            continue;
        }

        uint32_t cur_esp_ms = now_ms();

        /* ── 3. Đánh giá từng peer ───────────────────────────────────── */
        alert_result_t best = {0};

        vehicle_state_t self = ci.ego;
        /* Trong hệ ENU lấy ego làm gốc: self.x = self.y = 0 */
        self.x = 0.0f;
        self.y = 0.0f;

        for (int i = 0; i < ci.n_peers; i++) {
            vehicle_state_t peer = ci.peers[i];
            /* Không bỏ qua peer !gps_valid: peer cũng dùng DR 100 Hz nên
             * lat/lon luôn hợp lệ miễn là peer đã từng có ít nhất 1 GPS fix.
             * Chỉ bỏ qua nếu tọa độ peer vẫn là (0,0) — tức chưa từng có fix. */
            if (peer.lat == 0.0f && peer.lon == 0.0f) continue;

            /* 3a. Chuyển lat/lon peer → ENU offset (m) so với ego */
            float px, py;
            geo_latlon_to_enu(self.lat, self.lon,
                               peer.lat, peer.lon,
                               &px, &py);

            /* 3b. Bù trễ mạng: ngoại suy vị trí peer theo thời gian packet nằm trong queue.
             *     update_ts_ms = esp_timer lúc EGO nhận packet → sai số ~70 ms (~1.2 m @60 km/h). */
            uint32_t age_ms = cur_esp_ms - peer.update_ts_ms;
            if (age_ms > 0 && age_ms <= CFG_PKT_STALE_MS) {
                float dt_net = age_ms * 0.001f;
                px += peer.velocity * sinf(peer.heading) * dt_net;
                py += peer.velocity * cosf(peer.heading) * dt_net;
            }

            peer.x = px;
            peer.y = py;

            /* 3c. Gọi EBBL: kiểm tra phanh gấp và TTC */
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
        xQueueSend(q_alert_buzzer, &best, 0);
    }
}

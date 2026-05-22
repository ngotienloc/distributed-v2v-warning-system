/* collision/ebbl/ebbl.c — Đánh giá va chạm EBBL (Emergency Brake + TTC). */
#include "ebbl.h"
#include "config.h"
#include "fusion/math_utils.h"
#include <math.h>
#include <string.h>

/* Tính tốc độ tiếp cận (approach) và khoảng cách (dist) giữa self và peer.
 * Chỉ xét peer nằm trong cone phía trước self (±CFG_EBBL_CONE_DEG).
 * Trả về false nếu peer không trong cone hoặc ngoài tầm. */
static bool compute_approach(const vehicle_state_t *self,
                              const vehicle_state_t *peer,
                              float *out_dist,
                              float *out_approach)
{
    float dx = peer->x - self->x;
    float dy = peer->y - self->y;
    float d  = sqrtf(dx * dx + dy * dy);

    /* Bỏ qua quá gần (nhiễu cảm biến) hoặc quá xa */
    if (d > CFG_EBBL_MAX_DIST_M || d < 0.5f) return false;

    /* Góc từ self đến peer (North=0, CW dương) → so sánh với heading */
    float bearing_abs = atan2f(dx, dy);          /* atan2(East, North) */
    float bearing_rel = angle_diff(bearing_abs, self->heading);

    if (fabsf(bearing_rel) > DEG2RAD(CFG_EBBL_CONE_DEG)) return false;

    /* Chiếu vận tốc tương đối lên trục self→peer */
    float ux = dx / d;
    float uy = dy / d;

    float vsx = self->velocity * sinf(self->heading);
    float vsy = self->velocity * cosf(self->heading);
    float vpx = peer->velocity * sinf(peer->heading);
    float vpy = peer->velocity * cosf(peer->heading);

    /* approach > 0: đang tiếp cận; approach < 0: đang tách ra */
    float approach = dot2d(vsx, vsy, ux, uy) - dot2d(vpx, vpy, ux, uy);

    *out_dist     = d;
    *out_approach = approach;
    return true;
}

/* Ánh xạ TTC (giây) → mức cảnh báo */
static alert_level_t ttc_to_level(float ttc)
{
    if      (ttc < CFG_EBBL_TTC_CRIT_S) return ALERT_LEVEL_CRITICAL;
    else if (ttc < CFG_EBBL_TTC_WARN_S) return ALERT_LEVEL_WARNING;
    else if (ttc < CFG_EBBL_TTC_INFO_S) return ALERT_LEVEL_INFO;
    return ALERT_LEVEL_NONE;
}

alert_result_t ebbl_eval(const vehicle_state_t *self,
                          const vehicle_state_t *peer)
{
    alert_result_t res = {0};

    float d, approach;
    if (!compute_approach(self, peer, &d, &approach)) return res;

    /* ── Path A: EBBL — peer đang phanh gấp ─────────────────────────
     * Chỉ cảnh báo khi peer gửi gia tốc âm mạnh (phanh thật sự)
     * VÀ xe đang tiếp cận (approach > 0). */
    if (peer->accel_x_lin < CFG_EBBL_BRAKE_MS2) {
        if (approach <= 0.0f) return res;  /* đang tách ra — an toàn */

        float ttc = d / approach;
        alert_level_t lvl = ttc_to_level(ttc);
        if (lvl == ALERT_LEVEL_NONE) return res;

        res.type    = ALERT_TYPE_EBBL;
        res.level   = lvl;
        res.ttc_s   = ttc;
        res.dist_m  = d;
        memcpy(res.peer_id, peer->id, 4);
        return res;
    }

    /* ── Path B: TTC proximity — peer không phanh nhưng ego tiếp cận nhanh ──
     * Bao gồm trường hợp peer đứng yên (accel~0) mà ego lao tới.
     * Ngưỡng 1.4 m/s (~5 km/h) tránh cảnh báo giả khi đậu xe. */
    if (approach > 1.4f) {
        float ttc = d / approach;
        alert_level_t lvl = ttc_to_level(ttc);
        if (lvl == ALERT_LEVEL_NONE) return res;

        res.type    = ALERT_TYPE_TTC;
        res.level   = lvl;
        res.ttc_s   = ttc;
        res.dist_m  = d;
        memcpy(res.peer_id, peer->id, 4);
        return res;
    }

    return res;
}

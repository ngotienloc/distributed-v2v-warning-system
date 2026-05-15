#include "ebbl.h"
#include "config.h"
#include "fusion/math_utils.h"
#include <math.h>
#include <string.h>

/* ── Shared geometry helper ───────────────────────────────────────────── */

/* Returns approach speed (positive = closing) and fills dist.
 * Returns false if peer is outside the front cone or moving apart. */
static bool compute_approach(const vehicle_state_t *self,
                              const vehicle_state_t *peer,
                              float *out_dist,
                              float *out_approach)
{
    float dx = peer->x - self->x;
    float dy = peer->y - self->y;
    float d  = sqrtf(dx * dx + dy * dy);

    /* Ignore too close (sensor noise) or too far */
    if (d > CFG_EBBL_MAX_DIST_M || d < 0.5f) return false;

    /* Is peer within the front cone?
     * bearing_abs = absolute bearing from self to peer (North=0, CW)
     * bearing_rel = relative to ego heading -> positive = right side */
    float bearing_abs = atan2f(dx, dy);   /* atan2(East, North) = bearing */
    float bearing_rel = angle_diff(bearing_abs, self->heading);

    if (fabsf(bearing_rel) > DEG2RAD(CFG_EBBL_CONE_DEG)) return false;

    /* Approach speed projection onto self->peer axis */
    float ux = dx / d;
    float uy = dy / d;

    float vsx = self->velocity * sinf(self->heading);
    float vsy = self->velocity * cosf(self->heading);
    float vpx = peer->velocity * sinf(peer->heading);
    float vpy = peer->velocity * cosf(peer->heading);

    float v_self_proj = dot2d(vsx, vsy, ux, uy);
    float v_peer_proj = dot2d(vpx, vpy, ux, uy);
    float approach    = v_self_proj - v_peer_proj;

    *out_dist     = d;
    *out_approach = approach;
    return true;
}

static alert_level_t ttc_to_level(float ttc)
{
    if      (ttc < CFG_EBBL_TTC_CRIT_S) return ALERT_LEVEL_CRITICAL;
    else if (ttc < CFG_EBBL_TTC_WARN_S) return ALERT_LEVEL_WARNING;
    else if (ttc < CFG_EBBL_TTC_INFO_S) return ALERT_LEVEL_INFO;
    return ALERT_LEVEL_NONE;
}

/* ── Main evaluation function ─────────────────────────────────────────── */

alert_result_t ebbl_eval(const vehicle_state_t *self,
                          const vehicle_state_t *peer)
{
    alert_result_t res = {0};

    float d, approach;
    if (!compute_approach(self, peer, &d, &approach)) return res;

    /* ---- Path A: EBBL — peer is actively braking -------------------
     * accel_x_lin is negative when braking (forward = positive axis).
     * Only alert when peer reports strong deceleration. */
    if (peer->accel_x_lin < CFG_EBBL_BRAKE_MS2) {
        if (approach <= 0.0f) return res;  /* moving apart — safe */

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

    /* ---- Path B: [Fix #5] TTC proximity — peer not braking but ego
     * is closing fast. Covers the case where peer is stationary or slow
     * (accel~0) but ego is approaching at high speed.
     * Only fire when approach speed > 5 km/h (1.4 m/s) to avoid noise. */
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

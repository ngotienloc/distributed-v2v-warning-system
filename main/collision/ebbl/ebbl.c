#include "ebbl.h"
#include "config.h"
#include "fusion/math_utils.h"
#include <math.h>
#include <string.h>

alert_result_t ebbl_eval(const vehicle_state_t *self,
                          const vehicle_state_t *peer)
{
    alert_result_t res = {0};

    /* ---- Step 1: Is the peer vehicle actually braking? ----------
     * Only respond when peer reports strong deceleration.
     * accel_x_lin is negative when braking (forward = positive axis).
     * ----------------------------------------------------------- */
    if (peer->accel_x_lin >= CFG_EBBL_BRAKE_MS2) return res;

    /* ---- Step 2: Distance and direction vector (self -> peer) ---
     * x/y are in local ENU frame, self is always at (0,0).
     * ----------------------------------------------------------- */
    float dx = peer->x - self->x;
    float dy = peer->y - self->y;
    float d  = sqrtf(dx * dx + dy * dy);

    /* Ignore if too close (sensor noise) or too far */
    if (d > CFG_EBBL_MAX_DIST_M || d < 0.5f) return res;

    /* ---- Step 3: Is peer within the front cone? ----------------
     * bearing_abs = absolute bearing from self to peer (North=0, CW)
     * bearing_rel = relative to ego heading -> positive = right side
     * ----------------------------------------------------------- */
    float bearing_abs = atan2f(dx, dy);   /* atan2(East, North) = bearing */
    float bearing_rel = angle_diff(bearing_abs, self->heading);

    if (fabsf(bearing_rel) > DEG2RAD(CFG_EBBL_CONE_DEG)) return res;

    /* ---- Step 4: Approach speed projection --------------------
     * Unit vector from self to peer.
     * Project both velocity vectors onto this axis.
     *   v_self_proj  = speed of ego in self->peer direction
     *   v_peer_proj  = speed of peer in self->peer direction
     *   approach     = v_self_proj - v_peer_proj
     *               > 0 means closing in.
     * ----------------------------------------------------------- */
    float ux = dx / d;
    float uy = dy / d;

    /* ENU world-frame velocity vectors */
    float vsx = self->velocity * sinf(self->heading);
    float vsy = self->velocity * cosf(self->heading);
    float vpx = peer->velocity * sinf(peer->heading);
    float vpy = peer->velocity * cosf(peer->heading);

    float v_self_proj = dot2d(vsx, vsy, ux, uy);
    float v_peer_proj = dot2d(vpx, vpy, ux, uy);
    float approach    = v_self_proj - v_peer_proj;

    if (approach <= 0.0f) return res;  /* Moving apart */

    /* ---- Step 5: Time-to-Collision ----------------------------  */
    float ttc = d / approach;

    alert_level_t lvl = ALERT_LEVEL_NONE;
    if      (ttc < CFG_EBBL_TTC_CRIT_S) lvl = ALERT_LEVEL_CRITICAL;
    else if (ttc < CFG_EBBL_TTC_WARN_S) lvl = ALERT_LEVEL_WARNING;
    else if (ttc < CFG_EBBL_TTC_INFO_S) lvl = ALERT_LEVEL_INFO;

    if (lvl == ALERT_LEVEL_NONE) return res;

    /* ---- Step 6: Fill result ----------------------------------- */
    res.type    = ALERT_TYPE_EBBL;
    res.level   = lvl;
    res.ttc_s   = ttc;
    res.dist_m  = d;
    memcpy(res.peer_id, peer->id, 4);
    return res;
}


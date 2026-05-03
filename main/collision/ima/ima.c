#include "ima.h"
#include "config.h"
#include "fusion/math_utils.h"
#include <math.h>
#include <string.h>
#include <stddef.h>


static bool ray_intersect(float ax, float ay, float adx, float ady,
                           float bx, float by, float bdx, float bdy,
                           float *t, float *s)
{
    float denom = adx * bdy - ady * bdx;
    if (fabsf(denom) < 1e-6f) return false;

    float dx = bx - ax;
    float dy = by - ay;
    *t = (dx * bdy - dy * bdx) / denom;
    *s = (dx * ady - dy * adx) / denom;
    return true;
}

/* ----------------------------------------------------------------
 * IMA evaluation
 * ---------------------------------------------------------------- */
alert_result_t ima_eval(const vehicle_state_t *self,
                         const vehicle_state_t *peer)
{
    alert_result_t res = {0};

    /* Minimum speed: GPS heading unreliable at low speed */
    if (self->velocity < 0.5f || peer->velocity < 0.5f) return res;

    /* ---- Step 1: Trajectory crossing angle check -------------- */
    float heading_diff = fabsf(RAD2DEG(angle_diff(self->heading, peer->heading)));

    /* Allow 360->0 wrap: pick the supplementary angle if needed */
    if (heading_diff > 180.0f) heading_diff = 360.0f - heading_diff;

    if (heading_diff < CFG_IMA_ANGLE_MIN_DEG ||
        heading_diff > CFG_IMA_ANGLE_MAX_DEG) {
        return res; /* Parallel or head-on: not an intersection scenario */
    }

    /* ---- Step 2: Find trajectory intersection point ----------- */
    float sha = sinf(self->heading), cha = cosf(self->heading);
    float shb = sinf(peer->heading), chb = cosf(peer->heading);

    float t = 0.0f, s = 0.0f;
    float ix, iy;  /* intersection point in ENU */

    /* Self is always at origin (0,0) in this frame */
    if (ray_intersect(self->x, self->y, sha, cha,
                       peer->x, peer->y, shb, chb,
                       &t, &s))
    {
        /* Intersection exists: t>0 means it's ahead of self */
        if (t < 0.0f || s < 0.0f) return res; /* Behind one vehicle */
        ix = self->x + sha * t;
        iy = self->y + cha * t;
    } else {
        /* Fallback: assume perpendicular paths
         * self on Y axis, peer on X axis -> crossing at peer's position */
        ix = self->x;
        iy = peer->y;
    }

    /* ---- Step 3: Verify both are approaching intersection ----- */
    /* dot(velocity_dir, self->intersection) > 0 means approaching */
    float dx_si = ix - self->x, dy_si = iy - self->y;
    float dx_pi = ix - peer->x, dy_pi = iy - peer->y;

    float self_approach = dot2d(sha, cha, dx_si, dy_si);
    float peer_approach = dot2d(shb, chb, dx_pi, dy_pi);

    if (self_approach <= 0.0f || peer_approach <= 0.0f) return res;

    /* ---- Step 4: Time of arrival at intersection --------------- */
    float d_self = dist2d(self->x, self->y, ix, iy);
    float d_peer = dist2d(peer->x, peer->y, ix, iy);

    /* Beyond IMA radius? Skip. */
    if (d_self > CFG_IMA_RADIUS_M && d_peer > CFG_IMA_RADIUS_M) return res;

    float t_self = d_self / self->velocity;
    float t_peer = d_peer / peer->velocity;
    float delta  = fabsf(t_self - t_peer);

    /* ---- Step 5: Alert level mapping -------------------------- */
    alert_level_t lvl = ALERT_LEVEL_NONE;
    if      (delta < CFG_IMA_DT_CRIT_S) lvl = ALERT_LEVEL_CRITICAL;
    else if (delta < CFG_IMA_DT_WARN_S) lvl = ALERT_LEVEL_WARNING;

    if (lvl == ALERT_LEVEL_NONE) return res;

    res.type      = ALERT_TYPE_IMA;
    res.level     = lvl;
    res.delta_t_s = delta;
    res.dist_m    = d_self;
    memcpy(res.peer_id, peer->id, 4);
    return res;
}

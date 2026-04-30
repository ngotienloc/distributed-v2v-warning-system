#include "dead_reckoning.h"
#include "config.h"
#include "math_utils.h"
#include <string.h>
#include <math.h>

void dr_init(dr_state_t *dr)
{
    memset(dr, 0, sizeof(*dr));
}

void dr_update(dr_state_t *dr, float ax_lin, float ay_lin, float heading, float dt)
{

    // Body convention: +X = forward, +Y = left
    // World convention: +X = East, +Y = North
    /*Rotation:
      Xw = ax * sin(h) + ay * cos(h)   -> East component
      Yw = ax * cos(h) - ay * sin(h)   -> North component*/
    float sh = sinf(heading);
    float ch = cosf(heading);
    float ax_w = ax_lin * sh + ay_lin * ch;
    float ay_w = ax_lin * ch - ay_lin * sh;

    dr->vx += ax_w * dt;
    dr->vy += ay_w * dt;

    if (SQ(dr->vx) + SQ(dr->vy) < SQ(CFG_ZVU_SPEED_MS)) {
        dr->vx = 0.0f;
        dr->vy = 0.0f;
    }

    dr->x += dr->vx * dt + 0.5f * ax_w * dt * dt;
    dr->y += dr->vy * dt + 0.5f * ay_w * dt * dt;

}

void dr_reset_from_gps(dr_state_t *dr, float x_gps,float y_gps, float speed,float heading, float fix_age_s)
{

    float x_now = x_gps;
    float y_now = y_gps;

    if (fix_age_s > 0.0f && fix_age_s < 1.0f) {
        x_now += speed * sinf(heading) * fix_age_s;
        y_now += speed * cosf(heading) * fix_age_s;
    }

    dr->x = x_now;
    dr->y = y_now;

    dr->vx = speed * sinf(heading);
    dr->vy = speed * cosf(heading);
}



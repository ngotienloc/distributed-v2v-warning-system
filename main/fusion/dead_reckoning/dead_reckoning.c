/* fusion/dead_reckoning/dead_reckoning.c — Triển khai Dead Reckoning. */
#include "dead_reckoning.h"
#include "config.h"
#include "math_utils.h"
#include <string.h>
#include <math.h>

void dr_init(dr_state_t *dr)
{
    memset(dr, 0, sizeof(*dr));
}

/* Cập nhật DR một bước:
 *   Body frame: +X = trước mũi xe, +Y = phải.
 *   World frame: +X = East, +Y = North.
 *   Rotation: Xw = ax*sin(h) + ay*cos(h),  Yw = ax*cos(h) - ay*sin(h)
 *
 * Zero Velocity Update (ZVU): nếu |v| < CFG_ZVU_SPEED_MS → cưỡng bức v = 0
 * để loại nhiễu IMU khi xe đứng yên. */
void dr_update(dr_state_t *dr, float ax_lin, float ay_lin, float heading, float dt)
{
    float sh = sinf(heading);
    float ch = cosf(heading);

    /* Xoay gia tốc từ body frame → world frame (ENU) */
    float ax_w = ax_lin * sh + ay_lin * ch;
    float ay_w = ax_lin * ch - ay_lin * sh;

    dr->vx += ax_w * dt;
    dr->vy += ay_w * dt;

    /* ZVU: kẹp vận tốc về 0 nếu quá nhỏ */
    if (SQ(dr->vx) + SQ(dr->vy) < SQ(CFG_ZVU_SPEED_MS)) {
        dr->vx = 0.0f;
        dr->vy = 0.0f;
    }

    /* Tích phân vị trí (công thức chuyển động đều có gia tốc) */
    dr->x += dr->vx * dt + 0.5f * ax_w * dt * dt;
    dr->y += dr->vy * dt + 0.5f * ay_w * dt * dt;
}

/* Reset DR về vị trí GPS, ngoại suy bù trễ fix_age_s:
 *   x_now = x_gps + speed * sin(heading) * fix_age_s  (nếu age < 1s)
 * Dùng speed và heading từ RMC làm vận tốc khởi tạo. */
void dr_reset_from_gps(dr_state_t *dr, float x_gps, float y_gps,
                        float speed, float heading, float fix_age_s)
{
    float x_now = x_gps;
    float y_now = y_gps;

    /* Bù trễ pipeline: ngoại suy vị trí theo age (chỉ khi age < 1s) */
    if (fix_age_s > 0.0f && fix_age_s < 1.0f) {
        x_now += speed * sinf(heading) * fix_age_s;
        y_now += speed * cosf(heading) * fix_age_s;
    }

    dr->x = x_now;
    dr->y = y_now;

    /* Khởi tạo vận tốc từ GPS speed */
    dr->vx = speed * sinf(heading);
    dr->vy = speed * cosf(heading);
}

/* Giảm dần vận tốc theo hàm mũ khi mất GPS.
 * factor = 1 - decay_per_s * dt; vx *= factor, vy *= factor.
 * Ngăn vận tốc tích lũy sai do nhiễu IMU khi xe đứng yên dưới hầm. */
void dr_apply_velocity_decay(dr_state_t *dr, float decay_per_s, float dt)
{
    float factor = 1.0f - decay_per_s * dt;
    if (factor < 0.0f) factor = 0.0f;
    dr->vx *= factor;
    dr->vy *= factor;
}

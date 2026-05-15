#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float x, y;    /* ENU position (m) */
    float vx, vy;  /* ENU velocity (m/s) */
} dr_state_t;

void dr_init(dr_state_t *dr);
void dr_update(dr_state_t *dr, float ax_lin, float ay_lin, float heading, float dt);
void dr_reset_from_gps(dr_state_t *dr, float x_gps, float y_gps, float speed, float heading, float fix_age_s);

/* Áp dụng exponential decay lên velocity khi GPS mất — giảm drift tích lũy.
 * decay_per_s: hệ số giảm mỗi giây, ví dụ 0.5 = giảm 50%/s, 0.95 = giảm nhẹ.
 * Gọi mỗi chu kỳ localization khi !gps_valid. */
void dr_apply_velocity_decay(dr_state_t *dr, float decay_per_s, float dt);

/* fusion/dead_reckoning/dead_reckoning.h — Dead Reckoning (DR).
 *
 * Ước tính vị trí khi mất GPS bằng cách tích phân gia tốc → vận tốc → vị trí.
 * Hệ tọa độ ENU: x = East (m), y = North (m). */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Trạng thái DR: vị trí và vận tốc trong hệ ENU */
typedef struct {
    float x, y;    /* vị trí (m) */
    float vx, vy;  /* vận tốc (m/s) */
} dr_state_t;

/* Khởi tạo trạng thái DR về 0 */
void dr_init(dr_state_t *dr);

/* Cập nhật DR một bước: chuyển accel body → world, tích phân v và x */
void dr_update(dr_state_t *dr, float ax_lin, float ay_lin, float heading, float dt);

/* Reset DR về vị trí GPS, ngoại suy bù trễ fix_age_s giây */
void dr_reset_from_gps(dr_state_t *dr, float x_gps, float y_gps,
                        float speed, float heading, float fix_age_s);

/* Giảm dần vận tốc DR khi mất GPS (tránh drift tích lũy do nhiễu IMU).
 * decay_per_s: ví dụ 0.3 = giảm 30%/s. Gọi mỗi chu kỳ khi !gps_valid. */
void dr_apply_velocity_decay(dr_state_t *dr, float decay_per_s, float dt);

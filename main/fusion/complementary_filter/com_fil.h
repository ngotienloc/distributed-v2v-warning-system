/* fusion/complementary_filter/com_fil.h — Complementary Filter cho IMU.
 *
 * Kết hợp gyro (tốc độ cao, drift) + accelerometer (ổn định, nhiễu)
 * để ước tính pitch/roll (và heading tích hợp gyro_z).
 * Alpha động: tăng lên 1.0 khi accel bị nhiễu mạnh (phanh/rung). */
#pragma once
#include "types.h"
#include <stdbool.h>

/* ── Trạng thái bộ lọc ───────────────────────────────────────────────── */
typedef struct {
    float    pitch;         /* góc pitch (rad) */
    float    roll;          /* góc roll (rad) */
    float    heading;       /* hướng đi (rad, world frame, tích phân gyro_z + fuse GPS) */
    float    accel_x_lin;   /* gia tốc tịnh tiến X sau khi loại trọng lực (m/s²) */
    float    accel_y_lin;
    uint32_t tick;          /* đếm số lần update (dùng cho warmup) */
    uint8_t  brake_count;   /* số mẫu liên tiếp vượt ngưỡng phanh gấp */
} imu_filter_state_t;

/* Khởi tạo trạng thái bộ lọc về 0 */
void imu_filter_init(imu_filter_state_t *s);

/* Cập nhật một bước: pitch/roll từ CF, heading từ gyro_z, loại trọng lực */
void imu_filter_update(imu_filter_state_t *s, const imu_data_t *imu, float dt);

/* Fuse heading GPS vào trạng thái CF (chỉ khi tốc độ > CFG_HDG_MIN_SPEED_MS) */
void imu_filter_fuse_gps_heading(imu_filter_state_t *s, float gps_heading, float gps_speed);

/* Kiểm tra trạng thái phanh gấp (level-triggered, không phải edge):
 * Trả về true khi brake_count ≥ CFG_EBBL_BRAKE_COUNT (accel_x_lin < ngưỡng
 * liên tiếp đủ số mẫu). Hàm trả true trong suốt thời gian đang phanh gấp,
 * không chỉ một lần — cooldown ở tầng task_fusion chống spam event. */
bool imu_filter_is_braking(imu_filter_state_t *s);

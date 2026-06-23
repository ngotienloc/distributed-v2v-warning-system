/* fusion/complementary_filter/com_fil.c — Triển khai Complementary Filter. */
#include "com_fil.h"
#include "config.h"
#include "math_utils.h"
#include <string.h>
#include <math.h>

/* Loại bỏ thành phần trọng lực khỏi accel đo được.
 * Công thức: ax_lin = ax_raw - (-g*sin(pitch)), ay_lin = ay_raw - (g*sin(roll)*cos(pitch)) */
static void remove_gravity(const imu_filter_state_t *s, float ax_raw, float ay_raw,
                            float *ax_lin, float *ay_lin)
{
    float g = 9.81f;
    *ax_lin = ax_raw - (-sinf(s->pitch) * g);
    *ay_lin = ay_raw - ( sinf(s->roll) * cosf(s->pitch) * g);
}

void imu_filter_init(imu_filter_state_t *s)
{
    memset(s, 0, sizeof(*s));
}

/* Tính alpha động: khi gia tốc bị nhiễu (|a| khác 9.81), tin gyro nhiều hơn.
 *   err ≤ SOFT → alpha = CFG_CF_ALPHA_BASE (tin accel bình thường)
 *   err ≥ HARD → alpha = 1.0 (hoàn toàn tin gyro, bỏ qua accel) */
static float compute_cf_alpha(const imu_data_t *imu)
{
    float g   = 9.81f;
    float a   = sqrtf(SQ(imu->accel_x) + SQ(imu->accel_y) + SQ(imu->accel_z));
    float err = fabsf(a - g);

    if (err <= CFG_CF_ACCEL_ERR_SOFT) return CFG_CF_ALPHA_BASE;
    if (err >= CFG_CF_ACCEL_ERR_HARD) return 1.0f;

    /* Nội suy tuyến tính giữa hai ngưỡng */
    float k = (err - CFG_CF_ACCEL_ERR_SOFT) /
              (CFG_CF_ACCEL_ERR_HARD - CFG_CF_ACCEL_ERR_SOFT);
    return CFG_CF_ALPHA_BASE + (1.0f - CFG_CF_ALPHA_BASE) * k;
}

/* Cập nhật một bước CF:
 *   pitch/roll = alpha*(ước_lượng_trước + gyro*dt) + (1-alpha)*accel_angle
 *   heading    = tích phân gyro_z (fuse GPS riêng qua imu_filter_fuse_gps_heading) */
void imu_filter_update(imu_filter_state_t *s, const imu_data_t *imu, float dt)
{
    s->tick++;

    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;

    /* Ước tính góc từ accelerometer */
    float accel_pitch = atan2f(-imu->accel_x, sqrtf(SQ(imu->accel_y) + SQ(imu->accel_z)));
    float accel_roll  = atan2f( imu->accel_y, imu->accel_z);

    float alpha = compute_cf_alpha(imu);
    s->pitch = alpha * (s->pitch + imu->gyro_y * dt) + (1.0f - alpha) * accel_pitch;
    s->roll  = alpha * (s->roll  + imu->gyro_x * dt) + (1.0f - alpha) * accel_roll;

    /* Heading: freeze trong warmup để tránh tích lũy bias nhiệt chưa ổn định.
     * Sau CFG_CF_WARMUP_TICKS (500ms) mới bắt đầu tích phân — GPS sẽ anchor
     * heading về hướng thực ngay khi xe chạy > CFG_HDG_MIN_SPEED_MS. */
    if (s->tick >= CFG_CF_WARMUP_TICKS) {
        s->heading = normalize_angle(s->heading - imu->gyro_z * dt);
    }

    /* Loại trọng lực sau warmup (cần pitch/roll ổn định trước) */
    if (s->tick >= CFG_CF_WARMUP_TICKS) {
        remove_gravity(s, imu->accel_x, imu->accel_y,
                       &s->accel_x_lin, &s->accel_y_lin);
    } else {
        /* Trong giai đoạn warmup: dùng accel thô tạm thời */
        s->accel_x_lin = imu->accel_x;
        s->accel_y_lin = imu->accel_y;
    }
}

/* Fuse heading từ GPS: correction = alpha * angle_diff(gps, imu)
 * Bỏ qua khi tốc độ thấp (GPS heading không tin cậy khi xe đứng yên). */
void imu_filter_fuse_gps_heading(imu_filter_state_t *s,
                                  float               gps_heading,
                                  float               gps_speed)
{
    if (gps_speed < CFG_HDG_MIN_SPEED_MS) return;
    float error = angle_diff(gps_heading, s->heading);
    s->heading  = normalize_angle(s->heading + CFG_HDG_GPS_ALPHA * error);
}

/* Kiểm tra trạng thái phanh gấp: đếm số mẫu liên tiếp accel_x_lin < ngưỡng.
 * Cần ≥ CFG_EBBL_BRAKE_COUNT mẫu liên tiếp để tránh spike nhiễu.
 * Level-triggered: trả true trong suốt khoảng thời gian phanh gấp,
 * không chỉ một lần — cooldown ở task_fusion chống spam EBBL event. */
bool imu_filter_is_braking(imu_filter_state_t *s)
{
    if (s->accel_x_lin < CFG_EBBL_BRAKE_MS2) {
        if (s->brake_count < CFG_EBBL_BRAKE_COUNT)
            s->brake_count++;
    } else {
        s->brake_count = 0;  /* reset nếu không còn dưới ngưỡng */
    }
    return s->brake_count >= CFG_EBBL_BRAKE_COUNT;
}
#include "com_fil.h"
#include "config.h"
#include "math_utils.h"
#include <string.h>
#include <math.h>

static void remove_gravity(const imu_filter_state_t *s, float ax_raw, float ay_raw, float *ax_lin, float *ay_lin) {
    float g = 9.81f;
    *ax_lin = ax_raw - (-sinf(s->pitch) * g);
    *ay_lin = ay_raw - ( sinf(s->roll) * cosf(s->pitch) * g);
}

void imu_filter_init(imu_filter_state_t *s)
{
    memset(s, 0, sizeof(*s));
}

static float compute_cf_alpha(const imu_data_t *imu)
{
    float g = 9.81f;
    float a = sqrtf(SQ(imu->accel_x) + SQ(imu->accel_y) + SQ(imu->accel_z));
    float err = fabsf(a - g);

    if (err <= CFG_CF_ACCEL_ERR_SOFT) {
        return CFG_CF_ALPHA_BASE;
    }
    if (err >= CFG_CF_ACCEL_ERR_HARD) {
        return 1.0f;
    }

    float k = (err - CFG_CF_ACCEL_ERR_SOFT) /
              (CFG_CF_ACCEL_ERR_HARD - CFG_CF_ACCEL_ERR_SOFT);
    return CFG_CF_ALPHA_BASE + (1.0f - CFG_CF_ALPHA_BASE) * k;

}


void imu_filter_update(imu_filter_state_t *s, const imu_data_t   *imu, float dt){
    s->tick++; 

    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;

    float accel_pitch = atan2f(-imu->accel_x, sqrtf(SQ(imu->accel_y) + SQ(imu->accel_z))); // pitch by accel
    float accel_roll  = atan2f( imu->accel_y, imu->accel_z);    // roll by accel

    s->pitch = CFG_CF_ALPHA_BASE * (s->pitch + imu->gyro_y * dt)
             + (1.0f - CFG_CF_ALPHA_BASE) * accel_pitch;
    s->roll  = CFG_CF_ALPHA_BASE * (s->roll  + imu->gyro_x * dt)
             + (1.0f - CFG_CF_ALPHA_BASE) * accel_roll;

    s->heading = normalize_angle(s->heading - imu->gyro_z * dt);    //update heading imu 

    if (s->tick >= CFG_CF_WARMUP_TICKS) {
        remove_gravity(s, imu->accel_x, imu->accel_y,
                       &s->accel_x_lin, &s->accel_y_lin);
    } else {
        s->accel_x_lin = imu->accel_x;
        s->accel_y_lin = imu->accel_y;
    }

}

void imu_filter_fuse_gps_heading(imu_filter_state_t *s,
                                  float               gps_heading,
                                  float               gps_speed)
{
    if (gps_speed < CFG_HDG_MIN_SPEED_MS) return;
    float error = angle_diff(gps_heading, s->heading);
    s->heading  = normalize_angle(s->heading + CFG_HDG_GPS_ALPHA * error);
}


    
/* fusion/math_utils.c — Triển khai tiện ích toán học. */
#include "math_utils.h"

/* Wrap góc về [-π, π] — lặp để xử lý drift lớn */
float normalize_angle(float rad)
{
    while (rad >  (float)M_PI) rad -= 2.0f * (float)M_PI;
    while (rad < -(float)M_PI) rad += 2.0f * (float)M_PI;
    return rad;
}

/* Hiệu hai góc (a - b) được chuẩn hóa về [-π, π] */
float angle_diff(float a, float b)
{
    return normalize_angle(a - b);
}

/* Tích vô hướng 2D: a⃗ · b⃗ */
float dot2d(float ax, float ay, float bx, float by)
{
    return ax * bx + ay * by;
}

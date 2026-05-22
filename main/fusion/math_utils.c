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

/* Khoảng cách Euclid giữa hai điểm 2D */
float dist2d(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1, dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

/* Xoay vector (x, y) bằng ma trận xoay với (sin_h, cos_h) cho trước */
void vec2_rotate(float x, float y, float sin_h, float cos_h,
                 float *out_x, float *out_y)
{
    *out_x =  x * cos_h + y * sin_h;
    *out_y = -x * sin_h + y * cos_h;
}

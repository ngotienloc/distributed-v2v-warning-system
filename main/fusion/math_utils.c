#include "math_utils.h"

float normalize_angle(float rad)
{
    /* Iterative wrap - fast for small drifts, correct for large ones */
    while (rad >  (float)M_PI) rad -= 2.0f * (float)M_PI;
    while (rad < -(float)M_PI) rad += 2.0f * (float)M_PI;
    return rad;
}

float angle_diff(float a, float b)
{
    return normalize_angle(a - b);
}

float dot2d(float ax, float ay, float bx, float by)
{
    return ax * bx + ay * by;
}

float dist2d(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1, dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

void vec2_rotate(float x, float y, float sin_h, float cos_h,
                 float *out_x, float *out_y)
{
    *out_x =  x * cos_h + y * sin_h;
    *out_y = -x * sin_h + y * cos_h;
}

#pragma once
#include <math.h> 

#define DEG2RAD(d)  ((d) * (float)M_PI / 180.0f)
#define RAD2DEG(r)  ((r) * 180.0f / (float)M_PI)
#define SQ(x)       ((x)*(x))

float normalize_angle(float rad);   //wrapped to [-π, π]

float angle_diff(float a, float b);

float dot2d(float ax, float ay, float bx, float by);

float dist2d(float x1, float y1, float x2, float y2);

void vec2_rotate(float x, float y, float sin_h, float cos_h, float *out_x, float *out_y);

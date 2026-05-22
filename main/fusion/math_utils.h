/* fusion/math_utils.h — Tiện ích toán học dùng chung trong toàn dự án. */
#pragma once
#include <math.h>

/* ── Macro chuyển đổi và tiện ích ───────────────────────────────────── */
#define DEG2RAD(d)  ((d) * (float)M_PI / 180.0f)  /* độ → radian */
#define RAD2DEG(r)  ((r) * 180.0f / (float)M_PI)  /* radian → độ */
#define SQ(x)       ((x)*(x))                      /* bình phương */

/* Chuẩn hóa góc về khoảng [-π, π] */
float normalize_angle(float rad);

/* Hiệu hai góc: a - b chuẩn hóa về [-π, π] */
float angle_diff(float a, float b);

/* Tích vô hướng 2D */
float dot2d(float ax, float ay, float bx, float by);

/* Khoảng cách Euclid 2D */
float dist2d(float x1, float y1, float x2, float y2);

/* Xoay vector 2D theo góc (sin_h, cos_h) */
void vec2_rotate(float x, float y, float sin_h, float cos_h, float *out_x, float *out_y);

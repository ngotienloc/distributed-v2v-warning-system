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

/* Tích vô hướng 2D — dùng trong collision/ebbl */
float dot2d(float ax, float ay, float bx, float by);

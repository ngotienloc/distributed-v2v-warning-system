/* fusion/geo_utils/geo_utils.h — Chuyển đổi tọa độ GPS → ENU (m). */
#pragma once
#include <stdint.h>

/* Chuyển tọa độ GPS (lat/lon) của một điểm sang hệ ENU (East-North-Up)
 * lấy (lat0, lon0) làm gốc tọa độ.
 *
 * @param lat0, lon0  Gốc tọa độ (ego vehicle, decimal degrees)
 * @param lat,  lon   Điểm cần chuyển (peer, decimal degrees)
 * @param x           Offset East (m) — output
 * @param y           Offset North (m) — output
 */
void geo_latlon_to_enu(float lat0, float lon0,
                        float lat,  float lon,
                        float *x,   float *y);

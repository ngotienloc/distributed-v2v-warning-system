/* fusion/geo_utils/geo_utils.c — Chuyển đổi tọa độ GPS → ENU.
 *
 * Xấp xỉ phẳng (flat-earth) — hợp lệ trong bán kính < 1 km.
 * x (East)  = (lon - lon0) * cos(lat0) * M_PER_LON_DEG
 * y (North) = (lat - lat0) * M_PER_LAT_DEG */
#include "geo_utils.h"
#include "config.h"
#include "math_utils.h"
#include <math.h>

void geo_latlon_to_enu(float lat0, float lon0,
                        float lat,  float lon,
                        float *x,   float *y)
{
    float cos_lat0 = cosf(DEG2RAD(lat0));
    *x = (lon - lon0) * cos_lat0 * CFG_M_PER_LON_DEG_BASE;  /* East (m) */
    *y = (lat - lat0) * CFG_M_PER_LAT_DEG;                   /* North (m) */
}

/* Nghịch đảo: ENU (m) → GPS (lat/lon) — xấp xỉ phẳng.
 * Giải hệ phương trình:
 *   y = (lat - lat0) * M_PER_LAT_DEG  →  lat = lat0 + y / M_PER_LAT_DEG
 *   x = (lon - lon0) * cos(lat0) * M_PER_LON_DEG_BASE
 *                                      →  lon = lon0 + x / (cos(lat0) * M_PER_LON_DEG_BASE) */
void geo_enu_to_latlon(float lat0, float lon0,
                        float x,    float y,
                        float *lat, float *lon)
{
    float cos_lat0 = cosf(DEG2RAD(lat0));
    *lat = lat0 + y / CFG_M_PER_LAT_DEG;
    /* Bảo vệ chia-cho-0 gần cực (thực tế không bao giờ xảy ra với xe trên mặt đất) */
    if (cos_lat0 > 1e-6f) {
        *lon = lon0 + x / (cos_lat0 * CFG_M_PER_LON_DEG_BASE);
    } else {
        *lon = lon0;
    }
}


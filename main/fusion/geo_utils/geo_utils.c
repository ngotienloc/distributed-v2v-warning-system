#include "geo_utils.h"
#include "config.h"
#include "math_utils.h"
#include <math.h>

void geo_latlon_to_enu(float lat0, float lon0,
                        float lat,  float lon,
                        float *x,   float *y)
{
    float cos_lat0 = cosf(DEG2RAD(lat0));
    *x = (lon - lon0) * cos_lat0 * CFG_M_PER_LON_DEG_BASE;
    *y = (lat - lat0) * CFG_M_PER_LAT_DEG;
}

#pragma once
#include <stdint.h>

/**
 * @param lat0  Self latitude (decimal degrees)
 * @param lon0  Self longitude (decimal degrees)
 * @param lat   Peer latitude
 * @param lon   Peer longitude
 * @param x     East offset (m) - output
 * @param y     North offset (m) - output
 */

void geo_latlon_to_enu(float lat0, float lon0,
                        float lat,  float lon,
                        float *x,   float *y);

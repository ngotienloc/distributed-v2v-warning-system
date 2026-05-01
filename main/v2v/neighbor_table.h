#pragma once 
#include "types.h"
#include <stdint.h> 

void ntable_init(void);

void ntable_upsert(const vehicle_state_t *peer);

void ntable_evict_stale(uint32_t now_ms);

int ntable_get_all(vehicle_state_t *out, int max);

int ntable_count(void);


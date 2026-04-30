#pragma once 
#include "types.h"

typedef struct {
    float x;    // east
    float y;    // north
    float vx; 
    float vy;   
} dr_state_t;

void dr_init(dr_state_t *dr);

void dr_update(dr_state_t *dr,float ax_lin,float ay_lin,float heading, float dt);

void dr_reset_from_gps(dr_state_t *dr,float x_gps,float y_gps, float speed, float heading,float fix_age_s);





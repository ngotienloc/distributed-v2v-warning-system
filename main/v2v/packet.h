#pragma once 
#include "types.h"
#include "esp_err.h"
#include <stdbool.h>

void packet_serialize(const vehicle_state_t *self,
                      alert_type_t           alert_type,
                      alert_level_t          alert_level,
                      v2v_packet_t          *out);          // Transmit 

bool packet_deserialize(const v2v_packet_t *pkt, vehicle_state_t *out); //Recive 

// Check magic byte only.
bool packet_is_valid(const v2v_packet_t *pkt);

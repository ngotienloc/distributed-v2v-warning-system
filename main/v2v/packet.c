#include "packet.h"
#include "esp_timer.h"
#include "config.h"
#include <string.h>

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

bool packet_is_valid(const v2v_packet_t *pkt)
{
    return pkt && pkt->magic == CFG_PKT_MAGIC;
}

void packet_serialize(const vehicle_state_t *self,
                      alert_type_t           alert_type,
                      alert_level_t          alert_level,
                      v2v_packet_t          *out)
{
    out->magic        = CFG_PKT_MAGIC;
    memcpy(out->id, self->id, 4);
    out->lat          = self->lat;
    out->lon          = self->lon;
    out->speed        = self->velocity;   /* wire field 'speed' <- state field 'velocity' */
    out->heading      = self->heading;
    out->accel_x_lin  = self->accel_x_lin;
    out->gyro_z       = self->gyro_z;
    out->alert_type   = (uint8_t)alert_type;
    out->alert_level  = (uint8_t)alert_level;
    out->nmea_time_ms = self->nmea_time_ms;
}

bool packet_deserialize(const v2v_packet_t *pkt, vehicle_state_t *out)
{
    if (!packet_is_valid(pkt)) return false;

    /* NOTE: stale check is handled by neighbor_table via update_ts_ms.
     * Here we only populate vehicle_state_t from the wire packet. */

    memset(out, 0, sizeof(*out));
    memcpy(out->id, pkt->id, 4);
    out->lat          = pkt->lat;
    out->lon          = pkt->lon;
    out->velocity     = pkt->speed;   
    out->heading      = pkt->heading;
    out->accel_x_lin  = pkt->accel_x_lin;
    out->gyro_z       = pkt->gyro_z;
    out->nmea_time_ms = pkt->nmea_time_ms;
    out->gps_valid    = (pkt->lat != 0.0f || pkt->lon != 0.0f);
    out->update_ts_ms = now_ms();
    // x, y = 0; collision_task computes from GPS lat/lon via geo_utils 
    return true;
}


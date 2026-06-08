#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ── Gói tin V2V test range ────────────────────────────────────────────
 * Giữ đúng format gói thật để thực tế đo băng thông.
 * packed: không padding, kích thước cố định. */
typedef struct __attribute__((packed)) {
    uint8_t  magic;         /* CFG_PKT_MAGIC = 0xB6                  */
    uint32_t seq;           /* số thứ tự gói (0 → 499)               */
    uint32_t tx_ts_ms;      /* timestamp phía TX (ms)                 */
    uint8_t  id[4];         /* 4 byte MAC cuối của TX                 */
    /* Padding để giữ kích thước tương đương gói V2V thật (~35 byte) */
    float    lat;
    float    lon;
    float    speed;
    float    heading;
    float    accel_x_lin;
    float    gyro_z;
    uint8_t  alert_type;
    uint8_t  alert_level;
    uint8_t  gps_valid;
} v2v_packet_t;

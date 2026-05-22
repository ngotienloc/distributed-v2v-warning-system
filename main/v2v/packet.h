/* v2v/packet.h — Serialize/deserialize gói tin V2V (vehicle_state_t ↔ v2v_packet_t). */
#pragma once
#include "types.h"
#include "esp_err.h"
#include <stdbool.h>

/* Đóng gói trạng thái xe + thông tin cảnh báo → v2v_packet_t để gửi qua ESP-NOW */
void packet_serialize(const vehicle_state_t *self,
                      alert_type_t           alert_type,
                      alert_level_t          alert_level,
                      v2v_packet_t          *out);

/* Giải gói v2v_packet_t → vehicle_state_t; gán update_ts_ms = now_ms() khi nhận.
 * Trả về false nếu magic byte sai. */
bool packet_deserialize(const v2v_packet_t *pkt, vehicle_state_t *out);

/* Kiểm tra magic byte — nhanh, dùng để lọc gói trước khi deserialize */
bool packet_is_valid(const v2v_packet_t *pkt);

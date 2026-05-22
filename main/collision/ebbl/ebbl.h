/* collision/ebbl/ebbl.h — Public API đánh giá va chạm EBBL. */
#pragma once
#include "types.h"

/* Đánh giá nguy cơ va chạm giữa xe mình (self) và một xe peer.
 *
 * Kiểm tra hai trường hợp:
 *   Path A (EBBL): peer đang phanh gấp (accel_x_lin < ngưỡng) và đang tiếp cận.
 *   Path B (TTC) : ego đang tiếp cận nhanh (approach > 1.4 m/s) bất kể peer phanh.
 *
 * Điều kiện tiên quyết:
 *   - peer phải nằm trong cone phía trước self (±CFG_EBBL_CONE_DEG).
 *   - khoảng cách < CFG_EBBL_MAX_DIST_M.
 *
 * @return alert_result_t với level = NONE nếu an toàn. */
alert_result_t ebbl_eval(const vehicle_state_t *self, const vehicle_state_t *peer);

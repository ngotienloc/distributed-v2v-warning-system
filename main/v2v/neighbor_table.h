/* v2v/neighbor_table.h — Bảng quản lý xe lân cận (peer vehicles). */
#pragma once
#include "types.h"
#include <stdint.h>

/* Khởi tạo bảng và mutex bảo vệ (gọi một lần trong app_main) */
void ntable_init(void);

/* Thêm/cập nhật trạng thái peer theo ID 4 byte.
 * Nếu bảng đầy → đuổi entry cũ nhất (update_ts_ms nhỏ nhất). */
void ntable_upsert(const vehicle_state_t *peer);

/* Xóa các entry không nhận được gói tin trong CFG_PKT_STALE_MS ms */
void ntable_evict_stale(uint32_t now_ms);

/* Sao chép tối đa max entry đang active vào out[], trả về số lượng */
int ntable_get_all(vehicle_state_t *out, int max);

/* Trả về số peer đang active */
int ntable_count(void);

/* drivers/gps/gps.h — Public API của GPS driver. */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Độ dài tối đa một câu NMEA (theo chuẩn NMEA 0183) */
#define NMEA_MAX_LEN 82

/* ── Kết quả một GPS fix ─────────────────────────────────────────────── */
typedef struct {
    float    lat;           /* vĩ độ (°, WGS84) */
    float    lon;           /* kinh độ (°) */
    float    speed_ms;      /* tốc độ (m/s, từ RMC) */
    float    heading_rad;   /* hướng đi (rad, world frame, North=0 CW) */
    bool     valid;         /* true = fix hợp lệ (RMC status = 'A') */
    uint32_t timestamp_ms;  /* thời điểm nhận fix (esp_timer, ms) */
} gps_fix_t;

/* Callback được gọi mỗi khi có GPS fix mới hợp lệ */
typedef void (*gps_fix_cb_t)(const gps_fix_t *fix, void *ctx);

/* Đăng ký callback — phải gọi trước gps_init() hoặc ngay sau */
void gps_register_cb(gps_fix_cb_t cb, void *ctx);

/* Khởi tạo UART, cấu hình module theo GPS_MODEL trong config.h, tạo task đọc/parse.
 *   GPS_MODEL_NEO6M → 5 Hz, 38400 baud
 *   GPS_MODEL_NEO8M → 10 Hz, 115200 baud */
esp_err_t gps_init(void);

/* Trả về true nếu đã nhận được ít nhất một fix hợp lệ */
bool gps_has_fix(void);

/* Thời gian tính từ fix cuối cùng (ms); trả về UINT32_MAX nếu chưa có fix */
uint32_t gps_fix_age_ms(void);

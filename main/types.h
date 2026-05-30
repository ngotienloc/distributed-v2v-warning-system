#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ── Dữ liệu thô từ IMU (MPU6050) ────────────────────────────────────────
 * dt: khoảng thời gian giữa hai lần đọc (giây), dùng cho tích phân. */
typedef struct {
    float accel_x;  /* gia tốc trục X (m/s²) */
    float accel_y;
    float accel_z;
    float gyro_x;   /* vận tốc góc trục X (rad/s) */
    float gyro_y;
    float gyro_z;
    float dt;       /* chu kỳ mẫu (s) */
} imu_data_t;

/* ── Dữ liệu GPS từ NMEA/RMC ─────────────────────────────────────────── */
typedef struct {
    float latitude;
    float longitude;
    float speed;        /* tốc độ (m/s) */
    float course;       /* hướng đi (rad, world frame, North=0 CW) */
    bool  valid;        /* true = fix hợp lệ */
    uint32_t timestamp_ms;  /* thời điểm nhận fix (esp_timer, ms) */
} gps_data_t;

/* ── Góc định hướng xe (rad) ──────────────────────────────────────────── */
typedef struct {
    float roll;
    float pitch;
    float heading;  /* hướng mũi xe so với North (rad, world frame) */
} orientation_t;

/* ── Đầu ra của tầng Fusion (CF + GPS) ───────────────────────────────────
 * Gửi từ task_fusion → task_localization mỗi chu kỳ IMU (~100 Hz). */
typedef struct {
    orientation_t orient;       /* roll / pitch / heading */
    float         accel_x_lin;  /* gia tốc tịnh tiến X sau khi loại trọng lực (m/s²) */
    float         accel_y_lin;
    float         dt;           /* chu kỳ mẫu (s) */
    gps_data_t    gps;          /* fix GPS cuối cùng hợp lệ */
    bool          gps_updated;  /* true = có fix GPS mới trong chu kỳ này */
} fusion_output_t;

/* ── Trạng thái xe (ego hoặc peer) ──────────────────────────────────────
 * Dùng chung cho xe mình (ego) và xe lân cận (peer) trong bảng neighbor. */
typedef struct {
    /* Vị trí & chuyển động */
    float    x;             /* East offset so với gốc DR (m) */
    float    y;             /* North offset (m) */
    float    velocity;      /* tốc độ (m/s) */
    float    heading;       /* hướng (rad, world frame) */

    /* Thông tin bổ sung cho EBBL */
    uint8_t  id[4];         /* 4 byte MAC làm ID xe */
    float    lat;           /* vĩ độ (°) */
    float    lon;           /* kinh độ (°) */
    float    accel_x_lin;   /* gia tốc tịnh tiến X (m/s²) — phát hiện phanh */
    float    gyro_z;        /* vận tốc góc quay (rad/s) */
    bool     gps_valid;     /* true = đang có GPS fix */
    uint32_t local_ts_ms;   /* thời điểm GPS fix gần nhất (ms) */
    uint32_t update_ts_ms;  /* thời điểm cập nhật trạng thái gần nhất (ms) */
} vehicle_state_t;


/* ── Phân loại cảnh báo ──────────────────────────────────────────────── */
typedef enum {
    ALERT_TYPE_NONE = 0,
    ALERT_TYPE_EBBL = 1,    /* xe trước phanh gấp (Emergency Brake) */
    ALERT_TYPE_TTC  = 2,    /* xe đang tiếp cận nhanh (Time-To-Collision) */
} alert_type_t;

/* ── Mức độ cảnh báo ─────────────────────────────────────────────────── */
typedef enum {
    ALERT_LEVEL_NONE     = 0,
    ALERT_LEVEL_INFO     = 1,
    ALERT_LEVEL_WARNING  = 2,
    ALERT_LEVEL_CRITICAL = 3,
} alert_level_t;

/* ── Kết quả đánh giá va chạm ────────────────────────────────────────── */
typedef struct {
    alert_type_t  type;
    alert_level_t level;
    float         ttc_s;      /* Time-To-Collision ước tính (giây) */
    float         dist_m;     /* khoảng cách tới peer (m) */
    uint8_t       peer_id[4]; /* ID xe gây ra cảnh báo */
    int           n_peers;    /* số xe lân cận trong vùng phủ */
} alert_result_t;

/* ── Gói tin V2V truyền qua ESP-NOW ──────────────────────────────────────
 * packed: không padding, kích thước cố định để truyền qua không dây. */
typedef struct __attribute__((packed)) {
    uint8_t  magic;         /* byte nhận dạng gói (CFG_PKT_MAGIC = 0xB6) */
    uint8_t  id[4];
    float    lat;
    float    lon;
    float    speed;         /* m/s */
    float    heading;       /* rad */
    float    accel_x_lin;   /* m/s² — để peer phát hiện phanh gấp */
    float    gyro_z;        /* rad/s */
    uint8_t  alert_type;
    uint8_t  alert_level;
    uint8_t  gps_valid;     /* 1 = GPS fix hợp lệ, 0 = đang dùng DR */
} v2v_packet_t;

/* ── Đầu vào cho task_collision ─────────────────────────────────────────
 * Chứa trạng thái ego + danh sách tối đa 16 xe lân cận. */
#define COLLISION_MAX_PEERS  16
typedef struct {
    vehicle_state_t ego;
    vehicle_state_t peers[COLLISION_MAX_PEERS];
    int             n_peers;
} collision_input_t;

/* ── Kết quả một lần chạy Dead Reckoning test (2.4) ─────────────────────
 * Được tạo trong task_localization khi GPS phục hồi sau khi bị che.
 * Gửi qua q_dr_result → task_display_tft hiển thị lên màn hình TFT. */
typedef struct {
    int   run_num;       /* số thứ tự lần chạy (1, 2, 3...) */
    float dr_dist_m;     /* quãng đường tính từ DR tích phân IMU (m) */
    float gps_dist_m;    /* quãng đường thực tế từ GPS trước/sau che (m) */
    float drift_m;       /* |dr_dist - gps_dist| (m) */
    float drift_pct;     /* drift_m / gps_dist_m * 100 (%) */
    float blackout_s;    /* thời gian mất GPS thực tế (giây) */
    float avg_vel_kmh;   /* vận tốc trung bình lúc mất sóng (km/h) */
} dr_test_result_t;

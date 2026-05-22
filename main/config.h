#pragma once
#include "driver/i2c.h"

/* ── Feature flags ───────────────────────────────────────────────────── */
// #define V2V_HW_STUB      /* Bật để chạy không cần TFT/GPS/IMU thật */

/* Chế độ test: 0 = production, 1 = test mode */
#define ENABLE_TEST_MODE        0

/* In dữ liệu ra UART để vẽ đồ thị (Serial Plotter) */
#define ENABLE_UART_PLOT        1

/* ── GPS (NEO-6M, UART) ──────────────────────────────────────────────── */
#define CFG_GPS_UART_PORT       UART_NUM_1
#define CFG_GPS_UART_TX_PIN     17
#define CFG_GPS_UART_RX_PIN     18
#define CFG_GPS_UART_BAUD       38400   /* baud sau khi cấu hình UBX (5Hz RMC ~350B/s) */
#define CFG_GPS_UART_BAUD_BOOT  9600    /* baud mặc định khi NEO-6M khởi động */
#define CFG_GPS_UART_BUF        512
#define CFG_GPS_SENTENCE_QLEN   8       /* độ sâu queue NMEA sentence */
#define CFG_GPS_RATE_HZ         5       /* tốc độ cập nhật GPS (Hz) */

/* ── IMU (MPU6050, I2C) ──────────────────────────────────────────────── */
#define CFG_IMU_I2C_PORT        I2C_NUM_0
#define CFG_IMU_I2C_SDA         38
#define CFG_IMU_I2C_SCL         39
#define CFG_IMU_I2C_FREQ_HZ     400000
#define CFG_IMU_I2C_ADDR        0x68    /* địa chỉ I2C mặc định MPU6050 */
#define CFG_IMU_CALIB_S         3       /* thời gian hiệu chỉnh (giây), xe phải đứng yên */

/* ── TFT ST7735 (SPI) ────────────────────────────────────────────────── */
#define CFG_TFT_SPI_HOST        SPI2_HOST
#define CFG_TFT_MOSI_PIN        16
#define CFG_TFT_SCLK_PIN        15
#define CFG_TFT_CS_PIN          5
#define CFG_TFT_DC_PIN          2
#define CFG_TFT_RST_PIN         4
#define CFG_TFT_BL_PIN          6
#define CFG_TFT_SPI_SPEED_HZ    27000000
#define CFG_TFT_WIDTH           128     /* pixel */
#define CFG_TFT_HEIGHT          160

/* ── Stack size mỗi task (bytes) ─────────────────────────────────────── */
#define CFG_STACK_IMU           3072
#define CFG_STACK_GPS           3072
#define CFG_STACK_FUSION        3072
#define CFG_STACK_LOCALIZATION  3584
#define CFG_STACK_V2V           4096
#define CFG_STACK_COLLISION     4096
#define CFG_STACK_DISPLAY_TFT   5120

/* ── Độ ưu tiên task (cao hơn = ưu tiên hơn) ────────────────────────── */
#define CFG_PRIO_IMU            6   /* cao nhất — đọc cảm biến real-time */
#define CFG_PRIO_GPS            5
#define CFG_PRIO_FUSION         4
#define CFG_PRIO_LOCALIZATION   4
#define CFG_PRIO_V2V            3
#define CFG_PRIO_COLLISION      3
#define CFG_PRIO_DISPLAY_TFT    2   /* thấp nhất — UI không quan trọng bằng safety */

/* ── CPU core cho mỗi task ──────────────────────────────────────────── */
#define CFG_CORE_IMU            0
#define CFG_CORE_GPS            0
#define CFG_CORE_FUSION         0
#define CFG_CORE_LOCALIZATION   0
#define CFG_CORE_V2V            0
#define CFG_CORE_COLLISION      1   /* tách sang core 1 để không tranh với pipeline cảm biến */
#define CFG_CORE_DISPLAY_TFT    1

/* ── Chu kỳ task (ms) ────────────────────────────────────────────────── */
#define CFG_PERIOD_IMU_MS       10   /* 100 Hz */
#define CFG_PERIOD_FUSION_MS    10
#define CFG_PERIOD_V2V_MS       67   /* ~15 Hz broadcast */
#define CFG_PERIOD_COLLISION_MS 100  /* 10 Hz đánh giá va chạm */
#define CFG_PERIOD_TFT_MS       100  /* 10 Hz refresh màn hình */

/* ── Độ sâu queue ─────────────────────────────────────────────────────── */
#define CFG_QLEN_IMU            4
#define CFG_QLEN_GPS            2
#define CFG_QLEN_FUSION_OUT     2
#define CFG_QLEN_EGO_STATE      2
#define CFG_QLEN_V2V_RX         8
#define CFG_QLEN_COLLISION_IN   2
#define CFG_QLEN_ALERT          4

/* ── V2V (ESP-NOW) ───────────────────────────────────────────────────── */
#define CFG_ESPNOW_CHANNEL      1
#define CFG_ESPNOW_BCAST        {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}  /* broadcast MAC */
#define CFG_PKT_STALE_MS        1000  /* thời gian tối đa packet được coi là hợp lệ (ms) */
#define CFG_PKT_MAGIC           0xB6  /* byte nhận dạng gói tin V2V */

/* ── Ngưỡng mất tín hiệu GPS ─────────────────────────────────────────── */
#define CFG_GPS_STALE_MS        800   /* 5Hz → 200ms/fix; mất 4 fix liên tiếp = stale */

/* ── Hệ số chuyển đổi cảm biến ──────────────────────────────────────── */
#define CFG_ACCEL_SCALE         (1.0f / 4096.0f * 9.81f)   /* ±8g → m/s² */
#define CFG_GYRO_SCALE          (1.0f / 65.5f * 3.14159265f / 180.0f)  /* ±500°/s → rad/s */

/* ── Complementary Filter — ước tính pitch/roll ───────────────────────
 * Alpha gần 1.0 = tin gyro nhiều hơn accelerometer. */
#define CFG_CF_ALPHA_BASE       0.98f   /* alpha mặc định khi gia tốc ổn định */
#define CFG_CF_ACCEL_ERR_SOFT   0.8f    /* ngưỡng bắt đầu giảm alpha (m/s²) */
#define CFG_CF_ACCEL_ERR_HARD   1.5f    /* ngưỡng alpha = 1.0 (tin hoàn toàn gyro) */
#define CFG_CF_WARMUP_TICKS     50      /* số chu kỳ khởi động trước khi loại trọng lực */

/* ── Fuse heading GPS + IMU ──────────────────────────────────────────── */
#define CFG_HDG_GPS_ALPHA       0.10f   /* trọng số GPS khi fuse heading */
#define CFG_HDG_MIN_SPEED_MS    0.5f    /* tốc độ tối thiểu để GPS heading tin cậy */

/* ── Dead Reckoning ──────────────────────────────────────────────────── */
#define CFG_ZVU_SPEED_MS        0.3f    /* ngưỡng Zero Velocity Update (m/s) — lọc nhiễu IMU khi đứng yên */

/* ── Chuyển đổi tọa độ GPS → ENU (m) ────────────────────────────────── */
#define CFG_M_PER_LAT_DEG       110540.0f
#define CFG_M_PER_LON_DEG_BASE  111320.0f

/* ── EBBL — Emergency Brake / Collision Warning ──────────────────────── */
#define CFG_EBBL_BRAKE_MS2      (-2.5f)  /* ngưỡng gia tốc phanh gấp (m/s²) */
#define CFG_EBBL_BRAKE_COUNT    3        /* số mẫu liên tiếp cần vượt ngưỡng để kích hoạt */
#define CFG_EBBL_CONE_DEG       30.0f    /* góc nửa cone phía trước (°) */
#define CFG_EBBL_MAX_DIST_M     150.0f   /* khoảng cách tối đa cảnh báo (m) */
#define CFG_EBBL_TTC_CRIT_S     2.0f     /* TTC < 2s → CRITICAL */
#define CFG_EBBL_TTC_WARN_S     4.0f     /* TTC < 4s → WARNING */
#define CFG_EBBL_TTC_INFO_S     6.0f     /* TTC < 6s → INFO */
#define CFG_EBBL_BURST_COUNT    3        /* số lần phát burst khi phanh gấp */
#define CFG_EBBL_BURST_MS       20       /* khoảng cách giữa các gói burst (ms) */
#define CFG_EBBL_COOLDOWN_MS    500      /* thời gian chờ giữa hai lần kích hoạt EBBL (ms) */

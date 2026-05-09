/**
 * @file   main_ebbl_test.c
 * @brief  EBBL Hardware-in-the-Loop Test
 *
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  CÁCH SỬ DỤNG                                                   ║
 * ║  1. Copy file này vào thư mục main/, đổi tên thành main.c       ║
 * ║     (backup file main.c gốc trước: main_real.c)                 ║
 * ║  2. Chỉnh EBBL_ROLE trước khi flash:                            ║
 * ║       #define EBBL_ROLE 0  → Xe TRƯỚC (front), 50 km/h          ║
 * ║       #define EBBL_ROLE 1  → Xe SAU  (rear),  60 km/h           ║
 * ║  3. Flash lên từng thiết bị, mở Serial Monitor                  ║
 * ║  4. Bấm nút BOOT (GPIO0) trên thiết bị bất kỳ → phanh gấp      ║
 * ║     → thiết bị kia sẽ hiện cảnh báo PHANH GAP! trên màn hình   ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * SCENARIO (thẳng hàng – inline EBBL):
 *   Xe trước (FRONT, role=0): heading=Bắc, v=50 km/h, (lat=BASE_LAT)
 *   Xe sau  (REAR,  role=1): heading=Bắc, v=60 km/h, cách 80m phía sau
 *   → Approach speed ≈ 2.78 m/s, dist=80m → TTC ≈ 28s (INFO)
 *   → Khi bấm BOOT: accel = -5.0 m/s² < CFG_EBBL_BRAKE_MS2 (-2.5)
 *     → xe kia nhận được → EBBL kích hoạt cảnh báo
 *
 * TASK PIPELINE (rút gọn):
 *   task_fake_ego  → [q_ego_state] → task_v2v
 *                         ↓ (ESP-NOW broadcast via task_fake_tx)
 *   [q_v2v_rx] → task_v2v → [q_collision_in] + [q_tft_collision]
 *                         ↓
 *              task_collision → [q_alert_tft] → task_display_tft
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "config.h"
#include "task/app_queues.h"
#include "v2v/espnow_comm.h"
#include "v2v/neighbor_table.h"
#include "v2v/packet.h"
#include "drivers/imu/mpu6050.h"
#include "types.h"
#include <string.h>
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════
 * COMPILE-TIME CONFIGURATION
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * EBBL_ROLE:
 *   0 = Xe TRƯỚC (front vehicle)
 *   1 = Xe SAU   (rear vehicle)
 *
 * Thay đổi giá trị này và flash lại cho từng thiết bị.
 */
#ifndef EBBL_ROLE
#define EBBL_ROLE           0
#endif

/* ═══════════════════════════════════════════════════════════════════
 * CONSTANTS
 * ═══════════════════════════════════════════════════════════════════ */

#define FAKE_EGO_MS         50            /* Cập nhật ego 20 Hz (~MPU sample)    */
#define FAKE_TX_MS          67            /* Broadcast ESP-NOW ~15 Hz             */

/* Tốc độ ban đầu (m/s) — fake, tích phân sẽ điều chỉnh theo gia tốc thực */
#define SPEED_FRONT_MS      (50.0f / 3.6f)   /* ≈ 13.89 m/s */
#define SPEED_REAR_MS       (60.0f / 3.6f)   /* ≈ 16.67 m/s */

/* Khoảng cách ban đầu (m) — xe sau lùi về phía Nam */
#define INIT_DIST_M         40.0f

/* Vị trí gốc */
#define BASE_LAT            10.00000f
#define BASE_LON            106.00000f

/* Lọc gia tốc (EMA) để giảm nhiễu MPU */
#define ACCEL_EMA_ALPHA     0.2f

/* ═══════════════════════════════════════════════════════════════════
 * INTERNAL STATE — shared between fake_ego and fake_tx via mutex
 * ═══════════════════════════════════════════════════════════════════ */

static const char *TAG = "ebbl_test";

static SemaphoreHandle_t s_ego_mutex = NULL;
static vehicle_state_t   s_ego_shared = {0};  /* protected by s_ego_mutex */

/* Forward declarations */
void task_v2v(void *arg);
void task_collision(void *arg);
void task_display_tft(void *arg);

/* ═══════════════════════════════════════════════════════════════════
 * HELPERS
 * ═══════════════════════════════════════════════════════════════════ */

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ═══════════════════════════════════════════════════════════════════
 * TASK: task_fake_ego
 *
 * - Đọc gia tốc THỰC từ MPU6050 mỗi FAKE_EGO_MS
 * - Tích phân accel_x → velocity (dead reckoning đơn giản)
 * - Vị trí (lat/lon/x/y) vẫn fake — bắt đầu từ BASE_LAT/BASE_LON
 * - Ghi vào q_ego_state (task_v2v) và s_ego_shared (task_fake_tx)
 * ═══════════════════════════════════════════════════════════════════ */
static void task_fake_ego(void *arg)
{
    /* ── Khởi tạo trạng thái ego ── */
    vehicle_state_t ego = {0};

    ego.id[0] = 0xEB;
    ego.id[1] = 0xBB;
    ego.id[2] = (EBBL_ROLE == 0) ? 0xFE : 0xBE;
    ego.id[3] = (EBBL_ROLE == 0) ? 0x01 : 0x02;

    ego.heading   = 0.0f;   /* Bắc = 0 rad */
    ego.velocity  = (EBBL_ROLE == 0) ? SPEED_FRONT_MS : SPEED_REAR_MS;
    ego.gps_valid = true;

    float lat_offset = INIT_DIST_M / CFG_M_PER_LAT_DEG;
    ego.lat = BASE_LAT - (EBBL_ROLE == 1 ? lat_offset : 0.0f);
    ego.lon = BASE_LON;
    ego.x   = 0.0f;
    ego.y   = (EBBL_ROLE == 0) ? 0.0f : -INIT_DIST_M;

    /* EMA state cho lọc nhiễu gia tốc */
    float accel_ema = 0.0f;

    uint32_t tick    = 0;
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "[fake_ego] started | role=%s | v=%.2f m/s | MPU gia toc thuc",
             (EBBL_ROLE == 0) ? "FRONT" : "REAR", ego.velocity);

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FAKE_EGO_MS));
        tick++;

        float dt = FAKE_EGO_MS / 1000.0f;

        /* ── Đọc MPU6050 ── */
        imu_data_t imu = {0};
        if (mpu_read(&imu) == ESP_OK) {
            /* accel_x là trục tiến/lùi (forward positive)
             * Lọc EMA để giảm nhiễu */
            accel_ema = ACCEL_EMA_ALPHA * imu.accel_x
                      + (1.0f - ACCEL_EMA_ALPHA) * accel_ema;
        }
        /* Nếu đọc lỗi: giữ nguyên giá trị EMA cũ */

        ego.accel_x_lin = accel_ema;
        ego.gyro_z      = imu.gyro_z;

        /* ── Tích phân gia tốc → tốc độ (không xuống âm) ── */
        ego.velocity += accel_ema * dt;
        if (ego.velocity < 0.0f) ego.velocity = 0.0f;

        /* ── Vị trí CỐ ĐỊNH trong suốt test ───────────────────────────
         * lat/lon KHÔNG tích phân — giữ nguyên giá trị ban đầu.
         * Mục đích: khoảng cách 2 xe luôn = INIT_DIST_M (≈ 80m)
         * → EBBL luôn có đủ điều kiện distance và approach speed.
         *
         * ego.x = 0, ego.y = 0: task_collision dùng geo_latlon_to_enu
         * để tính peer position tương đối — tránh lẫn lộn frame toạ độ. */
        ego.x = 0.0f;
        ego.y = 0.0f;
        /* ego.lat, ego.lon: không đổi (set tại init) */

        /* ── Timestamp ── */
        ego.local_ts_ms  = now_ms();
        ego.nmea_time_ms = ego.local_ts_ms;

        /* ── Ghi vào q_ego_state (drain-then-send, depth=2) ── */
        {
            vehicle_state_t _dummy;
            while (xQueueReceive(q_ego_state, &_dummy, 0) == pdTRUE) {}
            xQueueSend(q_ego_state, &ego, 0);
        }

        /* ── Ghi vào s_ego_shared (task_fake_tx) ── */
        if (xSemaphoreTake(s_ego_mutex, 0) == pdTRUE) {
            s_ego_shared = ego;
            xSemaphoreGive(s_ego_mutex);
        }

        /* ── Log mỗi 1 giây ── */
        if (tick % (1000 / FAKE_EGO_MS) == 0) {
            ESP_LOGI(TAG,
                     "[%s] v=%.2f m/s | accel=%.2f m/s2 | dist=%.0fm | %s",
                     (EBBL_ROLE == 0) ? "FRONT" : "REAR",
                     ego.velocity, accel_ema, INIT_DIST_M,
                     (accel_ema < CFG_EBBL_BRAKE_MS2) ? ">>> BRAKING!" : "normal");
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * TASK: task_fake_tx
 *
 * Đọc ego từ s_ego_shared và broadcast qua ESP-NOW ~15 Hz.
 * Dùng packet_serialize() giống như task_v2v thật.
 * ═══════════════════════════════════════════════════════════════════ */
static void task_fake_tx(void *arg)
{
    ESP_LOGI(TAG, "[fake_tx] started (~15 Hz)");

    TickType_t    last_wake = xTaskGetTickCount();
    v2v_packet_t  pkt       = {0};
    vehicle_state_t snap    = {0};

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FAKE_TX_MS));

        /* Lấy snapshot ego */
        if (xSemaphoreTake(s_ego_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            snap = s_ego_shared;
            xSemaphoreGive(s_ego_mutex);
        } else {
            continue;  /* mutex busy, bỏ cycle này */
        }

        /* Serialize: dùng alert NONE (bản thân xe không cảnh báo) */
        packet_serialize(&snap, ALERT_TYPE_NONE, ALERT_LEVEL_NONE, &pkt);

        /* Broadcast ESP-NOW */
        esp_err_t err = espnow_broadcast(&pkt);
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "[fake_tx] broadcast err: %s", esp_err_to_name(err));
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * app_main — Entry point
 * ═══════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║        EBBL TEST — Role: %-6s              ║",
             (EBBL_ROLE == 0) ? "FRONT" : "REAR");
    ESP_LOGI(TAG, "║  Nhan BOOT (GPIO0) = PHANH GAP              ║");
    ESP_LOGI(TAG, "║  Speed: %s                                  ║",
             (EBBL_ROLE == 0) ? "50 km/h (front)" : "60 km/h (rear) ");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════╝");

    /* ── 1. NVS ── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── 2. Event loop ── */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* ── 3. Queues ── */
    ESP_ERROR_CHECK(app_queues_init());

    /* ── 4. Mutex bảo vệ s_ego_shared ── */
    s_ego_mutex = xSemaphoreCreateMutex();
    configASSERT(s_ego_mutex != NULL);

    /* ── 5. MPU6050 (gia tốc thực) ── */
    ESP_LOGI(TAG, "Initializing MPU6050...");
    ESP_ERROR_CHECK(mpu_init());
    ESP_ERROR_CHECK(mpu_calibrate());   /* ~3s đứng yên để calibrate */

    /* ── 6. ESP-NOW ── */
    ESP_ERROR_CHECK(espnow_init(q_v2v_rx));
    ntable_init();

    /* ── 6. Task pipeline (chỉ các task cần thiết cho EBBL test) ── */
    ESP_LOGI(TAG, "Creating test task pipeline...");

    /*
     * task_fake_ego  — tạo dữ liệu ego fake, đọc nút BOOT
     * Prio cao nhất trong nhóm test để timestamp chính xác
     */
    xTaskCreatePinnedToCore(task_fake_ego,
                             "fake_ego",
                             3072,
                             NULL, 6, NULL, 0);

    /*
     * task_fake_tx  — broadcast ego packet qua ESP-NOW ~15 Hz
     */
    xTaskCreatePinnedToCore(task_fake_tx,
                             "fake_tx",
                             3072,
                             NULL, 5, NULL, 0);

    /*
     * task_v2v  — nhận ESP-NOW → cập nhật neighbor table
     *           → đẩy collision_input_t vào q_collision_in + q_tft_collision
     * (giữ nguyên task gốc, không sửa)
     */
    xTaskCreatePinnedToCore(task_v2v,
                             "v2v",
                             CFG_STACK_V2V,
                             NULL, CFG_PRIO_V2V, NULL,
                             CFG_CORE_V2V);

    /*
     * task_collision  — EBBL + IMA → q_alert_tft
     * (giữ nguyên task gốc, không sửa)
     */
    xTaskCreatePinnedToCore(task_collision,
                             "collision",
                             CFG_STACK_COLLISION,
                             NULL, CFG_PRIO_COLLISION, NULL,
                             CFG_CORE_COLLISION);

    /*
     * task_display_tft  — hiển thị alert (cần màn hình TFT)
     * Nếu không có màn hình, define V2V_HW_STUB để log ra UART
     */
    xTaskCreatePinnedToCore(task_display_tft,
                             "display_tft",
                             CFG_STACK_DISPLAY_TFT,
                             NULL, CFG_PRIO_DISPLAY_TFT, NULL,
                             CFG_CORE_DISPLAY_TFT);

    ESP_LOGI(TAG, "5 tasks created. FreeRTOS scheduler running.");
}

/* drivers/gps/gps.c — GPS driver cho u-blox NEO-6M và NEO-8M.
 *
 * Model được chọn qua macro GPS_MODEL trong config.h:
 *   GPS_MODEL_NEO6M → gps_neo6m_configure(): 5 Hz, 38400 baud
 *   GPS_MODEL_NEO8M → gps_neo8m_configure(): 10 Hz, 115200 baud
 *
 * Kiến trúc 2 task nội bộ:
 *   uart_reader_task : đọc byte từ UART FIFO, ghép thành câu NMEA → s_sentence_q.
 *   parse_task_fn    : parse câu NMEA từ s_sentence_q → gọi callback s_cb.
 *
 * Khởi tạo (gps_init):
 *   A. UART ở 9600 baud (boot default của cả hai module).
 *   B. Gửi lệnh UBX cấu hình phù hợp với GPS_MODEL.
 *   C. Tạo hai task trên. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "gps.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "gps";

static QueueHandle_t s_uart_evt_q;  /* queue sự kiện UART từ driver ESP-IDF */
static QueueHandle_t s_sentence_q;  /* queue câu NMEA đã ghép (NMEA_MAX_LEN+1 bytes) */
static TaskHandle_t  s_reader_task;
static TaskHandle_t  s_parse_task;

static gps_fix_cb_t  s_cb     = NULL;  /* callback do người dùng đăng ký */
static void         *s_cb_ctx = NULL;

static gps_fix_t         s_last_fix = {0};
static volatile bool     s_has_fix  = false;
static volatile uint32_t s_fix_ms   = 0;  /* esp_timer lúc nhận fix cuối */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ── UBX protocol helpers ─────────────────────────────────────────────
 * NEO-6M dùng u-blox Binary Protocol (UBX) để cấu hình.
 * Frame: 0xB5 0x62 | class | id | len_lo len_hi | payload | CK_A CK_B
 * Checksum = Fletcher-8 tính trên (class, id, len_lo, len_hi, payload[]). */
static void ubx_send(uint8_t cls, uint8_t id,
                     const uint8_t *payload, uint16_t pay_len)
{
    uint8_t ck_a = 0, ck_b = 0;
    uint8_t len_lo = (uint8_t)(pay_len & 0xFF);
    uint8_t len_hi = (uint8_t)(pay_len >> 8);

    /* Tính Fletcher-8 checksum */
#define CK(b) do { ck_a += (b); ck_b += ck_a; } while(0)
    CK(cls); CK(id); CK(len_lo); CK(len_hi);
    for (uint16_t i = 0; i < pay_len; i++) CK(payload[i]);
#undef CK

    const uint8_t hdr[6]  = { 0xB5, 0x62, cls, id, len_lo, len_hi };
    const uint8_t tail[2] = { ck_a, ck_b };
    uart_write_bytes(CFG_GPS_UART_PORT, (const char *)hdr,     sizeof(hdr));
    uart_write_bytes(CFG_GPS_UART_PORT, (const char *)payload, pay_len);
    uart_write_bytes(CFG_GPS_UART_PORT, (const char *)tail,    sizeof(tail));
}

/* Tắt một loại câu NMEA trên cổng UART1 (rate = 0) */
static void ubx_disable_nmea(uint8_t nmea_cls, uint8_t nmea_id)
{
    /* CFG-MSG (8 byte): msgClass, msgID, rate[6] cho I2C/UART1/UART2/USB/SPI/reserved */
    const uint8_t payload[8] = { nmea_cls, nmea_id, 0, 0, 0, 0, 0, 0 };
    ubx_send(0x06, 0x01, payload, sizeof(payload));
    vTaskDelay(pdMS_TO_TICKS(30));
}

/* Cấu hình NEO-6M cho chế độ 5 Hz, chỉ xuất RMC:
 *   1. Đổi baud module → 38400 (UBX-CFG-PRT)
 *   2. Tắt GGA, GLL, GSA, GSV, VTG
 *   3. Đặt chu kỳ đo 200 ms = 5 Hz (UBX-CFG-RATE)
 * Gọi khi ESP32 UART vẫn ở 9600 baud (CFG_GPS_UART_BAUD_BOOT). */
#if GPS_MODEL == GPS_MODEL_NEO6M
static void gps_neo6m_configure(void)
{
    /* ── Bước 1: Đổi baud NEO-6M → 115200 (UBX-CFG-PRT, UART1, 20 byte) ── */
    const uint8_t cfg_prt[20] = {
        0x01,                          /* portID = UART1              */
        0x00,                          /* reserved1                   */
        0x00, 0x00,                    /* txReady = disabled          */
        0xC0, 0x08, 0x00, 0x00,       /* mode = 8N1                  */
        0x00, 0xC2, 0x01, 0x00,       /* baudRate = 115200 (little-endian) */
        0x07, 0x00,                    /* inProtoMask = UBX+NMEA+RTCM */
        0x03, 0x00,                    /* outProtoMask = UBX+NMEA     */
        0x00, 0x00,                    /* flags                       */
        0x00, 0x00,                    /* reserved2                   */
    };
    ubx_send(0x06, 0x00, cfg_prt, sizeof(cfg_prt));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_set_baudrate(CFG_GPS_UART_PORT, CFG_GPS_UART_BAUD);
    uart_flush_input(CFG_GPS_UART_PORT);
    ESP_LOGI(TAG, "NEO-6M baud -> %d", CFG_GPS_UART_BAUD);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* ── Bước 2: Tắt các câu không dùng, giữ lại RMC (F0/04) ─────────── */
    ubx_disable_nmea(0xF0, 0x00);   /* GGA */
    ubx_disable_nmea(0xF0, 0x01);   /* GLL */
    ubx_disable_nmea(0xF0, 0x02);   /* GSA */
    ubx_disable_nmea(0xF0, 0x03);   /* GSV */
    ubx_disable_nmea(0xF0, 0x05);   /* VTG */
    ESP_LOGI(TAG, "NEO-6M: GGA/GLL/GSA/GSV/VTG disabled, RMC only");

    /* ── Bước 3: Đặt tốc độ đo 5 Hz (measRate = 200 ms) ──────────────── */
    const uint8_t cfg_rate[6] = {
        0xC8, 0x00,   /* measRate = 200 ms (5 Hz) */
        0x01, 0x00,   /* navRate  = 1             */
        0x01, 0x00,   /* timeRef  = GPS           */
    };
    ubx_send(0x06, 0x08, cfg_rate, sizeof(cfg_rate));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "NEO-6M: rate set to %d Hz (measRate=200ms)", CFG_GPS_RATE_HZ);
}
#endif

/* Cấu hình NEO-8M cho chế độ 10 Hz, chỉ xuất RMC:
 *   1. Đổi baud module → 115200 (UBX-CFG-PRT)
 *   2. Tắt GGA, GLL, GSA, GSV, VTG, GNS
 *   3. Đặt chu kỳ đo 100 ms = 10 Hz (UBX-CFG-RATE)
 * NEO-8M hỗ trợ đa chòm sao (GPS+GLONASS+Galileo) → $GNRMC.
 * Gọi khi ESP32 UART vẫn ở 9600 baud (CFG_GPS_UART_BAUD_BOOT). */
#if GPS_MODEL == GPS_MODEL_NEO8M
static void gps_neo8m_configure(void)
{
    /* ── Bước 1: Đổi baud NEO-8M → 115200 (UBX-CFG-PRT, UART1, 20 byte) ─
     * baudRate = 115200 = 0x0001C200 → little-endian: 0x00, 0xC2, 0x01, 0x00 */
    const uint8_t cfg_prt[20] = {
        0x01,                          /* portID = UART1              */
        0x00,                          /* reserved1                   */
        0x00, 0x00,                    /* txReady = disabled          */
        0xC0, 0x08, 0x00, 0x00,       /* mode = 8N1                  */
        0x00, 0xC2, 0x01, 0x00,       /* baudRate = 115200 (little-endian) */
        0x07, 0x00,                    /* inProtoMask = UBX+NMEA+RTCM */
        0x03, 0x00,                    /* outProtoMask = UBX+NMEA     */
        0x00, 0x00,                    /* flags                       */
        0x00, 0x00,                    /* reserved2                   */
    };
    ubx_send(0x06, 0x00, cfg_prt, sizeof(cfg_prt));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_set_baudrate(CFG_GPS_UART_PORT, CFG_GPS_UART_BAUD);
    uart_flush_input(CFG_GPS_UART_PORT);
    ESP_LOGI(TAG, "NEO-8M baud -> %d", CFG_GPS_UART_BAUD);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* ── Bước 2: Tắt các câu không dùng, giữ lại RMC ─────────────────── */
    ubx_disable_nmea(0xF0, 0x00);   /* GGA */
    ubx_disable_nmea(0xF0, 0x01);   /* GLL */
    ubx_disable_nmea(0xF0, 0x02);   /* GSA */
    ubx_disable_nmea(0xF0, 0x03);   /* GSV */
    ubx_disable_nmea(0xF0, 0x05);   /* VTG */
    ubx_disable_nmea(0xF0, 0x0D);   /* GNS — câu thêm của NEO-8M multi-GNSS */
    ESP_LOGI(TAG, "NEO-8M: GGA/GLL/GSA/GSV/VTG/GNS disabled, RMC only");

    /* ── Bước 3: Đặt tốc độ đo 10 Hz (measRate = 100 ms) ─────────────── */
    const uint8_t cfg_rate[6] = {
        0x64, 0x00,   /* measRate = 100 ms (10 Hz) */
        0x01, 0x00,   /* navRate  = 1              */
        0x01, 0x00,   /* timeRef  = GPS            */
    };
    ubx_send(0x06, 0x08, cfg_rate, sizeof(cfg_rate));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "NEO-8M: rate set to %d Hz (measRate=100ms)", CFG_GPS_RATE_HZ);
}
#endif

/* Chuyển chuỗi NMEA dạng DDDMM.MMMM + chỉ hướng → decimal degrees */
static float parse_coord(const char *field, char dir)
{
    if (!field || field[0] == '\0') return 0.0f;
    float raw = strtof(field, NULL);
    int   deg = (int)(raw / 100);
    float min = raw - deg * 100.0f;
    float result = deg + min / 60.0f;
    if (dir == 'S' || dir == 'W') result = -result;
    return result;
}


/* Kiểm tra checksum XOR của câu NMEA (byte giữa '$' và '*') */
static bool checksum_ok(const char *sent)
{
    if (!sent || sent[0] != '$') return false;
    const char *star = strrchr(sent, '*');
    if (!star) return false;
    uint8_t calc = 0;
    for (const char *p = sent + 1; p < star; p++) calc ^= (uint8_t)*p;
    uint8_t got = (uint8_t)strtol(star + 1, NULL, 16);
    return calc == got;
}

/* Tách câu NMEA bằng dấu phẩy → mảng con trỏ fields[] (giữ lại các trường trống) */
static int tokenize(char *buf, char *fields[], int max)
{
    int n = 0;
    char *p = buf;
    while (p && n < max) {
        fields[n++] = p;
        p = strchr(p, ',');
        if (p) {
            *p = '\0';
            p++;
        }
    }
    return n;
}

/* Parse và xử lý một câu NMEA hoàn chỉnh:
 *   RMC → cập nhật đầy đủ lat/lon/speed/heading → gọi callback.
 *   GGA → chỉ cập nhật lat/lon nội bộ (không gọi callback vì thiếu speed/heading). */
static void process_sentence(const char *sentence)
{
    if (!checksum_ok(sentence)) return;
    char buf[NMEA_MAX_LEN + 1];
    strlcpy(buf, sentence, sizeof(buf));

    char *f[20] = {0};
    int   nf    = tokenize(buf, f, 20);
    if (nf < 6) return;

    bool is_rmc = (strncmp(f[0], "$GPRMC", 6) == 0 ||
                   strncmp(f[0], "$GNRMC", 6) == 0);
    bool is_gga = (strncmp(f[0], "$GPGGA", 6) == 0 ||
                   strncmp(f[0], "$GNGGA", 6) == 0);

    if (is_rmc && nf >= 9) {
        if (f[2][0] != 'A') return;  /* chỉ xử lý khi status = 'A' (Active/Fix) */
        s_last_fix.lat         = parse_coord(f[3], f[4][0]);
        s_last_fix.lon         = parse_coord(f[5], f[6][0]);
        s_last_fix.speed_ms    = strtof(f[7], NULL) * 0.51444f;  /* knots → m/s */
        s_last_fix.heading_rad = strtof(f[8], NULL) * (float)M_PI / 180.0f;

        uint32_t t              = now_ms();
        s_last_fix.valid        = true;
        s_last_fix.timestamp_ms = t;
        s_has_fix               = true;
        s_fix_ms                = t;

        if (s_cb) s_cb(&s_last_fix, s_cb_ctx);

        ESP_LOGD(TAG, "RMC fix lat=%.6f lon=%.6f spd=%.2f hdg=%.1f°",
                 s_last_fix.lat, s_last_fix.lon,
                 s_last_fix.speed_ms,
                 s_last_fix.heading_rad * 180.0f / (float)M_PI);

    } else if (is_gga && nf >= 7) {
        /* GGA không có speed/heading → chỉ lưu nội bộ, KHÔNG gọi callback
         * để tránh gửi speed/heading cũ (bị trễ 1 giây) xuống pipeline. */
        if (atoi(f[6]) == 0) return;  /* fix quality = 0 = invalid */
        s_last_fix.lat = parse_coord(f[2], f[3][0]);
        s_last_fix.lon = parse_coord(f[4], f[5][0]);
        ESP_LOGV(TAG, "GGA lat=%.6f lon=%.6f (no callback)",
                 s_last_fix.lat, s_last_fix.lon);
    }
}

/* Task đọc byte từ UART FIFO và ghép thành câu NMEA hoàn chỉnh */
static void uart_reader_task(void *arg)
{
    uart_event_t event;
    uint8_t      rx[CFG_GPS_UART_BUF];
    char         sentence[NMEA_MAX_LEN + 1];
    int          slen = 0;

    while (1) {
        if (!xQueueReceive(s_uart_evt_q, &event, portMAX_DELAY)) continue;

        if (event.type == UART_DATA) {
            int n = uart_read_bytes(CFG_GPS_UART_PORT, rx,
                                    event.size, pdMS_TO_TICKS(20));
            for (int i = 0; i < n; i++) {
                char c = (char)rx[i];
                if (c == '$') slen = 0;         /* bắt đầu câu mới */
                if (slen < NMEA_MAX_LEN) sentence[slen++] = c;
                if (c == '\n' && slen > 6) {
                    sentence[slen] = '\0';
                    /* Chỉ forward RMC và GGA */
                    if (strncmp(sentence, "$GPRMC", 6) == 0 ||
                        strncmp(sentence, "$GNRMC", 6) == 0 ||
                        strncmp(sentence, "$GPGGA", 6) == 0 ||
                        strncmp(sentence, "$GNGGA", 6) == 0) {
                        char copy[NMEA_MAX_LEN + 1];
                        memcpy(copy, sentence, (size_t)slen + 1);
                        xQueueSend(s_sentence_q, copy, 0);  /* drop nếu queue đầy */
                    }
                    slen = 0;
                }
            }
        } else if (event.type == UART_FIFO_OVF ||
                   event.type == UART_BUFFER_FULL) {
            /* Tràn buffer: xả sạch để tránh dữ liệu rác */
            ESP_LOGW(TAG, "UART overflow - flush");
            uart_flush_input(CFG_GPS_UART_PORT);
            xQueueReset(s_uart_evt_q);
            slen = 0;
        }
    }
}

/* Task parse: nhận câu NMEA từ queue → gọi process_sentence() */
static void parse_task_fn(void *arg)
{
    char sentence[NMEA_MAX_LEN + 1];
    ESP_LOGI(TAG, "parse task started");
    while (1) {
        if (xQueueReceive(s_sentence_q, sentence, portMAX_DELAY) == pdTRUE)
            process_sentence(sentence);
    }
}

void gps_register_cb(gps_fix_cb_t cb, void *ctx)
{
    s_cb     = cb;
    s_cb_ctx = ctx;
}

bool gps_has_fix(void)    { return s_has_fix; }

/* Trả về số ms từ lần fix cuối; UINT32_MAX nếu chưa có fix */
uint32_t gps_fix_age_ms(void)
{
    if (!s_has_fix || s_fix_ms == 0) return UINT32_MAX;
    return now_ms() - s_fix_ms;
}

esp_err_t gps_init(void)
{
    s_sentence_q = xQueueCreate(CFG_GPS_SENTENCE_QLEN, NMEA_MAX_LEN + 1);
    if (!s_sentence_q) return ESP_ERR_NO_MEM;

    /* ── Bước A: Khởi tạo UART ở 9600 baud (default NEO-6M) ─────────── */
    uart_config_t ucfg = {
        .baud_rate  = CFG_GPS_UART_BAUD_BOOT,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(CFG_GPS_UART_PORT, &ucfg));
    ESP_ERROR_CHECK(uart_set_pin(CFG_GPS_UART_PORT,
                                  CFG_GPS_UART_TX_PIN, CFG_GPS_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(CFG_GPS_UART_PORT,
                                         CFG_GPS_UART_BUF * 2,
                                         0, 16,
                                         &s_uart_evt_q, 0));

    /* ── Bước B: Gửi lệnh UBX cấu hình theo model đã chọn ─────────────── */
    vTaskDelay(pdMS_TO_TICKS(200));   /* chờ module sẵn sàng */
#if GPS_MODEL == GPS_MODEL_NEO6M
    gps_neo6m_configure();            /* baud→38400, RMC only, 5Hz  */
#elif GPS_MODEL == GPS_MODEL_NEO8M
    gps_neo8m_configure();            /* baud→115200, RMC only, 10Hz */
#endif

    /* ── Bước C: Tạo task đọc và parse ─────────────────────────────── */
    xTaskCreatePinnedToCore(uart_reader_task, "gps_rd",
                            3072, NULL, CFG_PRIO_GPS + 1,
                            &s_reader_task, CFG_CORE_GPS);
    xTaskCreatePinnedToCore(parse_task_fn, "gps_parse",
                            3072, NULL, CFG_PRIO_GPS,
                            &s_parse_task, CFG_CORE_GPS);

    ESP_LOGI(TAG, "GPS init OK (model=%s UART%d baud=%d RX=%d rate=%dHz)",
#if GPS_MODEL == GPS_MODEL_NEO6M
             "NEO-6M",
#else
             "NEO-8M",
#endif
             CFG_GPS_UART_PORT, CFG_GPS_UART_BAUD,
             CFG_GPS_UART_RX_PIN, CFG_GPS_RATE_HZ);
    return ESP_OK;
}

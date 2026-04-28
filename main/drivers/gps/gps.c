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

static QueueHandle_t s_uart_evt_q;   // UART driver EVENT queue 
static QueueHandle_t s_sentence_q;   // Raw NMEA SENTENCE queue 
static TaskHandle_t  s_reader_task;  
static TaskHandle_t  s_parse_task;

static gps_fix_cb_t  s_cb     = NULL;
static void         *s_cb_ctx = NULL;

static gps_fix_t         s_last_fix  = {0};
static volatile bool     s_has_fix   = false;
static volatile uint32_t s_fix_ms    = 0;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

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

static uint32_t parse_utc_ms(const char *s)
{
    if (!s || strlen(s) < 6) return 0;
    int h = (s[0]-'0')*10 + (s[1]-'0');
    int m = (s[2]-'0')*10 + (s[3]-'0');
    int sec = (s[4]-'0')*10 + (s[5]-'0');
    return (uint32_t)((h * 3600 + m * 60 + sec) * 1000);
}

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

//Split
static int tokenize(char *buf, char *fields[], int max)
{
    int n = 0;
    char *tok = strtok(buf, ",");
    while (tok && n < max) { fields[n++] = tok; tok = strtok(NULL, ","); }
    return n;
}

static void process_sentence(const char *sentence){
    if(!checksum_ok(sentence))  return; 
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
        // Field[2] = 'A' (active) required 
        if (f[2][0] != 'A') return;
        s_last_fix.lat         = parse_coord(f[3], f[4][0]);
        s_last_fix.lon         = parse_coord(f[5], f[6][0]);
        s_last_fix.speed_ms    = strtof(f[7], NULL) * 0.51444f; /* knots->m/s */
        s_last_fix.heading_rad = strtof(f[8], NULL) * (float)M_PI / 180.0f;
        s_last_fix.nmea_time_ms = parse_utc_ms(f[1]);
    } else if (is_gga && nf >= 7) {
        //Field[6] = fix quality, 0 = no fix 
        if (atoi(f[6]) == 0) return;
        s_last_fix.lat          = parse_coord(f[2], f[3][0]);
        s_last_fix.lon          = parse_coord(f[4], f[5][0]);
        s_last_fix.nmea_time_ms = parse_utc_ms(f[1]);
        // GGA does not carry speed/heading - keep previous values 
    } else {
        return;
    }

    uint32_t t          = now_ms();
    s_last_fix.valid        = true;
    s_last_fix.timestamp_ms = t;
    s_has_fix               = true;
    s_fix_ms                = t;

    if (s_cb) s_cb(&s_last_fix, s_cb_ctx);

    ESP_LOGD(TAG, "FIX lat=%.6f lon=%.6f spd=%.2f hdg=%.1f°",
             s_last_fix.lat, s_last_fix.lon,
             s_last_fix.speed_ms,
             s_last_fix.heading_rad * 180.0f / (float)M_PI);
}

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
                if (c == '$') slen = 0;         
                if (slen < NMEA_MAX_LEN) sentence[slen++] = c;
                if (c == '\n' && slen > 6) {
                    sentence[slen] = '\0';
                    /* Only forward sentences we care about */
                    if (strncmp(sentence, "$GPRMC", 6) == 0 ||
                        strncmp(sentence, "$GNRMC", 6) == 0 ||
                        strncmp(sentence, "$GPGGA", 6) == 0 ||
                        strncmp(sentence, "$GNGGA", 6) == 0) {
                        char copy[NMEA_MAX_LEN + 1];
                        memcpy(copy, sentence, (size_t)slen + 1);
                        xQueueSend(s_sentence_q, copy, 0); /* drop if full */
                    }
                    slen = 0;
                }
            }
        } else if (event.type == UART_FIFO_OVF ||
                   event.type == UART_BUFFER_FULL) {
            ESP_LOGW(TAG, "UART overflow - flush");
            uart_flush_input(CFG_GPS_UART_PORT);
            xQueueReset(s_uart_evt_q);
            slen = 0;
        }
    }
}

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

uint32_t gps_fix_age_ms(void)
{
    if (!s_has_fix || s_fix_ms == 0) return UINT32_MAX;
    return now_ms() - s_fix_ms;
}

esp_err_t gps_init(void)
{
    s_sentence_q = xQueueCreate(CFG_GPS_SENTENCE_QLEN, NMEA_MAX_LEN + 1);
    if (!s_sentence_q) return ESP_ERR_NO_MEM;

    /* UART config */
    const uart_config_t ucfg = {
        .baud_rate  = CFG_GPS_UART_BAUD,
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

    xTaskCreatePinnedToCore(uart_reader_task, "gps_rd",
                            3072, NULL, CFG_PRIO_GPS + 1,
                            &s_reader_task, CFG_CORE_GPS);
    xTaskCreatePinnedToCore(parse_task_fn, "gps_parse",
                            3072, NULL, CFG_PRIO_GPS,
                            &s_parse_task, CFG_CORE_GPS);

    ESP_LOGI(TAG, "GPS init OK (UART%d baud=%d RX=%d)",
             CFG_GPS_UART_PORT, CFG_GPS_UART_BAUD, CFG_GPS_UART_RX_PIN);
    return ESP_OK;
}

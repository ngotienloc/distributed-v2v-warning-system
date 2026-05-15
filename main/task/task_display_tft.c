#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app_queues.h"
#include "drivers/tft/tft_driver.h"
#include "../config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

static const char *TAG = "task_tft";

/* ── Radar geometry constants ──────────────────────────────── */
#define RADAR_CX      64          /* center X (screen pixels) */
#define RADAR_CY      64          /* center Y */
#define RADAR_R       56          /* radar radius (px)         */
#define RADAR_SCALE_M 150.0f      /* 150m ↔ RADAR_R px         */

/* Convert ENU meters → screen pixels (North = up = -Y screen) */
#define ENUx_TO_SCR(px) ((int)(RADAR_CX + (px) * RADAR_R / RADAR_SCALE_M))
#define ENUy_TO_SCR(py) ((int)(RADAR_CY - (py) * RADAR_R / RADAR_SCALE_M))

/* Screen area bounds */
#define STATUSBAR_H   8
#define FOOTER_Y      120
#define FOOTER_H      (CFG_TFT_HEIGHT - FOOTER_Y)  /* 40px */

/* ── Internal state ─────────────────────────────────────────── */
static collision_input_t s_ci   = {0};
static alert_result_t    s_alert = {0};
static bool              s_alert_active = false;
static uint32_t          s_alert_until  = 0;

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ── Queue drain helpers ─────────────────────────────────────── */
static void refresh_state(void)
{
    /* Drain q_tft_collision — dedicated copy for TFT, no race with collision task */
    collision_input_t tmp_ci;
    while (xQueueReceive(q_tft_collision, &tmp_ci, 0) == pdTRUE)
        s_ci = tmp_ci;

    /* Drain alerts — take latest; activate display on real alert */
    alert_result_t tmp_al;
    while (xQueueReceive(q_alert_tft, &tmp_al, 0) == pdTRUE) {
        s_alert = tmp_al;
        if (tmp_al.level > ALERT_LEVEL_NONE) {
            s_alert_active = true;
            s_alert_until  = now_ms() + 2000;
        }
    }

    if (s_alert_active && now_ms() > s_alert_until)
        s_alert_active = false;
}

/* ── Rendering helpers ───────────────────────────────────────── */

/* Peer dot color based on alert level */
static uint16_t peer_color(int peer_idx)
{
    /* If this peer is the one that triggered the active alert */
    if (s_alert_active &&
        memcmp(s_ci.peers[peer_idx].id, s_alert.peer_id, 4) == 0) {
        return (s_alert.level == ALERT_LEVEL_CRITICAL) ? TFT_DOT_CRIT :
               (s_alert.level == ALERT_LEVEL_WARNING)  ? TFT_DOT_WARN :
                                                         TFT_DOT_INFO;
    }
    return TFT_DOT_SAFE;
}

/* Draw ego vehicle: blue filled circle + heading arrow */
static void draw_ego(float heading)
{
    tft_fill_circle(RADAR_CX, RADAR_CY, 5, TFT_DOT_SELF);

    /* Heading arrow: 12px line from center toward heading direction
     * heading = 0 → North = up screen → −Y. CW positive. */
    int ax = RADAR_CX + (int)(12.0f * sinf(heading));
    int ay = RADAR_CY - (int)(12.0f * cosf(heading));
    tft_draw_line(RADAR_CX, RADAR_CY, ax, ay, TFT_WHITE);
}

/* Draw one peer vehicle */
static void draw_peer(const vehicle_state_t *peer, uint16_t color)
{
    int sx = ENUx_TO_SCR(peer->x);
    int sy = ENUy_TO_SCR(peer->y);

    /* Clip to radar area */
    if (sx < 0 || sx >= CFG_TFT_WIDTH) return;
    if (sy < STATUSBAR_H || sy >= FOOTER_Y) return;

    /* Draw peer as small filled circle */
    tft_fill_circle(sx, sy, 4, color);

    /* Heading tick */
    int tx = sx + (int)(8.0f * sinf(peer->heading));
    int ty = sy - (int)(8.0f * cosf(peer->heading));
    tft_draw_line(sx, sy, tx, ty, TFT_WHITE);
}

/* Draw radar grid: lightweight cross-hair (avoid heavy per-pixel rings) */
static void draw_radar_grid(void)
{
    /* Cross-hair */
    tft_draw_line(RADAR_CX, STATUSBAR_H,
                  RADAR_CX, FOOTER_Y - 1, TFT_RGB(25, 40, 55));
    tft_draw_line(0, RADAR_CY, CFG_TFT_WIDTH - 1,
                  RADAR_CY, TFT_RGB(25, 40, 55));

    /* "N" label at top */
    tft_draw_char(RADAR_CX - 2, STATUSBAR_H + 1, 'N',
                  TFT_GRAY, TFT_BG_IDLE, 1);
}

/* Draw status bar (top 8px row) */
static void draw_statusbar(const vehicle_state_t *ego, int n_peers)
{
    tft_fill_rect(0, 0, CFG_TFT_WIDTH, STATUSBAR_H, TFT_RGB(15, 25, 45));

    /* GPS indicator */
    uint16_t gps_col = ego->gps_valid ? TFT_RGB(0, 200, 80) :
                                        TFT_RGB(200, 50, 50);
    tft_fill_rect(1, 1, 5, 6, gps_col);

    /* Speed */
    char buf[24];
    int spd_kmh = (int)(ego->velocity * 3.6f);
    snprintf(buf, sizeof(buf), "%3dkm/h", spd_kmh);
    tft_draw_str(8, 1, buf, TFT_CYAN, TFT_RGB(15, 25, 45), 1);

    /* Peer count */
    snprintf(buf, sizeof(buf), "N:%d", n_peers);
    tft_draw_str(CFG_TFT_WIDTH - 24, 1, buf,
                 TFT_DOT_SAFE, TFT_RGB(15, 25, 45), 1);
}

/* Draw footer (bottom 40px) — alert or idle */
static void draw_footer(void)
{
    if (s_alert_active) {
        uint16_t bg = (s_alert.level == ALERT_LEVEL_CRITICAL) ? TFT_BG_CRIT :
                      (s_alert.level == ALERT_LEVEL_WARNING)  ? TFT_BG_WARN :
                                                                TFT_BG_INFO;
        tft_fill_rect(0, FOOTER_Y, CFG_TFT_WIDTH, FOOTER_H, bg);

        /* Alert type name */
        const char *name = (s_alert.type == ALERT_TYPE_EBBL) ? "PHANH GAP!" :
                           (s_alert.type == ALERT_TYPE_TTC)  ? "VA CHAM!"   :
                                                               "CAUTION";
        tft_draw_str(4, FOOTER_Y + 2, name, TFT_WHITE, bg, 2);

        /* Metric */
        char mbuf[20];
        snprintf(mbuf, sizeof(mbuf), "TTC %.1fs %.0fm",
                 s_alert.ttc_s, s_alert.dist_m);
        tft_draw_str(4, FOOTER_Y + 20, mbuf, TFT_WHITE, bg, 1);

    } else {
        /* Idle footer: heading */
        tft_fill_rect(0, FOOTER_Y, CFG_TFT_WIDTH, FOOTER_H, TFT_BG_IDLE);
        char buf[20];
        float hdg = s_ci.ego.heading * 57.2957795f;
        if (hdg < 0) hdg += 360.0f;
        snprintf(buf, sizeof(buf), "HDG %.0f deg", hdg);
        tft_draw_str(4, FOOTER_Y + 4,  buf, TFT_ACCENT, TFT_BG_IDLE, 1);
        tft_draw_str(4, FOOTER_Y + 16, "V2V READY", TFT_DOT_SAFE, TFT_BG_IDLE, 1);
    }
}

/* ── Full frame render ───────────────────────────────────────── */
static void render_frame(void)
{
    /* 1. Clear radar area only (avoid full clear → faster) */
    tft_fill_rect(0, STATUSBAR_H,
                  CFG_TFT_WIDTH, FOOTER_Y - STATUSBAR_H,
                  TFT_BG_IDLE);

    /* 2. Radar grid */
    draw_radar_grid();
    vTaskDelay(1); /* yield to IDLE to avoid WDT */

    /* 3. Peer vehicles */
    for (int i = 0; i < s_ci.n_peers && i < COLLISION_MAX_PEERS; i++) {
        if (!s_ci.peers[i].gps_valid) continue;
        draw_peer(&s_ci.peers[i], peer_color(i));
        if ((i & 3) == 3) vTaskDelay(1); /* periodic yield */
    }

    /* 4. Ego vehicle */
    draw_ego(s_ci.ego.heading);
    vTaskDelay(1);

    /* 5. Status bar */
    draw_statusbar(&s_ci.ego, s_ci.n_peers);

    /* 6. Footer */
    draw_footer();
    vTaskDelay(1);
}

/* ── Boot splash ─────────────────────────────────────────────── */
static void render_boot(void)
{
    tft_clear(TFT_BG_IDLE);
    tft_draw_str( 8, 30,  "V2V",     TFT_ACCENT, TFT_BG_IDLE, 3);
    tft_draw_str( 8, 60,  "WARNING", TFT_ACCENT, TFT_BG_IDLE, 2);
    tft_draw_str( 8, 90,  "INIT...", TFT_GRAY,   TFT_BG_IDLE, 1);
    vTaskDelay(pdMS_TO_TICKS(1500));
}

/* ── Task entry point ────────────────────────────────────────── */
void task_display_tft(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

#ifndef V2V_HW_STUB
    if (tft_init() != ESP_OK) {
        ESP_LOGE(TAG, "TFT init failed — task exits");
        vTaskDelete(NULL);
        return;
    }
    render_boot();
#endif

    ESP_LOGI(TAG, "started — q_tft_collision+q_alert_tft -> TFT (10 Hz)");

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CFG_PERIOD_TFT_MS));

#ifndef V2V_HW_STUB
        refresh_state();
        render_frame();
#else
        /* Stub: log to console */
        refresh_state();
        ESP_LOGD(TAG, "[TFT] ego v=%.1f hdg=%.0f | peers=%d | alert=%d",
                 s_ci.ego.velocity,
                 s_ci.ego.heading * 57.3f,
                 s_ci.n_peers,
                 s_alert.level);
#endif
    }
}

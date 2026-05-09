#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "neighbor_table.h"
#include "config.h"
#include "esp_log.h"
#include <string.h>
#include <stdint.h>

static const char *TAG = "ntable";

#define MAX_PEERS  16

typedef struct {
    vehicle_state_t state;
    bool            active;
} entry_t;

static entry_t          s_table[MAX_PEERS];
static SemaphoreHandle_t s_mutex;

static bool id_eq(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 4) == 0;
}

void ntable_init(void)
{
    memset(s_table, 0, sizeof(s_table));
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);
}

//Init
void ntable_upsert(const vehicle_state_t *peer)
{
    if (!peer) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* Search for existing entry */
    for (int i = 0; i < MAX_PEERS; i++) {
        if (s_table[i].active && id_eq(s_table[i].state.id, peer->id)) {
            s_table[i].state = *peer;
            xSemaphoreGive(s_mutex);
            return;
        }
    }

    /* Insert into first free slot */
    for (int i = 0; i < MAX_PEERS; i++) {
        if (!s_table[i].active) {
            s_table[i].state  = *peer;
            s_table[i].active = true;
            ESP_LOGD(TAG, "New peer [%02X%02X%02X%02X] added (slot %d)", peer->id[0], peer->id[1], peer->id[2], peer->id[3], i);
            xSemaphoreGive(s_mutex);
            return;
        }
    }

    /* Table full: evict oldest */
    uint32_t oldest_ts = UINT32_MAX;
    int      oldest_i  = 0;
    for (int i = 0; i < MAX_PEERS; i++) {
        if (s_table[i].state.update_ts_ms < oldest_ts) {
            oldest_ts = s_table[i].state.update_ts_ms;
            oldest_i  = i;
        }
    }
    ESP_LOGW(TAG, "Table full - evicting slot %d", oldest_i);
    s_table[oldest_i].state  = *peer;
    s_table[oldest_i].active = true;

    xSemaphoreGive(s_mutex);
}

void ntable_evict_stale(uint32_t now_ms)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_PEERS; i++) {
        if (!s_table[i].active) continue;
        uint32_t age = now_ms - s_table[i].state.update_ts_ms;
        if (age > CFG_PKT_STALE_MS) {
            ESP_LOGD(TAG, "Evicting stale peer [%02X%02X%02X%02X] age=%lums",
                     s_table[i].state.id[0], s_table[i].state.id[1],
                     s_table[i].state.id[2], s_table[i].state.id[3],
                     (unsigned long)age);
            s_table[i].active = false;
        }
    }
    xSemaphoreGive(s_mutex);
}

int ntable_get_all(vehicle_state_t *out, int max)
{
    int count = 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_PEERS && count < max; i++) {
        if (s_table[i].active)
            out[count++] = s_table[i].state;
    }
    xSemaphoreGive(s_mutex);
    return count;
}

int ntable_count(void)
{
    int count = 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_PEERS; i++)
        if (s_table[i].active) count++;
    xSemaphoreGive(s_mutex);
    return count;
}






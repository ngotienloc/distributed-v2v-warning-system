#include "task/app_queues.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "app_queues";

QueueHandle_t      q_imu          = NULL;
QueueHandle_t      q_gps          = NULL;
QueueHandle_t      q_fusion_out   = NULL;
QueueHandle_t      q_ego_state    = NULL;
QueueHandle_t      q_v2v_rx       = NULL;


esp_err_t app_queues_init(void)
{
    q_imu           = xQueueCreate(4, sizeof(imu_data_t));
    q_gps           = xQueueCreate(2, sizeof(gps_data_t));

    if (!q_imu || !q_gps) {
        ESP_LOGE(TAG, "Queue/EventGroup creation failed! Check heap.");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "All queues created:");
    ESP_LOGI(TAG, "  q_imu          (imu_data_t,      depth=4)");
    ESP_LOGI(TAG, "  q_gps          (gps_data_t,      depth=2)");
    return ESP_OK;
}
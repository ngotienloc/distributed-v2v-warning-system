#include "task/app_queues.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "app_queues";

QueueHandle_t      q_imu          = NULL;
QueueHandle_t      q_gps          = NULL;
QueueHandle_t      q_fusion_out   = NULL;
QueueHandle_t      q_ego_state    = NULL;
QueueHandle_t      q_v2v_rx       = NULL;
QueueHandle_t      q_collision_in = NULL;
QueueHandle_t      q_tft_collision = NULL;
QueueHandle_t      q_alert_tft    = NULL;
EventGroupHandle_t g_ebbl_evt     = NULL;


esp_err_t app_queues_init(void)
{
    q_imu           = xQueueCreate(4, sizeof(imu_data_t));
    q_gps           = xQueueCreate(2, sizeof(gps_data_t));
    q_fusion_out    = xQueueCreate(2, sizeof(fusion_output_t));
    q_ego_state     = xQueueCreate(2, sizeof(vehicle_state_t));
    q_v2v_rx        = xQueueCreate(8, sizeof(v2v_packet_t));
    q_collision_in  = xQueueCreate(2, sizeof(collision_input_t));
    q_tft_collision = xQueueCreate(2, sizeof(collision_input_t));
    q_alert_tft     = xQueueCreate(CFG_QLEN_ALERT, sizeof(alert_result_t));
    g_ebbl_evt      = xEventGroupCreate();

    if (!q_imu || !q_gps || !q_fusion_out || !q_ego_state ||
            !q_v2v_rx || !q_collision_in || !q_tft_collision ||
            !q_alert_tft || !g_ebbl_evt) {
            ESP_LOGE(TAG, "Queue/EventGroup creation failed! Check heap.");
            return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "All queues created:");
    ESP_LOGI(TAG, "  q_imu          (imu_data_t,      depth=4)");
    ESP_LOGI(TAG, "  q_gps          (gps_data_t,      depth=2)");
    ESP_LOGI(TAG, "  q_fusion_out   (fusion_output_t, depth=2)");
    ESP_LOGI(TAG, "  q_ego_state    (vehicle_state_t, depth=2)");
    ESP_LOGI(TAG, "  q_v2v_rx       (v2v_packet_t,    depth=8)");
    ESP_LOGI(TAG, "  q_collision_in (collision_input_t, depth=2)");
    ESP_LOGI(TAG, "  q_alert_tft    (alert_result_t,  depth=%d)", CFG_QLEN_ALERT);
    return ESP_OK;
}
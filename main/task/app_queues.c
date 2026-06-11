/* task/app_queues.c — Khởi tạo tất cả queue và event group toàn cục.
 *
 * Tất cả queue được tạo một lần duy nhất trong app_main, trước khi tạo task.
 * Các task truy cập qua biến extern khai báo trong app_queues.h. */
#include "task/app_queues.h"
#include "config.h"
#include "esp_log.h"

static const char *TAG = "app_queues";

/* ── Định nghĩa các queue & event group toàn cục ──────────────────────── */
QueueHandle_t      q_imu          = NULL;   /* imu_data_t,       IMU → Fusion      */
QueueHandle_t      q_gps          = NULL;   /* gps_data_t,       GPS → Fusion      */
QueueHandle_t      q_fusion_out   = NULL;   /* fusion_output_t,  Fusion → Localize */
QueueHandle_t      q_ego_state    = NULL;   /* vehicle_state_t,  Localize → V2V   */
QueueHandle_t      q_v2v_rx       = NULL;   /* v2v_packet_t,     ESP-NOW → V2V    */
QueueHandle_t      q_collision_in = NULL;   /* collision_input_t,V2V → Collision  */
QueueHandle_t      q_tft_collision = NULL;  /* collision_input_t,V2V → TFT (bản sao) */
QueueHandle_t      q_alert_tft    = NULL;   /* alert_result_t,   Collision → TFT  */
QueueHandle_t      q_alert_buzzer = NULL;   /* alert_result_t,   Collision → Buzzer */
EventGroupHandle_t g_ebbl_evt     = NULL;   /* EBBL_BRAKE_BIT: Fusion → V2V burst */


esp_err_t app_queues_init(void)
{
    q_imu           = xQueueCreate(CFG_QLEN_IMU,          sizeof(imu_data_t));
    q_gps           = xQueueCreate(CFG_QLEN_GPS,          sizeof(gps_data_t));
    q_fusion_out    = xQueueCreate(CFG_QLEN_FUSION_OUT,   sizeof(fusion_output_t));
    q_ego_state     = xQueueCreate(CFG_QLEN_EGO_STATE,    sizeof(vehicle_state_t));
    q_v2v_rx        = xQueueCreate(CFG_QLEN_V2V_RX,       sizeof(v2v_packet_t));
    q_collision_in  = xQueueCreate(CFG_QLEN_COLLISION_IN, sizeof(collision_input_t));
    q_tft_collision = xQueueCreate(CFG_QLEN_COLLISION_IN, sizeof(collision_input_t));
    q_alert_tft     = xQueueCreate(CFG_QLEN_ALERT,        sizeof(alert_result_t));
    q_alert_buzzer  = xQueueCreate(CFG_QLEN_ALERT,        sizeof(alert_result_t));
    g_ebbl_evt      = xEventGroupCreate();

    /* Kiểm tra tất cả đều được cấp phát thành công */
    if (!q_imu || !q_gps || !q_fusion_out || !q_ego_state ||
            !q_v2v_rx || !q_collision_in || !q_tft_collision ||
            !q_alert_tft || !q_alert_buzzer || !g_ebbl_evt) {
            ESP_LOGE(TAG, "Queue/EventGroup creation failed! Check heap.");
            return ESP_ERR_NO_MEM;
    }

    /* In thông tin khởi tạo thành công */
    ESP_LOGI(TAG, "All queues created:");
    ESP_LOGI(TAG, "  q_imu          (imu_data_t,      depth=4)");
    ESP_LOGI(TAG, "  q_gps          (gps_data_t,      depth=2)");
    ESP_LOGI(TAG, "  q_fusion_out   (fusion_output_t, depth=2)");
    ESP_LOGI(TAG, "  q_ego_state    (vehicle_state_t, depth=2)");
    ESP_LOGI(TAG, "  q_v2v_rx       (v2v_packet_t,    depth=8)");
    ESP_LOGI(TAG, "  q_collision_in (collision_input_t, depth=2)");
    ESP_LOGI(TAG, "  q_alert_tft    (alert_result_t,  depth=%d)", CFG_QLEN_ALERT);
    ESP_LOGI(TAG, "  q_alert_buzzer (alert_result_t,  depth=%d)", CFG_QLEN_ALERT);
    return ESP_OK;
}
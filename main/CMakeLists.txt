idf_component_register(
    SRCS
        # Root
        "main.c"

        # Collision
        "collision/ebbl/ebbl.c"

        # Drivers
        "drivers/gps/gps.c"
        "drivers/imu/i2c_bus.c"
        "drivers/imu/mpu6050.c"
        "drivers/tft/tft_driver.c"
        "drivers/buzzer/buzzer.c"

        # Fusion
        "fusion/complementary_filter/com_fil.c"
        "fusion/dead_reckoning/dead_reckoning.c"
        "fusion/geo_utils/geo_utils.c"
        "fusion/math_utils.c"

        # Tasks
        "task/app_queues.c"
        "task/task_collision.c"
        "task/task_display_tft.c"
        "task/task_fusion.c"
        "task/task_gps.c"
        "task/task_imu.c"
        "task/task_localization.c"
        "task/task_v2v.c"
        "task/task_buzzer.c"
        "task/task_button.c"

        # V2V layer
        "v2v/espnow_comm.c"
        "v2v/neighbor_table.c"
        "v2v/packet.c"

    INCLUDE_DIRS
        "."
        "drivers/gps"
        "drivers/imu"
        "drivers/buzzer"
        "task"
        "fusion"
        "v2v"


    REQUIRES
        driver
        esp_wifi
        esp_netif
        nvs_flash
        esp_timer
        freertos
        log
        esp_system
        esp_event
)


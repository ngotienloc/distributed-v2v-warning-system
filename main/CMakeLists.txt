file(GLOB_RECURSE MAIN_SRCS "main.c" "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
idf_component_register(
    SRCS
        ${MAIN_SRCS}

    INCLUDE_DIRS
        "."
        "drivers/gps"
        "drivers/imu"
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


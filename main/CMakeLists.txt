file(GLOB_RECURSE MAIN_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
idf_component_register(
    SRCS
        ${MAIN_SRCS}

    INCLUDE_DIRS
        "."
        "drivers"
        "task"
        "fusion"


    REQUIRES
        driver
        esp_timer
        freertos
        log
        esp_system
        esp_event
        nvs_flash
)


idf_component_register(
    SRCS
        "main.c"

        # TFT driver
        "drivers/tft/tft_driver.c"

        # V2V layer (ESP-NOW)
        "v2v/espnow_comm.c"

    INCLUDE_DIRS
        "."
        "drivers/tft"
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


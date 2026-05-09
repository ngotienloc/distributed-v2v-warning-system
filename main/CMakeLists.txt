# ── Source selection ─────────────────────────────────────────────────
# Set EBBL_TEST=1 to compile the EBBL hardware test instead of the real app.
#
# Flash xe TRƯỚC (role=0):   idf.py -DEBBL_TEST=1 -DEBBL_ROLE=0 build flash
# Flash xe SAU   (role=1):   idf.py -DEBBL_TEST=1 -DEBBL_ROLE=1 build flash
# Build bình thường:         idf.py build
# ─────────────────────────────────────────────────────────────────────
if(DEFINED EBBL_TEST AND EBBL_TEST EQUAL 1)
    message(STATUS "[V2V] *** EBBL_TEST MODE — using main_ebbl_test.c ***")
    set(ENTRY_SRC "main_ebbl_test.c")
else()
    set(ENTRY_SRC "main.c")
endif()

# Collect all .c files EXCEPT the non-selected entry point
file(GLOB_RECURSE ALL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
list(REMOVE_ITEM ALL_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/main_ebbl_test.c"
)
list(APPEND ALL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/${ENTRY_SRC}")

idf_component_register(
    SRCS
        ${ALL_SRCS}

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

# Pass EBBL_ROLE define into the build if set (default=0 if not given)
if(DEFINED EBBL_ROLE)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC EBBL_ROLE=${EBBL_ROLE})
endif()

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/ESP32TimeServerSettings.h")
file(READ "${CMAKE_CURRENT_LIST_DIR}/ESP32TimeServerSettings.h" TIME_SERVER_SETTINGS)
string(REGEX MATCH "#define[ \t]+NTP_SERVER_ENABLED[ \t]+1" NTP_SERVER_ENABLED "${TIME_SERVER_SETTINGS}")
string(REGEX MATCH "#define[ \t]+PTP_L2_SERVER_ENABLED[ \t]+1" PTP_L2_SERVER_ENABLED "${TIME_SERVER_SETTINGS}")
string(REGEX MATCH "#define[ \t]+PTP_L2_AUTHENTICATION_ENABLED[ \t]+1" PTP_L2_AUTH_ENABLED "${TIME_SERVER_SETTINGS}")
string(REGEX MATCH "#define[ \t]+NTP_AUTHENTICATION_ENABLED[ \t]+1" NTP_AUTH_ENABLED "${TIME_SERVER_SETTINGS}")

set(MAIN_SOURCES
        "main.cpp"
        "custom/app_metadata.cpp"
        "custom/health_check.cpp"
        "custom/determine_stack_sizes.cpp"
        "custom/gnss_receiver.cpp"
        "custom/gnss_uart.cpp"
)

set(MAIN_REQUIRES
        esp_eth
        esp_event
        esp_netif
        esp_http_server
        app_update
        bootloader_support
        mdns
        esp_hw_support
        esp_psram
        esp_timer
        nvs_flash
        driver
        esp_driver_mcpwm
        esp_driver_gpio
        esp_driver_uart
        esp_driver_ledc
        led_strip
        lwip
        mqtt
)

if(IDF_TARGET STREQUAL "esp32p4")
    list(APPEND MAIN_REQUIRES fatfs sdmmc hd44780 pcf8574)
elseif(IDF_TARGET STREQUAL "esp32s31")
    list(APPEND MAIN_REQUIRES jef-sure__hd44780 fatfs sdmmc esp_driver_sdspi esp_driver_spi)
endif()

list(APPEND MAIN_SOURCES "custom/native_ote.cpp")

if(NTP_SERVER_ENABLED)
    list(APPEND MAIN_SOURCES "custom/ntp_cache.c")
    if(NTP_AUTH_ENABLED)
        list(APPEND MAIN_SOURCES "custom/ntp_auth.cpp")
        list(APPEND MAIN_REQUIRES mbedtls)
    endif()
endif()

if(PTP_L2_SERVER_ENABLED AND PTP_L2_AUTH_ENABLED)
    list(APPEND MAIN_SOURCES "custom/ptp_l2_auth.cpp")
    list(APPEND MAIN_REQUIRES mbedtls)
endif()

idf_component_register(
    SRCS ${MAIN_SOURCES}
    INCLUDE_DIRS
        "."
    REQUIRES ${MAIN_REQUIRES}
)

# ---------------------------------------------------------------------------
# Embed custom metadata into the binary
# ---------------------------------------------------------------------------

# Build date/time
string(TIMESTAMP BUILD_DATE "%Y-%m-%d")
string(TIMESTAMP BUILD_TIME "%H:%M:%S")

# Git SHA (short)
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Metadata definitions injected into the main component
target_compile_definitions(${COMPONENT_TARGET} PRIVATE
    APP_PROJECT_NAME="${CMAKE_PROJECT_NAME}"
    APP_VERSION="${PROJECT_VER}"
    APP_COPYRIGHT="Copyright Rob Latour, 2026"
    APP_LICENSE="MIT"
    APP_HOMEPAGE="https://github.com/roblatour/ESP32TimeServer"
    APP_BUILD_DATE="${BUILD_DATE}"
    APP_BUILD_TIME="${BUILD_TIME}"
    APP_GIT_SHA="${GIT_SHA}"
)

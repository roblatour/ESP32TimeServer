file(READ "${CMAKE_CURRENT_LIST_DIR}/ESP32TimeServerSettings.h" TIME_SERVER_SETTINGS)
string(REGEX MATCH "#define[ \t]+SYMMETRIC_KEY_AUTHENTICATION_ENABLED[ \t]+1" NTP_AUTH_ENABLED "${TIME_SERVER_SETTINGS}")
string(REGEX MATCH "#define[ \t]+RBG_LED_WS2812_MODULE_ENABLED[ \t]+1" WS2812_ENABLED "${TIME_SERVER_SETTINGS}")

set(MAIN_SOURCES
        "main.cpp"
        "custom/app_metadata.cpp"
        "custom/ntp_cache.c"
)

set(MAIN_REQUIRES
        esp_eth
        esp_event
        esp_netif
        esp_hw_support
        esp_timer
        nvs_flash
        driver
        esp_driver_mcpwm
        lwip
        mqtt
        fatfs
        sdmmc
        hd44780
        pcf8574
        arduino-esp32
        SparkFun_u-blox_GNSS_v3
)

if(NTP_AUTH_ENABLED)
    list(APPEND MAIN_SOURCES "custom/ntp_auth.cpp")
    list(APPEND MAIN_REQUIRES mbedtls)
endif()

if(WS2812_ENABLED)
    list(APPEND MAIN_REQUIRES led_strip)
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

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32TimeServer
        VERSION 2.8.1
        SPDX_LICENSE MIT
        DESCRIPTION "ESP32 NTP Stratum 1 Time Server for your home network"
)

# Require ESP-IDF 6.1 or newer
if (IDF_VERSION_MAJOR LESS 6 OR
    (IDF_VERSION_MAJOR EQUAL 6 AND IDF_VERSION_MINOR LESS 1))
    message(FATAL_ERROR "This project requires ESP-IDF >= 6.1. \
Current version: ${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")
endif()

# Workaround for Arduino-ESP32 hosted-mode bug
if(TARGET espressif__arduino-esp32)
    target_compile_definitions(espressif__arduino-esp32 PRIVATE
        CONFIG_ESP_HOSTED_IDF_SLAVE_TARGET="esp32p4"
    )
endif()

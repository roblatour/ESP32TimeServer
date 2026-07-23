# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32TimeServer
        VERSION 2.2.1
        SPDX_LICENSE MIT
        DESCRIPTION "ESP32 NTP Stratum 1 Time Server for your home network"
        HOMEPAGE_URL "https://github.com/roblatour/ESP32TimeServer"
        )

# ==============================================================================
# WORKAROUND: Fix arduino-esp32 compilation error on ESP-IDF v5.5.5+ and v6.0.2+
# ==============================================================================
# The external 'espressif__arduino-esp32' library's 'esp32-hal-hosted.c' file
# hardcodes a reference to CONFIG_ESP_HOSTED_IDF_SLAVE_TARGET. However, this macro 
# is missing or renamed in newer ESP-IDF Kconfig structures when CONFIG_ESP_HOSTED 
# is disabled, causing a compilation failure.
#
# To avoid polluting the global namespace, we target-scope this definition strictly 
# to the arduino-esp32 component after the project has been fully initialized.
# ==============================================================================
if(TARGET espressif__arduino-esp32)
    target_compile_definitions(espressif__arduino-esp32 PRIVATE 
        CONFIG_ESP_HOSTED_IDF_SLAVE_TARGET="esp32c6"
    )
endif()

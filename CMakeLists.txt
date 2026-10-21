cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32TimeServer
        VERSION 3.0.5
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

# Third-party components and -O2 (CONFIG_COMPILER_OPTIMIZATION_PERF)
# ESP-IDF compiles every component with -Werror=all (CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS only replaces -Werror by
# -Werror=all), and esp-modbus even adds -Werror itself. With -O2 GCC runs additional flow analyses (uninitialized
# variables, string and format length checks) that it does not run with -Og, and it reports such warnings in library
# code this project does not maintain, e.g. -Wmaybe-uninitialized in SparkFun_u-blox_GNSS_v3 (3rdparty submodule) and
# -Wstringop-truncation in esp-modbus (dependency of arduino-esp32, not used here). For the components in this project's
# managed_components and components folders these warnings stay warnings instead of errors.
# The project's own code (main) and the ESP-IDF components keep the strict setting.
set(third_party_relaxed_warnings
    -Wno-error=maybe-uninitialized
    -Wno-error=uninitialized
    -Wno-error=stringop-truncation
    -Wno-error=stringop-overflow
    -Wno-error=stringop-overread
    -Wno-error=format-truncation
    -Wno-error=format-overflow
    -Wno-error=array-bounds
    -Wno-error=restrict
    -Wno-error=dangling-pointer
    -Wno-error=use-after-free
)
file(TO_CMAKE_PATH "${CMAKE_SOURCE_DIR}" project_dir)
string(TOLOWER "${project_dir}" project_dir_lower)
idf_build_get_property(build_components BUILD_COMPONENTS)
foreach(component ${build_components})
    if(component STREQUAL "main")
        continue()
    endif()
    idf_component_get_property(component_dir ${component} COMPONENT_DIR)
    idf_component_get_property(component_lib ${component} COMPONENT_LIB)
    if(NOT component_dir OR NOT component_lib OR NOT TARGET ${component_lib})
        continue()
    endif()
    file(TO_CMAKE_PATH "${component_dir}" component_dir)
    string(TOLOWER "${component_dir}" component_dir_lower)
    string(FIND "${component_dir_lower}" "${project_dir_lower}/" prefix_position)
    string(LENGTH "${project_dir_lower}/" prefix_length)
    set(relative_dir "")
    if(prefix_position EQUAL 0)
        string(SUBSTRING "${component_dir_lower}" ${prefix_length} -1 relative_dir)
    endif()
    if(relative_dir MATCHES "^(managed_components|components)/")
        get_target_property(component_lib_type ${component_lib} TYPE)
        if(NOT component_lib_type STREQUAL "INTERFACE_LIBRARY")
            target_compile_options(${component_lib} PRIVATE ${third_party_relaxed_warnings})
        endif()
    endif()
endforeach()

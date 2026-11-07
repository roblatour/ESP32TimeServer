cmake_minimum_required(VERSION 3.16)

if(DEFINED IDF_TARGET AND NOT IDF_TARGET STREQUAL "" AND
   NOT IDF_TARGET STREQUAL "esp32p4" AND NOT IDF_TARGET STREQUAL "esp32s31")
    message(FATAL_ERROR "ESP32TimeServer supports only esp32p4 and esp32s31 targets")
elseif(NOT DEFINED IDF_TARGET AND DEFINED ENV{IDF_TARGET} AND
       NOT "$ENV{IDF_TARGET}" STREQUAL "" AND NOT "$ENV{IDF_TARGET}" STREQUAL "esp32p4" AND
       NOT "$ENV{IDF_TARGET}" STREQUAL "esp32s31")
    message(FATAL_ERROR "ESP32TimeServer supports only esp32p4 and esp32s31 targets")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Each build directory resolves its own target-specific component graph.
idf_build_set_property(DEPENDENCIES_LOCK "${CMAKE_BINARY_DIR}/dependencies.lock")
project(ESP32TimeServer
        VERSION 4.0.0
        SPDX_LICENSE MIT
        DESCRIPTION "ESP32 NTP and PTP Stratum 1 Time Server for your home network"
)

if(NOT IDF_TARGET STREQUAL "esp32p4" AND NOT IDF_TARGET STREQUAL "esp32s31")
    message(FATAL_ERROR "ESP32TimeServer supports only esp32p4 and esp32s31 targets")
endif()

# Require ESP-IDF 6.1 or newer
if (IDF_VERSION_MAJOR LESS 6 OR
    (IDF_VERSION_MAJOR EQUAL 6 AND IDF_VERSION_MINOR LESS 1))
    message(FATAL_ERROR "This project requires ESP-IDF >= 6.1. \
Current version: ${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")
endif()

# Third-party components and -O2 (CONFIG_COMPILER_OPTIMIZATION_PERF)
# ESP-IDF compiles every component with -Werror=all (CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS only replaces -Werror by
# -Werror=all). With -O2 GCC runs additional flow analyses which can warn in
# third-party components. Project-owned code and ESP-IDF remain strict.
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

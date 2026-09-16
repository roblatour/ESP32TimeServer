// ESP32 Time Server  
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#include "app_metadata.h"

static const app_metadata_t metadata = {
    APP_PROJECT_NAME,
    APP_VERSION,
    APP_COPYRIGHT,
    APP_LICENSE,
    APP_HOMEPAGE,
    APP_BUILD_DATE,
    APP_BUILD_TIME,
    APP_GIT_SHA};

const app_metadata_t *get_app_metadata()
{
    return &metadata;
}

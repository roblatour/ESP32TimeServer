// ESP32 Time Server  
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

typedef struct
{
    const char *project_name;
    const char *version;
    const char *copyright;
    const char *license;
    const char *homepage;
    const char *build_date;
    const char *build_time;
    const char *git_sha;
} app_metadata_t;

const app_metadata_t *get_app_metadata();

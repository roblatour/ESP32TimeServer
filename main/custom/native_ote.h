#pragma once

#include "esp_err.h"
#include "esp_netif.h"

struct native_ote_callbacks_t
{
    void (*progress)(unsigned int, unsigned int);
    void (*failure)(const char *);
    void (*success)();
};

// Call once after Ethernet is initialized. Callbacks run on the HTTP server task.
esp_err_t native_ote_start(esp_netif_t *netif, const native_ote_callbacks_t &callbacks);

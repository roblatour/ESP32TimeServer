// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#include "ntp_cache.h"
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static ntp_client_record_t s_ipv4_records[MAX_TRACKED_CLIENTS_IPV4];
static ntp_client_record_ipv6_t s_ipv6_records[MAX_TRACKED_CLIENTS_IPV6];
static SemaphoreHandle_t s_mutex;

static int64_t current_time_ms(void)
{
    return esp_timer_get_time() / 1000;
}

bool ntp_cache_init(void)
{
    if (s_mutex != NULL)
        return true;
    memset(s_ipv4_records, 0, sizeof(s_ipv4_records));
    memset(s_ipv6_records, 0, sizeof(s_ipv6_records));
    s_mutex = xSemaphoreCreateMutex();
    return s_mutex != NULL;
}

void ntp_cache_deinit(void)
{
    if (s_mutex != NULL)
    {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }
}

bool ntp_cache_find(uint32_t ip, uint64_t origin, ntp_client_record_t *record)
{
    if (s_mutex == NULL || record == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    memset(record, 0, sizeof(*record));
    size_t match = MAX_TRACKED_CLIENTS_IPV4;
    const int64_t now_ms = current_time_ms();
    for (size_t i = 0; origin != 0 && i < MAX_TRACKED_CLIENTS_IPV4; ++i)
    {
        if (s_ipv4_records[i].is_active && s_ipv4_records[i].client_ip == ip &&
            s_ipv4_records[i].prev_t2 == origin &&
            now_ms - s_ipv4_records[i].last_seen_ms <= CACHE_TIMEOUT_MS_IPV4)
        {
            if (match != MAX_TRACKED_CLIENTS_IPV4)
            {
                // Ambiguous cookies must fall back to basic mode, never another client's T3.
                xSemaphoreGive(s_mutex);
                return true;
            }
            match = i;
        }
    }
    if (match != MAX_TRACKED_CLIENTS_IPV4)
    {
        *record = s_ipv4_records[match];
        s_ipv4_records[match].is_active = false;
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool ntp_cache_update(uint32_t ip, uint64_t t2, uint64_t t3)
{
    if (s_mutex == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    size_t selected = 0;
    int64_t oldest_ms = INT64_MAX;
    for (size_t i = 0; i < MAX_TRACKED_CLIENTS_IPV4; ++i)
    {
        if (!s_ipv4_records[i].is_active)
        {
            selected = i;
            break;
        }
        if (s_ipv4_records[i].last_seen_ms < oldest_ms)
        {
            selected = i;
            oldest_ms = s_ipv4_records[i].last_seen_ms;
        }
    }
    s_ipv4_records[selected] = (ntp_client_record_t){
        .client_ip = ip, .prev_t2 = t2, .prev_t3 = t3,
        .last_seen_ms = current_time_ms(), .is_active = true};
    xSemaphoreGive(s_mutex);
    return true;
}

bool ntp_cache_find_ipv6(const struct in6_addr *ip, uint64_t origin, ntp_client_record_ipv6_t *record)
{
    if (s_mutex == NULL || ip == NULL || record == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    memset(record, 0, sizeof(*record));
    size_t match = MAX_TRACKED_CLIENTS_IPV6;
    const int64_t now_ms = current_time_ms();
    for (size_t i = 0; origin != 0 && i < MAX_TRACKED_CLIENTS_IPV6; ++i)
    {
        if (s_ipv6_records[i].is_active && memcmp(&s_ipv6_records[i].client_ip, ip, sizeof(*ip)) == 0 &&
            s_ipv6_records[i].prev_t2 == origin &&
            now_ms - s_ipv6_records[i].last_seen_ms <= CACHE_TIMEOUT_MS_IPV6)
        {
            if (match != MAX_TRACKED_CLIENTS_IPV6)
            {
                xSemaphoreGive(s_mutex);
                return true;
            }
            match = i;
        }
    }
    if (match != MAX_TRACKED_CLIENTS_IPV6)
    {
        *record = s_ipv6_records[match];
        s_ipv6_records[match].is_active = false;
    }
    xSemaphoreGive(s_mutex);
    return true;
}

bool ntp_cache_update_ipv6(const struct in6_addr *ip, uint64_t t2, uint64_t t3)
{
    if (s_mutex == NULL || ip == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    size_t selected = 0;
    int64_t oldest_ms = INT64_MAX;
    for (size_t i = 0; i < MAX_TRACKED_CLIENTS_IPV6; ++i)
    {
        if (!s_ipv6_records[i].is_active)
        {
            selected = i;
            break;
        }
        if (s_ipv6_records[i].last_seen_ms < oldest_ms)
        {
            selected = i;
            oldest_ms = s_ipv6_records[i].last_seen_ms;
        }
    }
    s_ipv6_records[selected] = (ntp_client_record_ipv6_t){
        .client_ip = *ip, .prev_t2 = t2, .prev_t3 = t3,
        .last_seen_ms = current_time_ms(), .is_active = true};
    xSemaphoreGive(s_mutex);
    return true;
}

void ntp_cache_purge_expired(void)
{
    if (s_mutex == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE)
        return;

    const int64_t now_ms = current_time_ms();
    for (size_t i = 0; i < MAX_TRACKED_CLIENTS_IPV4; ++i)
    {
        if (s_ipv4_records[i].is_active && now_ms - s_ipv4_records[i].last_seen_ms > CACHE_TIMEOUT_MS_IPV4)
            memset(&s_ipv4_records[i], 0, sizeof(s_ipv4_records[i]));
    }
    for (size_t i = 0; i < MAX_TRACKED_CLIENTS_IPV6; ++i)
    {
        if (s_ipv6_records[i].is_active && now_ms - s_ipv6_records[i].last_seen_ms > CACHE_TIMEOUT_MS_IPV6)
            memset(&s_ipv6_records[i], 0, sizeof(s_ipv6_records[i]));
    }
    xSemaphoreGive(s_mutex);
}

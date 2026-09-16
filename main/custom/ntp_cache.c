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
    {
        return true;
    }
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

bool ntp_cache_find_or_create(uint32_t ip, uint16_t port, ntp_client_record_t *record)
{
    if (s_mutex == NULL || record == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }

    const int64_t now_ms = current_time_ms();
    size_t selected = 0;
    bool found = false;
    int64_t oldest_ms = INT64_MAX;

    for (size_t i = 0; i < MAX_TRACKED_CLIENTS_IPV4; ++i)
    {
        if (s_ipv4_records[i].is_active && s_ipv4_records[i].client_ip == ip)
        {
            selected = i;
            found = true;
            break;
        }
        if (!s_ipv4_records[i].is_active)
        {
            selected = i;
            oldest_ms = INT64_MIN;
        }
        else if (oldest_ms != INT64_MIN && s_ipv4_records[i].last_seen_ms < oldest_ms)
        {
            selected = i;
            oldest_ms = s_ipv4_records[i].last_seen_ms;
        }
    }

    if (!found)
    {
        memset(&s_ipv4_records[selected], 0, sizeof(s_ipv4_records[selected]));
        s_ipv4_records[selected].client_ip = ip;
        s_ipv4_records[selected].client_port = port;
        s_ipv4_records[selected].last_seen_ms = now_ms;
        s_ipv4_records[selected].is_active = true;
    }

    *record = s_ipv4_records[selected];
    xSemaphoreGive(s_mutex);
    return true;
}

bool ntp_cache_update(uint32_t ip, uint16_t port, uint32_t t2_sec, uint32_t t2_ns, uint32_t t3_sec, uint32_t t3_ns)
{
    if (s_mutex == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }

    const int64_t now_ms = current_time_ms();
    size_t selected = 0;
    bool found = false;
    int64_t oldest_ms = INT64_MAX;

    for (size_t i = 0; i < MAX_TRACKED_CLIENTS_IPV4; ++i)
    {
        if (s_ipv4_records[i].is_active && s_ipv4_records[i].client_ip == ip)
        {
            selected = i;
            found = true;
            break;
        }
        if (!s_ipv4_records[i].is_active)
        {
            selected = i;
            oldest_ms = INT64_MIN;
        }
        else if (oldest_ms != INT64_MIN && s_ipv4_records[i].last_seen_ms < oldest_ms)
        {
            selected = i;
            oldest_ms = s_ipv4_records[i].last_seen_ms;
        }
    }

    if (!found)
    {
        memset(&s_ipv4_records[selected], 0, sizeof(s_ipv4_records[selected]));
        s_ipv4_records[selected].client_ip = ip;
        s_ipv4_records[selected].is_active = true;
    }

    s_ipv4_records[selected].client_port = port;
    s_ipv4_records[selected].prev_t2_sec = t2_sec;
    s_ipv4_records[selected].prev_t2_ns = t2_ns;
    s_ipv4_records[selected].prev_t3_sec = t3_sec;
    s_ipv4_records[selected].prev_t3_ns = t3_ns;
    s_ipv4_records[selected].last_seen_ms = now_ms;
    xSemaphoreGive(s_mutex);
    return true;
}

bool ntp_cache_find_or_create_ipv6(const struct in6_addr *ip, uint16_t port, ntp_client_record_ipv6_t *record)
{
    if (s_mutex == NULL || ip == NULL || record == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }

    const int64_t now_ms = current_time_ms();
    size_t selected = 0;
    bool found = false;
    int64_t oldest_ms = INT64_MAX;

    for (size_t i = 0; i < MAX_TRACKED_CLIENTS_IPV6; ++i)
    {
        if (s_ipv6_records[i].is_active && memcmp(&s_ipv6_records[i].client_ip, ip, sizeof(*ip)) == 0)
        {
            selected = i;
            found = true;
            break;
        }
        if (!s_ipv6_records[i].is_active)
        {
            selected = i;
            oldest_ms = INT64_MIN;
        }
        else if (oldest_ms != INT64_MIN && s_ipv6_records[i].last_seen_ms < oldest_ms)
        {
            selected = i;
            oldest_ms = s_ipv6_records[i].last_seen_ms;
        }
    }

    if (!found)
    {
        memset(&s_ipv6_records[selected], 0, sizeof(s_ipv6_records[selected]));
        s_ipv6_records[selected].client_ip = *ip;
        s_ipv6_records[selected].client_port = port;
        s_ipv6_records[selected].last_seen_ms = now_ms;
        s_ipv6_records[selected].is_active = true;
    }

    *record = s_ipv6_records[selected];
    xSemaphoreGive(s_mutex);
    return true;
}

bool ntp_cache_update_ipv6(const struct in6_addr *ip, uint16_t port, uint64_t t2, uint64_t t3)
{
    ntp_client_record_ipv6_t record;
    if (!ntp_cache_find_or_create_ipv6(ip, port, &record) || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }

    for (size_t i = 0; i < MAX_TRACKED_CLIENTS_IPV6; ++i)
    {
        if (s_ipv6_records[i].is_active && memcmp(&s_ipv6_records[i].client_ip, ip, sizeof(*ip)) == 0)
        {
            s_ipv6_records[i].client_port = port;
            s_ipv6_records[i].prev_t2 = t2;
            s_ipv6_records[i].prev_t3 = t3;
            s_ipv6_records[i].last_seen_ms = current_time_ms();
            xSemaphoreGive(s_mutex);
            return true;
        }
    }

    xSemaphoreGive(s_mutex);
    return false;
}

void ntp_cache_purge_expired(void)
{
    if (s_mutex == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    const int64_t now_ms = current_time_ms();

    for (size_t i = 0; i < MAX_TRACKED_CLIENTS_IPV4; ++i)
    {
        if (s_ipv4_records[i].is_active && now_ms - s_ipv4_records[i].last_seen_ms > CACHE_TIMEOUT_MS_IPV4)
        {
            memset(&s_ipv4_records[i], 0, sizeof(s_ipv4_records[i]));
        }
    }
    for (size_t i = 0; i < MAX_TRACKED_CLIENTS_IPV6; ++i)
    {
        if (s_ipv6_records[i].is_active && now_ms - s_ipv6_records[i].last_seen_ms > CACHE_TIMEOUT_MS_IPV6)
        {
            memset(&s_ipv6_records[i], 0, sizeof(s_ipv6_records[i]));
        }
    }
    xSemaphoreGive(s_mutex);
}

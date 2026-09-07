#include "ntp_cache.h"

#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static ntp_client_record_t s_records[MAX_TRACKED_CLIENTS];
static SemaphoreHandle_t s_mutex;

static int64_t current_time_ms(void)
{
    return esp_timer_get_time() / 1000;
}

bool ntp_cache_init(void)
{
    if (s_mutex != NULL) {
        return true;
    }
    s_mutex = xSemaphoreCreateMutex();
    return s_mutex != NULL;
}

bool ntp_cache_find_or_create(uint32_t ip, uint16_t port, ntp_client_record_t *record)
{
    if (s_mutex == NULL || record == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    const int64_t now_ms = current_time_ms();
    size_t selected = 0;
    bool found = false;
    int64_t oldest_ms = INT64_MAX;

    for (size_t i = 0; i < MAX_TRACKED_CLIENTS; ++i) {
        if (s_records[i].is_active && s_records[i].client_ip == ip) {
            selected = i;
            found = true;
            break;
        }
        if (!s_records[i].is_active) {
            selected = i;
            oldest_ms = INT64_MIN;
        } else if (oldest_ms != INT64_MIN && s_records[i].last_seen_ms < oldest_ms) {
            selected = i;
            oldest_ms = s_records[i].last_seen_ms;
        }
    }

    if (!found) {
        memset(&s_records[selected], 0, sizeof(s_records[selected]));
        s_records[selected].client_ip = ip;
        s_records[selected].client_port = port;
        s_records[selected].last_seen_ms = now_ms;
        s_records[selected].is_active = true;
    }

    *record = s_records[selected];
    xSemaphoreGive(s_mutex);
    return true;
}

bool ntp_cache_update(uint32_t ip, uint16_t port, uint32_t t2_sec, uint32_t t2_ns, uint32_t t3_sec, uint32_t t3_ns)
{
    ntp_client_record_t record;
    if (!ntp_cache_find_or_create(ip, port, &record) || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    for (size_t i = 0; i < MAX_TRACKED_CLIENTS; ++i) {
        if (s_records[i].is_active && s_records[i].client_ip == ip) {
            s_records[i].client_port = port;
            s_records[i].prev_t2_sec = t2_sec;
            s_records[i].prev_t2_ns = t2_ns;
            s_records[i].prev_t3_sec = t3_sec;
            s_records[i].prev_t3_ns = t3_ns;
            s_records[i].last_seen_ms = current_time_ms();
            xSemaphoreGive(s_mutex);
            return true;
        }
    }

    xSemaphoreGive(s_mutex);
    return false;
}

void ntp_cache_purge_expired(void)
{
    if (s_mutex == NULL || xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    const int64_t now_ms = current_time_ms();
    for (size_t i = 0; i < MAX_TRACKED_CLIENTS; ++i) {
        if (s_records[i].is_active && now_ms - s_records[i].last_seen_ms > CACHE_TIMEOUT_MS) {
            memset(&s_records[i], 0, sizeof(s_records[i]));
        }
    }
    xSemaphoreGive(s_mutex);
}

void ntp_cache_purge_task(void *parameter)
{
    (void)parameter;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        ntp_cache_purge_expired();
    }
}

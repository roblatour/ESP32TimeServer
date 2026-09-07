#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TRACKED_CLIENTS 128
#define CACHE_TIMEOUT_MS 300000

typedef struct {
    uint32_t client_ip;
    uint16_t client_port;
    uint32_t prev_t2_sec;
    uint32_t prev_t2_ns;
    uint32_t prev_t3_sec;
    uint32_t prev_t3_ns;
    int64_t last_seen_ms;
    bool is_active;
} ntp_client_record_t;

bool ntp_cache_init(void);
bool ntp_cache_find_or_create(uint32_t ip, uint16_t port, ntp_client_record_t *record);
bool ntp_cache_update(uint32_t ip, uint16_t port, uint32_t t2_sec, uint32_t t2_ns, uint32_t t3_sec, uint32_t t3_ns);
void ntp_cache_purge_expired(void);
void ntp_cache_purge_task(void *parameter);

#ifdef __cplusplus
}
#endif

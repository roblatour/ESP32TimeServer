// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lwip/inet.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_TRACKED_CLIENTS_IPV4 128
#define CACHE_TIMEOUT_MS_IPV4 1800000
#define MAX_TRACKED_CLIENTS_IPV6 128
#define CACHE_TIMEOUT_MS_IPV6 1800000

    typedef struct
    {
        uint32_t client_ip;
        uint64_t prev_t2;
        uint64_t prev_t3;
        int64_t last_seen_ms;
        bool is_active;
    } ntp_client_record_t;

    typedef struct
    {
        struct in6_addr client_ip;
        uint64_t prev_t2;
        uint64_t prev_t3;
        int64_t last_seen_ms;
        bool is_active;
    } ntp_client_record_ipv6_t;

    bool ntp_cache_init(void);
    void ntp_cache_deinit(void);
    // A successful lookup consumes the address + echoed T2 cookie. A miss returns a zeroed record.
    bool ntp_cache_find(uint32_t ip, uint64_t origin, ntp_client_record_t *record);
    bool ntp_cache_update(uint32_t ip, uint64_t t2, uint64_t t3);
    bool ntp_cache_find_ipv6(const struct in6_addr *ip, uint64_t origin, ntp_client_record_ipv6_t *record);
    bool ntp_cache_update_ipv6(const struct in6_addr *ip, uint64_t t2, uint64_t t3);
    void ntp_cache_purge_expired(void);

#ifdef __cplusplus
}
#endif

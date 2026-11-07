// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

#include "determine_stack_sizes.h"

#ifndef STARTUP_HEALTH_TEST_ENABLED
#define STARTUP_HEALTH_TEST_ENABLED 0 // 0 = Disabled; 1 = Enabled
#endif

// Optional startup health test
//
// When enabled the startup health test will perform a series of checks to ensure the system is functioning correctly at boot.
//
// These tests include:
//
// Part 1: checks the readiness of the PTP service
// Only runs when PTP_ENABLED is enabled.
//
// Part 2: the program sends itself NTPv3 and NTPv4 (standard and interleaved ) requests via IPv4 and IPv6,
// using loop back and assigned addresses,
//
// Part 3: it health test the same logic that all NTP requests go through to receive those requests,
// process and reply to them using standard and RFC 9769-compatible interleaved  responses,
//
// Part 4: it verifying the responses to ensure proper operations and reports a pass/fail result.
// Parts 2-4 and NTP-related MQTT checks only run when NTP_SERVER_ENABLED is enabled.
//
// Additionally, this activity is reflected in regular receive/send/client stats within MQTT reporting. .
//
// The startup health test can be useful during development and troubleshooting, but it slightly increase boot time
// and resource usage.
//
// However, it is recommended to keep this option be disabled in production builds to minimize boot time and resource usage.
// Also, and perhaps more importantly, as its results may be misinterpreted.
// For example: if the user's network does not support IPv6 the IPv6 tests may fail even though the system is
// functioning correctly.
//
// Here is what a successful NTP-only startup health test looks like in the logs.
// When PTP is enabled, its worker readiness result is test 01 and NTP tests follow.
// The PTP check verifies local readiness, not on-wire interoperability.
/*


I (39724) main_cpp: Health Check started
I (39725) main_cpp: Health Check 01 - IPv4 loopback: NTPv3 standard      - Passed
I (39725) main_cpp: Health Check 02 - IPv4 loopback: NTPv4 standard      - Passed
I (39731) main_cpp: Health Check 03 - IPv4 loopback: NTPv4 interleaved   - Passed
I (39739) main_cpp: Health Check 04 - IPv4 loopback: NTPv4 auth standard - Passed
I (39746) main_cpp: Health Check 05 - IPv4 loopback: NTPv4 auth interlv  - Passed
I (39753) main_cpp: Health Check 06 - IPv4 loopback: NTPv4 invalid auth  - Passed
I (39760) main_cpp: Health Check 07 - IPv4 loopback: NTPv4 unknown key   - Passed
I (39767) main_cpp: Health Check 08 - IPv4 loopback: NTPv4 truncated MAC - Passed
I (39775) main_cpp: Health Check 09 - IPv4 loopback: NTPv4 altered reply - Passed
I (39782) main_cpp: Health Check 10 - IPv4 assigned: NTPv3 standard      - Passed
I (39789) main_cpp: Health Check 11 - IPv4 assigned: NTPv4 standard      - Passed
I (39796) main_cpp: Health Check 12 - IPv4 assigned: NTPv4 interleaved   - Passed
I (39803) main_cpp: Health Check 13 - IPv4 assigned: NTPv4 auth standard - Passed
I (39811) main_cpp: Health Check 14 - IPv4 assigned: NTPv4 auth interlv  - Passed
I (39818) main_cpp: Health Check 15 - IPv4 assigned: NTPv4 invalid auth  - Passed
I (39825) main_cpp: Health Check 16 - IPv4 assigned: NTPv4 unknown key   - Passed
I (39832) main_cpp: Health Check 17 - IPv4 assigned: NTPv4 truncated MAC - Passed
I (39839) main_cpp: Health Check 18 - IPv4 assigned: NTPv4 altered reply - Passed
I (39847) main_cpp: Health Check 19 - IPv6 loopback: NTPv3 standard      - Passed
I (39854) main_cpp: Health Check 20 - IPv6 loopback: NTPv4 standard      - Passed
I (39861) main_cpp: Health Check 21 - IPv6 loopback: NTPv4 interleaved   - Passed
I (39868) main_cpp: Health Check 22 - IPv6 loopback: NTPv4 auth standard - Passed
I (39876) main_cpp: Health Check 23 - IPv6 loopback: NTPv4 auth interlv  - Passed
I (39883) main_cpp: Health Check 24 - IPv6 loopback: NTPv4 invalid auth  - Passed
I (39890) main_cpp: Health Check 25 - IPv6 loopback: NTPv4 unknown key   - Passed
I (39897) main_cpp: Health Check 26 - IPv6 loopback: NTPv4 truncated MAC - Passed
I (39904) main_cpp: Health Check 27 - IPv6 loopback: NTPv4 altered reply - Passed
I (39912) main_cpp: Health Check 28 - IPv6 assigned: NTPv3 standard      - Passed
I (39919) main_cpp: Health Check 29 - IPv6 assigned: NTPv4 standard      - Passed
I (39926) main_cpp: Health Check 30 - IPv6 assigned: NTPv4 interleaved   - Passed
I (39933) main_cpp: Health Check 31 - IPv6 assigned: NTPv4 auth standard - Passed
I (39940) main_cpp: Health Check 32 - IPv6 assigned: NTPv4 auth interlv  - Passed
I (39948) main_cpp: Health Check 33 - IPv6 assigned: NTPv4 invalid auth  - Passed
I (39955) main_cpp: Health Check 34 - IPv6 assigned: NTPv4 unknown key   - Passed
I (39962) main_cpp: Health Check 35 - IPv6 assigned: NTPv4 truncated MAC - Passed
I (39969) main_cpp: Health Check 36 - IPv6 assigned: NTPv4 altered reply - Passed
I (39976) main_cpp: Health Check 37 - MQTT metrics                       - Passed
I (39984) main_cpp: Health Check completed

Most of the health checks have fairly obvious purposes based on their names, but a couple of them,
truncated MAC and altered reply tests, may require more explanation (detailed below).

For truncated MAC:
This test sends an otherwise valid NTP packet but deliberately cuts off part of the authentication field (the Message Authentication Code)
before the server processes it. The server receives a packet whose header and timestamps look fine, but whose MAC is shorter than expected,
so when it tries to verify the MAC at the end of the packet, the check fails and the reply is rejected or flagged as invalid. In effect,
it’s testing that the server does not accept packets where the integrity/authentication data has been damaged or partially missing, even
though the rest of the packet structure appears normal.

For altered reply:
This test takes a valid NTP response and then modifies some of its contents—typically one or more timestamp fields or other critical header
values—after the server has generated it but before the client treats it as trustworthy. When the client compares the altered reply against
what it expects (e.g., matching originate/receive/transmit timestamps or internal state), the inconsistency causes the response to be rejected
or treated as an error rather than used to adjust time. By doing this, the test checks that the client detects tampering or corruption in the
reply and doesn’t blindly accept time data that has been changed in transit.

*/

#if STARTUP_HEALTH_TEST_ENABLED
void start_startup_health_check();
#if PTP_ENABLED
void report_startup_health_ptp_readiness(bool ready);
#endif
#endif

#if STARTUP_HEALTH_TEST_ENABLED && NTP_SERVER_ENABLED
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static constexpr size_t Startup_Health_Test_Task_Stack_Size = 4096;

struct startup_health_addresses_t
{
    char ipv4[INET_ADDRSTRLEN];
    char ipv6[INET6_ADDRSTRLEN];
};

// Read-only bindings to state owned by the running time and network services.
struct startup_health_context_t
{
    uint64_t (*current_ntp_time)();
    uint64_t (*read_timestamp)(const uint8_t *, size_t);
    void (*write_timestamp)(uint8_t *, size_t, uint64_t);
    startup_health_addresses_t (*addresses)();
    bool (*network_ready)();
    bool (*time_ready)();
    int (*ipv6_scope_id)();
#if MQTT_ENABLED && MQTT_CLIENT_REPORTING_ENABLED
    const QueueHandle_t &mqtt_event_queue;
    const SemaphoreHandle_t &mqtt_stats_mutex;
    uint32_t mqtt_publish_delay_ms;
    const std::atomic<uint32_t> &ntp_telemetry_events_dropped;
    const std::atomic<bool> &mqtt_client_table_overflown;
    const std::atomic<uint32_t> &ntp_authenticated_valid_requests;
    const std::atomic<uint32_t> &ntp_unauthenticated_valid_requests;
    const std::atomic<uint32_t> &ntp_authenticated_invalid_requests;
    const std::atomic<uint32_t> &ntp_valid_requests;
    const std::atomic<uint32_t> &ntp_invalid_requests;
    const std::atomic<uint32_t> &ntp_responses_synchronized_and_disciplined;
    const std::atomic<uint32_t> &ntp_responses;
    bool (*has_expected_client_requests)(const char *, sa_family_t, uint32_t);
#endif
};

extern const startup_health_context_t s_startup_health_context;
void startup_health_test_task(void *parameter);
#endif

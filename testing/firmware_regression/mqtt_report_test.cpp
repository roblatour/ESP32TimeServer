#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>
#include "ptp_followers.h"
#include "mqtt_report_queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static constexpr int MQTT_QOS = 1;
#include "mqtt_report_settings_under_test.h"
#include "mqtt_report_limits_under_test.h"

static constexpr size_t PTP_FOLLOWER_CAPACITY =
    (PTP_L2_SERVER_ENABLED ? PTP_L2_FOLLOWER_CAPACITY : 0) +
    (PTP_L3_SERVER_ENABLED ? PTP_L3_UDP_FOLLOWER_CAPACITY * ((PreferIPvX != 2) + (PreferIPvX != 1)) : 0);

using sa_family_t = uint16_t;
static constexpr sa_family_t AF_UNSPEC = 0, AF_INET = 2, AF_INET6 = 10;
struct in_addr { uint8_t bytes[4]; };
struct in6_addr { uint8_t bytes[16]; };
struct sockaddr_in { sa_family_t sin_family; in_addr sin_addr; };
struct sockaddr_in6 { sa_family_t sin6_family; in6_addr sin6_addr; };
struct sockaddr_storage { sa_family_t ss_family; uint8_t bytes[16]; };
static constexpr size_t IP_ADDRESS_TEXT_SIZE = 46;
static constexpr int MALLOC_CAP_DMA = 0, MALLOC_CAP_8BIT = 1;
static constexpr const char *TAG = "mqtt";
static int64_t s_now_us = 1000000;
static int64_t esp_timer_get_time() { return s_now_us; }
static time_t utc_now() { return 1792598400; }
static uint32_t gnss_locked_seconds_this_period() { return 1; }
static uint32_t heap_caps_get_free_size(int) { return UINT32_MAX; }
static uint32_t heap_caps_get_largest_free_block(int) { return UINT32_MAX; }
static uint32_t esp_get_free_heap_size() { return UINT32_MAX; }
static uint32_t esp_get_minimum_free_heap_size() { return UINT32_MAX; }
static void format_time_to_ISO8601(time_t, char *out, size_t capacity)
{
    std::snprintf(out, capacity, "2026-10-21T12:00:00+0000");
}
// Longest possible text for each family.
static bool format_socket_address(const sockaddr_storage &address, char *out, size_t capacity)
{
    std::snprintf(out, capacity, "%s", address.ss_family == AF_INET ? "255.255.255.255" : "ffff:ffff:ffff:ffff:ffff:ffff:255.255.255.255");
    return true;
}
static bool s_qualified = true;
static bool ptp_time_qualified() { return s_qualified; }
static ptp::FollowerTable<PTP_FOLLOWER_CAPACITY, MQTT_PTP_CLIENT_REPORT_LIMIT> s_ptp_followers(30000000);
static portMUX_TYPE s_ptp_followers_mux = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_mqtt_stats_mutex = reinterpret_cast<void *>(1);
static uint32_t s_mqtt_queued_messages_discarded = 0;
static int64_t s_mqtt_last_link_up_us = 0;
struct mqtt_client_request_t { sa_family_t address_family; uint8_t address[16]; uint32_t requests; };
static mqtt_client_request_t s_mqtt_clients[MQTT_NTP_CLIENT_LIMIT]{};
static size_t s_mqtt_client_count = 0;
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define TIMING_LOGW(...) ((void)0)
#include "mqtt_metrics_under_test.h"
#include "mqtt_report_under_test.h"

static size_t report(size_t capacity, bool expected, bool print = true)
{
    std::vector<char> buffer(capacity + 32, '~');
    assert(mqtt_build_report(buffer.data(), capacity) == expected);
    assert(std::all_of(buffer.begin() + static_cast<ptrdiff_t>(capacity), buffer.end(),
                       [](char value) { return value == '~'; }));
    if (expected && print)
        std::puts(buffer.data());
    else if (!expected && capacity > 0)
        assert(buffer[0] == '\0');
    return expected ? std::strlen(buffer.data()) : 0;
}

static void observe_report_limit_followers()
{
    const uint8_t identity[10]{};
    for (unsigned i = 0; i < MQTT_PTP_CLIENT_REPORT_LIMIT + 36; ++i)
    {
        const uint8_t mac[6] = {0xff, 0xff, 0xff, 0xff, static_cast<uint8_t>(i >> 8), static_cast<uint8_t>(i)};
        s_ptp_followers.observe(0, identity, s_now_us, mac);
    }
}

static void fill_clients(size_t count, sa_family_t family, uint32_t requests)
{
    s_mqtt_client_count = MQTT_NTP_CLIENTS_REPORTED ? count : 0;
    for (size_t i = 0; i < s_mqtt_client_count; ++i)
    {
        s_mqtt_clients[i] = {family, {}, requests};
        s_mqtt_clients[i].address[0] = static_cast<uint8_t>(i >> 8);
        s_mqtt_clients[i].address[1] = static_cast<uint8_t>(i);
    }
}

// Every counter at its maximum and every optional block present, so the measured size bounds real reports.
static void set_worst_case_state()
{
    mqtt_metrics_set_maximum();
    s_ethernet_connected = false;
    s_gnss_locked = false;
    s_ntp_gnss_timing_valid = false;
    s_ntp_pps_active = false;
    s_pps_discipline_active = false;
    s_mqtt_queued_messages_count = UINT32_MAX;
    s_mqtt_queued_messages_discarded = UINT32_MAX;
    s_satellite_count = UINT8_MAX;
    s_eth_link_up_total_us = 4294967295LL * 1000000LL;
    s_mqtt_last_link_up_us = 0;
    s_now_us = 4294967295LL * 1000000LL;
#if NTP_SERVER_ENABLED && MQTT_CLIENT_REPORTING_ENABLED
    s_mqtt_client_table_overflown = true;
#endif
}

int main()
{
    s_ethernet_connected = true;
    s_ptp_service_active = true;
    s_gnss_locked = true;
    s_ntp_gnss_timing_valid = true;
    s_ntp_pps_active = true;
    s_pps_discipline_active = true;
    for (unsigned i = 0; i < PTP_FOLLOWER_CAPACITY + 1; ++i)
    {
        uint8_t identity[10]{};
        identity[0] = static_cast<uint8_t>(i);
        s_ptp_followers.observe(0, identity, s_now_us, identity);
    }
    s_mqtt_client_count = 2;
    s_mqtt_clients[0] = {AF_INET6, {}, 9};
    s_mqtt_clients[1] = {AF_INET, {}, 7};
    report(MQTT_REPORT_BUFFER_SIZE, true);
    report(MQTT_REPORT_BUFFER_SIZE, true);
    s_now_us += 30000000;
    report(MQTT_REPORT_BUFFER_SIZE, true);
    const uint8_t identity[10]{};
    s_ptp_followers.observe(0, identity, s_now_us, identity);
    s_qualified = false;
    report(MQTT_REPORT_BUFFER_SIZE, true);
    s_qualified = true;

    // A typical report: ten IPv4 clients.
    observe_report_limit_followers();
    fill_clients(10, AF_INET, 1000);
    const size_t typical = report(MQTT_REPORT_BUFFER_SIZE, true);
    if (!PTP_ENABLED)
        assert(typical < 2048);

    // Full client lists of the longest addresses.
    observe_report_limit_followers();
    fill_clients(MQTT_NTP_CLIENT_LIMIT, AF_INET6, UINT32_MAX);
    report(MQTT_REPORT_BUFFER_SIZE, true);

    set_worst_case_state();
    fill_clients(0, AF_INET6, 0);
    s_ptp_followers.clear();
    const size_t base = report(MQTT_REPORT_BUFFER_SIZE, true, false);
    assert(base + 1 <= MQTT_REPORT_BASE_BYTES + (PTP_ENABLED ? ptp::FOLLOWER_REPORT_HEADER_BYTES : 0));

    set_worst_case_state();
    observe_report_limit_followers();
    fill_clients(MQTT_NTP_CLIENT_LIMIT, AF_INET6, UINT32_MAX);
    const size_t worst = report(MQTT_REPORT_BUFFER_SIZE, true, false);
    assert(worst - base <= (MQTT_NTP_CLIENTS_REPORTED ? MQTT_NTP_CLIENT_REPORT_LIMIT * MQTT_NTP_CLIENT_RECORD_BYTES : 0) +
                               (PTP_ENABLED ? MQTT_PTP_CLIENT_REPORT_LIMIT * ptp::FOLLOWER_REPORT_RECORD_BYTES : 0));
    assert(mqtt_queue::ReportQueue<MQTT_RAM_QUEUE_BYTES>::record_bytes(worst) <= MQTT_RAM_QUEUE_BYTES);
    std::fprintf(stderr, "worst-case report %zu bytes (base %zu) in a %zu-byte buffer\n", worst, base, MQTT_REPORT_BUFFER_SIZE);

    for (size_t capacity : {size_t(0), size_t(1), size_t(2), size_t(32), size_t(128)})
        report(capacity, false);
}

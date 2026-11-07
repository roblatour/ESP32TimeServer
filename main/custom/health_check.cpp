// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#include "health_check.h"

#if STARTUP_HEALTH_TEST_ENABLED
#include <cstdint>
#include "esp_log.h"

static const char *TAG = "main_cpp";
static uint32_t s_startup_health_test_number = 0;

void start_startup_health_check()
{
    s_startup_health_test_number = 0;
    ESP_LOGI(TAG, "Health Check started");
}

#if PTP_ENABLED
void report_startup_health_ptp_readiness(bool ready)
{
    ++s_startup_health_test_number;
    ESP_LOG_LEVEL(ready ? ESP_LOG_INFO : ESP_LOG_ERROR, TAG,
                  "Health Check %02lu - Enabled PTP worker readiness         - %s",
                  static_cast<unsigned long>(s_startup_health_test_number), ready ? "Passed" : "Failed");
}
#endif
#endif

#if STARTUP_HEALTH_TEST_ENABLED && NTP_SERVER_ENABLED
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "esp_timer.h"
#include "lwip/inet.h"
#if NTP_AUTHENTICATION_ENABLED
#include "ntp_auth.h"
#endif

static constexpr uint16_t NTP_PORT = 123;
static constexpr size_t NTP_PACKET_SIZE = 48;
#if NTP_AUTHENTICATION_ENABLED
static constexpr size_t NTP_MAX_PACKET_SIZE = NTP_PACKET_SIZE + NTP_AUTH_TRAILER_SIZE;
#endif
static constexpr bool IPV4_ENABLED = PreferIPvX != 2;
static constexpr bool IPV6_ENABLED = PreferIPvX != 1;
#endif

#if STARTUP_HEALTH_TEST_ENABLED && NTP_SERVER_ENABLED
static constexpr uint32_t Startup_Health_Test_Ready_Timeout_Ms = 30000;
static constexpr uint32_t Startup_Health_Test_Response_Timeout_Ms = 1000;
static constexpr uint32_t Startup_Health_Test_Standard_Retries = 3;
static constexpr uint64_t Startup_Health_Test_interleaved_Tolerance = (1ULL << 32) * 5ULL / 1000ULL;

struct startup_health_endpoint_results_t
{
    bool ntpv3_standard = false;
    bool ntpv4_standard = false;
    bool ntpv4_interleaved = false;
#if NTP_AUTHENTICATION_ENABLED
    bool ntpv4_authenticated_standard = false;
    bool ntpv4_authenticated_interleaved = false;
    bool ntpv4_invalid_authentication_rejected = false;
    bool ntpv4_unknown_key_rejected = false;
    bool ntpv4_truncated_mac_rejected = false;
    bool ntpv4_altered_response_rejected = false;
#endif
};

struct startup_health_test_results_t
{
    bool prerequisites_ready = false;
    startup_health_endpoint_results_t ipv4_loopback{};
    startup_health_endpoint_results_t ipv4_assigned{};
    startup_health_endpoint_results_t ipv6_loopback{};
    startup_health_endpoint_results_t ipv6_assigned{};
};

struct startup_health_exchange_t
{
    uint64_t t1 = 0;
    uint64_t t2 = 0;
    uint64_t t3 = 0;
    uint64_t t4 = 0;
};

static constexpr size_t Startup_Health_Test_Log_Capacity = 64;
static constexpr size_t Startup_Health_Test_Log_Message_Size = 192;

struct startup_health_log_entry_t
{
    esp_log_level_t level = ESP_LOG_INFO;
    char message[Startup_Health_Test_Log_Message_Size] = "";
};

static startup_health_test_results_t s_startup_health_test_results{};
static startup_health_log_entry_t *s_startup_health_test_log = nullptr;
static size_t s_startup_health_test_log_count = 0;

enum class startup_health_mqtt_metrics_result_t : uint8_t
{
    passed,
    failed,
    could_not_determine,
};

static void queue_startup_health_log(esp_log_level_t level, const char *format, ...)
{
    if (s_startup_health_test_log == nullptr || s_startup_health_test_log_count >= Startup_Health_Test_Log_Capacity)
        return;

    startup_health_log_entry_t &entry = s_startup_health_test_log[s_startup_health_test_log_count++];
    entry.level = level;
    va_list arguments;
    va_start(arguments, format);
    vsnprintf(entry.message, sizeof(entry.message), format, arguments);
    va_end(arguments);
}

static void queue_startup_health_test_result(const char *endpoint, const char *test_name, bool passed)
{
    ++s_startup_health_test_number;
    queue_startup_health_log(passed ? ESP_LOG_INFO : ESP_LOG_WARN,
                             "Health Check %02lu - %s: %s - %s",
                             static_cast<unsigned long>(s_startup_health_test_number), endpoint, test_name,
                             passed ? "Passed" : "Failed");
}

static void flush_startup_health_log()
{
    for (size_t index = 0; index < s_startup_health_test_log_count; ++index)
        ESP_LOG_LEVEL(s_startup_health_test_log[index].level, TAG, "%s", s_startup_health_test_log[index].message);
    s_startup_health_test_log_count = 0;
}

static bool receive_startup_health_response(int socket_fd, uint8_t *reply, size_t reply_capacity, size_t *reply_length, uint64_t *arrival_time)
{
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);
    struct timeval timeout{};
    timeout.tv_sec = Startup_Health_Test_Response_Timeout_Ms / 1000;
    timeout.tv_usec = static_cast<suseconds_t>(Startup_Health_Test_Response_Timeout_Ms % 1000) * 1000;
    if (select(socket_fd + 1, &read_fds, nullptr, nullptr, &timeout) != 1)
        return false;

    int received_length = recvfrom(socket_fd, reply, reply_capacity, 0, nullptr, nullptr);
    if (received_length < 0)
        return false;

    *reply_length = static_cast<size_t>(received_length);
    *arrival_time = s_startup_health_context.current_ntp_time();
    return true;
}

static bool run_standard_startup_health_test(int socket_fd, const struct sockaddr *destination, socklen_t destination_length,
                                             uint8_t version, startup_health_exchange_t *exchange)
{
    uint8_t request[NTP_PACKET_SIZE] = {};
    [[maybe_unused]] uint8_t reply[NTP_PACKET_SIZE] = {};
    exchange->t1 = s_startup_health_context.current_ntp_time();
    request[0] = static_cast<uint8_t>(version << 3) | 3;
    s_startup_health_context.write_timestamp(request, 40, exchange->t1);

    for (uint32_t attempt = 0; attempt < Startup_Health_Test_Standard_Retries; ++attempt)
    {
        if (sendto(socket_fd, request, sizeof(request), 0, destination, destination_length) != static_cast<int>(sizeof(request)))
            continue;

#if NTP_AUTHENTICATION_REQUIRED
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(socket_fd, &read_fds);
        struct timeval timeout{};
        timeout.tv_sec = Startup_Health_Test_Response_Timeout_Ms / 1000;
        timeout.tv_usec = static_cast<suseconds_t>(Startup_Health_Test_Response_Timeout_Ms % 1000) * 1000;
        const int ready = select(socket_fd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready < 0)
            queue_startup_health_log(ESP_LOG_ERROR, "Health Check unsigned rejection: select failed: errno %d", errno);
        if (ready > 0)
            recvfrom(socket_fd, reply, sizeof(reply), 0, nullptr, nullptr);
        return ready == 0;
#else
        size_t reply_length = 0;
        if (!receive_startup_health_response(socket_fd, reply, sizeof(reply), &reply_length, &exchange->t4) || reply_length != sizeof(reply))
            continue;

        if (((reply[0] >> 3) & 0x07) != version || (reply[0] & 0x07) != 4 || s_startup_health_context.read_timestamp(reply, 24) != exchange->t1)
            continue;

        exchange->t2 = s_startup_health_context.read_timestamp(reply, 32);
        exchange->t3 = s_startup_health_context.read_timestamp(reply, 40);
        return exchange->t2 != 0 && exchange->t3 != 0;
#endif
    }

    return false;
}

static bool run_interleaved_startup_health_test(int socket_fd, const struct sockaddr *destination, socklen_t destination_length,
                                                uint8_t version, const startup_health_exchange_t &previous_exchange)
{
    uint8_t request[NTP_PACKET_SIZE] = {};
    uint8_t reply[NTP_PACKET_SIZE] = {};
    size_t reply_length = 0;
    uint64_t arrival_time = 0;
    request[0] = static_cast<uint8_t>(version << 3) | 3;
    s_startup_health_context.write_timestamp(request, 24, previous_exchange.t2);
    s_startup_health_context.write_timestamp(request, 32, previous_exchange.t4);
    s_startup_health_context.write_timestamp(request, 40, previous_exchange.t1);

    if (sendto(socket_fd, request, sizeof(request), 0, destination, destination_length) != static_cast<int>(sizeof(request)) ||
        !receive_startup_health_response(socket_fd, reply, sizeof(reply), &reply_length, &arrival_time) || reply_length != sizeof(reply))
        return false;

    const uint64_t reply_transmit_time = s_startup_health_context.read_timestamp(reply, 40);
    const uint64_t transmit_difference = reply_transmit_time >= previous_exchange.t3
                                             ? reply_transmit_time - previous_exchange.t3
                                             : previous_exchange.t3 - reply_transmit_time;
    return ((reply[0] >> 3) & 0x07) == version && (reply[0] & 0x07) == 4 &&
           s_startup_health_context.read_timestamp(reply, 24) == previous_exchange.t4 &&
           transmit_difference <= Startup_Health_Test_interleaved_Tolerance;
}

#if NTP_AUTHENTICATION_ENABLED
static bool run_authenticated_startup_health_test(int socket_fd, const struct sockaddr *destination, socklen_t destination_length,
                                                  startup_health_exchange_t *exchange, bool *altered_response_rejected)
{
    uint8_t request[NTP_MAX_PACKET_SIZE] = {};
    uint8_t reply[NTP_MAX_PACKET_SIZE] = {};
    size_t request_length = NTP_PACKET_SIZE;
    exchange->t1 = s_startup_health_context.current_ntp_time();
    request[0] = static_cast<uint8_t>(4U << 3) | 3U;
    s_startup_health_context.write_timestamp(request, 40, exchange->t1);
    if (!ntp_auth_append_response(request, sizeof(request), &request_length, ntp_auth_default_key_id()))
        return false;

    for (uint32_t attempt = 0; attempt < Startup_Health_Test_Standard_Retries; ++attempt)
    {
        if (sendto(socket_fd, request, request_length, 0, destination, destination_length) != static_cast<int>(request_length))
            continue;
        size_t reply_length = 0;
        if (!receive_startup_health_response(socket_fd, reply, sizeof(reply), &reply_length, &exchange->t4) ||
            ntp_auth_verify_request(reply, reply_length, 4, nullptr) != ntp_auth_result_t::valid)
            continue;
        if (((reply[0] >> 3) & 0x07) != 4 || (reply[0] & 0x07) != 4 || s_startup_health_context.read_timestamp(reply, 24) != exchange->t1)
            continue;
        reply[40] ^= 0x01;
        *altered_response_rejected = ntp_auth_verify_request(reply, reply_length, 4, nullptr) == ntp_auth_result_t::invalid_mac;
        reply[40] ^= 0x01;
        exchange->t2 = s_startup_health_context.read_timestamp(reply, 32);
        exchange->t3 = s_startup_health_context.read_timestamp(reply, 40);
        return exchange->t2 != 0 && exchange->t3 != 0;
    }
    return false;
}

enum class startup_health_invalid_auth_t : uint8_t
{
    wrong_mac,
    unknown_key,
    truncated_mac,
};

static bool run_invalid_authenticated_startup_health_test(int socket_fd, const struct sockaddr *destination, socklen_t destination_length,
                                                          startup_health_invalid_auth_t invalid_auth)
{
    uint8_t request[NTP_MAX_PACKET_SIZE] = {};
    size_t request_length = NTP_PACKET_SIZE;
    request[0] = static_cast<uint8_t>(4U << 3) | 3U;
    s_startup_health_context.write_timestamp(request, 40, s_startup_health_context.current_ntp_time());
    if (!ntp_auth_append_response(request, sizeof(request), &request_length, ntp_auth_default_key_id()))
        return false;
    switch (invalid_auth)
    {
    case startup_health_invalid_auth_t::wrong_mac:
        request[request_length - 1] ^= 0x01;
        break;
    case startup_health_invalid_auth_t::unknown_key:
        memset(request + NTP_PACKET_SIZE, 0, NTP_AUTH_KEY_ID_SIZE);
        break;
    case startup_health_invalid_auth_t::truncated_mac:
        --request_length;
        break;
    }
    if (sendto(socket_fd, request, request_length, 0, destination, destination_length) != static_cast<int>(request_length))
        return false;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);
    struct timeval timeout{};
    timeout.tv_sec = Startup_Health_Test_Response_Timeout_Ms / 1000;
    timeout.tv_usec = static_cast<suseconds_t>(Startup_Health_Test_Response_Timeout_Ms % 1000) * 1000;
    return select(socket_fd + 1, &read_fds, nullptr, nullptr, &timeout) == 0;
}

static bool run_authenticated_interleaved_startup_health_test(int socket_fd, const struct sockaddr *destination, socklen_t destination_length,
                                                              const startup_health_exchange_t &previous_exchange)
{
    uint8_t request[NTP_MAX_PACKET_SIZE] = {};
    uint8_t reply[NTP_MAX_PACKET_SIZE] = {};
    size_t request_length = NTP_PACKET_SIZE;
    uint64_t arrival_time = 0;
    request[0] = static_cast<uint8_t>(4U << 3) | 3U;
    s_startup_health_context.write_timestamp(request, 24, previous_exchange.t2);
    s_startup_health_context.write_timestamp(request, 32, previous_exchange.t4);
    s_startup_health_context.write_timestamp(request, 40, previous_exchange.t1);
    if (!ntp_auth_append_response(request, sizeof(request), &request_length, ntp_auth_default_key_id()) ||
        sendto(socket_fd, request, request_length, 0, destination, destination_length) != static_cast<int>(request_length))
        return false;
    size_t reply_length = 0;
    if (!receive_startup_health_response(socket_fd, reply, sizeof(reply), &reply_length, &arrival_time) ||
        ntp_auth_verify_request(reply, reply_length, 4, nullptr) != ntp_auth_result_t::valid)
        return false;
    const uint64_t reply_transmit_time = s_startup_health_context.read_timestamp(reply, 40);
    const uint64_t transmit_difference = reply_transmit_time >= previous_exchange.t3
                                             ? reply_transmit_time - previous_exchange.t3
                                             : previous_exchange.t3 - reply_transmit_time;
    return (reply[0] & 0x07) == 4 && s_startup_health_context.read_timestamp(reply, 24) == previous_exchange.t4 &&
           transmit_difference <= Startup_Health_Test_interleaved_Tolerance;
}
#endif

static startup_health_mqtt_metrics_result_t check_startup_health_mqtt_metrics(const startup_health_test_results_t &results)
{
#if MQTT_ENABLED && MQTT_CLIENT_REPORTING_ENABLED
    if (s_startup_health_context.mqtt_event_queue == nullptr || s_startup_health_context.mqtt_stats_mutex == nullptr)
        return startup_health_mqtt_metrics_result_t::could_not_determine;

    uint32_t expected_authenticated_valid = 0;
    uint32_t expected_unauthenticated_valid = 0;
    uint32_t expected_authenticated_invalid = 0;
    bool indeterminate = false;
#if NTP_AUTHENTICATION_ENABLED
    const bool authentication_available = ntp_auth_available();
#endif
    const auto count_endpoint = [&](const startup_health_endpoint_results_t &endpoint)
    {
        const bool unauthenticated_valid = endpoint.ntpv3_standard && endpoint.ntpv4_standard && endpoint.ntpv4_interleaved;
        if (!NTP_AUTHENTICATION_REQUIRED && unauthenticated_valid)
            expected_unauthenticated_valid += 3;
        else if (!unauthenticated_valid)
            indeterminate = true;
#if NTP_AUTHENTICATION_ENABLED
        if (authentication_available)
        {
            const bool authenticated_valid = endpoint.ntpv4_authenticated_standard && endpoint.ntpv4_authenticated_interleaved;
            if (authenticated_valid)
                expected_authenticated_valid += 2;
            else
                indeterminate = true;

            const bool authenticated_invalid = endpoint.ntpv4_invalid_authentication_rejected && endpoint.ntpv4_unknown_key_rejected &&
                                               endpoint.ntpv4_truncated_mac_rejected;
            if (authenticated_invalid)
                expected_authenticated_invalid += 3;
            else
                indeterminate = true;
        }
#endif
    };

    const startup_health_addresses_t addresses = s_startup_health_context.addresses();
    if (results.prerequisites_ready)
    {
        if (IPV4_ENABLED && addresses.ipv4[0] != '\0')
        {
            count_endpoint(results.ipv4_loopback);
            count_endpoint(results.ipv4_assigned);
        }
        if (IPV6_ENABLED && addresses.ipv6[0] != '\0')
        {
            count_endpoint(results.ipv6_loopback);
            count_endpoint(results.ipv6_assigned);
        }
    }

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(s_startup_health_context.mqtt_publish_delay_ms * 2U);
    while (uxQueueMessagesWaiting(s_startup_health_context.mqtt_event_queue) != 0 && xTaskGetTickCount() < deadline)
        vTaskDelay(pdMS_TO_TICKS(1));

    if (uxQueueMessagesWaiting(s_startup_health_context.mqtt_event_queue) != 0 || s_startup_health_context.ntp_telemetry_events_dropped.load(std::memory_order_relaxed) != 0 ||
        s_startup_health_context.mqtt_client_table_overflown.load(std::memory_order_relaxed))
        return startup_health_mqtt_metrics_result_t::could_not_determine;

    if (indeterminate || !results.prerequisites_ready)
        return startup_health_mqtt_metrics_result_t::could_not_determine;

    const uint32_t expected_valid = expected_authenticated_valid + expected_unauthenticated_valid;
    if (s_startup_health_context.ntp_authenticated_valid_requests.load(std::memory_order_relaxed) < expected_authenticated_valid ||
        s_startup_health_context.ntp_unauthenticated_valid_requests.load(std::memory_order_relaxed) < expected_unauthenticated_valid ||
        s_startup_health_context.ntp_authenticated_invalid_requests.load(std::memory_order_relaxed) < expected_authenticated_invalid ||
        s_startup_health_context.ntp_valid_requests.load(std::memory_order_relaxed) < expected_valid ||
        s_startup_health_context.ntp_invalid_requests.load(std::memory_order_relaxed) < expected_authenticated_invalid ||
        s_startup_health_context.ntp_responses_synchronized_and_disciplined.load(std::memory_order_relaxed) < expected_valid ||
        s_startup_health_context.ntp_responses.load(std::memory_order_relaxed) < expected_valid)
        return startup_health_mqtt_metrics_result_t::failed;

    const auto endpoint_expected_valid_requests = [
#if NTP_AUTHENTICATION_ENABLED
                                                      authentication_available
#endif
    ](const startup_health_endpoint_results_t &endpoint)
    {
        uint32_t expected_requests = !NTP_AUTHENTICATION_REQUIRED &&
                                             endpoint.ntpv3_standard && endpoint.ntpv4_standard && endpoint.ntpv4_interleaved
                                         ? 3U
                                         : 0U;
#if NTP_AUTHENTICATION_ENABLED
        if (authentication_available && endpoint.ntpv4_authenticated_standard && endpoint.ntpv4_authenticated_interleaved)
            expected_requests += 2;
#endif
        return expected_requests;
    };
    const auto has_expected_client_requests = s_startup_health_context.has_expected_client_requests;

    if (xSemaphoreTake(s_startup_health_context.mqtt_stats_mutex, portMAX_DELAY) != pdTRUE)
        return startup_health_mqtt_metrics_result_t::could_not_determine;
    const bool clients_match = (!IPV4_ENABLED ||
                                (has_expected_client_requests("127.0.0.1", AF_INET, endpoint_expected_valid_requests(results.ipv4_loopback)) &&
                                 has_expected_client_requests(addresses.ipv4, AF_INET, endpoint_expected_valid_requests(results.ipv4_assigned)))) &&
                               (!IPV6_ENABLED ||
                                (has_expected_client_requests("::1", AF_INET6, endpoint_expected_valid_requests(results.ipv6_loopback)) &&
                                 has_expected_client_requests(addresses.ipv6, AF_INET6, endpoint_expected_valid_requests(results.ipv6_assigned))));
    xSemaphoreGive(s_startup_health_context.mqtt_stats_mutex);
    return clients_match ? startup_health_mqtt_metrics_result_t::passed : startup_health_mqtt_metrics_result_t::failed;
#else
    (void)results;
    return startup_health_mqtt_metrics_result_t::could_not_determine;
#endif
}

static startup_health_endpoint_results_t run_startup_health_endpoint_test(const char *name, const struct sockaddr *destination,
                                                                          socklen_t destination_length, sa_family_t family)
{
    startup_health_endpoint_results_t results{};
    int socket_fd = socket(family, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_fd < 0)
    {
        queue_startup_health_log(ESP_LOG_WARN, "Health Check %s: unable to create socket: errno %d", name, errno);
        queue_startup_health_test_result(name, "NTPv3 standard     ", false);
        queue_startup_health_test_result(name, "NTPv4 standard     ", false);
        queue_startup_health_test_result(name, "NTPv4 interleaved  ", false);
#if NTP_AUTHENTICATION_ENABLED
        if (ntp_auth_available())
        {
            queue_startup_health_test_result(name, "NTPv4 auth standard", false);
            queue_startup_health_test_result(name, "NTPv4 auth interlv ", false);
            queue_startup_health_test_result(name, "NTPv4 invalid auth ", false);
            queue_startup_health_test_result(name, "NTPv4 unknown key  ", false);
            queue_startup_health_test_result(name, "NTPv4 truncated MAC", false);
            queue_startup_health_test_result(name, "NTPv4 altered reply", false);
        }
#endif
        return results;
    }

    startup_health_exchange_t ntpv3_exchange{};
    startup_health_exchange_t ntpv4_exchange{};
    results.ntpv3_standard = run_standard_startup_health_test(socket_fd, destination, destination_length, 3, &ntpv3_exchange);
    results.ntpv4_standard = run_standard_startup_health_test(socket_fd, destination, destination_length, 4, &ntpv4_exchange);
    if (results.ntpv4_standard && !NTP_AUTHENTICATION_REQUIRED)
        results.ntpv4_interleaved = run_interleaved_startup_health_test(socket_fd, destination, destination_length, 4, ntpv4_exchange);
    else if (NTP_AUTHENTICATION_REQUIRED)
        results.ntpv4_interleaved = results.ntpv4_standard;
#if NTP_AUTHENTICATION_ENABLED
    if (ntp_auth_available())
    {
        startup_health_exchange_t authenticated_exchange{};
        results.ntpv4_authenticated_standard = run_authenticated_startup_health_test(socket_fd, destination, destination_length, &authenticated_exchange,
                                                                                     &results.ntpv4_altered_response_rejected);
        if (results.ntpv4_authenticated_standard)
            results.ntpv4_authenticated_interleaved = run_authenticated_interleaved_startup_health_test(socket_fd, destination, destination_length, authenticated_exchange);
        results.ntpv4_invalid_authentication_rejected = run_invalid_authenticated_startup_health_test(socket_fd, destination, destination_length,
                                                                                                      startup_health_invalid_auth_t::wrong_mac);
        results.ntpv4_unknown_key_rejected = run_invalid_authenticated_startup_health_test(socket_fd, destination, destination_length,
                                                                                           startup_health_invalid_auth_t::unknown_key);
        results.ntpv4_truncated_mac_rejected = run_invalid_authenticated_startup_health_test(socket_fd, destination, destination_length,
                                                                                             startup_health_invalid_auth_t::truncated_mac);
    }
#endif

    closesocket(socket_fd);
    queue_startup_health_test_result(name, NTP_AUTHENTICATION_REQUIRED ? "NTPv3 unsigned drop  " : "NTPv3 standard       ", results.ntpv3_standard);
    queue_startup_health_test_result(name, NTP_AUTHENTICATION_REQUIRED ? "NTPv4 unsigned drop  " : "NTPv4 standard       ", results.ntpv4_standard);
    if (!NTP_AUTHENTICATION_REQUIRED)
        queue_startup_health_test_result(name, "NTPv4 interleaved    ", results.ntpv4_interleaved);
#if NTP_AUTHENTICATION_ENABLED
    if (ntp_auth_available())
    {
        queue_startup_health_test_result(name, "NTPv4 auth standard  ", results.ntpv4_authenticated_standard);
        queue_startup_health_test_result(name, "NTPv4 auth interlv   ", results.ntpv4_authenticated_interleaved);
        queue_startup_health_test_result(name, "NTPv4 invalid auth   ", results.ntpv4_invalid_authentication_rejected);
        queue_startup_health_test_result(name, "NTPv4 unknown key    ", results.ntpv4_unknown_key_rejected);
        queue_startup_health_test_result(name, "NTPv4 truncated MAC  ", results.ntpv4_truncated_mac_rejected);
        queue_startup_health_test_result(name, "NTPv4 altered reply  ", results.ntpv4_altered_response_rejected);
    }
#endif
    return results;
}

static startup_health_test_results_t run_startup_health_tests()
{
    startup_health_test_results_t results{};
    const int64_t deadline_us = esp_timer_get_time() + static_cast<int64_t>(Startup_Health_Test_Ready_Timeout_Ms) * 1000;
    bool network_ready = false;
    do
    {
        network_ready = s_startup_health_context.network_ready();
        if (network_ready)
            break;
        vTaskDelay(pdMS_TO_TICKS(100));
    } while (esp_timer_get_time() < deadline_us);

    if (!network_ready || !s_startup_health_context.time_ready())
    {
        queue_startup_health_log(ESP_LOG_ERROR, "Health Check prerequisites were not ready before timeout");
        return results;
    }

    results.prerequisites_ready = true;
    const startup_health_addresses_t addresses = s_startup_health_context.addresses();
    if (IPV4_ENABLED && addresses.ipv4[0] != '\0')
    {
        struct sockaddr_in ipv4_loopback{};
        ipv4_loopback.sin_family = AF_INET;
        ipv4_loopback.sin_port = htons(NTP_PORT);
        ipv4_loopback.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        results.ipv4_loopback = run_startup_health_endpoint_test("IPv4 loopback", reinterpret_cast<const struct sockaddr *>(&ipv4_loopback), sizeof(ipv4_loopback), AF_INET);

        struct sockaddr_in ipv4_assigned{};
        ipv4_assigned.sin_family = AF_INET;
        ipv4_assigned.sin_port = htons(NTP_PORT);
        if (inet_pton(AF_INET, addresses.ipv4, &ipv4_assigned.sin_addr) == 1)
            results.ipv4_assigned = run_startup_health_endpoint_test("IPv4 assigned", reinterpret_cast<const struct sockaddr *>(&ipv4_assigned), sizeof(ipv4_assigned), AF_INET);
        else
            queue_startup_health_log(ESP_LOG_ERROR, "Health Check IPv4 assigned: invalid address");
    }

    if (IPV6_ENABLED && addresses.ipv6[0] != '\0')
    {
        struct sockaddr_in6 ipv6_loopback{};
        ipv6_loopback.sin6_family = AF_INET6;
        ipv6_loopback.sin6_port = htons(NTP_PORT);
        if (inet_pton(AF_INET6, "::1", &ipv6_loopback.sin6_addr) != 1)
        {
            queue_startup_health_log(ESP_LOG_ERROR, "Health Check IPv6 loopback: unable to initialize address");
            return results;
        }
        results.ipv6_loopback = run_startup_health_endpoint_test("IPv6 loopback", reinterpret_cast<const struct sockaddr *>(&ipv6_loopback), sizeof(ipv6_loopback), AF_INET6);

        struct sockaddr_in6 ipv6_assigned{};
        ipv6_assigned.sin6_family = AF_INET6;
        ipv6_assigned.sin6_port = htons(NTP_PORT);
        ipv6_assigned.sin6_scope_id = s_startup_health_context.ipv6_scope_id();
        if (inet_pton(AF_INET6, addresses.ipv6, &ipv6_assigned.sin6_addr) == 1)
            results.ipv6_assigned = run_startup_health_endpoint_test("IPv6 assigned", reinterpret_cast<const struct sockaddr *>(&ipv6_assigned), sizeof(ipv6_assigned), AF_INET6);
        else
            queue_startup_health_log(ESP_LOG_ERROR, "Health Check IPv6 assigned: invalid address");
    }

    return results;
}

void startup_health_test_task(void *parameter)
{
    s_startup_health_test_log = static_cast<startup_health_log_entry_t *>(calloc(Startup_Health_Test_Log_Capacity, sizeof(startup_health_log_entry_t)));
    if (s_startup_health_test_log == nullptr)
    {
        ESP_LOGE(TAG, "Health Check could not allocate its log buffer");
        xTaskNotifyGive(static_cast<TaskHandle_t>(parameter));
        vTaskDelete(nullptr);
        return;
    }

    s_startup_health_test_results = run_startup_health_tests();
    const startup_health_mqtt_metrics_result_t mqtt_metrics_result = check_startup_health_mqtt_metrics(s_startup_health_test_results);
    ++s_startup_health_test_number;
    const char *mqtt_metrics_status = mqtt_metrics_result == startup_health_mqtt_metrics_result_t::passed
                                          ? "Passed"
                                      : mqtt_metrics_result == startup_health_mqtt_metrics_result_t::failed ? "Failed"
                                                                                                            : "Could not determine";
    queue_startup_health_log(mqtt_metrics_result == startup_health_mqtt_metrics_result_t::passed ? ESP_LOG_INFO : ESP_LOG_WARN,
                             "Health Check %02lu - MQTT metrics                         - %s", static_cast<unsigned long>(s_startup_health_test_number),
                             mqtt_metrics_status);
    queue_startup_health_log(ESP_LOG_INFO, "Health Check %s", s_startup_health_test_results.prerequisites_ready ? "completed" : "failed before testing");
    flush_startup_health_log();
    free(s_startup_health_test_log);
    s_startup_health_test_log = nullptr;
    xTaskNotifyGive(static_cast<TaskHandle_t>(parameter));
    vTaskDelete(nullptr);
}
#endif

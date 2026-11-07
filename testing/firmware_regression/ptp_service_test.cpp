#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "leap_fixture.h"
#include "../../main/custom/ptp_l2_auth.h"
#include "../../main/custom/ptp_l2_protocol.h"
#include "../../main/custom/ptp_followers.h"

#ifndef PTP_L2_AUTHENTICATION_ENABLED
#define PTP_L2_AUTHENTICATION_ENABLED 0
#endif
#if PTP_L2_AUTHENTICATION_ENABLED
#include "auth_settings.h"
#include "auth_crypto_fixture.h"
#endif
#include "esp_log.h"

#define DEBUG_ENABLED 0
static int64_t s_now_us = 1000000;
static int64_t esp_timer_get_time() { return s_now_us; }

using QueueHandle_t = void *;
using esp_eth_handle_t = void *;
using esp_err_t = int;
static constexpr int ESP_OK = 0, ESP_ERR_INVALID_STATE = 1;
static constexpr int PTP_L2_DOMAIN = 0, PTP_L2_LOG_SYNC_INTERVAL = 0, PTP_L2_LOG_ANNOUNCE_INTERVAL = 1;
static constexpr int PTP_L2_PRIORITY1 = 128, PTP_L2_PRIORITY2 = 128;
static constexpr int PTP_L2_FOLLOWER_TIMEOUT_SECONDS = 30, PTP_L2_FOLLOWER_CAPACITY = 32;
static constexpr int PTP_L2_FOREIGN_MASTER_CAPACITY = 8, PTP_L2_FOREIGN_ANNOUNCE_TIMEOUT_SECONDS = 10;
static const char *TAG = "ptp";
static constexpr unsigned pdMS_TO_TICKS(unsigned value) { return value; }
struct eth_mac_time_t { uint32_t seconds, nanoseconds; };
struct ntp_reply_status_t { uint8_t stratum = 1; bool holdover = false; };
static ntp_reply_status_t s_status;
static ntp_reply_status_t get_ntp_reply_status() { return s_status; }
static std::atomic<bool> s_ethernet_connected{true}, s_ptp_clock_ready{true};
static std::atomic<bool> s_ethernet_recovery_in_progress{false};
static std::atomic<bool> s_model_valid{true}, s_model_rate_valid{true}, s_ptp_comp_valid{true};
static std::atomic<uint32_t> s_ptp_clock_generation{2};
static esp_eth_handle_t s_eth_handle = reinterpret_cast<void *>(1);
static SemaphoreHandle_t s_hardware_ntp_transmit_mutex = reinterpret_cast<void *>(1);
static SemaphoreHandle_t s_time_mutex = reinterpret_cast<void *>(1);
static std::atomic<uint8_t> s_ptp_service_active{0};
static ptp::FollowerTable<32> s_ptp_followers{30000000};
static portMUX_TYPE s_ptp_followers_mux = portMUX_INITIALIZER_UNLOCKED;
static unsigned s_queue_sends = 0, s_transmits = 0;
static uint8_t s_last_transmit[128]{};
static size_t s_last_transmit_length = 0;
static bool s_queue_full = false;
static std::vector<int64_t> s_transmit_times;
static int xQueueReceive(QueueHandle_t, void *, unsigned);
static bool timestamp_is_valid(const eth_mac_time_t *timestamp)
{
    return timestamp != nullptr && timestamp->seconds != 0 && timestamp->nanoseconds < 1000000000U;
}
static bool ptp_write_timestamp(uint8_t *, const eth_mac_time_t &timestamp)
{
    return timestamp_is_valid(&timestamp);
}
static constexpr int CLOCK_PTP_SYSTEM = 123;
static int test_clock_gettime(int, struct timespec *value)
{
    value->tv_sec = 1700000000;
    value->tv_nsec = 10000;
    return 0;
}
#define clock_gettime test_clock_gettime
static int xQueueSend(QueueHandle_t, const void *, unsigned)
{
    ++s_queue_sends;
    return s_queue_full ? 0 : pdTRUE;
}
static esp_err_t esp_eth_transmit_ctrl_vargs(esp_eth_handle_t, eth_mac_time_t *timestamp, unsigned count,
                                            uint8_t *frame, uint32_t length)
{
    assert(count == 2 && length >= 60);
    ++s_transmits;
    std::memcpy(s_last_transmit, frame, length);
    s_last_transmit_length = length;
    s_transmit_times.push_back(esp_timer_get_time());
    if (timestamp != nullptr)
        *timestamp = {1700000000U, 1234};
    return ESP_OK;
}
#define TIMING_LOGW(...) ((void)0)
#include "ptp_service_under_test.h"

struct esp_eth_mac_t
{
    esp_err_t (*add_mac_filter)(esp_eth_mac_t *, uint8_t *);
};
static esp_eth_mac_t s_mac{[](esp_eth_mac_t *, uint8_t *) { return ESP_OK; }};
static constexpr int ETH_CMD_G_MAC_ADDR = 1, pdPASS = 1;
static constexpr unsigned PTP_L2_REQUEST_QUEUE_DEPTH = 8, PTP_L2_Service_Task_Stack_Size = 4096;
static unsigned s_task_creations = 0;
static esp_err_t esp_eth_ioctl(esp_eth_handle_t, int, uint8_t *mac)
{
    const uint8_t source[] = {2, 1, 2, 3, 4, 5};
    std::memcpy(mac, source, sizeof(source));
    return ESP_OK;
}
static esp_err_t esp_eth_get_mac_instance(esp_eth_handle_t, esp_eth_mac_t **mac)
{
    *mac = &s_mac;
    return ESP_OK;
}
static QueueHandle_t xQueueCreate(unsigned, size_t) { return reinterpret_cast<void *>(1); }
static void vQueueDelete(QueueHandle_t) {}
static int xTaskCreatePinnedToCore(void (*)(void *), const char *, unsigned, void *, unsigned, void *, unsigned)
{
    ++s_task_creations;
    return pdPASS;
}
#include "ptp_startup_under_test.h"

static bool signed_mode()
{
#if PTP_L2_AUTHENTICATION_ENABLED
    return ptp_l2_auth_available();
#else
    return false;
#endif
}

#if PTP_L2_AUTHENTICATION_ENABLED
static void append_test_authentication(uint8_t *message, size_t capacity, size_t &length)
{
    message[1] = 0x12;
    message[32] = 0;
    if (signed_mode())
    {
        assert(ptp_l2_auth_append(message, capacity, &length));
        return;
    }
    assert(capacity - length >= PTP_L2_AUTH_TLV_SIZE);
    uint8_t *tlv = message + length;
    std::memset(tlv, 0, PTP_L2_AUTH_TLV_SIZE);
    tlv[0] = 0x80; tlv[1] = 9; tlv[3] = 22;
    tlv[4] = PTP_L2_AUTHENTICATION_SPP; tlv[9] = 1;
    length += PTP_L2_AUTH_TLV_SIZE;
    ptp_write_u16(message + 2, static_cast<uint16_t>(length));
}
#endif

struct ServiceEvent
{
    int64_t time_us;
    ptp_l2_received_frame_t received{};
    bool deliver = false;
    bool recovery = false;
    bool holdover = false;
};
struct TaskComplete {};
static std::vector<ServiceEvent> s_events;
static size_t s_event_index = 0;

static int xQueueReceive(QueueHandle_t, void *output, unsigned)
{
    if (s_event_index == s_events.size())
        throw TaskComplete{};
    const auto &event = s_events[s_event_index++];
    s_now_us = event.time_us;
    s_status.holdover = event.holdover;
    if (event.recovery)
        s_ptp_clock_generation.fetch_add(2);
    if (!event.deliver)
        return 0;
    std::memcpy(output, &event.received, sizeof(event.received));
    return pdTRUE;
}

static ServiceEvent announce_event(int64_t now_us, uint16_t sequence, bool sign, bool corrupt = false, uint8_t priority = 127)
{
    ServiceEvent event{now_us};
    event.deliver = true;
    event.received.clock_generation = 2;
    ptp_l2_build_frame(event.received.data, 11, sequence, PTP_L2_ANNOUNCE_SIZE);
    event.received.data[6] = 2;
    uint8_t *message = event.received.data + 14;
    message[20] = 0x10;
    message[47] = priority;
    message[53] = 0x10;
    size_t length = PTP_L2_ANNOUNCE_SIZE;
#if PTP_L2_AUTHENTICATION_ENABLED
    if (sign)
        append_test_authentication(message, sizeof(event.received.data) - 14, length);
#else
    (void)sign;
#endif
    event.received.length = static_cast<uint8_t>(length + 14);
    if (corrupt)
        message[length - 1] ^= 1;
    return event;
}

static ServiceEvent delay_event(int64_t now_us, bool sign = signed_mode(), uint8_t version = 0)
{
    ServiceEvent event{now_us};
    event.deliver = true;
    event.received.clock_generation = 2;
    event.received.timestamp = {1700000000U, 1234};
    ptp_l2_build_frame(event.received.data, 1, 1, PTP_L2_EVENT_SIZE);
    event.received.data[6] = 2;
    uint8_t *message = event.received.data + 14;
    message[20] = 0x30;
    message[1] = version == 0 ? (signed_mode() ? 0x12 : 2) : version;
    message[32] = message[1] == 2 ? 1 : 0;
    size_t length = PTP_L2_EVENT_SIZE;
#if PTP_L2_AUTHENTICATION_ENABLED
    if (sign)
        append_test_authentication(message, sizeof(event.received.data) - 14, length);
#else
    (void)sign;
#endif
    event.received.length = static_cast<uint8_t>(std::max(size_t{60}, length + 14));
    return event;
}

static void run_service(std::vector<ServiceEvent> events)
{
    s_events = std::move(events);
    s_event_index = 0;
    s_now_us = 0;
    s_transmit_times.clear();
    s_ptp_clock_generation = 2;
    s_ptp_l2_tx_healthy = true;
    s_ptp_l2_rx_healthy = true;
    s_status = {};
    set_test_leap(0);
    try
    {
        ptp_l2_service_task(nullptr);
        assert(false);
    }
    catch (const TaskComplete &)
    {
        assert(s_event_index == s_events.size());
    }
}

static void test_service_election()
{
    const bool sign = signed_mode();
    s_ptp_identity[0] = 0x20;
    run_service({{4000000}, announce_event(5000000, 1, sign), announce_event(6000000, 2, sign), {7000000}});
    assert(!s_ptp_service_active);
    for (const int64_t sent_us : s_transmit_times)
        assert(sent_us < (PTP_L2_MASTER_SELECTION_ENABLED ? 6000000 : 5000000));
    const uint32_t old_responses = s_ptp_delay_responses.load();
    run_service({{4000000}, announce_event(5000000, 1, sign), announce_event(6000000, 2, sign),
                 delay_event(7000000), delay_event(17000000)});
    assert(s_ptp_service_active);
    assert(s_ptp_delay_responses.load() == old_responses + 1);
    run_service({{4000000}, announce_event(5000000, 1, sign, false, 129),
                 announce_event(6000000, 2, sign, false, 129), {7000000}});
    assert((s_ptp_service_active.load() != 0) == bool(PTP_L2_MASTER_SELECTION_ENABLED));

#if PTP_L2_AUTHENTICATION_ENABLED
    run_service({{4000000}, announce_event(5000000, 1, true, true), announce_event(6000000, 2, true, true), {7000000}});
    assert(s_ptp_service_active);
    auto invalid_request = delay_event(4500000, true);
    invalid_request.received.data[invalid_request.received.length - 1] ^= 1;
    const uint32_t rejected_responses = s_ptp_delay_responses.load();
    run_service({{4000000}, invalid_request});
    assert(s_ptp_service_active && s_ptp_delay_responses.load() == rejected_responses);
    run_service({{4000000}, announce_event(5000000, 1, false), announce_event(6000000, 2, false), {7000000}});
    assert((s_ptp_service_active.load() != 0) == bool(PTP_L2_AUTHENTICATION_REQUIRED));
#endif

    run_service({{4000000}, announce_event(5000000, 1, sign), announce_event(6000000, 2, sign),
                 {7000000, {}, false, true}, announce_event(8000000, 3, sign),
                 announce_event(10000000, 4, sign), {11000000}});
    assert((s_ptp_service_active.load() != 0) == bool(PTP_L2_MASTER_SELECTION_ENABLED));
    ServiceEvent holdover{4000000};
    holdover.holdover = true;
    run_service({holdover});
    assert(!s_ptp_service_active && s_transmit_times.empty());
    assert(s_status.stratum == 1 && s_status.holdover);
}

static void test_linuxptp_delay_exchange()
{
    for (const uint8_t version : {uint8_t{2}, uint8_t{0x12}})
    {
        for (const bool sign : {false, true})
        {
            if (sign && (!signed_mode() || version == 2))
                continue;
            const auto request = delay_event(4500000, sign, version);
            const uint32_t old_responses = s_ptp_delay_responses.load();
            run_service({{4000000}, request});
            const bool accepted = !PTP_L2_AUTHENTICATION_REQUIRED || sign;
            assert(s_ptp_service_active);
            assert(s_ptp_delay_responses.load() == old_responses + (accepted ? 1U : 0U));
            if (accepted)
            {
                const auto followers = s_ptp_followers.snapshot_and_reset(s_now_us);
                assert(followers.current == 1 && followers.client_count == 1);
                assert(std::memcmp(followers.clients[0].mac, request.received.data + 6, 6) == 0);
                assert(s_last_transmit[14] == 9);
                assert(s_last_transmit[44] == 0 && s_last_transmit[45] == 1);
                assert(std::memcmp(s_last_transmit + 58, request.received.data + 34, 10) == 0);
#if PTP_L2_AUTHENTICATION_ENABLED
                assert(ptp_l2_auth_verify(s_last_transmit + 14, s_last_transmit_length - 14) ==
                       (signed_mode() ? ptp_l2_auth_result_t::valid : ptp_l2_auth_result_t::unauthenticated));
#endif
            }
        }
    }
}

int main()
{
    set_test_leap(esp_timer_get_time());
#if PTP_L2_AUTHENTICATION_ENABLED
    if (!ptp_l2_auth_initialize())
    {
        assert(!start_ptp_l2_service() && !start_ptp_l2_service());
        assert(s_task_creations == 0 && !s_ptp_l2_worker_ready && s_ptp_l2_queue == nullptr);
        assert(!ptp_l2_auth_available());
        uint8_t rejected[PTP_L2_TX_FRAME_SIZE]{};
        ptp_l2_build_frame(rejected, 0, 1, PTP_L2_EVENT_SIZE);
        assert(!ptp_l2_transmit(rejected, PTP_L2_EVENT_SIZE, sizeof(rejected)) && s_transmits == 0);
        std::puts("PASS: failed authentication policy never publishes PTP or transmits unsigned");
        return 0;
    }
#endif
    assert(start_ptp_l2_service() && s_task_creations == 1 && s_ptp_l2_worker_ready);
    const unsigned startup_warnings = s_fixture_warning_logs;
    const bool authenticated = signed_mode();
    assert(ptp_time_qualified());
    s_status.holdover = true;
    assert(!ptp_time_qualified());
    s_status.holdover = false;
    s_status.stratum = 16;
    assert(!ptp_time_qualified());
    s_status.stratum = 1;
    s_model_rate_valid = false;
    assert(!ptp_time_qualified());
    s_model_rate_valid = true;
    s_ptp_comp_valid = false;
    assert(!ptp_time_qualified());
    s_ptp_comp_valid = true;
    uint8_t frame[PTP_L2_TX_FRAME_SIZE]{};
    ptp_l2_build_frame(frame, 0, 0x1234, PTP_L2_EVENT_SIZE);
    assert(frame[12] == 0x88 && frame[13] == 0xf7 && frame[14] == 0);
    assert(frame[20] == 2 && frame[44] == 0x12 && frame[45] == 0x34);
    ptp_l2_build_frame(frame, 8, 0x1234, PTP_L2_EVENT_SIZE);
    assert(frame[14] == 8 && frame[20] == 0 && frame[46] == (authenticated ? 0 : 2));
    ptp_l2_build_frame(frame, 11, 3, PTP_L2_ANNOUNCE_SIZE);
    assert(frame[14] == 11 && frame[17] == 64 && frame[47] == 1);
    assert(frame[21] == 0x0c && frame[58] == 0 && frame[59] == 37);
    for (const int8_t change : {int8_t{1}, int8_t{-1}})
    {
        const timescale::LeapSeconds leap{true, 37, change, 1893456000, esp_timer_get_time()};
        s_leap_state.publish(leap);
        s_test_tai_seconds = leap.event_tai() - 1;
        ptp_l2_build_frame(frame, 11, 3, PTP_L2_ANNOUNCE_SIZE);
        assert(frame[21] == (change > 0 ? 0x0d : 0x0e) && frame[59] == 37);
        ++s_test_tai_seconds;
        ptp_l2_build_frame(frame, 11, 3, PTP_L2_ANNOUNCE_SIZE);
        assert(frame[21] == 0x0c && frame[59] == 37 + change);
    }
    s_timescale_requalification_required = true;
    assert(!ptp_time_qualified());
    s_timescale_requalification_required = false;
    s_leap_state.publish({});
    assert(!ptp_time_qualified());
    for (const int64_t utc : {timescale::CONTINUOUS_UTC_START - 1, timescale::CONTINUOUS_UTC_START,
                             timescale::BULLETIN_EXPIRES_UTC + 86400})
    {
        timescale::LeapSeconds policy{};
        assert(timescale::select_leap_seconds(utc, esp_timer_get_time(), {}, {}, policy));
        s_leap_state.publish(policy);
        s_test_tai_seconds = static_cast<time_t>(utc + 37);
        assert(ptp_time_qualified());
        ptp_l2_build_frame(frame, 11, 3, PTP_L2_ANNOUNCE_SIZE);
        assert(frame[21] == 0x0c && frame[59] == 37);
    }
    set_test_leap(esp_timer_get_time());
    ptp_l2_build_frame(frame, 0, 0x1234, PTP_L2_EVENT_SIZE);
    assert(ptp_l2_transmit(frame, PTP_L2_EVENT_SIZE, sizeof(frame)));
    assert(s_transmits == 1);
#if PTP_L2_AUTHENTICATION_ENABLED
    if (authenticated)
    {
        assert(s_last_transmit_length == 14 + 44 + PTP_L2_AUTH_TLV_SIZE);
        assert(s_last_transmit[15] == 0x12);
        assert(ptp_l2_auth_verify(s_last_transmit + 14, s_last_transmit_length - 14) == ptp_l2_auth_result_t::valid);
    }
#endif
    if (!authenticated)
        assert(s_last_transmit_length == 60 && s_last_transmit[15] == 2);
    s_ethernet_recovery_in_progress = true;
    assert(!ptp_l2_transmit(frame, PTP_L2_EVENT_SIZE, sizeof(frame)));
    assert(s_transmits == 1 && !ptp_time_qualified());
    s_ethernet_recovery_in_progress = false;
    s_ptp_l2_queue = reinterpret_cast<void *>(1);
    s_ptp_l2_worker_ready = true;
    const eth_mac_time_t timestamp{1700000000U, 1234};
    assert(ptp_l2_receive_frame(frame, sizeof(frame), &timestamp));
    assert(s_queue_sends == 1);
    frame[13] = 0;
    assert(!ptp_l2_receive_frame(frame, sizeof(frame), &timestamp));
    assert(s_queue_sends == 1);
    frame[13] = 0xf7;
    s_ptp_clock_generation = 3;
    assert(ptp_l2_receive_frame(frame, sizeof(frame), &timestamp));
    assert(s_queue_sends == 1);
    s_ptp_clock_generation = 4;
    s_ethernet_recovery_in_progress = true;
    assert(ptp_l2_receive_frame(frame, sizeof(frame), &timestamp));
    assert(s_queue_sends == 1);
    s_ethernet_recovery_in_progress = false;
    s_queue_full = true;
    assert(ptp_l2_receive_frame(frame, sizeof(frame), &timestamp));
    assert(s_ptp_queue_drops == 1);
    assert(ptp_l2_receive_frame(frame, 14, &timestamp));
    assert(s_ptp_queue_drops == 2);
    s_queue_full = false;
    for (const auto &entry : {std::array<size_t, 2>{0, 44}, {8, 44}, {9, 54}, {11, 64}})
    {
        ptp_l2_build_frame(frame, static_cast<uint8_t>(entry[0]), 17, static_cast<uint16_t>(entry[1]));
        const uint8_t legacy_control = entry[0] == 0 ? 0 : entry[0] == 8 ? 2 : entry[0] == 9 ? 3 : 5;
        assert(frame[46] == (authenticated ? 0 : legacy_control));
        assert(frame[15] == (authenticated ? 0x12 : 2));
        assert(ptp_l2_transmit(frame, entry[1], sizeof(frame)));
#if PTP_L2_AUTHENTICATION_ENABLED
        if (authenticated)
            assert(ptp_l2_auth_verify(s_last_transmit + 14, s_last_transmit_length - 14) == ptp_l2_auth_result_t::valid);
#endif
        if (!authenticated)
        {
            assert(s_last_transmit_length == std::max(size_t{60}, entry[1] + 14));
            assert(s_last_transmit[16] == 0 && s_last_transmit[17] == entry[1]);
        }
    }
    for (const uint8_t type : {uint8_t{1}, uint8_t{11}})
    {
        const size_t body = type == 1 ? 44 : 64;
        ptp_l2_build_frame(frame, type, 33, static_cast<uint16_t>(body));
        frame[6] = 2;
        uint8_t *message = frame + 14;
        message[20] = 1;
        message[32] = type == 1 ? 1 : 5;
        if (type == 11)
        {
            message[48] = 248;
            message[53] = 2;
        }
        ptp::ParsedMessage parsed{};
        const auto expected = type == 1 ? ptp::MessageKind::DelayRequest : ptp::MessageKind::Announce;
        assert(ptp_l2_parse_received_frame(frame, std::max(size_t{60}, body + 14), parsed) ==
               (PTP_L2_AUTHENTICATION_REQUIRED ? ptp::MessageKind::Invalid : expected));
#if PTP_L2_AUTHENTICATION_ENABLED
        size_t signed_size = body;
        append_test_authentication(message, sizeof(frame) - 14, signed_size);
        assert(ptp_l2_parse_received_frame(frame, signed_size + 14, parsed) ==
               (authenticated ? expected : ptp::MessageKind::Invalid));
        message[34] ^= 1;
        assert(ptp_l2_parse_received_frame(frame, signed_size + 14, parsed) == ptp::MessageKind::Invalid);
#endif
    }
    assert(s_ptp_authentication_rejections ==
           (PTP_L2_AUTHENTICATION_ENABLED ? (PTP_L2_AUTHENTICATION_REQUIRED ? 2U : 0U) + (authenticated ? 2U : 4U) : 0U));
#if PTP_L2_AUTHENTICATION_ENABLED
    const unsigned transmits = s_transmits;
    s_fixture_crypto_failure = true;
    ptp_l2_build_frame(frame, 0, 1, PTP_L2_EVENT_SIZE);
    assert(ptp_l2_transmit(frame, PTP_L2_EVENT_SIZE, sizeof(frame)) == !authenticated);
    assert(s_transmits == transmits + (authenticated ? 0U : 1U));
    assert(ptp_l2_auth_available() == authenticated && ptp_l2_auth_initialize());
    s_fixture_crypto_failure = false;
#endif
    test_service_election();
    test_linuxptp_delay_exchange();
    assert(s_fixture_warning_logs == startup_warnings);
#if PTP_L2_AUTHENTICATION_ENABLED
    assert(s_fixture_init_calls == (authenticated ? 1U : 0U));
    assert(s_fixture_import_calls == (authenticated ? 2U : 0U));
#endif
    std::puts("PASS: PTP wire headers, PPS/holdover qualification, shared TX and bounded recovery-aware ingress");
}

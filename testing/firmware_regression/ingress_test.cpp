#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>

#define MQTT_ENABLED 1
using esp_err_t = int;
using esp_eth_handle_t = void *;
struct esp_netif_t {};
static esp_netif_t s_netif;
struct eth_mac_time_t { uint32_t seconds, nanoseconds; };
static constexpr int ESP_OK = 0, pdTRUE = 1, pdFALSE = 0;
static constexpr bool IPV4_ENABLED = true, IPV6_ENABLED = true;
static constexpr size_t UDP_HEADER_SIZE = 8, HARDWARE_NTP_REQUEST_BUFFER_SIZE = 200;
static uint32_t NTP_Requests_Per_Second_Load_Shedding_Threshold = 10000;
static std::atomic<int64_t> s_last_load_shedding_us{0}, s_last_ntp_input_us{0}, s_first_ntp_input_us{0};
static std::atomic<uint64_t> s_ntp_admission_deadline{0};
static std::atomic<uint32_t> s_ntp_input_count{0}, s_ethernet_rx_frames{0}, s_ntp_requests_this_second{0};
static std::atomic<bool> s_ntp_shed_during_period{false};
static std::atomic<bool> s_ntp_external_responses_enabled{true}, s_hardware_ntp_accepting{true};
static int64_t s_now_us = 1000000;
static int64_t esp_timer_get_time() { return s_now_us; }
static unsigned s_sleep_calls = 0;
[[maybe_unused]] static void vTaskDelay(uint32_t ticks)
{
    ++s_sleep_calls;
    s_now_us += static_cast<int64_t>(ticks) * 1000;
}
static int s_buffer_queue, s_request_queue;
static void *s_hardware_ntp_request_queue = &s_request_queue;
static void *s_hardware_ntp_request_buffer_queue = &s_buffer_queue;
static uint8_t s_request_buffer[HARDWARE_NTP_REQUEST_BUFFER_SIZE]{};
struct hardware_ntp_request_t
{
    esp_eth_handle_t handle;
    uint8_t *frame;
    uint32_t length;
    esp_netif_t *netif;
    eth_mac_time_t rx_timestamp;
};
static uint32_t s_received_freed = 0, s_enqueued = 0, s_forwarded = 0, s_pool_returns = 0;
static bool s_pool_available = true, s_queue_available = true;
static bool s_timed_worker = false;
static uint32_t s_pending = 0, s_completed = 0, s_peak_pending = 0, s_pool_pressure_drops = 0;
static int64_t s_next_completion_us = 0;
static void drain_worker(int64_t now_us)
{
    while (s_pending > 0 && s_next_completion_us <= now_us)
    {
        --s_pending;
        ++s_completed;
        s_next_completion_us += 100;
    }
}
static int xQueueReceive(void *queue, uint8_t **buffer, uint32_t wait)
{
    assert(queue == s_hardware_ntp_request_buffer_queue && wait == 0);
    if (s_timed_worker && s_pending == 64)
    {
        ++s_pool_pressure_drops;
        return pdFALSE;
    }
    *buffer = s_request_buffer;
    return s_pool_available ? pdTRUE : pdFALSE;
}
static int xQueueSend(void *queue, const hardware_ntp_request_t *request, uint32_t wait)
{
    assert(queue == s_hardware_ntp_request_queue && wait == 0);
    assert(request->frame == s_request_buffer);
    if (!s_queue_available)
        return pdFALSE;
    ++s_enqueued;
    if (s_timed_worker)
    {
        if (s_pending == 0)
            s_next_completion_us = s_now_us + 100;
        ++s_pending;
        if (s_pending > s_peak_pending)
            s_peak_pending = s_pending;
    }
    return pdTRUE;
}
static void release_hardware_ntp_request_buffer(uint8_t *buffer)
{
    assert(buffer == s_request_buffer);
    ++s_pool_returns;
}
static void release_received_frame(uint8_t *) { ++s_received_freed; }
static esp_err_t esp_netif_receive(esp_netif_t *, uint8_t *, uint32_t, void *)
{
    ++s_forwarded;
    return ESP_OK;
}
static bool is_ipv4_ntp_request(const uint8_t *frame, uint32_t, size_t *ip_offset, size_t *udp_offset)
{
    *ip_offset = 1;
    *udp_offset = 2;
    return frame[0] == 4;
}
static bool is_ipv6_ntp_request(const uint8_t *frame, uint32_t, size_t *ip_offset, size_t *udp_offset)
{
    *ip_offset = 1;
    *udp_offset = 2;
    return frame[0] == 6;
}
static bool hardware_ntp_frame_is_acceptable(const uint8_t *, bool, size_t, size_t) { return true; }
static bool timestamp_is_valid(const eth_mac_time_t *timestamp) { return timestamp != nullptr; }
#include "load_under_test.h"
#define free release_received_frame
#include "ingress_under_test.h"
#undef free

static void reset()
{
    s_ntp_admission_deadline = 0;
    s_ntp_input_count = s_ethernet_rx_frames = s_ntp_requests_this_second = 0;
    s_last_load_shedding_us = s_last_ntp_input_us = s_first_ntp_input_us = 0;
    s_ntp_shed_during_period = false;
    s_received_freed = s_enqueued = s_forwarded = s_pool_returns = s_sleep_calls = 0;
    s_pool_available = s_queue_available = true;
    s_timed_worker = false;
    s_pending = s_completed = s_peak_pending = s_pool_pressure_drops = 0;
    s_next_completion_us = 0;
}
int main()
{
    for (const uint8_t family : {uint8_t{4}, uint8_t{6}})
    {
        for (uint32_t offered : {9000U, 10000U, 11000U, 13000U, 50000U})
        {
            reset();
            uint8_t frame[80]{};
            frame[0] = family;
            frame[2 + UDP_HEADER_SIZE] = 0x23;
            eth_mac_time_t timestamp{1, 0};
            constexpr uint32_t seconds = 5;
            for (uint32_t i = 0; i < offered * seconds; ++i)
            {
                s_now_us = 1000000 + static_cast<int64_t>(i) * 1000000 / offered;
                const int64_t arrival_us = s_now_us;
                assert(ntp_ethernet_input(nullptr, frame, sizeof(frame), &s_netif, &timestamp) == ESP_OK);
                assert(s_sleep_calls == 0 && s_now_us == arrival_us);
            }
            const uint32_t expected = (offered < 10000 ? offered : 10000) * seconds +
                                      (offered > 10000 ? NTP_ADMISSION_BURST_REQUESTS - 1 : 0);
            assert(s_enqueued == expected);
            assert(s_ntp_input_count == offered * seconds && s_received_freed == offered * seconds);
            assert(s_ntp_shed_during_period == (offered > 10000));
            assert(ntp_load_shedding_active() == (offered > 10000));
            frame[0] = 0;
            assert(ntp_ethernet_input(nullptr, frame, sizeof(frame), &s_netif, &timestamp) == ESP_OK);
            assert(s_forwarded == 1 && s_sleep_calls == 0);
        }
    }
    uint8_t frame[80]{};
    frame[0] = 4;
    frame[2 + UDP_HEADER_SIZE] = 0x23;
    eth_mac_time_t timestamp{1, 0};
    reset();
    assert(ntp_ethernet_input(nullptr, frame, sizeof(frame), nullptr, &timestamp) == ESP_OK);
    assert(s_received_freed == 1 && s_forwarded == 0 && s_enqueued == 0 && s_ntp_input_count == 0);
    reset();
    s_pool_available = false;
    assert(ntp_ethernet_input(nullptr, frame, sizeof(frame), &s_netif, &timestamp) == ESP_OK);
    assert(s_received_freed == 1 && s_enqueued == 0 && s_sleep_calls == 0);
    assert(s_ntp_shed_during_period && ntp_load_shedding_active());
    reset();
    s_queue_available = false;
    assert(ntp_ethernet_input(nullptr, frame, sizeof(frame), &s_netif, &timestamp) == ESP_OK);
    assert(s_received_freed == 1 && s_enqueued == 0 && s_pool_returns == 1 && s_sleep_calls == 0);
    assert(s_ntp_shed_during_period && ntp_load_shedding_active());
    for (uint32_t offered : {10000U, 11000U, 13000U, 50000U})
    {
        reset();
        s_timed_worker = true;
        for (uint32_t cycle = 0; cycle < 5; ++cycle)
        {
            const int64_t start_us = 1000000 + static_cast<int64_t>(cycle) * 15000000;
            for (uint32_t i = 0; i < offered * 5; ++i)
            {
                s_now_us = start_us + static_cast<int64_t>(i) * 1000000 / offered;
                drain_worker(s_now_us);
                assert(ntp_ethernet_input(nullptr, frame, sizeof(frame), &s_netif, &timestamp) == ESP_OK);
            }
            drain_worker(start_us + 15000000);
            if (s_pool_pressure_drops != 0)
            {
                std::printf("Timed 10k/s worker at %u offered: peak=%u, pool drops=%u\n",
                            offered, s_peak_pending, s_pool_pressure_drops);
                std::fflush(stdout);
            }
            assert(s_pool_pressure_drops == 0 && s_peak_pending <= NTP_ADMISSION_BURST_REQUESTS);
            assert(s_completed >= 50000 * (cycle + 1) && s_pending == 0);
            assert(s_completed <= (50000 + NTP_ADMISSION_BURST_REQUESTS - 1) * (cycle + 1));
        }
        assert(s_sleep_calls == 0);
    }
    std::puts("PASS: bounded ingress at 9k/10k/11k/13k/50k, timed 10k/s worker with no pool saturation, control frames and full queues");
}

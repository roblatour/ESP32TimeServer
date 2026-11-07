#include <atomic>
#include <cassert>
#include <cstdint>
#include <cinttypes>
#include <cstdio>
#include <string>
#include <vector>

#define MQTT_ENABLED 1
#define CALCULATE_STACK_SIZES_ENABLED 0
#define pdMS_TO_TICKS(ms) (ms)
using EventBits_t = uint32_t;
static constexpr EventBits_t ETH_GOT_IP_BIT = 1;
#include "recovery_constants_under_test.h"
static std::atomic<uint32_t> s_ntp_input_count{0};
static std::atomic<uint32_t> s_ethernet_rx_frames{0};
static std::atomic<int64_t> s_last_ntp_response_us{0}, s_first_ntp_input_us{0}, s_last_ntp_input_us{0};
static std::atomic<int64_t> s_last_load_shedding_us{0}, s_last_physical_link_down_us{0};
static std::atomic<int64_t> s_last_hardware_ntp_response_us{0};
static std::atomic<bool> s_ethernet_connected{true}, s_ntp_external_responses_enabled{true};
static std::atomic<bool> s_ipv4_ever_acquired{true}, s_mqtt_has_connected{true}, s_mqtt_connected{false};
static std::atomic<bool> s_hw_tx_timestamp_recovery_requested{false};
enum class ntp_transport_health_t : uint8_t { link_down, no_ip, idle, healthy, stalled };
static std::atomic<ntp_transport_health_t> s_ntp_transport_health{ntp_transport_health_t::idle};
enum class ethernet_recovery_result_t : uint8_t
{
    recovered, retry_later, link_unavailable, no_ip, receive_unavailable, driver_failed, timestamp_failed
};
static int s_net_event_group = 0;
static void *s_hardware_ntp_request_queue = nullptr, *s_hardware_ntp_request_buffer_queue = nullptr;
static unsigned uxQueueMessagesWaiting(void *) { return 0; }
static EventBits_t s_bits = 0;
static int64_t s_now_us = 100000000;
static int64_t esp_timer_get_time() { return s_now_us; }
static EventBits_t xEventGroupGetBits(int) { return s_bits; }
static bool has_usable_network_address(EventBits_t bits) { return (bits & ETH_GOT_IP_BIT) != 0; }
enum class physical_link_status_t : uint8_t { up, down, unknown };
static physical_link_status_t s_physical_link = physical_link_status_t::up;
static physical_link_status_t physical_ethernet_link_status() { return s_physical_link; }
static unsigned s_iterations = 0;
static bool s_jump = false, s_disconnect = false, s_restore = false, s_active_input = false;
static bool s_rx_active = false, s_recovered_then_dead = false;
static bool s_active_replies = false, s_sustained_shedding = false, s_disconnect_during_recovery = false;
static int64_t s_restart_us = 0, s_recovery_duration_us = 0;
struct SimulationDone {};
struct Restart { std::string reason; };
static void controlled_restart(const char *reason)
{
    s_restart_us = s_now_us;
    throw Restart{reason};
}
static void vTaskDelay(uint32_t ms)
{
    if (++s_iterations > 300)
        throw SimulationDone{};
    s_now_us += s_jump ? 200000000 : static_cast<int64_t>(ms) * 1000;
    s_jump = false;
    if (s_disconnect)
    {
        s_last_physical_link_down_us = s_now_us;
        s_ethernet_connected = false;
        s_physical_link = physical_link_status_t::down;
        s_disconnect = false;
    }
    if (s_restore)
    {
        s_last_ntp_input_us = s_now_us;
        s_ntp_input_count.fetch_add(1);
        s_ethernet_rx_frames.fetch_add(1);
        s_last_hardware_ntp_response_us = s_now_us;
        s_bits = ETH_GOT_IP_BIT;
        s_mqtt_connected = true;
        s_restore = false;
    }
    if (s_active_input)
    {
        s_last_ntp_input_us = s_now_us;
        s_ntp_input_count.fetch_add(1);
    }
    if (s_rx_active)
        s_ethernet_rx_frames.fetch_add(1);
    if (s_active_replies)
    {
        s_last_ntp_response_us = s_now_us;
        s_last_hardware_ntp_response_us = s_now_us;
    }
    if (s_sustained_shedding)
        s_last_load_shedding_us = s_now_us;
}
static std::vector<int64_t> s_attempts;
static ethernet_recovery_result_t s_result = ethernet_recovery_result_t::no_ip;
static ethernet_recovery_result_t recover_ethernet_with_timeout()
{
    s_attempts.push_back(s_now_us);
    s_now_us += s_recovery_duration_us;
    if (s_disconnect_during_recovery)
    {
        s_physical_link = physical_link_status_t::down;
        s_ethernet_connected = false;
        s_last_physical_link_down_us = s_now_us;
    }
    if (s_result == ethernet_recovery_result_t::recovered)
        s_hw_tx_timestamp_recovery_requested = false;
    if (s_recovered_then_dead && s_attempts.size() > 1)
        return ethernet_recovery_result_t::receive_unavailable;
    return s_result;
}
template <typename... Args> static void log_message(Args...) {}
#define ESP_LOGW(...) log_message(__VA_ARGS__)
static const char *TAG = "test";
#include "transport_under_test.h"

static void reset()
{
    s_now_us = 100000000;
    s_iterations = 0;
    s_attempts.clear();
    s_bits = 0;
    s_ntp_input_count = 1;
    s_ethernet_rx_frames = 0;
    s_last_ntp_input_us = 60000000;
    s_first_ntp_input_us = 60000000;
    s_last_ntp_response_us = 60000000;
    s_last_load_shedding_us = 60000000;
    s_last_hardware_ntp_response_us = 60000000;
    s_last_physical_link_down_us = 0;
    s_ethernet_connected = true;
    s_ipv4_ever_acquired = true;
    s_mqtt_has_connected = true;
    s_ntp_external_responses_enabled = true;
    s_physical_link = physical_link_status_t::up;
    s_mqtt_connected = false;
    s_hw_tx_timestamp_recovery_requested = false;
    s_ntp_transport_health = ntp_transport_health_t::idle;
    s_result = ethernet_recovery_result_t::no_ip;
    s_jump = s_disconnect = s_restore = s_active_input = false;
    s_rx_active = s_recovered_then_dead = false;
    s_active_replies = s_sustained_shedding = s_disconnect_during_recovery = false;
    s_restart_us = s_recovery_duration_us = 0;
}
static bool run()
{
    try { ethernet_transport_recovery_task(nullptr); }
    catch (const Restart &restart)
    {
        assert(restart.reason == "ethernet_recovery_failed" ||
               restart.reason == "ethernet_recovery_timestamp_unverified");
        return true;
    }
    catch (const SimulationDone &) { return false; }
    assert(false);
    return false;
}
int main()
{
    for (const auto result : {ethernet_recovery_result_t::no_ip,
                              ethernet_recovery_result_t::receive_unavailable,
                              ethernet_recovery_result_t::retry_later,
                              ethernet_recovery_result_t::driver_failed,
                              ethernet_recovery_result_t::timestamp_failed})
    {
        reset();
        s_result = result;
        s_jump = true;
        s_recovery_duration_us = 20000000;
        assert(run());
        assert(s_attempts.size() == 1 && s_attempts[0] == 300000000);
        assert(s_restart_us == s_attempts[0] + s_recovery_duration_us);
    }
    reset();
    s_disconnect = true;
    assert(!run() && s_attempts.empty());
    reset();
    s_restore = true;
    assert(!run() && s_attempts.empty());
    reset();
    s_result = ethernet_recovery_result_t::recovered;
    s_bits = ETH_GOT_IP_BIT;
    s_last_ntp_input_us = s_now_us;
    s_hw_tx_timestamp_recovery_requested = true;
    s_active_input = true;
    assert(run() && s_attempts.size() == 1);
    reset();
    s_result = ethernet_recovery_result_t::timestamp_failed;
    assert(run() && s_attempts.size() == 1 && s_attempts[0] == 135000000);
    reset();
    s_ethernet_connected = false;
    s_physical_link = physical_link_status_t::down;
    assert(!run() && s_attempts.empty());
    reset();
    s_bits = ETH_GOT_IP_BIT;
    s_mqtt_connected = true;
    s_last_load_shedding_us = 0;
    assert(!run() && s_attempts.empty());
    reset();
    s_bits = ETH_GOT_IP_BIT;
    s_last_load_shedding_us = 0;
    s_rx_active = true;
    assert(!run() && s_attempts.empty());
    reset();
    s_bits = ETH_GOT_IP_BIT;
    s_mqtt_has_connected = false;
    s_last_hardware_ntp_response_us = s_last_ntp_input_us.load() + 1000;
    assert(run() && s_attempts.size() == 1);
    reset();
    s_ntp_input_count = 0;
    s_first_ntp_input_us = s_last_ntp_input_us = s_last_load_shedding_us = 0;
    assert(run() && s_attempts.size() == 1);
    reset();
    s_ethernet_connected = false;
    assert(run() && s_attempts.size() == 1);
    reset();
    s_physical_link = physical_link_status_t::unknown;
    assert(run() && s_attempts.size() == 1);
    reset();
    s_bits = ETH_GOT_IP_BIT;
    s_result = ethernet_recovery_result_t::recovered;
    s_recovered_then_dead = true;
    assert(run() && s_attempts.size() == 2);
    reset();
    s_disconnect_during_recovery = true;
    assert(!run() && s_attempts.size() == 1 && s_restart_us == 0);
    reset();
    s_result = ethernet_recovery_result_t::link_unavailable;
    s_disconnect_during_recovery = true;
    assert(!run() && s_attempts.size() == 1 && s_restart_us == 0);
    reset();
    s_bits = ETH_GOT_IP_BIT;
    s_last_ntp_input_us = s_last_ntp_response_us = s_now_us;
    s_active_input = s_rx_active = s_active_replies = s_sustained_shedding = true;
    assert(!run() && s_attempts.empty());
    std::puts("PASS: first failed attempt restarts immediately, no-MQTT/no-input detection, stale link, healthy shedding, cable loss");
}

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "leap_fixture.h"
#include "../../main/custom/ptp_l3_udp.h"
#include "../../main/custom/ptp_followers.h"

#define PTP_L2_SERVER_ENABLED 0
#define PTP_L3_UDP_MASTER_SELECTION_ENABLED 1
#define PTP_L3_UDP_FOREIGN_MASTER_CAPACITY 8
#define PTP_L3_UDP_FOREIGN_ANNOUNCE_TIMEOUT_SECONDS 10
#define PTP_L3_UDP_DOMAIN 0
#define PTP_L3_UDP_LOG_SYNC_INTERVAL 0
#define PTP_L3_UDP_LOG_ANNOUNCE_INTERVAL 1
#define PTP_L3_UDP_PRIORITY1 128
#define PTP_L3_UDP_PRIORITY2 128
#define PTP_L3_UDP_REQUEST_QUEUE_DEPTH 8
#define PTP_L3_UDP_FOLLOWER_TIMEOUT_SECONDS 30
#define PTP_L3_UDP_FOLLOWER_CAPACITY 32

using QueueHandle_t = void *;
using esp_eth_handle_t = void *;
using esp_err_t = int;
struct eth_mac_time_t { uint32_t seconds, nanoseconds; };
static constexpr int ESP_OK = 0, ESP_ERR_INVALID_STATE = 1, pdPASS = 1;
static constexpr int ETH_CMD_G_MAC_ADDR = 1, CLOCK_PTP_SYSTEM = 123;
static constexpr size_t PTP_L3_UDP_Service_Task_Stack_Size = 6144;
static constexpr bool IPV4_ENABLED = true, IPV6_ENABLED = true;
static const char *TAG = "test";
static SemaphoreHandle_t s_time_mutex = reinterpret_cast<void *>(1);
static SemaphoreHandle_t s_hardware_ntp_transmit_mutex = reinterpret_cast<void *>(1);
static esp_eth_handle_t s_eth_handle = reinterpret_cast<void *>(1);
static void *s_eth_netif = reinterpret_cast<void *>(1);
static int64_t s_now_us = 1000000;
static int64_t esp_timer_get_time() { return s_now_us; }
static unsigned pdMS_TO_TICKS(unsigned value) { return value; }
static std::atomic<bool> s_ethernet_connected{true}, s_ptp_clock_ready{true}, s_ethernet_recovery_in_progress{false};
static std::atomic<bool> s_model_valid{true}, s_model_rate_valid{true}, s_ptp_comp_valid{true};
static std::atomic<uint32_t> s_ptp_clock_generation{2};
static std::atomic<uint8_t> s_ptp_service_active{0};
static std::atomic<uint32_t> s_own_ipv4_address_be{0x0100000a};
static ptp::FollowerTable<96, 100> s_ptp_followers{30000000};
static portMUX_TYPE s_ptp_followers_mux = portMUX_INITIALIZER_UNLOCKED;
struct ntp_reply_status_t { uint8_t stratum = 1; bool holdover = false; };
static ntp_reply_status_t s_status;
static ntp_reply_status_t get_ntp_reply_status() { return s_status; }
static bool timestamp_is_valid(const eth_mac_time_t *value)
{
    return value != nullptr && value->seconds != 0 && value->nanoseconds < 1000000000;
}
static bool ptp_write_timestamp(uint8_t *output, const eth_mac_time_t &value)
{
    if (!timestamp_is_valid(&value))
        return false;
    output[5] = static_cast<uint8_t>(value.seconds);
    output[9] = static_cast<uint8_t>(value.nanoseconds);
    return true;
}
static bool s_bad_tx_timestamp = false, s_tx_failure = false;
static std::vector<std::vector<uint8_t>> s_frames;
static esp_err_t esp_eth_transmit_ctrl_vargs(esp_eth_handle_t, eth_mac_time_t *timestamp, unsigned,
                                            uint8_t *frame, uint32_t length)
{
    if (s_tx_failure)
        return ESP_ERR_INVALID_STATE;
    s_frames.emplace_back(frame, frame + length);
    if (timestamp != nullptr)
        *timestamp = s_bad_tx_timestamp ? eth_mac_time_t{} : eth_mac_time_t{1700000000, 1234};
    return ESP_OK;
}
static int test_clock_gettime(int, timespec *value) { *value = {1700000000, 10000}; return 0; }
#define clock_gettime test_clock_gettime
#define ESP_LOGE(...) ((void)TAG)
#define ESP_LOGI(...) ((void)TAG)
#define TIMING_LOGW(...) ((void)TAG)
#include "ptp_shared_under_test.h"

// Socket stubs check independent ports, IPv6-only binding and Ethernet-scoped membership.
static constexpr int AF_INET = 2, AF_INET6 = 10, SOCK_DGRAM = 2, IPPROTO_UDP = 17;
static constexpr int IPPROTO_IP = 0, IPPROTO_IPV6 = 41;
static constexpr int IP_ADD_MEMBERSHIP = 1, IPV6_V6ONLY = 2, IPV6_JOIN_GROUP = 3;
struct sockaddr { uint16_t family; };
struct sockaddr_in { uint16_t sin_family, sin_port; uint32_t sin_addr = 0; };
struct sockaddr_in6 { uint16_t sin6_family, sin6_port; uint32_t sin6_scope_id; };
struct ip_mreq { uint32_t imr_multiaddr, imr_interface; };
struct ipv6_mreq { uint8_t ipv6mr_multiaddr[16]; unsigned ipv6mr_interface; };
struct esp_ip6_addr_t { uint32_t addr[4]; };
static constexpr uint8_t LOCAL_MAC[6] = {2, 1, 2, 3, 4, 5};
static constexpr uint8_t LOCAL_IP6[16] = {0xfe, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
static unsigned s_socket_calls = 0, s_close_calls = 0, s_joins = 0, s_v6only = 0;
static int s_fail_option = 0;
static uint16_t htons(uint16_t value) { return static_cast<uint16_t>((value << 8) | (value >> 8)); }
static int socket(int family, int, int) { assert(family == AF_INET || family == AF_INET6); return static_cast<int>(++s_socket_calls); }
static int bind(int, sockaddr *local, size_t)
{
    const auto *v4 = reinterpret_cast<sockaddr_in *>(local);
    assert(htons(v4->sin_port) == 319 || htons(v4->sin_port) == 320);
    return 0;
}
static int setsockopt(int, int, int option, const void *value, size_t)
{
    if (option == s_fail_option)
        return -1;
    if (option == IPV6_V6ONLY) { assert(*static_cast<const int *>(value) == 1); ++s_v6only; }
    if (option == IP_ADD_MEMBERSHIP)
    {
        assert(std::memcmp(&static_cast<const ip_mreq *>(value)->imr_multiaddr, ptp_l3_udp::IPV4_GROUP, 4) == 0);
        ++s_joins;
    }
    if (option == IPV6_JOIN_GROUP)
    {
        assert(static_cast<const ipv6_mreq *>(value)->ipv6mr_interface == 3);
        ++s_joins;
    }
    return 0;
}
static int close(int) { ++s_close_calls; return 0; }
static int esp_netif_get_netif_impl_index(void *) { return 3; }
static esp_err_t esp_netif_get_ip6_linklocal(void *, esp_ip6_addr_t *address)
{
    memcpy(address->addr, LOCAL_IP6, 16);
    return ESP_OK;
}
static esp_err_t esp_eth_ioctl(esp_eth_handle_t, int, uint8_t *mac) { memcpy(mac, LOCAL_MAC, 6); return ESP_OK; }
static QueueHandle_t xQueueCreate(unsigned, size_t) { return reinterpret_cast<void *>(1); }
static void vQueueDelete(QueueHandle_t) {}
static int xTaskCreatePinnedToCore(void (*)(void *), const char *, size_t, void *, int, void *, int) { return pdPASS; }
static int xQueueReceive(QueueHandle_t, void *, unsigned);
static int xQueueSend(QueueHandle_t, const void *, unsigned);
#include "../../main/custom/ptp_l3_udp_service.h"

struct Event
{
    int64_t now;
    bool deliver = false;
    ptp_l3_udp_received_t received{};
    bool holdover = false, step = false, disconnect = false, bad_timestamp = false;
};
struct Complete {};
static std::vector<Event> s_events;
static size_t s_event_index = 0;
static ptp_l3_udp_received_t s_queued{};
static bool s_queue_full = false;
static int xQueueSend(QueueHandle_t, const void *value, unsigned)
{
    if (s_queue_full) return 0;
    memcpy(&s_queued, value, sizeof(s_queued));
    return pdTRUE;
}
static int xQueueReceive(QueueHandle_t, void *output, unsigned)
{
    if (s_event_index == s_events.size()) throw Complete{};
    const auto &event = s_events[s_event_index++];
    s_now_us = event.now;
    set_test_leap(s_now_us);
    s_status.holdover = event.holdover;
    s_ethernet_connected = !event.disconnect;
    s_bad_tx_timestamp = event.bad_timestamp;
    if (event.step) s_ptp_clock_generation.fetch_add(2);
    memcpy(output, &event.received, sizeof(event.received));
    return event.deliver ? pdTRUE : 0;
}
static void run_events(std::vector<Event> events)
{
    s_events = std::move(events);
    s_event_index = 0;
    try { ptp_l3_udp_service_task(nullptr); } catch (const Complete &) {}
}
static Event request(unsigned family, int64_t now, uint8_t type = 1)
{
    Event event{now, true};
    event.received.family = static_cast<uint8_t>(family);
    event.received.generation = s_ptp_clock_generation;
    event.received.network_generation = 1;
    event.received.length = type == 1 ? 44 : 64;
    event.received.timestamp = {1700000000, 1234};
    memcpy(event.received.mac, LOCAL_MAC, 6);
    ptp_l3_udp_build_message(event.received.message, type, 55, event.received.length, family);
    event.received.message[20] = 3;
    if (type == 1) event.received.message[32] = 1;
    else { event.received.message[47] = 127; event.received.message[53] = 3; }
    return event;
}

int main()
{
    set_test_leap(s_now_us);
    assert(start_ptp_l3_udp_service() && s_ptp_l3_udp_worker_ready);
    for (unsigned family = 0; family < 2; ++family)
    {
        const auto event = request(family, s_now_us);
        uint8_t frame[ptp_l3_udp::MAX_FRAME_SIZE]{};
        const uint8_t ip4[4] = {10, 0, 0, 1};
        const size_t length = ptp_l3_udp::encapsulate(frame, sizeof(frame), LOCAL_MAC,
            family == 0 ? ip4 : LOCAL_IP6, family != 0, event.received.message, 44);
        assert(length == (family == 0 ? 86U : 106U));
        if (family == 0)
        {
            const uint8_t expected[4] = {224, 0, 1, 129};
            assert(memcmp(frame + 30, expected, 4) == 0);
        }
        else
        {
            const uint8_t expected[16] = {0xff, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0x81};
            assert(memcmp(frame + 38, expected, 16) == 0);
        }
        ptp_l3_udp::Datagram datagram{};
        ptp::ParsedMessage parsed{};
        assert(ptp_l3_udp::decode(frame, length, 0, s_ptp_identity, datagram, parsed) == ptp::MessageKind::DelayRequest);
        assert(datagram.port == 319 && datagram.ipv6 == (family != 0) && parsed.sequence_id == 55);
        assert(ptp_l3_udp::decode(frame, length, 1, s_ptp_identity, datagram, parsed) == ptp::MessageKind::Ignored);
        assert(ptp_l3_udp_receive_frame(frame, static_cast<uint32_t>(length), &event.received.timestamp));
        assert(s_queued.family == family && s_queued.timestamp.nanoseconds == 1234);
        s_queue_full = true;
        const auto drops = s_ptp_queue_drops.load();
        assert(ptp_l3_udp_receive_frame(frame, static_cast<uint32_t>(length), &event.received.timestamp));
        assert(s_ptp_queue_drops == drops + 1);
        s_queue_full = false;
        for (size_t truncated = 0; truncated < length; ++truncated)
        {
            datagram = {};
            assert(ptp_l3_udp::decode(frame, truncated, 0, s_ptp_identity, datagram, parsed) != ptp::MessageKind::DelayRequest);
        }
        frame[length - 1] ^= 1;
        assert(ptp_l3_udp::decode(frame, length, 0, s_ptp_identity, datagram, parsed) == ptp::MessageKind::Invalid);
        frame[length - 1] ^= 1;
        assert(ptp_l3_udp::encapsulate(frame, length - 1, LOCAL_MAC, ip4, family != 0, event.received.message, 44) == 0);
    }
    auto v4 = request(0, 5100000), v6 = request(1, 5200000);
    Event holdover{5300000}; holdover.holdover = true;
    Event step{6000000}; step.step = true;
    auto stale = v4; stale.now = 6100000;
    auto announce = request(0, 10100000, 11);
    announce.received.generation = 4;
    announce.received.network_generation = 2;
    auto second = announce; second.now += 100000; second.received.message[31]++;
    Event broken{11000000}; broken.bad_timestamp = true;
    Event down{12000000}; down.disconnect = true;
    run_events({{1000000}, {5000000}, v4, v6, holdover, step, stale, {10000000},
                announce, second, broken, down});
    assert(s_socket_calls == 8 && s_joins == 8 && s_v6only == 4);
    unsigned responses = 0, followups = 0;
    for (const auto &frame : s_frames)
    {
        const bool ipv6 = frame[12] == 0x86;
        const size_t offset = ipv6 ? 62 : 42;
        const auto type = frame[offset] & 15;
        assert(type == 0 || type == 8 || type == 11 || type == 9);
        assert(frame[offset + 1] == 2);
        assert(ptp::detail::read_u16(frame.data() + offset + 28) == (ipv6 ? 3 : 2));
        assert(ptp::detail::read_u16(frame.data() + offset - 6) == (type == 0 ? 319 : 320));
        if (type == 0) assert(frame[offset + 43] == static_cast<uint8_t>(10000)); // approximate originTimestamp
        if (type == 8) { assert(frame[offset + 43] == static_cast<uint8_t>(1234)); ++followups; }
        if (type == 9)
        {
            assert(frame[offset + 43] == static_cast<uint8_t>(1234));
            assert(frame[offset + 33] == 0);
            assert(frame[offset + 44] == 3 && ptp::detail::read_u16(frame.data() + offset + 52) == (ipv6 ? 3 : 2));
            ++responses;
        }
    }
    assert(s_ptp_delay_responses == 2 && responses == 2 && followups == 4);
    assert(s_ptp_followers.snapshot_and_reset(s_now_us).max == 2);
    assert(s_ptp_followers.snapshot_and_reset(s_now_us).current == 0);
    assert(s_ptp_l3_udp_families[0].selection.role(10200000) == ptp::SelectionRole::Standby);
    assert(s_ptp_l3_udp_families[1].selection.role(10200000) == ptp::SelectionRole::Master);
    assert(s_ptp_l3_udp_timestamp_fault && s_ptp_tx_timestamp_misses > 0);
    assert(!s_ptp_service_active && s_close_calls == 8);
    s_ptp_service_active = 1;
    run_events({down});
    assert(s_ptp_service_active == 1); // UDP withdrawal must not clear the Layer 2 activity bit.
    ptp_l3_udp_family_t failure{};
    memcpy(failure.source, LOCAL_IP6, 16);
    s_fail_option = IPV6_JOIN_GROUP;
    assert(!ptp_l3_udp_open(1, failure) && failure.sockets[0] == -1 && failure.sockets[1] == -1);
    ptp::FollowerTable<3> followers{100};
    uint8_t identity[10] = {1};
    assert(followers.observe(0, identity, 0, LOCAL_MAC, 0, 10, 1));
    identity[0] = 2;
    assert(!followers.observe(0, identity, 1, LOCAL_MAC, 0, 10, 1));
    identity[0] = 1;
    assert(followers.observe(0, identity, 1, LOCAL_MAC, 1, 100, 1));
    auto snapshot = followers.snapshot_and_reset(2);
    assert(snapshot.current == 2 && snapshot.client_count == 1 && snapshot.overflow_drops == 1);
    followers.clear_transport(0);
    snapshot = followers.snapshot_and_reset(3);
    assert(snapshot.current == 1 && snapshot.clients[0].active_entire_period);
    assert(followers.snapshot_and_reset(101).current == 0);
    std::puts("PASS: UDP multicast wire checks, dual sockets, E2E hardware timestamps, PHC epoch isolation, election, holdover rejection and socket cleanup");
}

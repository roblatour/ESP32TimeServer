#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>

#define OTE_UPDATES_ENABLED 0
#define MQTT_ENABLED 0
static constexpr int PreferIPvX = TEST_NETWORK_MODE;
static constexpr int lcdColumns = TEST_LCD_COLUMNS;
static constexpr size_t IP_ADDRESS_TEXT_SIZE = 46;
using EventBits_t = uint32_t;
static constexpr EventBits_t ETH_CONNECTED_BIT = 1, ETH_GOT_IP_BIT = 2, ETH_GOT_ROUTABLE_IP6_BIT = 8;
enum class ipv6_address_priority_t : uint8_t { none, link_local, unique_local, global_unicast };
struct network_address_snapshot_t
{
    char selected[IP_ADDRESS_TEXT_SIZE], ipv4[16], ipv6[IP_ADDRESS_TEXT_SIZE];
    ipv6_address_priority_t ipv6_priority;
};
static network_address_snapshot_t s_addresses{"192.0.2.1", "192.0.2.1", "2001:db8::1",
                                              ipv6_address_priority_t::global_unicast};
[[maybe_unused]] static network_address_snapshot_t get_network_address_snapshot() { return s_addresses; }
static std::string s_lcd_line;
static void display_line(int row, const char *text)
{
    assert(row == 3 && std::strlen(text) <= lcdColumns);
    s_lcd_line = text;
}
using esp_err_t = int;
static constexpr int ESP_OK = 0, ESP_ERR_INVALID_ARG = -2, ESP_ERR_INVALID_STATE = -3;
static constexpr int ETH_CMD_G_MAC_ADDR = 1;
static int s_driver;
static void *s_eth_handle = &s_driver;
static int s_mac_result = ESP_OK;
static unsigned s_mac_reads = 0, s_errors = 0;
static const char *TAG = "test";
[[maybe_unused]] static const char *esp_err_to_name(int) { return "injected error"; }
template <typename... Args> static void log_error(Args...) { ++s_errors; }
template <typename... Args> static void log_info(Args...) {}
#define ESP_LOGE(...) log_error(__VA_ARGS__)
#define ESP_LOGI(...) log_info(__VA_ARGS__)
[[maybe_unused]] static esp_err_t esp_eth_ioctl(void *handle, int command, uint8_t *mac)
{
    assert(handle == &s_driver && command == ETH_CMD_G_MAC_ADDR);
    const uint8_t actual_mac[]{0x02, 0xAB, 0xCD, 0xEF, 0x12, 0x34};
    std::memcpy(mac, actual_mac, sizeof(actual_mac));
    ++s_mac_reads;
    return s_mac_result;
}
#if STARTUP_HEALTH_TEST_ENABLED
static unsigned s_health_starts = 0, s_ntp_tests = 0, s_ptp_tests = 0;
static void start_startup_health_check() { ++s_health_starts; }
#if PTP_L2_SERVER_ENABLED
static std::atomic<bool> s_ptp_l2_worker_ready{true}, s_ptp_clock_ready{true};
static void report_startup_health_ptp_readiness(bool ready)
{
    assert(ready);
    ++s_ptp_tests;
}
#endif
#if NTP_SERVER_ENABLED
static constexpr size_t Startup_Health_Test_Task_Stack_Size = 4096;
static constexpr int pdPASS = 1, pdTRUE = 1, tskNO_AFFINITY = -1;
static constexpr uint32_t portMAX_DELAY = UINT32_MAX;
static void startup_health_test_task(void *) {}
static void *xTaskGetCurrentTaskHandle() { return &s_driver; }
static int xTaskCreatePinnedToCore(void (*task)(void *), const char *, size_t, void *, int, void *, int)
{
    assert(task == startup_health_test_task);
    ++s_ntp_tests;
    return pdPASS;
}
static void ulTaskNotifyTake(int, uint32_t) {}
#endif
#endif

#include "network_modes_under_test.h"

#if !NTP_SERVER_ENABLED
using esp_eth_handle_t = void *;
struct esp_netif_t {};
struct eth_mac_time_t { uint32_t seconds, nanoseconds; };
static std::atomic<uint32_t> s_ethernet_rx_frames{0};
static unsigned s_ptp_frames = 0, s_released_frames = 0, s_forwarded_frames = 0;
static bool ptp_l2_receive_frame(const uint8_t *frame, uint32_t, const eth_mac_time_t *)
{
    if (frame[12] != 0x88 || frame[13] != 0xF7)
        return false;
    ++s_ptp_frames;
    return true;
}
static esp_err_t esp_netif_receive(esp_netif_t *, uint8_t *, uint32_t, void *)
{
    ++s_forwarded_frames;
    return ESP_OK;
}
static void release_frame(uint8_t *) { ++s_released_frames; }
#define free release_frame
#include "ingress_under_test.h"
#undef free
#endif

int main()
{
    assert(IPV4_ENABLED == (PreferIPvX != 2 && PreferIPvX != 7));
    assert(IPV6_ENABLED == (PreferIPvX != 1 && PreferIPvX != 7));
    assert(!has_usable_network_address(0));
    assert(has_usable_network_address(ETH_CONNECTED_BIT) == L2_ONLY);
    assert(has_usable_network_address(ETH_GOT_IP_BIT) == IPV4_ENABLED);
    assert(has_usable_network_address(ETH_GOT_ROUTABLE_IP6_BIT) == IPV6_ENABLED);

    s_eth_handle = nullptr;
    display_selected_ip_address(0);
    assert(s_mac_reads == 0 && s_errors == 0);
    if (L2_ONLY || PreferIPvX == 9)
        assert(s_lcd_line.empty());
    s_eth_handle = &s_driver;
#if PTP_L2_SERVER_ENABLED
    if (PreferIPvX != 9)
    {
        s_mac_result = -1;
        display_selected_ip_address(45);
        assert(s_mac_reads == 1 && s_errors == 1);
        if (L2_ONLY || PreferIPvX == 0)
            assert(s_lcd_line.empty());
        s_mac_result = ESP_OK;
    }
#endif
    display_selected_ip_address(0);
    const unsigned cached_mac_reads = s_mac_reads;
    if (L2_ONLY)
    {
        assert(s_mac_reads == 2);
        assert(s_lcd_line == "02:AB:CD:EF:12:34");
        s_eth_handle = nullptr;
        display_selected_ip_address(0);
        assert(s_lcd_line.empty() && s_mac_reads == cached_mac_reads);
        s_eth_handle = &s_driver;
    }
    else
    {
#if PTP_L2_SERVER_ENABLED
        assert(s_mac_reads == (PreferIPvX == 9 ? 0u : 2u));
#else
        assert(s_mac_reads == 0);
#endif
        if (PreferIPvX == 9)
            assert(s_lcd_line.empty());
        else if (PreferIPvX == 8)
            assert(s_lcd_line == "IPv4: OK  IPv6: GUA");
        else if (PreferIPvX == 0 || PreferIPvX == 1 || PreferIPvX == 4)
            assert(s_lcd_line == "192.0.2.1");
        else
        {
            display_selected_ip_address(45);
            assert(s_lcd_line == ":db8::1");
        }
    }
#if NTP_SERVER_ENABLED && PTP_L2_SERVER_ENABLED
    if (PreferIPvX == 0)
    {
        display_selected_ip_address(15);
        assert(s_lcd_line == "2001");
        display_selected_ip_address(30);
        assert(s_lcd_line == ":db8::1");
        display_selected_ip_address(45);
        assert(s_lcd_line == "02:AB:CD:EF:12:34");
        display_selected_ip_address(59);
        assert(s_lcd_line == "02:AB:CD:EF:12:34");
    }
#endif
    const unsigned reads_before_final_display = s_mac_reads;
    display_selected_ip_address(0);
#if PTP_L2_SERVER_ENABLED
    if (PreferIPvX != 9 && s_eth_handle != nullptr)
        assert(s_mac_reads == reads_before_final_display + 1);
    else
        assert(s_mac_reads == reads_before_final_display);
#else
    assert(s_mac_reads == reads_before_final_display);
#endif
    perform_health_check();
#if STARTUP_HEALTH_TEST_ENABLED
    assert(s_health_starts == 1);
    assert(s_ntp_tests == NTP_SERVER_ENABLED);
    assert(s_ptp_tests == PTP_L2_SERVER_ENABLED);
#endif
#if !NTP_SERVER_ENABLED
    uint8_t frame[64]{};
    assert(ntp_ethernet_input(s_eth_handle, frame, sizeof(frame), nullptr, nullptr) == ESP_OK);
    assert(s_released_frames == 1 && s_ptp_frames == 0 && s_forwarded_frames == 0);
    frame[12] = 0x88;
    frame[13] = 0xF7;
    assert(ntp_ethernet_input(s_eth_handle, frame, sizeof(frame), nullptr, nullptr) == ESP_OK);
    assert(s_released_frames == 2 && s_ptp_frames == 1 && s_forwarded_frames == 0);
    assert(s_ethernet_rx_frames == 2);
#endif
    std::printf("PASS: network mode %d, LCD width %d, health checks %d (NTP=%d, PTP=%d)\n",
                PreferIPvX, lcdColumns, STARTUP_HEALTH_TEST_ENABLED, NTP_SERVER_ENABLED, PTP_L2_SERVER_ENABLED);
}

#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#define MQTT_ENABLED 1
template <typename... Args>
static void log_message(Args...) {}
#define ESP_LOGI(...) log_message(__VA_ARGS__)
#define ESP_LOGW(...) log_message(__VA_ARGS__)
#define ESP_LOGE(...) log_message(__VA_ARGS__)
#define display_line(...) ((void)0)
#define IP2STR(...) 0
#define IPSTR "%d"
using esp_err_t = int;
using esp_event_base_t = const char *;
using esp_eth_handle_t = void *;
static const char *TAG = "test";
static const char *esp_err_to_name(int) { return "error"; }
static constexpr int ESP_OK = 0, ESP_FAIL = -1;
static constexpr int ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED = 2;
static constexpr int ESP_IPADDR_TYPE_V4 = 0, ESP_NETIF_DNS_MAIN = 0, ESP_NETIF_DNS_BACKUP = 1;
static constexpr int ETHERNET_EVENT_START = 0, ETHERNET_EVENT_CONNECTED = 1;
static constexpr int ETHERNET_EVENT_DISCONNECTED = 2, ETHERNET_EVENT_STOP = 3;
static constexpr int ESP_EVENT_ANY_ID = -1;
static const char *ETH_EVENT = "eth";
static constexpr int IP_EVENT_ETH_GOT_IP = 0, IP_EVENT_GOT_IP6 = 1, IP_EVENT_ETH_LOST_IP = 2;
static constexpr uint32_t ETH_CONNECTED_BIT = 1, ETH_GOT_IP_BIT = 2;
static constexpr uint32_t ETH_GOT_IP6_BIT = 4, ETH_GOT_ROUTABLE_IP6_BIT = 8;
struct esp_netif_t
{
};
struct esp_ip4_addr_t
{
    uint32_t addr;
};
struct esp_netif_ip_info_t
{
    esp_ip4_addr_t ip, netmask, gw;
};
struct esp_netif_dns_info_t
{
    struct
    {
        int type;
        struct
        {
            esp_ip4_addr_t ip4;
        } u_addr;
    } ip;
};
struct ip_event_got_ip_t
{
    esp_netif_t *esp_netif;
    esp_netif_ip_info_t ip_info;
};
struct ip_event_got_ip6_t
{
    esp_netif_t *esp_netif;
    struct
    {
        int ip;
    } ip6_info;
};
static esp_netif_t s_netif, s_other_netif;
static esp_netif_t *s_eth_netif = &s_netif;
static esp_eth_handle_t s_eth_handle = &s_netif;
static std::atomic<bool> s_ethernet_connected{false}, s_ethernet_recovery_in_progress{false};
static std::atomic<bool> s_mqtt_has_connected{true}, s_mqtt_connected{false}, s_ipv4_ever_acquired{false};
static std::atomic<int64_t> s_eth_link_connected_us{0}, s_mqtt_disconnected_since_us{0};
static std::atomic<int64_t> s_last_physical_link_down_us{0}, s_first_ntp_input_us{0}, s_last_ntp_response_us{0};
static bool IPV4_ENABLED = true, IPV6_ENABLED = true;
static bool L2_ONLY = false;
static int s_net_event_group = 0;
static uint32_t s_bits = 0;
static int s_ipv4_updates = 0, s_ipv6_updates = 0, s_ipv4_clears = 0, s_clears = 0;
static int s_mqtt_disconnects = 0, s_linklocal_requests = 0;
static bool s_routable = false, s_netif_up = false;
using event_handler_t = void (*)(void *, esp_event_base_t, int32_t, void *);
struct registered_ethernet_handler_t
{
    int32_t event;
    event_handler_t handler;
    void *argument;
};
static std::vector<registered_ethernet_handler_t> s_ethernet_handlers;
static void dispatch_ethernet_event(int32_t event, esp_eth_handle_t *handle)
{
    for (const auto &entry : s_ethernet_handlers)
        if (entry.event == event || entry.event == ESP_EVENT_ANY_ID)
            entry.handler(entry.argument, ETH_EVENT, event, handle);
}
static void netif_ethernet_event_handler(void *, esp_event_base_t, int32_t event, void *data)
{
    if (*static_cast<esp_eth_handle_t *>(data) != s_eth_handle)
        return;
    if (event == ETHERNET_EVENT_CONNECTED)
        s_netif_up = true;
    else if (event == ETHERNET_EVENT_DISCONNECTED || event == ETHERNET_EVENT_STOP)
        s_netif_up = false;
}
static int64_t esp_timer_get_time() { return 123456; }
static void xEventGroupSetBits(int, uint32_t bits) { s_bits |= bits; }
static void xEventGroupClearBits(int, uint32_t bits) { s_bits &= ~bits; }
static esp_err_t esp_netif_create_ip6_linklocal(esp_netif_t *netif)
{
    assert(netif != nullptr && netif == s_eth_netif && s_netif_up);
    ++s_linklocal_requests;
    return ESP_OK;
}
static void refresh_network_ipv6_addresses() {}
static void mqtt_note_ethernet_disconnected() { ++s_mqtt_disconnects; }
static void clear_network_addresses() { ++s_clears; }
static void clear_network_ipv4_address() { ++s_ipv4_clears; }
static void set_network_ipv4_address(esp_ip4_addr_t, esp_ip4_addr_t) { ++s_ipv4_updates; }
static bool set_network_ipv6_address(int)
{
    ++s_ipv6_updates;
    return s_routable;
}
static void display_selected_ip_address(int) {}
#if DEBUG_ENABLED
struct network_address_snapshot_t
{
    char ipv6[8];
};
static network_address_snapshot_t get_network_address_snapshot() { return {}; }
#endif
static const char *StaticIPAddress = "", *Gateway = "", *SubnetMask = "", *PrimaryDNS = "", *SecondaryDNS = "";
static std::vector<std::string> s_configuration_calls;
static uint32_t s_dns_addresses[2]{};
static int s_dhcp_result = ESP_OK;
static esp_err_t esp_netif_str_to_ip4(const char *text, esp_ip4_addr_t *address)
{
    if (std::strcmp(text, "invalid") == 0 || text[0] == '\0')
        return ESP_FAIL;
    address->addr = std::strcmp(text, "192.0.2.53") == 0 ? 53 : std::strcmp(text, "192.0.2.54") == 0 ? 54
                                                                                                     : 1;
    return ESP_OK;
}
static esp_err_t esp_netif_dhcpc_stop(esp_netif_t *)
{
    s_configuration_calls.emplace_back("stop_dhcp");
    return s_dhcp_result;
}
static esp_err_t esp_netif_set_ip_info(esp_netif_t *, const esp_netif_ip_info_t *)
{
    s_configuration_calls.emplace_back("set_ip");
    return ESP_OK;
}
static esp_err_t esp_netif_set_dns_info(esp_netif_t *, int which, const esp_netif_dns_info_t *info)
{
    assert(info->ip.type == ESP_IPADDR_TYPE_V4);
    if (info->ip.u_addr.ip4.addr == 0)
        return ESP_FAIL;
    s_configuration_calls.emplace_back(which == ESP_NETIF_DNS_MAIN ? "dns_main" : "dns_backup");
    s_dns_addresses[which] = info->ip.u_addr.ip4.addr;
    return ESP_OK;
}
#define ESP_ERROR_CHECK(result)                    \
    do                                             \
    {                                              \
        if ((result) != ESP_OK)                    \
            throw std::runtime_error("IDF error"); \
    } while (0)
struct esp_netif_config_t
{
};
using esp_eth_netif_glue_handle_t = void *;
#define ESP_NETIF_DEFAULT_ETH() \
    esp_netif_config_t {}
static constexpr int ESP_ERR_NO_MEM = -2;
static const char *HostName = "test", *IP_EVENT = "ip";
static std::vector<std::string> s_stack_calls;
static esp_err_t esp_netif_init()
{
    s_stack_calls.emplace_back("init");
    return ESP_OK;
}
static esp_netif_t *esp_netif_new(const esp_netif_config_t *)
{
    s_stack_calls.emplace_back("new");
    return &s_netif;
}
static esp_eth_netif_glue_handle_t esp_eth_new_netif_glue(esp_eth_handle_t)
{
    s_stack_calls.emplace_back("glue");
    s_ethernet_handlers.push_back({ESP_EVENT_ANY_ID, netif_ethernet_event_handler, nullptr});
    return &s_netif;
}
static esp_err_t esp_netif_attach(esp_netif_t *, esp_eth_netif_glue_handle_t)
{
    s_stack_calls.emplace_back("attach");
    return ESP_OK;
}
static esp_err_t esp_netif_set_hostname(esp_netif_t *, const char *)
{
    s_stack_calls.emplace_back("hostname");
    return ESP_OK;
}
static esp_err_t esp_event_handler_register(esp_event_base_t base, int32_t event,
                                            event_handler_t handler, void *argument)
{
    if (base == ETH_EVENT)
    {
        s_stack_calls.emplace_back("eth_handler");
        s_ethernet_handlers.push_back({event, handler, argument});
    }
    else
        s_stack_calls.emplace_back("ip_handler");
    return ESP_OK;
}
static bool s_driver_started = false;
static unsigned int active_mac_reports = 0;
static esp_err_t esp_eth_driver_install(const int *, esp_eth_handle_t *handle)
{
    s_driver_started = false;
    s_stack_calls.emplace_back("driver_install");
    *handle = &s_netif;
    return ESP_OK;
}
static esp_err_t esp_eth_start(esp_eth_handle_t handle)
{
    assert(handle == s_eth_handle);
    assert(s_ethernet_handlers.size() == (L2_ONLY ? 1u : 2u));
    s_stack_calls.emplace_back("driver_start");
    s_driver_started = true;
    return ESP_OK;
}
static esp_err_t log_active_ethernet_mac()
{
    assert(s_driver_started);
    ++active_mac_reports;
    return ESP_OK;
}
static esp_err_t ntp_ethernet_input() { return ESP_OK; }
static esp_err_t esp_eth_update_input_path_info(esp_eth_handle_t, esp_err_t (*input)(), void *netif)
{
    assert(input == ntp_ethernet_input && netif == nullptr);
    s_stack_calls.emplace_back("l2_input");
    return ESP_OK;
}
#include "network_under_test.h"

static void start_ethernet_driver()
{
    const int eth_config = 0;
#include "ethernet_startup_under_test.h"
}

int main()
{
    start_ethernet_driver();
    esp_eth_handle_t handle = s_eth_handle, other = &s_other_netif;
    dispatch_ethernet_event(ETHERNET_EVENT_CONNECTED, &other);
    assert(!s_ethernet_connected && s_bits == 0);
    dispatch_ethernet_event(ETHERNET_EVENT_CONNECTED, &handle);
    assert(s_ethernet_connected && s_bits == ETH_CONNECTED_BIT);
    assert(s_linklocal_requests == 1 && s_mqtt_disconnected_since_us == 123456);
    assert((s_stack_calls == std::vector<std::string>{
                                 "driver_install", "init", "new", "glue", "attach", "hostname",
                                 "ip_handler", "ip_handler", "ip_handler", "eth_handler", "driver_start"}));

    ip_event_got_ip_t ip{&s_other_netif, {}};
    ethernet_ip_event_handler(nullptr, nullptr, IP_EVENT_ETH_GOT_IP, &ip);
    assert(s_ipv4_updates == 0);
    ip.esp_netif = s_eth_netif;
    ethernet_ip_event_handler(nullptr, nullptr, IP_EVENT_ETH_GOT_IP, &ip);
    assert(s_ipv4_updates == 1 && s_ipv4_ever_acquired && (s_bits & ETH_GOT_IP_BIT));
    IPV4_ENABLED = false;
    ethernet_ip_event_handler(nullptr, nullptr, IP_EVENT_ETH_GOT_IP, &ip);
    assert(s_ipv4_updates == 1);
    IPV4_ENABLED = true;

    ip_event_got_ip6_t ip6{&s_other_netif, {}};
    ethernet_ip_event_handler(nullptr, nullptr, IP_EVENT_GOT_IP6, &ip6);
    assert(s_ipv6_updates == 0);
    ip6.esp_netif = s_eth_netif;
    ethernet_ip_event_handler(nullptr, nullptr, IP_EVENT_GOT_IP6, &ip6);
    assert(s_ipv6_updates == 1 && (s_bits & ETH_GOT_IP6_BIT) && !(s_bits & ETH_GOT_ROUTABLE_IP6_BIT));
    s_routable = true;
    ethernet_ip_event_handler(nullptr, nullptr, IP_EVENT_GOT_IP6, &ip6);
    assert(s_bits & ETH_GOT_ROUTABLE_IP6_BIT);
    IPV6_ENABLED = false;
    ethernet_ip_event_handler(nullptr, nullptr, IP_EVENT_GOT_IP6, &ip6);
    assert(s_ipv6_updates == 2);

    ethernet_ip_event_handler(nullptr, nullptr, IP_EVENT_ETH_LOST_IP, &ip);
    assert(!(s_bits & ETH_GOT_IP_BIT) && (s_bits & ETH_GOT_ROUTABLE_IP6_BIT) && s_ipv4_clears == 1);
    dispatch_ethernet_event(ETHERNET_EVENT_DISCONNECTED, &handle);
    assert(!s_ethernet_connected && s_bits == 0 && s_clears == 1 && s_mqtt_disconnects == 1);
    assert(s_last_physical_link_down_us == 123456);
    s_last_physical_link_down_us = 0;
    s_ethernet_recovery_in_progress = true;
    dispatch_ethernet_event(ETHERNET_EVENT_DISCONNECTED, &handle);
    assert(s_last_physical_link_down_us == 0);
    dispatch_ethernet_event(ETHERNET_EVENT_CONNECTED, &handle);
    assert(s_linklocal_requests == 1 && s_ethernet_connected);
    IPV6_ENABLED = true;
    dispatch_ethernet_event(ETHERNET_EVENT_DISCONNECTED, &handle);
    dispatch_ethernet_event(ETHERNET_EVENT_CONNECTED, &handle);
    assert(s_netif_up && s_linklocal_requests == 2);
    dispatch_ethernet_event(ETHERNET_EVENT_STOP, &handle);
    assert(!s_ethernet_connected && s_bits == 0);

    assert(!configure_static_ip() && s_configuration_calls.empty());
    StaticIPAddress = "invalid";
    assert(!configure_static_ip() && s_configuration_calls.empty());
    StaticIPAddress = "192.0.2.4";
    Gateway = "192.0.2.1";
    SubnetMask = "255.255.255.0";
    PrimaryDNS = "invalid";
    assert(!configure_static_ip() && s_configuration_calls.empty());
    PrimaryDNS = "";
    SecondaryDNS = "invalid";
    assert(!configure_static_ip() && s_configuration_calls.empty());
    SecondaryDNS = "";
    assert(configure_static_ip());
    assert((s_configuration_calls == std::vector<std::string>{"stop_dhcp", "set_ip"}));
    for (const bool primary : {false, true})
    {
        for (const bool secondary : {false, true})
        {
            s_configuration_calls.clear();
            s_dns_addresses[0] = s_dns_addresses[1] = 0;
            PrimaryDNS = primary ? "192.0.2.53" : "";
            SecondaryDNS = secondary ? "192.0.2.54" : "";
            assert(configure_static_ip());
            std::vector<std::string> expected{"stop_dhcp", "set_ip"};
            if (primary)
                expected.emplace_back("dns_main");
            if (secondary)
                expected.emplace_back("dns_backup");
            assert(s_configuration_calls == expected);
            assert(s_dns_addresses[0] == (primary ? 53u : 0u));
            assert(s_dns_addresses[1] == (secondary ? 54u : 0u));
        }
    }
    const std::vector<std::string> expected{"stop_dhcp", "set_ip", "dns_main", "dns_backup"};
    s_configuration_calls.clear();
    s_dhcp_result = ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED;
    assert(configure_static_ip() && s_configuration_calls == expected);
    s_dhcp_result = ESP_FAIL;
    try
    {
        configure_static_ip();
        assert(false);
    }
    catch (const std::runtime_error &)
    {
    }

    s_dhcp_result = ESP_OK;
    s_configuration_calls.clear();
    s_stack_calls.clear();
    s_ethernet_handlers.clear();
    s_eth_netif = nullptr;
    start_ethernet_driver();
    assert(s_eth_netif == &s_netif);
    assert((s_stack_calls == std::vector<std::string>{
                                 "driver_install", "init", "new", "glue", "attach", "hostname",
                                 "ip_handler", "ip_handler", "ip_handler", "eth_handler", "driver_start"}));
    assert(s_configuration_calls == expected);
    dispatch_ethernet_event(ETHERNET_EVENT_CONNECTED, &handle);
    assert(s_netif_up && s_linklocal_requests == 3);
    dispatch_ethernet_event(ETHERNET_EVENT_DISCONNECTED, &handle);

    L2_ONLY = true;
    IPV4_ENABLED = IPV6_ENABLED = false;
    s_stack_calls.clear();
    s_ethernet_handlers.clear();
    s_configuration_calls.clear();
    s_eth_netif = nullptr;
    start_ethernet_driver();
    assert(s_eth_netif == nullptr);
    assert((s_stack_calls == std::vector<std::string>{
                                 "driver_install", "l2_input", "eth_handler", "driver_start"}));
    assert(s_configuration_calls.empty());
    s_linklocal_requests = 0;
    dispatch_ethernet_event(ETHERNET_EVENT_CONNECTED, &handle);
    assert(s_ethernet_connected && s_bits == ETH_CONNECTED_BIT && s_linklocal_requests == 0);
    dispatch_ethernet_event(ETHERNET_EVENT_DISCONNECTED, &handle);
    assert(!s_ethernet_connected && s_bits == 0);
    dispatch_ethernet_event(ETHERNET_EVENT_CONNECTED, &handle);
    assert(s_bits == ETH_CONNECTED_BIT && s_linklocal_requests == 0 && s_configuration_calls.empty());
    std::puts("PASS: native Ethernet events, static-IP ordering and optional primary/secondary DNS combinations");
    std::puts("PASS: IPv6 link-local creation follows netif link-up with DHCP, static IPv4 and reconnection");
    std::puts("PASS: L2 attaches only raw Ethernet input; no IP stack, DHCP, static IP, DNS or IPv6 link-local");
}

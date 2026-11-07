#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>

#define PTP_FREE_RUNNING 0
#define CALCULATE_STACK_SIZES_ENABLED 0
#define pdMS_TO_TICKS(ms) (ms)
using esp_err_t = int;
using esp_eth_handle_t = void *;
using TaskHandle_t = void *;
using EventBits_t = uint32_t;
static constexpr int ESP_OK = 0, ESP_FAIL = -1, pdTRUE = 1, pdFALSE = 0, pdPASS = 1;
static constexpr int ETH_CMD_READ_PHY_REG = 0, CLOCK_PTP_SYSTEM = 0, eSetValueWithOverwrite = 0;
static constexpr EventBits_t ETH_CONNECTED_BIT = 1, ETH_GOT_IP_BIT = 2, ETH_GOT_IP6_BIT = 4, ETH_GOT_ROUTABLE_IP6_BIT = 8;
#include "recovery_constants_under_test.h"
static constexpr size_t Ethernet_Transport_Recovery_Task_Stack_Size = 2409;
static constexpr const char *TAG = "test";
static int PreferIPvX = 0;
static int s_handle = 0;
static esp_eth_handle_t s_eth_handle = &s_handle;
static void *s_eth_netif = &s_handle;
static void *s_hardware_ntp_transmit_mutex = &s_handle, *s_time_mutex = &s_handle;
static int s_net_event_group = 3;
static std::atomic<bool> s_ethernet_connected{true}, s_ethernet_recovery_in_progress{false};
static std::atomic<bool> s_hardware_ntp_accepting{true}, s_ptp_clock_ready{true}, s_ptp_comp_valid{true};
static std::atomic<bool> s_ipv4_ever_acquired{true};
static std::atomic<uint32_t> s_ethernet_rx_frames{0};
static int64_t s_now_us = 100000000;
static EventBits_t s_bits = ETH_CONNECTED_BIT | ETH_GOT_IP_BIT;
static bool s_started = true, s_deliver_rx = true, s_old_rx_only = false, s_lock_available = true;
static bool s_create_worker = true, s_worker_hung = false, s_notify_pending = false;
static uint32_t s_notification = 0, s_requested_wait_ms = 0;
static esp_err_t s_stop_result = ESP_OK, s_start_result = ESP_OK, s_ptp_result = ESP_OK, s_phy_result = ESP_OK;
static uint32_t s_phy_status = 4, s_phy_reads = 0;
static unsigned int s_active_mac_reports = 0;
static esp_err_t s_active_mac_report_result = ESP_OK;
static EventBits_t s_start_bits = ETH_CONNECTED_BIT | ETH_GOT_IP_BIT;
template <typename... Args> static void log_message(Args...) {}
#define ESP_LOGE(...) log_message(__VA_ARGS__)
static esp_err_t log_active_ethernet_mac()
{
    ++s_active_mac_reports;
    return s_active_mac_report_result;
}
struct esp_eth_phy_reg_rw_data_t { uint32_t reg_addr; uint32_t *reg_value_p; };
static esp_err_t esp_eth_ioctl(esp_eth_handle_t, int command, esp_eth_phy_reg_rw_data_t *reg)
{
    assert(command == ETH_CMD_READ_PHY_REG && reg->reg_addr == 1);
    ++s_phy_reads;
    *reg->reg_value_p = s_phy_status;
    return s_phy_result;
}
static int64_t esp_timer_get_time() { return s_now_us; }
static int xSemaphoreTake(void *, uint32_t) { return s_lock_available ? pdTRUE : pdFALSE; }
static void xSemaphoreGive(void *) {}
static void xEventGroupClearBits(int, EventBits_t bits) { s_bits &= ~bits; }
static EventBits_t xEventGroupGetBits(int) { return s_bits; }
static void vTaskDelay(uint32_t ms)
{
    s_now_us += static_cast<int64_t>(ms) * 1000;
    if (s_started && s_deliver_rx)
        s_ethernet_rx_frames.fetch_add(1);
}
static EventBits_t xEventGroupWaitBits(int, EventBits_t bits, int, int, uint32_t ms)
{
    if ((s_bits & bits) == 0)
        vTaskDelay(ms);
    return s_bits;
}
static esp_err_t esp_eth_stop(esp_eth_handle_t)
{
    s_started = false;
    s_ethernet_connected = false;
    if (s_old_rx_only)
        s_ethernet_rx_frames.fetch_add(1);
    return s_stop_result;
}
static esp_err_t esp_eth_start(esp_eth_handle_t)
{
    s_started = true;
    s_bits = s_start_bits;
    s_ethernet_connected = (s_bits & ETH_CONNECTED_BIT) != 0;
    return s_start_result;
}
struct esp_eth_mac_t {};
struct eth_mac_ptp_config_t {};
struct eth_mac_time_t { uint32_t seconds, nanoseconds; };
#define ETH_MAC_ESP_PTP_DEFAULT_CONFIG() eth_mac_ptp_config_t{}
static esp_eth_mac_t s_mac;
static esp_err_t esp_eth_get_mac_instance(esp_eth_handle_t, esp_eth_mac_t **mac) { *mac = &s_mac; return ESP_OK; }
static esp_err_t esp_eth_mac_ptp_enable(esp_eth_mac_t *, const eth_mac_ptp_config_t *) { return s_ptp_result; }
static esp_err_t esp_eth_mac_enable_ts4all(esp_eth_mac_t *, bool) { return ESP_OK; }
static esp_err_t ntp_ethernet_input() { return ESP_OK; }
static esp_err_t esp_eth_update_input_path_info(esp_eth_handle_t, esp_err_t (*)(), void *) { return ESP_OK; }
struct timeval { long tv_sec, tv_usec; };
static void gettimeofday(timeval *time, void *) { time->tv_sec = 1700000000; time->tv_usec = 0; }
static int clock_settime(int, const timespec *) { return 0; }
static esp_err_t esp_eth_mac_get_ptp_time(esp_eth_mac_t *, eth_mac_time_t *time)
{
    time->seconds = static_cast<uint32_t>(s_now_us / 1000000);
    time->nanoseconds = static_cast<uint32_t>(s_now_us % 1000000) * 1000;
    return ESP_OK;
}
static void record_hw_tx_timestamp_result(bool valid) { assert(valid); }
struct Restart { std::string reason; };
static void controlled_restart(const char *reason) { throw Restart{reason}; }
struct TaskDeleted {};
static void vTaskDelete(void *) { throw TaskDeleted{}; }
static TaskHandle_t xTaskGetCurrentTaskHandle() { return &s_handle; }
static int xTaskNotify(TaskHandle_t task, uint32_t value, int)
{
    assert(task == &s_handle);
    s_notification = value;
    s_notify_pending = true;
    return pdTRUE;
}
static int xTaskNotifyWait(uint32_t, uint32_t, uint32_t *result, uint32_t ms)
{
    if (ms > 0)
        s_requested_wait_ms = ms;
    if (!s_notify_pending)
        return pdFALSE;
    *result = s_notification;
    s_notify_pending = false;
    return pdTRUE;
}
static int xTaskCreatePinnedToCore(void (*task)(void *), const char *, size_t stack, void *parameter, int priority, void *, int core)
{
    assert(stack == Ethernet_Transport_Recovery_Task_Stack_Size);
    assert(priority == 16 && core == 0);
    if (!s_create_worker)
        return pdFALSE;
    if (!s_worker_hung)
    {
        try { task(parameter); }
        catch (const TaskDeleted &) {}
    }
    return pdPASS;
}
#include "recovery_under_test.h"

static void reset()
{
    s_now_us = 100000000;
    s_bits = s_start_bits = ETH_CONNECTED_BIT | ETH_GOT_IP_BIT;
    s_ethernet_rx_frames = 0;
    s_ethernet_connected = s_hardware_ntp_accepting = s_started = s_deliver_rx = true;
    s_ethernet_recovery_in_progress = false;
    s_old_rx_only = s_worker_hung = s_notify_pending = false;
    s_lock_available = s_create_worker = true;
    s_stop_result = s_start_result = s_ptp_result = s_phy_result = ESP_OK;
    s_phy_status = 4;
    s_phy_reads = 0;
    s_active_mac_reports = 0;
    s_active_mac_report_result = ESP_OK;
    PreferIPvX = 0;
}
int main()
{
    reset();
    assert(physical_ethernet_link_status() == physical_link_status_t::up && s_phy_reads == 2);
    s_ethernet_connected = false;
    assert(physical_ethernet_link_status() == physical_link_status_t::up);
    s_phy_status = 0x7809;
    assert(physical_ethernet_link_status() == physical_link_status_t::down);
    s_phy_result = ESP_FAIL;
    assert(physical_ethernet_link_status() == physical_link_status_t::unknown);
    s_phy_result = ESP_OK;
    s_phy_status = 0xFFFF;
    assert(physical_ethernet_link_status() == physical_link_status_t::unknown);
    s_phy_status = 0;
    assert(physical_ethernet_link_status() == physical_link_status_t::unknown);
    reset();
    assert(restart_ethernet_transport() == ethernet_recovery_result_t::recovered);
    assert(s_active_mac_reports == 1);
    assert(!s_ethernet_recovery_in_progress && s_hardware_ntp_accepting);
    reset();
    s_active_mac_report_result = ESP_FAIL;
    assert(restart_ethernet_transport() == ethernet_recovery_result_t::driver_failed);
    assert(s_active_mac_reports == 1 && !s_ethernet_recovery_in_progress);
    reset();
    s_deliver_rx = false;
    s_old_rx_only = true;
    assert(restart_ethernet_transport() == ethernet_recovery_result_t::receive_unavailable);
    reset();
    s_start_bits = ETH_CONNECTED_BIT | ETH_GOT_IP6_BIT;
    assert(restart_ethernet_transport() == ethernet_recovery_result_t::no_ip);
    reset();
    PreferIPvX = 2;
    s_start_bits = ETH_CONNECTED_BIT | ETH_GOT_ROUTABLE_IP6_BIT;
    assert(restart_ethernet_transport() == ethernet_recovery_result_t::recovered);
    reset();
    s_start_bits = 0;
    assert(restart_ethernet_transport() == ethernet_recovery_result_t::driver_failed);
    reset();
    s_start_bits = 0;
    s_phy_status = 0x7809;
    assert(restart_ethernet_transport() == ethernet_recovery_result_t::link_unavailable);
    reset();
    s_stop_result = ESP_FAIL;
    assert(restart_ethernet_transport() == ethernet_recovery_result_t::driver_failed);
    reset();
    s_ptp_result = ESP_FAIL;
    assert(restart_ethernet_transport() == ethernet_recovery_result_t::timestamp_failed);
    reset();
    s_lock_available = false;
    assert(restart_ethernet_transport() == ethernet_recovery_result_t::retry_later);
    reset();
    assert(recover_ethernet_with_timeout() == ethernet_recovery_result_t::recovered);
    assert(s_requested_wait_ms == ETHERNET_RECOVERY_TIMEOUT_MS);
    reset();
    s_create_worker = false;
    assert(recover_ethernet_with_timeout() == ethernet_recovery_result_t::driver_failed);
    reset();
    s_worker_hung = true;
    try { (void)recover_ethernet_with_timeout(); assert(false); }
    catch (const Restart &restart) { assert(restart.reason == "ethernet_recovery_timeout"); }
    assert(s_requested_wait_ms == 45000);
    std::puts("PASS: real RX required after restart; IPv6 link-local rejected; independent PHY checks; 45-second recovery supervision");
}

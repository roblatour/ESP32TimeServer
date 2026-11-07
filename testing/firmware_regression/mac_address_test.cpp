#include <cassert>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

using esp_err_t = int;
using esp_eth_handle_t = void *;

static constexpr esp_err_t ESP_OK = 0;
static constexpr esp_err_t ESP_FAIL = -1;
static constexpr esp_err_t ESP_ERR_INVALID_ARG = -2;
static constexpr esp_err_t ESP_ERR_INVALID_STATE = -3;
static constexpr int ETH_CMD_G_MAC_ADDR = 1;
static constexpr int ESP_MAC_ETH = 2;
static constexpr const char *TAG = "mac_test";

#define DEBUG_ENABLED 1
#define MACAddress s_mac_address_setting

static const char *s_mac_address_setting = "";
static esp_eth_handle_t s_eth_handle = reinterpret_cast<void *>(1);
static uint8_t s_mock_active_mac[6] = {0x1e, 0x29, 0x04, 0xd0, 0x3e, 0x17};
static uint8_t s_mock_configured_eth_mac[6]{};
static esp_err_t s_ioctl_result = ESP_OK;
static esp_err_t s_eth_set_result = ESP_OK;
static unsigned int s_ioctl_calls = 0;
static unsigned int s_eth_set_calls = 0;
static unsigned int s_info_logs = 0;
static unsigned int s_error_logs = 0;
static char s_last_info[128]{};
static char s_last_error[128]{};

static void record_log(char level, const char *format, ...)
{
    char *destination = level == 'I' ? s_last_info : s_last_error;
    const size_t capacity = level == 'I' ? sizeof(s_last_info) : sizeof(s_last_error);
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(destination, capacity, format, arguments);
    va_end(arguments);

    if (level == 'I')
        ++s_info_logs;
    else
        ++s_error_logs;
}

#define ESP_LOGI(tag, ...) record_log('I', __VA_ARGS__)
#define ESP_LOGE(tag, ...) record_log('E', __VA_ARGS__)

static const char *esp_err_to_name(esp_err_t result)
{
    return result == ESP_FAIL ? "ESP_FAIL" : "ESP_ERR_TEST";
}

static esp_err_t esp_eth_ioctl(esp_eth_handle_t handle, int command, void *output)
{
    ++s_ioctl_calls;
    if (handle != reinterpret_cast<void *>(1) || command != ETH_CMD_G_MAC_ADDR || output == nullptr)
        return ESP_ERR_INVALID_STATE;
    if (s_ioctl_result != ESP_OK)
        return s_ioctl_result;
    std::memcpy(output, s_mock_active_mac, sizeof(s_mock_active_mac));
    return ESP_OK;
}

static esp_err_t esp_iface_mac_addr_set(const uint8_t *mac, int type)
{
    ++s_eth_set_calls;
    if (type != ESP_MAC_ETH || mac == nullptr)
        return ESP_ERR_INVALID_ARG;
    if (s_eth_set_result != ESP_OK)
        return s_eth_set_result;
    std::memcpy(s_mock_configured_eth_mac, mac, sizeof(s_mock_configured_eth_mac));
    return ESP_OK;
}

static int hex_digit(char value)
{
    if (value >= '0' && value <= '9')
        return value - '0';
    if (value >= 'a' && value <= 'f')
        return value - 'a' + 10;
    if (value >= 'A' && value <= 'F')
        return value - 'A' + 10;
    return -1;
}

static bool parse_mac_id_string(const char *text, uint8_t mac[6])
{
    if (text == nullptr || mac == nullptr)
        return false;

    for (size_t index = 0; index < 6; ++index)
    {
        const int high = hex_digit(text[index * 3]);
        const int low = hex_digit(text[index * 3 + 1]);
        if (high < 0 || low < 0 || (index < 5 && text[index * 3 + 2] != ':'))
            return false;
        mac[index] = static_cast<uint8_t>((high << 4) | low);
    }
    return text[17] == '\0';
}

#include "mac_address_under_test.h"

static void clear_logs()
{
    s_info_logs = 0;
    s_error_logs = 0;
    s_last_info[0] = '\0';
    s_last_error[0] = '\0';
}

int main()
{
    s_mac_address_setting = "";
    assert(configure_mac_address() == ESP_OK);
    assert(s_eth_set_calls == 0);

    s_mac_address_setting = "02:11:22:33:44:55";
    assert(configure_mac_address() == ESP_OK);
    assert(s_eth_set_calls == 1);
    assert(std::memcmp(s_mock_configured_eth_mac, "\x02\x11\x22\x33\x44\x55", 6) == 0);

    struct guarded_mac_t
    {
        uint8_t before[2];
        uint8_t value[6];
        uint8_t after[2];
    } guarded{{0xa5, 0x5a}, {}, {0x3c, 0xc3}};
    assert(read_active_ethernet_mac(guarded.value) == ESP_OK);
    assert(guarded.before[0] == 0xa5 && guarded.before[1] == 0x5a);
    assert(guarded.after[0] == 0x3c && guarded.after[1] == 0xc3);
    assert(std::memcmp(guarded.value, s_mock_active_mac, sizeof(guarded.value)) == 0);

    clear_logs();
    assert(log_active_ethernet_mac() == ESP_OK);
    assert(s_info_logs == 1 && s_error_logs == 0);
    assert(std::strstr(s_last_info, "1E:29:04:D0:3E:17") != nullptr);
    assert(std::strstr(s_last_info, "02:11:22:33:44:55") == nullptr);
    assert(std::strstr(s_last_info, "1c:29:04:ff:fe:d0") == nullptr);

    clear_logs();
    s_mac_address_setting = "invalid";
    const unsigned int eth_calls_before_invalid = s_eth_set_calls;
    assert(configure_mac_address() == ESP_ERR_INVALID_ARG);
    assert(s_eth_set_calls == eth_calls_before_invalid);
    assert(s_error_logs == 1);

    clear_logs();
    s_mac_address_setting = "01:11:22:33:44:55";
    assert(configure_mac_address() == ESP_ERR_INVALID_ARG);
    assert(s_eth_set_calls == eth_calls_before_invalid);
    assert(s_error_logs == 1);

    clear_logs();
    s_mac_address_setting = "00:00:00:00:00:00";
    assert(configure_mac_address() == ESP_ERR_INVALID_ARG);
    assert(s_eth_set_calls == eth_calls_before_invalid);
    assert(s_error_logs == 1);

    clear_logs();
    s_mac_address_setting = "02:11:22:33:44:55";
    s_eth_set_result = ESP_FAIL;
    assert(configure_mac_address() == ESP_FAIL);
    assert(s_error_logs == 1);
    s_eth_set_result = ESP_OK;

    clear_logs();
    s_ioctl_result = ESP_FAIL;
    const unsigned int info_logs_before_failure = s_info_logs;
    assert(log_active_ethernet_mac() == ESP_FAIL);
    assert(s_error_logs == 1);
    assert(s_info_logs == info_logs_before_failure);
    assert(s_last_info[0] == '\0');

    clear_logs();
    s_ioctl_result = ESP_OK;
    s_eth_handle = nullptr;
    assert(log_active_ethernet_mac() == ESP_ERR_INVALID_STATE);
    assert(s_error_logs == 1 && s_info_logs == 0);

    std::puts("PASS: exact Ethernet-MAC override, unicast validation, six-byte active readback and MAC reporting");
}

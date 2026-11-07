#include <cassert>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#define __attribute__(x)
#define FINE_LOCAL_TIMEBASE 1
#define PTP_FREE_RUNNING 1
#define MQTT_ENABLED 1
#define LIQUID_CRYSTAL_DISPLAY_ENABLED 1
#define UPTIME_RESTART_BUTTON_ENABLED 1
#define RBG_LED_KY_016_MODULE_ENABLED 0
#define RBG_LED_WS2812_MODULE_ENABLED 1
#if CONFIG_IDF_TARGET_ESP32P4
#define ESP32_CHIP_TYPE "P4"
#else
#define ESP32_CHIP_TYPE "S31"
#endif

using esp_err_t = int;
using esp_chip_model_t = int;
using esp_eth_handle_t = int *;
static constexpr int ESP_OK = 0;
static constexpr int ESP_FAIL = 1;
static constexpr int ESP_ERR_INVALID_ARG = 2;
static constexpr int ESP_ERR_NOT_SUPPORTED = 3;
static constexpr int CHIP_ESP32P4 = 4;
static constexpr int CHIP_ESP32S31 = 31;
#ifndef TEST_MQTT_QOS
#define TEST_MQTT_QOS 1
#endif
static constexpr int MQTT_QOS = TEST_MQTT_QOS;
static constexpr const char *TAG = "board-test";
struct esp_chip_info_t { int model; unsigned revision; };
struct check_failure {};
static int logs = 0;
static void record_log(const char *, const char *, ...) { ++logs; }
#define ESP_LOGE(tag, ...) record_log(tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) record_log(tag, __VA_ARGS__)
#define TIMING_LOGI(tag, ...) record_log(tag, __VA_ARGS__)
#define GPIO_IS_VALID_GPIO(pin) ((pin) >= 0 && (pin) < 62)
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) (GPIO_IS_VALID_GPIO(pin) && (pin) != 61)
#define ESP_ERROR_CHECK(expression) do { if ((expression) != ESP_OK) throw check_failure{}; } while (0)

#include "board_pins_fixture.h"

#if CONFIG_IDF_TARGET_ESP32S31
static const char *esp_err_to_name(esp_err_t) { return "injected error"; }
static constexpr int ETH_CMD_S_AUTONEGO = 1;
static constexpr int ETH_CMD_WRITE_PHY_REG = 2;
static constexpr int ETH_CMD_READ_PHY_REG = 3;
struct esp_eth_phy_reg_rw_data_t { uint32_t reg_addr = 0; uint32_t *reg_value_p = nullptr; };
static int ioctl_count = 0, fail_ioctl = -1;
static uint32_t selected_register = 0, rx_register = 0, tx_register = 0;
static esp_err_t esp_eth_ioctl(esp_eth_handle_t, int command, void *argument)
{
    const int call = ioctl_count++;
    if (call == fail_ioctl)
        return ESP_FAIL;
    if (command == ETH_CMD_S_AUTONEGO)
    {
        assert(*static_cast<bool *>(argument));
        return ESP_OK;
    }
    auto &access = *static_cast<esp_eth_phy_reg_rw_data_t *>(argument);
    if (command == ETH_CMD_WRITE_PHY_REG && access.reg_addr == 0x1E)
    {
        selected_register = *access.reg_value_p;
        return ESP_OK;
    }
    assert(access.reg_addr == 0x1F && (selected_register == 0xA001 || selected_register == 0xA003));
    uint32_t &value = selected_register == 0xA001 ? rx_register : tx_register;
    if (command == ETH_CMD_READ_PHY_REG)
        *access.reg_value_p = value;
    else
    {
        assert(command == ETH_CMD_WRITE_PHY_REG);
        value = *access.reg_value_p;
    }
    return ESP_OK;
}
#endif

#include "board_under_test.h"

extern "C" uint64_t esp_timer_impl_get_counter_reg(void) { return 16; }

struct rmii_clock_t { int clock_mode = -1, clock_gpio = -1; };
struct rgmii_clock_t { int clock_rx_gpio = -1, clock_tx_gpio = -1, clock_phy_ref_gpio = -2; };
struct rmii_data_t
{
    int tx_en_num = -1, txd0_num = -1, txd1_num = -1;
    int crs_dv_num = -1, rxd0_num = -1, rxd1_num = -1;
};
struct rgmii_data_t
{
    int tx_ctl_num = -1, txd0_num = -1, txd1_num = -1, txd2_num = -1, txd3_num = -1;
    int rx_ctl_num = -1, rxd0_num = -1, rxd1_num = -1, rxd2_num = -1, rxd3_num = -1;
};
struct eth_esp32_emac_config_t
{
    struct { int mdc_num = -1, mdio_num = -1; } smi_gpio;
    int interface = -1;
    struct { rmii_clock_t rmii; rgmii_clock_t rgmii; } clock_config;
    struct { rmii_data_t rmii; rgmii_data_t rgmii; } emac_dataif_gpio;
};
#define ETH_ESP32_EMAC_DEFAULT_CONFIG() eth_esp32_emac_config_t{}
static constexpr int EMAC_DATA_INTERFACE_RMII = 1;
static constexpr int EMAC_DATA_INTERFACE_RGMII = 2;
static constexpr int EMAC_CLK_EXT_IN = 3;

struct eth_config_t {};
static eth_config_t eth_config;
static int ethernet_handle = 1, installs = 0, attachments = 0, registrations = 0, starts = 0;
static esp_eth_handle_t s_eth_handle = nullptr;
static constexpr int ETH_EVENT = 1, ESP_EVENT_ANY_ID = -1;
static unsigned int active_mac_reports = 0;
static esp_err_t log_active_ethernet_mac()
{
    ++active_mac_reports;
    return ESP_OK;
}
static void ethernet_event_handler() {}
static esp_err_t esp_eth_driver_install(const eth_config_t *, esp_eth_handle_t *handle)
{
    ++installs;
    *handle = &ethernet_handle;
    return ESP_OK;
}
static void attach_ethernet_network() { ++attachments; }
static esp_err_t esp_event_handler_register(int, int, void (*)(), void *)
{
    ++registrations;
    return ESP_OK;
}
static esp_err_t esp_eth_start(esp_eth_handle_t) { ++starts; return ESP_OK; }

static void driver_install_and_start()
{
#include "ethernet_startup_under_test.h"
}

static void test_emac_configuration()
{
#include "emac_config_under_test.h"
    assert(emac_config.smi_gpio.mdc_num == EthernetMDCGPIO);
    assert(emac_config.smi_gpio.mdio_num == EthernetMDIOGPIO);
#if CONFIG_IDF_TARGET_ESP32P4
    assert(emac_config.interface == EMAC_DATA_INTERFACE_RMII && EthernetPHYAddress == 1);
    assert(emac_config.clock_config.rmii.clock_mode == EMAC_CLK_EXT_IN);
    assert(emac_config.clock_config.rmii.clock_gpio == 50);
    const rmii_data_t &data = emac_config.emac_dataif_gpio.rmii;
    assert(data.tx_en_num == 49 && data.txd0_num == 34 && data.txd1_num == 35);
    assert(data.crs_dv_num == 28 && data.rxd0_num == 29 && data.rxd1_num == 30);
    assert(emac_config.clock_config.rgmii.clock_rx_gpio == -1);
#else
    assert(emac_config.interface == EMAC_DATA_INTERFACE_RGMII && EthernetPHYAddress == -1);
    assert(emac_config.clock_config.rgmii.clock_rx_gpio == 14);
    assert(emac_config.clock_config.rgmii.clock_tx_gpio == 13);
    assert(emac_config.clock_config.rgmii.clock_phy_ref_gpio == -1);
    const rgmii_data_t &data = emac_config.emac_dataif_gpio.rgmii;
    assert(data.tx_ctl_num == 12 && data.txd0_num == 8 && data.txd1_num == 9 &&
           data.txd2_num == 10 && data.txd3_num == 11);
    assert(data.rx_ctl_num == 15 && data.rxd0_num == 19 && data.rxd1_num == 18 &&
           data.rxd2_num == 17 && data.rxd3_num == 16);
    assert(emac_config.clock_config.rmii.clock_gpio == -1);
#endif
}

static void test_chip_pins_and_timebase()
{
    uint32_t threshold = 0;
    assert(ntp_threshold_for_chip({CHIP_ESP32P4, 103}, threshold) && threshold == 7000);
    assert(ntp_threshold_for_chip({CHIP_ESP32P4, 104}, threshold) && threshold == 10000);
    assert(ntp_threshold_for_chip({CHIP_ESP32S31, 0}, threshold) && threshold == 8500);
    assert(ntp_threshold_for_chip({CHIP_ESP32S31, 300}, threshold) && threshold == 8500);
    assert(!ntp_threshold_for_chip({999, 300}, threshold));

#if CONFIG_IDF_TARGET_ESP32P4
    assert(validate_chip_configuration({CHIP_ESP32P4, 300}) == ESP_OK);
    assert(validate_chip_configuration({CHIP_ESP32P4, 299}) == ESP_ERR_NOT_SUPPORTED);
    assert(validate_chip_configuration({CHIP_ESP32P4, 400}) == ESP_ERR_NOT_SUPPORTED);
    assert(validate_chip_configuration({CHIP_ESP32S31, 0}) == ESP_ERR_NOT_SUPPORTED);
    assert(!ptp_pps_output_available({CHIP_ESP32P4, 299}));
    assert(ptp_pps_output_available({CHIP_ESP32P4, 300}));
#else
    assert(validate_chip_configuration({CHIP_ESP32S31, 0}) == ESP_OK);
    assert(validate_chip_configuration({CHIP_ESP32S31, 99}) == ESP_OK);
    assert(validate_chip_configuration({CHIP_ESP32S31, 100}) == ESP_ERR_NOT_SUPPORTED);
    assert(validate_chip_configuration({CHIP_ESP32P4, 300}) == ESP_ERR_NOT_SUPPORTED);
    assert(ptp_pps_output_available({CHIP_ESP32S31, 0}));
    assert(ptp_pps_output_available({CHIP_ESP32S31, 99}));
#endif

    constexpr board_pin_t unique[] = {{"in", 1, false, false}, {"out", 2, true, false}, {"unused", -1, false, true}};
    constexpr board_pin_t duplicate[] = {{"one", 4, false, false}, {"two", 4, true, false}};
    constexpr board_pin_t negative[] = {{"invalid", -2, false, true}};
    static_assert(distinct_board_gpios(unique));
    static_assert(!distinct_board_gpios(duplicate));
    static_assert(!distinct_board_gpios(negative));
    assert(validate_board_pins(s_board_pins, sizeof(s_board_pins) / sizeof(s_board_pins[0])) == ESP_OK);
    assert(validate_board_pins(unique, 3) == ESP_OK);
    assert(validate_board_pins(duplicate, 2) == ESP_ERR_INVALID_ARG);
    assert(validate_board_pins(negative, 1) == ESP_ERR_INVALID_ARG);
    const board_pin_t required[] = {{"required", -1, false, false}};
    const board_pin_t unavailable_output[] = {{"output", 61, true, false}};
    const board_pin_t out_of_range[] = {{"out of range", 62, false, false}};
    assert(validate_board_pins(required, 1) == ESP_ERR_INVALID_ARG);
    assert(validate_board_pins(unavailable_output, 1) == ESP_ERR_INVALID_ARG);
    assert(validate_board_pins(out_of_range, 1) == ESP_ERR_INVALID_ARG);
    assert(LOCAL_TICKS_PER_US == 16 && LOCAL_TIMEBASE_PRECISION_EXPONENT == -24);
    assert(local_ticks() == 16 && local_ticks_to_ns(1) == 62 && local_ticks_to_ns(2) == 125);
    for (uint64_t ticks = 1; ticks < 100; ++ticks)
        assert(local_ticks_to_ns(ticks) > local_ticks_to_ns(ticks - 1));
    std::puts("PASS: model/revision thresholds, board GPIOs and 16 MHz timebase");
}

static void test_driver_startup()
{
#if CONFIG_IDF_TARGET_ESP32S31
    for (int fail = 0; fail < 7; ++fail)
    {
        installs = attachments = registrations = starts = ioctl_count = 0;
        active_mac_reports = 0;
        fail_ioctl = fail;
        rx_register = 0xA052;
        tx_register = 0xB123;
        try
        {
            driver_install_and_start();
            assert(false && "failed PHY initialization must stop startup");
        }
        catch (const check_failure &)
        {
            assert(installs == 1 && ioctl_count == fail + 1);
            assert(attachments == 0 && registrations == 0 && starts == 0);
        }
    }
    fail_ioctl = -1;
    ioctl_count = 0;
    rx_register = 0xA052;
    tx_register = 0xB123;
#endif
    installs = attachments = registrations = starts = 0;
    active_mac_reports = 0;
    driver_install_and_start();
    assert(installs == 1 && attachments == 1 && registrations == 1 && starts == 1);
    assert(active_mac_reports == 1);
#if CONFIG_IDF_TARGET_ESP32S31
    assert(ioctl_count == 7 && rx_register == 0xA152 && tx_register == 0xB1DD);
#if TIME_SERVER_BOARD_HAS_TF_CARD
    assert(TFCardChipSelectPin == 0 && TFCardMOSIPin == 1 &&
           TFCardClockPin == 2 && TFCardMISOPin == 3);
#endif
#endif
    std::puts("PASS: target-specific Ethernet fields, checked PHY initialization and startup ordering");
}

int main()
{
    test_chip_pins_and_timebase();
    test_emac_configuration();
    test_driver_startup();
}

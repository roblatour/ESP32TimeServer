// ESP32 Time Server v2.9.1
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//
// ESP32 Dev Board:     ESP32-P4-ETH https://www.waveshare.com/esp32-p4-eth.htm
//                                   https://www.waveshare.com/wiki/ESP32-P4-ETH?srsltid=AfmBOoo6nZm5hsPAhtpzT6lWSHd2zhWNPM_mqgbNvyoESbjvbO7uykcH
//
//                      NOTE: Powering the ESP32-P4_ETH by either a USB C cable or, with its optional POE had installed,
//                      a POE Ethernet cable is sufficient to power the ESP32-P4-ETH, GNSS module and LCD screen.
//
//                      ************************************************************************************************
//                      * HOWEVER DO NOT POWER THE ESP32-P4_ETH VIA BOTH ITS USB C CONNECTION AND POE AT THE SAME TIME *
//                      ************************************************************************************************
//
// GNSS (recommended):  SparkFun GNSS Receiver Breakout - MAX-M10S  https://www.sparkfun.com/sparkfun-gnss-receiver-breakout-max-m10s-qwiic.html
//
// LCD2004:             blue/green screen with HD44780 I2C serial interface adapter https://www.aliexpress.com/item/1005006829045609.html.
//
// Wiring:
//
// (mandatory) GNSS module wiring to and from the ESP32-P4-ETH board:
// GNSS GND            <- -> ESP32-P4-ETH GND
// GNSS VCC            <- -> ESP32-P4-ETH 3V3
// GNSS TXD            <- -> ESP32-P4-ETH GPIO17 (RX)
// GNSS RXD            <- -> ESP32-P4-ETH GPIO16 (TX)
// GNSS PPS            <- -> ESP32-P4-ETH GPIO18
//
// (optional) LCD204A V1.5 (HD44780 + PCF8574T I2C backpack) wiring to and from the ESP32-P4-ETH board:
// LCD GND            <- -> ESP32-P4-ETH GND
// LCD VCC            <- -> ESP32-P4-ETH 3V3
// LCD SDA            <- -> ESP32-P4-ETH GPIO8 (SDA)
// LCD SLC/SCL        <- -> ESP32-P4-ETH GPIO7 (SCL)
//
// (optional) Uptime momentary button to and from teh ESP32-P4-ETH board:
// one terminal       <- -> ESP32-P4-ETH GPI03
// the other terminal <- -> ESP32-P4-ETH GND

#include <atomic>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <algorithm>
#include <string>
#include <vector>

#include "Arduino.h"
#include "ArduinoOTA.h"
#include "ETH.h"
#include "SparkFun_u-blox_GNSS_v3.h"
#include "ESP32TimeServerSettings.h"
#include "custom/app_metadata.h"
#include "custom/ntp_cache.h"

extern "C"
{
#include "esp_event.h"
#include "esp_eth_clock.h"
#include "esp_eth_driver.h"
#include "esp_eth_mac_esp.h"
#include "esp_log.h"
#include "ff.h"
#include "esp_mac.h"
#if MQTT_ENABLED
#include "esp_heap_caps.h"
#include "mqtt_client.h"
#endif
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "driver/mcpwm_cap.h"
#include "driver/sdmmc_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "sdmmc_cmd.h"
#include "nvs_flash.h"
#if RBG_LED_ENABLED
#include "driver/gpio.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#if LIQUID_CRYSTAL_DISPLAY_ENABLED
#include "hd44780.h"
#include "pcf8574.h"
#endif
#include "lwip/inet.h"
#include "lwip/sockets.h"
}

static const char *TAG = "main_cpp";

// Optional startup health test
//
// When enabled the startup health test will perform a series of checks to ensure the system is functioning correctly at boot.
//
// These tests include:
//
// Part 1: the program sends itself NTPv3 and NTPv4 (standard and interleaved ) requests via IPv4 and IPv6,
// using loop back and assigned addresses,
//
// Part 2: it health test the same logic that all NTP requests go through to receive those requests,
// process and reply to them using standard and RFC 9769-compatible interleaved  responses,
//
// Part 3: it verifying the responses to ensure proper operations and reports a pass/fail result.
//
// Additionally, this activity is reflected in regular receive/send/client stats within MQTT reporting. .
//
// The startup health test can be useful during development and troubleshooting, but it slightly increase boot time
// and resource usage.
//
// However, it is recommended to keep this option be disabled in production builds to minimize boot time and resource usage.
// Also, and perhaps more importantly, as its results may be misinterpreted.
// For example: if the user's network does not support IPv6 the IPv6 tests may fail even though the system is
// functioning correctly.
//
// Here is what a successful startup health test looks like in the logs:
/*

I (27880) main_cpp: Health Check started
I (27880) main_cpp: Health Check 01 - IPv4 loopback: NTPv3 standard    - Passed
I (27880) main_cpp: Health Check 02 - IPv4 loopback: NTPv4 standard    - Passed
I (27886) main_cpp: Health Check 03 - IPv4 loopback: NTPv4 interleaved - Passed
I (27893) main_cpp: Health Check 04 - IPv4 assigned: NTPv3 standard    - Passed
I (27901) main_cpp: Health Check 05 - IPv4 assigned: NTPv4 standard    - Passed
I (27908) main_cpp: Health Check 06 - IPv4 assigned: NTPv4 interleaved - Passed
I (27915) main_cpp: Health Check 07 - IPv6 loopback: NTPv3 standard    - Passed
I (27922) main_cpp: Health Check 08 - IPv6 loopback: NTPv4 standard    - Passed
I (27929) main_cpp: Health Check 09 - IPv6 loopback: NTPv4 interleaved - Passed
I (27936) main_cpp: Health Check 10 - IPv6 assigned: NTPv3 standard    - Passed
I (27943) main_cpp: Health Check 11 - IPv6 assigned: NTPv4 standard    - Passed
I (27950) main_cpp: Health Check 12 - IPv6 assigned: NTPv4 interleaved - Passed
I (27957) main_cpp: Health Check completed

*/

#define STARTUP_HEALTH_TEST_ENABLED 0 // 0 = Disabled; 1 = Enabled

// The following conditional compile flag is used to determine the ideal stack sizes for
// xTaskCreatePinnedToCore calls used throughout the program.
//
// Unless you are changing the code associated with options below and need to know the impacts of
// those changes have on the stack sizes then the conditional compile flags below should be disabled

#define CALCULATE_STACK_SIZES_ENABLED 0 // 0 = Disabled; 1 = Enabled

// FreeRTOS task stack allocations, in bytes.

static constexpr size_t Ethernet_Transport_Recovery_Task_Stack_Size = 3072;
static constexpr size_t GNSS_Recovery_Task_Stack_Size = 12288;
static constexpr size_t GNSS_Time_Sync_Task_Stack_Size = 2517;
static constexpr size_t Hardware_NTP_Server_Task_Stack_Size = 4096;
static constexpr size_t LED_LCD_Button_Task_Stack_Size = 3100;
static constexpr size_t MQTT_Service_Task_Stack_Size = 3512;
static constexpr size_t NTP_Cache_Purge_Task_Stack_Size = 2304;
static constexpr size_t NTP_Server_Task_Stack_Size = 3560;
static constexpr size_t OTE_Service_Task_Stack_Size = 3560; // unlikely to exceed this stack size (based on current implementation)
static constexpr size_t PPS_Discipline_Task_Stack_Size = 2347;

static constexpr unsigned int Default_Safety_Margin_Percent = 30; // unless otherwise specified add this percentage to the highest stack usage as a safety margin

#if CALCULATE_STACK_SIZES_ENABLED

// For the purposes of calculating stack sizes force DEBUG_ENABLED if any of the stack size calculations are enabled
// so that the task includes all debug processing is included in the stack size calculations
#define DEBUG_ENABLED 1

// For the purposes of calculating stack sizes force all MQTT tasks as enabled if MQTT_ENABLED is enabled
#if MQTT_ENABLED
#define MQTT_CLIENT_REPORTING_ENABLED 1
#define MQTT_MEMORY_REPORTING_ENABLED 1
#define MQTT_HISTORICAL_REPORTING_ENABLED 1
#endif

typedef enum
{
    Ethernet_Transport_Recovery,
    GNSS_Recovery,
    GNSS_Time_Sync,
    Hardware_NTP_Server,
    LED_LCD_Button,
    MQTT_Service,
    NTP_Cache_Purge,
    NTP_Server,
    OTE_Service,
    PPS_Discipline
} TaskToPinToCore_t;

static size_t Highest_Ethernet_Transport_Recovery_Task_Stack_Size = 0;
static size_t Highest_GNSS_Recovery_Task_Stack_Size = 0;
static size_t Highest_GNSS_Time_Sync_Task_Stack_Size = 0;
static size_t Highest_Hardware_NTP_Server_Task_Stack_Size = 0;
static size_t Highest_LED_LCD_Button_Task_Stack_Size = 0;
static size_t Highest_MQTT_Service_Task_Stack_Size = 0;
static size_t Highest_NTP_Cache_Purge_Task_Stack_Size = 0;
static size_t Highest_NTP_Server_Task_Stack_Size = 0;
static size_t Highest_OTE_Service_Task_Stack_Size = 0;
static size_t Highest_PPS_Discipline_Task_Stack_Size = 0;

static SemaphoreHandle_t s_task_stack_usage_mutex = nullptr;

static void report_current_task_stack_usage(TaskToPinToCore_t task_id)
{
    xSemaphoreTake(s_task_stack_usage_mutex, portMAX_DELAY);

    const char *task_name = nullptr;
    size_t allocated_bytes = 0;
    unsigned int safety_margin_percent = 30;
    bool highest_value_changed = false;
    bool show_details = false;

    switch (task_id)
    {
    case Ethernet_Transport_Recovery:
        task_name = "ethernet_transport_task";
        allocated_bytes = Ethernet_Transport_Recovery_Task_Stack_Size;
        break;
    case GNSS_Recovery:
        task_name = "gnss_recovery_task";
        allocated_bytes = GNSS_Recovery_Task_Stack_Size;
        break;
    case GNSS_Time_Sync:
        task_name = "gnss_time_sync_task";
        allocated_bytes = GNSS_Time_Sync_Task_Stack_Size;
        break;
    case Hardware_NTP_Server:
        task_name = "hardware_ntp_server_task";
        allocated_bytes = Hardware_NTP_Server_Task_Stack_Size;
        break;
    case LED_LCD_Button:
        task_name = "LED_LCD_Button_task";
        allocated_bytes = LED_LCD_Button_Task_Stack_Size;
        safety_margin_percent = 70;
        break;
    case MQTT_Service:
        task_name = "mqtt_service_task";
        allocated_bytes = MQTT_Service_Task_Stack_Size;
        break;
    case NTP_Cache_Purge:
        task_name = "ntp_cache_purge_task";
        allocated_bytes = NTP_Cache_Purge_Task_Stack_Size;
        break;
    case NTP_Server:
        task_name = "ntp_server_task";
        allocated_bytes = NTP_Server_Task_Stack_Size;
        safety_margin_percent = 50;
        break;
    case OTE_Service:
        task_name = "ote_service_task";
        allocated_bytes = OTE_Service_Task_Stack_Size;
        break;
    case PPS_Discipline:
        task_name = "pps_discipline_task";
        allocated_bytes = PPS_Discipline_Task_Stack_Size;
        break;
    default:
        task_name = "unknown_task";
        break;
    }

    // calculate the suggested stack size based on the peak usage and safety margin

    const size_t high_watermark_bytes = uxTaskGetStackHighWaterMark(nullptr);
    const size_t peak_usage_bytes = allocated_bytes > high_watermark_bytes ? allocated_bytes - high_watermark_bytes : 0;
    const size_t curent_suggestion = (peak_usage_bytes * (100U + safety_margin_percent) + 99U) / 100U;

    if (show_details)
    {
        ESP_LOGI(task_name,
                 "Stack report: Allocated=%u bytes, HighWater=%u bytes unused, PeakUsage=%u bytes, Suggested=%u bytes",
                 static_cast<unsigned int>(allocated_bytes),
                 static_cast<unsigned int>(high_watermark_bytes),
                 static_cast<unsigned int>(peak_usage_bytes),
                 static_cast<unsigned int>(curent_suggestion));
    }

    if (curent_suggestion == 0)
        ESP_LOGE(task_name, "Error: Suggested stack size is 0 bytes. Try increasing the allocted bytes of %u", static_cast<unsigned int>(allocated_bytes));

    switch (task_id)
    {
    case Ethernet_Transport_Recovery:
        if (curent_suggestion > Highest_Ethernet_Transport_Recovery_Task_Stack_Size)
        {
            Highest_Ethernet_Transport_Recovery_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case GNSS_Recovery:
        if (curent_suggestion > Highest_GNSS_Recovery_Task_Stack_Size)
        {
            Highest_GNSS_Recovery_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case GNSS_Time_Sync:
        if (curent_suggestion > Highest_GNSS_Time_Sync_Task_Stack_Size)
        {
            Highest_GNSS_Time_Sync_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case Hardware_NTP_Server:
        if (curent_suggestion > Highest_Hardware_NTP_Server_Task_Stack_Size)
        {
            Highest_Hardware_NTP_Server_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case LED_LCD_Button:
        if (curent_suggestion > Highest_LED_LCD_Button_Task_Stack_Size)
        {
            Highest_LED_LCD_Button_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case MQTT_Service:
        if (curent_suggestion > Highest_MQTT_Service_Task_Stack_Size)
        {
            Highest_MQTT_Service_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case NTP_Cache_Purge:
        if (curent_suggestion > Highest_NTP_Cache_Purge_Task_Stack_Size)
        {
            Highest_NTP_Cache_Purge_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case NTP_Server:
        if (curent_suggestion > Highest_NTP_Server_Task_Stack_Size)
        {
            Highest_NTP_Server_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case OTE_Service:
        if (curent_suggestion > Highest_OTE_Service_Task_Stack_Size)
        {
            Highest_OTE_Service_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }

        break;
    case PPS_Discipline:
        if (curent_suggestion > Highest_PPS_Discipline_Task_Stack_Size)
        {
            Highest_PPS_Discipline_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    default:

        break;
    }

    // Report the highest suggested stack sizes for all tasks to date in such a way that they can be easily copied into the code above
    // If no new highest value has yet been reported for a task the value shown will be 0, the old value will remain as the comment
    if (highest_value_changed)
    {
        ESP_LOGI(TAG, "Suggested updated task stack sizes:");
        ESP_LOGI(TAG, " ");

        if (Highest_Ethernet_Transport_Recovery_Task_Stack_Size > Ethernet_Transport_Recovery_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t Ethernet_Transport_Recovery_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_Ethernet_Transport_Recovery_Task_Stack_Size), static_cast<unsigned int>(Ethernet_Transport_Recovery_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t Ethernet_Transport_Recovery_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_Ethernet_Transport_Recovery_Task_Stack_Size), static_cast<unsigned int>(Ethernet_Transport_Recovery_Task_Stack_Size));

        if (Highest_GNSS_Recovery_Task_Stack_Size > GNSS_Recovery_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t GNSS_Recovery_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_GNSS_Recovery_Task_Stack_Size), static_cast<unsigned int>(GNSS_Recovery_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t GNSS_Recovery_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_GNSS_Recovery_Task_Stack_Size), static_cast<unsigned int>(GNSS_Recovery_Task_Stack_Size));

        if (Highest_GNSS_Time_Sync_Task_Stack_Size > GNSS_Time_Sync_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t GNSS_Time_Sync_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_GNSS_Time_Sync_Task_Stack_Size), static_cast<unsigned int>(GNSS_Time_Sync_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t GNSS_Time_Sync_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_GNSS_Time_Sync_Task_Stack_Size), static_cast<unsigned int>(GNSS_Time_Sync_Task_Stack_Size));

        if (Highest_Hardware_NTP_Server_Task_Stack_Size > Hardware_NTP_Server_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t Hardware_NTP_Server_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_Hardware_NTP_Server_Task_Stack_Size), static_cast<unsigned int>(Hardware_NTP_Server_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t Hardware_NTP_Server_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_Hardware_NTP_Server_Task_Stack_Size), static_cast<unsigned int>(Hardware_NTP_Server_Task_Stack_Size));

        if (Highest_LED_LCD_Button_Task_Stack_Size > LED_LCD_Button_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t LED_LCD_Button_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_LED_LCD_Button_Task_Stack_Size), static_cast<unsigned int>(LED_LCD_Button_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t LED_LCD_Button_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_LED_LCD_Button_Task_Stack_Size), static_cast<unsigned int>(LED_LCD_Button_Task_Stack_Size));

        if (Highest_MQTT_Service_Task_Stack_Size > MQTT_Service_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t MQTT_Service_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_MQTT_Service_Task_Stack_Size), static_cast<unsigned int>(MQTT_Service_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t MQTT_Service_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_MQTT_Service_Task_Stack_Size), static_cast<unsigned int>(MQTT_Service_Task_Stack_Size));

        if (Highest_NTP_Cache_Purge_Task_Stack_Size > NTP_Cache_Purge_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t NTP_Cache_Purge_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_NTP_Cache_Purge_Task_Stack_Size), static_cast<unsigned int>(NTP_Cache_Purge_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t NTP_Cache_Purge_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_NTP_Cache_Purge_Task_Stack_Size), static_cast<unsigned int>(NTP_Cache_Purge_Task_Stack_Size));

        if (Highest_NTP_Server_Task_Stack_Size > NTP_Server_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t NTP_Server_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_NTP_Server_Task_Stack_Size), static_cast<unsigned int>(NTP_Server_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t NTP_Server_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_NTP_Server_Task_Stack_Size), static_cast<unsigned int>(NTP_Server_Task_Stack_Size));

        if (Highest_OTE_Service_Task_Stack_Size > OTE_Service_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t OTE_Service_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_OTE_Service_Task_Stack_Size), static_cast<unsigned int>(OTE_Service_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t OTE_Service_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_OTE_Service_Task_Stack_Size), static_cast<unsigned int>(OTE_Service_Task_Stack_Size));

        if (Highest_PPS_Discipline_Task_Stack_Size > PPS_Discipline_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t PPS_Discipline_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_PPS_Discipline_Task_Stack_Size), static_cast<unsigned int>(PPS_Discipline_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t PPS_Discipline_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_PPS_Discipline_Task_Stack_Size), static_cast<unsigned int>(PPS_Discipline_Task_Stack_Size));

        ESP_LOGI(TAG, " ");
    };

    xSemaphoreGive(s_task_stack_usage_mutex);
}

#endif

#ifndef UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
#define UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED 1
#endif

static constexpr gpio_num_t ETH_MDC_GPIO = GPIO_NUM_31;
static constexpr gpio_num_t ETH_MDIO_GPIO = GPIO_NUM_52;
static constexpr gpio_num_t ETH_PHY_RST_GPIO = GPIO_NUM_51;
static constexpr int ETH_PHY_ADDRESS = 1;

#if LIQUID_CRYSTAL_DISPLAY_ENABLED
static constexpr i2c_port_t LCD_I2C_PORT = I2C_NUM_0;
static constexpr gpio_num_t LCD_I2C_SDA_GPIO = GPIO_NUM_8;
static constexpr gpio_num_t LCD_I2C_SCL_GPIO = GPIO_NUM_7;
#endif

static constexpr uint16_t NTP_PORT = 123;
static constexpr size_t NTP_PACKET_SIZE = 48;
static constexpr uint64_t NTP_EPOCH_OFFSET = 2208988800ULL;
static constexpr int8_t NTP_PRECISION_EXPONENT = -13;
static constexpr uint32_t NTP_ROOT_DISPERSION = 66;
static constexpr uint64_t NTP_CACHE_TIMESTAMP_MATCH_TOLERANCE = 8;
static constexpr size_t NTP_SOCKET_BATCH_LIMIT = 16;
static constexpr EventBits_t ETH_CONNECTED_BIT = BIT0;
static constexpr EventBits_t ETH_GOT_IP_BIT = BIT1;
static constexpr EventBits_t ETH_GOT_IP6_BIT = BIT2;
static constexpr size_t IP_ADDRESS_TEXT_SIZE = INET6_ADDRSTRLEN;
static constexpr uint32_t OTE_Failure_Display_Time_Ms = 10000;
static constexpr uint32_t OTE_Reboot_Delay_Ms = 5000;

#if LIQUID_CRYSTAL_DISPLAY_ENABLED
static i2c_dev_t s_lcd_io{};
static uint8_t s_lcd_addr = 0;
static bool s_lcd_ready = false;
static hd44780_t s_lcd{};
static char s_lcd_last_lines[lcdRows][lcdColumns + 1] = {};
static bool s_lcd_line_cached[lcdRows] = {};
#else
#define display_line(...)
#endif

struct PpsCaptureEvent
{
    int64_t approximate_edge_us;
};

static constexpr uint32_t PPS_CAPTURE_RESOLUTION_HZ = 80000000;

static EventGroupHandle_t s_net_event_group = nullptr;
static SemaphoreHandle_t s_time_mutex = nullptr;
static SemaphoreHandle_t s_pps_semaphore = nullptr;
static QueueHandle_t s_pps_timestamp_queue = nullptr;
static QueueHandle_t s_pps_sync_timestamp_queue = nullptr;
static mcpwm_cap_timer_handle_t s_pps_capture_timer = nullptr;
static mcpwm_cap_channel_handle_t s_pps_capture_channel = nullptr;
static SemaphoreHandle_t s_ote_mutex = nullptr;
#if LIQUID_CRYSTAL_DISPLAY_ENABLED
static SemaphoreHandle_t s_lcd_mutex = nullptr;
#endif
static SemaphoreHandle_t s_sync_state_mutex = nullptr;

#if OTE_UPDATES_ENABLED
static bool s_ote_in_progress = false;
static bool s_ote_failed = false;
static bool s_ote_success = false;
static int s_ote_progress_percent = -1;
static int64_t s_ote_failure_display_until_us = 0;
static int64_t s_ote_reboot_at_us = 0;
static char s_ote_error_reason[lcdColumns + 1] = "";
#endif

static std::atomic<bool> s_safe_guard_tripped{false};
static std::atomic<bool> s_time_setting_in_progress{false};
static std::atomic<bool> s_time_has_been_set{false};
static std::atomic<uint64_t> s_ntp_reference_time_64{0};
static std::atomic<bool> s_ntp_reference_valid{false};
static std::atomic<bool> s_ptp_clock_ready{false};

static_assert(PreferIPvX == 0 || PreferIPvX == 4 || PreferIPvX == 6, "PreferIPvX must be 0, 4, or 6");

static char s_ip_address[IP_ADDRESS_TEXT_SIZE] = "";
static char s_ipv4_address[INET_ADDRSTRLEN] = "";
static char s_ipv6_address[IP_ADDRESS_TEXT_SIZE] = "";

static void update_selected_ip_address()
{
    const bool has_ipv4 = s_ipv4_address[0] != '\0';
    const bool has_ipv6 = s_ipv6_address[0] != '\0';

    const char *selected_address = "";
    if (PreferIPvX == 4)
        selected_address = has_ipv4 ? s_ipv4_address : s_ipv6_address;
    else if (PreferIPvX == 6)
        selected_address = has_ipv6 ? s_ipv6_address : s_ipv4_address;
    else if (strcmp(s_ip_address, s_ipv4_address) == 0 && has_ipv4)
        selected_address = s_ipv4_address;
    else if (strcmp(s_ip_address, s_ipv6_address) == 0 && has_ipv6)
        selected_address = s_ipv6_address;
    else
        selected_address = has_ipv4 ? s_ipv4_address : s_ipv6_address;

    snprintf(s_ip_address, sizeof(s_ip_address), "%s", selected_address);
}

static std::atomic<bool> s_ethernet_connected{false};
static std::atomic<bool> s_hardware_ntp_accepting{false};
static std::atomic<bool> s_ntp_external_responses_enabled{false};
static std::atomic<bool> s_ntp_server_ready{false};
#if RBG_LED_ENABLED
static std::atomic<bool> s_open_for_business_message_written{false};
static std::atomic<bool> s_gnss_pps_startup_qualification_in_progress{false};
static void control_KY_016_RGB_LED(RGB_LED_Color color, bool enabled);
#endif
static std::atomic<int64_t> s_last_hardware_ntp_response_us{0};
static SemaphoreHandle_t s_hardware_ntp_transmit_mutex = nullptr;

static bool configure_static_ip();
static bool configure_hardware_timestamps();
static bool start_ethernet_driver();

static bool format_socket_address(const struct sockaddr_storage &address, char *buffer, size_t buffer_size)
{
    if (buffer_size == 0)
        return false;

    const void *source = nullptr;
    if (address.ss_family == AF_INET)
        source = &reinterpret_cast<const struct sockaddr_in *>(&address)->sin_addr;
    else if (address.ss_family == AF_INET6)
        source = &reinterpret_cast<const struct sockaddr_in6 *>(&address)->sin6_addr;
    else
    {
        buffer[0] = '\0';
        return false;
    }

    return inet_ntop(address.ss_family, source, buffer, buffer_size) != nullptr;
}

#if MQTT_ENABLED

// MQTT_MAX_REPORT_SIZE is based on MQTT_TF_Client_Limit (in ESP32TimeServerSetting.h)
// please see the spreadsheet at tools/json_message_calculator.xlsx for calculation details
// Note: increasing this limit will increase memory usage for MQTT report buffering and may impact system performance.
static constexpr size_t MQTT_MAX_REPORT_SIZE = 31944; // examples: for 256 clients use 2060; for 500 clients use 31944

// MQTT_REPORT_SIZE based on MQTT_CLIENT_SIZE (below)
// please see the spreadsheet at tools/json_message_calculator.xlsx for calculation details
// Note: increasing this limit will increase memory usage for MQTT report buffering and may impact system performance.
static constexpr size_t MQTT_CLIENT_LIMIT = 50;
static constexpr size_t MQTT_REPORT_SIZE = 4944;

static constexpr size_t MQTT_NTP_EVENT_QUEUE_DEPTH = 1024;
static constexpr size_t MQTT_REPORT_QUEUE_DEPTH = 4;
static constexpr size_t MQTT_NTP_EVENT_BATCH_LIMIT = 128;

static constexpr uint32_t MQTT_RESTART_PUBLISH_TIMEOUT_MS = 1000;
static constexpr uint32_t MQTT_QUEUED_PUBLISH_DELAY_MS = 250;
static constexpr char TF_MOUNT_POINT[] = "/tfcard";
static constexpr char TF_QUEUE_DIRECTORY[] = "/tfcard/Queue";

struct mqtt_client_request_t
{
    sa_family_t address_family = AF_UNSPEC;
    uint8_t address[sizeof(struct in6_addr)] = {};
    uint32_t requests = 0;
};

struct mqtt_report_t
{
    char payload[MQTT_REPORT_SIZE] = "";
};

static SemaphoreHandle_t s_mqtt_stats_mutex = nullptr;
static QueueHandle_t s_mqtt_ntp_event_queue = nullptr;
static esp_mqtt_client_handle_t s_mqtt_client = nullptr;
static std::atomic<bool> s_mqtt_setup_failed{false};
static std::atomic<bool> s_mqtt_connected{false};
static std::atomic<bool> s_mqtt_has_connected{false};
static std::atomic<int64_t> s_mqtt_disconnected_since_us{0};
static std::atomic<int> s_mqtt_restart_publish_id{-1};
static std::atomic<bool> s_mqtt_restart_publish_completed{false};
static std::atomic<uint32_t> s_pps_pulses{0};
static std::atomic<uint32_t> s_ntp_requests_this_second{0};
static std::atomic<uint32_t> s_ntp_valid_requests{0};
static std::atomic<uint32_t> s_ntp_invalid_requests{0};
static std::atomic<uint32_t> s_ntp_responses{0};
static std::atomic<uint32_t> s_ntp_responses_synchronized_and_disciplined{0};
static std::atomic<uint32_t> s_ntp_responses_gnss_unsynchronized{0};
static std::atomic<uint32_t> s_ntp_responses_pps_undisciplined{0};
static std::atomic<uint32_t> s_ntp_telemetry_events_dropped{0};
static std::atomic<uint8_t> s_satellite_count{0};
static std::atomic<uint8_t> s_satellite_min{UINT8_MAX};
static std::atomic<uint8_t> s_satellite_max{0};

static void mqtt_note_satellite_count(uint8_t satellites)
{
    s_satellite_count.store(satellites, std::memory_order_relaxed);

    uint8_t minimum = s_satellite_min.load(std::memory_order_relaxed);
    while (satellites < minimum &&
           !s_satellite_min.compare_exchange_weak(minimum, satellites, std::memory_order_relaxed, std::memory_order_relaxed))
    {
    }

    uint8_t maximum = s_satellite_max.load(std::memory_order_relaxed);
    while (satellites > maximum &&
           !s_satellite_max.compare_exchange_weak(maximum, satellites, std::memory_order_relaxed, std::memory_order_relaxed))
    {
    }
}

static std::atomic<int64_t> s_eth_link_connected_us{0};
static std::atomic<int64_t> s_eth_link_up_total_us{0};
#if MQTT_ENABLED && MQTT_HISTORICAL_REPORTING_ENABLED
static std::atomic<time_t> s_last_synchronized_and_disciplined{0};
static std::atomic<time_t> s_last_gnss_unsynchronized{0};
static std::atomic<time_t> s_last_pps_undisciplined{0};
static std::atomic<bool> s_historical_gnss_fault_active{false};
static std::atomic<bool> s_historical_pps_fault_active{false};
#endif

#if MQTT_ENABLED && MQTT_CLIENT_REPORTING_ENABLED
static std::atomic<bool> s_mqtt_client_table_overflown{false};
static size_t s_mqtt_client_count = 0;
#endif

static int64_t s_mqtt_last_link_up_us = 0;
static mqtt_report_t s_mqtt_reports[MQTT_REPORT_QUEUE_DEPTH]{};
static char s_mqtt_payload[MQTT_MAX_REPORT_SIZE] = "";
static size_t s_mqtt_report_head = 0;
static std::atomic<size_t> s_mqtt_queued_messages_count{0};
static uint32_t s_mqtt_queued_messages_discarded = 0;
static mqtt_client_request_t s_mqtt_clients[MQTT_TF_Client_Limit]{};
static std::atomic<bool> s_tf_queue_available{false};
static sdmmc_card_t *s_tf_card = nullptr;
static uint64_t s_tf_queue_next_sequence = 0;
static std::atomic<uint32_t> s_ntp_most_requests_per_second{0};
static char s_mqtt_uri[64] = "";
static char s_mqtt_report_topic[128] = "";
static char s_mqtt_status_topic[128] = "";

static void mqtt_publish_final_report();
static bool mqtt_publish_or_queue_restart_notification(const char *reason);
static void mqtt_enqueue_ntp_request(const struct sockaddr_storage &source_address);
#endif

static HardwareSerial s_gnss_serial(1);
static SFE_UBLOX_GNSS_SERIAL s_gnss;
static uint32_t s_detected_gnss_baud = 0;
static std::atomic<bool> s_saved_gnss_baud_communication_failed{false};
static bool s_gnss_required_assume_success = false;

// The following flag for National Marine Electronics Association (NEMA) fallback doesn't determine if it is allowed or not
// rather the program sets it to true if it is required (due to an older / ubox noncompliant hardware gnss module being used)

static bool s_use_nmea_fallback = false;
static bool s_gnss_is_max_m10s = false;
static std::atomic<bool> s_gnss_locked{false};
static std::atomic<int64_t> s_gnss_lock_started_us{0};
static std::atomic<int64_t> s_gnss_locked_total_us{0};
static std::atomic<bool> s_pps_discipline_active{false};
static std::atomic<bool> s_gnss_recovery_in_progress{false};
static std::atomic<int64_t> s_last_gnss_recovery_us{0};

struct sync_faults_t
{
    bool pps_missing = false;
    bool gnss_invalid = false;
    bool sanity_mismatch = false;
    bool sync_stale = false;
};

static bool has_sync_fault(const sync_faults_t &faults)
{
    return faults.pps_missing || faults.gnss_invalid || faults.sanity_mismatch || faults.sync_stale;
}

struct sync_state_t
{
    int64_t last_successful_sync_us = 0;
    int64_t last_sync_attempt_us = 0;
    int64_t last_pps_seen_us = 0;
    time_t last_sync_delta_seconds = 0;
    uint32_t consecutive_sync_failures = 0;
    uint32_t consecutive_sanity_failures = 0;
    bool holdover_mode = false;
    bool pps_active = false;
    bool gnss_timing_valid = false;
    int64_t last_gnss_valid_us = 0;
    sync_faults_t faults{};
};

struct sync_candidate_t
{
    time_t candidate_time = 0;
    bool use_pps_alignment = false;
    bool used_nmea_fallback = false;
    int64_t pps_release_time_us = 0;
    sync_faults_t failures{};
};

static sync_state_t s_sync_state{};
static sync_state_t get_sync_state_snapshot();
static std::atomic<uint32_t> s_ntp_sync_state_sequence{0};
static std::atomic<int64_t> s_ntp_last_successful_sync_us{0};
static std::atomic<int64_t> s_ntp_last_gnss_valid_us{0};
static std::atomic<bool> s_ntp_pps_active{false};
static std::atomic<bool> s_ntp_gnss_timing_valid{false};
static std::atomic<bool> s_ntp_pps_missing{false};
static std::atomic<bool> s_ntp_gnss_invalid{false};
static std::atomic<bool> s_ntp_sanity_mismatch{false};
static std::atomic<bool> s_ntp_sync_stale{false};
static constexpr int64_t Sync_Stale_After_Us = static_cast<int64_t>(periodicGNSSRefreshEveryThisNumberOfMinutes) * 60LL * 1000000LL * 3LL;
static constexpr int64_t Gnss_Validity_Timeout_Us = 3500000LL;
static constexpr int64_t Sync_Reboot_After_Us = 30LL * 60LL * 1000000LL;
static constexpr int64_t Max_Sync_Attempt_Us = 10000000LL;
static constexpr int64_t Gnss_Invalid_Reacquisition_After_Us = 30LL * 1000000LL;
static constexpr int64_t Runtime_gnss_Recovery_Min_Interval_Us = 5LL * 60LL * 1000000LL;
static constexpr uint32_t Sync_Failures_Before_Runtime_Recovery = 20;
static constexpr uint32_t Sanity_Failures_Before_Fault = 2;
static constexpr uint32_t gnss_Startup_Qualification_Duration_Ms = 15000UL;
static constexpr uint32_t gnss_Startup_Qualification_PPS_Edges = 10;
static constexpr int64_t gnss_Startup_Min_PPS_Interval_Us = 800000LL;
static constexpr int64_t gnss_Startup_Max_PPS_Interval_Us = 1200000LL;

static constexpr char gnss_NVS_NAMESPACE[] = "gnss_state";
static constexpr char gnss_NVS_KEY_ID_TYPE[] = "id_type";
static constexpr char gnss_NVS_KEY_ID_VALUE[] = "id_value";
static constexpr char gnss_NVS_KEY_MAX_BAUD[] = "max_baud";
static constexpr char gnss_NVS_KEY_INITIAL_BAUD[] = "initial_baud";
static constexpr char gnss_NVS_KEY_ATTEMPT_NO_SIGNAL_RECOVERY[] = "no_sig_rcv";
static constexpr char restart_NVS_NAMESPACE[] = "restart_evt";
static constexpr char restart_NVS_KEY_PENDING[] = "pending";

static constexpr char gnss_ID_TYPE_UNIQID[] = "uniqid";
static constexpr char gnss_ID_TYPE_MODULE_FP[] = "module_fp";
static constexpr char gnss_ID_TYPE_GENERIC[] = "generic";
static constexpr char gnss_ID_VALUE_UNDETERMINED[] = "undetermined_gnss_board";

/* pre-release code - commented out
static constexpr bool reduceGNSSUART1OutputToTimeMessages = true;
static constexpr bool saveReducedGNSSUART1OutputPermanently = false;
*/
struct gnss_identity_t
{
    bool valid = false;
    char type[16] = "";
    char value[96] = "";
};

struct gnss_nvs_data_t
{
    bool has_stored_data = false;
    bool has_id_type = false;
    char id_type[16] = "";
    char id_value[96] = "";
    uint32_t initial_baud = 0;
    uint32_t max_baud = 0;
};

static uint32_t s_gnss_target_baud = 0;

struct nmea_rmc_time_t
{
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
};

static void refresh_sync_state_locked(sync_state_t *state, int64_t now_us)
{
    if (state == nullptr)
        return;

    bool sync_stale = state->last_successful_sync_us > 0 && (now_us - state->last_successful_sync_us) > Sync_Stale_After_Us;
    bool pps_missing = s_time_has_been_set.load() && !state->pps_active;
    bool gnss_missing = s_time_has_been_set.load() && (!state->gnss_timing_valid ||
                                                       state->last_gnss_valid_us == 0 ||
                                                       (now_us - state->last_gnss_valid_us) > Gnss_Validity_Timeout_Us);

    state->faults.sync_stale = sync_stale;
    state->faults.gnss_invalid = gnss_missing;
    state->faults.pps_missing = pps_missing;

#if DEBUG_ENABLED
    if (gnss_missing)
        ESP_LOGE(TAG, "GNSS invalid - GNSS missing.");
#endif

    state->holdover_mode = has_sync_fault(state->faults);
}

static void publish_ntp_sync_state_locked(const sync_state_t &state)
{
    s_ntp_sync_state_sequence.fetch_add(1, std::memory_order_release);
    s_ntp_last_successful_sync_us.store(state.last_successful_sync_us, std::memory_order_relaxed);
    s_ntp_last_gnss_valid_us.store(state.last_gnss_valid_us, std::memory_order_relaxed);
    s_ntp_pps_active.store(state.pps_active, std::memory_order_relaxed);
    s_ntp_gnss_timing_valid.store(state.gnss_timing_valid, std::memory_order_relaxed);
    s_ntp_pps_missing.store(state.faults.pps_missing, std::memory_order_relaxed);
    s_ntp_gnss_invalid.store(state.faults.gnss_invalid, std::memory_order_relaxed);
    s_ntp_sanity_mismatch.store(state.faults.sanity_mismatch, std::memory_order_relaxed);
    s_ntp_sync_stale.store(state.faults.sync_stale, std::memory_order_relaxed);
    s_ntp_sync_state_sequence.fetch_add(1, std::memory_order_release);
}

static void sync_state_note_attempt()
{
    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.last_sync_attempt_us = esp_timer_get_time();
        refresh_sync_state_locked(&s_sync_state, s_sync_state.last_sync_attempt_us);
        publish_ntp_sync_state_locked(s_sync_state);
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static void sync_state_note_pps_edge(int64_t edge_us)
{
    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.last_pps_seen_us = edge_us;
        s_sync_state.pps_active = true;
        refresh_sync_state_locked(&s_sync_state, edge_us);
        publish_ntp_sync_state_locked(s_sync_state);
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static void sync_state_note_pps_timeout(int64_t now_us)
{
#if MQTT_ENABLED && MQTT_HISTORICAL_REPORTING_ENABLED
    if (s_time_has_been_set.load() && !s_historical_pps_fault_active.exchange(true))
        s_last_pps_undisciplined.store(time(nullptr));
#endif

    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.pps_active = false;
        refresh_sync_state_locked(&s_sync_state, now_us);
        publish_ntp_sync_state_locked(s_sync_state);
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static void set_gnss_lock_state(bool valid)
{
    int64_t now_us = esp_timer_get_time();
    bool previous = s_gnss_locked.exchange(valid);
    if (previous == valid)
        return;

    if (valid)
    {
        s_gnss_lock_started_us.store(now_us);
    }
    else
    {
        int64_t lock_started_us = s_gnss_lock_started_us.exchange(0);
        if (lock_started_us > 0 && now_us > lock_started_us)
            s_gnss_locked_total_us.fetch_add(now_us - lock_started_us);
    }
}

static void sync_state_note_gnss_validity(bool valid)
{
    set_gnss_lock_state(valid);

#if MQTT_ENABLED && MQTT_HISTORICAL_REPORTING_ENABLED
    if (!valid && s_time_has_been_set.load() && !s_historical_gnss_fault_active.exchange(true))
        s_last_gnss_unsynchronized.store(time(nullptr));
#endif

    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        int64_t now_us = esp_timer_get_time();
        s_sync_state.gnss_timing_valid = valid;
        if (valid)
            s_sync_state.last_gnss_valid_us = now_us;
        refresh_sync_state_locked(&s_sync_state, now_us);
        publish_ntp_sync_state_locked(s_sync_state);
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static uint32_t sync_state_note_failure(const sync_faults_t &faults, time_t update_delta)
{
    uint32_t failure_count = 0;

    if (faults.gnss_invalid)
        set_gnss_lock_state(false);

#if MQTT_ENABLED && MQTT_HISTORICAL_REPORTING_ENABLED
    if (s_time_has_been_set.load())
    {
        if (faults.gnss_invalid && !s_historical_gnss_fault_active.exchange(true))
            s_last_gnss_unsynchronized.store(time(nullptr));
        if (faults.pps_missing && !s_historical_pps_fault_active.exchange(true))
            s_last_pps_undisciplined.store(time(nullptr));
    }
#endif

    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.last_sync_delta_seconds = update_delta;
        s_sync_state.consecutive_sync_failures++;
        s_sync_state.faults.pps_missing = s_sync_state.faults.pps_missing || faults.pps_missing;
        s_sync_state.faults.gnss_invalid = s_sync_state.faults.gnss_invalid || faults.gnss_invalid;
        s_sync_state.faults.sanity_mismatch = s_sync_state.faults.sanity_mismatch || faults.sanity_mismatch;
        s_sync_state.faults.sync_stale = s_sync_state.faults.sync_stale || faults.sync_stale;
        refresh_sync_state_locked(&s_sync_state, esp_timer_get_time());
        publish_ntp_sync_state_locked(s_sync_state);
        failure_count = s_sync_state.consecutive_sync_failures;
        xSemaphoreGive(s_sync_state_mutex);
    }

    return failure_count;
}

static uint32_t sync_state_note_sanity_retry(time_t update_delta)
{
    uint32_t failure_count = 0;

    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.last_sync_delta_seconds = update_delta;
        s_sync_state.consecutive_sync_failures++;
        s_sync_state.consecutive_sanity_failures++;
        refresh_sync_state_locked(&s_sync_state, esp_timer_get_time());
        publish_ntp_sync_state_locked(s_sync_state);
        failure_count = s_sync_state.consecutive_sanity_failures;
        xSemaphoreGive(s_sync_state_mutex);
    }

    return failure_count;
}

static void sync_state_clear_sanity_failures()
{
    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.consecutive_sanity_failures = 0;
        refresh_sync_state_locked(&s_sync_state, esp_timer_get_time());
        publish_ntp_sync_state_locked(s_sync_state);
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static void sync_state_reset_failure_counters()
{
    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.consecutive_sync_failures = 0;
        s_sync_state.consecutive_sanity_failures = 0;
        refresh_sync_state_locked(&s_sync_state, esp_timer_get_time());
        publish_ntp_sync_state_locked(s_sync_state);
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static void sync_state_note_success(time_t update_delta)
{
    set_gnss_lock_state(true);

#if MQTT_ENABLED && MQTT_HISTORICAL_REPORTING_ENABLED
    bool recovery_completed = s_historical_gnss_fault_active.exchange(false) ||
                              s_historical_pps_fault_active.exchange(false);
    if (s_last_synchronized_and_disciplined.load() == 0 || recovery_completed)
        s_last_synchronized_and_disciplined.store(time(nullptr));
#endif

    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.last_successful_sync_us = esp_timer_get_time();
        s_sync_state.gnss_timing_valid = true;
        s_sync_state.last_gnss_valid_us = s_sync_state.last_successful_sync_us;
        s_sync_state.last_sync_delta_seconds = update_delta;
        s_sync_state.consecutive_sync_failures = 0;
        s_sync_state.consecutive_sanity_failures = 0;
        s_sync_state.faults = {};
        refresh_sync_state_locked(&s_sync_state, s_sync_state.last_successful_sync_us);
        publish_ntp_sync_state_locked(s_sync_state);
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static sync_state_t get_sync_state_snapshot()
{
    sync_state_t snapshot{};

    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        refresh_sync_state_locked(&s_sync_state, esp_timer_get_time());
        publish_ntp_sync_state_locked(s_sync_state);
        snapshot = s_sync_state;
        xSemaphoreGive(s_sync_state_mutex);
    }

    return snapshot;
}

static bool first_sync_candidates_are_plausible(const sync_candidate_t &first_candidate, const sync_candidate_t &second_candidate)
{
    time_t delta = second_candidate.candidate_time - first_candidate.candidate_time;
    return delta >= 1 && delta <= 3;
}

#if LIQUID_CRYSTAL_DISPLAY_ENABLED
static esp_err_t lcd_write_i2c(const hd44780_t *lcd, uint8_t data)
{
    (void)lcd;
    return pcf8574_port_write(&s_lcd_io, data);
}

static void init_lcd_descriptor()
{
    memset(&s_lcd, 0, sizeof(s_lcd));
    s_lcd.write_cb = lcd_write_i2c;
    s_lcd.font = HD44780_FONT_5X8;
    s_lcd.lines = lcdRows;
    s_lcd.pins.rs = 0;
    s_lcd.pins.e = 2;
    s_lcd.pins.d4 = 4;
    s_lcd.pins.d5 = 5;
    s_lcd.pins.d6 = 6;
    s_lcd.pins.d7 = 7;
    s_lcd.pins.bl = 3;
    s_lcd.backlight = true;
}

static void display_line(uint8_t row, const char *text)
{

    // LCD rows once displayed are cached and only updated when changed

    char padded[lcdColumns + 1];
    memset(padded, ' ', sizeof(padded) - 1);
    padded[lcdColumns] = '\0';

    if (text != nullptr)
    {
        size_t len = strlen(text);
        if (len > lcdColumns)
        {
            len = lcdColumns;
        }
        memcpy(padded, text, len);
    }

    if (!s_lcd_ready)
    {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "LCD not ready for row %u", row);
#endif
        return;
    }

    if (s_lcd_mutex != nullptr)
        xSemaphoreTake(s_lcd_mutex, portMAX_DELAY);

    if (row < lcdRows && s_lcd_line_cached[row] && strcmp(s_lcd_last_lines[row], padded) == 0)
    {
        if (s_lcd_mutex != nullptr)
            xSemaphoreGive(s_lcd_mutex);
        return;
    }

    esp_err_t err = hd44780_gotoxy(&s_lcd, 0, row);
    if (err != ESP_OK)
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "hd44780_gotoxy failed: %s", esp_err_to_name(err));
#endif
        if (s_lcd_mutex != nullptr)
            xSemaphoreGive(s_lcd_mutex);
        return;
    }

    err = hd44780_puts(&s_lcd, padded);
    if (err != ESP_OK)
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "hd44780_puts failed: %s", esp_err_to_name(err));
#endif
    }
    else if (row < lcdRows)
    {
        memcpy(s_lcd_last_lines[row], padded, sizeof(s_lcd_last_lines[row]));
        s_lcd_line_cached[row] = true;
    }

    if (s_lcd_mutex != nullptr)
        xSemaphoreGive(s_lcd_mutex);
}

static esp_err_t lcd_try_device_address(uint8_t address)
{
    uint8_t port_state = 0;

    memset(&s_lcd_io, 0, sizeof(s_lcd_io));
    esp_err_t err = pcf8574_init_desc(&s_lcd_io, address, LCD_I2C_PORT, LCD_I2C_SDA_GPIO, LCD_I2C_SCL_GPIO);
    if (err != ESP_OK)
        return err;

    err = pcf8574_port_read(&s_lcd_io, &port_state);
    if (err != ESP_OK)
    {
        pcf8574_free_desc(&s_lcd_io);
        memset(&s_lcd_io, 0, sizeof(s_lcd_io));
        return err;
    }

    s_lcd_addr = address;
    return ESP_OK;
}

static esp_err_t setup_lcd()
{
    init_lcd_descriptor();

    esp_err_t err = i2cdev_init();
    if (err != ESP_OK)
        return err;

    err = lcd_try_device_address(lcdI2CAddressPrimary);
    if (err != ESP_OK)
        err = lcd_try_device_address(lcdI2CAddressSecondary);

    if (err != ESP_OK)
        return err;

    err = hd44780_init(&s_lcd);
    if (err != ESP_OK)
        return err;

    err = hd44780_switch_backlight(&s_lcd, true);
    if (err != ESP_OK)
        return err;

    err = hd44780_control(&s_lcd, true, false, false);
    if (err != ESP_OK)
        return err;

    err = hd44780_clear(&s_lcd);
    if (err != ESP_OK)
        return err;

    s_lcd_ready = true;
    return ESP_OK;
}
#endif

static void display_selected_ip_address(int seconds)
{
    if (s_ip_address[0] == '\0')
    {
        display_line(3, "");
        return;
    }

    if (strcmp(s_ip_address, s_ipv4_address) == 0)
    {
        display_line(3, s_ip_address);
        return;
    }

    char ipv6_part1[IP_ADDRESS_TEXT_SIZE] = "";
    char ipv6_part2[IP_ADDRESS_TEXT_SIZE] = "";
    size_t total_len = strlen(s_ip_address);
    size_t half_len = total_len / 2;
    while (half_len < total_len && s_ip_address[half_len] != ':')
        half_len++;

    size_t remainder_len = total_len - half_len;
    strncpy(ipv6_part1, s_ip_address, half_len);
    ipv6_part1[half_len] = '\0';
    strncpy(ipv6_part2, s_ip_address + half_len, remainder_len);
    ipv6_part2[remainder_len] = '\0';

    if (seconds < 30)
        display_line(3, ipv6_part1);
    else
        display_line(3, ipv6_part2);
}

static void apply_timezone_settings()
{
    setenv("TZ", timeZoneSpec, 1);
    tzset();
}

static void format_local_date_time(time_t utc_time, char *date_string, size_t date_size, char *time_string, size_t time_size)
{
    struct tm local_tm{};
    localtime_r(&utc_time, &local_tm);

    snprintf(date_string, date_size, "%04d-%02d-%02d", local_tm.tm_year + 1900, local_tm.tm_mon + 1, local_tm.tm_mday);

    int hour_value = local_tm.tm_hour % 12;
    if (hour_value == 0)
        hour_value = 12;

    const char *ampm = local_tm.tm_hour < 12 ? "AM" : "PM";
    char zone[8] = "";
    if (displayTimeZone)
        strftime(zone, sizeof(zone), "%Z", &local_tm);

    if (displayTimeZone && zone[0] != '\0')
    {
        snprintf(time_string, time_size, "%d:%02d:%02d %s %s", hour_value, local_tm.tm_min, local_tm.tm_sec, ampm, zone);
    }
    else
    {
        snprintf(time_string, time_size, "%d:%02d:%02d %s", hour_value, local_tm.tm_min, local_tm.tm_sec, ampm);
    }
}

#if UPTIME_RESTART_BUTTON_ENABLED
static void get_uptime(char *buffer, size_t buffer_size)
{
    uint64_t total_seconds = static_cast<uint64_t>(esp_timer_get_time() / 1000000ULL);
    uint64_t days = total_seconds / 86400ULL;
    total_seconds %= 86400ULL;
    uint64_t hours = total_seconds / 3600ULL;
    total_seconds %= 3600ULL;
    uint64_t minutes = total_seconds / 60ULL;
    uint64_t seconds = total_seconds % 60ULL;
    snprintf(buffer, buffer_size, "%llu %02llu:%02llu:%02llu", days, hours, minutes, seconds);
}
#endif

static bool initialize_nvs_storage()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        err = nvs_flash_erase();
        if (err != ESP_OK)
            return false;
        err = nvs_flash_init();
    }

    if (err != ESP_OK)
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
#endif
        return false;
    }

    return true;
}

static bool parse_mac_id_string(const char *text, uint8_t mac[6])
{
    if (text == nullptr || mac == nullptr || text[0] == '\0' || std::strlen(text) != 17)
        return false;

    if (text[2] != ':' || text[5] != ':' || text[8] != ':' || text[11] != ':' || text[14] != ':')
        return false;

    unsigned int values[6] = {};
    if (std::sscanf(text,
                    "%2x:%2x:%2x:%2x:%2x:%2x",
                    &values[0],
                    &values[1],
                    &values[2],
                    &values[3],
                    &values[4],
                    &values[5]) != 6)
        return false;

    for (size_t index = 0; index < 6; ++index)
        mac[index] = static_cast<uint8_t>(values[index]);

    return true;
}

static uint32_t get_highest_candidate_gnss_baud()
{
    return 921600;
}

static void build_candidate_baud_rates(std::vector<uint32_t> &out)
{
    static constexpr uint32_t all_candidate_baud_rates[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

    out.assign(std::begin(all_candidate_baud_rates), std::end(all_candidate_baud_rates));
}

static bool load_gnss_nvs_data(gnss_nvs_data_t *data)
{
    if (data == nullptr)
        return false;

    *data = gnss_nvs_data_t{};

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(gnss_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
        return true;
    if (err != ESP_OK)
        return false;

    bool has_id_type_key = false;
    size_t type_length = 0;
    err = nvs_get_str(handle, gnss_NVS_KEY_ID_TYPE, nullptr, &type_length);
    if (err == ESP_OK && type_length > 0)
    {
        has_id_type_key = true;
        if (type_length > sizeof(data->id_type))
            type_length = sizeof(data->id_type);
        if (nvs_get_str(handle, gnss_NVS_KEY_ID_TYPE, data->id_type, &type_length) == ESP_OK)
            data->has_id_type = data->id_type[0] != '\0';
    }

    bool has_id_value_key = false;
    size_t value_length = 0;
    err = nvs_get_str(handle, gnss_NVS_KEY_ID_VALUE, nullptr, &value_length);
    if (err == ESP_OK && value_length > 0)
    {
        has_id_value_key = true;
        if (value_length > sizeof(data->id_value))
            value_length = sizeof(data->id_value);
        (void)nvs_get_str(handle, gnss_NVS_KEY_ID_VALUE, data->id_value, &value_length);
    }

    bool has_initial_baud_key = nvs_get_u32(handle, gnss_NVS_KEY_INITIAL_BAUD, &data->initial_baud) == ESP_OK;
    bool has_max_baud_key = nvs_get_u32(handle, gnss_NVS_KEY_MAX_BAUD, &data->max_baud) == ESP_OK;

    data->has_stored_data = has_id_type_key || has_id_value_key || has_initial_baud_key || has_max_baud_key;

    nvs_close(handle);
    return true;
}

static bool save_gnss_nvs_data(const gnss_identity_t &identity, uint32_t initial_baud, uint32_t max_baud)
{
    if (!identity.valid || identity.type[0] == '\0' || identity.value[0] == '\0' || initial_baud == 0 || max_baud == 0)
        return false;

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(gnss_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return false;

    uint32_t stored_initial_baud = 0;
    if (nvs_get_u32(handle, gnss_NVS_KEY_INITIAL_BAUD, &stored_initial_baud) == ESP_OK &&
        stored_initial_baud > 0 && stored_initial_baud < initial_baud)
        initial_baud = stored_initial_baud;

    err = nvs_set_str(handle, gnss_NVS_KEY_ID_TYPE, identity.type);
    if (err == ESP_OK)
        err = nvs_set_str(handle, gnss_NVS_KEY_ID_VALUE, identity.value);
    if (err == ESP_OK)
        err = nvs_set_u32(handle, gnss_NVS_KEY_INITIAL_BAUD, initial_baud);
    if (err == ESP_OK)
        err = nvs_set_u32(handle, gnss_NVS_KEY_MAX_BAUD, max_baud);
    if (err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);
    return err == ESP_OK;
}

static bool save_gnss_nvs_initial_baud(uint32_t initial_baud)
{
    if (initial_baud == 0)
        return false;

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(gnss_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK)
    {
        uint32_t stored_initial_baud = 0;
        if (nvs_get_u32(handle, gnss_NVS_KEY_INITIAL_BAUD, &stored_initial_baud) == ESP_OK &&
            stored_initial_baud > 0 && stored_initial_baud < initial_baud)
            initial_baud = stored_initial_baud;
        err = nvs_set_u32(handle, gnss_NVS_KEY_INITIAL_BAUD, initial_baud);
    }
    if (err == ESP_OK)
        err = nvs_commit(handle);

    if (handle != 0)
        nvs_close(handle);
    return err == ESP_OK;
}

static void clear_gnss_nvs_data()
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(gnss_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return;

    err = nvs_erase_key(handle, gnss_NVS_KEY_ID_TYPE);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return;
    }

    err = nvs_erase_key(handle, gnss_NVS_KEY_ID_VALUE);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return;
    }

    err = nvs_erase_key(handle, gnss_NVS_KEY_INITIAL_BAUD);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return;
    }

    err = nvs_erase_key(handle, gnss_NVS_KEY_MAX_BAUD);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return;
    }

    err = nvs_erase_key(handle, gnss_NVS_KEY_ATTEMPT_NO_SIGNAL_RECOVERY);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return;
    }

    (void)nvs_commit(handle);
    nvs_close(handle);
}

static gnss_identity_t query_gnss_identity()
{
    gnss_identity_t identity{};

    UBX_SEC_UNIQID_data_t unique_chip_data{};
    if (s_gnss.getUniqueChipId(&unique_chip_data, 2000))
    {
        const char *unique_chip_id = s_gnss.getUniqueChipIdStr(&unique_chip_data, 2000);
        if (unique_chip_id != nullptr && unique_chip_id[0] != '\0')
        {
            snprintf(identity.type, sizeof(identity.type), "%s", gnss_ID_TYPE_UNIQID);
            snprintf(identity.value, sizeof(identity.value), "%s", unique_chip_id);
            identity.valid = true;
            return identity;
        }
    }

    if (s_gnss.getModuleInfo(2000))
    {
        const char *module_name = s_gnss.getModuleName(2000);
        const char *firmware_type = s_gnss.getFirmwareType(2000);
        uint8_t firmware_high = s_gnss.getFirmwareVersionHigh(2000);
        uint8_t firmware_low = s_gnss.getFirmwareVersionLow(2000);
        uint8_t protocol_high = s_gnss.getProtocolVersionHigh(2000);
        uint8_t protocol_low = s_gnss.getProtocolVersionLow(2000);

        if (module_name != nullptr && module_name[0] != '\0')
        {
            snprintf(identity.type, sizeof(identity.type), "%s", gnss_ID_TYPE_MODULE_FP);
            snprintf(identity.value,
                     sizeof(identity.value),
                     "%s|%s|FW%u.%u|PR%u.%u",
                     module_name,
                     firmware_type == nullptr ? "" : firmware_type,
                     static_cast<unsigned int>(firmware_high),
                     static_cast<unsigned int>(firmware_low),
                     static_cast<unsigned int>(protocol_high),
                     static_cast<unsigned int>(protocol_low));
            identity.valid = true;
            return identity;
        }
    }

    snprintf(identity.type, sizeof(identity.type), "%s", gnss_ID_TYPE_GENERIC);
    snprintf(identity.value, sizeof(identity.value), "%s", gnss_ID_VALUE_UNDETERMINED);
    identity.valid = true;
    return identity;
}

static bool gnss_identity_matches(const gnss_nvs_data_t &stored, const gnss_identity_t &current)
{
    if (!stored.has_id_type || !current.valid)
        return false;

    return strcmp(stored.id_type, current.type) == 0 && strcmp(stored.id_value, current.value) == 0;
}

#if UPTIME_RESTART_BUTTON_ENABLED
static bool check_uptime_request()
{

    static bool button_was_pressed = false;
    static uint32_t button_press_start_ms = 0;

    bool button_pressed = gpio_get_level(static_cast<gpio_num_t>(upTimeRestartPin)) == 0;

    if (button_pressed)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        button_pressed = gpio_get_level(static_cast<gpio_num_t>(upTimeRestartPin)) == 0;
    }

    if (!button_pressed)
    {
        button_was_pressed = false;
        button_press_start_ms = 0;
        return false;
    }

    if (!button_was_pressed)
    {
        button_was_pressed = true;
        button_press_start_ms = millis();
    }
    else if (millis() - button_press_start_ms >= holdUpTimeRestartButtonForThisManySecondsToTriggerAReset * 1000UL)
    {
        clear_gnss_nvs_data();
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "Uptime/reset button restart requested: cleared stored GNSS NVS values.");
#endif
#if MQTT_ENABLED
        mqtt_publish_final_report();
#endif
        esp_restart();
    }

    return true;
}
#endif

static int64_t days_from_civil(int year, unsigned month, unsigned day)
{
    year -= month <= 2;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(year - era * 400);
    const unsigned doy = (153U * (month + (month > 2 ? static_cast<unsigned>(-3) : 9U)) + 2U) / 5U + day - 1U;
    const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
    return static_cast<int64_t>(era) * 146097LL + static_cast<int64_t>(doe) - 719468LL;
}

static time_t epoch_from_utc(int year, int month, int day, int hour, int minute, int second)
{
    int64_t days = days_from_civil(year, static_cast<unsigned>(month), static_cast<unsigned>(day));
    int64_t seconds = days * 86400LL + hour * 3600LL + minute * 60LL + second;
    return static_cast<time_t>(seconds);
}

static uint64_t get_current_time_in_ntp64_format()

{

    struct timeval now{};

    gettimeofday(&now, nullptr);

    uint64_t seconds = NTP_EPOCH_OFFSET + static_cast<uint64_t>(now.tv_sec);

    uint64_t fraction = (static_cast<uint64_t>(now.tv_usec) << 32) / 1000000ULL;

    return (seconds << 32) | fraction;
}

static void write_ntp_timestamp(uint8_t *reply, size_t offset, uint64_t timestamp)
{
    reply[offset + 0] = static_cast<uint8_t>((timestamp >> 56) & 0xFF);
    reply[offset + 1] = static_cast<uint8_t>((timestamp >> 48) & 0xFF);
    reply[offset + 2] = static_cast<uint8_t>((timestamp >> 40) & 0xFF);
    reply[offset + 3] = static_cast<uint8_t>((timestamp >> 32) & 0xFF);
    reply[offset + 4] = static_cast<uint8_t>((timestamp >> 24) & 0xFF);
    reply[offset + 5] = static_cast<uint8_t>((timestamp >> 16) & 0xFF);
    reply[offset + 6] = static_cast<uint8_t>((timestamp >> 8) & 0xFF);
    reply[offset + 7] = static_cast<uint8_t>(timestamp & 0xFF);
}

struct ntp_reply_status_t
{
    uint8_t leap_indicator = 3;
    uint8_t stratum = 16;
    const char *reference_id = "INIT";
    bool reference_time_valid = false;
    bool gnss_synchronized = false;
    bool pps_disciplined = false;
};

static ntp_reply_status_t get_ntp_reply_status()
{
    for (size_t attempt = 0; attempt < 3; ++attempt)
    {
        uint32_t sequence_before = s_ntp_sync_state_sequence.load(std::memory_order_acquire);
        if ((sequence_before & 1U) != 0)
            continue;

        int64_t last_successful_sync_us = s_ntp_last_successful_sync_us.load(std::memory_order_relaxed);
        int64_t last_gnss_valid_us = s_ntp_last_gnss_valid_us.load(std::memory_order_relaxed);
        bool pps_active = s_ntp_pps_active.load(std::memory_order_relaxed);
        bool gnss_timing_valid = s_ntp_gnss_timing_valid.load(std::memory_order_relaxed);
        sync_faults_t faults{
            s_ntp_pps_missing.load(std::memory_order_relaxed),
            s_ntp_gnss_invalid.load(std::memory_order_relaxed),
            s_ntp_sanity_mismatch.load(std::memory_order_relaxed),
            s_ntp_sync_stale.load(std::memory_order_relaxed)};

        if (sequence_before != s_ntp_sync_state_sequence.load(std::memory_order_acquire))
            continue;

        int64_t now_us = esp_timer_get_time();
        bool gnss_recent = gnss_timing_valid && last_gnss_valid_us > 0 &&
                           (now_us - last_gnss_valid_us) <= Gnss_Validity_Timeout_Us;
        bool sync_stale = last_successful_sync_us > 0 &&
                          (now_us - last_successful_sync_us) > Sync_Stale_After_Us;
        bool gnss_synchronized = gnss_recent && !sync_stale && !faults.gnss_invalid;
        bool stratum_one = s_time_has_been_set.load(std::memory_order_acquire) &&
                           !s_time_setting_in_progress.load(std::memory_order_acquire) &&
                           !has_sync_fault(faults) && pps_active && gnss_synchronized &&
                           s_ntp_reference_valid.load(std::memory_order_acquire);

        if (stratum_one)
            return {0, 1, "GPS", true, gnss_synchronized, pps_active};
        return {3, 16, "INIT", false, gnss_synchronized, pps_active};
    }

    return {};
}

static constexpr uint8_t NTP_REPLY_TEMPLATE[NTP_PACKET_SIZE] = {
    0, 0, 4, static_cast<uint8_t>(NTP_PRECISION_EXPONENT),
    0, 0, 0, 0,
    0, 0, 0, static_cast<uint8_t>(NTP_ROOT_DISPERSION)};

static void build_ntp_reply(const uint8_t *request, uint8_t *reply, uint8_t version, uint64_t receive_time, const ntp_reply_status_t &status)
{
    memcpy(reply, NTP_REPLY_TEMPLATE, sizeof(NTP_REPLY_TEMPLATE));

    reply[0] = static_cast<uint8_t>((status.leap_indicator << 6) | (version << 3) | 4);
    reply[1] = status.stratum;
    memcpy(reply + 12, status.reference_id, 4);

    if (status.reference_time_valid)
        write_ntp_timestamp(reply, 16, s_ntp_reference_time_64.load(std::memory_order_acquire));

    memcpy(reply + 24, request + 40, 8);
    write_ntp_timestamp(reply, 32, receive_time);
}

static constexpr size_t ETH_HEADER_SIZE = 14;
static constexpr size_t IPV4_HEADER_SIZE = 20;
static constexpr size_t IPV6_HEADER_SIZE = 40;
static constexpr size_t UDP_HEADER_SIZE = 8;
static constexpr size_t RAW_NTP_FRAME_SIZE = ETH_HEADER_SIZE + IPV4_HEADER_SIZE + UDP_HEADER_SIZE + NTP_PACKET_SIZE;
static constexpr size_t RAW_IPV6_NTP_FRAME_SIZE = ETH_HEADER_SIZE + IPV6_HEADER_SIZE + UDP_HEADER_SIZE + NTP_PACKET_SIZE;
static constexpr size_t HARDWARE_NTP_REQUEST_QUEUE_DEPTH = 256;
static constexpr size_t HARDWARE_NTP_REQUEST_BUFFER_COUNT = 64; // Do not increase this value, larger pools caused unacceptable internal/DMA-memory pressure
static constexpr size_t HARDWARE_NTP_REQUEST_BUFFER_SIZE = ETH_HEADER_SIZE + IPV6_HEADER_SIZE + 60 + UDP_HEADER_SIZE + NTP_PACKET_SIZE;
static constexpr uint32_t HARDWARE_NTP_TRANSMIT_RETRY_COUNT = 3;
static constexpr uint32_t HARDWARE_NTP_TRANSMIT_RETRY_DELAY_US = 10;
static constexpr uint32_t ETHERNET_RECOVERY_DELAY_MS = 100;
static constexpr uint32_t ETHERNET_RECOVERY_STOP_TIMEOUT_MS = 1000;
static constexpr uint32_t ETHERNET_RECOVERY_IP_TIMEOUT_MS = 10000;
static constexpr int64_t NTP_TRANSPORT_STALL_TIMEOUT_US = 35000000LL;

struct hardware_ntp_request_t
{
    esp_eth_handle_t handle;
    uint8_t *frame;
    uint32_t length;
    esp_netif_t *netif;
    eth_mac_time_t rx_timestamp;
};

static QueueHandle_t s_hardware_ntp_request_queue = nullptr;
static QueueHandle_t s_hardware_ntp_request_buffer_queue = nullptr;
static TaskHandle_t s_ntp_cache_purge_task_handle = nullptr;
static TaskHandle_t s_hardware_ntp_server_task_handle = nullptr;
static uint8_t s_hardware_ntp_request_buffers[HARDWARE_NTP_REQUEST_BUFFER_COUNT][HARDWARE_NTP_REQUEST_BUFFER_SIZE]{};

static bool initialize_hardware_ntp_request_buffers()
{
    for (size_t index = 0; index < HARDWARE_NTP_REQUEST_BUFFER_COUNT; ++index)
    {
        uint8_t *buffer = s_hardware_ntp_request_buffers[index];
        if (xQueueSend(s_hardware_ntp_request_buffer_queue, &buffer, 0) != pdTRUE)
            return false;
    }
    return true;
}

static void release_hardware_ntp_request_buffer(uint8_t *buffer)
{
    if (buffer != nullptr && s_hardware_ntp_request_buffer_queue != nullptr)
        xQueueSend(s_hardware_ntp_request_buffer_queue, &buffer, 0);
}

static void deinitialize_hardware_ntp_server()
{
    s_hardware_ntp_accepting.store(false, std::memory_order_release);
    if (s_hardware_ntp_server_task_handle != nullptr)
    {
        vTaskDelete(s_hardware_ntp_server_task_handle);
        s_hardware_ntp_server_task_handle = nullptr;
    }
    if (s_ntp_cache_purge_task_handle != nullptr)
    {
        vTaskDelete(s_ntp_cache_purge_task_handle);
        s_ntp_cache_purge_task_handle = nullptr;
    }
    if (s_hardware_ntp_request_queue != nullptr)
    {
        vQueueDelete(s_hardware_ntp_request_queue);
        s_hardware_ntp_request_queue = nullptr;
    }
    if (s_hardware_ntp_request_buffer_queue != nullptr)
    {
        vQueueDelete(s_hardware_ntp_request_buffer_queue);
        s_hardware_ntp_request_buffer_queue = nullptr;
    }
    if (s_hardware_ntp_transmit_mutex != nullptr)
    {
        vSemaphoreDelete(s_hardware_ntp_transmit_mutex);
        s_hardware_ntp_transmit_mutex = nullptr;
    }
    ntp_cache_deinit();
}

static uint16_t internet_checksum(const uint8_t *data, size_t length)
{
    uint32_t sum = 0;
    while (length >= 2)
    {
        sum += static_cast<uint16_t>((data[0] << 8) | data[1]);
        data += 2;
        length -= 2;
    }
    if (length != 0)
        sum += static_cast<uint16_t>(data[0] << 8);
    while ((sum >> 16) != 0)
        sum = (sum & 0xFFFFU) + (sum >> 16);
    return static_cast<uint16_t>(~sum);
}

static uint64_t ntp64_from_hardware_timestamp(const eth_mac_time_t &timestamp)
{
    const uint64_t seconds = NTP_EPOCH_OFFSET + timestamp.seconds;
    const uint64_t fraction = (static_cast<uint64_t>(timestamp.nanoseconds) << 32) / 1000000000ULL;
    return (seconds << 32) | fraction;
}

static bool timestamp_is_valid(const eth_mac_time_t *timestamp)
{
    return timestamp != nullptr && (timestamp->seconds != 0 || timestamp->nanoseconds != 0);
}

static bool is_ipv4_ntp_request(const uint8_t *frame, uint32_t length, size_t *ip_offset, size_t *udp_offset)
{
    if (frame == nullptr || length < RAW_NTP_FRAME_SIZE || frame[12] != 0x08 || frame[13] != 0x00)
        return false;

    const size_t ipv4_offset = ETH_HEADER_SIZE;
    const uint8_t version_ihl = frame[ipv4_offset];
    const size_t header_length = static_cast<size_t>(version_ihl & 0x0F) * 4;
    if ((version_ihl >> 4) != 4 || header_length < IPV4_HEADER_SIZE || length < ETH_HEADER_SIZE + header_length + UDP_HEADER_SIZE + NTP_PACKET_SIZE)
        return false;
    if (frame[ipv4_offset + 9] != IPPROTO_UDP)
        return false;

    const size_t udp_header_offset = ipv4_offset + header_length;
    const uint16_t udp_length = static_cast<uint16_t>((frame[udp_header_offset + 4] << 8) | frame[udp_header_offset + 5]);
    const uint16_t destination_port = static_cast<uint16_t>((frame[udp_header_offset + 2] << 8) | frame[udp_header_offset + 3]);
    if (destination_port != NTP_PORT || udp_length != UDP_HEADER_SIZE + NTP_PACKET_SIZE)
        return false;

    *ip_offset = ipv4_offset;
    *udp_offset = udp_header_offset;
    return true;
}

static bool is_ipv6_extension_header(uint8_t next_header)
{
    return next_header == 0 || next_header == 43 || next_header == 60 || next_header == 135 || next_header == 51;
}

static bool is_ipv6_ntp_request(const uint8_t *frame, uint32_t length, size_t *ip_offset, size_t *udp_offset)
{
    if (frame == nullptr || length < RAW_IPV6_NTP_FRAME_SIZE || frame[12] != 0x86 || frame[13] != 0xDD)
        return false;

    const size_t ipv6_offset = ETH_HEADER_SIZE;
    if ((frame[ipv6_offset] >> 4) != 6)
        return false;

    const size_t payload_length = static_cast<size_t>((frame[ipv6_offset + 4] << 8) | frame[ipv6_offset + 5]);
    const size_t payload_end = ipv6_offset + IPV6_HEADER_SIZE + payload_length;
    if (payload_end > length)
        return false;

    uint8_t next_header = frame[ipv6_offset + 6];
    size_t offset = ipv6_offset + IPV6_HEADER_SIZE;
    for (size_t count = 0; is_ipv6_extension_header(next_header); ++count)
    {
        if (count == 8 || offset + 2 > payload_end)
            return false;
        const size_t extension_length = next_header == 51
                                            ? static_cast<size_t>(frame[offset + 1] + 2) * 4
                                            : static_cast<size_t>(frame[offset + 1] + 1) * 8;
        if (extension_length < 8 || offset + extension_length > payload_end)
            return false;
        next_header = frame[offset];
        offset += extension_length;
    }

    if (next_header != IPPROTO_UDP || offset + UDP_HEADER_SIZE + NTP_PACKET_SIZE > payload_end)
        return false;

    const uint16_t udp_length = static_cast<uint16_t>((frame[offset + 4] << 8) | frame[offset + 5]);
    const uint16_t destination_port = static_cast<uint16_t>((frame[offset + 2] << 8) | frame[offset + 3]);
    if (destination_port != NTP_PORT || udp_length != UDP_HEADER_SIZE + NTP_PACKET_SIZE ||
        offset + udp_length != payload_end)
        return false;

    *ip_offset = ipv6_offset;
    *udp_offset = offset;
    return true;
}

static bool is_internal_ntp_ipv4_address(const struct in_addr &address)
{
    if (address.s_addr == htonl(INADDR_LOOPBACK))
        return true;

    struct in_addr assigned_address{};
    return s_ipv4_address[0] != '\0' && inet_pton(AF_INET, s_ipv4_address, &assigned_address) == 1 &&
           address.s_addr == assigned_address.s_addr;
}

static bool is_internal_ntp_ipv6_address(const struct in6_addr &address)
{
    struct in6_addr loopback_address{};
    if (inet_pton(AF_INET6, "::1", &loopback_address) == 1 &&
        memcmp(&address, &loopback_address, sizeof(address)) == 0)
        return true;

    struct in6_addr assigned_address{};
    return s_ipv6_address[0] != '\0' && inet_pton(AF_INET6, s_ipv6_address, &assigned_address) == 1 &&
           memcmp(&address, &assigned_address, sizeof(address)) == 0;
}

static bool is_internal_ntp_client(const struct sockaddr_storage &address)
{
    if (address.ss_family == AF_INET)
        return is_internal_ntp_ipv4_address(reinterpret_cast<const struct sockaddr_in *>(&address)->sin_addr);
    if (address.ss_family == AF_INET6)
        return is_internal_ntp_ipv6_address(reinterpret_cast<const struct sockaddr_in6 *>(&address)->sin6_addr);
    return false;
}

static uint16_t ipv6_udp_checksum(const uint8_t *source, const uint8_t *destination, const uint8_t *udp, size_t udp_length)
{
    uint8_t checksum_data[IPV6_HEADER_SIZE + UDP_HEADER_SIZE + NTP_PACKET_SIZE] = {};
    memcpy(checksum_data, source, 16);
    memcpy(checksum_data + 16, destination, 16);
    checksum_data[35] = static_cast<uint8_t>(udp_length);
    checksum_data[39] = IPPROTO_UDP;
    memcpy(checksum_data + IPV6_HEADER_SIZE, udp, udp_length);
    const uint16_t checksum = internet_checksum(checksum_data, IPV6_HEADER_SIZE + udp_length);
    return checksum == 0 ? 0xFFFF : checksum;
}

static uint64_t read_ntp_timestamp(const uint8_t *packet, size_t offset);

static esp_err_t process_hardware_ntp_request(esp_eth_handle_t handle, uint8_t *frame, uint32_t length, void *netif, void *info)
{
    size_t ip_offset = 0;
    size_t udp_offset = 0;
    const eth_mac_time_t *rx_timestamp = static_cast<const eth_mac_time_t *>(info);
    if (!is_ipv4_ntp_request(frame, length, &ip_offset, &udp_offset) || !timestamp_is_valid(rx_timestamp))
    {
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }

    const eth_mac_time_t receive_timestamp = *rx_timestamp;
    const uint8_t *request = frame + udp_offset + UDP_HEADER_SIZE;
    const uint8_t version = (request[0] >> 3) & 0x07;
    const uint8_t mode = request[0] & 0x07;
    if (version < 3 || version > 4 || mode != 3)
    {
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }

    const uint32_t client_ip = (static_cast<uint32_t>(frame[ip_offset + 12]) << 24) |
                               (static_cast<uint32_t>(frame[ip_offset + 13]) << 16) |
                               (static_cast<uint32_t>(frame[ip_offset + 14]) << 8) |
                               frame[ip_offset + 15];
    const uint16_t client_port = static_cast<uint16_t>((frame[udp_offset] << 8) | frame[udp_offset + 1]);
    ntp_client_record_t client_record{};
    if (!ntp_cache_find_or_create(client_ip, client_port, &client_record))
    {
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }

    const uint64_t receive_time = ntp64_from_hardware_timestamp(receive_timestamp);
#if MQTT_ENABLED
    struct sockaddr_storage source_address{};
    auto *source_ipv4_address = reinterpret_cast<struct sockaddr_in *>(&source_address);
    source_ipv4_address->sin_family = AF_INET;
    source_ipv4_address->sin_port = htons(client_port);
    memcpy(&source_ipv4_address->sin_addr, frame + ip_offset + 12, sizeof(source_ipv4_address->sin_addr));
    s_ntp_valid_requests.fetch_add(1, std::memory_order_relaxed);
    mqtt_enqueue_ntp_request(source_address);
#endif
    uint8_t reply[NTP_PACKET_SIZE] = {};
    const ntp_reply_status_t status = get_ntp_reply_status();
#if MQTT_ENABLED
    if (status.gnss_synchronized && status.pps_disciplined)
        s_ntp_responses_synchronized_and_disciplined.fetch_add(1, std::memory_order_relaxed);
    if (!status.gnss_synchronized)
        s_ntp_responses_gnss_unsynchronized.fetch_add(1, std::memory_order_relaxed);
    if (!status.pps_disciplined)
        s_ntp_responses_pps_undisciplined.fetch_add(1, std::memory_order_relaxed);
#endif
    build_ntp_reply(request, reply, version, receive_time, status);

    const uint64_t client_receive_timestamp =
        (static_cast<uint64_t>(request[32]) << 56) | (static_cast<uint64_t>(request[33]) << 48) |
        (static_cast<uint64_t>(request[34]) << 40) | (static_cast<uint64_t>(request[35]) << 32) |
        (static_cast<uint64_t>(request[36]) << 24) | (static_cast<uint64_t>(request[37]) << 16) |
        (static_cast<uint64_t>(request[38]) << 8) | request[39];
    const uint64_t client_origin_timestamp =
        (static_cast<uint64_t>(request[24]) << 56) | (static_cast<uint64_t>(request[25]) << 48) |
        (static_cast<uint64_t>(request[26]) << 40) | (static_cast<uint64_t>(request[27]) << 32) |
        (static_cast<uint64_t>(request[28]) << 24) | (static_cast<uint64_t>(request[29]) << 16) |
        (static_cast<uint64_t>(request[30]) << 8) | request[31];
    const uint64_t client_transmit_timestamp =
        (static_cast<uint64_t>(request[40]) << 56) | (static_cast<uint64_t>(request[41]) << 48) |
        (static_cast<uint64_t>(request[42]) << 40) | (static_cast<uint64_t>(request[43]) << 32) |
        (static_cast<uint64_t>(request[44]) << 24) | (static_cast<uint64_t>(request[45]) << 16) |
        (static_cast<uint64_t>(request[46]) << 8) | request[47];
    const uint64_t previous_t2 = client_record.prev_t2;
    const uint64_t previous_t3 = client_record.prev_t3;
    const bool interleaved_reply = previous_t2 != 0 &&
                                   client_receive_timestamp != client_transmit_timestamp &&
                                   client_origin_timestamp == previous_t2;
    if (interleaved_reply)
    {
        write_ntp_timestamp(reply, 24, client_receive_timestamp);
        write_ntp_timestamp(reply, 40, previous_t3);
    }
    else
    {
        write_ntp_timestamp(reply, 40, get_current_time_in_ntp64_format());
    }

    uint8_t response_frame[RAW_NTP_FRAME_SIZE] = {};
    memcpy(response_frame, frame + 6, 6);
    memcpy(response_frame + 6, frame, 6);
    response_frame[12] = 0x08;
    response_frame[13] = 0x00;
    uint8_t *ip = response_frame + ETH_HEADER_SIZE;
    ip[0] = 0x45;
    ip[2] = 0;
    ip[3] = IPV4_HEADER_SIZE + UDP_HEADER_SIZE + NTP_PACKET_SIZE;
    ip[8] = 64;
    ip[9] = IPPROTO_UDP;
    memcpy(ip + 12, frame + ip_offset + 16, 4);
    memcpy(ip + 16, frame + ip_offset + 12, 4);
    const uint16_t checksum = internet_checksum(ip, IPV4_HEADER_SIZE);
    ip[10] = static_cast<uint8_t>(checksum >> 8);
    ip[11] = static_cast<uint8_t>(checksum);
    uint8_t *udp = ip + IPV4_HEADER_SIZE;
    udp[0] = 0;
    udp[1] = static_cast<uint8_t>(NTP_PORT);
    udp[2] = frame[udp_offset];
    udp[3] = frame[udp_offset + 1];
    udp[4] = 0;
    udp[5] = UDP_HEADER_SIZE + NTP_PACKET_SIZE;
    memcpy(udp + UDP_HEADER_SIZE, reply, sizeof(reply));

    esp_err_t result = ESP_ERR_INVALID_STATE;
    if (s_hardware_ntp_accepting.load(std::memory_order_acquire) &&
        s_hardware_ntp_transmit_mutex != nullptr)
    {
        if (xSemaphoreTake(s_hardware_ntp_transmit_mutex, portMAX_DELAY) == pdTRUE)
        {
            if (s_hardware_ntp_accepting.load(std::memory_order_acquire))
            {
                for (uint32_t attempt = 0; attempt < HARDWARE_NTP_TRANSMIT_RETRY_COUNT; ++attempt)
                {
                    if (!interleaved_reply)
                    {
                        write_ntp_timestamp(reply, 40, get_current_time_in_ntp64_format());
                        memcpy(udp + UDP_HEADER_SIZE, reply, sizeof(reply));
                    }

                    result = esp_eth_transmit(handle, response_frame, sizeof(response_frame));
                    if (result != ESP_ERR_NO_MEM)
                        break;
                    esp_rom_delay_us(HARDWARE_NTP_TRANSMIT_RETRY_DELAY_US);
                }
            }
            xSemaphoreGive(s_hardware_ntp_transmit_mutex);
        }
    }
    if (result == ESP_OK)
    {
        ntp_cache_update(client_ip, client_port, receive_time, read_ntp_timestamp(reply, 40));
        s_last_hardware_ntp_response_us.store(esp_timer_get_time(), std::memory_order_release);
#if MQTT_ENABLED
        s_ntp_responses.fetch_add(1, std::memory_order_relaxed);
#endif
    }

    release_hardware_ntp_request_buffer(frame);
    return result;
}

static uint64_t read_ntp_timestamp(const uint8_t *packet, size_t offset)
{
    return (static_cast<uint64_t>(packet[offset]) << 56) |
           (static_cast<uint64_t>(packet[offset + 1]) << 48) |
           (static_cast<uint64_t>(packet[offset + 2]) << 40) |
           (static_cast<uint64_t>(packet[offset + 3]) << 32) |
           (static_cast<uint64_t>(packet[offset + 4]) << 24) |
           (static_cast<uint64_t>(packet[offset + 5]) << 16) |
           (static_cast<uint64_t>(packet[offset + 6]) << 8) |
           static_cast<uint64_t>(packet[offset + 7]);
}

static esp_err_t process_hardware_ipv6_ntp_request(esp_eth_handle_t handle, uint8_t *frame, uint32_t length, void *netif, void *info)
{
    size_t ip_offset = 0;
    size_t udp_offset = 0;
    const eth_mac_time_t *rx_timestamp = static_cast<const eth_mac_time_t *>(info);
    if (!is_ipv6_ntp_request(frame, length, &ip_offset, &udp_offset) || !timestamp_is_valid(rx_timestamp))
    {
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }

    const eth_mac_time_t receive_timestamp = *rx_timestamp;
    const uint8_t *request = frame + udp_offset + UDP_HEADER_SIZE;
    const uint8_t version = (request[0] >> 3) & 0x07;
    const uint8_t mode = request[0] & 0x07;
    if (version < 3 || version > 4 || mode != 3)
    {
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }

    struct in6_addr client_ip{};
    memcpy(&client_ip, frame + ip_offset + 8, sizeof(client_ip));
    const uint16_t client_port = static_cast<uint16_t>((frame[udp_offset] << 8) | frame[udp_offset + 1]);
    ntp_client_record_ipv6_t client_record{};
    if (!ntp_cache_find_or_create_ipv6(&client_ip, client_port, &client_record))
    {
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }

    const uint64_t receive_time = ntp64_from_hardware_timestamp(receive_timestamp);
#if MQTT_ENABLED
    struct sockaddr_storage source_address{};
    auto *source_ipv6_address = reinterpret_cast<struct sockaddr_in6 *>(&source_address);
    source_ipv6_address->sin6_family = AF_INET6;
    source_ipv6_address->sin6_port = htons(client_port);
    source_ipv6_address->sin6_scope_id = esp_netif_get_netif_impl_index(static_cast<esp_netif_t *>(netif));
    source_ipv6_address->sin6_addr = client_ip;
    s_ntp_valid_requests.fetch_add(1, std::memory_order_relaxed);
    mqtt_enqueue_ntp_request(source_address);
#endif
    uint8_t reply[NTP_PACKET_SIZE] = {};
    const ntp_reply_status_t status = get_ntp_reply_status();
#if MQTT_ENABLED
    if (status.gnss_synchronized && status.pps_disciplined)
        s_ntp_responses_synchronized_and_disciplined.fetch_add(1, std::memory_order_relaxed);
    if (!status.gnss_synchronized)
        s_ntp_responses_gnss_unsynchronized.fetch_add(1, std::memory_order_relaxed);
    if (!status.pps_disciplined)
        s_ntp_responses_pps_undisciplined.fetch_add(1, std::memory_order_relaxed);
#endif
    build_ntp_reply(request, reply, version, receive_time, status);

    const uint64_t client_receive_timestamp = read_ntp_timestamp(request, 32);
    const uint64_t client_origin_timestamp = read_ntp_timestamp(request, 24);
    const uint64_t client_transmit_timestamp = read_ntp_timestamp(request, 40);
    const bool interleaved_reply = client_record.prev_t2 != 0 &&
                                   client_receive_timestamp != client_transmit_timestamp &&
                                   client_origin_timestamp == client_record.prev_t2;
    if (interleaved_reply)
    {
        write_ntp_timestamp(reply, 24, client_receive_timestamp);
        write_ntp_timestamp(reply, 40, client_record.prev_t3);
    }
    else
    {
        write_ntp_timestamp(reply, 40, get_current_time_in_ntp64_format());
    }

    uint8_t response_frame[RAW_IPV6_NTP_FRAME_SIZE] = {};
    memcpy(response_frame, frame + 6, 6);
    memcpy(response_frame + 6, frame, 6);
    response_frame[12] = 0x86;
    response_frame[13] = 0xDD;
    uint8_t *ip = response_frame + ETH_HEADER_SIZE;
    ip[0] = 0x60;
    ip[5] = UDP_HEADER_SIZE + NTP_PACKET_SIZE;
    ip[6] = IPPROTO_UDP;
    ip[7] = 64;
    memcpy(ip + 8, frame + ip_offset + 24, 16);
    memcpy(ip + 24, frame + ip_offset + 8, 16);
    uint8_t *udp = ip + IPV6_HEADER_SIZE;
    udp[1] = static_cast<uint8_t>(NTP_PORT);
    udp[2] = frame[udp_offset];
    udp[3] = frame[udp_offset + 1];
    udp[5] = UDP_HEADER_SIZE + NTP_PACKET_SIZE;
    memcpy(udp + UDP_HEADER_SIZE, reply, sizeof(reply));
    const uint16_t checksum = ipv6_udp_checksum(ip + 8, ip + 24, udp, UDP_HEADER_SIZE + NTP_PACKET_SIZE);
    udp[6] = static_cast<uint8_t>(checksum >> 8);
    udp[7] = static_cast<uint8_t>(checksum);

    esp_err_t result = ESP_ERR_INVALID_STATE;
    if (s_hardware_ntp_accepting.load(std::memory_order_acquire) && s_hardware_ntp_transmit_mutex != nullptr &&
        xSemaphoreTake(s_hardware_ntp_transmit_mutex, portMAX_DELAY) == pdTRUE)
    {
        if (s_hardware_ntp_accepting.load(std::memory_order_acquire))
        {
            for (uint32_t attempt = 0; attempt < HARDWARE_NTP_TRANSMIT_RETRY_COUNT; ++attempt)
            {
                if (!interleaved_reply)
                {
                    write_ntp_timestamp(reply, 40, get_current_time_in_ntp64_format());
                    memcpy(udp + UDP_HEADER_SIZE, reply, sizeof(reply));
                    udp[6] = 0;
                    udp[7] = 0;
                    const uint16_t refreshed_checksum = ipv6_udp_checksum(ip + 8, ip + 24, udp, UDP_HEADER_SIZE + NTP_PACKET_SIZE);
                    udp[6] = static_cast<uint8_t>(refreshed_checksum >> 8);
                    udp[7] = static_cast<uint8_t>(refreshed_checksum);
                }
                result = esp_eth_transmit(handle, response_frame, sizeof(response_frame));
                if (result != ESP_ERR_NO_MEM)
                    break;
                esp_rom_delay_us(HARDWARE_NTP_TRANSMIT_RETRY_DELAY_US);
            }
        }
        xSemaphoreGive(s_hardware_ntp_transmit_mutex);
    }
    if (result == ESP_OK)
    {
        ntp_cache_update_ipv6(&client_ip, client_port, receive_time, read_ntp_timestamp(reply, 40));
        s_last_hardware_ntp_response_us.store(esp_timer_get_time(), std::memory_order_release);
#if MQTT_ENABLED
        s_ntp_responses.fetch_add(1, std::memory_order_relaxed);
#endif
    }

    release_hardware_ntp_request_buffer(frame);
    return result;
}

static esp_err_t ntp_ethernet_input(esp_eth_handle_t handle, uint8_t *frame, uint32_t length, void *netif, void *info)
{
    size_t ip_offset = 0;
    size_t udp_offset = 0;
    const eth_mac_time_t *rx_timestamp = static_cast<const eth_mac_time_t *>(info);
    if (!is_ipv4_ntp_request(frame, length, &ip_offset, &udp_offset) &&
        !is_ipv6_ntp_request(frame, length, &ip_offset, &udp_offset))
        return esp_netif_receive(static_cast<esp_netif_t *>(netif), frame, length, nullptr);
    const uint8_t *request = frame + udp_offset + UDP_HEADER_SIZE;
    const uint8_t version = (request[0] >> 3) & 0x07;
    const uint8_t mode = request[0] & 0x07;
    if (version < 3 || version > 4 || mode != 3)
    {
        return esp_netif_receive(static_cast<esp_netif_t *>(netif), frame, length, nullptr);
    }

    bool internal_request = false;
    if (frame[12] == 0x08 && frame[13] == 0x00)
    {
        struct in_addr source_address{};
        memcpy(&source_address, frame + ip_offset + 12, sizeof(source_address));
        internal_request = is_internal_ntp_ipv4_address(source_address);
    }
    else
    {
        struct in6_addr source_address{};
        memcpy(&source_address, frame + ip_offset + 8, sizeof(source_address));
        internal_request = is_internal_ntp_ipv6_address(source_address);
    }
    if (!s_ntp_external_responses_enabled.load(std::memory_order_acquire) && !internal_request)
    {
        free(frame);
        return ESP_OK;
    }

    if (!timestamp_is_valid(rx_timestamp))
    {
        free(frame);
        return ESP_OK;
    }

#if MQTT_ENABLED
    s_ntp_requests_this_second.fetch_add(1, std::memory_order_relaxed);
#endif
    const eth_mac_time_t receive_timestamp = *rx_timestamp;
    if (!s_hardware_ntp_accepting.load(std::memory_order_acquire) || s_hardware_ntp_request_queue == nullptr ||
        s_hardware_ntp_request_buffer_queue == nullptr)
    {
        free(frame);
    }
    else
    {
        uint8_t *request_buffer = nullptr;
        if (length > HARDWARE_NTP_REQUEST_BUFFER_SIZE ||
            xQueueReceive(s_hardware_ntp_request_buffer_queue, &request_buffer, 0) != pdTRUE)
        {
            free(frame);
            return ESP_OK;
        }
        memcpy(request_buffer, frame, length);
        free(frame);

        const hardware_ntp_request_t ntp_request{handle, request_buffer, length, static_cast<esp_netif_t *>(netif), receive_timestamp};
        if (xQueueSend(s_hardware_ntp_request_queue, &ntp_request, 0) != pdTRUE)
        {
            release_hardware_ntp_request_buffer(request_buffer);
        }
    }
    return ESP_OK;
}

static void ntp_cache_purge_task(void *parameter)
{
    (void)parameter;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(60000));
        ntp_cache_purge_expired();

#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(NTP_Cache_Purge);
#endif
    }
}

// Handles NTP requests from IPv4 sources using Ethernet hardware timestamps to provide high-precision, GNSS-disciplined time responses.
// If the hardware timestamp is not available or valid, the request will be handled by the standard NTP server (ntp_server_task).
static void hardware_ntp_server_task(void *parameter)
{
    (void)parameter;
    hardware_ntp_request_t ntp_request{};
    for (;;)
    {
        if (xQueueReceive(s_hardware_ntp_request_queue, &ntp_request, portMAX_DELAY) == pdTRUE)
        {
            if (ntp_request.frame[12] == 0x86 && ntp_request.frame[13] == 0xDD)
                process_hardware_ipv6_ntp_request(ntp_request.handle, ntp_request.frame, ntp_request.length,
                                                  ntp_request.netif, &ntp_request.rx_timestamp);
            else
                process_hardware_ntp_request(ntp_request.handle, ntp_request.frame, ntp_request.length,
                                             ntp_request.netif, &ntp_request.rx_timestamp);
#if CALCULATE_STACK_SIZES_ENABLED
            report_current_task_stack_usage(Hardware_NTP_Server);
#endif
        }
    }
}

#if MQTT_ENABLED
static void recover_ethernet_transport()
{
    if (ETH.handle() == nullptr || s_hardware_ntp_transmit_mutex == nullptr ||
        xSemaphoreTake(s_hardware_ntp_transmit_mutex, portMAX_DELAY) != pdTRUE)
        return;

    ESP_LOGW(TAG, "MQTT and NTP transport stalled; restarting Ethernet");
    s_hardware_ntp_accepting.store(false, std::memory_order_release);
    hardware_ntp_request_t request{};
    while (xQueueReceive(s_hardware_ntp_request_queue, &request, 0) == pdTRUE)
    {
        release_hardware_ntp_request_buffer(request.frame);
    }

    s_ptp_clock_ready.store(false, std::memory_order_release);
    s_hardware_ntp_accepting.store(false, std::memory_order_release);
    xSemaphoreGive(s_hardware_ntp_transmit_mutex);

    if (mqtt_publish_or_queue_restart_notification("ethernet_transport_stalled"))
    {
        ESP_LOGW(TAG, "Restarting after NTP transport stall");
        esp_restart();
    }

    ESP_LOGE(TAG, "Ethernet transport restart notification could not be published or queued");
}

static void ethernet_transport_recovery_task(void *parameter)
{
    (void)parameter;
    for (;;)
    {
        const int64_t now_us = esp_timer_get_time();
        const int64_t disconnected_since_us = s_mqtt_disconnected_since_us.load(std::memory_order_acquire);
        const int64_t last_ntp_response_us = s_last_hardware_ntp_response_us.load(std::memory_order_acquire);
        if (s_mqtt_has_connected.load(std::memory_order_acquire) &&
            !s_mqtt_connected.load(std::memory_order_acquire) && disconnected_since_us > 0 &&
            now_us - disconnected_since_us >= NTP_TRANSPORT_STALL_TIMEOUT_US &&
            last_ntp_response_us > 0 && now_us - last_ntp_response_us >= NTP_TRANSPORT_STALL_TIMEOUT_US)
        {
            recover_ethernet_transport();
            s_mqtt_disconnected_since_us.store(esp_timer_get_time(), std::memory_order_release);

#if CALCULATE_STACK_SIZES_ENABLED
            report_current_task_stack_usage(Ethernet_Transport_Recovery);
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

static void synchronize_hardware_clock()
{
    if (!s_ptp_clock_ready.load(std::memory_order_acquire))
        return;

    struct timeval now{};
    gettimeofday(&now, nullptr);
    const struct timespec clock_time{
        now.tv_sec,
        static_cast<long>(now.tv_usec) * 1000L};
    clock_settime(CLOCK_PTP_SYSTEM, &clock_time);
}

static bool configure_hardware_timestamps()
{
    const esp_eth_handle_t handle = ETH.handle();
    if (handle == nullptr)
        return false;

    const esp_eth_clock_cfg_t clock_config{CLOCK_PTP_SYSTEM};
    if (esp_eth_clock_init(handle, &clock_config) != ESP_OK)
        return false;

    esp_eth_mac_t *mac = nullptr;
    if (esp_eth_get_mac_instance(handle, &mac) != ESP_OK ||
        esp_eth_mac_enable_ts4all(mac, true) != ESP_OK)
        return false;

    s_ptp_clock_ready.store(true, std::memory_order_release);
    synchronize_hardware_clock();

    const bool input_path_updated = esp_eth_update_input_path_info(handle, ntp_ethernet_input, ETH.netif()) == ESP_OK;
    s_hardware_ntp_accepting.store(input_path_updated, std::memory_order_release);
    return input_path_updated;
}

static const char *fix_type_to_text(uint8_t fix_type)
{
    switch (fix_type)
    {
    case 1:
        return "Dead reckoning";
    case 2:
        return "2D";
    case 3:
        return "3D";
    case 4:
        return "GNSS + Dead reckoning";
    case 5:
        return "Date and time";
    default:
        return "No fix";
    }
}

static bool is_digit_char(char c)
{
    return c >= '0' && c <= '9';
}

static bool parse_two_digits(const char *text, int &value)
{
    if (text == nullptr || !is_digit_char(text[0]) || !is_digit_char(text[1]))
        return false;

    value = (text[0] - '0') * 10 + (text[1] - '0');
    return true;
}

static bool get_nmea_field(const char *sentence, int field_index, char *output, size_t output_size)
{
    if (sentence == nullptr || output == nullptr || output_size == 0 || field_index < 0)
        return false;

    const char *cursor = sentence;
    if (*cursor == '$')
        cursor++;

    int current_field = 0;
    const char *field_start = cursor;

    while (true)
    {
        char current = *cursor;
        bool is_delimiter = (current == ',') || (current == '*') || (current == '\0') || (current == '\r') || (current == '\n');

        if (is_delimiter)
        {
            if (current_field == field_index)
            {
                size_t length = static_cast<size_t>(cursor - field_start);
                if (length >= output_size)
                    length = output_size - 1;

                memcpy(output, field_start, length);
                output[length] = '\0';
                return true;
            }

            if (current != ',')
                break;

            current_field++;
            cursor++;
            field_start = cursor;
            continue;
        }

        cursor++;
    }

    output[0] = '\0';
    return false;
}

static bool parse_nmea_rmc_sentence(const char *sentence, nmea_rmc_time_t *time_data)
{
    if (sentence == nullptr || time_data == nullptr)
        return false;

    char sentence_type[16] = "";
    if (!get_nmea_field(sentence, 0, sentence_type, sizeof(sentence_type)))
        return false;

    size_t sentence_type_len = strlen(sentence_type);
    if (sentence_type_len < 3 || strcmp(sentence_type + sentence_type_len - 3, "RMC") != 0)
        return false;

    char status_field[4] = "";
    if (!get_nmea_field(sentence, 2, status_field, sizeof(status_field)))
        return false;

    if (status_field[0] != 'A')
        return false;

    char time_field[16] = "";
    char date_field[16] = "";
    if (!get_nmea_field(sentence, 1, time_field, sizeof(time_field)) || !get_nmea_field(sentence, 9, date_field, sizeof(date_field)))
        return false;

    if (strlen(time_field) < 6 || strlen(date_field) < 6)
        return false;

    int hour = 0;
    int minute = 0;
    int second = 0;
    int day = 0;
    int month = 0;
    int year_two_digit = 0;

    if (!parse_two_digits(time_field + 0, hour) ||
        !parse_two_digits(time_field + 2, minute) ||
        !parse_two_digits(time_field + 4, second) ||
        !parse_two_digits(date_field + 0, day) ||
        !parse_two_digits(date_field + 2, month) ||
        !parse_two_digits(date_field + 4, year_two_digit))
    {
        return false;
    }

    int year = 2000 + year_two_digit;

    if (year <= 2022 || month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60)
        return false;

    time_data->year = year;
    time_data->month = month;
    time_data->day = day;
    time_data->hour = hour;
    time_data->minute = minute;
    time_data->second = second;
    return true;
}

#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
#else
static constexpr size_t NMEA_GSV_GROUP_LIMIT = 8;
static constexpr uint32_t NMEA_GSV_GROUP_EXPIRY_MS = 10000UL;

struct nmea_gsv_group_t
{
    char talker[3] = "";
    char signal_id[4] = "";
    uint8_t message_count = 0;
    uint8_t next_message_number = 0;
    uint8_t satellites_in_view = 0;
    uint32_t last_updated_ms = 0;
    bool published = false;
};

static nmea_gsv_group_t s_nmea_gsv_groups[NMEA_GSV_GROUP_LIMIT]{};

static bool is_nmea_checksum_valid(const char *sentence)
{
    if (sentence == nullptr || sentence[0] != '$')
        return false;

    const char *checksum_marker = strrchr(sentence, '*');
    if (checksum_marker == nullptr || checksum_marker[1] == '\0' || checksum_marker[2] == '\0' || checksum_marker[3] != '\0')
        return false;

    unsigned int expected_checksum = 0;
    if (sscanf(checksum_marker + 1, "%2X", &expected_checksum) != 1)
        return false;

    uint8_t actual_checksum = 0;
    for (const char *cursor = sentence + 1; cursor < checksum_marker; ++cursor)
        actual_checksum ^= static_cast<uint8_t>(*cursor);

    return actual_checksum == expected_checksum;
}

static bool parse_nmea_uint8_field(const char *sentence, int field_index, uint8_t *value)
{
    if (value == nullptr)
        return false;

    char field[4] = "";
    if (!get_nmea_field(sentence, field_index, field, sizeof(field)) || field[0] == '\0')
        return false;

    unsigned int parsed_value = 0;
    if (sscanf(field, "%u", &parsed_value) != 1 || parsed_value > UINT8_MAX)
        return false;

    for (const char *cursor = field; *cursor != '\0'; ++cursor)
    {
        if (!is_digit_char(*cursor))
            return false;
    }

    *value = static_cast<uint8_t>(parsed_value);
    return true;
}

static void publish_nmea_gsv_satellite_count(uint32_t now_ms)
{
#if MQTT_ENABLED
    bool has_constellation_specific_group = false;
    bool has_reportable_group = false;
    for (const nmea_gsv_group_t &group : s_nmea_gsv_groups)
    {
        if (!group.published || (now_ms - group.last_updated_ms) > NMEA_GSV_GROUP_EXPIRY_MS)
            continue;

        has_reportable_group = true;
        if (strcmp(group.talker, "GN") != 0)
        {
            has_constellation_specific_group = true;
            break;
        }
    }

    if (!has_reportable_group)
        return;

    uint16_t total_satellites = 0;
    for (const nmea_gsv_group_t &group : s_nmea_gsv_groups)
    {
        if (group.published && (now_ms - group.last_updated_ms) <= NMEA_GSV_GROUP_EXPIRY_MS &&
            (!has_constellation_specific_group || strcmp(group.talker, "GN") != 0))
            total_satellites += group.satellites_in_view;
    }
    mqtt_note_satellite_count(static_cast<uint8_t>(std::min<uint16_t>(total_satellites, UINT8_MAX)));
#else
    (void)now_ms;
#endif
}

static void expire_nmea_gsv_groups(uint32_t now_ms)
{
    bool group_expired = false;
    for (nmea_gsv_group_t &group : s_nmea_gsv_groups)
    {
        if (group.talker[0] != '\0' && (now_ms - group.last_updated_ms) > NMEA_GSV_GROUP_EXPIRY_MS)
        {
            group = nmea_gsv_group_t{};
            group_expired = true;
        }
    }

    if (group_expired)
        publish_nmea_gsv_satellite_count(now_ms);
}

static void process_nmea_gsv_sentence(const char *sentence)
{
    if (!is_nmea_checksum_valid(sentence))
        return;

    char sentence_type[16] = "";
    if (!get_nmea_field(sentence, 0, sentence_type, sizeof(sentence_type)) || strlen(sentence_type) != 5 ||
        strcmp(sentence_type + 2, "GSV") != 0)
        return;

    uint8_t message_count = 0;
    uint8_t message_number = 0;
    uint8_t satellites_in_view = 0;
    if (!parse_nmea_uint8_field(sentence, 1, &message_count) ||
        !parse_nmea_uint8_field(sentence, 2, &message_number) ||
        !parse_nmea_uint8_field(sentence, 3, &satellites_in_view) ||
        message_count == 0 || message_number == 0 || message_number > message_count)
        return;

    const uint16_t satellite_offset = static_cast<uint16_t>(message_number - 1) * 4U;
    const uint8_t satellites_in_message = satellites_in_view > satellite_offset
                                              ? std::min<uint8_t>(4, static_cast<uint8_t>(satellites_in_view - satellite_offset))
                                              : 0;
    char signal_id[4] = "";
    (void)get_nmea_field(sentence, 4 + satellites_in_message * 4, signal_id, sizeof(signal_id));

    const uint32_t now_ms = millis();
    expire_nmea_gsv_groups(now_ms);

    nmea_gsv_group_t *group = nullptr;
    nmea_gsv_group_t *available_group = nullptr;
    for (nmea_gsv_group_t &candidate : s_nmea_gsv_groups)
    {
        if (candidate.talker[0] == '\0')
        {
            available_group = &candidate;
            continue;
        }
        if (strncmp(candidate.talker, sentence_type, 2) == 0 && strcmp(candidate.signal_id, signal_id) == 0)
        {
            group = &candidate;
            break;
        }
    }

    if (group == nullptr && available_group != nullptr)
    {
        group = available_group;
        snprintf(group->talker, sizeof(group->talker), "%.2s", sentence_type);
        snprintf(group->signal_id, sizeof(group->signal_id), "%s", signal_id);
    }

    if (group == nullptr || (message_number != 1 && (group->message_count != message_count || group->next_message_number != message_number)))
        return;

    group->message_count = message_count;
    group->next_message_number = static_cast<uint8_t>(message_number + 1);
    group->satellites_in_view = satellites_in_view;
    group->last_updated_ms = now_ms;
    group->published = message_number == message_count;

    if (group->published)
        publish_nmea_gsv_satellite_count(now_ms);
}
#endif

static bool wait_for_nmea_rmc_time(nmea_rmc_time_t *time_data, uint32_t timeout_ms)
{
    if (time_data == nullptr)
        return false;

    uint32_t start_ms = millis();
    char sentence[128] = "";
    size_t index = 0;
    bool collecting = false;

    while ((millis() - start_ms) < timeout_ms)
    {
        while (s_gnss_serial.available() > 0)
        {
            int value = s_gnss_serial.read();
            if (value < 0)
                break;

            char ch = static_cast<char>(value);
            if (ch == '$')
            {
                collecting = true;
                index = 0;
                sentence[index++] = ch;
                continue;
            }

            if (!collecting)
                continue;

            if (ch == '\r' || ch == '\n')
            {
                sentence[index] = '\0';
#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
#else
                process_nmea_gsv_sentence(sentence);
#endif
                if (parse_nmea_rmc_sentence(sentence, time_data))
                    return true;

                collecting = false;
                index = 0;
                continue;
            }

            if (ch >= 32 && ch <= 126)
            {
                if (index < sizeof(sentence) - 1)
                {
                    sentence[index++] = ch;
                }
                else
                {
                    collecting = false;
                    index = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return false;
}

struct gnss_probe_result_t
{
    bool saw_data = false;
    bool saw_valid_protocol_traffic = false;
    size_t bytes_seen = 0;
    char sample[33] = "";
};

static bool current_gnss_timing_is_valid()
{
    if (s_use_nmea_fallback)
    {
        nmea_rmc_time_t nmea_time{};
        return wait_for_nmea_rmc_time(&nmea_time, 1200UL);
    }

    // Get the latest Position/Velocity/Time solution and fill all global variables
    static constexpr uint16_t status_query_timeout_ms = 1500;
    if (!s_gnss.getPVT(status_query_timeout_ms))
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "getPVT failed");
#endif
        return false;
    };

    uint8_t fix_type = s_gnss.getFixType(status_query_timeout_ms); // 0 - No fix; 1 - Dead reckoning only; 2 - 2D-fix; 3: 3D-fix; 4 - GNSS + dead reckoning combined; 5 - Time only fix
#if MQTT_ENABLED
    uint8_t satellites = s_gnss.getSIV(status_query_timeout_ms);
    mqtt_note_satellite_count(satellites);
#endif
    bool gnss_fix_ok = s_gnss.getGnssFixOk(status_query_timeout_ms);
    bool date_valid = s_gnss.getDateValid(status_query_timeout_ms);
    bool time_valid = s_gnss.getTimeValid(status_query_timeout_ms);

#if DEBUG_ENABLED

    if (fix_type < 3)
        ESP_LOGW(TAG, "GNSS fix type - %s", fix_type == 0 ? "No fix" : fix_type == 1 ? "Dead reckoning only"
                                                                                     : "2D-fix");

    if (!gnss_fix_ok)
        ESP_LOGW(TAG, "GNSS fix is not ok");

    if (!date_valid)
        ESP_LOGW(TAG, "GNSS date is invalid");

    if (!time_valid)
        ESP_LOGW(TAG, "GNSS time is invalid");

#endif

    return (fix_type > 2) && gnss_fix_ok && date_valid && time_valid;
}

static gnss_probe_result_t probe_gnss_uart(uint32_t baud);

static gnss_probe_result_t probe_gnss_uart(uint32_t baud)
{
    static constexpr uint16_t gnssProbeListenTimeMs = 1500;
    static constexpr uint16_t max_ubx_payload_length = 1024;
    gnss_probe_result_t result{};
    size_t sample_len = 0;
    uint8_t nmea_header_length = 0;
    uint8_t ubx_state = 0;
    uint16_t ubx_payload_length = 0;
    uint16_t ubx_payload_received = 0;
    uint8_t ubx_checksum_a = 0;
    uint8_t ubx_checksum_b = 0;
    uint8_t ubx_received_checksum_a = 0;

    s_gnss_serial.end();
    s_gnss_serial.begin(baud, SERIAL_8N1, RXPin, TXPin);
    vTaskDelay(pdMS_TO_TICKS(150));

    uint32_t start_ms = millis();
    while ((millis() - start_ms) < gnssProbeListenTimeMs)
    {
        while (s_gnss_serial.available() > 0)
        {
            int value = s_gnss_serial.read();
            if (value < 0)
                break;

            uint8_t byte = static_cast<uint8_t>(value);
            result.saw_data = true;
            result.bytes_seen++;

            if (sample_len < (sizeof(result.sample) - 1))
            {
                char character = static_cast<char>(byte);
                if (character >= 32 && character <= 126)
                    result.sample[sample_len++] = character;
                else if (character == '\r' || character == '\n' || character == '\t')
                    result.sample[sample_len++] = ' ';
                else
                    result.sample[sample_len++] = '.';
                result.sample[sample_len] = '\0';
            }

            if (nmea_header_length == 0)
                nmea_header_length = byte == '$' ? 1 : 0;
            else if (nmea_header_length == 1)
                nmea_header_length = byte == 'G' ? 2 : (byte == '$' ? 1 : 0);
            else if (byte >= 'A' && byte <= 'Z')
            {
                nmea_header_length++;
                if (nmea_header_length == 6)
                    result.saw_valid_protocol_traffic = true;
            }
            else
                nmea_header_length = byte == '$' ? 1 : 0;

            switch (ubx_state)
            {
            case 0:
                ubx_state = byte == 0xB5 ? 1 : 0;
                break;
            case 1:
                ubx_state = byte == 0x62 ? 2 : (byte == 0xB5 ? 1 : 0);
                break;
            case 2:
            case 3:
            case 4:
            case 5:
                if (ubx_state == 2)
                {
                    ubx_checksum_a = byte;
                    ubx_checksum_b = byte;
                }
                else
                {
                    ubx_checksum_a += byte;
                    ubx_checksum_b += ubx_checksum_a;
                }

                if (ubx_state == 4)
                    ubx_payload_length = byte;
                else if (ubx_state == 5)
                {
                    ubx_payload_length |= static_cast<uint16_t>(byte) << 8;
                    ubx_payload_received = 0;
                    ubx_state = ubx_payload_length == 0 ? 7 : (ubx_payload_length <= max_ubx_payload_length ? 6 : 0);
                    break;
                }
                ubx_state++;
                break;
            case 6:
                ubx_checksum_a += byte;
                ubx_checksum_b += ubx_checksum_a;
                if (++ubx_payload_received >= ubx_payload_length)
                    ubx_state = 7;
                break;
            case 7:
                ubx_received_checksum_a = byte;
                ubx_state = 8;
                break;
            default:
                if (ubx_received_checksum_a == ubx_checksum_a && byte == ubx_checksum_b)
                    result.saw_valid_protocol_traffic = true;
                ubx_state = byte == 0xB5 ? 1 : 0;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return result;
}

static bool try_gnss_begin(uint32_t baud, bool assume_success, bool *saw_serial_data)
{

    static constexpr uint16_t gnssBeginMaxWaitMs = 2500;

    gnss_probe_result_t probe_result = probe_gnss_uart(baud);
    if (probe_result.saw_data && saw_serial_data != nullptr)
        *saw_serial_data = true;

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GNSS probe at %lu baud: %u bytes seen, valid protocol traffic=%s, sample: %s",
             static_cast<unsigned long>(baud),
             static_cast<unsigned int>(probe_result.bytes_seen),
             probe_result.saw_valid_protocol_traffic ? "yes" : "no",
             probe_result.saw_data ? probe_result.sample : "<none>");
#endif

    s_gnss_serial.end();
    s_gnss_serial.begin(baud, SERIAL_8N1, RXPin, TXPin);
    vTaskDelay(pdMS_TO_TICKS(150));

    bool begin_result = s_gnss.begin(s_gnss_serial, gnssBeginMaxWaitMs, assume_success);
    bool communication_confirmed = begin_result || (assume_success && probe_result.saw_valid_protocol_traffic);
#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GNSS begin at %lu baud with assume_success=%s -> %s",
             static_cast<unsigned long>(baud),
             assume_success ? "true" : "false",
             communication_confirmed ? "success" : "failed");
#endif

    return communication_confirmed;
}

static bool confirm_saved_gnss_baud_rate(uint32_t baud)
{
    bool saw_serial_data = false;

    s_detected_gnss_baud = 0;
    s_gnss_required_assume_success = false;

    for (int attempt = 0; attempt < 2; ++attempt)
    {
        bool begin_without_assume = try_gnss_begin(baud, false, &saw_serial_data);
        bool begin_with_assume = false;
        if (!begin_without_assume)
            begin_with_assume = try_gnss_begin(baud, true, &saw_serial_data);

        if (begin_without_assume || begin_with_assume)
        {
            s_detected_gnss_baud = baud;
            s_gnss_required_assume_success = begin_with_assume;
            return true;
        }
    }

    return false;
}

static bool find_initial_gnss_baud_rate(int max_attempts, gnss_identity_t *working_baud_identity = nullptr)
{
    std::vector<uint32_t> candidate_baud_rates;
    build_candidate_baud_rates(candidate_baud_rates);

    bool saw_any_serial_data = false;
    s_detected_gnss_baud = 0;
    s_gnss_required_assume_success = false;

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "GNSS initial-baud discovery attempt %d of %d", attempt + 1, max_attempts);
#endif
        for (uint32_t candidate_baud : candidate_baud_rates)
        {
            bool begin_without_assume = try_gnss_begin(candidate_baud, false, &saw_any_serial_data);
            bool begin_with_assume = false;
            if (!begin_without_assume)
                begin_with_assume = try_gnss_begin(candidate_baud, true, &saw_any_serial_data);

            if (begin_without_assume || begin_with_assume)
            {
                s_detected_gnss_baud = candidate_baud;
                s_gnss_required_assume_success = begin_with_assume;
                if (working_baud_identity != nullptr)
                    *working_baud_identity = query_gnss_identity();
                return true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    return false;
}

static bool change_gnss_baud_rate(uint32_t from_baud, uint32_t to_baud)
{
    if (from_baud == to_baud)
        return confirm_saved_gnss_baud_rate(to_baud);

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GNSS responded at %lu baud. Attempting to switch to %lu baud.",
             static_cast<unsigned long>(from_baud),
             static_cast<unsigned long>(to_baud));
#endif

    bool baud_change_command_reported_success = s_gnss.setSerialRate(to_baud, COM_PORT_UART1, VAL_LAYER_RAM_BBR);
    vTaskDelay(pdMS_TO_TICKS(200));
    if (!confirm_saved_gnss_baud_rate(to_baud))
        return false;

#if DEBUG_ENABLED
    if (!baud_change_command_reported_success)
        ESP_LOGI(TAG, "While the GNSS baud-rate change command to %lu was not acknowledged, the reconnect succeeded at the new baud.", static_cast<unsigned long>(to_baud));
#endif
    return true;
}

static bool set_gnss_baud_rate(int max_attempts, gnss_identity_t *working_baud_identity = nullptr, uint32_t *initial_baud = nullptr)
{

    std::vector<uint32_t> candidate_baud_rates;
    build_candidate_baud_rates(candidate_baud_rates);

    bool saw_any_serial_data = false;
    s_detected_gnss_baud = 0;
    s_gnss_required_assume_success = false;

    char baud_line[lcdColumns + 1];

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "GNSS initialization attempt %d of %d", attempt + 1, max_attempts);
#endif

        for (uint32_t candidate_baud : candidate_baud_rates)
        {

            char dots[10] = "";
            for (int i = 0; i < (attempt % 4); i++)
                strcat(dots, ".");

            snprintf(baud_line, sizeof(baud_line), "Baud: %lu %s", static_cast<unsigned long>(candidate_baud), dots);
            display_line(2, baud_line);

            bool begin_without_assume = try_gnss_begin(candidate_baud, false, &saw_any_serial_data);
            bool begin_with_assume = false;
            if (!begin_without_assume)
                begin_with_assume = try_gnss_begin(candidate_baud, true, &saw_any_serial_data);

            if (begin_without_assume || begin_with_assume)
            {
                s_detected_gnss_baud = candidate_baud;
                s_gnss_required_assume_success = begin_with_assume;
                if (initial_baud != nullptr)
                    *initial_baud = candidate_baud;

                if (working_baud_identity != nullptr)
                    *working_baud_identity = query_gnss_identity();

                uint32_t highest_working_baud = candidate_baud;
                bool receiver_reachable = true;
                for (uint32_t higher_baud : candidate_baud_rates)
                {
                    if (higher_baud <= highest_working_baud)
                        continue;

#if DEBUG_ENABLED
                    ESP_LOGI(TAG,
                             "GNSS responded at %lu baud. Attempting to switch to %lu baud.",
                             static_cast<unsigned long>(highest_working_baud),
                             static_cast<unsigned long>(higher_baud));
#endif

                    bool baud_change_command_reported_success = s_gnss.setSerialRate(higher_baud, COM_PORT_UART1, VAL_LAYER_RAM_BBR);
                    vTaskDelay(pdMS_TO_TICKS(200));

                    if (confirm_saved_gnss_baud_rate(higher_baud))
                    {
                        highest_working_baud = higher_baud;
#if DEBUG_ENABLED
                        if (!baud_change_command_reported_success)
                            ESP_LOGI(TAG, "While the GNSS baud-rate change command to %lu was not acknowledged, the reconnect succeeded at the new baud.", static_cast<unsigned long>(higher_baud));
#endif
                        continue;
                    }

                    if (!confirm_saved_gnss_baud_rate(highest_working_baud))
                    {
                        receiver_reachable = false;
                        break;
                    }
                }

                if (!receiver_reachable)
                    continue;

                s_detected_gnss_baud = highest_working_baud;

#if DEBUG_ENABLED
                if (s_gnss_required_assume_success)
                {
                    ESP_LOGW(TAG,
                             "GNSS communication was established only with assume_success=true at %lu baud. The attached module may have limited u-blox compatibility.",
                             static_cast<unsigned long>(s_detected_gnss_baud));
                };

                ESP_LOGI(TAG, "GNSS initialization will continue at %lu baud.", static_cast<unsigned long>(s_detected_gnss_baud));
#endif
                return true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

#if DEBUG_ENABLED
    if (saw_any_serial_data)
        ESP_LOGW(TAG, "GNSS serial data was detected, but SparkFun u-blox GNSS v3 could not initialize the module. The module may be an older NEO-6/7/M8 variant.");
    else
        ESP_LOGW(TAG, "No GNSS serial data was detected on GPIO%d/GPIO%d at any tested baud rate. Check power, TX/RX wiring, and signal levels.", RXPin, TXPin);
#endif

    return false;
}

static bool configure_gnss_outputs(bool *uart1_output_set_result)
{
#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
    bool i2c_output_disabled = s_gnss.setI2COutput(0);
    bool uart2_output_disabled = s_gnss.setUART2Output(0);
#else
    bool i2c_output_disabled = true;
    bool uart2_output_disabled = true;
#endif

    bool uart1_output_set = s_gnss.setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA);

    if (uart1_output_set_result != nullptr)
        *uart1_output_set_result = uart1_output_set;

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GNSS port config results: I2C off=%s, UART1 UBX=%s, UART2 off=%s",
             i2c_output_disabled ? "ok" : "failed",
             uart1_output_set ? "ok" : "failed",
             uart2_output_disabled ? "ok" : "failed");

    if (!uart2_output_disabled)
        ESP_LOGI(TAG, "UART2 output disable is not supported by this module/firmware. Continuing with UART1 configuration.");

    if (!i2c_output_disabled)
        ESP_LOGI(TAG, "I2C output disable did not succeed. This is non-fatal for UART1 operation.");

#endif

    return uart1_output_set;
}

/* pre-release code - commented out

static bool configure_gnss_time_only_uart1_output()
{
    if (!reduceGNSSUART1OutputToTimeMessages)
        return true;

    uint8_t layer = saveReducedGNSSUART1OutputPermanently ? VAL_LAYER_ALL : VAL_LAYER_RAM_BBR;
    auto set_message_rate = [layer](uint32_t key, uint8_t rate, const char *message_name)
    {
        bool command_worked = s_gnss.setVal8(key, rate, layer);
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "GNSS UART1 %s: %s", message_name, command_worked ? "ok" : "failed");
#endif
        return command_worked;
    };

    bool configured = true;
    configured = set_message_rate(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1, 1, "RMC enabled") && configured;
    configured = set_message_rate(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1, 0, "GGA disabled") && configured;
    configured = set_message_rate(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1, 0, "GLL disabled") && configured;
    configured = set_message_rate(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1, 0, "GSA disabled") && configured;
    configured = set_message_rate(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1, 0, "GSV disabled") && configured;
    configured = set_message_rate(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1, 0, "VTG disabled") && configured;

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GNSS time-only UART1 message configuration: %s (%s)",
             configured ? "ok" : "partially applied; continuing with available output",
             saveReducedGNSSUART1OutputPermanently ? "saved to receiver flash" : "RAM/BBR only");
#endif

    return configured;
}

end of pre-release code */

static void halt_with_display(const char *line1, const char *line2, const char *line3)
{
    display_line(1, line1);
    display_line(2, line2);
    display_line(3, line3);
    s_saved_gnss_baud_communication_failed.store(true);

    while (true)
        vTaskDelay(pdMS_TO_TICKS(1000));
}

static bool IRAM_ATTR pps_capture_callback(mcpwm_cap_channel_handle_t,
                                           const mcpwm_capture_event_data_t *,
                                           void *)
{
    PpsCaptureEvent event{};
    event.approximate_edge_us = esp_timer_get_time();

    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_pps_semaphore != nullptr)
        xSemaphoreGiveFromISR(s_pps_semaphore, &higher_priority_task_woken);
    if (s_pps_timestamp_queue != nullptr)
        xQueueOverwriteFromISR(s_pps_timestamp_queue, &event, &higher_priority_task_woken);
    if (s_pps_sync_timestamp_queue != nullptr)
        xQueueOverwriteFromISR(s_pps_sync_timestamp_queue, &event, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

static void setup_pps_input()
{
    mcpwm_capture_timer_config_t timer_config{};
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = PPS_CAPTURE_RESOLUTION_HZ;
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&timer_config, &s_pps_capture_timer));

    mcpwm_capture_channel_config_t channel_config{};
    channel_config.gpio_num = PPSPin;
    channel_config.prescale = 1;
    channel_config.flags.pos_edge = true;
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(s_pps_capture_timer, &channel_config, &s_pps_capture_channel));

    mcpwm_capture_event_callbacks_t callbacks{};
    callbacks.on_cap = pps_capture_callback;
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(s_pps_capture_channel, &callbacks, nullptr));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(s_pps_capture_channel));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(s_pps_capture_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(s_pps_capture_timer));
}

static void clear_pps_events()
{
    while (xSemaphoreTake(s_pps_semaphore, 0) == pdTRUE)
    {
    }

    PpsCaptureEvent event{};
    while (xQueueReceive(s_pps_sync_timestamp_queue, &event, 0) == pdTRUE)
    {
    }
}

static bool wait_for_pps_capture_event(PpsCaptureEvent *event, TickType_t timeout)
{
    return xQueueReceive(s_pps_sync_timestamp_queue, event, timeout) == pdTRUE;
}

static void finish_ntp_request_rate_second()
{
#if MQTT_ENABLED
    const uint32_t requests_this_second = s_ntp_requests_this_second.exchange(0, std::memory_order_relaxed);
    uint32_t most_requests_per_second = s_ntp_most_requests_per_second.load(std::memory_order_relaxed);
    while (most_requests_per_second < requests_this_second &&
           !s_ntp_most_requests_per_second.compare_exchange_weak(most_requests_per_second, requests_this_second,
                                                                 std::memory_order_relaxed, std::memory_order_relaxed))
    {
    }
#endif
}

static void pps_discipline_task(void *parameter)
{
    static constexpr int64_t ppsTimeoutUs = 3500000;
    static constexpr int64_t maxPhaseErrorUsToCorrect = 250000;
    static constexpr int64_t minCorrectionMagnitudeUs = 2;

    int64_t last_pps_us = esp_timer_get_time();
    bool logged_active = false;

    for (;;)
    {
        PpsCaptureEvent capture_event{};
        if (xQueueReceive(s_pps_timestamp_queue, &capture_event, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            int64_t edge_us = capture_event.approximate_edge_us;
            last_pps_us = edge_us;
            s_pps_discipline_active.store(true);
#if MQTT_ENABLED
            s_pps_pulses.fetch_add(1);
#endif
            sync_state_note_pps_edge(edge_us);
            finish_ntp_request_rate_second();

            if (!logged_active)
            {
#if DEBUG_ENABLED
                ESP_LOGI(TAG, "PPS discipline active.");
#endif
                logged_active = true;
            }

            if (!s_time_has_been_set.load() || s_time_setting_in_progress.load())
                continue;

            if (xSemaphoreTake(s_time_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
                continue;

            int64_t processing_now_us = esp_timer_get_time();
            struct timeval now{};
            gettimeofday(&now, nullptr);

            int64_t current_time_us = static_cast<int64_t>(now.tv_sec) * 1000000LL + static_cast<int64_t>(now.tv_usec);
            int64_t wake_delay_us = processing_now_us - edge_us;
            if (wake_delay_us < 0)
                wake_delay_us = 0;
            int64_t edge_time_us = current_time_us - wake_delay_us;

            int64_t phase_error_us = edge_time_us % 1000000LL;
            if (phase_error_us > 500000)
                phase_error_us -= 1000000;
            else if (phase_error_us < -500000)
                phase_error_us += 1000000;

            int64_t correction_us = -phase_error_us;
            if (llabs(correction_us) >= minCorrectionMagnitudeUs && llabs(phase_error_us) <= maxPhaseErrorUsToCorrect)
            {
                struct timeval delta{};
                delta.tv_sec = static_cast<time_t>(correction_us / 1000000LL);
                delta.tv_usec = static_cast<suseconds_t>(correction_us % 1000000LL);
                if (delta.tv_usec < 0)
                {
                    delta.tv_usec += 1000000;
                    delta.tv_sec -= 1;
                }

                if (adjtime(&delta, nullptr) == 0)
                {
                    synchronize_hardware_clock();
                    s_ntp_reference_time_64 = get_current_time_in_ntp64_format();
                    s_ntp_reference_valid = true;
                }
            }

            xSemaphoreGive(s_time_mutex);
        }
        else
        {
            int64_t now_us = esp_timer_get_time();
            if ((now_us - last_pps_us) > ppsTimeoutUs)
            {
                if (logged_active && DEBUG_ENABLED)
                    ESP_LOGW(TAG, "PPS discipline inactive (PPS signal unavailable).");
                logged_active = false;
                s_pps_discipline_active.store(false);
                sync_state_note_pps_timeout(now_us);
            }
        }

#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(PPS_Discipline);
#endif
    }
}

static bool wait_for_gnss_startup_qualification()
{
    int64_t valid_started_us = 0;
    int64_t last_pps_us = 0;
    uint32_t stable_pps_edges = 0;

    clear_pps_events();

#if RBG_LED_ENABLED
    s_gnss_pps_startup_qualification_in_progress.store(true, std::memory_order_release);
#endif

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "Qualifying GNSS and PPS stability before startup.");
#endif

    for (;;)
    {
        int64_t now_us = esp_timer_get_time();
        if (!current_gnss_timing_is_valid())
        {
            valid_started_us = 0;
            last_pps_us = 0;
            stable_pps_edges = 0;
            clear_pps_events();
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        if (valid_started_us == 0)
        {
            valid_started_us = now_us;
            display_line(1, "GNSS Time Valid");
            display_line(2, "Qualifying PPS");
        }

        int64_t qualification_deadline_us = valid_started_us +
                                            static_cast<int64_t>(gnss_Startup_Qualification_Duration_Ms) * 1000LL;
        if (now_us >= qualification_deadline_us)
        {
            if (stable_pps_edges >= gnss_Startup_Qualification_PPS_Edges)
            {
#if DEBUG_ENABLED
                ESP_LOGI(TAG, "GNSS and PPS startup qualification complete after %lu stable PPS edges.",
                         static_cast<unsigned long>(stable_pps_edges));
#endif
#if RBG_LED_ENABLED
                s_gnss_pps_startup_qualification_in_progress.store(false, std::memory_order_release);
#endif
                return true;
            }

            valid_started_us = 0;
            last_pps_us = 0;
            stable_pps_edges = 0;
            clear_pps_events();

            char pps_edges_line[lcdColumns + 1];
            snprintf(pps_edges_line,
                     sizeof(pps_edges_line),
                     "Edges %lu of %lu",
                     static_cast<unsigned long>(stable_pps_edges),
                     static_cast<unsigned long>(gnss_Startup_Qualification_PPS_Edges));
            display_line(1, "PPS Stabilizing");
            display_line(2, pps_edges_line);
            valid_started_us = esp_timer_get_time();
            continue;
        }

        if (xSemaphoreTake(s_pps_semaphore, 0) == pdTRUE)
        {
            now_us = esp_timer_get_time();
            if (last_pps_us == 0 ||
                ((now_us - last_pps_us) >= gnss_Startup_Min_PPS_Interval_Us &&
                 (now_us - last_pps_us) <= gnss_Startup_Max_PPS_Interval_Us))
            {
                stable_pps_edges++;
            }
            else
            {
                stable_pps_edges = 1;
            }
            last_pps_us = now_us;

            char pps_edges_line[lcdColumns + 1];
            snprintf(pps_edges_line,
                     sizeof(pps_edges_line),
                     "Edges %lu of %lu",
                     static_cast<unsigned long>(stable_pps_edges),
                     static_cast<unsigned long>(gnss_Startup_Qualification_PPS_Edges));
            display_line(1, "PPS Stabilizing");
            display_line(2, pps_edges_line);
        }

        if (stable_pps_edges >= gnss_Startup_Qualification_PPS_Edges)
        {
            now_us = esp_timer_get_time();
            int64_t remaining_us = qualification_deadline_us - now_us;
            if (remaining_us > 0)
            {
                char dwell_line[lcdColumns + 1];
                snprintf(dwell_line,
                         sizeof(dwell_line),
                         "Dwelling for %lu secs",
                         static_cast<unsigned long>(remaining_us / 1000000LL));
                display_line(1, "PPS Stabilizing");
                display_line(2, dwell_line);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void setup_gnss()
{

#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
    static constexpr int maxAttemptsToInitializeGNSS = 10;
    gnss_nvs_data_t stored_gnss_data{};
    bool gnss_nvs_load_ok = load_gnss_nvs_data(&stored_gnss_data);
    bool has_complete_gnss_nvs_data = gnss_nvs_load_ok && stored_gnss_data.has_id_type &&
                                      stored_gnss_data.id_value[0] != '\0' && stored_gnss_data.max_baud > 0;
    bool first_time_initial_setup = !has_complete_gnss_nvs_data;
    if (first_time_initial_setup && (!gnss_nvs_load_ok || stored_gnss_data.has_stored_data))
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Stored GNSS startup data could not be retrieved completely; clearing it and performing first-time setup.");
#endif
        clear_gnss_nvs_data();
    }
    bool gnss_nvs_should_be_saved = first_time_initial_setup;
    uint32_t highest_candidate_baud = get_highest_candidate_gnss_baud();
#else
    bool first_time_initial_setup = false;
    uint32_t highest_candidate_baud = baudRateForUbloxNonCompliantGNSSReceiver;
#endif

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "Startup mode: %s", first_time_initial_setup ? "First time initial setup" : "Not first time initial setup");
#endif

    uint32_t startup_target_baud = highest_candidate_baud;

#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
    if (has_complete_gnss_nvs_data)
        startup_target_baud = stored_gnss_data.max_baud;

    bool known_module_fast_path = has_complete_gnss_nvs_data;
#else
    bool known_module_fast_path = false;
#endif

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GNSS startup path: %s (target baud=%lu)",
             known_module_fast_path ? "known module fast path" : "first-time scan path",
             static_cast<unsigned long>(startup_target_baud));
#endif

#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED

    bool gnss_communication_confirmed = false;
    uint32_t initial_working_baud = 0;
    gnss_identity_t current_identity{};
    if (known_module_fast_path)
    {
        s_gnss_target_baud = startup_target_baud;
        gnss_communication_confirmed = confirm_saved_gnss_baud_rate(s_gnss_target_baud);
        if (!gnss_communication_confirmed)
        {
#if DEBUG_ENABLED
            ESP_LOGW(TAG, "GNSS communication failed at the saved highest baud rate of %lu; attempting recovery using the initial baud rate.", static_cast<unsigned long>(s_gnss_target_baud));
#endif
            initial_working_baud = stored_gnss_data.initial_baud;
            if (initial_working_baud == 0)
            {
#if DEBUG_ENABLED
                ESP_LOGI(TAG, "Saved GNSS data has no initial baud rate; scanning to recover it.");
#endif
                if (find_initial_gnss_baud_rate(maxAttemptsToInitializeGNSS, &current_identity))
                {
                    initial_working_baud = s_detected_gnss_baud;
                    if (save_gnss_nvs_initial_baud(initial_working_baud))
                        stored_gnss_data.initial_baud = initial_working_baud;
                    else
                        ESP_LOGE(TAG, "Could not save recovered GNSS initial baud rate %lu to non-volatile storage.", static_cast<unsigned long>(initial_working_baud));
                }
            }

            bool initial_baud_confirmed = initial_working_baud > 0 &&
                                          (s_detected_gnss_baud == initial_working_baud ||
                                           confirm_saved_gnss_baud_rate(initial_working_baud));
            if (initial_baud_confirmed)
            {
                gnss_communication_confirmed = change_gnss_baud_rate(initial_working_baud, s_gnss_target_baud);
                if (gnss_communication_confirmed)
                {
#if DEBUG_ENABLED
                    ESP_LOGI(TAG, "GNSS baud recovery succeeded at saved highest baud rate %lu.", static_cast<unsigned long>(s_gnss_target_baud));
#endif
                }
            }

            if (!gnss_communication_confirmed)
            {
#if DEBUG_ENABLED
                ESP_LOGW(TAG, "GNSS baud recovery to %lu failed; scanning for a newly supported highest baud rate.", static_cast<unsigned long>(s_gnss_target_baud));
#endif
                s_gnss_target_baud = highest_candidate_baud;
                if (!set_gnss_baud_rate(maxAttemptsToInitializeGNSS, &current_identity, &initial_working_baud))
                {
#if DEBUG_ENABLED
                    ESP_LOGE(TAG, "GNSS comms failed - check TX/RX + power");
#endif
                    halt_with_display("GNSS comms failed", "Check TX/RX + power", "See serial log");
                }
                gnss_communication_confirmed = true;
                gnss_nvs_should_be_saved = true;
                known_module_fast_path = false;
            }
        }
    }

    if (!gnss_communication_confirmed)
    {
        s_gnss_target_baud = highest_candidate_baud;
        if (!set_gnss_baud_rate(maxAttemptsToInitializeGNSS, &current_identity, &initial_working_baud))
        {
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS comms failed - check TX/RX + power");
#endif
            halt_with_display("GNSS comms failed", "Check TX/RX + power", "See serial log");
        }
        gnss_communication_confirmed = true;
    }

    if (!current_identity.valid)
        current_identity = query_gnss_identity();

    if (known_module_fast_path && stored_gnss_data.max_baud < highest_candidate_baud &&
        !gnss_identity_matches(stored_gnss_data, current_identity))
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Stored GNSS identity does not match the connected receiver; clearing GNSS startup data and performing first-time setup.");
#endif
        clear_gnss_nvs_data();
        first_time_initial_setup = true;
        gnss_nvs_should_be_saved = true;
        s_gnss_target_baud = highest_candidate_baud;
        if (!set_gnss_baud_rate(maxAttemptsToInitializeGNSS, &current_identity, &initial_working_baud))
        {
            ESP_LOGE(TAG, "GNSS comms failed after receiver identity mismatch");
            halt_with_display("GNSS comms failed", "Check TX/RX + power", "See serial log");
        }
        current_identity = query_gnss_identity();
    }

    s_gnss_is_max_m10s = strstr(current_identity.value, "MAX-M10S") != nullptr;

#else

    s_gnss_target_baud = highest_candidate_baud;
    s_detected_gnss_baud = highest_candidate_baud;
    s_gnss_is_max_m10s = false;
    s_gnss_required_assume_success = false;
    s_use_nmea_fallback = true;
    uint32_t baud = highest_candidate_baud;
    static constexpr uint16_t gnssBeginMaxWaitMs = 2500;
    bool assume_success = false;

    s_gnss_serial.end();
    s_gnss_serial.begin(baud, SERIAL_8N1, RXPin, TXPin);
    vTaskDelay(pdMS_TO_TICKS(150));

    bool begin_result = s_gnss.begin(s_gnss_serial, gnssBeginMaxWaitMs, assume_success);
#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GNSS begin at %lu baud with assume_success=%s -> %s",
             static_cast<unsigned long>(baud),
             assume_success ? "true" : "false",
             begin_result ? "success" : "failed");
#endif

#endif

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "GNSS module profile: %s", s_gnss_is_max_m10s ? "MAX-M10S" : "generic/fallback");

    ESP_LOGI(TAG, "GNSS startup mode: baud=%lu, initialization=%s",
             static_cast<unsigned long>(s_detected_gnss_baud),
             s_gnss_required_assume_success ? "assume_success" : "confirmed");
#endif

#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
    if (gnss_nvs_should_be_saved && s_detected_gnss_baud > 0)
    {
        bool save_ok = save_gnss_nvs_data(current_identity, initial_working_baud, s_detected_gnss_baud);
#if DEBUG_ENABLED
        ESP_LOGI(TAG,
                 "GNSS identity persistence: type=%s, value=%s, initial_baud=%lu, max_baud=%lu, save=%s",
                 current_identity.type,
                 current_identity.value,
                 static_cast<unsigned long>(initial_working_baud),
                 static_cast<unsigned long>(s_detected_gnss_baud),
                 save_ok ? "ok" : "failed");
#endif
    }

#endif

    bool uart1_output_set = false;
    configure_gnss_outputs(&uart1_output_set);

    /* pre-release code - commented out
    configure_gnss_time_only_uart1_output();
    end of pre-release code */
#if DEBUG_ENABLED
    if (!uart1_output_set)
        ESP_LOGW(TAG, "GNSS UART1 output configuration failed. Continuing with the module's current output settings.");
#endif

#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
    s_use_nmea_fallback = s_gnss_required_assume_success || !uart1_output_set;
#else
    s_use_nmea_fallback = true;
#endif

    if (s_use_nmea_fallback && !allowFallbackProcessing)
    {
        ESP_LOGE(TAG, "NMEA fallback processing is required but not allowed - check the ESP32TimeServerSetting.h file.");
        halt_with_display("GNSS fallback required", "Fallback not allowed", "Check settings");
    }
#if DEBUG_ENABLED
    if (s_use_nmea_fallback)
        ESP_LOGW(TAG, "NMEA fallback mode enabled for GNSS fix/time acquisition.");
#endif

    display_line(1, "Waiting for GNSS fix");
    display_line(2, "");

    static constexpr uint32_t gnssFixEscalationToNmeaFallbackMs = 120000UL;

    bool nmea_fallback_enabled = s_use_nmea_fallback;
    uint32_t wait_start_ms = millis();
    uint32_t last_status_log_ms = 0;
    uint32_t last_long_wait_warning_ms = 0;

    for (;;)
    {
        uint8_t fix_type = 0;
        bool gnss_fix_ok = false;
        uint8_t satellites_used = 0;
        bool date_valid = false;
        bool time_valid = false;
        bool ubx_fix_ready = false;
        bool nmea_fix_ready = false;

        if (!s_use_nmea_fallback)
        {
            fix_type = s_gnss.getFixType();
            gnss_fix_ok = s_gnss.getGnssFixOk();
            satellites_used = s_gnss.getSIV();
            date_valid = s_gnss.getDateValid();
            time_valid = s_gnss.getTimeValid();
            ubx_fix_ready = (fix_type == 3 || fix_type == 4 || fix_type == 5) && gnss_fix_ok && date_valid && time_valid;

            if ((fix_type == 3 || fix_type == 4) && satellites_used >= 4 && !gnss_fix_ok)
            {
                display_line(1, "3D Fix Detected");
                display_line(2, "Verifying GNSS");
            }
            else if (ubx_fix_ready)
            {
                display_line(1, "GNSS Time Valid");
                display_line(2, "Qualifying PPS");
            }
            else
            {
                char satellites_line[lcdColumns + 1];
                snprintf(satellites_line, sizeof(satellites_line), "Satellites %u of 4", static_cast<unsigned int>(satellites_used));
                display_line(1, "Acquiring GNSS");
                display_line(2, satellites_line);
            }
        }

        uint32_t now_ms = millis();
        if (!ubx_fix_ready && !nmea_fallback_enabled && allowFallbackProcessing && (now_ms - wait_start_ms) >= gnssFixEscalationToNmeaFallbackMs)
        {
            nmea_fallback_enabled = true;
#if DEBUG_ENABLED
            ESP_LOGW(TAG, "No UBX fix after %lu ms. Enabling NMEA fallback for additional acquisition path.", static_cast<unsigned long>(now_ms - wait_start_ms));
#endif
        }

        if (!ubx_fix_ready && nmea_fallback_enabled)
        {
            nmea_rmc_time_t nmea_time{};
            nmea_fix_ready = wait_for_nmea_rmc_time(&nmea_time, 1200UL);
        }

        if (ubx_fix_ready || nmea_fix_ready)
        {
            if (nmea_fix_ready)
                s_use_nmea_fallback = true;

#if DEBUG_ENABLED
            if (ubx_fix_ready)
                ESP_LOGI(TAG, "GNSS fix obtained after %lu ms: fix_type=%u (%s)",
                         static_cast<unsigned long>(millis() - wait_start_ms),
                         static_cast<unsigned int>(fix_type),
                         fix_type_to_text(fix_type));
            else
                ESP_LOGI(TAG, "GNSS fix obtained after %lu ms via NMEA fallback.",
                         static_cast<unsigned long>(millis() - wait_start_ms));
#endif

            if (wait_for_gnss_startup_qualification())
                return;
        }

        if (last_status_log_ms == 0 || (now_ms - last_status_log_ms) >= 5000UL)
        {
#if DEBUG_ENABLED

            if (s_use_nmea_fallback)

                ESP_LOGI(TAG, "Waiting for GNSS fix: elapsed=%lu ms, NMEA fallback active",
                         static_cast<unsigned long>(now_ms - wait_start_ms));
            else
                ESP_LOGI(TAG, "Waiting for GNSS fix: elapsed=%lu ms, fix_type=%u (%s), gnss_fix_ok=%s, date_valid=%s, time_valid=%s, SIV=%u",
                         static_cast<unsigned long>(now_ms - wait_start_ms),
                         static_cast<unsigned int>(fix_type),
                         fix_type_to_text(fix_type),
                         gnss_fix_ok ? "true" : "false",
                         date_valid ? "true" : "false",
                         time_valid ? "true" : "false",
                         static_cast<unsigned int>(satellites_used));
#endif
            last_status_log_ms = now_ms;
        }

        if ((now_ms - wait_start_ms) >= 60000UL && (last_long_wait_warning_ms == 0 || (now_ms - last_long_wait_warning_ms) >= 60000UL))
        {
            // output a warning if GNSS has not produced a usable fix within the expected time frame
#if DEBUG_ENABLED
            ESP_LOGW(TAG, "GNSS has not produced a usable fix yet. Check antenna placement, sky view, and module compatibility.");
#endif
            last_long_wait_warning_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void format_time_to_ISO8601(time_t value, char *output, size_t output_size)
{
    // Returns local time using the ISO-8601 time format = for example 2026-08-25T21:57:14-0400
    // ISO 8601 is an international standard for formatting dates and times from largest to smallest unit
    // concluding with an offset indicating how many hours and minutes that specific time is ahead of or behind GMT/UTC.
    // YYYY-MM-DDTHH:mm:ss—Z

    if (value <= 0)
    {
        if (output_size > 0)
            output[0] = '\0';
        return;
    }

    struct tm local_tm{};
    localtime_r(&value, &local_tm);
    strftime(output, output_size, "%Y-%m-%dT%H:%M:%S%z", &local_tm);
}

#if MQTT_ENABLED

static void mqtt_enqueue_ntp_request(const struct sockaddr_storage &source_address)
{
    if (s_mqtt_ntp_event_queue == nullptr ||
        xQueueSend(s_mqtt_ntp_event_queue, &source_address, 0) != pdTRUE)
        s_ntp_telemetry_events_dropped.fetch_add(1, std::memory_order_relaxed);
}

static void mqtt_note_ntp_request(const struct sockaddr_storage &source_address)
{
    size_t address_size = 0;
    const void *address = nullptr;
    if (source_address.ss_family == AF_INET)
    {
        address = &reinterpret_cast<const struct sockaddr_in *>(&source_address)->sin_addr;
        address_size = sizeof(struct in_addr);
    }
    else if (source_address.ss_family == AF_INET6)
    {
        address = &reinterpret_cast<const struct sockaddr_in6 *>(&source_address)->sin6_addr;
        address_size = sizeof(struct in6_addr);
    }
    else
        return;

    if (s_mqtt_stats_mutex == nullptr || xSemaphoreTake(s_mqtt_stats_mutex, portMAX_DELAY) != pdTRUE)
        return;

#if MQTT_CLIENT_REPORTING_ENABLED

    for (size_t index = 0; index < s_mqtt_client_count; index++)
    {
        if (s_mqtt_clients[index].address_family == source_address.ss_family &&
            memcmp(s_mqtt_clients[index].address, address, address_size) == 0)
        {
            s_mqtt_clients[index].requests++;
            xSemaphoreGive(s_mqtt_stats_mutex);
            return;
        }
    }

    if (s_mqtt_client_count < (s_tf_queue_available.load() ? MQTT_TF_Client_Limit : MQTT_CLIENT_LIMIT))
    {
        mqtt_client_request_t &client = s_mqtt_clients[s_mqtt_client_count];
        client.address_family = source_address.ss_family;
        memcpy(client.address, address, address_size);
        client.requests = 1;
        s_mqtt_client_count++;
        xSemaphoreGive(s_mqtt_stats_mutex);
        return;
    }

#if DEBUG_ENABLED
    if (!s_mqtt_client_table_overflown)
    {
        char address_text[IP_ADDRESS_TEXT_SIZE] = "";
        format_socket_address(source_address, address_text, sizeof(address_text));
        ESP_LOGW(TAG, "Client table has overflowed, request for %s not recorded.", address_text);
    }
#endif

    s_mqtt_client_table_overflown = true;

#endif

    xSemaphoreGive(s_mqtt_stats_mutex);
}

static void mqtt_note_ethernet_disconnected()
{
    int64_t connected_since_us = s_eth_link_connected_us.exchange(0);
    if (connected_since_us > 0)
        s_eth_link_up_total_us.fetch_add(esp_timer_get_time() - connected_since_us);
}

static void mqtt_event_handler(void *arguments, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)arguments;
    (void)base;
    if (event_id == MQTT_EVENT_CONNECTED)
    {
        s_mqtt_connected.store(true);
        s_mqtt_has_connected.store(true);
        s_mqtt_disconnected_since_us.store(0);
    }
    else if (event_id == MQTT_EVENT_DISCONNECTED || event_id == MQTT_EVENT_ERROR)
    {
        s_mqtt_connected.store(false);
        if (s_mqtt_has_connected.load() && s_mqtt_disconnected_since_us.load() == 0)
            s_mqtt_disconnected_since_us.store(esp_timer_get_time());
    }
    else if (event_id == MQTT_EVENT_PUBLISHED)
    {
        auto *event = static_cast<esp_mqtt_event_t *>(event_data);
        if (event != nullptr && event->msg_id == s_mqtt_restart_publish_id.load())
            s_mqtt_restart_publish_completed.store(true);
    }
}

static bool mqtt_publish_report(const char *payload)
{
    if (!s_mqtt_connected.load())
        return false;

    return esp_mqtt_client_publish(s_mqtt_client, s_mqtt_report_topic, payload, 0, MQTT_QOS, MQTTBrokerRetain) >= 0;
}

static std::vector<std::string> mqtt_tf_queue_files()
{
    std::vector<std::string> files;
    DIR *directory = opendir(TF_QUEUE_DIRECTORY);
    if (directory == nullptr)
        return files;

    dirent *entry = nullptr;
    while ((entry = readdir(directory)) != nullptr)
    {
        unsigned long high = 0;
        unsigned long low = 0;
        if (strlen(entry->d_name) == 12 && sscanf(entry->d_name, "R%7lX.%3lX", &high, &low) == 2)
            files.emplace_back(std::string(TF_QUEUE_DIRECTORY) + "/" + entry->d_name);
    }
    closedir(directory);
    std::sort(files.begin(), files.end());
    return files;
}

static void mqtt_tf_refresh_queue_count()
{
    s_mqtt_queued_messages_count.store(mqtt_tf_queue_files().size());
}

static bool mqtt_tf_write_report(const char *payload)
{
    char path[128] = "";
    char temporary_path[128] = "";
    uint64_t sequence = s_tf_queue_next_sequence++;
    unsigned long high = static_cast<unsigned long>(sequence >> 12);
    unsigned long low = static_cast<unsigned long>(sequence & 0xFFF);
    snprintf(path, sizeof(path), "%s/R%07lX.%03lX", TF_QUEUE_DIRECTORY, high, low);
    snprintf(temporary_path, sizeof(temporary_path), "%s/R%07lX.TMP", TF_QUEUE_DIRECTORY, high);
    FILE *file = fopen(temporary_path, "wb");
    if (file == nullptr)
        return false;

    const size_t length = strlen(payload);
    bool written = fwrite(payload, 1, length, file) == length && fflush(file) == 0 && fsync(fileno(file)) == 0;
    fclose(file);
    if (!written || rename(temporary_path, path) != 0)
    {
        unlink(temporary_path);
        return false;
    }
    mqtt_tf_refresh_queue_count();
    return true;
}

static bool mqtt_enqueue_report(const char *payload)
{
    if (MQTT_QOS == 0)
        return false;

    if (s_tf_queue_available.load())
    {
        while (!mqtt_tf_write_report(payload))
        {
            std::vector<std::string> files = mqtt_tf_queue_files();
            if (files.empty() || unlink(files.front().c_str()) != 0)
            {
#if DEBUG_ENABLED
                ESP_LOGE(TAG, "Unable to queue MQTT report on TF card");
#endif
                return false;
            }
            s_mqtt_queued_messages_discarded++;
            mqtt_tf_refresh_queue_count();
        }
        return true;
    }

    if (s_mqtt_queued_messages_count.load() == MQTT_REPORT_QUEUE_DEPTH)
    {
        s_mqtt_report_head = (s_mqtt_report_head + 1) % MQTT_REPORT_QUEUE_DEPTH;
        s_mqtt_queued_messages_count--;
        s_mqtt_queued_messages_discarded++;
    }

    size_t index = (s_mqtt_report_head + s_mqtt_queued_messages_count.load()) % MQTT_REPORT_QUEUE_DEPTH;
    snprintf(s_mqtt_reports[index].payload, sizeof(s_mqtt_reports[index].payload), "%s", payload);
    s_mqtt_queued_messages_count++;
    return true;
}

static bool mqtt_publish_queued_report(const char *payload)
{
    if (MQTT_QOS != 2)
        return mqtt_publish_report(payload);

    s_mqtt_restart_publish_id.store(-1);
    s_mqtt_restart_publish_completed.store(false);
    int message_id = esp_mqtt_client_publish(s_mqtt_client, s_mqtt_report_topic, payload, 0, MQTT_QOS, MQTTBrokerRetain);
    if (message_id < 0)
        return false;

    s_mqtt_restart_publish_id.store(message_id);
    TickType_t wait_started = xTaskGetTickCount();
    while (!s_mqtt_restart_publish_completed.load() && s_mqtt_connected.load() &&
           (xTaskGetTickCount() - wait_started) < pdMS_TO_TICKS(MQTT_RESTART_PUBLISH_TIMEOUT_MS))
        vTaskDelay(pdMS_TO_TICKS(10));
    bool published = s_mqtt_restart_publish_completed.load();
    s_mqtt_restart_publish_id.store(-1);
    return published;
}

static void mqtt_send_queued_messages()
{
    if (MQTT_QOS == 0 || !s_mqtt_connected.load() || s_mqtt_queued_messages_count.load() == 0)
        return;

    if (!s_tf_queue_available.load())
    {
        if (mqtt_publish_queued_report(s_mqtt_reports[s_mqtt_report_head].payload))
        {
#if DEBUG_ENABLED
            ESP_LOGI(TAG, "Published queued message (RAM): %.200s", s_mqtt_reports[s_mqtt_report_head].payload);
#endif
            s_mqtt_report_head = (s_mqtt_report_head + 1) % MQTT_REPORT_QUEUE_DEPTH;
            s_mqtt_queued_messages_count--;
        }
        return;
    }

    std::vector<std::string> files = mqtt_tf_queue_files();
    if (files.empty())
    {
        mqtt_tf_refresh_queue_count();
        return;
    }

    FILE *file = fopen(files.front().c_str(), "rb");
    if (file == nullptr)
        return;
    std::string payload;
    char buffer[512];
    size_t read = 0;
    while ((read = fread(buffer, 1, sizeof(buffer), file)) > 0)
        payload.append(buffer, read);
    bool read_failed = ferror(file) != 0;
    fclose(file);
    if (!read_failed && mqtt_publish_queued_report(payload.c_str()))
    {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "Published queued message (TF): %.200s", payload.c_str());
#endif
        if (unlink(files.front().c_str()) == 0)
            mqtt_tf_refresh_queue_count();
    }
}

static uint32_t gnss_locked_seconds_this_period()
{
    static int64_t previous_total_us = 0;
    int64_t total_us = s_gnss_locked_total_us.load();
    int64_t lock_started_us = s_gnss_lock_started_us.load();

    if (s_gnss_locked.load() && lock_started_us > 0)
    {
        int64_t now_us = esp_timer_get_time();
        if (now_us > lock_started_us)
            total_us += now_us - lock_started_us;
    }

    int64_t period_us = total_us - previous_total_us;
    previous_total_us = total_us;
    if (period_us <= 0)
        return 0;

    return static_cast<uint32_t>(period_us / 1000000LL);
}

static void mqtt_build_report(char *payload, size_t payload_size)
{
    uint32_t queued_messages = static_cast<uint32_t>(s_mqtt_queued_messages_count.load());
    uint32_t queued_messages_discarded = s_mqtt_queued_messages_discarded;
    s_mqtt_queued_messages_discarded = 0;
    uint32_t most_requests_per_second = s_ntp_most_requests_per_second.exchange(0, std::memory_order_relaxed);

    char publishing_date_and_time[25] = "";
    format_time_to_ISO8601(time(nullptr), publishing_date_and_time, sizeof(publishing_date_and_time));

#if MQTT_HISTORICAL_REPORTING_ENABLED
    char last_synchronized_and_disciplined[25] = "";
    char last_gnss_unsynchronized[25] = "";
    char last_pps_undisciplined[25] = "";
    format_time_to_ISO8601(s_last_synchronized_and_disciplined.load(), last_synchronized_and_disciplined, sizeof(last_synchronized_and_disciplined));
    format_time_to_ISO8601(s_last_gnss_unsynchronized.load(), last_gnss_unsynchronized, sizeof(last_gnss_unsynchronized));
    format_time_to_ISO8601(s_last_pps_undisciplined.load(), last_pps_undisciplined, sizeof(last_pps_undisciplined));
#endif

    int64_t link_up_total_us = s_eth_link_up_total_us.load();
    int64_t connected_since_us = s_eth_link_connected_us.load();
    if (connected_since_us > 0)
        link_up_total_us += esp_timer_get_time() - connected_since_us;
    int64_t link_up_us = link_up_total_us - s_mqtt_last_link_up_us;
    s_mqtt_last_link_up_us = link_up_total_us;

    uint8_t satellite_min = s_satellite_min.exchange(UINT8_MAX);
    uint8_t satellite_max = s_satellite_max.exchange(0);
    if (satellite_min == UINT8_MAX)
    {
        satellite_min = s_satellite_count.load();
        satellite_max = satellite_min;
    }

    size_t len = 0;

    len += snprintf(payload + len, payload_size - len, "{");

    len += snprintf(payload + len, payload_size - len, "\"current\":{");
    len += snprintf(payload + len, payload_size - len, "\"time\":\"%s\",", publishing_date_and_time);
    len += snprintf(payload + len, payload_size - len, "\"uptime\":%lu,", (unsigned long)(esp_timer_get_time() / 1000000LL));
    len += snprintf(payload + len, payload_size - len, "\"ethernet_up\":%s,", s_ethernet_connected.load() ? "true" : "false");

    // to make this easy on the user - the following have been simplified for reporting so the user sees true when things are ok and false when they are not
    bool current_gnss_locked = s_gnss_locked.load();                 // Indicates whether GNSS currently has a satellite lock
    bool current_gnss_timing_valid = s_ntp_gnss_timing_valid.load(); // Tracks if GNSS timing data is valid
    bool current_gnss_valid = !s_ntp_gnss_invalid.load();            // Signals GNSS timing missing, expired, or unusable
    bool current_sync_fresh = !s_ntp_sync_stale.load();              // Shows if last successful sync is too old
    bool current_sanity_matched = !s_ntp_sanity_mismatch.load();     // Marks detected mismatch between GNSS time and sanity checks (if the gnss time is outside the timeframe that it should be)
    bool current_gnss_ok = current_gnss_locked && current_gnss_timing_valid && current_gnss_valid && current_sync_fresh && current_sanity_matched;

    len += snprintf(payload + len, payload_size - len, "\"gnss_synchronized\":%s,", current_gnss_ok ? "true" : "false");
    if (!current_gnss_ok)
    {
        len += snprintf(payload + len, payload_size - len, "\"gnss_synchronized_indicators\":{");
        len += snprintf(payload + len, payload_size - len, "\"locked\":%s,", current_gnss_locked ? "true" : "false");
        len += snprintf(payload + len, payload_size - len, "\"timing\":%s,", current_gnss_timing_valid ? "true" : "false");
        len += snprintf(payload + len, payload_size - len, "\"gnss_valid\":%s,", current_gnss_valid ? "true" : "false");
        len += snprintf(payload + len, payload_size - len, "\"sync_fresh\":%s,", current_sync_fresh ? "true" : "false");
        len += snprintf(payload + len, payload_size - len, "\"sanity_check_passed\":%s", current_sanity_matched ? "true" : "false");
        len += snprintf(payload + len, payload_size - len, "},");
    }

    // to make this easy on the user - the following have been simplified for reporting so the user sees true when things are ok and false when they are not
    bool current_pps_active = s_ntp_pps_active.load();                   // Is PPS present right now?
    bool current_pps_discipline_active = s_pps_discipline_active.load(); // Is the PPS disciplining algorithm engaged?
    bool current_pps_synchronized = !s_ntp_pps_missing.load();           // Is PPS missing in a way that constitutes a synchronization fault?
    bool current_pps_ok = current_pps_active && current_pps_discipline_active && current_pps_synchronized;

    len += snprintf(payload + len, payload_size - len, "\"pps_disciplined\":%s,", current_pps_ok ? "true" : "false");
    if (!current_pps_ok)
    {
        len += snprintf(payload + len, payload_size - len, "\"pps_disciplined_indicators\":{");
        len += snprintf(payload + len, payload_size - len, "\"pps_signals_present\":%s,", current_pps_active ? "true" : "false");
        len += snprintf(payload + len, payload_size - len, "\"discipline_active\":%s,", current_pps_discipline_active ? "true" : "false");
        len += snprintf(payload + len, payload_size - len, "\"pps_synchronized\":%s", current_pps_synchronized ? "true" : "false");
        len += snprintf(payload + len, payload_size - len, "},");
    }

    len += snprintf(payload + len, payload_size - len, "\"satellites\":%u", (unsigned int)s_satellite_count.load());

#if MQTT_MEMORY_REPORTING_ENABLED
    len += snprintf(payload + len, payload_size - len, ",");
    len += snprintf(payload + len, payload_size - len, "\"memory\":{");
    // len += snprintf(payload + len, payload_size - len, "\"malloc_cap_8bit\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT));
    // len += snprintf(payload + len, payload_size - len, "\"malloc_cap_32bit\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_32BIT));
    // len += snprintf(payload + len, payload_size - len, "\"malloc_cap_internal\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    len += snprintf(payload + len, payload_size - len, "\"malloc_cap_dma\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
    // len += snprintf(payload + len, payload_size - len, "\"malloc_cap_spiram\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    // len += snprintf(payload + len, payload_size - len, "\"malloc_cap_default\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    len += snprintf(payload + len, payload_size - len, "\"free_heap\":%lu,", (unsigned long)esp_get_free_heap_size());
    len += snprintf(payload + len, payload_size - len, "\"minimum_free_heap\":%lu,", (unsigned long)esp_get_minimum_free_heap_size());
    len += snprintf(payload + len, payload_size - len, "\"largest_free_8bit_block\":%lu", (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    len += snprintf(payload + len, payload_size - len, "}},");

#else
    len += snprintf(payload + len, payload_size - len, "},");
#endif

    if (MQTT_QOS > 0)
    {
        len += snprintf(payload + len, payload_size - len, "\"queued_messages\":{");
        len += snprintf(payload + len, payload_size - len, "\"held\":%lu,", (unsigned long)queued_messages);
        len += snprintf(payload + len, payload_size - len, "\"discarded\":%lu", (unsigned long)queued_messages_discarded);
        len += snprintf(payload + len, payload_size - len, "},");
    };

    len += snprintf(payload + len, payload_size - len, "\"this_period\":{");
    len += snprintf(payload + len, payload_size - len, "\"ethernet_up_secs\":%lld,", (long long)(link_up_us / 1000000LL));
    len += snprintf(payload + len, payload_size - len, "\"pps_pulses\":%lu,", (unsigned long)s_pps_pulses.exchange(0));
    len += snprintf(payload + len, payload_size - len, "\"gnss_locked_secs\":%lu,", (unsigned long)gnss_locked_seconds_this_period());
    len += snprintf(payload + len, payload_size - len, "\"satellites\":{");
    len += snprintf(payload + len, payload_size - len, "\"min\":%u,", (unsigned int)satellite_min);
    len += snprintf(payload + len, payload_size - len, "\"max\":%u", (unsigned int)satellite_max);
    len += snprintf(payload + len, payload_size - len, "},");
    len += snprintf(payload + len, payload_size - len, "\"ntp\":{");
    len += snprintf(payload + len, payload_size - len, "\"requests\":{");
    len += snprintf(payload + len, payload_size - len, "\"valid\":%lu,", (unsigned long)s_ntp_valid_requests.exchange(0));
    len += snprintf(payload + len, payload_size - len, "\"invalid\":%lu,", (unsigned long)s_ntp_invalid_requests.exchange(0));
    len += snprintf(payload + len, payload_size - len, "\"telemetry_dropped\":%lu,", (unsigned long)s_ntp_telemetry_events_dropped.exchange(0));
    len += snprintf(payload + len, payload_size - len, "\"max_per_second\":%lu", (unsigned long)most_requests_per_second);
    len += snprintf(payload + len, payload_size - len, "},");
    len += snprintf(payload + len, payload_size - len, "\"responses\":{");
    len += snprintf(payload + len, payload_size - len, "\"synchronized_and_disciplined\":%lu,", (unsigned long)s_ntp_responses_synchronized_and_disciplined.exchange(0));
    len += snprintf(payload + len, payload_size - len, "\"gnss_unsynchronized\":%lu,", (unsigned long)s_ntp_responses_gnss_unsynchronized.exchange(0));
    len += snprintf(payload + len, payload_size - len, "\"pps_undisciplined\":%lu", (unsigned long)s_ntp_responses_pps_undisciplined.exchange(0));
    len += snprintf(payload + len, payload_size - len, "}");

#if MQTT_CLIENT_REPORTING_ENABLED
    len += snprintf(payload + len, payload_size - len, "},");
    len += snprintf(payload + len, payload_size - len, "\"clients\":[");
    bool first_client = true;
    if (xSemaphoreTake(s_mqtt_stats_mutex, portMAX_DELAY) == pdTRUE)
    {
        std::sort(s_mqtt_clients, s_mqtt_clients + s_mqtt_client_count,
                  [](const mqtt_client_request_t &left, const mqtt_client_request_t &right)
                  {
                      auto address_family_order = [](sa_family_t address_family)
                      {
                          if (address_family == AF_INET)
                              return 0;
                          if (address_family == AF_INET6)
                              return 1;
                          return 2;
                      };
                      int left_order = address_family_order(left.address_family);
                      int right_order = address_family_order(right.address_family);
                      if (left_order != right_order)
                          return left_order < right_order;
                      size_t address_size = left.address_family == AF_INET ? sizeof(struct in_addr) : sizeof(struct in6_addr);
                      return memcmp(left.address, right.address, address_size) < 0;
                  });
        for (size_t index = 0; index < s_mqtt_client_count && len < payload_size; index++)
        {
            struct sockaddr_storage address{};
            address.ss_family = s_mqtt_clients[index].address_family;
            if (address.ss_family == AF_INET)
                memcpy(&reinterpret_cast<struct sockaddr_in *>(&address)->sin_addr, s_mqtt_clients[index].address, sizeof(struct in_addr));
            else if (address.ss_family == AF_INET6)
                memcpy(&reinterpret_cast<struct sockaddr_in6 *>(&address)->sin6_addr, s_mqtt_clients[index].address, sizeof(struct in6_addr));
            else
                continue;

            char address_text[IP_ADDRESS_TEXT_SIZE] = "";
            if (!format_socket_address(address, address_text, sizeof(address_text)))
                continue;
            int written = snprintf(payload + len, payload_size - len,
                                   "%s{\"address\":\"%s\",\"requests\":%lu}",
                                   first_client ? "" : ",", address_text,
                                   static_cast<unsigned long>(s_mqtt_clients[index].requests));
            if (written < 0 || static_cast<size_t>(written) >= payload_size - len)
                break;
            len += static_cast<size_t>(written);
            first_client = false;
        }
        s_mqtt_client_count = 0;
        xSemaphoreGive(s_mqtt_stats_mutex);
    }
    len += snprintf(payload + len, payload_size - len, "],");
    len += snprintf(payload + len, payload_size - len, "\"clients_overflown\":%s", s_mqtt_client_table_overflown.exchange(false) ? "true" : "false");
#else
    len += snprintf(payload + len, payload_size - len, "}");
#endif

#if MQTT_HISTORICAL_REPORTING_ENABLED
    len += snprintf(payload + len, payload_size - len, "},");
    len += snprintf(payload + len, payload_size - len, "\"historical\":{");
    len += snprintf(payload + len, payload_size - len, "\"gnss_receiver_last\":{");
    len += snprintf(payload + len, payload_size - len, "\"synchronized_and_disciplined\":\"%s\",", last_synchronized_and_disciplined);
    len += snprintf(payload + len, payload_size - len, "\"gnss_unsynchronized\":\"%s\",", last_gnss_unsynchronized);
    len += snprintf(payload + len, payload_size - len, "\"pps_undisciplined\":\"%s\"", last_pps_undisciplined);
    len += snprintf(payload + len, payload_size - len, "}");
#endif

    len += snprintf(payload + len, payload_size - len, "}");
    len += snprintf(payload + len, payload_size - len, "}");

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "Published:");
    ESP_LOGI(TAG, "\n\r%s", payload);
#endif
}

static void mqtt_publish_final_report()
{
    if (!s_mqtt_connected.load())
        return;

    char payload[MQTT_REPORT_SIZE] = "";
    mqtt_build_report(payload, sizeof(payload));

    s_mqtt_restart_publish_id.store(-1);
    s_mqtt_restart_publish_completed.store(false);
    int message_id = esp_mqtt_client_publish(s_mqtt_client, s_mqtt_report_topic, payload, 0, MQTT_QOS, MQTTBrokerRetain);
    if (message_id < 0)
        return;

    s_mqtt_restart_publish_id.store(message_id);
    TickType_t wait_started = xTaskGetTickCount();
    TickType_t wait_timeout = pdMS_TO_TICKS(MQTT_RESTART_PUBLISH_TIMEOUT_MS);
    while (!s_mqtt_restart_publish_completed.load() && (xTaskGetTickCount() - wait_started) < wait_timeout)
        vTaskDelay(pdMS_TO_TICKS(10));

    s_mqtt_restart_publish_id.store(-1);
}

static bool mqtt_publish_or_queue_restart_notification(const char *reason)
{
    char payload[128] = "";
    snprintf(payload, sizeof(payload), "{\"event\":\"controlled_restart\",\"reason\":\"%s\"}", reason);

    bool published = false;
    if (s_mqtt_connected.load())
    {
        s_mqtt_restart_publish_id.store(-1);
        s_mqtt_restart_publish_completed.store(false);
        int message_id = esp_mqtt_client_publish(s_mqtt_client, s_mqtt_report_topic, payload, 0, MQTT_QOS, MQTTBrokerRetain);
        if (message_id >= 0)
        {
            s_mqtt_restart_publish_id.store(message_id);
            TickType_t wait_started = xTaskGetTickCount();
            while (!s_mqtt_restart_publish_completed.load() && s_mqtt_connected.load() &&
                   (xTaskGetTickCount() - wait_started) < pdMS_TO_TICKS(MQTT_RESTART_PUBLISH_TIMEOUT_MS))
                vTaskDelay(pdMS_TO_TICKS(10));
            published = s_mqtt_restart_publish_completed.load();
        }
        s_mqtt_restart_publish_id.store(-1);
    }

    if (published)
        return true;
    if (s_tf_queue_available.load())
        return mqtt_enqueue_report(payload);

    nvs_handle_t handle = 0;
    esp_err_t result = nvs_open(restart_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (result == ESP_OK)
    {
        result = nvs_set_str(handle, restart_NVS_KEY_PENDING, payload);
        if (result == ESP_OK)
            result = nvs_commit(handle);
        nvs_close(handle);
    }
    return result == ESP_OK;
}

static bool mqtt_publish_pending_restart_notification()
{
    if (!s_mqtt_connected.load())
        return false;

    char payload[512] = "";
    nvs_handle_t handle = 0;
    if (nvs_open(restart_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        return false;

    size_t payload_size = sizeof(payload);
    esp_err_t result = nvs_get_str(handle, restart_NVS_KEY_PENDING, payload, &payload_size);
    if (result == ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return false;
    }
    if (result != ESP_OK)
    {
        nvs_close(handle);
        return false;
    }

    s_mqtt_restart_publish_id.store(-1);
    s_mqtt_restart_publish_completed.store(false);
    int message_id = esp_mqtt_client_publish(s_mqtt_client, s_mqtt_report_topic, payload, 0, MQTT_QOS, MQTTBrokerRetain);
    if (message_id >= 0)
    {
        s_mqtt_restart_publish_id.store(message_id);
        TickType_t wait_started = xTaskGetTickCount();
        while (!s_mqtt_restart_publish_completed.load() && s_mqtt_connected.load() &&
               (xTaskGetTickCount() - wait_started) < pdMS_TO_TICKS(MQTT_RESTART_PUBLISH_TIMEOUT_MS))
            vTaskDelay(pdMS_TO_TICKS(10));
    }
    const bool published = s_mqtt_restart_publish_completed.load();
    s_mqtt_restart_publish_id.store(-1);
    if (published)
    {
        result = nvs_erase_key(handle, restart_NVS_KEY_PENDING);
        if (result == ESP_OK)
            result = nvs_commit(handle);
    }
    nvs_close(handle);
    return published && result == ESP_OK;
}

static void mqtt_service_task(void *parameter)
{
    bool previously_connected = false;
    TickType_t next_report = xTaskGetTickCount() + pdMS_TO_TICKS(MQTTReportingPeriod * 1000UL);
    for (;;)
    {
        struct sockaddr_storage source_address{};
        for (size_t count = 0; count < MQTT_NTP_EVENT_BATCH_LIMIT &&
                               xQueueReceive(s_mqtt_ntp_event_queue, &source_address, 0) == pdTRUE;
             ++count)
            mqtt_note_ntp_request(source_address);

        bool connected = s_mqtt_connected.load();
        if (connected && !previously_connected)
            esp_mqtt_client_publish(s_mqtt_client, s_mqtt_status_topic, "online", 0, MQTT_QOS, 1);
        previously_connected = connected;

        if (connected)
            mqtt_publish_pending_restart_notification();

        if (connected && s_mqtt_queued_messages_count.load() > 0)
            mqtt_send_queued_messages();

        if (xTaskGetTickCount() >= next_report)
        {
            mqtt_build_report(s_mqtt_payload, sizeof(s_mqtt_payload));
            if (!mqtt_publish_report(s_mqtt_payload))
                mqtt_enqueue_report(s_mqtt_payload);
            next_report += pdMS_TO_TICKS(MQTTReportingPeriod * 1000UL);
        }
        vTaskDelay(pdMS_TO_TICKS(MQTT_QUEUED_PUBLISH_DELAY_MS));

#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(MQTT_Service);
#endif
    }
}

static void mqtt_log_tf_directory(const char *path)
{
#if DEBUG_ENABLED
    DIR *directory = opendir(path);
    if (directory == nullptr)
        return;
    dirent *entry = nullptr;
    while ((entry = readdir(directory)) != nullptr)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;
        std::string child_path = std::string(path) + "/" + entry->d_name;
        struct stat information{};
        if (stat(child_path.c_str(), &information) != 0)
            continue;
        ESP_LOGI(TAG, "%s", child_path.c_str());
        if (S_ISDIR(information.st_mode))
            mqtt_log_tf_directory(child_path.c_str());
    }
    closedir(directory);
#else
    (void)path;
#endif
}

#endif

static void setup_mqtt_tf_queue()
{

#if MQTT_ENABLED

    if (MQTT_QOS == 0)
        return;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sd_pwr_ctrl_ldo_config_t ldo_config{};
    ldo_config.ldo_chan_id = 4;
    sd_pwr_ctrl_handle_t power_control = nullptr;
    esp_err_t result = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &power_control);
    if (result != ESP_OK)
    {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "Unable to enable TF card power: %s. Queued MQTT messages will be stored in ram.", esp_err_to_name(result));
#endif
        return;
    }
    host.pwr_ctrl_handle = power_control;
    auto cleanup_power_control = [&power_control]()
    {
        if (power_control != nullptr)
        {
            sd_pwr_ctrl_del_on_chip_ldo(power_control);
            power_control = nullptr;
        }
    };

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;
    slot.clk = static_cast<gpio_num_t>(TFCardClockPin);
    slot.cmd = static_cast<gpio_num_t>(TFCardCommandPin);
    slot.d0 = static_cast<gpio_num_t>(TFCardData0Pin);
    slot.d1 = static_cast<gpio_num_t>(TFCardData1Pin);
    slot.d2 = static_cast<gpio_num_t>(TFCardData2Pin);
    slot.d3 = static_cast<gpio_num_t>(TFCardData3Pin);
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    esp_vfs_fat_mount_config_t mount_config{};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 4;
    mount_config.allocation_unit_size = 16 * 1024;

    result = esp_vfs_fat_sdmmc_mount(TF_MOUNT_POINT, &host, &slot, &mount_config, &s_tf_card);
    if (result != ESP_OK)
    {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "TF card unavailable: %s. Queued MQTT messages will be stored in ram.", esp_err_to_name(result));
#endif
        cleanup_power_control();
        return;
    }

    FATFS *filesystem = nullptr;
    DWORD free_clusters = 0;
    if (f_getfree("0:", &free_clusters, &filesystem) != FR_OK || filesystem == nullptr || filesystem->fs_type != FS_FAT32)
    {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "TF card is not FAT32. Queued MQTT messages will be stored in ram.");
#endif
        esp_vfs_fat_sdcard_unmount(TF_MOUNT_POINT, s_tf_card);
        s_tf_card = nullptr;
        cleanup_power_control();
        return;
    }

    mqtt_log_tf_directory(TF_MOUNT_POINT);
    bool queue_directory_created = mkdir(TF_QUEUE_DIRECTORY, 0775) == 0;
    if (!queue_directory_created && errno != EEXIST)
    {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "Unable to create TF MQTT queue directory. Queued MQTT messages will be stored in ram.");
#endif
        esp_vfs_fat_sdcard_unmount(TF_MOUNT_POINT, s_tf_card);
        s_tf_card = nullptr;
        cleanup_power_control();
        return;
    }
#if DEBUG_ENABLED
    if (queue_directory_created)
        ESP_LOGI(TAG, "%s", TF_QUEUE_DIRECTORY);
#endif

    char probe_path[128] = "";
    snprintf(probe_path, sizeof(probe_path), "%s/PROBE.TMP", TF_QUEUE_DIRECTORY);
    FILE *probe = fopen(probe_path, "wb+");
    char probe_value = 0;
    bool usable = probe != nullptr && fwrite("T", 1, 1, probe) == 1 && fflush(probe) == 0 &&
                  fseek(probe, 0, SEEK_SET) == 0 && fread(&probe_value, 1, 1, probe) == 1 && probe_value == 'T';
    if (probe != nullptr)
        fclose(probe);
    unlink(probe_path);
    if (!usable)
    {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "TF card read/write verification failed. Queued MQTT messages will be stored in ram.");
#endif
        esp_vfs_fat_sdcard_unmount(TF_MOUNT_POINT, s_tf_card);
        s_tf_card = nullptr;
        cleanup_power_control();
        return;
    }

    std::vector<std::string> files = mqtt_tf_queue_files();
    for (const std::string &path : files)
    {
        unsigned long high = 0;
        unsigned long low = 0;
        if (sscanf(path.c_str(), "/tfcard/Queue/R%7lX.%3lX", &high, &low) == 2)
        {
            uint64_t sequence = (static_cast<uint64_t>(high) << 12) | low;
            if (sequence >= s_tf_queue_next_sequence)
                s_tf_queue_next_sequence = sequence + 1;
        }
    }
    s_mqtt_queued_messages_count.store(files.size());
    s_tf_queue_available.store(true);
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "TF card MQTT queue enabled");
    if (files.empty())
        ESP_LOGI(TAG, "No queued messages found");
#endif

#endif
}

static void setup_mqtt()
{
#if MQTT_ENABLED

    if (MQTTServerIPAddress[0] == '\0' || MQTT_QOS < 0 || MQTT_QOS > 2)
    {
        s_mqtt_setup_failed.store(true);
        return;
    }

    s_mqtt_stats_mutex = xSemaphoreCreateMutex();
    s_mqtt_ntp_event_queue = xQueueCreate(MQTT_NTP_EVENT_QUEUE_DEPTH, sizeof(struct sockaddr_storage));
    if (s_mqtt_stats_mutex == nullptr || s_mqtt_ntp_event_queue == nullptr)
    {
        s_mqtt_setup_failed.store(true);
        return;
    }
    snprintf(s_mqtt_uri, sizeof(s_mqtt_uri), "mqtt://%s:%u", MQTTServerIPAddress, static_cast<unsigned int>(MQTTPort));
    snprintf(s_mqtt_report_topic, sizeof(s_mqtt_report_topic), "%s/report", MQTTTopic);
    snprintf(s_mqtt_status_topic, sizeof(s_mqtt_status_topic), "%s/status", MQTTTopic);
    esp_mqtt_client_config_t config{};
    config.broker.address.uri = s_mqtt_uri;
    config.credentials.username = MQTTUsername;
    config.credentials.authentication.password = MQTTPassword;
    config.session.keepalive = MQTTFrequencyOfKeepAliveRequest;
    config.session.last_will.topic = s_mqtt_status_topic;
    config.session.last_will.msg = "offline";
    config.session.last_will.qos = MQTT_QOS;
    config.session.last_will.retain = 1;
    config.task.priority = 23;
    s_mqtt_client = esp_mqtt_client_init(&config);
    if (s_mqtt_client == nullptr)
    {
        s_mqtt_setup_failed.store(true);
        return;
    }
    if (esp_mqtt_client_register_event(s_mqtt_client, MQTT_EVENT_ANY, mqtt_event_handler, nullptr) != ESP_OK)
    {
        s_mqtt_setup_failed.store(true);
        return;
    }
    if (esp_mqtt_client_start(s_mqtt_client) != ESP_OK)
    {
        s_mqtt_setup_failed.store(true);
        return;
    }

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "MQTT setup. Keep alive set at %u seconds", MQTTFrequencyOfKeepAliveRequest);
#endif
    if (xTaskCreatePinnedToCore(mqtt_service_task, "mqtt_service", MQTT_Service_Task_Stack_Size, nullptr, 5, nullptr, 0) != pdPASS ||
        xTaskCreatePinnedToCore(ethernet_transport_recovery_task, "eth_recovery", Ethernet_Transport_Recovery_Task_Stack_Size, nullptr, 6, nullptr, 0) != pdPASS)
        s_mqtt_setup_failed.store(true);
#endif
}

static void arduino_eth_event_handler(arduino_event_id_t event, arduino_event_info_t info)
{
    switch (event)
    {
    case ARDUINO_EVENT_ETH_START:
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "Ethernet driver started");
#endif
        ETH.setHostname(DeviceName);
        display_line(1, "Ethernet started");
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        s_ethernet_connected.store(true);
#if MQTT_ENABLED
        s_eth_link_connected_us.store(esp_timer_get_time());
#endif
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "Ethernet link connected");
#endif
        xEventGroupSetBits(s_net_event_group, ETH_CONNECTED_BIT);
        display_line(1, "Ethernet connected");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        snprintf(s_ipv4_address, sizeof(s_ipv4_address), IPSTR, IP2STR(&info.got_ip.ip_info.ip));
        update_selected_ip_address();
#if DEBUG_ENABLED
        ESP_LOGI(TAG,
                 "Ethernet IPv4 acquired: ip=" IPSTR ", mask=" IPSTR ", gw=" IPSTR,
                 IP2STR(&info.got_ip.ip_info.ip),
                 IP2STR(&info.got_ip.ip_info.netmask),
                 IP2STR(&info.got_ip.ip_info.gw));
#endif
        xEventGroupSetBits(s_net_event_group, ETH_GOT_IP_BIT);
        display_selected_ip_address(static_cast<int>(time(nullptr) % 10));
        break;
    case ARDUINO_EVENT_ETH_GOT_IP6:
        if (inet_ntop(AF_INET6, &info.got_ip6.ip6_info.ip, s_ipv6_address, sizeof(s_ipv6_address)) == nullptr)
            s_ipv6_address[0] = '\0';
        update_selected_ip_address();
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "Ethernet IPv6 acquired: %s", s_ipv6_address);
#endif
        xEventGroupSetBits(s_net_event_group, ETH_GOT_IP6_BIT);
        display_selected_ip_address(static_cast<int>(time(nullptr) % 10));
        break;
    case ARDUINO_EVENT_ETH_LOST_IP:
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "Ethernet lost IPv4 address");
#endif
        xEventGroupClearBits(s_net_event_group, ETH_GOT_IP_BIT);
        s_ipv4_address[0] = '\0';
        update_selected_ip_address();
        display_line(1, "Ethernet lost IP");
        display_selected_ip_address(static_cast<int>(time(nullptr) % 10));
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        s_ethernet_connected.store(false);
#if MQTT_ENABLED
        mqtt_note_ethernet_disconnected();
#endif
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "Ethernet link disconnected");
#endif
        xEventGroupClearBits(s_net_event_group, ETH_CONNECTED_BIT | ETH_GOT_IP_BIT | ETH_GOT_IP6_BIT);
        s_ipv4_address[0] = '\0';
        s_ipv6_address[0] = '\0';
        update_selected_ip_address();
        display_line(1, "Ethernet disconnect");
        display_line(3, "");
        break;
    case ARDUINO_EVENT_ETH_STOP:
        s_ethernet_connected.store(false);
#if MQTT_ENABLED
        mqtt_note_ethernet_disconnected();
#endif
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "Ethernet driver stopped");
#endif
        xEventGroupClearBits(s_net_event_group, ETH_CONNECTED_BIT | ETH_GOT_IP_BIT | ETH_GOT_IP6_BIT);
        s_ipv4_address[0] = '\0';
        s_ipv6_address[0] = '\0';
        update_selected_ip_address();
        display_line(1, "Ethernet stopped");
        display_line(3, "");
        break;
    default:
        break;
    }
}

// Apply the optional static IP address configuration from ESP32TimeServerSettings.h.
// When StaticIPAddress is empty this is a no-op and the Ethernet interface falls back
// to DHCP (the default behaviour). Returns true when a static IP was applied, false otherwise.
static bool configure_static_ip()
{
    // An empty StaticIPAddress means DHCP should be used - nothing to do here
    if (StaticIPAddress[0] == '\0')
        return false;

    IPAddress local_ip, gateway, subnet;
    if (!local_ip.fromString(StaticIPAddress))
    {
        ESP_LOGE(TAG, "Configured StaticIPAddress is invalid: %s", StaticIPAddress);
        return false;
    }
    if (!gateway.fromString(Gateway))
    {
        ESP_LOGE(TAG, "Configured Gateway is invalid: %s", Gateway);
        return false;
    }
    if (!subnet.fromString(SubnetMask))
    {
        ESP_LOGE(TAG, "Configured SubnetMask is invalid: %s", SubnetMask);
        return false;
    }

    // DNS servers are optional; default to 0.0.0.0 (unset) when left blank
    IPAddress dns1, dns2;
    if (PrimaryDNS[0] != '\0')
    {
        if (!dns1.fromString(PrimaryDNS))
        {
            ESP_LOGE(TAG, "Configured PrimaryDNS is invalid: %s", PrimaryDNS);
            return false;
        }
    }
    if (SecondaryDNS[0] != '\0')
    {
        if (!dns2.fromString(SecondaryDNS))
        {
            ESP_LOGE(TAG, "Configured SecondaryDNS is invalid: %s", SecondaryDNS);
            return false;
        }
    }

    if (!ETH.config(local_ip, gateway, subnet, dns1, dns2))
    {
        ESP_LOGE(TAG, "ETH.config() failed to apply the static IP configuration");
        return false;
    }

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "Static IP configuration applied: ip=%s, mask=%s, gw=%s",
             StaticIPAddress, SubnetMask, Gateway);
#endif
    return true;
}

static bool start_ethernet_driver()
{
    if (!ETH.enableIPv6())
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Unable to enable Ethernet IPv6 support");
#endif
    }

    return ETH.begin(ETH_PHY_IP101,
                     ETH_PHY_ADDRESS,
                     static_cast<int>(ETH_MDC_GPIO),
                     static_cast<int>(ETH_MDIO_GPIO),
                     static_cast<int>(ETH_PHY_RST_GPIO),
                     EMAC_CLK_EXT_IN);
}

static void setup_ethernet()
{
    if (s_net_event_group == nullptr)
        s_net_event_group = xEventGroupCreate();

    s_ip_address[0] = '\0';
    s_ipv4_address[0] = '\0';
    s_ipv6_address[0] = '\0';
    xEventGroupClearBits(s_net_event_group, ETH_CONNECTED_BIT | ETH_GOT_IP_BIT | ETH_GOT_IP6_BIT);

    Network.onEvent(arduino_eth_event_handler);

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "Starting Arduino Ethernet with phy_addr=%d, mdc=%d, mdio=%d, power=%d",
             ETH_PHY_ADDRESS,
             static_cast<int>(ETH_MDC_GPIO),
             static_cast<int>(ETH_MDIO_GPIO),
             static_cast<int>(ETH_PHY_RST_GPIO));
#endif

    if (!start_ethernet_driver()) // needed for hardware time stamping
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "ETH.begin() failed");
#endif
        return;
    }

    if (!ntp_cache_init() ||
        (s_hardware_ntp_transmit_mutex = xSemaphoreCreateMutex()) == nullptr ||
        (s_hardware_ntp_request_queue = xQueueCreate(HARDWARE_NTP_REQUEST_QUEUE_DEPTH, sizeof(hardware_ntp_request_t))) == nullptr ||
        (s_hardware_ntp_request_buffer_queue = xQueueCreate(HARDWARE_NTP_REQUEST_BUFFER_COUNT, sizeof(uint8_t *))) == nullptr ||
        !initialize_hardware_ntp_request_buffers() ||
        xTaskCreatePinnedToCore(ntp_cache_purge_task, "ntp_cache_purge", NTP_Cache_Purge_Task_Stack_Size, nullptr, 5, &s_ntp_cache_purge_task_handle, tskNO_AFFINITY) != pdPASS ||
        xTaskCreatePinnedToCore(hardware_ntp_server_task, "hardware_ntp", Hardware_NTP_Server_Task_Stack_Size, nullptr, 17, &s_hardware_ntp_server_task_handle, 1) != pdPASS) // do not change the task priority higher than 17 causes conflicts with other mechanisms
    {
        ESP_LOGE(TAG, "Unable to initialize hardware NTP serving");
        deinitialize_hardware_ntp_server();
    }
    else if (!configure_hardware_timestamps())
    {
        ESP_LOGE(TAG, "Unable to configure Ethernet hardware timestamping");
    }

    // Apply the optional static IP address (if configured). When StaticIPAddress is
    // left empty this is a no-op and DHCP is used as usual.
    bool static_ip_applied = configure_static_ip();

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "Waiting for Ethernet %s address...",
             static_ip_applied ? "static" : "DHCP");
#endif
    xEventGroupWaitBits(s_net_event_group, ETH_GOT_IP_BIT | ETH_GOT_IP6_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "Ethernet setup complete, current address: %s", s_ip_address[0] == '\0' ? "<none>" : s_ip_address);
#endif
}

void write_opening_messages_to_the_console()
{

    // Note: the console writes in the routine are purposefully not guarded by a DEBUG_ENABLE check - they should always be written

    Serial.begin(serialMonitorSpeed);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "******************* Application Startup *******************");

    // the values below are drawn from the CMakeLists.txt files (one in the root folder and one in the /main folder)
    const app_metadata_t *meta = get_app_metadata();
    ESP_LOGI(TAG, "%s v%s", meta->project_name, meta->version);
    ESP_LOGI(TAG, "%s", meta->copyright);
    ESP_LOGI(TAG, "License: %s", meta->license);
    ESP_LOGI(TAG, "Website: %s", meta->homepage);
    ESP_LOGI(TAG, " ");

#if RBG_LED_ENABLED
    ESP_LOGI(TAG, "LED support: Enabled");
#else
    ESP_LOGW(TAG, "LED support: Disabled");
#endif

#if LIQUID_CRYSTAL_DISPLAY_ENABLED
    ESP_LOGI(TAG, "LCD support: Enabled");
#else
    ESP_LOGW(TAG, "LCD support: Disabled");
#endif

#if UPTIME_RESTART_BUTTON_ENABLED
    ESP_LOGI(TAG, "Uptime / Reset button support: Enabled");
#else
    ESP_LOGW(TAG, "Uptime / Reset button support: Disabled");
#endif

#if OTE_UPDATES_ENABLED
    ESP_LOGI(TAG, "Over the Ethernet update support: Enabled");
#else
    ESP_LOGW(TAG, "Over the Ethernet update support: Disabled");
#endif

#if MQTT_ENABLED
    ESP_LOGI(TAG, "MQTT support: Enabled");

#if MQTT_CLIENT_REPORTING_ENABLED
    ESP_LOGI(TAG, "MQTT client reporting: Enabled");
#else
    ESP_LOGW(TAG, "MQTT client reporting: Disabled");
#endif

#if MQTT_MEMORY_REPORTING_ENABLED
    ESP_LOGI(TAG, "MQTT memory reporting: Enabled");
#else
    ESP_LOGW(TAG, "MQTT memory reporting: Disabled");
#endif

#if MQTT_HISTORICAL_REPORTING_ENABLED
    ESP_LOGI(TAG, "MQTT historical reporting: Enabled");
#else
    ESP_LOGW(TAG, "MQTT historical reporting: Disabled");
#endif

#else
    ESP_LOGW(TAG, "MQTT support: Disabled");
#endif

    ESP_LOGI(TAG, "");
}

void write_open_for_business_messages_to_the_console()
{

    // The line of code below 'opens the gate' for external NTP responses to be processed.
    //
    // Gating criteria:
    //  - An IPv4 or IPv6 address has been acquired; either one is sufficient to facilitate external NTP responses to be processed
    //  - The time has been synchronized and the PPS disciplined
    //  - The NTP server is running and ready to respond to external NTP requests
    //
    // Non-gating criteria:
    //  - MQTT need not be connected to its broker
    //    if its broker connection is unavailable, MQTT messages will be queued if the QOS > 0
    //  - IPv4 and IPv6 addresses need not be currently available;
    //    although at least one has been recently available as evidence by at least one acquired IP address
    //

    if (s_ipv4_address[0] != '\0')
        ESP_LOGI(TAG, "The IPv4 connection is up (%s)", s_ipv4_address);
    else
        ESP_LOGW(TAG, "The IPv4 connection is down");

    if (s_ipv6_address[0] != '\0')
        ESP_LOGI(TAG, "The IPv6 connection is up (%s)", s_ipv6_address);
    else
        ESP_LOGW(TAG, "The IPv6 connection is down");

    s_ntp_external_responses_enabled.store(true, std::memory_order_release);

    // Note: 'Open for business' message is purposefully not guarded by a DEBUG_ENABLE check - it should always be written to the console.
    char s_open_for_business_date_and_time[25] = "";
    format_time_to_ISO8601(time(nullptr), s_open_for_business_date_and_time, sizeof(s_open_for_business_date_and_time));
    ESP_LOGI(TAG, "***********************************************");
    ESP_LOGI(TAG, "* Open for business: %s *", s_open_for_business_date_and_time);
    ESP_LOGI(TAG, "***********************************************");
    ESP_LOGI(TAG, " ");

#if RBG_LED_ENABLED
    s_open_for_business_message_written.store(true, std::memory_order_release);
#endif

#if DEBUG_ENABLED
#else
    ESP_LOGW(TAG, "DEBUG_ENABLED is disabled in the settings file; this will be the last console message from main_cpp");
#endif
}

void setup_NVS_storage(void)
{
    if (!initialize_nvs_storage())
        ESP_LOGE(TAG, "NVS initialization failed. GNSS module settings persistence is unavailable.");
}

void create_mutexes_and_semaphores(void)
{

    s_time_mutex = xSemaphoreCreateMutex();
    s_pps_semaphore = xSemaphoreCreateBinary();
    s_pps_timestamp_queue = xQueueCreate(1, sizeof(PpsCaptureEvent));
    s_pps_sync_timestamp_queue = xQueueCreate(1, sizeof(PpsCaptureEvent));
    s_ote_mutex = xSemaphoreCreateMutex();
    s_sync_state_mutex = xSemaphoreCreateMutex();

#if LIQUID_CRYSTAL_DISPLAY_ENABLED
    s_lcd_mutex = xSemaphoreCreateMutex();
#endif

#if CALCULATE_STACK_SIZES_ENABLED
    s_task_stack_usage_mutex = xSemaphoreCreateMutex();
#endif
}

void setup_the_LCD(void)
{

#if LIQUID_CRYSTAL_DISPLAY_ENABLED
    if (setup_lcd() == ESP_OK)
    {
        display_line(0, "ESP32 Time Server");
        display_line(1, "");
        display_line(2, "");
        display_line(3, "");
    }
    else
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "LCD setup failed");
#endif
    }
#endif
}

static void setup_up_the_RGB_LED()
{
#if RBG_LED_ENABLED
    gpio_config_t config{};
    config.pin_bit_mask = (1ULL << LEDBluePin) | (1ULL << LEDGreenPin) | (1ULL << LEDRedPin);
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&config));
    control_KY_016_RGB_LED(LED_startup, true);
#endif
}

static void setup_up_the_button()
{

#if UPTIME_RESTART_BUTTON_ENABLED
    gpio_config_t config{};
    config.pin_bit_mask = 1ULL << upTimeRestartPin;
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&config));
#endif
}

static std::string configure_mac_address()
{
    uint8_t real_mac_address[6];
    char real_mac_address_str[18] = {0};

    esp_err_t ret = esp_efuse_mac_get_default(real_mac_address);

    if (ret == ESP_OK)
    {
        std::snprintf(real_mac_address_str, sizeof(real_mac_address_str),
                      "%02x:%02x:%02x:%02x:%02x:%02x",
                      real_mac_address[0], real_mac_address[1], real_mac_address[2],
                      real_mac_address[3], real_mac_address[4], real_mac_address[5]);

#if DEBUG_ENABLED
        ESP_LOGI(TAG, "Real MAC address for this ESP32 is: %s", real_mac_address_str);
#endif
    }

#if DEBUG_ENABLED
    else
    {
        ESP_LOGE(TAG, "Failed to get the MAC address for this ESP32. Error: %d", ret);
    }
#endif

    // If MACAddress is empty or matches the real MAC, do nothing
    if ((MACAddress[0] == '\0') || (std::strcmp(real_mac_address_str, MACAddress) == 0))
    {
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "No need to change the MAC address.");
#endif
        return std::string(real_mac_address_str);
    }

    // Parse the desired MAC
    uint8_t mac[6] = {};
    if (!parse_mac_id_string(MACAddress, mac))
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "The MAC address in settings is invalid: %s - the MAC ID will not be changed", MACAddress);
#endif
        return std::string(real_mac_address_str);
    }

    // Apply the new MAC
    esp_err_t err = esp_base_mac_addr_set(mac);
    if (err != ESP_OK)
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Failed to set MAC address to %s: %s", MACAddress, esp_err_to_name(err));
#endif
        return std::string(real_mac_address_str);
    }

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "MAC address changed to: %s", MACAddress);
#endif

    return std::string(MACAddress);
}

void setup_ethernet_connection()
{

    std::string MACToBeUsed = configure_mac_address();
    ESP_LOGI(TAG, "The MAC address that will be used for this device is: %s", MACToBeUsed.c_str());

    display_line(1, "Connecting Ethernet");
    display_line(2, "");
    setup_ethernet();
    display_selected_ip_address(static_cast<int>(time(nullptr) % 10));
}

#if OTE_UPDATES_ENABLED

static void OTE_copy_reason(char *destination, size_t destination_size, const char *reason)
{
    if (destination_size == 0)
        return;

    if (reason == nullptr || reason[0] == '\0')
    {
        destination[0] = '\0';
        return;
    }

    snprintf(destination, destination_size, "%s", reason);
}

static void OTE_set_running_state(unsigned int progress, unsigned int total)
{
    if (xSemaphoreTake(s_ote_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_ote_in_progress = true;
        s_ote_failed = false;
        s_ote_success = false;
        s_ote_failure_display_until_us = 0;
        s_ote_reboot_at_us = 0;
        s_ote_error_reason[0] = '\0';
        s_ote_progress_percent = total == 0 ? 0 : static_cast<int>((progress * 100U) / total);
        xSemaphoreGive(s_ote_mutex);
    }
}

static void OTE_set_failure_state(const char *reason)
{
    if (xSemaphoreTake(s_ote_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_ote_in_progress = false;
        s_ote_failed = true;
        s_ote_success = false;
        s_ote_failure_display_until_us = esp_timer_get_time() + static_cast<int64_t>(OTE_Failure_Display_Time_Ms) * 1000LL;
        s_ote_reboot_at_us = 0;
        OTE_copy_reason(s_ote_error_reason, sizeof(s_ote_error_reason), reason);
        xSemaphoreGive(s_ote_mutex);
    }
}

static void OTE_set_success_state()
{
    if (xSemaphoreTake(s_ote_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_ote_in_progress = false;
        s_ote_failed = false;
        s_ote_success = true;
        s_ote_progress_percent = 100;
        s_ote_failure_display_until_us = 0;
        s_ote_reboot_at_us = esp_timer_get_time() + static_cast<int64_t>(OTE_Reboot_Delay_Ms) * 1000LL;
        s_ote_error_reason[0] = '\0';
        xSemaphoreGive(s_ote_mutex);
    }
}

static void format_ote_error_reason(ota_error_t error, char *buffer, size_t buffer_size)
{
    const char *update_error = Update.errorString();
    if ((error == OTA_BEGIN_ERROR || error == OTA_END_ERROR) && update_error != nullptr && strcmp(update_error, "No Error") != 0)
    {
        snprintf(buffer, buffer_size, "%s", update_error);
        return;
    }

    switch (error)
    {
    case OTA_AUTH_ERROR:
        snprintf(buffer, buffer_size, "%s", "Auth Failed");
        break;
    case OTA_BEGIN_ERROR:
        snprintf(buffer, buffer_size, "%s", "Begin Failed");
        break;
    case OTA_CONNECT_ERROR:
        snprintf(buffer, buffer_size, "%s", "Connect Failed");
        break;
    case OTA_RECEIVE_ERROR:
        snprintf(buffer, buffer_size, "%s", "Receive Failed");
        break;
    case OTA_END_ERROR:
        snprintf(buffer, buffer_size, "%s", "End Failed");
        break;
    default:
        snprintf(buffer, buffer_size, "%s", "Unknown Error");
        break;
    }
}

static bool render_ote_display()
{
    bool OTE_in_progress = false;
    bool OTE_failed = false;
    bool OTE_success = false;
    int OTE_progress_percent = -1;
    int64_t OTE_failure_display_until_us = 0;
    int64_t OTE_reboot_at_us = 0;
    char ote_error_reason[lcdColumns + 1] = "";

    if (xSemaphoreTake(s_ote_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    OTE_in_progress = s_ote_in_progress;
    OTE_failed = s_ote_failed;
    OTE_success = s_ote_success;
    OTE_progress_percent = s_ote_progress_percent;
    OTE_failure_display_until_us = s_ote_failure_display_until_us;
    OTE_reboot_at_us = s_ote_reboot_at_us;
    OTE_copy_reason(ote_error_reason, sizeof(ote_error_reason), s_ote_error_reason);
    xSemaphoreGive(s_ote_mutex);

    int64_t now_us = esp_timer_get_time();
    if (!OTE_in_progress && !OTE_failed && !OTE_success)
        return false;

    if (OTE_failed && OTE_failure_display_until_us > 0 && now_us >= OTE_failure_display_until_us)
    {
        if (xSemaphoreTake(s_ote_mutex, portMAX_DELAY) == pdTRUE)
        {
            s_ote_failed = false;
            s_ote_progress_percent = -1;
            s_ote_error_reason[0] = '\0';
            s_ote_failure_display_until_us = 0;
            xSemaphoreGive(s_ote_mutex);
        }
        return false;
    }

    if (OTE_in_progress)
    {
        char progress_line[lcdColumns + 1];
        snprintf(progress_line, sizeof(progress_line), "%d%% complete", OTE_progress_percent < 0 ? 0 : OTE_progress_percent);
        display_line(1, "OTE update started");
        display_line(2, progress_line);
        display_line(3, "Uploading firmware");
        return true;
    }

    if (OTE_success)
    {
        (void)OTE_reboot_at_us;
        display_line(1, "OTE successful");
        display_line(2, "100% complete");
        display_line(3, "Rebooting in 5 sec");
        return true;
    }

    display_line(1, "OTE failed");
    display_line(2, ote_error_reason[0] == '\0' ? "Unknown reason" : ote_error_reason);
    display_line(3, "Resuming in 10 sec");
    return true;
}

static void ote_service_task(void *parameter)
{
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "OTE service task started on port %u", static_cast<unsigned int>(OTEPort));
#endif

    // uint32_t last_heartbeat_ms = 0;   // uncomment this line and the block below if you want to see a heart beat message in the console log every 10 seconds

    for (;;)
    {
        ArduinoOTA.handle();

        // uncomment the following if you want to see a heart beat message in the console log every 10 seconds
        /*
        uint32_t now_ms = millis();
        if (last_heartbeat_ms == 0 || (now_ms - last_heartbeat_ms) >= 10000UL)
        {

            #if DEBUG_ENABLED
                ESP_LOGI(TAG,
                         "OTE heartbeat: online=%s, ip=%s, in_progress=%s",
                         Network.isOnline() ? "true" : "false",
                         s_ip_address[0] == '\0' ? "<none>" : s_ip_address,
                         s_ote_in_progress ? "true" : "false");

            last_heartbeat_ms = now_ms;
        }
        */

        bool should_reboot = false;
        TickType_t loop_delay_ticks = pdMS_TO_TICKS(50);
        if (xSemaphoreTake(s_ote_mutex, portMAX_DELAY) == pdTRUE)
        {
            bool OTE_idle = !s_ote_in_progress && !s_ote_failed && !s_ote_success;
            if (OTE_idle)
                loop_delay_ticks = pdMS_TO_TICKS(200);

            if (s_ote_success && s_ote_reboot_at_us > 0 && esp_timer_get_time() >= s_ote_reboot_at_us)
                should_reboot = true;
            xSemaphoreGive(s_ote_mutex);
        }

        vTaskDelay(loop_delay_ticks);
        if (should_reboot)
        {
#if MQTT_ENABLED
            mqtt_publish_final_report();
#endif
            esp_restart();
        }

#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(OTE_Service);
#endif
    }
}

static void setup_ota()
{
    if (xSemaphoreTake(s_ote_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_ote_in_progress = false;
        s_ote_failed = false;
        s_ote_success = false;
        s_ote_progress_percent = -1;
        s_ote_failure_display_until_us = 0;
        s_ote_reboot_at_us = 0;
        s_ote_error_reason[0] = '\0';
        xSemaphoreGive(s_ote_mutex);
    };

    bool network_begin_ok = Network.begin();

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "Arduino Network.begin()=%s, online=%s, current_ip=%s",
             network_begin_ok ? "true" : "false",
             Network.isOnline() ? "true" : "false",
             s_ip_address[0] == '\0' ? "<none>" : s_ip_address);

    ESP_LOGI(TAG,
             "Configuring ArduinoOTA: host=%s, port=%u, password_length=%u",
             DeviceName,
             static_cast<unsigned int>(OTEPort),
             static_cast<unsigned int>(strlen(OTEPassword)));
#endif

    ArduinoOTA.setPort(OTEPort);
    ArduinoOTA.setHostname(DeviceName);
    ArduinoOTA.setPassword(OTEPassword);
    ArduinoOTA.setRebootOnSuccess(false);
    ArduinoOTA.onStart([]()
                       {
                           OTE_set_running_state(0, 1);
#if DEBUG_ENABLED
                           ESP_LOGI(TAG, "OTE update started");
#endif
                       });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { OTE_set_running_state(progress, total); });
    ArduinoOTA.onEnd([]()
                     {
                         OTE_set_success_state();
#if DEBUG_ENABLED
                         ESP_LOGI(TAG, "OTE update completed successfully");
#endif
                     });
    ArduinoOTA.onError([](ota_error_t error)
                       {
                           char reason[lcdColumns + 1];
                           format_ote_error_reason(error, reason, sizeof(reason));
                           OTE_set_failure_state(reason);
#if DEBUG_ENABLED
                           ESP_LOGE(TAG, "OTE update failed: %s", reason);
#endif
                       });
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "Calling ArduinoOTA.begin()...");
#endif

    ArduinoOTA.begin();

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "ArduinoOTA.begin() returned, listener should be available on %s:%u",
             s_ip_address[0] == '\0' ? DeviceName : s_ip_address,
             static_cast<unsigned int>(OTEPort));
#endif
}

#endif

void setup_for_ote_updates()
{
#if OTE_UPDATES_ENABLED

    display_line(1, "Setup OTE");
    display_line(2, "");
    setup_ota();

    // note: this task is intentionally pinned to core 0 (as opposed to tskNO_AFFINITY)
    xTaskCreatePinnedToCore(ote_service_task, "ote_service", OTE_Service_Task_Stack_Size, nullptr, 5, nullptr, 0);

#endif
}

static bool acquire_sync_candidate(sync_candidate_t *candidate)
{
    if (candidate == nullptr)
        return false;

    *candidate = sync_candidate_t{};
    candidate->used_nmea_fallback = s_use_nmea_fallback;
    candidate->pps_release_time_us = esp_timer_get_time();
    int64_t attempt_start_us = candidate->pps_release_time_us;

    if (s_use_nmea_fallback)
    {
        nmea_rmc_time_t nmea_time{};
        if (!wait_for_nmea_rmc_time(&nmea_time, 3000UL))
        {
            candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid checkpoint 1.");
#endif
            return false;
        }

        candidate->candidate_time = epoch_from_utc(nmea_time.year, nmea_time.month, nmea_time.day, nmea_time.hour, nmea_time.minute, nmea_time.second);

        clear_pps_events();
        PpsCaptureEvent capture_event{};
        if (wait_for_pps_capture_event(&capture_event, pdMS_TO_TICKS(1500)))
        {
            candidate->use_pps_alignment = true;
            candidate->pps_release_time_us = capture_event.approximate_edge_us;
            candidate->candidate_time += 1;
        }
        else
        {

#if DEBUG_ENABLED
            ESP_LOGE(TAG, "NMEA fallback is running without PPS.");
#endif

            candidate->failures.pps_missing = true;
            return false;
        }
    }
    else
    {
        clear_pps_events();
        PpsCaptureEvent capture_event{};
        if (!wait_for_pps_capture_event(&capture_event, pdMS_TO_TICKS(1500)))
        {
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "UBX mode is running without PPS.");
#endif

            candidate->failures.pps_missing = true;
            return false;
        }

        if (!s_gnss.getPVT())
        {
            candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - Position - Velocity - Time");
#endif
            return false;
        }

        uint8_t fix_type = s_gnss.getFixType();
        if ((fix_type != 3 && fix_type != 4 && fix_type != 5) || !s_gnss.getGnssFixOk() || !s_gnss.getDateValid() || !s_gnss.getTimeValid())
        {
            candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - Fix type.");
#endif
            return false;
        }

        int year = s_gnss.getYear();
        int month = s_gnss.getMonth();
        int day = s_gnss.getDay();
        int hour = s_gnss.getHour();
        int minute = s_gnss.getMinute();
        int second = s_gnss.getSecond();

        if (year <= 2025 || month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60)
        {
            candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - bad date or time.");
#endif
            return false;
        }

        candidate->candidate_time = epoch_from_utc(year, month, day, hour, minute, second) + 1;

        vTaskDelay(pdMS_TO_TICKS(200));

        clear_pps_events();
        if (!wait_for_pps_capture_event(&capture_event, pdMS_TO_TICKS(1500)))
        {
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "UBX mode lost PPS alignment pulse.");
#endif

            candidate->failures.pps_missing = true;
            return false;
        }

        candidate->pps_release_time_us = capture_event.approximate_edge_us;
        candidate->use_pps_alignment = true;
    }

    if ((esp_timer_get_time() - attempt_start_us) > Max_Sync_Attempt_Us)
    {
        candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "GNSS invalid checkpoint 2.");
#endif
        return false;
    }

    candidate->used_nmea_fallback = s_use_nmea_fallback;
    return true;
}

static void gnss_runtime_recovery_task(void *parameter)
{
    TaskHandle_t sync_task_handle = reinterpret_cast<TaskHandle_t>(parameter);

    setup_gnss();

#if CALCULATE_STACK_SIZES_ENABLED
    report_current_task_stack_usage(GNSS_Recovery);
#endif

    s_gnss_recovery_in_progress.store(false);
    xTaskNotifyGive(sync_task_handle);
    vTaskDelete(nullptr);
}

static void handle_runtime_sync_failure(const sync_faults_t &faults, time_t update_delta, uint32_t retry_delay_ms)
{
    uint32_t failure_count = sync_state_note_failure(faults, update_delta);
    sync_state_t snapshot = get_sync_state_snapshot();

    s_time_setting_in_progress.store(false);

    if (failure_count >= Sync_Failures_Before_Runtime_Recovery && (failure_count % Sync_Failures_Before_Runtime_Recovery) == 0)
    {
        int64_t now_us = esp_timer_get_time();
        int64_t last_recovery_us = s_last_gnss_recovery_us.load();
        bool recovery_due = last_recovery_us == 0 || (now_us - last_recovery_us) >= Runtime_gnss_Recovery_Min_Interval_Us;

        if (recovery_due && !s_gnss_recovery_in_progress.exchange(true))
        {
#if DEBUG_ENABLED
            ESP_LOGW(TAG, "Runtime GNSS recovery attempt after %lu consecutive sync failures.", static_cast<unsigned long>(failure_count));
#endif
            s_last_gnss_recovery_us.store(now_us);
            TaskHandle_t sync_task_handle = xTaskGetCurrentTaskHandle();
            BaseType_t created = xTaskCreatePinnedToCore(gnss_runtime_recovery_task,
                                                         "gnss_recovery",
                                                         GNSS_Recovery_Task_Stack_Size,
                                                         sync_task_handle,
                                                         15,
                                                         nullptr,
                                                         tskNO_AFFINITY);
            if (created == pdPASS)
            {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                sync_state_reset_failure_counters();
            }
            else
            {
                s_gnss_recovery_in_progress.store(false);
#if DEBUG_ENABLED
                ESP_LOGE(TAG, "Unable to create the GNSS recovery task.");
#endif
            }
        }
    }

    if (snapshot.last_successful_sync_us > 0 && (esp_timer_get_time() - snapshot.last_successful_sync_us) > Sync_Reboot_After_Us)
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Rebooting after extended holdover without a successful GNSS resync.");
#endif
        vTaskDelay(pdMS_TO_TICKS(200));
#if MQTT_ENABLED
        mqtt_publish_final_report();
#endif
        esp_restart();
    }

    vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
}

static void gnss_time_sync_task(void *parameter)
{
    bool first_sync = true;

    for (;;)
    {

        s_time_setting_in_progress.store(true);
        sync_state_note_attempt();

        /*  pre-release code - commented out
        sync_candidate_t candidate{};

        // Start stopwatch
        int64_t start_us = esp_timer_get_time();

        bool ok = acquire_sync_candidate(&candidate);

        // Stop stopwatch
        int64_t end_us = esp_timer_get_time();

        // Calculate time to sync
        int64_t elapsed_us = end_us - start_us;

        if (!ok)
        {
            // Display elapsed time even on failure
            ESP_LOGE(TAG, "acquire_sync_candidate() failed after %lld us\n", elapsed_us);

            handle_runtime_sync_failure(candidate.failures, 0, 1000);
            continue;
        }

        // Success path
        ESP_LOGI(TAG, "acquire_sync_candidate() succeeded in %lld us\n", elapsed_us);

        // end of performance test code *************************

        end of pre-release code */

        // start of code block to be replaced with the above when released

        sync_candidate_t candidate{};
        if (!acquire_sync_candidate(&candidate))
        {
            handle_runtime_sync_failure(candidate.failures, 0, 1000);
            continue;
        }

        // end of code block to be replaced with the above when released

        if (first_sync)
        {
            sync_candidate_t confirmation_candidate{};
            if (!acquire_sync_candidate(&confirmation_candidate))
            {
                handle_runtime_sync_failure(confirmation_candidate.failures, 0, 1000);
                continue;
            }

            if (!first_sync_candidates_are_plausible(candidate, confirmation_candidate))
            {
                handle_runtime_sync_failure({false, true, false, false}, 0, 1000);
                continue;
            }

            candidate = confirmation_candidate;
        }

        time_t update_delta = 0;
        if (!first_sync)
        {
            time_t current_time = time(nullptr);
            update_delta = current_time - candidate.candidate_time;
            bool sanity_check_passed = (update_delta >= -safeguardThresholdInSeconds) && (update_delta <= safeguardThresholdInSeconds);
            if (!sanity_check_passed)
            {
                uint32_t sanity_failure_count = sync_state_note_sanity_retry(update_delta);
                s_time_setting_in_progress.store(false);

#if DEBUG_ENABLED
                ESP_LOGE(TAG, "Sanity check failed with delta %lld on attempt %lu.", static_cast<long long>(update_delta), static_cast<unsigned long>(sanity_failure_count));
#endif

                if (sanity_failure_count < Sanity_Failures_Before_Fault)
                {
                    vTaskDelay(pdMS_TO_TICKS(250));
                    continue;
                }

                s_safe_guard_tripped.store(true);
                sync_state_note_failure({false, false, true, false}, update_delta);

                if (rebootIfSanityCheckFails)
                {
#if DEBUG_ENABLED
                    ESP_LOGE(TAG, "Restarting according to settings.");
#endif
                    vTaskDelay(pdMS_TO_TICKS(200));
#if MQTT_ENABLED
                    mqtt_publish_final_report();
#endif
                    esp_restart();
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        sync_state_clear_sanity_failures();

        if (xSemaphoreTake(s_time_mutex, portMAX_DELAY) == pdTRUE)
        {
            int64_t elapsed_us = candidate.use_pps_alignment ? (esp_timer_get_time() - candidate.pps_release_time_us) : 0;
            if (elapsed_us < 0)
                elapsed_us = 0;

            struct timeval tv{};
            tv.tv_sec = candidate.candidate_time + static_cast<time_t>(elapsed_us / 1000000LL);
            tv.tv_usec = static_cast<suseconds_t>(elapsed_us % 1000000LL);
            settimeofday(&tv, nullptr);
            synchronize_hardware_clock();
            s_ntp_reference_time_64 = get_current_time_in_ntp64_format();
            s_ntp_reference_valid = true;

            xSemaphoreGive(s_time_mutex);

            s_safe_guard_tripped.store(false);
            s_time_setting_in_progress.store(false);
            s_time_has_been_set.store(true);
            first_sync = false;
            sync_state_note_success(update_delta);

#if DEBUG_ENABLED
            char date_string[16] = "";
            char time_string[24] = "";
            time_t now_utc = time(nullptr);
            format_local_date_time(now_utc, date_string, sizeof(date_string), time_string, sizeof(time_string));

            if (candidate.used_nmea_fallback)
                ESP_LOGI(TAG, "GNSS time sync ( using NMEA fallback %s ) on %s at %s", candidate.use_pps_alignment ? "with PPS alignment" : "without PPS alignment", date_string, time_string);
            else
                ESP_LOGI(TAG, "GNSS time sync on %s at %s", date_string, time_string);
#endif

            uint32_t refresh_start_ms = millis();
            uint32_t refresh_interval_ms = periodicGNSSRefreshEveryThisNumberOfMinutes * 60UL * 1000UL;
            int64_t invalid_started_us = 0;

            while ((millis() - refresh_start_ms) < refresh_interval_ms)
            {
                bool gnss_valid = current_gnss_timing_is_valid();

                if (gnss_valid)
                {
                    invalid_started_us = 0;
                    sync_state_note_gnss_validity(true);
                }
                else
                {
                    if (invalid_started_us == 0)
                        invalid_started_us = esp_timer_get_time();

                    if ((esp_timer_get_time() - invalid_started_us) >= Gnss_Invalid_Reacquisition_After_Us)
                    {
                        sync_state_note_gnss_validity(false);

#if DEBUG_ENABLED
                        ESP_LOGW(TAG,
                                 "GNSS timing invalid for more than %lu seconds; starting reacquisition.",
                                 static_cast<unsigned long>(Gnss_Invalid_Reacquisition_After_Us / 1000000LL));
#endif

                        break;
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(GNSS_Time_Sync);
#endif
    }
}

void setup_the_gnss()
{

    display_line(1, "GNSS setup underway");
    display_line(2, "");

    apply_timezone_settings();

    setup_gnss();

    display_line(1, "Getting date & time");
    display_line(2, "");

    xTaskCreatePinnedToCore(gnss_time_sync_task, "gnss_time_sync", GNSS_Time_Sync_Task_Stack_Size, nullptr, 15, nullptr, tskNO_AFFINITY);

    xTaskCreatePinnedToCore(pps_discipline_task, "pps_discipline", PPS_Discipline_Task_Stack_Size, nullptr, 14, nullptr, tskNO_AFFINITY);

    while (!s_time_has_been_set.load())
        vTaskDelay(pdMS_TO_TICKS(100));

    display_line(1, "Time Synchronized");
    display_line(2, "PPS Disciplined");
}

// Serves standard IPv6 NTP client requests and returns the device’s current synchronized time.
// Also servers IPv4 NTP client requests that the IPv4 NTP-related traffic the hardware interception path rejects
static void ntp_server_task(void *parameter)
{
    TaskHandle_t startup_task = static_cast<TaskHandle_t>(parameter);
    int ipv4_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int ipv6_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    int ipv6_link_local_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (ipv4_socket < 0 || ipv6_socket < 0)
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Unable to create NTP UDP sockets: errno %d", errno);
#endif
        if (ipv4_socket >= 0)
            closesocket(ipv4_socket);
        if (ipv6_socket >= 0)
            closesocket(ipv6_socket);
        if (ipv6_link_local_socket >= 0)
            closesocket(ipv6_link_local_socket);
        if (startup_task != nullptr)
            xTaskNotifyGive(startup_task);
        vTaskDelete(nullptr);
        return;
    }

    struct sockaddr_in ipv4_listen_addr{};
    ipv4_listen_addr.sin_family = AF_INET;
    ipv4_listen_addr.sin_port = htons(NTP_PORT);
    ipv4_listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int ipv6_only = 1;
    int reuse_address = 1;
    if (setsockopt(ipv6_socket, IPPROTO_IPV6, IPV6_V6ONLY, &ipv6_only, sizeof(ipv6_only)) != 0 ||
        setsockopt(ipv6_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address)) != 0 ||
        bind(ipv4_socket, reinterpret_cast<struct sockaddr *>(&ipv4_listen_addr), sizeof(ipv4_listen_addr)) != 0)
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Unable to configure NTP UDP sockets: errno %d", errno);
#endif
        closesocket(ipv4_socket);
        closesocket(ipv6_socket);
        if (ipv6_link_local_socket >= 0)
            closesocket(ipv6_link_local_socket);
        if (startup_task != nullptr)
            xTaskNotifyGive(startup_task);
        vTaskDelete(nullptr);
        return;
    }

    struct sockaddr_in6 ipv6_listen_addr{};
    ipv6_listen_addr.sin6_family = AF_INET6;
    ipv6_listen_addr.sin6_port = htons(NTP_PORT);
    ipv6_listen_addr.sin6_addr = in6addr_any;
    if (bind(ipv6_socket, reinterpret_cast<struct sockaddr *>(&ipv6_listen_addr), sizeof(ipv6_listen_addr)) != 0)
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Unable to bind IPv6 NTP UDP socket: errno %d", errno);
#endif
        closesocket(ipv4_socket);
        closesocket(ipv6_socket);
        if (ipv6_link_local_socket >= 0)
            closesocket(ipv6_link_local_socket);
        if (startup_task != nullptr)
            xTaskNotifyGive(startup_task);
        vTaskDelete(nullptr);
        return;
    }

    esp_ip6_addr_t link_local_address{};
    if (ipv6_link_local_socket < 0 ||
        setsockopt(ipv6_link_local_socket, IPPROTO_IPV6, IPV6_V6ONLY, &ipv6_only, sizeof(ipv6_only)) != 0 ||
        setsockopt(ipv6_link_local_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address)) != 0 ||
        esp_netif_get_ip6_linklocal(ETH.netif(), &link_local_address) != ESP_OK)
    {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "IPv6 link-local NTP listener unavailable: errno %d", errno);
#endif
        if (ipv6_link_local_socket >= 0)
            closesocket(ipv6_link_local_socket);
        ipv6_link_local_socket = -1;
    }
    else
    {
        struct sockaddr_in6 ipv6_link_local_listen_addr{};
        ipv6_link_local_listen_addr.sin6_family = AF_INET6;
        ipv6_link_local_listen_addr.sin6_port = htons(NTP_PORT);
        memcpy(&ipv6_link_local_listen_addr.sin6_addr, link_local_address.addr, sizeof(ipv6_link_local_listen_addr.sin6_addr));
        ipv6_link_local_listen_addr.sin6_scope_id = esp_netif_get_netif_impl_index(ETH.netif());
        if (bind(ipv6_link_local_socket,
                 reinterpret_cast<struct sockaddr *>(&ipv6_link_local_listen_addr),
                 sizeof(ipv6_link_local_listen_addr)) != 0)
        {
#if DEBUG_ENABLED
            ESP_LOGW(TAG, "IPv6 link-local NTP listener unavailable: errno %d", errno);
#endif
            closesocket(ipv6_link_local_socket);
            ipv6_link_local_socket = -1;
        }
    }

    s_ntp_server_ready.store(true, std::memory_order_release);
    if (startup_task != nullptr)
        xTaskNotifyGive(startup_task);

    for (;;)
    {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(ipv4_socket, &read_fds);
        FD_SET(ipv6_socket, &read_fds);
        if (ipv6_link_local_socket >= 0)
            FD_SET(ipv6_link_local_socket, &read_fds);
        struct timeval timeout{};
        timeout.tv_sec = 1;
        int max_socket = ipv4_socket;
        if (ipv6_socket > max_socket)
            max_socket = ipv6_socket;
        if (ipv6_link_local_socket >= 0 && ipv6_link_local_socket > max_socket)
            max_socket = ipv6_link_local_socket;
        int ready = select(max_socket + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready < 0)
        {
#if DEBUG_ENABLED
            ESP_LOGW(TAG, "NTP socket select failed: errno %d", errno);
#endif
#if CALCULATE_STACK_SIZES_ENABLED
            report_current_task_stack_usage(NTP_Server);
#endif
            continue;
        }
        if (ready == 0)
            continue;

        static uint8_t next_socket = 0;
        const int sockets[] = {ipv4_socket, ipv6_socket, ipv6_link_local_socket};
        const size_t socket_count = sizeof(sockets) / sizeof(sockets[0]);
        uint8_t start_socket = next_socket;

        for (size_t offset = 0; offset < socket_count; ++offset)
        {
            size_t index = (start_socket + offset) % socket_count;
            int sock = sockets[index];
            if (sock < 0 || !FD_ISSET(sock, &read_fds))
            {
#if CALCULATE_STACK_SIZES_ENABLED
                report_current_task_stack_usage(NTP_Server);
#endif
                continue;
            }

            next_socket = static_cast<uint8_t>((index + 1) % socket_count);
            for (size_t batch_count = 0; batch_count < NTP_SOCKET_BATCH_LIMIT; ++batch_count)
            {
                uint8_t request[NTP_PACKET_SIZE + 1];
                uint8_t reply[NTP_PACKET_SIZE];
                struct sockaddr_storage source_addr{};
                socklen_t source_addr_len = sizeof(source_addr);
                int len = recvfrom(sock, request, sizeof(request), MSG_DONTWAIT,
                                   reinterpret_cast<struct sockaddr *>(&source_addr), &source_addr_len);
                if (len < 0)
                {
                    if (errno == EAGAIN || errno == EWOULDBLOCK)
                    {
#if CALCULATE_STACK_SIZES_ENABLED
                        report_current_task_stack_usage(NTP_Server);
#endif
                        break;
                    }
#if DEBUG_ENABLED
                    ESP_LOGW(TAG, "recvfrom failed: errno %d", errno);
#endif
#if CALCULATE_STACK_SIZES_ENABLED
                    report_current_task_stack_usage(NTP_Server);
#endif
                    break;
                }

                uint64_t receive_time = get_current_time_in_ntp64_format();
                if (len != static_cast<int>(NTP_PACKET_SIZE))
                {
#if MQTT_ENABLED
                    s_ntp_invalid_requests.fetch_add(1, std::memory_order_relaxed);
#endif
#if CALCULATE_STACK_SIZES_ENABLED
                    report_current_task_stack_usage(NTP_Server);
#endif
                    continue;
                }

                uint8_t ntp_version = (request[0] >> 3) & 0x07;
                uint8_t ntp_mode = request[0] & 0x07;
                if (ntp_version < 3 || ntp_version > 4 || ntp_mode != 3)
                {
#if MQTT_ENABLED
                    s_ntp_invalid_requests.fetch_add(1, std::memory_order_relaxed);
#endif
#if CALCULATE_STACK_SIZES_ENABLED
                    report_current_task_stack_usage(NTP_Server);
#endif
                    continue;
                }

                if (!s_ntp_external_responses_enabled.load(std::memory_order_acquire) && !is_internal_ntp_client(source_addr))
                    continue;

#if MQTT_ENABLED
                s_ntp_requests_this_second.fetch_add(1, std::memory_order_relaxed);
                s_ntp_valid_requests.fetch_add(1, std::memory_order_relaxed);
                mqtt_enqueue_ntp_request(source_addr);
#endif
                const struct sockaddr_in *source_ipv4 = nullptr;
                const struct in6_addr *client_ipv6 = nullptr;
                ntp_client_record_t client_ipv4_record{};
                ntp_client_record_ipv6_t client_ipv6_record{};
                if (source_addr.ss_family == AF_INET)
                {
                    source_ipv4 = reinterpret_cast<const struct sockaddr_in *>(&source_addr);
                    ntp_cache_find_or_create(ntohl(source_ipv4->sin_addr.s_addr), ntohs(source_ipv4->sin_port), &client_ipv4_record);
                }
                else if (source_addr.ss_family == AF_INET6)
                {
                    client_ipv6 = &reinterpret_cast<const struct sockaddr_in6 *>(&source_addr)->sin6_addr;
                    ntp_cache_find_or_create_ipv6(client_ipv6,
                                                  ntohs(reinterpret_cast<const struct sockaddr_in6 *>(&source_addr)->sin6_port),
                                                  &client_ipv6_record);
                }

                ntp_reply_status_t status = get_ntp_reply_status();
#if MQTT_ENABLED
                if (status.gnss_synchronized && status.pps_disciplined)
                    s_ntp_responses_synchronized_and_disciplined.fetch_add(1, std::memory_order_relaxed);
                if (!status.gnss_synchronized)
                    s_ntp_responses_gnss_unsynchronized.fetch_add(1, std::memory_order_relaxed);
                if (!status.pps_disciplined)
                    s_ntp_responses_pps_undisciplined.fetch_add(1, std::memory_order_relaxed);
#endif
                build_ntp_reply(request, reply, ntp_version, receive_time, status);
                const uint64_t client_receive_timestamp =
                    (static_cast<uint64_t>(request[32]) << 56) | (static_cast<uint64_t>(request[33]) << 48) |
                    (static_cast<uint64_t>(request[34]) << 40) | (static_cast<uint64_t>(request[35]) << 32) |
                    (static_cast<uint64_t>(request[36]) << 24) | (static_cast<uint64_t>(request[37]) << 16) |
                    (static_cast<uint64_t>(request[38]) << 8) | request[39];
                const uint64_t client_origin_timestamp =
                    (static_cast<uint64_t>(request[24]) << 56) | (static_cast<uint64_t>(request[25]) << 48) |
                    (static_cast<uint64_t>(request[26]) << 40) | (static_cast<uint64_t>(request[27]) << 32) |
                    (static_cast<uint64_t>(request[28]) << 24) | (static_cast<uint64_t>(request[29]) << 16) |
                    (static_cast<uint64_t>(request[30]) << 8) | request[31];
                const uint64_t client_transmit_timestamp =
                    (static_cast<uint64_t>(request[40]) << 56) | (static_cast<uint64_t>(request[41]) << 48) |
                    (static_cast<uint64_t>(request[42]) << 40) | (static_cast<uint64_t>(request[43]) << 32) |
                    (static_cast<uint64_t>(request[44]) << 24) | (static_cast<uint64_t>(request[45]) << 16) |
                    (static_cast<uint64_t>(request[46]) << 8) | request[47];
                const uint64_t previous_ipv4_t2 = client_ipv4_record.prev_t2;
                const uint64_t previous_ipv4_t3 = client_ipv4_record.prev_t3;
                const uint64_t ipv4_t2_difference = client_origin_timestamp >= previous_ipv4_t2
                                                        ? client_origin_timestamp - previous_ipv4_t2
                                                        : previous_ipv4_t2 - client_origin_timestamp;
                const bool ipv4_interleaved_reply = source_ipv4 != nullptr && previous_ipv4_t2 != 0 &&
                                                    client_receive_timestamp != client_transmit_timestamp &&
                                                    ipv4_t2_difference <= NTP_CACHE_TIMESTAMP_MATCH_TOLERANCE;
                const bool ipv6_interleaved_reply = client_ipv6 != nullptr && client_ipv6_record.prev_t2 != 0 &&
                                                    client_receive_timestamp != client_transmit_timestamp &&
                                                    client_origin_timestamp == client_ipv6_record.prev_t2;
                const bool interleaved_reply = ipv4_interleaved_reply || ipv6_interleaved_reply;
                const uint64_t transmit_time = get_current_time_in_ntp64_format();
                if (interleaved_reply)
                {
                    write_ntp_timestamp(reply, 24, client_receive_timestamp);
                    write_ntp_timestamp(reply, 40, ipv4_interleaved_reply ? previous_ipv4_t3 : client_ipv6_record.prev_t3);
                }
                else
                {
                    write_ntp_timestamp(reply, 40, transmit_time);
                }

                int sent = sendto(sock, reply, sizeof(reply), 0, reinterpret_cast<struct sockaddr *>(&source_addr), source_addr_len);
                if (sent == static_cast<int>(sizeof(reply)))
                {
                    if (source_ipv4 != nullptr)
                    {
                        ntp_cache_update(ntohl(source_ipv4->sin_addr.s_addr), ntohs(source_ipv4->sin_port),
                                         receive_time, read_ntp_timestamp(reply, 40));
                    }
                    else if (client_ipv6 != nullptr)
                    {
                        ntp_cache_update_ipv6(client_ipv6,
                                              ntohs(reinterpret_cast<const struct sockaddr_in6 *>(&source_addr)->sin6_port),
                                              receive_time, read_ntp_timestamp(reply, 40));
                    }
                }
#if MQTT_ENABLED
                if (sent == static_cast<int>(sizeof(reply)))
                    s_ntp_responses.fetch_add(1, std::memory_order_relaxed);
#else
                (void)sent;
#endif
#if DEBUG_ENABLED
                char source_address[IP_ADDRESS_TEXT_SIZE] = "";
                if (format_socket_address(source_addr, source_address, sizeof(source_address)))
                    ESP_LOGI(TAG, "NTP -> %s", source_address);
#endif
#if CALCULATE_STACK_SIZES_ENABLED
                report_current_task_stack_usage(NTP_Server);
#endif
            }
#if CALCULATE_STACK_SIZES_ENABLED
            report_current_task_stack_usage(NTP_Server);
#endif
        }
#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(NTP_Server);
#endif
    }
#if CALCULATE_STACK_SIZES_ENABLED
    report_current_task_stack_usage(NTP_Server);
#endif
}

#if STARTUP_HEALTH_TEST_ENABLED
static constexpr uint32_t Startup_Health_Test_Ready_Timeout_Ms = 30000;
static constexpr uint32_t Startup_Health_Test_Response_Timeout_Ms = 1000;
static constexpr uint32_t Startup_Health_Test_Standard_Retries = 3;
static constexpr uint64_t Startup_Health_Test_interleaved_Tolerance = (1ULL << 32) * 5ULL / 1000ULL;
static constexpr size_t Startup_Health_Test_Task_Stack_Size = 4096;

struct startup_health_endpoint_results_t
{
    bool ntpv3_standard = false;
    bool ntpv4_standard = false;
    bool ntpv4_interleaved = false;
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
static uint32_t s_startup_health_test_number = 0;

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

static bool receive_startup_health_response(int socket_fd, uint8_t *reply, uint64_t *arrival_time)
{
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);
    struct timeval timeout{};
    timeout.tv_sec = Startup_Health_Test_Response_Timeout_Ms / 1000;
    timeout.tv_usec = static_cast<suseconds_t>(Startup_Health_Test_Response_Timeout_Ms % 1000) * 1000;
    if (select(socket_fd + 1, &read_fds, nullptr, nullptr, &timeout) != 1)
        return false;

    int reply_length = recvfrom(socket_fd, reply, NTP_PACKET_SIZE, 0, nullptr, nullptr);
    if (reply_length != static_cast<int>(NTP_PACKET_SIZE))
        return false;

    *arrival_time = get_current_time_in_ntp64_format();
    return true;
}

static bool run_standard_startup_health_test(int socket_fd, const struct sockaddr *destination, socklen_t destination_length,
                                             uint8_t version, startup_health_exchange_t *exchange)
{
    uint8_t request[NTP_PACKET_SIZE] = {};
    uint8_t reply[NTP_PACKET_SIZE] = {};
    exchange->t1 = get_current_time_in_ntp64_format();
    request[0] = static_cast<uint8_t>(version << 3) | 3;
    write_ntp_timestamp(request, 40, exchange->t1);

    for (uint32_t attempt = 0; attempt < Startup_Health_Test_Standard_Retries; ++attempt)
    {
        if (sendto(socket_fd, request, sizeof(request), 0, destination, destination_length) != static_cast<int>(sizeof(request)))
            continue;

        if (!receive_startup_health_response(socket_fd, reply, &exchange->t4))
            continue;

        if (((reply[0] >> 3) & 0x07) != version || (reply[0] & 0x07) != 4 || read_ntp_timestamp(reply, 24) != exchange->t1)
            continue;

        exchange->t2 = read_ntp_timestamp(reply, 32);
        exchange->t3 = read_ntp_timestamp(reply, 40);
        return exchange->t2 != 0 && exchange->t3 != 0;
    }

    return false;
}

static bool run_interleaved_startup_health_test(int socket_fd, const struct sockaddr *destination, socklen_t destination_length,
                                                uint8_t version, const startup_health_exchange_t &previous_exchange)
{
    uint8_t request[NTP_PACKET_SIZE] = {};
    uint8_t reply[NTP_PACKET_SIZE] = {};
    uint64_t arrival_time = 0;
    request[0] = static_cast<uint8_t>(version << 3) | 3;
    write_ntp_timestamp(request, 24, previous_exchange.t2);
    write_ntp_timestamp(request, 32, previous_exchange.t4);
    write_ntp_timestamp(request, 40, previous_exchange.t1);

    if (sendto(socket_fd, request, sizeof(request), 0, destination, destination_length) != static_cast<int>(sizeof(request)) ||
        !receive_startup_health_response(socket_fd, reply, &arrival_time))
        return false;

    const uint64_t reply_transmit_time = read_ntp_timestamp(reply, 40);
    const uint64_t transmit_difference = reply_transmit_time >= previous_exchange.t3
                                             ? reply_transmit_time - previous_exchange.t3
                                             : previous_exchange.t3 - reply_transmit_time;
    return ((reply[0] >> 3) & 0x07) == version && (reply[0] & 0x07) == 4 &&
           read_ntp_timestamp(reply, 24) == previous_exchange.t4 &&
           transmit_difference <= Startup_Health_Test_interleaved_Tolerance;
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
        return results;
    }

    startup_health_exchange_t ntpv3_exchange{};
    startup_health_exchange_t ntpv4_exchange{};
    results.ntpv3_standard = run_standard_startup_health_test(socket_fd, destination, destination_length, 3, &ntpv3_exchange);
    results.ntpv4_standard = run_standard_startup_health_test(socket_fd, destination, destination_length, 4, &ntpv4_exchange);
    if (results.ntpv4_standard)
        results.ntpv4_interleaved = run_interleaved_startup_health_test(socket_fd, destination, destination_length, 4, ntpv4_exchange);

    closesocket(socket_fd);
    queue_startup_health_test_result(name, "NTPv3 standard   ", results.ntpv3_standard);
    queue_startup_health_test_result(name, "NTPv4 standard   ", results.ntpv4_standard);
    queue_startup_health_test_result(name, "NTPv4 interleaved", results.ntpv4_interleaved);
    return results;
}

static startup_health_test_results_t run_startup_health_tests()
{
    startup_health_test_results_t results{};
    const EventBits_t ready_bits = ETH_CONNECTED_BIT | ETH_GOT_IP_BIT | ETH_GOT_IP6_BIT;
    const EventBits_t ready = xEventGroupWaitBits(s_net_event_group, ready_bits, pdFALSE, pdTRUE,
                                                  pdMS_TO_TICKS(Startup_Health_Test_Ready_Timeout_Ms));
    if ((ready & ready_bits) != ready_bits || !s_time_has_been_set.load(std::memory_order_acquire) ||
        !s_pps_discipline_active.load(std::memory_order_acquire))
    {
        queue_startup_health_log(ESP_LOG_ERROR, "Health Check prerequisites were not ready before timeout");
        return results;
    }

    results.prerequisites_ready = true;
    struct sockaddr_in ipv4_loopback{};
    ipv4_loopback.sin_family = AF_INET;
    ipv4_loopback.sin_port = htons(NTP_PORT);
    ipv4_loopback.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    results.ipv4_loopback = run_startup_health_endpoint_test("IPv4 loopback", reinterpret_cast<const struct sockaddr *>(&ipv4_loopback), sizeof(ipv4_loopback), AF_INET);

    struct sockaddr_in ipv4_assigned{};
    ipv4_assigned.sin_family = AF_INET;
    ipv4_assigned.sin_port = htons(NTP_PORT);
    if (inet_pton(AF_INET, s_ipv4_address, &ipv4_assigned.sin_addr) == 1)
        results.ipv4_assigned = run_startup_health_endpoint_test("IPv4 assigned", reinterpret_cast<const struct sockaddr *>(&ipv4_assigned), sizeof(ipv4_assigned), AF_INET);
    else
        queue_startup_health_log(ESP_LOG_ERROR, "Health Check IPv4 assigned: invalid address");

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
    ipv6_assigned.sin6_scope_id = esp_netif_get_netif_impl_index(ETH.netif());
    if (inet_pton(AF_INET6, s_ipv6_address, &ipv6_assigned.sin6_addr) == 1)
        results.ipv6_assigned = run_startup_health_endpoint_test("IPv6 assigned", reinterpret_cast<const struct sockaddr *>(&ipv6_assigned), sizeof(ipv6_assigned), AF_INET6);
    else
        queue_startup_health_log(ESP_LOG_ERROR, "Health Check IPv6 assigned: invalid address");

    return results;
}

static void startup_health_test_task(void *parameter)
{
    s_startup_health_test_log = static_cast<startup_health_log_entry_t *>(calloc(Startup_Health_Test_Log_Capacity, sizeof(startup_health_log_entry_t)));
    if (s_startup_health_test_log == nullptr)
    {
        ESP_LOGE(TAG, "Health Check could not allocate its log buffer");
        xTaskNotifyGive(static_cast<TaskHandle_t>(parameter));
        vTaskDelete(nullptr);
        return;
    }

    queue_startup_health_log(ESP_LOG_INFO, "Health Check started");
    s_startup_health_test_results = run_startup_health_tests();
    queue_startup_health_log(ESP_LOG_INFO, "Health Check %s", s_startup_health_test_results.prerequisites_ready ? "completed" : "failed before testing");
    flush_startup_health_log();
    free(s_startup_health_test_log);
    s_startup_health_test_log = nullptr;
    xTaskNotifyGive(static_cast<TaskHandle_t>(parameter));
    vTaskDelete(nullptr);
}
#endif

static int get_required_top_line_message()
{
    sync_state_t sync_snapshot = get_sync_state_snapshot();
    int required_top_line_message = 0;

    if (s_saved_gnss_baud_communication_failed.load())
        return 9;
    if (s_time_setting_in_progress.load())
        return 99;

    if (!s_ethernet_connected.load())
        required_top_line_message = 1;
#if MQTT_ENABLED
    else if (s_mqtt_queued_messages_count.load() > 0)
        required_top_line_message = 3;
    else if (s_mqtt_setup_failed.load() || !s_mqtt_connected.load())
        required_top_line_message = 2;
#endif

    if (sync_snapshot.faults.sanity_mismatch)
        return 4;
    if (sync_snapshot.faults.pps_missing)
        return 5;
    if (sync_snapshot.faults.gnss_invalid)
        return 6;
    if (sync_snapshot.faults.sync_stale)
        return 7;
    if (required_top_line_message == 0 && !s_gnss_locked.load())
        return 8;

    return required_top_line_message;
}

#if RBG_LED_ENABLED
static void control_KY_016_RGB_LED(RGB_LED_Color color, bool enabled)
{
    const bool illuminate = enabled && color != RGB_LED_Color::off;
    const bool red = illuminate && (color == RGB_LED_Color::red || color == RGB_LED_Color::yellow || color == RGB_LED_Color::white);
    const bool green = illuminate && (color == RGB_LED_Color::green || color == RGB_LED_Color::yellow || color == RGB_LED_Color::white);
    const bool blue = illuminate && (color == RGB_LED_Color::blue || color == RGB_LED_Color::white);

    gpio_set_level(static_cast<gpio_num_t>(LEDRedPin), red ? 1 : 0);
    gpio_set_level(static_cast<gpio_num_t>(LEDGreenPin), green ? 1 : 0);
    gpio_set_level(static_cast<gpio_num_t>(LEDBluePin), blue ? 1 : 0);
}

static void update_RGB_LED()
{
    RGB_LED_Color color = LED_startup;
    bool flashing = false;

    if (!s_open_for_business_message_written.load(std::memory_order_acquire))
    {
        flashing = s_gnss_pps_startup_qualification_in_progress.load(std::memory_order_acquire) ||
                   s_pps_discipline_active.load();
    }
    else
    {
        switch (get_required_top_line_message())
        {
        case 9:
            color = LED_critical;
            flashing = true;
            break;
        case 99:
            color = LED_sync;
            break;
        case 1:
            color = LED_critical;
            break;
        case 2:
        case 3:
            color = LED_warning;
            flashing = true;
            break;
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
            color = LED_warning;
            break;
        default:
            color = LED_normal;
            break;
        }
    }

    const bool enabled = !flashing || ((esp_timer_get_time() / 1000000LL) % 2 == 0);
    control_KY_016_RGB_LED(color, enabled);
}
#endif

static void update_LED_LCD_Button_task(void *parameter)

{
#if LIQUID_CRYSTAL_DISPLAY_ENABLED
    int previous_top_line_message = -1;
    int previous_second = -1;
#if UPTIME_RESTART_BUTTON_ENABLED
    int display_uptime_seconds_counter = 0;
#endif

    for (;;)
    {

#if RBG_LED_ENABLED
        update_RGB_LED();
#endif

        bool update_display = true;
#if RBG_LED_ENABLED
        update_display = s_open_for_business_message_written.load(std::memory_order_acquire);
#endif

#if OTE_UPDATES_ENABLED
        if (update_display && render_ote_display())
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
#endif

        time_t now_utc = time(nullptr);
        struct tm utc_tm{};
        gmtime_r(&now_utc, &utc_tm);

        if (update_display && utc_tm.tm_sec != previous_second)
        {
            previous_second = utc_tm.tm_sec;

#if UPTIME_RESTART_BUTTON_ENABLED
            if (check_uptime_request())
                display_uptime_seconds_counter = upTimeDisplayWillStayActiveForThisManySeconds;
#endif

            int required_top_line_message = get_required_top_line_message();

            // Determine message code for the first line:
            //
            // Standard 1st line display value .................................... "ESP32 Time Server   "
            // Ethernet not connected ............................................. "ESP32 Time Server[1]"
            // MQTT setup failed .................................................. "ESP32 Time Server[2]"
            // MQTT there are queued items ........................................ "ESP32 Time Server[3]"
            // Sanity check mismatch .............................................. "ESP32 Time Server[4]"
            // PPS missing ........................................................ "ESP32 Time Server[5]"
            // GNSS missing or invalid ............................................ "ESP32 Time Server[6]"
            // GNSS sync stale .................................................... "ESP32 Time Server[7]"
            // GNSS unlocked ...................................................... "ESP32 Time Server[8]"
            // Communication failure with the GNSS receiver ....................... "ESP32 Time Server[9]"

            // 98 used for when the button is pressed (normal periodic behaviour) . "ESP32 Time Server's "
            // 99 Time sync underway (normal periodic behaviour)                  . "ESP32 Time Server * "

            // Regarding statuses [6], [7] and [8], each condition represents a distinct failure mode:
            // [6] - GNSS timing is unusable. This is a hard fault.
            // [7] - The system hasn’t synced with GNSS as expected due to processing delays or blockages
            // [8] - GNSS module lost satellite lock. This is a raw GNSS status, not a sync fault.

            // Regarding: sync_snapshot.holdover_mode
            // In timekeeping terminology, holdover is the state where a time server continues providing time from its internal
            // oscillator after losing its external reference (in this case the GNSS). The device is no longer actively
            // synchronized but is "coasting" on its last-known good time.
            // Holdover mode is not expressly reported on the LCD's top line as it is implied when
            // Sanity check mismatch, PPS missing, GNSS missing or invalid, GNSS snyc stale, or GNSS unlocked
            // are reported.

#if UPTIME_RESTART_BUTTON_ENABLED
            if (display_uptime_seconds_counter > 0)
                required_top_line_message = 98;
#endif

            // Update top line only if it has changed
            if (required_top_line_message != previous_top_line_message)
            {
                char top_line_message[21]; // 20 chars + null
                memset(top_line_message, ' ', sizeof(top_line_message));
                memcpy(top_line_message, "ESP32 Time Server", 17);

                if (required_top_line_message == 99)
                {
                    top_line_message[18] = '*';
                }
                else if (required_top_line_message > 0 && required_top_line_message < 10)
                {
                    top_line_message[17] = '[';
                    top_line_message[18] = '0' + required_top_line_message;
                    top_line_message[19] = ']';
                }
                else if (required_top_line_message == 10)
                {
                    memcpy(top_line_message, "ESP32 Time Server's", 20);
                }

                top_line_message[20] = '\0';

                display_line(0, top_line_message);
                previous_top_line_message = required_top_line_message;
            }

#if UPTIME_RESTART_BUTTON_ENABLED
            if (display_uptime_seconds_counter > 0)
            {
                char uptime_buffer[lcdColumns + 1];
                char centered[lcdColumns + 1];
                memset(centered, ' ', lcdColumns);
                centered[lcdColumns] = '\0';

                get_uptime(uptime_buffer, sizeof(uptime_buffer));
                size_t uptime_len = strlen(uptime_buffer);
                int left_pad = static_cast<int>((lcdColumns - uptime_len) / 2);
                if (left_pad < 0)
                    left_pad = 0;
                if (uptime_len > lcdColumns)
                    uptime_len = lcdColumns;

                memcpy(centered + left_pad, uptime_buffer, uptime_len);

                display_line(1, "uptime is");
                display_line(2, centered);
                display_line(3, " days hrs:mins:secs");
                display_uptime_seconds_counter--;
            }
            else
#endif
            {
                if (s_time_has_been_set.load(std::memory_order_acquire))
                {
                    char date_line[16];
                    char time_line[24];
                    format_local_date_time(now_utc, date_line, sizeof(date_line), time_line, sizeof(time_line));
                    display_line(1, date_line);
                    display_line(2, time_line);
                }
                display_selected_ip_address(utc_tm.tm_sec);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));

#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(LED_LCD_Button);
#endif
    }
#else
    for (;;)
    {
#if RBG_LED_ENABLED
        update_RGB_LED();
#endif
#if UPTIME_RESTART_BUTTON_ENABLED
        (void)check_uptime_request();
#endif
        vTaskDelay(pdMS_TO_TICKS(50));
    }
#endif
}

extern "C" void app_main()
{
    setup_pps_input();

    initArduino();

    write_opening_messages_to_the_console();

    setup_mqtt_tf_queue();

    setup_NVS_storage();

    create_mutexes_and_semaphores();

    setup_up_the_RGB_LED();

    setup_the_LCD();

    setup_up_the_button();

    xTaskCreatePinnedToCore(update_LED_LCD_Button_task, "LED_LCD_Button_service", LED_LCD_Button_Task_Stack_Size, nullptr, 10, nullptr, tskNO_AFFINITY);

    setup_ethernet_connection();

    setup_for_ote_updates();

    setup_the_gnss();

    setup_mqtt();

    if (xTaskCreatePinnedToCore(ntp_server_task, "ntp_server", NTP_Server_Task_Stack_Size,
                                xTaskGetCurrentTaskHandle(), 20, nullptr, tskNO_AFFINITY) != pdPASS)
    {
        ESP_LOGE(TAG, "Unable to start NTP server");
        return;
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!s_ntp_server_ready.load(std::memory_order_acquire))
    {
        ESP_LOGE(TAG, "NTP server did not become ready");
        return;
    }

#if STARTUP_HEALTH_TEST_ENABLED
    if (xTaskCreatePinnedToCore(startup_health_test_task, "startup_health", Startup_Health_Test_Task_Stack_Size,
                                xTaskGetCurrentTaskHandle(), 5, nullptr, tskNO_AFFINITY) != pdPASS)
    {
        ESP_LOGE(TAG, "Health Check could not start");
    }
    else
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
#endif

    write_open_for_business_messages_to_the_console();
}

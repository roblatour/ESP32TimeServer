// ESP32 Time Server v3.0.5
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//
// ESP32 Dev Board:     ESP32-P4-ETH https://www.waveshare.com/esp32-p4-eth.htm
//                                   https://www.waveshare.com/wiki/ESP32-P4-ETH?srsltid=AfmBOoo6nZm5hsPAhtpzT6lWSHd2zhWNPM_mqgbNvyoESbjvbO7uykcH
//
//                      NOTE: Powering a Waveshare ESP32-P4 POE development board by either a USB C cable or
//                      via POE Ethernet cable is sufficient to power this project,
//
//                      *****************************************************************************************************
//                      * HOWEVER DO NOT POWER THE DEVELOPMENT BOARD VIA BOTH ITS USB C CONNECTION AND POE AT THE SAME TIME *
//                      *****************************************************************************************************
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
// (optional) Uptime momentary button to and from the ESP32-P4-ETH board:
// one terminal       <- -> ESP32-P4-ETH GPI03
// the other terminal <- -> ESP32-P4-ETH GND

#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstdarg>
#include <cstdint>
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
#include "SparkFun_u-blox_GNSS_v3.h"
#include "ESP32TimeServerSettings.h"
#include "custom/app_metadata.h"
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
#include "custom/ntp_auth.h"
#endif
#include "custom/ntp_cache.h"
#include "custom/native_ote.h"

extern "C"
{
#include "esp_chip_info.h"
#include "esp_idf_version.h"
#include "esp_event.h"
#include "esp_eth_clock.h"
#include "esp_eth_driver.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_phy.h"
#include "esp_eth_netif_glue.h"
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
#if RBG_LED_KY_016_MODULE_ENABLED
#include "driver/ledc.h"
#endif
#if RBG_LED_WS2812_MODULE_ENABLED
#include "led_strip.h"
#endif
#include "driver/sdmmc_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "sdmmc_cmd.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
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

#define TIMING_LOGI(...)           \
    do                             \
    {                              \
        if (DEBUG_ENABLED)         \
            ESP_LOGI(__VA_ARGS__); \
    } while (0)
#define TIMING_LOGW(...)           \
    do                             \
    {                              \
        if (DEBUG_ENABLED)         \
            ESP_LOGW(__VA_ARGS__); \
    } while (0)
#define TIMING_LOGE(...)           \
    do                             \
    {                              \
        if (DEBUG_ENABLED)         \
            ESP_LOGE(__VA_ARGS__); \
    } while (0)

// the following is used to determine the NTP load shedding threshold based on the ESP32-P4 chip version
// load shedding is used to limit the number of NTP requests per second that the server will process
// it can prevent the server from being overwhelmed by too many requests but has a limited effectiveness
// for example if the threshold is set at 10,000 it can allow for ~10,000 to be processed and shed the rest
// however, it cannot prevent the server from being overwhelmed if the number of requests exceeds the threshold by a
// large margin in which case the server will stop responding and preform a controlled restart
static constexpr uint32_t NTP_Requests_Per_Second_Load_Shedding_Threshold_ESP32P4_ChipVersion1_3 = 7000;
static constexpr uint32_t NTP_Requests_Per_Second_Load_Shedding_Threshold_ESP32P4_ChipVersion_above_1_3 = 10000;  
static uint32_t NTP_Requests_Per_Second_Load_Shedding_Threshold = 0;
static constexpr uint32_t NTP_ADMISSION_BURST_REQUESTS = 8;
static constexpr uint64_t NTP_ADMISSION_TOKEN_SCALE = 1000000ULL;

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


I (39724) main_cpp: Health Check started
I (39725) main_cpp: Health Check 01 - IPv4 loopback: NTPv3 standard      - Passed
I (39725) main_cpp: Health Check 02 - IPv4 loopback: NTPv4 standard      - Passed
I (39731) main_cpp: Health Check 03 - IPv4 loopback: NTPv4 interleaved   - Passed
I (39739) main_cpp: Health Check 04 - IPv4 loopback: NTPv4 auth standard - Passed
I (39746) main_cpp: Health Check 05 - IPv4 loopback: NTPv4 auth interlv  - Passed
I (39753) main_cpp: Health Check 06 - IPv4 loopback: NTPv4 invalid auth  - Passed
I (39760) main_cpp: Health Check 07 - IPv4 loopback: NTPv4 unknown key   - Passed
I (39767) main_cpp: Health Check 08 - IPv4 loopback: NTPv4 truncated MAC - Passed
I (39775) main_cpp: Health Check 09 - IPv4 loopback: NTPv4 altered reply - Passed
I (39782) main_cpp: Health Check 10 - IPv4 assigned: NTPv3 standard      - Passed
I (39789) main_cpp: Health Check 11 - IPv4 assigned: NTPv4 standard      - Passed
I (39796) main_cpp: Health Check 12 - IPv4 assigned: NTPv4 interleaved   - Passed
I (39803) main_cpp: Health Check 13 - IPv4 assigned: NTPv4 auth standard - Passed
I (39811) main_cpp: Health Check 14 - IPv4 assigned: NTPv4 auth interlv  - Passed
I (39818) main_cpp: Health Check 15 - IPv4 assigned: NTPv4 invalid auth  - Passed
I (39825) main_cpp: Health Check 16 - IPv4 assigned: NTPv4 unknown key   - Passed
I (39832) main_cpp: Health Check 17 - IPv4 assigned: NTPv4 truncated MAC - Passed
I (39839) main_cpp: Health Check 18 - IPv4 assigned: NTPv4 altered reply - Passed
I (39847) main_cpp: Health Check 19 - IPv6 loopback: NTPv3 standard      - Passed
I (39854) main_cpp: Health Check 20 - IPv6 loopback: NTPv4 standard      - Passed
I (39861) main_cpp: Health Check 21 - IPv6 loopback: NTPv4 interleaved   - Passed
I (39868) main_cpp: Health Check 22 - IPv6 loopback: NTPv4 auth standard - Passed
I (39876) main_cpp: Health Check 23 - IPv6 loopback: NTPv4 auth interlv  - Passed
I (39883) main_cpp: Health Check 24 - IPv6 loopback: NTPv4 invalid auth  - Passed
I (39890) main_cpp: Health Check 25 - IPv6 loopback: NTPv4 unknown key   - Passed
I (39897) main_cpp: Health Check 26 - IPv6 loopback: NTPv4 truncated MAC - Passed
I (39904) main_cpp: Health Check 27 - IPv6 loopback: NTPv4 altered reply - Passed
I (39912) main_cpp: Health Check 28 - IPv6 assigned: NTPv3 standard      - Passed
I (39919) main_cpp: Health Check 29 - IPv6 assigned: NTPv4 standard      - Passed
I (39926) main_cpp: Health Check 30 - IPv6 assigned: NTPv4 interleaved   - Passed
I (39933) main_cpp: Health Check 31 - IPv6 assigned: NTPv4 auth standard - Passed
I (39940) main_cpp: Health Check 32 - IPv6 assigned: NTPv4 auth interlv  - Passed
I (39948) main_cpp: Health Check 33 - IPv6 assigned: NTPv4 invalid auth  - Passed
I (39955) main_cpp: Health Check 34 - IPv6 assigned: NTPv4 unknown key   - Passed
I (39962) main_cpp: Health Check 35 - IPv6 assigned: NTPv4 truncated MAC - Passed
I (39969) main_cpp: Health Check 36 - IPv6 assigned: NTPv4 altered reply - Passed
I (39976) main_cpp: Health Check 37 - MQTT metrics                       - Passed
I (39984) main_cpp: Health Check completed

Most of the health checks have fairly obvious purposes based on their names, but a couple of them,
truncated MAC and altered reply tests, may require more explanation (detailed below).

For truncated MAC:
This test sends an otherwise valid NTP packet but deliberately cuts off part of the authentication field (the Message Authentication Code)
before the server processes it. The server receives a packet whose header and timestamps look fine, but whose MAC is shorter than expected,
so when it tries to verify the MAC at the end of the packet, the check fails and the reply is rejected or flagged as invalid. In effect,
it’s testing that the server does not accept packets where the integrity/authentication data has been damaged or partially missing, even
though the rest of the packet structure appears normal.

For altered reply:
This test takes a valid NTP response and then modifies some of its contents—typically one or more timestamp fields or other critical header
values—after the server has generated it but before the client treats it as trustworthy. When the client compares the altered reply against
what it expects (e.g., matching originate/receive/transmit timestamps or internal state), the inconsistency causes the response to be rejected
or treated as an error rather than used to adjust time. By doing this, the test checks that the client detects tampering or corruption in the
reply and doesn’t blindly accept time data that has been changed in transit.

*/

#define STARTUP_HEALTH_TEST_ENABLED 0 // 0 = Disabled; 1 = Enabled

// The following conditional compile flag is used to determine the ideal stack sizes for
// xTaskCreatePinnedToCore calls used throughout the program.
//
// Unless you are changing the code associated with options below and need to know the impacts of
// those changes have on the stack sizes then the conditional compile flags below should be disabled

#define CALCULATE_STACK_SIZES_ENABLED 0 // 0 = Disabled; 1 = Enabled

// FreeRTOS task stack allocations, in bytes.

static constexpr size_t Ethernet_Transport_Recovery_Task_Stack_Size = 2409;
static constexpr size_t GNSS_Recovery_Task_Stack_Size = 12288;
static constexpr size_t GNSS_Time_Sync_Task_Stack_Size = 2522;
static constexpr size_t Hardware_NTP_Server_Task_Stack_Size = 2719;
static constexpr size_t LED_LCD_Button_Task_Stack_Size = 3239;
static constexpr size_t Ethernet_Setup_Task_Stack_Size = 4096;
static constexpr size_t MQTT_Service_Task_Stack_Size = 3512;
static constexpr size_t NTP_Cache_Purge_Task_Stack_Size = 2222;
static constexpr size_t NTP_Server_Task_Stack_Size = 3560;
static constexpr size_t OTE_Service_Task_Stack_Size = 3560;
static constexpr size_t PPS_Discipline_Task_Stack_Size = 3584; // was 3135: the PTP servo runs in this task (measure again)

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

static esp_eth_handle_t s_eth_handle = nullptr;
static esp_netif_t *s_eth_netif = nullptr;

#if LIQUID_CRYSTAL_DISPLAY_ENABLED
static constexpr i2c_port_t LCD_I2C_PORT = I2C_NUM_0;
static constexpr gpio_num_t LCD_I2C_SDA_GPIO = GPIO_NUM_8;
static constexpr gpio_num_t LCD_I2C_SCL_GPIO = GPIO_NUM_7;
#endif

static constexpr uint16_t NTP_PORT = 123;
static constexpr size_t NTP_PACKET_SIZE = 48;
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
static constexpr size_t NTP_MAX_PACKET_SIZE = NTP_PACKET_SIZE + NTP_AUTH_TRAILER_SIZE;
#else
static constexpr size_t NTP_MAX_PACKET_SIZE = NTP_PACKET_SIZE;
#endif
static constexpr uint64_t NTP_EPOCH_OFFSET = 2208988800ULL;
// NTP_PRECISION_EXPONENT is defined after the local timebase (it follows its resolution)
static constexpr uint32_t NTP_ROOT_DISPERSION = 2; // NTP short format, 1/65536 s per unit: 2 = 30.5 us (served at stratum 1 with active PPS only)
static constexpr size_t NTP_SOCKET_BATCH_LIMIT = 16;
static constexpr EventBits_t ETH_CONNECTED_BIT = BIT0;
static constexpr EventBits_t ETH_GOT_IP_BIT = BIT1;
static constexpr EventBits_t ETH_GOT_IP6_BIT = BIT2;
static constexpr EventBits_t ETH_GOT_ROUTABLE_IP6_BIT = BIT3;
static constexpr size_t IP_ADDRESS_TEXT_SIZE = INET6_ADDRSTRLEN;
static constexpr uint32_t ROUTABLE_IPV6_ACQUISITION_TIMEOUT_MS = 30000;
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

static std::string MACToBeUsed;

struct PpsCaptureEvent
{
    uint32_t cap_ticks;          // raw MCPWM capture ticks
    int64_t approximate_edge_us; // absolute edge time in microseconds (esp_timer_get_time scale)
    int64_t isr_local_ns;        // the same instant on the fine local timebase in ns (see local_now_ns)
};

static constexpr uint32_t PPS_CAPTURE_RESOLUTION_HZ = 80000000;

static EventGroupHandle_t s_net_event_group = nullptr;
static SemaphoreHandle_t s_network_address_mutex = nullptr;
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

static std::atomic<bool> s_time_setting_in_progress{false};
static std::atomic<bool> s_time_has_been_set{false};
static std::atomic<uint64_t> s_ntp_reference_time_64{0};
static std::atomic<bool> s_ntp_reference_valid{false};
static std::atomic<bool> s_ptp_clock_ready{false};

// ---- PPS / clock compensation ----
// Actual tick rate of the MCPWM capture timer (queried in setup_pps_input()).
static uint32_t s_pps_capture_hz = PPS_CAPTURE_RESOLUTION_HZ;
// Default for the interrupt latency between the PPS edge and the capture callback. It is the central value of the latency
// (mean of the lowest 75 % of the samples). The first measurement on the development board gave 2.5 us as minimum and as median.
// With PPS_ISR_LATENCY_AUTO the latency is measured once per chip / configuration, stored in NVS and used from there;
// the default applies until then.
static constexpr double PPS_ISR_LATENCY_DEFAULT_US = 2.6;
// Value in use in nanoseconds (lock-free atomic so it can be changed at run time)
static std::atomic<int32_t> s_pps_isr_latency_ns{static_cast<int32_t>(PPS_ISR_LATENCY_DEFAULT_US * 1000.0 + 0.5)};

// ---- PTP clock operating mode ----
// 1 = free-running PTP clock (recommended). The PTP clock is set once and then never written again. Its rate is
//     corrected through the frequency adjustment of the EMAC (fine update addend), so it runs at the GNSS rate.
//     Once per second the offset of the PTP clock against the PPS is measured (by software readings, or in hardware
//     with the PTP PPS loopback, see PTPPPSLoopbackPin in ESP32TimeServerSettings.h). A filter estimates offset and
//     rate from these measurements, a servo steers the frequency, and the hardware time stamps are converted with
//     the estimate (continuous piecewise linear mapping, so time stamps taken before an update convert correctly).
//     No clock steps every second, no write latency, no time stamp that straddles a write.
// 0 = previous behaviour: the PTP clock is written from the PPS model every second (see PTP_OFFSET_MEASUREMENT)
#define PTP_FREE_RUNNING 1

// ---- PTP write latency: calibrated at run time and stored in NVS ----
// Latency between the local time reading taken just before clock_settime(CLOCK_PTP_SYSTEM) and the instant the new
// time is latched by the EMAC (the PTP clock lags by about this much unless it is compensated).
// 1 = if no stored value exists for this configuration, the latency is measured at run time (10 min warm-up after
//     the PPS lock, then 30 to 40 min of measurement), applied immediately and stored in NVS. At every later start
//     the stored value is used and no calibration is done. Until a stored or calibrated value exists,
//     PTP_WRITE_LATENCY_DEFAULT_NS is used.
// 0 = always use PTP_WRITE_LATENCY_DEFAULT_NS
// Post-write offset measurement removes the remaining write error in legacy mode.
#define PTP_LATENCY_AUTO 0
#if PTP_FREE_RUNNING
// The free-running PTP clock is not written every second, so there are no samples for the calibration. Its rare
// clock steps (only for offsets above 100 us) use PTP_WRITE_LATENCY_DEFAULT_NS; the servo removes the rest.
#undef PTP_LATENCY_AUTO
#define PTP_LATENCY_AUTO 0
#endif
// 1 = ignore the stored value and calibrate again at the next start (set back to 0 afterwards)
#define PTP_LATENCY_FORCE_RECALIBRATION 0
static constexpr int32_t PTP_WRITE_LATENCY_DEFAULT_NS = 3850; // measured on the development hardware
// Value in use in nanoseconds (lock-free atomic so it can be changed at run time)
static std::atomic<int32_t> s_ptp_write_latency_ns{PTP_WRITE_LATENCY_DEFAULT_NS};
#if PTP_LATENCY_AUTO
static constexpr int ptp_cal_block_samples = 600;  // one sample per PPS second: 10 minutes per block
static std::atomic<bool> s_ptp_cal_collect{false}; // the PPS task may append samples
static std::atomic<int> s_ptp_cal_count{0};        // samples in the current block
static int32_t *s_ptp_cal_buf = nullptr;           // block buffer, allocated while a calibration is needed
static bool s_ptp_cal_required = true;             // cleared when a stored value for this configuration was loaded
static portMUX_TYPE s_ptp_cal_mux = portMUX_INITIALIZER_UNLOCKED;

static void ptp_latency_calibration_reset(bool collect)
{
    portENTER_CRITICAL(&s_ptp_cal_mux);
    s_ptp_cal_collect.store(collect, std::memory_order_relaxed);
    s_ptp_cal_count.store(0, std::memory_order_relaxed);
    portEXIT_CRITICAL(&s_ptp_cal_mux);
}

// Called by the PPS task with the measured write latency (ns) of the first write of each second.
// Samples are only collected after the warm-up and while the calibration task has room in the block.
static inline void ptp_latency_calibration_add_sample(int64_t latency_ns)
{
    if (latency_ns < 0 || latency_ns > 100000) // implausible reading
        return;
    portENTER_CRITICAL(&s_ptp_cal_mux);
    const int n = s_ptp_cal_count.load(std::memory_order_relaxed);
    if (s_ptp_cal_collect.load(std::memory_order_relaxed) && s_ptp_cal_buf != nullptr && n < ptp_cal_block_samples)
    {
        s_ptp_cal_buf[n] = static_cast<int32_t>(latency_ns);
        s_ptp_cal_count.store(n + 1, std::memory_order_release);
    }
    portEXIT_CRITICAL(&s_ptp_cal_mux);
}
#endif
static std::atomic<bool> s_clock_comp_valid{false}; // system clock drift compensation is valid
static std::atomic<int64_t> s_clock_anchor_us{0};   // esp_timer time at which the system clock was exact
static std::atomic<bool> s_ptp_comp_valid{false};   // PTP clock drift compensation is valid
static std::atomic<int32_t> s_clock_ppb{0};         // oscillator error in ppb, >0 = clock runs fast
// Do not let an NTP reader preempt a writer while its publication sequence is odd.
static portMUX_TYPE s_clock_publish_mux = portMUX_INITIALIZER_UNLOCKED;

#if !PTP_FREE_RUNNING
// ---- PTP clock compensation, PTP_FREE_RUNNING 0 (seqlock, the PPS task is the only writer) ----
// A raw EMAC time stamp ts (ns) is converted to true time like the model clock:
//   el = ts - offset - anchor;  true = anchor + el - el * ppb / 1e9
// anchor = second of the last PPS edge in ns, offset = measured difference between the PTP clock and the model clock
// right after the PTP clock was written (mean of several bracketed read-backs, see set_hardware_clock_from_pps).
// The offset removes the error of the individual write (write latency jitter, esp_timer quantization).
// 1 = measure the offset after every write (recommended), 0 = assume the write was exact (offset 0)
#define PTP_OFFSET_MEASUREMENT 1
static std::atomic<uint32_t> s_ptp_comp_seq{0};
static std::atomic<int64_t> s_ptp_anchor_ns{0}; // second of the last PPS edge in ns
static std::atomic<int64_t> s_ptp_offset_ns{0}; // PTP clock minus model clock, measured after the last write
#if DEBUG_ENABLED
static std::atomic<int32_t> s_ptp_offset_dbg_ns{0}; // last measured offset (debug output only)
static std::atomic<int32_t> s_ptp_offset_dbg_n{0};  // number of read-backs used for it
#endif

static void ptp_comp_publish(int64_t anchor_ns, int64_t offset_ns)
{
    portENTER_CRITICAL(&s_clock_publish_mux);
    const uint32_t s = s_ptp_comp_seq.load(std::memory_order_relaxed);
    s_ptp_comp_seq.store(s + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    s_ptp_anchor_ns.store(anchor_ns, std::memory_order_relaxed);
    s_ptp_offset_ns.store(offset_ns, std::memory_order_relaxed);
    s_ptp_comp_seq.store(s + 2, std::memory_order_release);
    portEXIT_CRITICAL(&s_clock_publish_mux);
}

static bool ptp_comp_read(int64_t &anchor_ns, int64_t &offset_ns)
{
    uint32_t s1, s2;
    for (size_t attempt = 0; attempt < 3; ++attempt)
    {
        s1 = s_ptp_comp_seq.load(std::memory_order_acquire);
        if (s1 & 1u)
            continue;
        anchor_ns = s_ptp_anchor_ns.load(std::memory_order_relaxed);
        offset_ns = s_ptp_offset_ns.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        s2 = s_ptp_comp_seq.load(std::memory_order_relaxed);
        if (s1 == s2)
            return true;
    }
    return false;
}
#endif

#if PTP_FREE_RUNNING
// ---- PTP mapping, PTP_FREE_RUNNING 1 (seqlock, the PPS task is the only writer) ----
// offset(raw) = raw PTP time - true time, piecewise linear around a reference point:
//   offset(raw) = ref_offset + rate * (raw - ref_raw)   with rate = rate_new for raw >= ref_raw, rate_old before
//   true = raw - offset(raw)
// The rates are in milli-ppb (1e-12). The reference point is the instant of the last frequency update, so the two
// segments are the offset rates before and after that update and the mapping is continuous.
static std::atomic<uint32_t> s_ptpmap_seq{0};
static std::atomic<int64_t> s_ptpmap_ref_raw_ns{0};
static std::atomic<int64_t> s_ptpmap_ref_offset_ns{0};
static std::atomic<int64_t> s_ptpmap_rate_old_mppb{0};
static std::atomic<int64_t> s_ptpmap_rate_new_mppb{0};
// Ethernet MAC of the PTP clock (set when the PTP clock is enabled; frequency commands and direct reads)
static esp_eth_mac_t *s_ptp_mac = nullptr;
// Incremented whenever the PTP clock is set or re-initialised: the servo starts its estimate again
static std::atomic<uint32_t> s_ptp_epoch{0};
// Set when the EMAC PTP block was initialised again (Ethernet recovery): the frequency command is back at 0
static std::atomic<bool> s_ptp_addend_reset{false};
// PTP PPS loopback (see PTPPPSLoopbackPin): MCPWM capture ticks of the last PTP pulses, written by the interrupt
static constexpr uint32_t ptp_pps_capture_ring_size = 4;
static std::atomic<uint32_t> s_ptp_pps_captures[ptp_pps_capture_ring_size]{};
static std::atomic<uint32_t> s_ptp_pps_capture_head{0};
static std::atomic<bool> s_ptp_loopback_active{false};

static constexpr int64_t ptpmap_max_age_ns = 3600LL * 1000000000LL; // like the model clock: holdover for one hour
static constexpr int64_t ptpmap_max_rate_mppb = 500000000LL;        // +-500 ppm (plausibility limit)

static void ptp_map_publish(int64_t ref_raw_ns, int64_t ref_offset_ns, int64_t rate_old_mppb, int64_t rate_new_mppb)
{
    rate_old_mppb = std::clamp(rate_old_mppb, -ptpmap_max_rate_mppb, ptpmap_max_rate_mppb);
    rate_new_mppb = std::clamp(rate_new_mppb, -ptpmap_max_rate_mppb, ptpmap_max_rate_mppb);
    portENTER_CRITICAL(&s_clock_publish_mux);
    const uint32_t s = s_ptpmap_seq.load(std::memory_order_relaxed);
    s_ptpmap_seq.store(s + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    s_ptpmap_ref_raw_ns.store(ref_raw_ns, std::memory_order_relaxed);
    s_ptpmap_ref_offset_ns.store(ref_offset_ns, std::memory_order_relaxed);
    s_ptpmap_rate_old_mppb.store(rate_old_mppb, std::memory_order_relaxed);
    s_ptpmap_rate_new_mppb.store(rate_new_mppb, std::memory_order_relaxed);
    s_ptpmap_seq.store(s + 2, std::memory_order_release);
    portEXIT_CRITICAL(&s_clock_publish_mux);
}

// Offset of the PTP clock (raw - true, ns) at the raw PTP time raw_ns. Returns false if raw_ns is too far from the
// reference point. The caller checks s_ptp_comp_valid first.
static bool ptp_map_offset_at(int64_t raw_ns, int64_t &offset_ns)
{
    int64_t ref_raw = 0, ref_offset = 0, rate_old = 0, rate_new = 0;
    uint32_t s1, s2;
    bool consistent = false;
    for (size_t attempt = 0; attempt < 3; ++attempt)
    {
        s1 = s_ptpmap_seq.load(std::memory_order_acquire);
        if (s1 & 1u)
            continue;
        ref_raw = s_ptpmap_ref_raw_ns.load(std::memory_order_relaxed);
        ref_offset = s_ptpmap_ref_offset_ns.load(std::memory_order_relaxed);
        rate_old = s_ptpmap_rate_old_mppb.load(std::memory_order_relaxed);
        rate_new = s_ptpmap_rate_new_mppb.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        s2 = s_ptpmap_seq.load(std::memory_order_relaxed);
        if (s1 == s2)
        {
            consistent = true;
            break;
        }
    }
    if (!consistent)
        return false;

    const int64_t d = raw_ns - ref_raw;
    if (d < -ptpmap_max_age_ns || d > ptpmap_max_age_ns)
        return false;
    const int64_t rate = (d >= 0) ? rate_new : rate_old;
    // rate [1e-12] * d [ns] in two parts, so the product cannot overflow: whole seconds and the rest
    const int64_t d_s = d / 1000000000LL;
    const int64_t d_r = d % 1000000000LL;
    offset_ns = ref_offset + (d_s * rate) / 1000LL + (d_r * rate) / 1000000000000LL;
    return true;
}
#endif

// ---- Fine local timebase ----
// esp_timer_get_time() returns whole microseconds: the systimer counter (16 MHz on the ESP32-P4) divided by 16 and
// truncated. All time critical readings (PPS interrupt, model clock reads, PTP offset measurements, interrupt latency
// test) use the counter itself instead, so the 1 us truncation (0.29 us RMS per reading) no longer enters the
// software time stamps and the PTP measurements. Resolution: 62.5 ns. esp_timer_get_time() is still used for
// timeouts and statistics, where 1 us does not matter.
// 1 = raw systimer counter (recommended), 0 = esp_timer_get_time() (1 us, previous behaviour)
#define FINE_LOCAL_TIMEBASE 1
#if FINE_LOCAL_TIMEBASE && defined(CONFIG_IDF_TARGET_ESP32P4)
// Counter behind esp_timer_get_time() (esp_timer/src/esp_timer_impl_systimer.c, placed in IRAM like esp_timer_get_time).
// It is declared in a private esp_timer header, therefore the declaration is repeated here.
extern "C" uint64_t esp_timer_impl_get_counter_reg(void);
static constexpr uint32_t LOCAL_TICKS_PER_US = 16; // ESP32-P4 systimer: 16 MHz from the 40 MHz crystal
static inline __attribute__((always_inline)) uint64_t local_ticks() { return esp_timer_impl_get_counter_reg(); }
static inline __attribute__((always_inline)) int64_t local_ticks_to_ns(uint64_t ticks) { return static_cast<int64_t>((ticks * 125u) >> 1); } // 62.5 ns per tick
static constexpr int8_t LOCAL_TIMEBASE_PRECISION_EXPONENT = -24;                                                                             // 2^-24 s = 60 ns
#else
static constexpr uint32_t LOCAL_TICKS_PER_US = 1; // 1 tick = 1 us
static inline __attribute__((always_inline)) uint64_t local_ticks() { return static_cast<uint64_t>(esp_timer_get_time()); }
static inline __attribute__((always_inline)) int64_t local_ticks_to_ns(uint64_t ticks) { return static_cast<int64_t>(ticks) * 1000LL; }
static constexpr int8_t LOCAL_TIMEBASE_PRECISION_EXPONENT = -20; // 2^-20 s = 0.95 us
#endif
// Precision announced in NTP replies: resolution of the software clock (the EMAC time stamps have 40 ns steps)
static constexpr int8_t NTP_PRECISION_EXPONENT = LOCAL_TIMEBASE_PRECISION_EXPONENT;

// Local time in ns since boot on the fine timebase (same zero point as esp_timer_get_time).
// always_inline: these are also used by the PPS capture interrupt (IRAM)
static inline __attribute__((always_inline)) int64_t local_now_ns() { return local_ticks_to_ns(local_ticks()); }
// The same instant on the esp_timer_get_time() scale (whole microseconds, truncated like esp_timer)
static inline __attribute__((always_inline)) int64_t local_ticks_to_esp_us(uint64_t ticks) { return static_cast<int64_t>(ticks / LOCAL_TICKS_PER_US); }

// ---- Model clock: replaces gettimeofday() for NTP time stamps ----
// true time = second of the last PPS edge + time since that edge (fine local timebase), corrected by the measured ppb.
static std::atomic<uint32_t> s_model_seq{0};
static std::atomic<bool> s_model_valid{false};
static std::atomic<bool> s_model_rate_valid{false};
static std::atomic<int64_t> s_model_sec{0};     // Unix second of the last PPS edge
static std::atomic<int64_t> s_model_edge_ns{0}; // local time of that edge in ns (local_now_ns scale)
static std::atomic<int32_t> s_model_ppb{0};     // >0 = local clock runs fast

// The PPS task is the only writer (seqlock, so readers never see mixed values)
static void model_publish(int64_t unix_sec, int64_t edge_ns, int32_t ppb)
{
    portENTER_CRITICAL(&s_clock_publish_mux);
    const uint32_t s = s_model_seq.load(std::memory_order_relaxed);
    s_model_seq.store(s + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    s_model_sec.store(unix_sec, std::memory_order_relaxed);
    s_model_edge_ns.store(edge_ns, std::memory_order_relaxed);
    s_model_ppb.store(ppb, std::memory_order_relaxed);
    s_model_seq.store(s + 2, std::memory_order_release);
    s_model_valid.store(true, std::memory_order_release);
    portEXIT_CRITICAL(&s_clock_publish_mux);
}

static void model_invalidate()
{
    s_model_valid.store(false, std::memory_order_release);
    s_model_rate_valid.store(false, std::memory_order_release);
}

// Model time (Unix ns) for a local time in ns (local_now_ns scale). Returns false if the model is not usable.
static bool model_ns_at(int64_t esp_ns, int64_t &unix_ns)
{
    if (!s_model_valid.load(std::memory_order_acquire))
        return false;

    int64_t sec = 0, edge_ns = 0;
    int32_t ppb = 0;
    uint32_t s1, s2;
    // A higher-priority reader can preempt the writer on the same core.
    bool consistent = false;
    for (size_t attempt = 0; attempt < 3; ++attempt)
    {
        s1 = s_model_seq.load(std::memory_order_acquire);
        if (s1 & 1u)
            continue;
        sec = s_model_sec.load(std::memory_order_relaxed);
        edge_ns = s_model_edge_ns.load(std::memory_order_relaxed);
        ppb = s_model_ppb.load(std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_acquire);
        s2 = s_model_seq.load(std::memory_order_relaxed);
        if (s1 == s2)
        {
            consistent = true;
            break;
        }
    }
    if (!consistent)
        return false;

    const int64_t el = esp_ns - edge_ns;
    if (el < -1000000000LL || el > 3600000000000LL) // holdover for at most one hour
        return false;

    unix_ns = sec * 1000000000LL + el - (el * static_cast<int64_t>(ppb)) / 1000000000LL;
    return true;
}

static_assert(PreferIPvX == 0 || PreferIPvX == 1 || PreferIPvX == 2 || PreferIPvX == 4 || PreferIPvX == 6 || PreferIPvX == 8 || PreferIPvX == 9,
              "PreferIPvX must be 0, 1, 2, 4, 6, 8, or 9");

static char s_ip_address[IP_ADDRESS_TEXT_SIZE] = "";
static char s_ipv4_address[INET_ADDRSTRLEN] = "";
static char s_ipv6_address[IP_ADDRESS_TEXT_SIZE] = "";
enum class ipv6_address_priority_t : uint8_t
{
    none,
    link_local,
    unique_local,
    global_unicast
};
static ipv6_address_priority_t s_ipv6_address_priority = ipv6_address_priority_t::none;
// Assigned IPv4 address in network byte order (0 = none); read lock-free by the hardware NTP path
static std::atomic<uint32_t> s_own_ipv4_address_be{0};
static std::atomic<uint32_t> s_own_ipv4_broadcast_be{0};
static std::atomic<uint32_t> s_own_ipv6_sequence{0};
static std::atomic<uint32_t> s_own_ipv6_count{0};
static std::atomic<uint32_t> s_own_ipv6_addresses[CONFIG_LWIP_IPV6_NUM_ADDRESSES][4]{};

struct network_address_snapshot_t
{
    char selected[IP_ADDRESS_TEXT_SIZE];
    char ipv4[INET_ADDRSTRLEN];
    char ipv6[IP_ADDRESS_TEXT_SIZE];
    ipv6_address_priority_t ipv6_priority;
};

static constexpr bool IPV4_ENABLED = PreferIPvX != 2;
static constexpr bool IPV6_ENABLED = PreferIPvX != 1;

static bool has_usable_network_address(EventBits_t event_bits)
{
    if (PreferIPvX == 1)
        return (event_bits & ETH_GOT_IP_BIT) != 0;
    if (PreferIPvX == 2)
        return (event_bits & ETH_GOT_ROUTABLE_IP6_BIT) != 0;
    return (event_bits & (ETH_GOT_IP_BIT | ETH_GOT_ROUTABLE_IP6_BIT)) != 0;
}

static void update_selected_ip_address_locked()
{
    const bool has_ipv4 = s_ipv4_address[0] != '\0';
    const bool has_ipv6 = s_ipv6_address[0] != '\0';

    const char *selected_address = "";
    if (PreferIPvX == 1)
        selected_address = s_ipv4_address;
    else if (PreferIPvX == 2)
        selected_address = s_ipv6_address;
    else if (PreferIPvX == 4)
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

static network_address_snapshot_t get_network_address_snapshot()
{
    network_address_snapshot_t snapshot{};
    if (s_network_address_mutex != nullptr && xSemaphoreTake(s_network_address_mutex, portMAX_DELAY) == pdTRUE)
    {
        snprintf(snapshot.selected, sizeof(snapshot.selected), "%s", s_ip_address);
        snprintf(snapshot.ipv4, sizeof(snapshot.ipv4), "%s", s_ipv4_address);
        snprintf(snapshot.ipv6, sizeof(snapshot.ipv6), "%s", s_ipv6_address);
        snapshot.ipv6_priority = s_ipv6_address_priority;
        xSemaphoreGive(s_network_address_mutex);
    }
    return snapshot;
}

static void clear_network_addresses()
{
    if (s_network_address_mutex == nullptr || xSemaphoreTake(s_network_address_mutex, portMAX_DELAY) != pdTRUE)
        return;

    s_ip_address[0] = '\0';
    s_ipv4_address[0] = '\0';
    s_ipv6_address[0] = '\0';
    s_ipv6_address_priority = ipv6_address_priority_t::none;
    s_own_ipv4_address_be.store(0, std::memory_order_relaxed);
    s_own_ipv4_broadcast_be.store(0, std::memory_order_relaxed);
    s_own_ipv6_count.store(0, std::memory_order_release);
    xSemaphoreGive(s_network_address_mutex);
}

static void set_network_ipv4_address(const esp_ip4_addr_t &address, const esp_ip4_addr_t &netmask)
{
    if (s_network_address_mutex == nullptr || xSemaphoreTake(s_network_address_mutex, portMAX_DELAY) != pdTRUE)
        return;

    snprintf(s_ipv4_address, sizeof(s_ipv4_address), IPSTR, IP2STR(&address));
    s_own_ipv4_address_be.store(address.addr, std::memory_order_relaxed);
    s_own_ipv4_broadcast_be.store(address.addr | ~netmask.addr, std::memory_order_relaxed);
    update_selected_ip_address_locked();
    xSemaphoreGive(s_network_address_mutex);
}

static void clear_network_ipv4_address()
{
    if (s_network_address_mutex == nullptr || xSemaphoreTake(s_network_address_mutex, portMAX_DELAY) != pdTRUE)
        return;

    s_ipv4_address[0] = '\0';
    s_own_ipv4_address_be.store(0, std::memory_order_relaxed);
    s_own_ipv4_broadcast_be.store(0, std::memory_order_relaxed);
    update_selected_ip_address_locked();
    xSemaphoreGive(s_network_address_mutex);
}

static ipv6_address_priority_t get_ipv6_address_priority(const struct in6_addr &address)
{
    if ((address.s6_addr[0] & 0xE0) == 0x20)
        return ipv6_address_priority_t::global_unicast;
    if ((address.s6_addr[0] & 0xFE) == 0xFC)
        return ipv6_address_priority_t::unique_local;
    return ipv6_address_priority_t::link_local;
}

static bool set_network_ipv6_address(const esp_ip6_addr_t &address)
{
    struct in6_addr parsed_address{};
    memcpy(&parsed_address, address.addr, sizeof(parsed_address));
    const ipv6_address_priority_t priority = get_ipv6_address_priority(parsed_address);
    const bool is_routable = priority >= ipv6_address_priority_t::unique_local;

    char text_address[IP_ADDRESS_TEXT_SIZE] = "";
    if (inet_ntop(AF_INET6, &parsed_address, text_address, sizeof(text_address)) == nullptr)
        return false;

    if (s_network_address_mutex == nullptr || xSemaphoreTake(s_network_address_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    if (priority >= s_ipv6_address_priority)
    {
        snprintf(s_ipv6_address, sizeof(s_ipv6_address), "%s", text_address);
        s_ipv6_address_priority = priority;
        update_selected_ip_address_locked();
    }
    xSemaphoreGive(s_network_address_mutex);
    return is_routable;
}

static void refresh_network_ipv6_addresses()
{
    if (!IPV6_ENABLED || s_eth_netif == nullptr)
        return;

    esp_ip6_addr_t addresses[CONFIG_LWIP_IPV6_NUM_ADDRESSES]{};
    const int address_count = esp_netif_get_all_ip6(s_eth_netif, addresses);
    if (s_network_address_mutex != nullptr && xSemaphoreTake(s_network_address_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_own_ipv6_sequence.fetch_add(1, std::memory_order_relaxed);
        std::atomic_thread_fence(std::memory_order_release);
        const uint32_t count = static_cast<uint32_t>(std::clamp(address_count, 0, CONFIG_LWIP_IPV6_NUM_ADDRESSES));
        for (uint32_t index = 0; index < count; ++index)
            for (size_t word = 0; word < 4; ++word)
                s_own_ipv6_addresses[index][word].store(addresses[index].addr[word], std::memory_order_relaxed);
        s_own_ipv6_count.store(count, std::memory_order_relaxed);
        s_own_ipv6_sequence.fetch_add(1, std::memory_order_release);
        xSemaphoreGive(s_network_address_mutex);
    }
    bool routable_address_acquired = false;
    for (int index = 0; index < address_count; ++index)
        routable_address_acquired = set_network_ipv6_address(addresses[index]) || routable_address_acquired;

    if (address_count > 0)
        xEventGroupSetBits(s_net_event_group, ETH_GOT_IP6_BIT);
    if (routable_address_acquired)
        xEventGroupSetBits(s_net_event_group, ETH_GOT_ROUTABLE_IP6_BIT);
}

static void log_network_ipv6_addresses()
{
    if (!IPV6_ENABLED || s_eth_netif == nullptr)
        return;

    esp_ip6_addr_t addresses[CONFIG_LWIP_IPV6_NUM_ADDRESSES]{};
    const int address_count = esp_netif_get_all_ip6(s_eth_netif, addresses);
    if (address_count == 0)
    {
        ESP_LOGW(TAG, "Ethernet IPv6 candidates: none");
        return;
    }

    for (int index = 0; index < address_count; ++index)
    {
        struct in6_addr address{};
        memcpy(&address, addresses[index].addr, sizeof(address));
        char text_address[IP_ADDRESS_TEXT_SIZE] = "";
        if (inet_ntop(AF_INET6, &address, text_address, sizeof(text_address)) == nullptr)
            continue;

        const char *type = "link-local";
        switch (get_ipv6_address_priority(address))
        {
        case ipv6_address_priority_t::global_unicast:
            type = "global-unicast";
            break;
        case ipv6_address_priority_t::unique_local:
            type = "unique-local";
            break;
        default:
            break;
        }
        ESP_LOGI(TAG, "Ethernet IPv6 candidate: %s (%s)", text_address, type);
    }
}

static std::atomic<bool> s_ethernet_connected{false};
static std::atomic<bool> s_ipv4_ever_acquired{false};
static std::atomic<int64_t> s_last_physical_link_down_us{0};
static std::atomic<bool> s_ethernet_recovery_in_progress{false};
static std::atomic<uint32_t> s_ethernet_rx_frames{0};
static std::atomic<bool> s_hardware_ntp_accepting{false};
static std::atomic<bool> s_ntp_external_responses_enabled{false};
static std::atomic<bool> s_ntp_server_ready{false};
#if RBG_LED_KY_016_MODULE_ENABLED || RBG_LED_WS2812_MODULE_ENABLED
static std::atomic<bool> s_gnss_pps_startup_qualification_in_progress{false};
static void control_RGB_LED(RGB_LED_Colour Colour, bool enabled);
#endif
#if RBG_LED_WS2812_MODULE_ENABLED
static led_strip_handle_t s_ws2812_strip = nullptr;
static uint8_t s_ws2812_brightness = 0;
#endif
static std::atomic<bool> s_open_for_business_message_written{false};
static std::atomic<int64_t> s_last_hardware_ntp_response_us{0};
static std::atomic<int64_t> s_last_ntp_input_us{0};
static std::atomic<int64_t> s_first_ntp_input_us{0};
static std::atomic<int64_t> s_last_ntp_response_us{0};
static std::atomic<int64_t> s_last_load_shedding_us{0};
static std::atomic<uint64_t> s_ntp_admission_deadline{0};
static std::atomic<uint32_t> s_ntp_input_count{0};
static std::atomic<bool> s_ntp_shed_during_period{false};
static SemaphoreHandle_t s_hardware_ntp_transmit_mutex = nullptr;

enum class ntp_transport_health_t : uint8_t
{
    link_down,
    no_ip,
    idle,
    healthy,
    stalled
};
static std::atomic<ntp_transport_health_t> s_ntp_transport_health{ntp_transport_health_t::idle};

static bool ntp_load_shedding_active()
{
    const int64_t last_shedding_us = s_last_load_shedding_us.load(std::memory_order_relaxed);
    return last_shedding_us > 0 && esp_timer_get_time() - last_shedding_us < 1500000LL;
}

static void record_ntp_input(int64_t now_us)
{
    s_ntp_input_count.fetch_add(1, std::memory_order_relaxed);
    s_last_ntp_input_us.store(now_us, std::memory_order_relaxed);
    int64_t first_input_us = 0;
    s_first_ntp_input_us.compare_exchange_strong(first_input_us, now_us, std::memory_order_relaxed);
}

static void record_ntp_load_shedding(int64_t now_us)
{
    s_ntp_shed_during_period.store(true, std::memory_order_relaxed);
    s_last_load_shedding_us.store(now_us, std::memory_order_relaxed);
}

static bool admit_ntp_request()
{
    const int64_t now_us = esp_timer_get_time();
    record_ntp_input(now_us);

    // Scaled virtual time preserves fractional request intervals (including 8,500/s).
    const uint64_t now_scaled = static_cast<uint64_t>(now_us) * NTP_Requests_Per_Second_Load_Shedding_Threshold;
    const uint64_t latest_admission = now_scaled + (NTP_ADMISSION_BURST_REQUESTS - 1) * NTP_ADMISSION_TOKEN_SCALE;
    uint64_t current = s_ntp_admission_deadline.load(std::memory_order_relaxed);
    for (;;)
    {
        const uint64_t base = std::max(current, now_scaled);
        if (NTP_Requests_Per_Second_Load_Shedding_Threshold == 0 || base > latest_admission)
        {
            record_ntp_load_shedding(now_us);
            return false;
        }
        const uint64_t next = base + NTP_ADMISSION_TOKEN_SCALE;
        if (s_ntp_admission_deadline.compare_exchange_weak(current, next, std::memory_order_relaxed))
            return true;
    }
}

static bool configure_static_ip();
static bool configure_hardware_timestamps();
#if PTP_FREE_RUNNING
static void setup_ptp_pps_loopback(esp_eth_mac_t *mac);
#endif
static void start_ethernet_driver();

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

static constexpr uint32_t MQTTFrequencyOfKeepAliveRequest = 30; // in seconds (do not raise above 30

// MQTT_MAX_REPORT_SIZE is based on MQTT_TF_Client_Limit (in ESP32TimeServerSetting.h)
// please see the spreadsheet at tools/json_message_calculator.xlsx for calculation details
// Note: increasing this limit will increase memory usage for MQTT report buffering and may impact system performance.
static constexpr size_t MQTT_MAX_REPORT_SIZE = 39166;

// MQTT_REPORT_SIZE based on MQTT_CLIENT_SIZE (below)
// please see the spreadsheet at tools/json_message_calculator.xlsx for calculation details
// Note: increasing this limit will increase memory usage for MQTT report buffering and may impact system performance.
static constexpr size_t MQTT_CLIENT_LIMIT = 50;
static constexpr size_t MQTT_REPORT_SIZE = 4966;

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
static std::atomic<uint32_t> s_ntp_authenticated_valid_requests{0};
static std::atomic<uint32_t> s_ntp_authenticated_invalid_requests{0};
static std::atomic<uint32_t> s_ntp_unauthenticated_valid_requests{0};
static std::atomic<uint32_t> s_ntp_unauthenticated_invalid_requests{0};
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
static char s_mqtt_final_payload[MQTT_REPORT_SIZE] = "";
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
static void mqtt_enqueue_ntp_request(const struct sockaddr_storage &source_address);
#endif

static void controlled_restart(const char *reason);

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
static std::atomic<int64_t> s_ntp_last_pps_seen_us{0};
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
}

static void publish_ntp_sync_state_locked(const sync_state_t &state)
{
    // seqlock writer (always called with s_sync_state_mutex held): odd sequence while the values change
    s_ntp_sync_state_sequence.fetch_add(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release); // the data stores must not move before the odd sequence
    s_ntp_last_successful_sync_us.store(state.last_successful_sync_us, std::memory_order_relaxed);
    s_ntp_last_gnss_valid_us.store(state.last_gnss_valid_us, std::memory_order_relaxed);
    s_ntp_last_pps_seen_us.store(state.last_pps_seen_us, std::memory_order_relaxed);
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

// Both candidates of the first synchronisation have to agree: their labels have to advance by exactly the number of
// seconds between their PPS edges (a wrongly labelled edge is detected here)
static bool first_sync_candidates_are_plausible(const sync_candidate_t &first_candidate, const sync_candidate_t &second_candidate)
{
    const time_t label_delta = second_candidate.candidate_time - first_candidate.candidate_time;
    const int64_t edge_delta_us = second_candidate.pps_release_time_us - first_candidate.pps_release_time_us;
    const int64_t edge_delta_s = (edge_delta_us + 500000LL) / 1000000LL;
    return label_delta >= 1 && label_delta <= 3 && static_cast<int64_t>(label_delta) == edge_delta_s &&
           llabs(edge_delta_us - edge_delta_s * 1000000LL) < 100000LL;
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
    if (PreferIPvX == 9)
    {
        display_line(3, "");
        return;
    }

    static char last_known_IPv6_addresss[IP_ADDRESS_TEXT_SIZE] = "";

    static char ipv6_part1[IP_ADDRESS_TEXT_SIZE] = "";
    static char ipv6_part2[IP_ADDRESS_TEXT_SIZE] = "";

    network_address_snapshot_t addresses = get_network_address_snapshot();

    if (PreferIPvX == 8)
    {
        const char *ipv6_status = "N/A";
        if (addresses.ipv6[0] != '\0')
        {
            switch (addresses.ipv6_priority)
            {
            case ipv6_address_priority_t::global_unicast:
                ipv6_status = "GUA";
                break;
            case ipv6_address_priority_t::unique_local:
                ipv6_status = "ULA";
                break;
            case ipv6_address_priority_t::link_local:
                ipv6_status = "LLA";
                break;
            default:
                break;
            }
        }
        char status_line[lcdColumns + 1];
        snprintf(status_line, sizeof(status_line), "IPv4: %s  IPv6: %s", addresses.ipv4[0] != '\0' ? "OK" : "N/A", ipv6_status);
        display_line(3, status_line);
        return;
    }

    // if no IP address is selected, clear the display line
    if (addresses.selected[0] == '\0')
    {
        display_line(3, "");
        return;
    }

    // An IPv6 can change from Link-Local Address (LLA) to Unique-Local Address (ULA) to Global-Unicast Address (GUA)
    // so if the selected IPv6 address has changed reset the IPv6 address parts
    if (PreferIPvX == 0 || PreferIPvX == 2 || PreferIPvX == 6)
    {
        if (strcmp(addresses.ipv6, last_known_IPv6_addresss) != 0)
        {
            size_t total_len = strlen(addresses.ipv6);
            if (total_len == 0)
            {
                ipv6_part1[0] = '\0';
                ipv6_part2[0] = '\0';
                last_known_IPv6_addresss[0] = '\0';
            }
            else
            {
                size_t half_len = total_len / 2;
                while (half_len > 0 && addresses.ipv6[half_len] != ':')
                    half_len--;

                size_t remainder_len = total_len - half_len;
                strncpy(ipv6_part1, addresses.ipv6, half_len);
                ipv6_part1[half_len] = '\0';
                strncpy(ipv6_part2, addresses.ipv6 + half_len, remainder_len);
                ipv6_part2[remainder_len] = '\0';

                strncpy(last_known_IPv6_addresss, addresses.ipv6, IP_ADDRESS_TEXT_SIZE - 1);
                last_known_IPv6_addresss[IP_ADDRESS_TEXT_SIZE - 1] = '\0';
            }
        }
    }

    switch (PreferIPvX)
    {
    case 0:

        // no preference between IPv4 and IPv6
        // each minute:
        // display the IPv4 address for the first 20 seconds
        // display the first part of the IPv6 address in next 20 seconds
        // display the second part of the IPv6 address in the last 20 seconds

        if (seconds < 20)
            display_line(3, addresses.ipv4);
        else if (seconds < 40)
            display_line(3, ipv6_part1);
        else
            display_line(3, ipv6_part2);

        break;

    case 1:
    case 4:
        // Show the IPv4 address only
        if (strcmp(addresses.selected, addresses.ipv4) == 0)
        {
            display_line(3, addresses.selected);
            return;
        }
        break;

    case 2:
    case 6:
        // Show the IPv6 address only

        // each minute:
        // display the first part of the IPv6 address for the first 30 seconds
        // display the second part of the IPv6 address for the next 30 seconds
        if (seconds < 30)
            display_line(3, ipv6_part1);
        else
            display_line(3, ipv6_part2);
        break;
    }
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

    // The values are limited to their field widths so that the compiler can prove the output length
    // (with -O2 GCC reports a possible truncation otherwise, and -Werror=all makes that an error)
    const unsigned int year = static_cast<unsigned int>(local_tm.tm_year + 1900) % 10000u;
    const unsigned int month = static_cast<unsigned int>(local_tm.tm_mon + 1) % 100u;
    const unsigned int day = static_cast<unsigned int>(local_tm.tm_mday) % 100u;
    snprintf(date_string, date_size, "%04u-%02u-%02u", year, month, day);

    unsigned int hour_value = static_cast<unsigned int>(local_tm.tm_hour) % 24u;
    if (!display24HourFormat)
    {
        hour_value %= 12;
        if (hour_value == 0)
            hour_value = 12;
    }

    char zone[8] = "";

    if (displayTimeZone)
        strftime(zone, sizeof(zone), "%Z", &local_tm);

    const unsigned int minute = static_cast<unsigned int>(local_tm.tm_min) % 60u;
    const unsigned int second = static_cast<unsigned int>(local_tm.tm_sec) % 61u; // 60 = leap second
    const char *hour_format = displayHoursWithPaddedZero ? "%02u:%02u:%02u" : "%u:%02u:%02u";
    int len = snprintf(time_string, time_size, hour_format, hour_value, minute, second);

    if (!display24HourFormat && len >= 0 && static_cast<size_t>(len) < time_size)
        len += snprintf(time_string + len, time_size - len, " %s", local_tm.tm_hour < 12 ? "AM" : "PM");

    if (displayTimeZone && zone[0] != '\0' && len >= 0 && static_cast<size_t>(len) < time_size)
        snprintf(time_string + len, time_size - len, " %s", zone);
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
        ESP_LOGE(TAG, "NVS recovery requires operator action; automatic erase is disabled to preserve authentication storage");
        return false;
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
        controlled_restart("manual_restart");
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

static inline uint64_t ntp64_from_unix_ns(int64_t unix_ns)
{
    const uint64_t sec = NTP_EPOCH_OFFSET + static_cast<uint64_t>(unix_ns / 1000000000LL);
    const uint64_t rem = static_cast<uint64_t>(unix_ns % 1000000000LL);
    return (sec << 32) | ((rem << 32) / 1000000000ULL);
}

static uint64_t get_disciplined_time_in_ntp64_format()
{
    int64_t unix_ns = 0;
    if (model_ns_at(local_now_ns(), unix_ns) && unix_ns > 0)
        return ntp64_from_unix_ns(unix_ns);
    return 0;
}

static uint64_t get_current_time_in_ntp64_format()
{
    const uint64_t disciplined_time = get_disciplined_time_in_ntp64_format();
    if (disciplined_time != 0)
        return disciplined_time;
    // Fallback (model not valid yet): system clock with drift compensation
    struct timeval now{};
    gettimeofday(&now, nullptr);

    const uint64_t seconds = NTP_EPOCH_OFFSET + static_cast<uint64_t>(now.tv_sec);
    const uint64_t fraction = (static_cast<uint64_t>(now.tv_usec) << 32) / 1000000ULL;
    uint64_t ts = (seconds << 32) | fraction;

    if (s_clock_comp_valid.load(std::memory_order_acquire))
    {
        const int64_t elapsed_us = esp_timer_get_time() - s_clock_anchor_us.load(std::memory_order_relaxed);
        if (elapsed_us > 0 && elapsed_us < 3600000000LL)
        {
            const int64_t corr_ns = elapsed_us * static_cast<int64_t>(s_clock_ppb.load(std::memory_order_relaxed)) / 1000000LL;
            ts = static_cast<uint64_t>(static_cast<int64_t>(ts) - (corr_ns * 4294967296LL) / 1000000000LL);
        }
    }
    return ts;
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
    bool holdover = false;
    uint32_t root_dispersion = NTP_ROOT_DISPERSION;
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
        int64_t last_pps_us = s_ntp_last_pps_seen_us.load(std::memory_order_relaxed);
        bool pps_active = s_ntp_pps_active.load(std::memory_order_relaxed);
        bool gnss_timing_valid = s_ntp_gnss_timing_valid.load(std::memory_order_relaxed);
        sync_faults_t faults{
            s_ntp_pps_missing.load(std::memory_order_relaxed),
            s_ntp_gnss_invalid.load(std::memory_order_relaxed),
            s_ntp_sanity_mismatch.load(std::memory_order_relaxed),
            s_ntp_sync_stale.load(std::memory_order_relaxed)};

        std::atomic_thread_fence(std::memory_order_acquire); // the data loads must not move after the second sequence load
        if (sequence_before != s_ntp_sync_state_sequence.load(std::memory_order_relaxed))
            continue;

        int64_t now_us = esp_timer_get_time();
        bool gnss_recent = gnss_timing_valid && last_gnss_valid_us > 0 &&
                           (now_us - last_gnss_valid_us) <= Gnss_Validity_Timeout_Us;
        bool sync_stale = last_successful_sync_us > 0 &&
                          (now_us - last_successful_sync_us) > Sync_Stale_After_Us;
        bool gnss_synchronized = gnss_recent && !sync_stale && !faults.gnss_invalid;
        const int64_t pps_age_us = now_us - last_pps_us;
        const bool pps_recent = last_pps_us > 0 && pps_age_us >= 0 && pps_age_us <= Gnss_Validity_Timeout_Us;
        bool stratum_one = s_time_has_been_set.load(std::memory_order_acquire) &&
                           !s_time_setting_in_progress.load(std::memory_order_acquire) &&
                           !has_sync_fault(faults) && pps_active && pps_recent && gnss_synchronized &&
                           s_ntp_reference_valid.load(std::memory_order_acquire);

        if (stratum_one)
            return {0, 1, "GPS", true, gnss_synchronized, pps_active};
        const bool holdover = s_time_has_been_set.load(std::memory_order_acquire) &&
                              !s_time_setting_in_progress.load(std::memory_order_acquire) &&
                              last_pps_us > 0 && pps_age_us > Gnss_Validity_Timeout_Us &&
                              pps_age_us < ntpHoldoverDurationUs &&
                              last_gnss_valid_us > 0 &&
                              last_pps_us - last_gnss_valid_us <= Gnss_Validity_Timeout_Us &&
                              !faults.sanity_mismatch && !faults.sync_stale && !sync_stale &&
                              s_model_valid.load(std::memory_order_acquire) &&
                              s_model_rate_valid.load(std::memory_order_acquire) &&
                              s_ntp_reference_valid.load(std::memory_order_acquire) &&
                              (!s_ptp_clock_ready.load(std::memory_order_acquire) ||
                               s_ptp_comp_valid.load(std::memory_order_acquire));
        if (holdover)
        {
            const uint32_t growth = static_cast<uint32_t>(
                (pps_age_us * ntpHoldoverDispersionPpm * 65536LL + 999999999999LL) / 1000000000000LL);
            return {0, 1, "GPS", true, false, false, true, NTP_ROOT_DISPERSION + growth};
        }
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
    reply[8] = static_cast<uint8_t>(status.root_dispersion >> 24);
    reply[9] = static_cast<uint8_t>(status.root_dispersion >> 16);
    reply[10] = static_cast<uint8_t>(status.root_dispersion >> 8);
    reply[11] = static_cast<uint8_t>(status.root_dispersion);
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
static constexpr size_t RAW_NTP_FRAME_SIZE = ETH_HEADER_SIZE + IPV4_HEADER_SIZE + UDP_HEADER_SIZE + NTP_MAX_PACKET_SIZE;
static constexpr size_t RAW_IPV6_NTP_FRAME_SIZE = ETH_HEADER_SIZE + IPV6_HEADER_SIZE + UDP_HEADER_SIZE + NTP_MAX_PACKET_SIZE;
static constexpr size_t HARDWARE_NTP_REQUEST_QUEUE_DEPTH = 256;
static constexpr size_t HARDWARE_NTP_REQUEST_BUFFER_COUNT = 64; // Do not increase this value, larger pools caused unacceptable internal/DMA-memory pressure
static constexpr size_t HARDWARE_NTP_REQUEST_BUFFER_SIZE = ETH_HEADER_SIZE + IPV6_HEADER_SIZE + 60 + UDP_HEADER_SIZE + NTP_MAX_PACKET_SIZE;
static constexpr uint32_t HARDWARE_NTP_TRANSMIT_RETRY_COUNT = 3;
static constexpr uint32_t HARDWARE_NTP_TRANSMIT_RETRY_DELAY_US = 10;
static constexpr uint32_t ETHERNET_RECOVERY_DELAY_MS = 100;
static constexpr uint32_t ETHERNET_RECOVERY_STOP_TIMEOUT_MS = 1000;
static constexpr uint32_t ETHERNET_RECOVERY_IP_TIMEOUT_MS = 10000;
// Preempt the default EMAC RX task (15) without blocking its receive callback.
static constexpr uint32_t ETHERNET_RECOVERY_TASK_PRIORITY = 16;
static constexpr uint32_t ETHERNET_RECOVERY_TIMEOUT_MS = 45000;
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

// One's complement sum (not folded, not inverted) of a byte range, added to sum
static uint32_t ones_complement_add(uint32_t sum, const uint8_t *data, size_t length)
{
    while (length >= 2)
    {
        sum += static_cast<uint16_t>((data[0] << 8) | data[1]);
        data += 2;
        length -= 2;
    }
    if (length != 0)
        sum += static_cast<uint16_t>(data[0] << 8);
    return sum;
}

static uint16_t ones_complement_fold(uint32_t sum)
{
    while ((sum >> 16) != 0)
        sum = (sum & 0xFFFFU) + (sum >> 16);
    return static_cast<uint16_t>(sum);
}

static uint16_t internet_checksum(const uint8_t *data, size_t length)
{
    return static_cast<uint16_t>(~ones_complement_fold(ones_complement_add(0, data, length)));
}

// One's complement sum of the UDP pseudo header (source, destination, protocol, UDP length)
static uint32_t udp_pseudo_header_sum(const uint8_t *source, const uint8_t *destination, size_t address_length, size_t udp_length)
{
    uint32_t sum = ones_complement_add(0, source, address_length);
    sum = ones_complement_add(sum, destination, address_length);
    sum += IPPROTO_UDP;
    sum += static_cast<uint32_t>(udp_length & 0xFFFFU);
    sum += static_cast<uint32_t>(udp_length >> 16);
    return sum;
}

// true if the UDP checksum of a received datagram is correct (for IPv4 a checksum of 0 means "not used")
static bool udp_checksum_is_valid(const uint8_t *source, const uint8_t *destination, size_t address_length,
                                  const uint8_t *udp, size_t udp_length)
{
    if (address_length == 4 && udp[6] == 0 && udp[7] == 0)
        return true;
    const uint32_t sum = ones_complement_add(udp_pseudo_header_sum(source, destination, address_length, udp_length), udp, udp_length);
    return ones_complement_fold(sum) == 0xFFFFU;
}

static uint64_t ntp64_from_hardware_timestamp(const eth_mac_time_t &timestamp)
{
    if (timestamp.nanoseconds >= 1000000000U)
        return 0;

#if PTP_FREE_RUNNING
    // Remove the estimated offset of the disciplined PTP clock (see ptp_servo_update)
    if (s_ptp_comp_valid.load(std::memory_order_acquire))
    {
        const int64_t ptp_ns = static_cast<int64_t>(timestamp.seconds) * 1000000000LL +
                               static_cast<int64_t>(timestamp.nanoseconds);
        int64_t offset_ns = 0;
        if (ptp_map_offset_at(ptp_ns, offset_ns))
        {
            const int64_t unix_ns = ptp_ns - offset_ns;
            if (unix_ns > 0)
                return ntp64_from_unix_ns(unix_ns);
        }
    }
#else
    // Map the PTP clock to true time exactly like the model clock (same oscillator as esp_timer):
    // remove the measured offset of the last write and the drift since the last PPS edge
    if (s_ptp_comp_valid.load(std::memory_order_acquire))
    {
        int64_t anchor_ns = 0;
        int64_t offset_ns = 0;
        if (!ptp_comp_read(anchor_ns, offset_ns))
            return 0;
        const int64_t ptp_ns = static_cast<int64_t>(timestamp.seconds) * 1000000000LL +
                               static_cast<int64_t>(timestamp.nanoseconds);
        const int64_t el = ptp_ns - offset_ns - anchor_ns; // time since the PPS edge in oscillator units
        if (el > -1000000000LL && el < 3600000000000LL)
        {
            const int64_t unix_ns = anchor_ns + el - (el * static_cast<int64_t>(s_clock_ppb.load(std::memory_order_relaxed))) / 1000000000LL;
            if (unix_ns > 0)
                return ntp64_from_unix_ns(unix_ns);
        }
    }
#endif
    return 0;
}

static bool timestamp_is_valid(const eth_mac_time_t *timestamp)
{
    return timestamp != nullptr && timestamp->nanoseconds < 1000000000U &&
           (timestamp->seconds != 0 || timestamp->nanoseconds != 0);
}

// Set to 0 to send NTP responses without requesting a hardware TX time stamp from the EMAC.
#ifndef NTP_HW_TX_TIMESTAMPS
#define NTP_HW_TX_TIMESTAMPS 1
#endif

static std::atomic<uint32_t> s_hw_tx_timestamp_failures{0};
static std::atomic<bool> s_hw_tx_timestamp_recovery_requested{false};

static void record_hw_tx_timestamp_result(bool valid)
{
    if (valid)
    {
        s_hw_tx_timestamp_failures.store(0, std::memory_order_relaxed);
        s_hw_tx_timestamp_recovery_requested.store(false, std::memory_order_release);
    }
    else if (s_hw_tx_timestamp_failures.fetch_add(1, std::memory_order_relaxed) + 1 >= 5 &&
             !s_hw_tx_timestamp_recovery_requested.exchange(true, std::memory_order_acq_rel))
    {
        TIMING_LOGW(TAG, "Repeated hardware TX timestamp failures; Ethernet recovery requested");
    }
}

// Transmits a complete Ethernet frame. If possible the EMAC returns the hardware TX time stamp, converted
// to NTP format with the same drift compensation as T2. *tx_time_ntp64 stays 0 if no usable time stamp exists.
static esp_err_t transmit_with_hw_timestamp(esp_eth_handle_t handle, uint8_t *frame, size_t length, uint64_t *tx_time_ntp64)
{
    if (tx_time_ntp64 != nullptr)
        *tx_time_ntp64 = 0;

#if NTP_HW_TX_TIMESTAMPS
    if (!s_ptp_clock_ready.load(std::memory_order_acquire))
        return ESP_ERR_INVALID_STATE;

    eth_mac_time_t tx_timestamp{};
    esp_err_t result = esp_eth_transmit_ctrl_vargs(handle, &tx_timestamp, 2, frame, static_cast<uint32_t>(length));

    if (result == ESP_OK)
    {
        bool usable_timestamp = false;
        if (timestamp_is_valid(&tx_timestamp))
        {
            struct timespec now_ptp{};
            if (clock_gettime(CLOCK_PTP_SYSTEM, &now_ptp) == 0)
            {
                const int64_t now_ns = static_cast<int64_t>(now_ptp.tv_sec) * 1000000000LL + now_ptp.tv_nsec;
                const int64_t tx_ns = static_cast<int64_t>(tx_timestamp.seconds) * 1000000000LL + tx_timestamp.nanoseconds;
                const int64_t age_ns = now_ns - tx_ns;
                if (age_ns > -2000000LL && age_ns < 100000000LL)
                {
                    usable_timestamp = true;
                    if (tx_time_ntp64 != nullptr)
                        *tx_time_ntp64 = ntp64_from_hardware_timestamp(tx_timestamp);
                }
            }
        }
        record_hw_tx_timestamp_result(usable_timestamp);
        return result;
    }

    if (result == ESP_ERR_NO_MEM)
        return result; // the caller retries

    record_hw_tx_timestamp_result(false);
    return result;
#else
    return esp_eth_transmit(handle, frame, length);
#endif
}

// ---- Transmit time compensation for basic mode responses ----
// In basic mode T3 has to be written into the packet before it is sent, so it is read from the model clock just
// before the transmit call. The real transmit instant is later by the time needed for the rest of the packet
// (checksum, MAC with authentication) and the EMAC DMA. The hardware TX time stamp of every response measures this
// delay; its running mean is added to the T3 of later basic mode responses. Separate estimates are kept for
// IPv4 / IPv6 and with / without authentication, because these paths take different times.
// Interleaved responses are not affected (their T3 is the hardware TX time stamp itself).
// 1 = compensate, 0 = T3 is the clock reading before the transmit call (previous behaviour)
#define NTP_BASIC_T3_TX_COMPENSATION 1
#if NTP_BASIC_T3_TX_COMPENSATION && NTP_HW_TX_TIMESTAMPS
struct tx_delay_estimate_t
{
    std::atomic<int32_t> delay_ns{0}; // running mean of hardware TX time - software T3
    std::atomic<uint32_t> samples{0};
};
static tx_delay_estimate_t s_tx_delay[2][2];         // [0 = IPv4, 1 = IPv6][0 = unauthenticated, 1 = authenticated]
static constexpr uint32_t tx_delay_min_samples = 16; // compensate only after this many samples
static constexpr int32_t tx_delay_max_ns = 500000;   // plausible range of a sample
static constexpr int32_t tx_delay_ewma_shift = 5;    // running mean with weight 1/32

// Adds the estimated transmit delay to a software T3 (NTP 32.32 format)
static uint64_t compensate_basic_t3(uint64_t t3, bool ipv6, bool authenticated)
{
    const tx_delay_estimate_t &estimate = s_tx_delay[ipv6 ? 1 : 0][authenticated ? 1 : 0];
    if (estimate.samples.load(std::memory_order_relaxed) < tx_delay_min_samples)
        return t3;
    const int64_t delay_ns = estimate.delay_ns.load(std::memory_order_relaxed);
    return t3 + static_cast<uint64_t>((delay_ns << 32) / 1000000000LL);
}

// Called by the hardware NTP task (only writer) with the uncompensated software T3 and the hardware TX time stamp
static void update_tx_delay_estimate(bool ipv6, bool authenticated, uint64_t software_t3, uint64_t hardware_t3)
{
    if (software_t3 == 0 || hardware_t3 == 0)
        return;
    const int64_t difference = static_cast<int64_t>(hardware_t3 - software_t3); // 32.32 fixed point
    if (difference < 0 || difference > 4294967LL)                               // 0 .. 1 ms (also avoids an overflow below)
        return;
    const int64_t sample_ns = (difference * 1000000000LL) >> 32;
    if (sample_ns < 0 || sample_ns > tx_delay_max_ns)
        return;

    tx_delay_estimate_t &estimate = s_tx_delay[ipv6 ? 1 : 0][authenticated ? 1 : 0];
    const uint32_t n = estimate.samples.load(std::memory_order_relaxed);
    const int32_t current = estimate.delay_ns.load(std::memory_order_relaxed);
    int32_t updated;
    if (n < (1u << tx_delay_ewma_shift))
        updated = static_cast<int32_t>((static_cast<int64_t>(current) * n + sample_ns) / (n + 1)); // plain mean at the start
    else
        updated = static_cast<int32_t>(current + ((sample_ns - current) >> tx_delay_ewma_shift));
    estimate.delay_ns.store(updated, std::memory_order_relaxed);
    if (n < UINT32_MAX)
        estimate.samples.store(n + 1, std::memory_order_relaxed);
}
#else
static inline uint64_t compensate_basic_t3(uint64_t t3, bool, bool) { return t3; }
static inline void update_tx_delay_estimate(bool, bool, uint64_t, uint64_t) {}
#endif

static bool is_ipv4_ntp_request(const uint8_t *frame, uint32_t length, size_t *ip_offset, size_t *udp_offset)
{
    if (frame == nullptr || length < ETH_HEADER_SIZE + IPV4_HEADER_SIZE + UDP_HEADER_SIZE + NTP_PACKET_SIZE || frame[12] != 0x08 || frame[13] != 0x00)
        return false;

    const size_t ipv4_offset = ETH_HEADER_SIZE;
    const uint8_t version_ihl = frame[ipv4_offset];
    const size_t header_length = static_cast<size_t>(version_ihl & 0x0F) * 4;
    if ((version_ihl >> 4) != 4 || header_length < IPV4_HEADER_SIZE || length < ETH_HEADER_SIZE + header_length + UDP_HEADER_SIZE + NTP_PACKET_SIZE)
        return false;
    if (frame[ipv4_offset + 9] != IPPROTO_UDP)
        return false;

    // Fragments (MF set or fragment offset != 0) are left to lwIP, which reassembles them
    const uint16_t fragment_field = static_cast<uint16_t>((frame[ipv4_offset + 6] << 8) | frame[ipv4_offset + 7]);
    if ((fragment_field & 0x3FFFU) != 0)
        return false;

    // The IP total length has to cover the UDP datagram and has to fit into the frame (Ethernet padding is ignored)
    const size_t total_length = static_cast<size_t>((frame[ipv4_offset + 2] << 8) | frame[ipv4_offset + 3]);
    if (total_length < header_length + UDP_HEADER_SIZE + NTP_PACKET_SIZE || ipv4_offset + total_length > length)
        return false;

    // Header checksum (a correct header sums up to 0xFFFF)
    if (internet_checksum(frame + ipv4_offset, header_length) != 0)
        return false;

    const size_t udp_header_offset = ipv4_offset + header_length;
    const uint16_t udp_length = static_cast<uint16_t>((frame[udp_header_offset + 4] << 8) | frame[udp_header_offset + 5]);
    const uint16_t destination_port = static_cast<uint16_t>((frame[udp_header_offset + 2] << 8) | frame[udp_header_offset + 3]);
    if (destination_port != NTP_PORT || udp_length < UDP_HEADER_SIZE + NTP_PACKET_SIZE ||
        udp_length > UDP_HEADER_SIZE + NTP_MAX_PACKET_SIZE || udp_length != total_length - header_length)
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
    if (frame == nullptr || length < ETH_HEADER_SIZE + IPV6_HEADER_SIZE + UDP_HEADER_SIZE + NTP_PACKET_SIZE || frame[12] != 0x86 || frame[13] != 0xDD)
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
    if (destination_port != NTP_PORT || udp_length < UDP_HEADER_SIZE + NTP_PACKET_SIZE ||
        udp_length > UDP_HEADER_SIZE + NTP_MAX_PACKET_SIZE ||
        offset + udp_length != payload_end)
        return false;

    *ip_offset = ipv6_offset;
    *udp_offset = offset;
    return true;
}

// Additional checks for the raw (hardware time stamped) NTP path. The reply is built by swapping the addresses of
// the request, so only well-formed unicast requests to this device are answered there. Everything else is handed
// to lwIP, which applies its own rules (broadcast requests, fragments, foreign addresses, ...).
static bool hardware_ntp_frame_is_acceptable(const uint8_t *frame, bool ipv4, size_t ip_offset, size_t udp_offset)
{
    // Ethernet: destination must be our unicast address (no broadcast / multicast), the source must be unicast
    // (the reply is sent to it)
    if ((frame[0] & 0x01) != 0 || (frame[6] & 0x01) != 0)
        return false;

    // UDP: source port 0 cannot be answered
    if (frame[udp_offset] == 0 && frame[udp_offset + 1] == 0)
        return false;

    const size_t udp_length = static_cast<size_t>((frame[udp_offset + 4] << 8) | frame[udp_offset + 5]);

    if (ipv4)
    {
        const uint8_t *source = frame + ip_offset + 12;
        const uint8_t *destination = frame + ip_offset + 16;

        // Destination must be the assigned IPv4 address (the reply uses it as its source address)
        uint32_t destination_be = 0;
        memcpy(&destination_be, destination, sizeof(destination_be));
        const uint32_t own_be = s_own_ipv4_address_be.load(std::memory_order_relaxed);
        if (own_be == 0 || destination_be != own_be)
            return false;

        // Source must be a plausible unicast address: not 0.0.0.0/8, 127.0.0.0/8, multicast, reserved or broadcast,
        // and not our own address (that cannot arrive from the wire)
        if (source[0] == 0 || source[0] == 127 || source[0] >= 224)
            return false;
        uint32_t source_be = 0;
        memcpy(&source_be, source, sizeof(source_be));
        if (source_be == own_be || source_be == s_own_ipv4_broadcast_be.load(std::memory_order_relaxed))
            return false;

        return udp_checksum_is_valid(source, destination, 4, frame + udp_offset, udp_length);
    }

    const uint8_t *source = frame + ip_offset + 8;
    const uint8_t *destination = frame + ip_offset + 24;
    static constexpr uint8_t unspecified[16] = {};
    static constexpr uint8_t loopback[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};

    // Destination must not be multicast (ff00::/8): the reply would use it as its source address
    if (destination[0] == 0xFF)
        return false;
    const uint32_t sequence = s_own_ipv6_sequence.load(std::memory_order_acquire);
    if ((sequence & 1U) != 0)
        return false;
    const uint32_t count = s_own_ipv6_count.load(std::memory_order_acquire);
    bool own_destination = false;
    for (uint32_t index = 0; index < count; ++index)
    {
        uint32_t address[4];
        for (size_t word = 0; word < 4; ++word)
            address[word] = s_own_ipv6_addresses[index][word].load(std::memory_order_relaxed);
        own_destination = own_destination || memcmp(destination, address, sizeof(address)) == 0;
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    if (!own_destination || sequence != s_own_ipv6_sequence.load(std::memory_order_relaxed))
        return false;
    // Source must be unicast: not multicast, unspecified or loopback
    if (source[0] == 0xFF || memcmp(source, unspecified, 16) == 0 || memcmp(source, loopback, 16) == 0)
        return false;

    // The UDP checksum is mandatory for IPv6
    if (frame[udp_offset + 6] == 0 && frame[udp_offset + 7] == 0)
        return false;
    return udp_checksum_is_valid(source, destination, 16, frame + udp_offset, udp_length);
}

static bool is_internal_ntp_ipv4_address(const struct in_addr &address)
{
    if (address.s_addr == htonl(INADDR_LOOPBACK))
        return true;

    const network_address_snapshot_t addresses = get_network_address_snapshot();
    struct in_addr assigned_address{};
    return addresses.ipv4[0] != '\0' && inet_pton(AF_INET, addresses.ipv4, &assigned_address) == 1 &&
           address.s_addr == assigned_address.s_addr;
}

static bool is_internal_ntp_ipv6_address(const struct in6_addr &address)
{
    struct in6_addr loopback_address{};
    if (inet_pton(AF_INET6, "::1", &loopback_address) == 1 &&
        memcmp(&address, &loopback_address, sizeof(address)) == 0)
        return true;

    const network_address_snapshot_t addresses = get_network_address_snapshot();
    struct in6_addr assigned_address{};
    return addresses.ipv6[0] != '\0' && inet_pton(AF_INET6, addresses.ipv6, &assigned_address) == 1 &&
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

// UDP checksum for an outgoing IPv6 datagram (the checksum field in udp must be 0)
static uint16_t ipv6_udp_checksum(const uint8_t *source, const uint8_t *destination, const uint8_t *udp, size_t udp_length)
{
    const uint32_t sum = ones_complement_add(udp_pseudo_header_sum(source, destination, 16, udp_length), udp, udp_length);
    const uint16_t checksum = static_cast<uint16_t>(~ones_complement_fold(sum));
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

#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
    const size_t request_length = static_cast<size_t>((frame[udp_offset + 4] << 8) | frame[udp_offset + 5]) - UDP_HEADER_SIZE;
    uint32_t authentication_key_id = 0;
    const ntp_auth_result_t authentication_result = ntp_auth_verify_request(request, request_length, version, &authentication_key_id);
    if (authentication_result != ntp_auth_result_t::unauthenticated && authentication_result != ntp_auth_result_t::valid)
    {
#if MQTT_ENABLED
        s_ntp_invalid_requests.fetch_add(1, std::memory_order_relaxed);
        if (version == 4)
            s_ntp_authenticated_invalid_requests.fetch_add(1, std::memory_order_relaxed);
        else
            s_ntp_unauthenticated_invalid_requests.fetch_add(1, std::memory_order_relaxed);
        s_ntp_requests_this_second.fetch_add(1, std::memory_order_relaxed);
#endif
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }
    const bool authenticated_request = authentication_result == ntp_auth_result_t::valid;
#else
    const bool authenticated_request = false;
#endif

    const uint32_t client_ip = (static_cast<uint32_t>(frame[ip_offset + 12]) << 24) |
                               (static_cast<uint32_t>(frame[ip_offset + 13]) << 16) |
                               (static_cast<uint32_t>(frame[ip_offset + 14]) << 8) |
                               frame[ip_offset + 15];
    const uint16_t client_port = static_cast<uint16_t>((frame[udp_offset] << 8) | frame[udp_offset + 1]);
    ntp_client_record_t client_record{};
    if (!ntp_cache_find(client_ip,
                        read_ntp_timestamp(request, 32) != read_ntp_timestamp(request, 40) ? read_ntp_timestamp(request, 24) : 0,
                        &client_record))
    {
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }

    const uint64_t receive_time = ntp64_from_hardware_timestamp(receive_timestamp);
    if (receive_time == 0)
    {
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }
#if MQTT_ENABLED
    struct sockaddr_storage source_address{};
    auto *source_ipv4_address = reinterpret_cast<struct sockaddr_in *>(&source_address);
    source_ipv4_address->sin_family = AF_INET;
    source_ipv4_address->sin_port = htons(client_port);
    memcpy(&source_ipv4_address->sin_addr, frame + ip_offset + 12, sizeof(source_ipv4_address->sin_addr));
    s_ntp_valid_requests.fetch_add(1, std::memory_order_relaxed);
    if (authenticated_request)
        s_ntp_authenticated_valid_requests.fetch_add(1, std::memory_order_relaxed);
    else
        s_ntp_unauthenticated_valid_requests.fetch_add(1, std::memory_order_relaxed);
    mqtt_enqueue_ntp_request(source_address);
#endif
    uint8_t reply[NTP_MAX_PACKET_SIZE] = {};
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
    memcpy(udp + UDP_HEADER_SIZE, reply, NTP_PACKET_SIZE);

    uint64_t own_transmit_time = 0; // T3 as written into a basic mode response (compensated)
    uint64_t raw_transmit_time = 0; // clock reading before the transmit call
    uint64_t hardware_transmit_time = 0;
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
                    raw_transmit_time = get_disciplined_time_in_ntp64_format();
                    if (raw_transmit_time == 0)
                        break;
                    own_transmit_time = compensate_basic_t3(raw_transmit_time, false, authenticated_request);
                    if (!interleaved_reply)
                    {
                        write_ntp_timestamp(reply, 40, own_transmit_time);
                    }

                    size_t reply_length = NTP_PACKET_SIZE;
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
                    if (authenticated_request && !ntp_auth_append_response(reply, sizeof(reply), &reply_length, authentication_key_id))
                        break;
#endif
                    ip[2] = static_cast<uint8_t>((IPV4_HEADER_SIZE + UDP_HEADER_SIZE + reply_length) >> 8);
                    ip[3] = static_cast<uint8_t>(IPV4_HEADER_SIZE + UDP_HEADER_SIZE + reply_length);
                    ip[10] = 0;
                    ip[11] = 0;
                    const uint16_t refreshed_ip_checksum = internet_checksum(ip, IPV4_HEADER_SIZE);
                    ip[10] = static_cast<uint8_t>(refreshed_ip_checksum >> 8);
                    ip[11] = static_cast<uint8_t>(refreshed_ip_checksum);
                    udp[4] = static_cast<uint8_t>((UDP_HEADER_SIZE + reply_length) >> 8);
                    udp[5] = static_cast<uint8_t>(UDP_HEADER_SIZE + reply_length);
                    memcpy(udp + UDP_HEADER_SIZE, reply, reply_length);

                    result = transmit_with_hw_timestamp(handle, response_frame, ETH_HEADER_SIZE + IPV4_HEADER_SIZE + UDP_HEADER_SIZE + reply_length, &hardware_transmit_time);
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
        const uint64_t cached_transmit_time = NTP_HW_TX_TIMESTAMPS ? hardware_transmit_time : own_transmit_time;
        if (cached_transmit_time != 0)
            ntp_cache_update(client_ip, receive_time, cached_transmit_time);
        update_tx_delay_estimate(false, authenticated_request, raw_transmit_time, hardware_transmit_time);
        const int64_t response_us = esp_timer_get_time();
        if (cached_transmit_time != 0)
            s_last_hardware_ntp_response_us.store(response_us, std::memory_order_release);
        s_last_ntp_response_us.store(response_us, std::memory_order_relaxed);
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

#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
    const size_t request_length = static_cast<size_t>((frame[udp_offset + 4] << 8) | frame[udp_offset + 5]) - UDP_HEADER_SIZE;
    uint32_t authentication_key_id = 0;
    const ntp_auth_result_t authentication_result = ntp_auth_verify_request(request, request_length, version, &authentication_key_id);
    if (authentication_result != ntp_auth_result_t::unauthenticated && authentication_result != ntp_auth_result_t::valid)
    {
#if MQTT_ENABLED
        s_ntp_invalid_requests.fetch_add(1, std::memory_order_relaxed);
        if (version == 4)
            s_ntp_authenticated_invalid_requests.fetch_add(1, std::memory_order_relaxed);
        else
            s_ntp_unauthenticated_invalid_requests.fetch_add(1, std::memory_order_relaxed);
        s_ntp_requests_this_second.fetch_add(1, std::memory_order_relaxed);
#endif
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }
    const bool authenticated_request = authentication_result == ntp_auth_result_t::valid;
#else
    const bool authenticated_request = false;
#endif

    struct in6_addr client_ip{};
    memcpy(&client_ip, frame + ip_offset + 8, sizeof(client_ip));
    const uint16_t client_port = static_cast<uint16_t>((frame[udp_offset] << 8) | frame[udp_offset + 1]);
    ntp_client_record_ipv6_t client_record{};
    if (!ntp_cache_find_ipv6(&client_ip,
                             read_ntp_timestamp(request, 32) != read_ntp_timestamp(request, 40) ? read_ntp_timestamp(request, 24) : 0,
                             &client_record))
    {
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }

    const uint64_t receive_time = ntp64_from_hardware_timestamp(receive_timestamp);
    if (receive_time == 0)
    {
        release_hardware_ntp_request_buffer(frame);
        return ESP_OK;
    }
#if MQTT_ENABLED
    struct sockaddr_storage source_address{};
    auto *source_ipv6_address = reinterpret_cast<struct sockaddr_in6 *>(&source_address);
    source_ipv6_address->sin6_family = AF_INET6;
    source_ipv6_address->sin6_port = htons(client_port);
    source_ipv6_address->sin6_scope_id = esp_netif_get_netif_impl_index(static_cast<esp_netif_t *>(netif));
    source_ipv6_address->sin6_addr = client_ip;
    s_ntp_valid_requests.fetch_add(1, std::memory_order_relaxed);
    if (authenticated_request)
        s_ntp_authenticated_valid_requests.fetch_add(1, std::memory_order_relaxed);
    else
        s_ntp_unauthenticated_valid_requests.fetch_add(1, std::memory_order_relaxed);
    mqtt_enqueue_ntp_request(source_address);
#endif
    uint8_t reply[NTP_MAX_PACKET_SIZE] = {};
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
    memcpy(udp + UDP_HEADER_SIZE, reply, NTP_PACKET_SIZE);
    const uint16_t checksum = ipv6_udp_checksum(ip + 8, ip + 24, udp, UDP_HEADER_SIZE + NTP_PACKET_SIZE);
    udp[6] = static_cast<uint8_t>(checksum >> 8);
    udp[7] = static_cast<uint8_t>(checksum);

    uint64_t own_transmit_time = 0; // T3 as written into a basic mode response (compensated)
    uint64_t raw_transmit_time = 0; // clock reading before the transmit call
    uint64_t hardware_transmit_time = 0;
    esp_err_t result = ESP_ERR_INVALID_STATE;
    if (s_hardware_ntp_accepting.load(std::memory_order_acquire) && s_hardware_ntp_transmit_mutex != nullptr &&
        xSemaphoreTake(s_hardware_ntp_transmit_mutex, portMAX_DELAY) == pdTRUE)
    {
        if (s_hardware_ntp_accepting.load(std::memory_order_acquire))
        {
            for (uint32_t attempt = 0; attempt < HARDWARE_NTP_TRANSMIT_RETRY_COUNT; ++attempt)
            {
                raw_transmit_time = get_disciplined_time_in_ntp64_format();
                if (raw_transmit_time == 0)
                    break;
                own_transmit_time = compensate_basic_t3(raw_transmit_time, true, authenticated_request);
                if (!interleaved_reply)
                {
                    write_ntp_timestamp(reply, 40, own_transmit_time);
                }
                size_t reply_length = NTP_PACKET_SIZE;
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
                if (authenticated_request && !ntp_auth_append_response(reply, sizeof(reply), &reply_length, authentication_key_id))
                    break;
#endif
                ip[4] = static_cast<uint8_t>((UDP_HEADER_SIZE + reply_length) >> 8);
                ip[5] = static_cast<uint8_t>(UDP_HEADER_SIZE + reply_length);
                udp[4] = static_cast<uint8_t>((UDP_HEADER_SIZE + reply_length) >> 8);
                udp[5] = static_cast<uint8_t>(UDP_HEADER_SIZE + reply_length);
                memcpy(udp + UDP_HEADER_SIZE, reply, reply_length);
                udp[6] = 0;
                udp[7] = 0;
                const uint16_t refreshed_checksum = ipv6_udp_checksum(ip + 8, ip + 24, udp, UDP_HEADER_SIZE + reply_length);
                udp[6] = static_cast<uint8_t>(refreshed_checksum >> 8);
                udp[7] = static_cast<uint8_t>(refreshed_checksum);
                result = transmit_with_hw_timestamp(handle, response_frame, ETH_HEADER_SIZE + IPV6_HEADER_SIZE + UDP_HEADER_SIZE + reply_length, &hardware_transmit_time);
                if (result != ESP_ERR_NO_MEM)
                    break;
                esp_rom_delay_us(HARDWARE_NTP_TRANSMIT_RETRY_DELAY_US);
            }
        }
        xSemaphoreGive(s_hardware_ntp_transmit_mutex);
    }
    if (result == ESP_OK)
    {
        const uint64_t cached_transmit_time = NTP_HW_TX_TIMESTAMPS ? hardware_transmit_time : own_transmit_time;
        if (cached_transmit_time != 0)
            ntp_cache_update_ipv6(&client_ip, receive_time, cached_transmit_time);
        update_tx_delay_estimate(true, authenticated_request, raw_transmit_time, hardware_transmit_time);
        const int64_t response_us = esp_timer_get_time();
        if (cached_transmit_time != 0)
            s_last_hardware_ntp_response_us.store(response_us, std::memory_order_release);
        s_last_ntp_response_us.store(response_us, std::memory_order_relaxed);
#if MQTT_ENABLED
        s_ntp_responses.fetch_add(1, std::memory_order_relaxed);
#endif
    }

    release_hardware_ntp_request_buffer(frame);
    return result;
}

static esp_err_t ntp_ethernet_input(esp_eth_handle_t handle, uint8_t *frame, uint32_t length, void *netif, void *info)
{
    s_ethernet_rx_frames.fetch_add(1, std::memory_order_relaxed);
    size_t ip_offset = 0;
    size_t udp_offset = 0;
    const eth_mac_time_t *rx_timestamp = static_cast<const eth_mac_time_t *>(info);
    const bool ipv4_request = is_ipv4_ntp_request(frame, length, &ip_offset, &udp_offset);
    const bool ipv6_request = !ipv4_request && is_ipv6_ntp_request(frame, length, &ip_offset, &udp_offset);
    if ((!ipv4_request && !ipv6_request) || (ipv4_request && !IPV4_ENABLED) || (ipv6_request && !IPV6_ENABLED))
        return esp_netif_receive(static_cast<esp_netif_t *>(netif), frame, length, nullptr);
    const uint8_t *request = frame + udp_offset + UDP_HEADER_SIZE;
    const uint8_t version = (request[0] >> 3) & 0x07;
    const uint8_t mode = request[0] & 0x07;
    if (version < 3 || version > 4 || mode != 3)
    {
        return esp_netif_receive(static_cast<esp_netif_t *>(netif), frame, length, nullptr);
    }

    // Broadcast / multicast, foreign destination addresses, invalid sources and bad checksums: leave it to lwIP
    if (!hardware_ntp_frame_is_acceptable(frame, ipv4_request, ip_offset, udp_offset))
        return esp_netif_receive(static_cast<esp_netif_t *>(netif), frame, length, nullptr);

    // Requests of this device to itself (loopback, own address) never arrive through the EMAC, they take the lwIP
    // loopback path to ntp_server_task. Therefore no address lookup is needed here (this runs for every NTP frame).
    if (!s_ntp_external_responses_enabled.load(std::memory_order_acquire))
    {
        free(frame);
        return ESP_OK;
    }

    if (!timestamp_is_valid(rx_timestamp))
    {
        record_ntp_input(esp_timer_get_time());
        free(frame);
        return ESP_OK;
    }

#if MQTT_ENABLED
    s_ntp_requests_this_second.fetch_add(1, std::memory_order_relaxed);
#endif
    if (!admit_ntp_request())
    {
        free(frame);
        return ESP_OK;
    }
    const eth_mac_time_t receive_timestamp = *rx_timestamp;
    if (!s_hardware_ntp_accepting.load(std::memory_order_acquire) || s_hardware_ntp_request_queue == nullptr ||
        s_hardware_ntp_request_buffer_queue == nullptr)
    {
        free(frame);
    }
    else
    {
        uint8_t *request_buffer = nullptr;
        if (length > HARDWARE_NTP_REQUEST_BUFFER_SIZE)
        {
            free(frame);
            return ESP_OK;
        }
        if (xQueueReceive(s_hardware_ntp_request_buffer_queue, &request_buffer, 0) != pdTRUE)
        {
            record_ntp_load_shedding(esp_timer_get_time());
            free(frame);
            return ESP_OK;
        }
        memcpy(request_buffer, frame, length);
        free(frame);

        const hardware_ntp_request_t ntp_request{handle, request_buffer, length, static_cast<esp_netif_t *>(netif), receive_timestamp};
        if (xQueueSend(s_hardware_ntp_request_queue, &ntp_request, 0) != pdTRUE)
        {
            record_ntp_load_shedding(esp_timer_get_time());
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
            if (ntp_request.frame[12] == 0x86 && ntp_request.frame[13] == 0xDD && IPV6_ENABLED)
                process_hardware_ipv6_ntp_request(ntp_request.handle, ntp_request.frame, ntp_request.length,
                                                  ntp_request.netif, &ntp_request.rx_timestamp);
            else if (IPV4_ENABLED)
                process_hardware_ntp_request(ntp_request.handle, ntp_request.frame, ntp_request.length,
                                             ntp_request.netif, &ntp_request.rx_timestamp);
            else
                release_hardware_ntp_request_buffer(ntp_request.frame);
#if CALCULATE_STACK_SIZES_ENABLED
            report_current_task_stack_usage(Hardware_NTP_Server);
#endif
        }
    }
}

enum class ethernet_recovery_result_t : uint8_t
{
    recovered,
    retry_later,
    link_unavailable,
    no_ip,
    receive_unavailable,
    driver_failed,
    timestamp_failed
};

enum class physical_link_status_t : uint8_t
{
    up,
    down,
    unknown
};

static physical_link_status_t physical_ethernet_link_status()
{
    uint32_t status = 0;
    esp_eth_phy_reg_rw_data_t reg{1, &status}; // IEEE 802.3 Basic Status Register
    // Link status is latch-low: the second read reports the current physical link.
    if (s_eth_handle == nullptr ||
        esp_eth_ioctl(s_eth_handle, ETH_CMD_READ_PHY_REG, &reg) != ESP_OK ||
        esp_eth_ioctl(s_eth_handle, ETH_CMD_READ_PHY_REG, &reg) != ESP_OK ||
        status == 0 || status == 0xFFFF)
    {
        ESP_LOGE(TAG, "Unable to read Ethernet PHY link status");
        return physical_link_status_t::unknown;
    }
    return (status & (1U << 2)) != 0 ? physical_link_status_t::up : physical_link_status_t::down;
}

static ethernet_recovery_result_t restart_ethernet_transport()
{
    const esp_eth_handle_t handle = s_eth_handle;
    if (handle == nullptr || s_hardware_ntp_transmit_mutex == nullptr ||
        xSemaphoreTake(s_hardware_ntp_transmit_mutex, pdMS_TO_TICKS(ETHERNET_RECOVERY_STOP_TIMEOUT_MS)) != pdTRUE)
        return ethernet_recovery_result_t::retry_later;

    s_ethernet_recovery_in_progress.store(true, std::memory_order_release);
    s_hardware_ntp_accepting.store(false, std::memory_order_release);
    esp_err_t result = esp_eth_stop(handle);
    xSemaphoreGive(s_hardware_ntp_transmit_mutex);
    if (result != ESP_OK)
    {
        s_hardware_ntp_accepting.store(true, std::memory_order_release);
        s_ethernet_recovery_in_progress.store(false, std::memory_order_release);
        return ethernet_recovery_result_t::driver_failed;
    }

    xEventGroupClearBits(s_net_event_group, ETH_CONNECTED_BIT | ETH_GOT_IP_BIT | ETH_GOT_IP6_BIT | ETH_GOT_ROUTABLE_IP6_BIT);
    vTaskDelay(pdMS_TO_TICKS(ETHERNET_RECOVERY_DELAY_MS));
    const uint32_t rx_before_recovery = s_ethernet_rx_frames.load(std::memory_order_relaxed);
    result = esp_eth_start(handle);
    if (result != ESP_OK)
    {
        s_ethernet_recovery_in_progress.store(false, std::memory_order_release);
        return ethernet_recovery_result_t::driver_failed;
    }

    esp_eth_mac_t *mac = nullptr;
    const eth_mac_ptp_config_t ptp_config = ETH_MAC_ESP_PTP_DEFAULT_CONFIG();
    if (xSemaphoreTake(s_time_mutex, pdMS_TO_TICKS(ETHERNET_RECOVERY_STOP_TIMEOUT_MS)) != pdTRUE)
    {
        s_ethernet_recovery_in_progress.store(false, std::memory_order_release);
        return ethernet_recovery_result_t::timestamp_failed;
    }
    s_ptp_clock_ready.store(false, std::memory_order_release);
    if (esp_eth_get_mac_instance(handle, &mac) != ESP_OK || mac == nullptr)
    {
        xSemaphoreGive(s_time_mutex);
        s_ethernet_recovery_in_progress.store(false, std::memory_order_release);
        return ethernet_recovery_result_t::timestamp_failed;
    }
    if (esp_eth_mac_ptp_enable(mac, &ptp_config) != ESP_OK ||
        esp_eth_mac_enable_ts4all(mac, true) != ESP_OK ||
        esp_eth_update_input_path_info(handle, ntp_ethernet_input, s_eth_netif) != ESP_OK)
    {
        xSemaphoreGive(s_time_mutex);
        s_ethernet_recovery_in_progress.store(false, std::memory_order_release);
        return ethernet_recovery_result_t::timestamp_failed;
    }

    struct timeval system_time{};
    gettimeofday(&system_time, nullptr);
    const struct timespec ptp_time{system_time.tv_sec, static_cast<long>(system_time.tv_usec) * 1000L};
    s_ptp_comp_valid.store(false, std::memory_order_release);
#if PTP_FREE_RUNNING
    // esp_eth_mac_ptp_enable() set the EMAC frequency back to its base value and the clock was set again
    s_ptp_mac = mac;
    s_ptp_addend_reset.store(true, std::memory_order_release);
#endif
    if (clock_settime(CLOCK_PTP_SYSTEM, &ptp_time) != 0)
    {
        xSemaphoreGive(s_time_mutex);
        s_ethernet_recovery_in_progress.store(false, std::memory_order_release);
        return ethernet_recovery_result_t::timestamp_failed;
    }
#if PTP_FREE_RUNNING
    s_ptp_epoch.fetch_add(1, std::memory_order_acq_rel);
    setup_ptp_pps_loopback(mac);
#endif

    eth_mac_time_t first_time{};
    eth_mac_time_t second_time{};
    const bool first_read_ok = esp_eth_mac_get_ptp_time(mac, &first_time) == ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(ETHERNET_RECOVERY_DELAY_MS));
    if (!first_read_ok || esp_eth_mac_get_ptp_time(mac, &second_time) != ESP_OK ||
        (second_time.seconds < first_time.seconds ||
         (second_time.seconds == first_time.seconds && second_time.nanoseconds <= first_time.nanoseconds)))
    {
        xSemaphoreGive(s_time_mutex);
        s_ethernet_recovery_in_progress.store(false, std::memory_order_release);
        return ethernet_recovery_result_t::timestamp_failed;
    }

    s_ptp_clock_ready.store(true, std::memory_order_release);
    xSemaphoreGive(s_time_mutex);
    record_hw_tx_timestamp_result(true);
    s_hardware_ntp_accepting.store(true, std::memory_order_release);
    if ((xEventGroupWaitBits(s_net_event_group, ETH_CONNECTED_BIT, pdFALSE, pdFALSE,
                             pdMS_TO_TICKS(ETHERNET_RECOVERY_IP_TIMEOUT_MS)) &
         ETH_CONNECTED_BIT) == 0)
    {
        s_ethernet_recovery_in_progress.store(false, std::memory_order_release);
        return physical_ethernet_link_status() == physical_link_status_t::down
                   ? ethernet_recovery_result_t::link_unavailable
                   : ethernet_recovery_result_t::driver_failed;
    }
    const EventBits_t needed_ip = PreferIPvX == 1   ? ETH_GOT_IP_BIT
                                  : PreferIPvX == 2 ? ETH_GOT_ROUTABLE_IP6_BIT
                                  : PreferIPvX == 4 && s_ipv4_ever_acquired.load(std::memory_order_relaxed)
                                      ? ETH_GOT_IP_BIT
                                      : ETH_GOT_IP_BIT | ETH_GOT_ROUTABLE_IP6_BIT;
    xEventGroupWaitBits(s_net_event_group, needed_ip, pdFALSE, pdFALSE,
                        pdMS_TO_TICKS(ETHERNET_RECOVERY_IP_TIMEOUT_MS));
    const int64_t receive_deadline_us = esp_timer_get_time() + ETHERNET_RECOVERY_IP_TIMEOUT_MS * 1000LL;
    while (s_ethernet_rx_frames.load(std::memory_order_relaxed) == rx_before_recovery &&
           s_ethernet_connected.load(std::memory_order_acquire) &&
           esp_timer_get_time() < receive_deadline_us)
        vTaskDelay(pdMS_TO_TICKS(100));

    const bool linked = s_ethernet_connected.load(std::memory_order_acquire);
    const bool received_frame = s_ethernet_rx_frames.load(std::memory_order_relaxed) != rx_before_recovery;
    s_ethernet_recovery_in_progress.store(false, std::memory_order_release);
    if (!linked)
    {
        return physical_ethernet_link_status() == physical_link_status_t::down
                   ? ethernet_recovery_result_t::link_unavailable
                   : ethernet_recovery_result_t::driver_failed;
    }
    if (!received_frame)
        return ethernet_recovery_result_t::receive_unavailable;
    return (xEventGroupGetBits(s_net_event_group) & needed_ip) != 0
               ? ethernet_recovery_result_t::recovered
               : ethernet_recovery_result_t::no_ip;
}

static void ethernet_recovery_worker_task(void *parameter)
{
    const TaskHandle_t supervisor = static_cast<TaskHandle_t>(parameter);
    const auto result = restart_ethernet_transport();
#if CALCULATE_STACK_SIZES_ENABLED
    report_current_task_stack_usage(Ethernet_Transport_Recovery);
#endif
    xTaskNotify(supervisor, static_cast<uint32_t>(result) + 1, eSetValueWithOverwrite);
    vTaskDelete(nullptr);
}

static ethernet_recovery_result_t recover_ethernet_with_timeout()
{
    uint32_t result = 0;
    xTaskNotifyWait(0, UINT32_MAX, &result, 0);
    if (xTaskCreatePinnedToCore(ethernet_recovery_worker_task, "eth_recover_work",
                                Ethernet_Transport_Recovery_Task_Stack_Size,
                                xTaskGetCurrentTaskHandle(), ETHERNET_RECOVERY_TASK_PRIORITY, nullptr, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Unable to start Ethernet recovery worker");
        return ethernet_recovery_result_t::driver_failed;
    }
    if (xTaskNotifyWait(0, UINT32_MAX, &result, pdMS_TO_TICKS(ETHERNET_RECOVERY_TIMEOUT_MS)) != pdTRUE)
    {
        controlled_restart("ethernet_recovery_timeout");
        return ethernet_recovery_result_t::driver_failed;
    }
    return static_cast<ethernet_recovery_result_t>(result - 1);
}

static void ethernet_transport_recovery_task(void *parameter)
{
    (void)parameter;
    uint32_t previous_input = 0;
    uint32_t previous_rx_frames = s_ethernet_rx_frames.load(std::memory_order_relaxed);
    int64_t last_rx_progress_us = esp_timer_get_time();
    int64_t address_missing_since_us = 0;
    int64_t link_missing_since_us = 0;
    int64_t last_recovery_attempt_us = 0;
    int64_t recovery_verification_us = 0;
    for (;;)
    {
        const int64_t now_us = esp_timer_get_time();
        const uint32_t input = s_ntp_input_count.load(std::memory_order_relaxed);
        const uint32_t interval_input = input - previous_input;
        previous_input = input;
        const uint32_t rx_frames = s_ethernet_rx_frames.load(std::memory_order_relaxed);
        if (rx_frames != previous_rx_frames)
            last_rx_progress_us = now_us;
        previous_rx_frames = rx_frames;

        ntp_transport_health_t health = ntp_transport_health_t::idle;
        if (!s_ethernet_connected.load(std::memory_order_acquire))
            health = ntp_transport_health_t::link_down;
        else if (!has_usable_network_address(xEventGroupGetBits(s_net_event_group)))
            health = ntp_transport_health_t::no_ip;
        else if (s_ntp_external_responses_enabled.load(std::memory_order_acquire) && interval_input > 0)
        {
            const int64_t last_reply = s_last_ntp_response_us.load(std::memory_order_relaxed);
            const int64_t first_input = s_first_ntp_input_us.load(std::memory_order_relaxed);
            if (first_input > 0 && now_us - (last_reply > 0 ? last_reply : first_input) >= NTP_TRANSPORT_STALL_TIMEOUT_US)
                health = ntp_transport_health_t::stalled;
            else
                health = ntp_transport_health_t::healthy;
        }
        const ntp_transport_health_t old_health = s_ntp_transport_health.exchange(health, std::memory_order_relaxed);
        const int64_t last_input_us = s_last_ntp_input_us.load(std::memory_order_relaxed);
        const int64_t last_shedding_us = s_last_load_shedding_us.load(std::memory_order_relaxed);
        const bool link_up = s_ethernet_connected.load(std::memory_order_acquire);
        const int64_t last_physical_link_down_us = s_last_physical_link_down_us.load(std::memory_order_relaxed);
        if (last_recovery_attempt_us > 0 && last_physical_link_down_us >= last_recovery_attempt_us &&
            physical_ethernet_link_status() == physical_link_status_t::down)
        {
            recovery_verification_us = 0;
        }
        const bool ipv4_missing = s_ipv4_ever_acquired.load(std::memory_order_relaxed) &&
                                  (xEventGroupGetBits(s_net_event_group) & ETH_GOT_IP_BIT) == 0;
        const bool address_missing = ipv4_missing || !has_usable_network_address(xEventGroupGetBits(s_net_event_group));
        if (!link_up || !address_missing)
            address_missing_since_us = 0;
        else if (address_missing_since_us == 0)
            address_missing_since_us = now_us;
        if (link_up)
            link_missing_since_us = 0;
        else if (link_missing_since_us == 0)
            link_missing_since_us = now_us;
        bool mqtt_unavailable = false;
#if MQTT_ENABLED
        mqtt_unavailable = s_mqtt_has_connected.load(std::memory_order_relaxed) &&
                           !s_mqtt_connected.load(std::memory_order_relaxed);
#endif
        if (recovery_verification_us > 0)
        {
            if (s_last_hardware_ntp_response_us.load(std::memory_order_acquire) > recovery_verification_us)
                recovery_verification_us = 0;
            else if (link_up &&
                     s_last_physical_link_down_us.load(std::memory_order_relaxed) < recovery_verification_us &&
                     last_input_us > recovery_verification_us &&
                     now_us - recovery_verification_us >= NTP_TRANSPORT_STALL_TIMEOUT_US &&
                     now_us - last_input_us < 2000000LL &&
                     has_usable_network_address(xEventGroupGetBits(s_net_event_group)))
            {
                controlled_restart("ethernet_recovery_timestamp_unverified");
            }
        }
        const bool timestamp_failure = s_hw_tx_timestamp_recovery_requested.load(std::memory_order_acquire);
        const bool confirmed_stall = health == ntp_transport_health_t::stalled &&
                                     old_health == ntp_transport_health_t::stalled && link_up &&
                                     now_us - last_input_us < 2000000LL;
        const bool silent_transport_failure = link_up && s_ntp_external_responses_enabled.load(std::memory_order_acquire) &&
                                              last_input_us > s_last_physical_link_down_us.load(std::memory_order_relaxed) &&
                                              now_us - last_input_us >= NTP_TRANSPORT_STALL_TIMEOUT_US &&
                                              now_us - last_rx_progress_us >= NTP_TRANSPORT_STALL_TIMEOUT_US &&
                                              (ipv4_missing || mqtt_unavailable ||
                                               (last_shedding_us > last_physical_link_down_us &&
                                                last_shedding_us >= last_input_us - 1500000LL));
        const bool address_failure = link_up && s_ntp_external_responses_enabled.load(std::memory_order_acquire) &&
                                     address_missing_since_us > 0 &&
                                     now_us - address_missing_since_us >= NTP_TRANSPORT_STALL_TIMEOUT_US;
        const bool link_state_failure = !link_up && s_ntp_external_responses_enabled.load(std::memory_order_acquire) &&
                                        link_missing_since_us > 0 &&
                                        now_us - link_missing_since_us >= NTP_TRANSPORT_STALL_TIMEOUT_US &&
                                        physical_ethernet_link_status() != physical_link_status_t::down;
        if ((link_up || link_state_failure) &&
            (confirmed_stall || silent_transport_failure || address_failure || link_state_failure ||
             timestamp_failure) &&
            (last_recovery_attempt_us == 0 ||
             now_us - last_recovery_attempt_us >= 60000000LL))
        {
            last_recovery_attempt_us = now_us;
            recovery_verification_us = 0;
            ESP_LOGW(TAG, "Recovering Ethernet: stalled=%d silent=%d no_ip=%d link_state=%d timestamp=%d",
                     confirmed_stall, silent_transport_failure, address_failure, link_state_failure,
                     timestamp_failure);
            ESP_LOGW(TAG, "Ethernet ingress snapshot: frames=%" PRIu32 " NTP=%" PRIu32 " pending=%u free_buffers=%u",
                     rx_frames, input,
                     s_hardware_ntp_request_queue != nullptr
                         ? static_cast<unsigned>(uxQueueMessagesWaiting(s_hardware_ntp_request_queue))
                         : 0U,
                     s_hardware_ntp_request_buffer_queue != nullptr
                         ? static_cast<unsigned>(uxQueueMessagesWaiting(s_hardware_ntp_request_buffer_queue))
                         : 0U);
            const ethernet_recovery_result_t result = recover_ethernet_with_timeout();
            ESP_LOGW(TAG, "Ethernet recovery result: %u (0=recovered)", static_cast<unsigned>(result));
            if (result == ethernet_recovery_result_t::recovered)
            {
                recovery_verification_us = esp_timer_get_time();
            }
            else if (result != ethernet_recovery_result_t::link_unavailable &&
                     physical_ethernet_link_status() != physical_link_status_t::down)
            {
                controlled_restart("ethernet_recovery_failed");
            }
        }
#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(Ethernet_Transport_Recovery);
#endif
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void synchronize_hardware_clock()
{
    if (!s_ptp_clock_ready.load(std::memory_order_acquire))
        return;

    // This path sets no clean anchor, so the PTP drift compensation is switched off until the next PPS
    s_ptp_comp_valid.store(false, std::memory_order_release);

    struct timeval now{};
    gettimeofday(&now, nullptr);
    const struct timespec clock_time{
        now.tv_sec,
        static_cast<long>(now.tv_usec) * 1000L};
    TIMING_LOGI(TAG, "Setting PTP clock to %lld.%06ld.",
                static_cast<long long>(now.tv_sec), static_cast<long>(now.tv_usec));
    if (clock_settime(CLOCK_PTP_SYSTEM, &clock_time) != 0)
    {
        TIMING_LOGE(TAG, "Setting PTP clock failed (errno=%d).", errno);
        return;
    }
    TIMING_LOGI(TAG, "PTP clock set.");
#if PTP_FREE_RUNNING
    s_ptp_epoch.fetch_add(1, std::memory_order_acq_rel); // the servo starts its estimate again
#endif
}

#if PTP_FREE_RUNNING
// Capture of the EMAC PTP pulse (PTP PPS loopback): only the capture ticks are stored, the PPS task pairs them
// with the GNSS PPS edges
static bool IRAM_ATTR ptp_pps_capture_callback(mcpwm_cap_channel_handle_t,
                                               const mcpwm_capture_event_data_t *edata,
                                               void *)
{
    if (edata != nullptr)
    {
        const uint32_t head = s_ptp_pps_capture_head.load(std::memory_order_relaxed);
        s_ptp_pps_captures[head % ptp_pps_capture_ring_size].store(edata->cap_value, std::memory_order_relaxed);
        s_ptp_pps_capture_head.store(head + 1, std::memory_order_release);
    }
    return false;
}

// PTP PPS loopback: the EMAC outputs its PPS on PTPPPSLoopbackPin, and a second channel of the PPS capture timer
// reads the same pad back. Nothing has to be wired.
static void setup_ptp_pps_loopback(esp_eth_mac_t *mac)
{
    if (PTPPPSLoopbackPin < 0)
        return;

    esp_chip_info_t chip_info{};
    esp_chip_info(&chip_info);
    if (chip_info.revision < 300)
    {
        TIMING_LOGW(TAG, "PTP PPS loopback requires chip revision v3.0 or later (detected v%u.%u); software measurement is used",
                    static_cast<unsigned>(chip_info.revision / 100),
                    static_cast<unsigned>(chip_info.revision % 100));
        return;
    }

    if (s_ptp_loopback_active.load(std::memory_order_acquire))
    {
        // Already set up: after an Ethernet recovery only the output mode of the EMAC is set again
        esp_eth_mac_set_pps_out_freq(mac, 0);
        return;
    }
    if (PTPPPSLoopbackPin == PPSPin || s_pps_capture_timer == nullptr)
    {
        TIMING_LOGW(TAG, "PTP PPS loopback: GPIO %d cannot be used (it must not be the PPS pin)", PTPPPSLoopbackPin);
        return;
    }

    // 1 PPS output: rising edge at every second of the PTP clock
    esp_err_t err = esp_eth_mac_set_pps_out_gpio(mac, PTPPPSLoopbackPin);
    if (err == ESP_OK)
        err = esp_eth_mac_set_pps_out_freq(mac, 0);
    if (err != ESP_OK)
    {
        TIMING_LOGW(TAG, "PTP PPS loopback: PPS output on GPIO %d failed (%s), software measurement is used",
                    PTPPPSLoopbackPin, esp_err_to_name(err));
        return;
    }

    mcpwm_capture_channel_config_t channel_config{};
    channel_config.gpio_num = PTPPPSLoopbackPin; // also enables the input of the pad, the output stays connected
    channel_config.prescale = 1;
    channel_config.flags.pos_edge = true;
    mcpwm_cap_channel_handle_t channel = nullptr;
    err = mcpwm_new_capture_channel(s_pps_capture_timer, &channel_config, &channel);
    if (err == ESP_OK)
    {
        mcpwm_capture_event_callbacks_t callbacks{};
        callbacks.on_cap = ptp_pps_capture_callback;
        err = mcpwm_capture_channel_register_event_callbacks(channel, &callbacks, nullptr);
        if (err == ESP_OK)
            err = mcpwm_capture_channel_enable(channel);
    }
    if (err != ESP_OK)
    {
        TIMING_LOGW(TAG, "PTP PPS loopback: capture channel on GPIO %d failed (%s), software measurement is used",
                    PTPPPSLoopbackPin, esp_err_to_name(err));
        if (channel != nullptr)
            mcpwm_del_capture_channel(channel);
        return;
    }
    s_ptp_loopback_active.store(true, std::memory_order_release);
    TIMING_LOGI(TAG, "PTP PPS loopback on GPIO %d: the PTP clock is measured against the PPS in hardware", PTPPPSLoopbackPin);
}
#endif

static bool configure_hardware_timestamps()
{
    const esp_eth_handle_t handle = s_eth_handle;
    if (handle == nullptr)
        return false;

    const esp_eth_clock_cfg_t clock_config{CLOCK_PTP_SYSTEM};
    if (esp_eth_clock_init(handle, &clock_config) != ESP_OK)
        return false;

    esp_eth_mac_t *mac = nullptr;
    if (esp_eth_get_mac_instance(handle, &mac) != ESP_OK ||
        esp_eth_mac_enable_ts4all(mac, true) != ESP_OK)
        return false;

#if PTP_FREE_RUNNING
    s_ptp_mac = mac;
#endif
    if (xSemaphoreTake(s_time_mutex, portMAX_DELAY) != pdTRUE)
        return false;
    s_ptp_clock_ready.store(true, std::memory_order_release);
    synchronize_hardware_clock();
#if PTP_FREE_RUNNING
    setup_ptp_pps_loopback(mac);
    if (!s_ptp_loopback_active.load(std::memory_order_acquire))
        TIMING_LOGI(TAG, "PTP clock: free-running and frequency disciplined, measured against the PPS by software readings");
#endif
    xSemaphoreGive(s_time_mutex);

    const bool input_path_updated = esp_eth_update_input_path_info(handle, ntp_ethernet_input, s_eth_netif) == ESP_OK;
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

static bool wait_for_gnss_pvt(uint16_t timeout_ms)
{
    const int64_t deadline_us = esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000LL;
    do
    {
        if (s_gnss.getPVT(timeout_ms))
            return true;
        if (s_gnss.packetUBXNAVPVT == nullptr ||
            !s_gnss.packetUBXNAVPVT->automaticFlags.flags.bits.automatic ||
            esp_timer_get_time() >= deadline_us)
            return false;

        vTaskDelay(pdMS_TO_TICKS(10));
    } while (esp_timer_get_time() < deadline_us);
    return false;
}

static bool current_gnss_timing_is_valid()
{
    if (s_use_nmea_fallback)
    {
        nmea_rmc_time_t nmea_time{};
        return wait_for_nmea_rmc_time(&nmea_time, 1200UL);
    }

    // Get the latest Position/Velocity/Time solution and fill all global variables
    static constexpr uint16_t status_query_timeout_ms = 1500;
    if (!wait_for_gnss_pvt(status_query_timeout_ms))
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
    bool timing_output_set = uart1_output_set;
#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
    const bool auto_pvt_set = uart1_output_set &&
                              s_gnss.setAutoPVT(true, true, VAL_LAYER_RAM, 1500);
    timing_output_set = uart1_output_set && auto_pvt_set;
    if (auto_pvt_set)
        TIMING_LOGI(TAG, "GNSS timing: automatic NAV-PVT enabled on UART1 (RAM only).");
    else
        TIMING_LOGW(TAG, "GNSS automatic NAV-PVT configuration failed; NMEA fallback is required.");
#endif

    if (uart1_output_set_result != nullptr)
        *uart1_output_set_result = timing_output_set;

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

    return timing_output_set;
}

static void halt_with_display(const char *line1, const char *line2, const char *line3)
{
    display_line(1, line1);
    display_line(2, line2);
    display_line(3, line3);
    s_saved_gnss_baud_communication_failed.store(true);

    while (true)
        vTaskDelay(pdMS_TO_TICKS(1000));
}

// ---- PPS interrupt latency: measured once per chip / configuration and stored in NVS ----
// 1 = at the first start (and whenever the IDF version, CPU frequency, chip revision or
//     PPS_ISR_LATENCY_CODE_VERSION change) the interrupt latency of the PPS capture path is measured with a
//     software triggered capture event (about 110 s after the start), applied immediately and stored in NVS.
//     At every other start the stored value is used. Until the measurement is done, and if it fails,
//     PPS_ISR_LATENCY_DEFAULT_US is used.
// 0 = always use PPS_ISR_LATENCY_DEFAULT_US
#define PPS_ISR_LATENCY_AUTO 1
// 1 = ignore the stored value and measure again at the next start (set back to 0 afterwards)
#define PPS_ISR_LATENCY_FORCE_REMEASURE 0
#if PPS_ISR_LATENCY_AUTO
// Increase this number when the PPS interrupt path (pps_capture_callback) changes, to trigger a new measurement
// (2: the stored value is the central value, mean of the lowest 75 % of the samples; version 1 stored an expected minimum)
// (3: test triggers keep away from real PPS edges; the compiler optimization level is part of the fingerprint)
// (4: measured on the fine local timebase; with FINE_LOCAL_TIMEBASE 0 the version stays 3)
static constexpr uint32_t PPS_ISR_LATENCY_CODE_VERSION = (LOCAL_TICKS_PER_US > 1) ? 4 : 3;
static std::atomic<uint32_t> s_isr_latency_test_armed{0};
static std::atomic<uint32_t> s_isr_latency_test_fired{0};
static volatile int64_t s_isr_latency_test_cb_ns = 0; // local time (ns) of the test callback
static BaseType_t s_pps_isr_core = tskNO_AFFINITY;    // core on which the PPS interrupt was allocated
static bool s_pps_isr_latency_known = false;          // a stored value for this configuration was loaded
static std::atomic<uint32_t> s_last_pps_isr_us32{0};  // low 32 bits of the esp_timer time of the last real PPS edge
#endif

static bool IRAM_ATTR pps_capture_callback(mcpwm_cap_channel_handle_t,
                                           const mcpwm_capture_event_data_t *edata,
                                           void *)
{
    PpsCaptureEvent event{};

    // Local time of the callback (fine timebase) and the same instant in whole microseconds
    const uint64_t isr_ticks = local_ticks();
    event.isr_local_ns = local_ticks_to_ns(isr_ticks);
    event.approximate_edge_us = local_ticks_to_esp_us(isr_ticks);

    // Raw MCPWM capture ticks at the PPS edge (0 if no capture data, should not normally happen)
    event.cap_ticks = (edata != nullptr) ? edata->cap_value : 0;

#if PPS_ISR_LATENCY_AUTO
    // Software-triggered test event (see measure_pps_isr_latency_task): record the time and do not
    // pass it on as a PPS pulse
    if (s_isr_latency_test_armed.load(std::memory_order_relaxed) != 0)
    {
        s_isr_latency_test_cb_ns = event.isr_local_ns;
        s_isr_latency_test_armed.store(0, std::memory_order_relaxed);
        s_isr_latency_test_fired.store(1, std::memory_order_release);
        return false;
    }
    s_last_pps_isr_us32.store(static_cast<uint32_t>(event.approximate_edge_us), std::memory_order_relaxed);
#endif

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

    // Query the actual resolution: the driver may differ from the requested value
    uint32_t actual_hz = 0;
    ESP_ERROR_CHECK(mcpwm_capture_timer_get_resolution(s_pps_capture_timer, &actual_hz));
    if (actual_hz == 0)
        actual_hz = PPS_CAPTURE_RESOLUTION_HZ;
    s_pps_capture_hz = actual_hz;
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "PPS capture timer resolution: %lu Hz (requested %lu Hz)",
             static_cast<unsigned long>(actual_hz), static_cast<unsigned long>(PPS_CAPTURE_RESOLUTION_HZ));
#endif

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
#if PPS_ISR_LATENCY_AUTO
    s_pps_isr_core = static_cast<BaseType_t>(xPortGetCoreID());
#endif
}

#if PPS_ISR_LATENCY_AUTO || PTP_LATENCY_AUTO
// Compiler optimization level: it changes the code timing of the interrupt and PTP paths, so it is part of the
// configuration fingerprints (a change triggers a new measurement)
static constexpr uint32_t compiler_optimization_id()
{
#if defined(CONFIG_COMPILER_OPTIMIZATION_DEBUG)
    return 1u;
#elif defined(CONFIG_COMPILER_OPTIMIZATION_SIZE)
    return 2u;
#elif defined(CONFIG_COMPILER_OPTIMIZATION_PERF)
    return 3u;
#elif defined(CONFIG_COMPILER_OPTIMIZATION_NONE)
    return 4u;
#else
    return 0u;
#endif
}

// FNV-1a hash over 32 bit words (configuration fingerprints that are stored together with the latencies)
static uint32_t fnv1a_words(const uint32_t *items, size_t count)
{
    uint32_t hash = 2166136261u;
    for (size_t k = 0; k < count; ++k)
        for (int i = 0; i < 4; ++i)
        {
            hash ^= (items[k] >> (8 * i)) & 0xFFu;
            hash *= 16777619u;
        }
    return hash;
}

struct latency_calibration_t
{
    uint32_t fingerprint;
    int32_t ns;
};

static bool load_latency_calibration(const char *name, uint32_t fingerprint, int32_t min_ns, int32_t max_ns, int32_t &ns)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(name, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
        return false;
    if (err != ESP_OK)
    {
        TIMING_LOGW(TAG, "%s: NVS open failed: %s", name, esp_err_to_name(err));
        return false;
    }
    latency_calibration_t calibration{};
    size_t length = sizeof(calibration);
    err = nvs_get_blob(handle, "cal", &calibration, &length);
    nvs_close(handle);
    if (err != ESP_OK)
    {
        if (err != ESP_ERR_NVS_NOT_FOUND)
            TIMING_LOGW(TAG, "%s: NVS read failed: %s", name, esp_err_to_name(err));
        return false;
    }
    if (length != sizeof(calibration) || calibration.fingerprint != fingerprint ||
        calibration.ns < min_ns || calibration.ns > max_ns)
        return false;
    ns = calibration.ns;
    return true;
}

static bool save_latency_calibration(const char *name, uint32_t fingerprint, int32_t ns)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(name, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        TIMING_LOGW(TAG, "%s: NVS open failed: %s", name, esp_err_to_name(err));
        return false;
    }
    const latency_calibration_t calibration{fingerprint, ns};
    latency_calibration_t previous{};
    size_t length = sizeof(previous);
    err = nvs_get_blob(handle, "cal", &previous, &length);
    if (err == ESP_OK && length == sizeof(previous) &&
        previous.fingerprint == fingerprint && previous.ns == ns)
    {
        nvs_close(handle);
        return true;
    }
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND && err != ESP_ERR_NVS_INVALID_LENGTH)
    {
        nvs_close(handle);
        TIMING_LOGW(TAG, "%s: NVS read before save failed: %s", name, esp_err_to_name(err));
        return false;
    }
    err = nvs_set_blob(handle, "cal", &calibration, sizeof(calibration));
    if (err == ESP_OK)
        err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK)
        TIMING_LOGW(TAG, "%s: NVS save failed: %s; using calibration in RAM", name, esp_err_to_name(err));
    return err == ESP_OK;
}
#endif

#if PPS_ISR_LATENCY_AUTO
static constexpr char isr_lat_NVS_NAMESPACE[] = "isr_latency";
static constexpr int32_t isr_lat_MIN_NS = 500; // plausible range of the measured latency
static constexpr int32_t isr_lat_MAX_NS = 20000;

// Hash (FNV-1a) of everything that can change the interrupt latency of this build and chip
static uint32_t pps_isr_latency_fingerprint()
{
    esp_chip_info_t chip_info{};
    esp_chip_info(&chip_info);

    const uint32_t items[] = {
        PPS_ISR_LATENCY_CODE_VERSION,
        static_cast<uint32_t>(ESP_IDF_VERSION_MAJOR),
        static_cast<uint32_t>(ESP_IDF_VERSION_MINOR),
        static_cast<uint32_t>(ESP_IDF_VERSION_PATCH),
#ifdef CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
        static_cast<uint32_t>(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ),
#else
        0u,
#endif
#if defined(CONFIG_MCPWM_ISR_IRAM_SAFE) || defined(CONFIG_MCPWM_ISR_CACHE_SAFE)
        1u,
#else
        0u,
#endif
#ifdef CONFIG_MCPWM_ISR_HANDLER_IN_IRAM
        1u,
#else
        0u,
#endif
        compiler_optimization_id(),
        static_cast<uint32_t>(chip_info.model),
        static_cast<uint32_t>(chip_info.revision),
        static_cast<uint32_t>(chip_info.cores)};

    return fnv1a_words(items, sizeof(items) / sizeof(items[0]));
}

// Loads the stored latency if it belongs to this configuration. Call after the NVS was initialised.
static void load_pps_isr_latency_from_nvs()
{
    s_pps_isr_latency_known = false;

#if !PPS_ISR_LATENCY_FORCE_REMEASURE
    int32_t ns = 0;
    if (load_latency_calibration(isr_lat_NVS_NAMESPACE, pps_isr_latency_fingerprint(), isr_lat_MIN_NS, isr_lat_MAX_NS, ns))
    {
        s_pps_isr_latency_ns.store(ns, std::memory_order_relaxed);
        s_pps_isr_latency_known = true;
        TIMING_LOGI(TAG, "PPS ISR latency: using the stored value %.2f us", static_cast<double>(ns) / 1000.0);
        return;
    }
#endif

    TIMING_LOGI(TAG, "PPS ISR latency: no stored value for this configuration, using %.2f us until it is measured",
                static_cast<double>(s_pps_isr_latency_ns.load(std::memory_order_relaxed)) / 1000.0);
}

static bool save_pps_isr_latency_to_nvs(int32_t ns)
{
    return save_latency_calibration(isr_lat_NVS_NAMESPACE, pps_isr_latency_fingerprint(), ns);
}

// One-time measurement of the interrupt latency of the PPS capture path, without external equipment.
// The MCPWM capture channel is triggered by software. The time from the trigger until the callback reads
// esp_timer is the same path a real PPS edge takes (capture latch -> interrupt -> driver ISR ->
// pps_capture_callback). The result is applied immediately and stored in NVS (see PPS_ISR_LATENCY_AUTO).
static void measure_pps_isr_latency_task(void *)
{
    static constexpr int trials = 160;      // number of trigger attempts
    static constexpr int spacing_ms = 500;  // spacing between attempts
    static constexpr int64_t maxGapUs = 5;  // reject attempts that were interrupted between the two clock reads
    static constexpr int64_t waitUs = 2000; // give up waiting for the callback after 2 ms

    double *lat = static_cast<double *>(malloc(sizeof(double) * trials * 2));
    if (lat == nullptr)
    {
        TIMING_LOGW(TAG, "PPS ISR latency measurement: out of memory, using %.2f us",
                    static_cast<double>(s_pps_isr_latency_ns.load(std::memory_order_relaxed)) / 1000.0);
        vTaskDelete(nullptr);
        return;
    }
    double *sorted = lat + trials;
    int valid = 0;
    int tried = 0;
    bool trigger_failed = false;

    vTaskDelay(pdMS_TO_TICKS(30000)); // let the system settle (NTP server running, normal load)
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "PPS ISR latency measurement started (about %d s); PPS interrupt on core %d, measuring on core %d",
             trials * spacing_ms / 1000, static_cast<int>(s_pps_isr_core), static_cast<int>(xPortGetCoreID()));
#endif

    for (int i = 0; i < trials; ++i)
    {
        vTaskDelay(pdMS_TO_TICKS(spacing_ms));

        // Stay away from real PPS edges: an edge that arrives while a test is armed would be taken for the test
        // event (wrong sample) and would be lost for the PPS task. Trigger only between 50 ms and 950 ms after the
        // last edge (the PPS phase is checked with the low 32 bits of esp_timer, enough for this purpose).
        bool safe_phase = false;
        for (int wait = 0; wait < 20; ++wait)
        {
            const uint32_t last_edge = s_last_pps_isr_us32.load(std::memory_order_relaxed);
            const uint32_t phase_us = static_cast<uint32_t>(esp_timer_get_time()) - last_edge;
            if (last_edge != 0 && s_pps_discipline_active.load() && phase_us >= 50000u && phase_us <= 950000u)
            {
                safe_phase = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (!safe_phase)
            continue;

        s_isr_latency_test_fired.store(0, std::memory_order_relaxed);
        s_isr_latency_test_armed.store(1, std::memory_order_release);

        const int64_t t0 = local_now_ns();
        const esp_err_t err = mcpwm_capture_channel_trigger_soft_catch(s_pps_capture_channel);
        const int64_t t1 = local_now_ns();

        if (err != ESP_OK)
        {
            s_isr_latency_test_armed.store(0, std::memory_order_relaxed);
            TIMING_LOGE(TAG, "PPS ISR latency measurement: soft trigger failed: %s", esp_err_to_name(err));
            trigger_failed = true;
            break;
        }

        while (s_isr_latency_test_fired.load(std::memory_order_acquire) == 0 &&
               (local_now_ns() - t0) < waitUs * 1000LL)
        {
        }

        const bool fired = s_isr_latency_test_fired.load(std::memory_order_acquire) != 0;
        s_isr_latency_test_armed.store(0, std::memory_order_relaxed); // disarm in any case
        ++tried;
        if (!fired)
            continue;

        // The trigger happened somewhere between the two clock reads: use the middle (result in us)
        const double latency = (static_cast<double>(s_isr_latency_test_cb_ns) -
                                0.5 * (static_cast<double>(t0) + static_cast<double>(t1))) /
                               1000.0;
        if ((t1 - t0) <= maxGapUs * 1000LL && latency >= 0.0 && latency <= 500.0)
            lat[valid++] = latency;
    }

    if (!trigger_failed && valid >= 32)
    {
        for (int i = 0; i < valid; ++i)
            sorted[i] = lat[i];
        for (int i = 1; i < valid; ++i) // insertion sort
        {
            const double v = sorted[i];
            int j = i - 1;
            while (j >= 0 && sorted[j] > v)
            {
                sorted[j + 1] = sorted[j];
                --j;
            }
            sorted[j + 1] = v;
        }
        double sum = 0.0;
        for (int i = 0; i < valid; ++i)
            sum += lat[i];
        const double median = (valid % 2 == 1) ? sorted[valid / 2] : 0.5 * (sorted[valid / 2 - 1] + sorted[valid / 2]);

        // The value to use is the central value of the interrupt latency in the same sense as in pps_discipline_task:
        // the mean of the lowest 75 % of the samples (late outliers are dropped)
        const int keep = std::max(1, (valid * 3) / 4);
        double keep_sum = 0.0;
        for (int i = 0; i < keep; ++i)
            keep_sum += sorted[i];
        const double recommended = keep_sum / keep;

        TIMING_LOGI(TAG, "PPS ISR latency: %d of %d valid | min=%.2f median=%.2f mean=%.2f p90=%.2f max=%.2f us",
                    valid, tried, sorted[0], median, sum / valid, sorted[(valid * 9) / 10], sorted[valid - 1]);
        TIMING_LOGI(TAG, "PPS ISR latency: central value (mean of the lowest 75 %%) = %.2f us", recommended);

        const int32_t ns = static_cast<int32_t>(llround(recommended * 1000.0));
        if (ns >= isr_lat_MIN_NS && ns <= isr_lat_MAX_NS)
        {
            const int32_t previous = s_pps_isr_latency_ns.exchange(ns, std::memory_order_relaxed);
            const bool saved = save_pps_isr_latency_to_nvs(ns);
            s_pps_isr_latency_known = true;
            TIMING_LOGI(TAG, "PPS ISR latency: %.2f us applied (was %.2f us)%s",
                        static_cast<double>(ns) / 1000.0, static_cast<double>(previous) / 1000.0,
                        saved ? " and stored in NVS" : ", NVS is not available: the value is used until the next restart");
        }
        else
        {
            TIMING_LOGW(TAG, "PPS ISR latency: %.2f us is outside %.1f..%.1f us, ignored; %.2f us stays in use",
                        recommended, isr_lat_MIN_NS / 1000.0, isr_lat_MAX_NS / 1000.0,
                        static_cast<double>(s_pps_isr_latency_ns.load(std::memory_order_relaxed)) / 1000.0);
        }
    }
    else if (!trigger_failed)
    {
        TIMING_LOGW(TAG, "PPS ISR latency measurement: only %d of %d attempts valid, %.2f us stays in use (retry at the next start)",
                    valid, tried, static_cast<double>(s_pps_isr_latency_ns.load(std::memory_order_relaxed)) / 1000.0);
    }

    free(lat);
    vTaskDelete(nullptr);
}
#endif

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

#if PTP_LATENCY_AUTO
static constexpr char ptp_lat_NVS_NAMESPACE[] = "ptp_latency";
static constexpr int32_t ptp_lat_MIN_NS = 500; // plausible range of the latency
static constexpr int32_t ptp_lat_MAX_NS = 20000;
// Increase this number when the way the PTP clock is written changes, to trigger a new calibration
// (2: the reference reading is taken on the fine local timebase; with FINE_LOCAL_TIMEBASE 0 the version stays 1)
static constexpr uint32_t PTP_LATENCY_CODE_VERSION = (LOCAL_TICKS_PER_US > 1) ? 2 : 1;

// Hash of everything that can change the PTP write latency of this build and chip
static uint32_t ptp_write_latency_fingerprint()
{
    esp_chip_info_t chip_info{};
    esp_chip_info(&chip_info);

    const uint32_t items[] = {
        0x50545031u, // "PTP1": keeps this fingerprint apart from the one of the interrupt latency
        PTP_LATENCY_CODE_VERSION,
        compiler_optimization_id(),
        static_cast<uint32_t>(ESP_IDF_VERSION_MAJOR),
        static_cast<uint32_t>(ESP_IDF_VERSION_MINOR),
        static_cast<uint32_t>(ESP_IDF_VERSION_PATCH),
#ifdef CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
        static_cast<uint32_t>(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ),
#else
        0u,
#endif
        static_cast<uint32_t>(chip_info.model),
        static_cast<uint32_t>(chip_info.revision),
        static_cast<uint32_t>(chip_info.cores)};

    return fnv1a_words(items, sizeof(items) / sizeof(items[0]));
}

// Loads the stored latency if it belongs to this configuration. Call after the NVS was initialised.
static void load_ptp_write_latency_from_nvs()
{
    s_ptp_cal_required = true;

#if !PTP_LATENCY_FORCE_RECALIBRATION
    int32_t ns = 0;
    if (load_latency_calibration(ptp_lat_NVS_NAMESPACE, ptp_write_latency_fingerprint(), ptp_lat_MIN_NS, ptp_lat_MAX_NS, ns))
    {
        s_ptp_write_latency_ns.store(ns, std::memory_order_relaxed);
        s_ptp_cal_required = false;
        TIMING_LOGI(TAG, "PTP write latency: %d ns read from NVS (calibration skipped)", static_cast<int>(ns));
        return;
    }
    TIMING_LOGI(TAG, "PTP write latency: no stored value for this configuration, using the default %d ns until the calibration has finished "
                     "(it starts once the PPS is locked: 10 min warm-up, then 30 to 40 min of measurement)",
                static_cast<int>(s_ptp_write_latency_ns.load(std::memory_order_relaxed)));
#else
    TIMING_LOGI(TAG, "PTP write latency: recalibration forced, using %d ns until the calibration has finished",
                static_cast<int>(s_ptp_write_latency_ns.load(std::memory_order_relaxed)));
#endif
}

static bool save_ptp_write_latency_to_nvs(int32_t ns)
{
    return save_latency_calibration(ptp_lat_NVS_NAMESPACE, ptp_write_latency_fingerprint(), ns);
}

// Statistics of one block of samples: trimmed mean and standard deviation over the central 80 % of the samples.
// The buffer is sorted in place.
static void ptp_cal_block_stats(int32_t *v, int n, double &mean, double &stddev, int32_t &vmin, int32_t &vmax)
{
    std::sort(v, v + n);
    vmin = v[0];
    vmax = v[n - 1];

    const int cut = n / 10;
    double sum = 0.0;
    int m = 0;
    for (int i = cut; i < n - cut; ++i)
    {
        sum += v[i];
        ++m;
    }
    mean = sum / m;

    double var = 0.0;
    for (int i = cut; i < n - cut; ++i)
    {
        const double d = v[i] - mean;
        var += d * d;
    }
    stddev = std::sqrt(var / m);
}

// Measures the PTP write latency during normal operation. The PPS task delivers one sample per second (the read-back
// of the first write, see set_hardware_clock_from_pps). After a warm-up of 10 minutes, samples are collected in blocks
// of 10 minutes. Per block the trimmed mean is calculated. As soon as the means of the last three blocks agree within
// max_spread_ns the result is accepted (after 30 minutes at the earliest). If this has not happened after four blocks
// (40 minutes) the calibration is given up and tried again at the next start. Loss of the PPS lock restarts everything.
// The result is applied immediately and stored in NVS.
static void ptp_latency_calibration_task(void *)
{
    static constexpr int64_t warmup_us = 600LL * 1000000LL; // warm-up after the PPS lock
    static constexpr int min_blocks = 3;                    // three blocks of 10 minutes
    static constexpr int max_blocks = 4;                    // four blocks (40 minutes) at most
    static constexpr double max_spread_ns = 250.0;          // block means have to agree within this range

    double block_mean[max_blocks] = {};
    int blocks = 0;
    bool warming_up = true;
    bool warmup_running = false;
    int64_t warmup_start_us = 0;
#if DEBUG_ENABLED
    int64_t last_progress_us = 0;
#endif
    bool failed = false;
    double result_ns = 0.0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        const int64_t now_us = esp_timer_get_time();

        const bool locked = s_pps_discipline_active.load() && s_time_has_been_set.load() &&
                            s_model_valid.load(std::memory_order_acquire);
        if (!locked)
        {
#if DEBUG_ENABLED
            if (warmup_running)
                ESP_LOGI(TAG, "PTP write latency calibration: PPS lock lost, starting again");
#endif
            ptp_latency_calibration_reset(false);
            blocks = 0;
            warming_up = true;
            warmup_running = false;
            continue;
        }

        if (warming_up)
        {
            if (!warmup_running)
            {
                warmup_running = true;
                warmup_start_us = now_us;
                TIMING_LOGI(TAG, "PTP write latency calibration: PPS locked, warm-up started (%d min)",
                            static_cast<int>(warmup_us / 60000000LL));
                continue;
            }
            if (now_us - warmup_start_us < warmup_us)
                continue;

            warming_up = false;
            ptp_latency_calibration_reset(true);
#if DEBUG_ENABLED
            last_progress_us = now_us;
#endif
            TIMING_LOGI(TAG, "PTP write latency calibration: warm-up done, measuring (about 30 to 40 min)");
            continue;
        }

        const int n = s_ptp_cal_count.load(std::memory_order_acquire);
        if (n < ptp_cal_block_samples)
        {
#if DEBUG_ENABLED
            if (now_us - last_progress_us >= 120LL * 1000000LL)
            {
                last_progress_us = now_us;
                ESP_LOGI(TAG, "PTP write latency calibration: block %d of at least %d, %d of %d samples",
                         blocks + 1, min_blocks, n, ptp_cal_block_samples);
            }
#endif
            continue;
        }

        // The block is full (the PPS task does not append any more until the count is reset)
        double mean = 0.0;
        double stddev = 0.0;
        int32_t vmin = 0;
        int32_t vmax = 0;
        ptp_cal_block_stats(s_ptp_cal_buf, n, mean, stddev, vmin, vmax);
        block_mean[blocks++] = mean;
        s_ptp_cal_count.store(0, std::memory_order_release);
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "PTP write latency calibration: block %d finished: %d samples, trimmed mean %.0f ns, std %.0f ns, min %d ns, max %d ns",
                 blocks, n, mean, stddev, static_cast<int>(vmin), static_cast<int>(vmax));
#endif

        if (blocks >= min_blocks)
        {
            double lo = block_mean[blocks - min_blocks];
            double hi = lo;
            double sum = 0.0;
            for (int i = blocks - min_blocks; i < blocks; ++i)
            {
                lo = std::min(lo, block_mean[i]);
                hi = std::max(hi, block_mean[i]);
                sum += block_mean[i];
            }
#if DEBUG_ENABLED
            ESP_LOGI(TAG, "PTP write latency calibration: spread of the last %d blocks %.0f ns (limit %.0f ns)",
                     min_blocks, hi - lo, max_spread_ns);
#endif
            if (hi - lo <= max_spread_ns)
            {
                result_ns = sum / min_blocks;
                break;
            }
            if (blocks >= max_blocks)
            {
                failed = true;
                break;
            }
        }
    }

    portENTER_CRITICAL(&s_ptp_cal_mux);
    s_ptp_cal_collect.store(false, std::memory_order_relaxed);
    int32_t *buffer = s_ptp_cal_buf;
    s_ptp_cal_buf = nullptr;
    portEXIT_CRITICAL(&s_ptp_cal_mux);
    free(buffer);

    const int32_t ns = static_cast<int32_t>(llround(result_ns));
    if (!failed && ns >= ptp_lat_MIN_NS && ns <= ptp_lat_MAX_NS)
    {
        const int32_t previous = s_ptp_write_latency_ns.exchange(ns, std::memory_order_relaxed);
        const bool saved = save_ptp_write_latency_to_nvs(ns);
        s_ptp_cal_required = false;
        TIMING_LOGI(TAG, "PTP write latency calibration finished: %d ns (default was %d ns)%s",
                    static_cast<int>(ns), static_cast<int>(previous),
                    saved ? ", stored in NVS" : ", NVS is not available: the value is used until the next restart");
    }
    else if (failed)
    {
        TIMING_LOGW(TAG, "PTP write latency calibration failed: the readings were not stable, keeping %d ns; it will be tried again at the next start",
                    static_cast<int>(s_ptp_write_latency_ns.load(std::memory_order_relaxed)));
    }
    else
    {
        TIMING_LOGW(TAG, "PTP write latency calibration: result %d ns is outside %d..%d ns and ignored, keeping %d ns; it will be tried again at the next start",
                    static_cast<int>(ns), static_cast<int>(ptp_lat_MIN_NS), static_cast<int>(ptp_lat_MAX_NS),
                    static_cast<int>(s_ptp_write_latency_ns.load(std::memory_order_relaxed)));
    }

    vTaskDelete(nullptr);
}
#endif

// Writes PPS second + time elapsed since the edge (+ the expected write latency) into the EMAC PTP clock.
// The value is read back and rewritten if the write was unusually late (e.g. the task was interrupted).
// The read-back of the first write also measures the write latency (see PTP_LATENCY_AUTO).
// correct_drift: false = elapsed time in oscillator units (PTP_FREE_RUNNING 0, its conversion expects that),
//                true  = elapsed time corrected by the measured ppb, i.e. true time (PTP_FREE_RUNNING 1)
// Returns true if the clock was written.
static bool ptp_write_from_edge(int64_t pps_second, double edge_local_us, bool correct_drift)
{
    static constexpr int64_t acceptDevNs = 2000; // tolerated deviation from the expected write latency
    static constexpr int maxAttempts = 3;

    // Latency in use during this call (the calibration can change it between two calls)
    const int64_t latency_ns = s_ptp_write_latency_ns.load(std::memory_order_relaxed);
    const double ppb = correct_drift ? static_cast<double>(s_clock_ppb.load(std::memory_order_relaxed)) : 0.0;
    bool written = false;

    for (int attempt = 0; attempt < maxAttempts; ++attempt)
    {
        const int64_t ref_ns = local_now_ns();
        const double elapsed_ns = static_cast<double>(ref_ns) - edge_local_us * 1000.0;
        const int64_t total_ns = pps_second * 1000000000LL + llround(elapsed_ns - elapsed_ns * ppb * 1e-9) + latency_ns;

        struct timespec ts{};
        ts.tv_sec = static_cast<time_t>(total_ns / 1000000000LL);
        ts.tv_nsec = static_cast<long>(total_ns % 1000000000LL);
        if (clock_settime(CLOCK_PTP_SYSTEM, &ts) != 0)
            continue;
        written = true;

        // Read back, bracketed by two local time readings: the middle of the two readings is the time of the PTP read.
        // The PTP clock starts at total_ns when written and runs on from the write instant (ref + actual latency).
        // Therefore rb - (total + (time of the read - ref)) = -actual latency, and the deviation from the expected
        // latency is err + latency_ns.
        struct timespec rb{};
        const int64_t rb_a = local_now_ns();
        if (clock_gettime(CLOCK_PTP_SYSTEM, &rb) != 0)
            break;
        const int64_t rb_b = local_now_ns();
        const int64_t rb_ns = static_cast<int64_t>(rb.tv_sec) * 1000000000LL + rb.tv_nsec;
        const int64_t expected_ns = total_ns + (rb_a + rb_b) / 2 - ref_ns;
        const int64_t err_ns = rb_ns - expected_ns;
        const int64_t dev_ns = err_ns + latency_ns;

#if PTP_LATENCY_AUTO
        // Only the first write of a second is used for the calibration: it is not selected by the retry check below,
        // so the samples are not biased. Readings with a long bracket were interrupted and are dropped.
        if (attempt == 0 && (rb_b - rb_a) <= 3000)
            ptp_latency_calibration_add_sample(-err_ns);
#endif

        if (dev_ns > -acceptDevNs && dev_ns < acceptDevNs)
            break;

#if DEBUG_ENABLED
        ESP_LOGW(TAG, "PTP set retry %d: write deviation %lld ns", attempt + 1, static_cast<long long>(dev_ns));
#endif
    }
    return written;
}

#if !PTP_FREE_RUNNING
// PTP_FREE_RUNNING 0: sets the EMAC PTP clock to PPS second + time elapsed since the edge (every second) and measures
// the offset of the result against the model clock.
static void set_hardware_clock_from_pps(int64_t pps_second, double edge_local_us)
{
    if (!s_ptp_clock_ready.load(std::memory_order_acquire))
        return;

    s_ptp_comp_valid.store(false, std::memory_order_release);
    if (!ptp_write_from_edge(pps_second, edge_local_us, false))
        return;

    // Offset of the PTP clock against the model clock after this write: mean of several read-backs, each one
    // bracketed by two local time readings (the middle of the bracket is taken as the instant of the read).
    // The model time of that instant is pps_second + (middle - edge); both the PTP clock and the local timebase run on
    // the same oscillator, so no drift term is needed (the conversion applies the ppb to the whole time since the edge).
    // Averaging removes most of the error of the individual write (latency jitter).
    int64_t offset_ns = 0;
#if PTP_OFFSET_MEASUREMENT
    static constexpr int offsetReads = 16;        // about 16 x 3 us
    static constexpr int64_t maxBracketNs = 3000; // longer brackets were interrupted
    static constexpr int minValidReads = 4;
    int64_t offset_sum = 0;
    int valid_reads = 0;
    const int64_t pps_second_ns = pps_second * 1000000000LL;
    for (int i = 0; i < offsetReads; ++i)
    {
        struct timespec rb{};
        const int64_t a = local_now_ns();
        if (clock_gettime(CLOCK_PTP_SYSTEM, &rb) != 0)
            break;
        const int64_t b = local_now_ns();
        if (b - a > maxBracketNs)
            continue;
        const int64_t rb_ns = static_cast<int64_t>(rb.tv_sec) * 1000000000LL + rb.tv_nsec;
        const int64_t model_ns = pps_second_ns + llround(0.5 * static_cast<double>(a + b) - edge_local_us * 1000.0);
        offset_sum += rb_ns - model_ns;
        ++valid_reads;
    }
    if (valid_reads >= minValidReads)
    {
        offset_ns = offset_sum / valid_reads;
        // A deviation of more than 50 us cannot come from the write latency: do not trust the read-backs
        if (offset_ns > 50000 || offset_ns < -50000)
            offset_ns = 0;
    }
#if DEBUG_ENABLED
    s_ptp_offset_dbg_ns.store(static_cast<int32_t>(offset_ns), std::memory_order_relaxed);
    s_ptp_offset_dbg_n.store(valid_reads, std::memory_order_relaxed);
#endif
#endif
    ptp_comp_publish(pps_second * 1000000000LL, offset_ns);
    s_ptp_comp_valid.store(true, std::memory_order_release);
}
#else
// ---- Servo of the free-running PTP clock (PTP_FREE_RUNNING 1) ----
// Measurement, once per PPS edge: m = raw PTP time - true time at the edge
//   - with the PTP PPS loopback (PTPPPSLoopbackPin): the EMAC pulse at every PTP second and the GNSS PPS are captured
//     by the same MCPWM timer, m = -(capture of the PTP pulse - capture of the GNSS pulse). 12.5 ns resolution, no
//     software delay in the result. Whole seconds and a plausibility check come from a short software measurement.
//   - otherwise by software: several readings of the PTP clock, each one bracketed by two readings of the fine local
//     timebase; the middle of the bracket is taken as the instant of the reading (median of the shortest brackets).
//     The fixed delay of the reading within the bracket remains as a constant error (some 100 ns). With the loopback
//     this error is measured ("sw-hw" in the debug output) and also removed from software measurements afterwards.
// Filter (alpha-beta): estimates the offset and the free-running rate of the PTP clock (ppb, without the frequency
//   command). The frequency command is known exactly, so it is part of the prediction and not of the estimate.
// Servo: frequency command = -(estimated free-running rate) - offset / PTP_SERVO_TIME_CONSTANT_S, so the PTP clock
//   runs at the GNSS rate and its offset decays slowly. The served time stamps do not depend on the servo: they are
//   converted with the estimated offset (mapping); the servo only keeps the raw PTP clock close to true time.
static constexpr double ptp_filter_alpha = 0.25;          // weight of a new offset measurement
static constexpr double ptp_filter_beta = 0.035;          // weight for the rate (alpha^2 / (2 - alpha))
static constexpr double PTP_SERVO_TIME_CONSTANT_S = 16.0; // decay time of the offset under the servo
static constexpr double ptp_servo_max_slew_ppb = 20000.0; // offset correction at most 20 ppm (20 us per second)
static constexpr int32_t ptp_max_command_ppb = 5000000;   // limit of the frequency command (the HAL clamps at +-5,120,000 ppb)
static constexpr double ptp_step_threshold_ns = 100000.0; // larger offsets are stepped (clock_settime), smaller slewed
static constexpr double ptp_loopback_gate_ns = 20000.0;   // software and loopback measurement must agree within this
// Delay of the loopback path (EMAC pulse -> pad -> capture) minus the delay of the GNSS PPS input path, in ns.
// Both paths go through a pad and the GPIO matrix, so the difference is a few ns at most; 0 is fine.
static constexpr int64_t PTP_PPS_LOOPBACK_DELAY_NS = 0;
static constexpr int ptp_sw_reads_max = 16;

struct ptp_servo_state_t
{
    bool initialized = false;         // the filter has a valid offset estimate for the current epoch
    bool have_rate = false;           // free_ppb is an estimate (kept through holdover and re-initialisations)
    uint32_t epoch = 0;               // epoch of the PTP clock the estimate belongs to
    int64_t last_edge_sec = 0;        // second of the edge of the last estimate
    double offset_ns = 0.0;           // estimated offset at that edge (raw - true)
    double free_ppb = 0.0;            // estimated rate of the PTP clock without frequency command
    int32_t cmd_ppb = 0;              // frequency command in effect (absolute, against the base addend)
    int32_t prev_cmd_ppb = 0;         // command before the last update
    double update_after_edge_s = 0.0; // instant of the last command update, seconds after its edge
    double noise_ns = 0.0;            // running mean of |innovation|
    int rejects = 0;                  // measurements in a row rejected as outliers
    int settle = 0;                   // seconds after a start in which a loopback measurement may replace the estimate
    double sw_bias_ns = 0.0;          // delay error of the software measurement (software - loopback)
    uint32_t sw_bias_samples = 0;
    // statistics for the debug output
    double last_measurement_ns = 0.0;
    double last_innovation_ns = 0.0;
    bool last_from_loopback = false;
    uint32_t loopback_used = 0;
    uint32_t loopback_missing = 0;
    uint32_t loopback_rejected = 0;
    uint32_t outliers = 0;
    uint32_t steps = 0;
    uint32_t command_failures = 0;
};
static ptp_servo_state_t s_ptp_servo; // the PPS task is the only user

// Raw PTP time in ns (directly from the MAC, shorter than clock_gettime)
static bool ptp_read_raw_ns(int64_t &raw_ns)
{
    if (s_ptp_mac != nullptr)
    {
        eth_mac_time_t t{};
        if (esp_eth_mac_get_ptp_time(s_ptp_mac, &t) != ESP_OK)
            return false;
        raw_ns = static_cast<int64_t>(t.seconds) * 1000000000LL + static_cast<int64_t>(t.nanoseconds);
        return true;
    }
    struct timespec ts{};
    if (clock_gettime(CLOCK_PTP_SYSTEM, &ts) != 0)
        return false;
    raw_ns = static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
    return true;
}

// Software measurement of raw PTP - true time at the PPS edge. rate_ppb = current rate of the offset, used to refer
// the readings (some ms after the edge) back to the edge. Returns the number of readings used (0 = failed).
static int ptp_measure_offset_sw(int64_t edge_sec, double edge_local_us, double rate_ppb, int reads,
                                 double &offset_ns, int64_t &bracket_ns)
{
    static int64_t value[ptp_sw_reads_max]; // static: the PPS task has little stack
    static int64_t bracket[ptp_sw_reads_max];
    static constexpr int64_t maxBracketNs = 4000;  // longer brackets were interrupted
    static constexpr int64_t bracketSlackNs = 250; // readings within this of the shortest bracket are used

    const double ppb = static_cast<double>(s_clock_ppb.load(std::memory_order_relaxed));
    const double edge_ns = edge_local_us * 1000.0;
    const int64_t edge_sec_ns = edge_sec * 1000000000LL;
    int n = 0;
    int64_t min_bracket = INT64_MAX;
    reads = std::min(reads, ptp_sw_reads_max);
    for (int i = 0; i < reads; ++i)
    {
        int64_t raw = 0;
        const int64_t a = local_now_ns();
        const bool ok = ptp_read_raw_ns(raw);
        const int64_t b = local_now_ns();
        if (!ok)
            break;
        if (b - a > maxBracketNs)
            continue;
        const double el = static_cast<double>(a + (b - a) / 2) - edge_ns; // local ns since the edge
        const double el_true = el - el * ppb * 1e-9;                      // true ns since the edge
        value[n] = llround(static_cast<double>(raw - edge_sec_ns) - el_true - rate_ppb * el_true * 1e-9);
        bracket[n] = b - a;
        min_bracket = std::min(min_bracket, b - a);
        ++n;
    }
    if (n == 0)
        return 0;

    // Readings with the shortest brackets, median (robust against a reading that caught the second rollover
    // between the seconds and the sub-seconds register)
    int m = 0;
    for (int i = 0; i < n; ++i)
        if (bracket[i] <= min_bracket + bracketSlackNs)
            value[m++] = value[i];
    for (int i = 1; i < m; ++i) // insertion sort
    {
        const int64_t v = value[i];
        int j = i - 1;
        while (j >= 0 && value[j] > v)
        {
            value[j + 1] = value[j];
            --j;
        }
        value[j + 1] = v;
    }
    offset_ns = (m % 2 == 1) ? static_cast<double>(value[m / 2])
                             : 0.5 * (static_cast<double>(value[m / 2 - 1]) + static_cast<double>(value[m / 2]));
    bracket_ns = min_bracket;
    return m;
}

// Phase of the latest PTP loopback pulse against the GNSS PPS capture, in capture ticks (> 0: the PTP pulse came
// later). Waits up to 300 us for a PTP pulse that follows the GNSS pulse closely.
static bool ptp_loopback_phase_ticks(uint32_t gnss_cap_ticks, int32_t &phase_ticks)
{
    const int32_t half_second = static_cast<int32_t>(s_pps_capture_hz / 2);
    bool waited = false;
    for (int pass = 0; pass < 4; ++pass)
    {
        const uint32_t head = s_ptp_pps_capture_head.load(std::memory_order_acquire);
        bool found = false;
        int32_t best = 0;
        for (uint32_t k = 0; k < ptp_pps_capture_ring_size && k < head; ++k)
        {
            const uint32_t c = s_ptp_pps_captures[(head - 1 - k) % ptp_pps_capture_ring_size].load(std::memory_order_relaxed);
            const int32_t d = static_cast<int32_t>(c - gnss_cap_ticks); // modulo arithmetic handles the 32 bit wrap
            if (d > -half_second && d < half_second && (!found || std::llabs(d) < std::llabs(best)))
            {
                best = d;
                found = true;
            }
        }
        if (s_ptp_pps_capture_head.load(std::memory_order_acquire) != head)
            continue; // a capture arrived while reading: read again
        if (found)
        {
            phase_ticks = best;
            return true;
        }
        if (waited)
            return false;
        waited = true;
        const int64_t until_ns = local_now_ns() + 300000;
        while (local_now_ns() < until_ns && s_ptp_pps_capture_head.load(std::memory_order_acquire) == head)
        {
        }
    }
    return false;
}

// Applies a frequency command (ppb, absolute against the base addend). Returns false if the EMAC refused it.
static bool ptp_apply_command(int32_t cmd_ppb)
{
    ptp_servo_state_t &st = s_ptp_servo;
    if (cmd_ppb == st.cmd_ppb)
        return true;
    if (s_ptp_mac == nullptr || esp_eth_mac_adj_ptp_time(s_ptp_mac, cmd_ppb) != ESP_OK)
    {
        ++st.command_failures;
        return false;
    }
    st.cmd_ppb = cmd_ppb;
    return true;
}

// Sets the PTP clock to true time (for large offsets only); the filter starts again in the new epoch
static void ptp_servo_step(int64_t edge_sec, double edge_local_us, double offset_ns)
{
    ptp_servo_state_t &st = s_ptp_servo;
    s_ptp_comp_valid.store(false, std::memory_order_release);
    // Run at the estimated GNSS rate right away, so the raw clock is usable until the next measurement
    ptp_apply_command(static_cast<int32_t>(std::clamp(llround(-st.free_ppb), static_cast<long long>(-ptp_max_command_ppb),
                                                      static_cast<long long>(ptp_max_command_ppb))));
    const bool written = ptp_write_from_edge(edge_sec, edge_local_us, true);
    st.epoch = s_ptp_epoch.fetch_add(1, std::memory_order_acq_rel) + 1;
    st.initialized = false;
    ++st.steps;
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "PTP clock %s (offset was %.1f us)", written ? "set" : "could not be set", offset_ns / 1000.0);
#else
    (void)offset_ns;
    (void)written;
#endif
}

// Called by the PPS task for every accepted PPS edge (s_time_mutex held, after the model clock was published).
// rate_valid: the PPS task has a frequency estimate of the crystal (s_clock_ppb), used as the start value of the rate.
static void ptp_servo_update(int64_t edge_sec, double edge_local_us, uint32_t gnss_cap_ticks, bool rate_valid)
{
    ptp_servo_state_t &st = s_ptp_servo;
    if (!s_ptp_clock_ready.load(std::memory_order_acquire) || !rate_valid)
        return;

    const uint32_t epoch = s_ptp_epoch.load(std::memory_order_acquire);
    if (s_ptp_addend_reset.exchange(false, std::memory_order_acq_rel))
    {
        // The PTP block was initialised again (Ethernet recovery): the hardware runs without frequency command
        st.cmd_ppb = 0;
        st.prev_cmd_ppb = 0;
    }
    if (epoch != st.epoch)
    {
        st.epoch = epoch;
        st.initialized = false;
    }
    if (!st.have_rate)
    {
        // The PTP clock runs on the same crystal as the local timebase: start with the measured oscillator error
        st.free_ppb = static_cast<double>(s_clock_ppb.load(std::memory_order_relaxed));
        st.have_rate = true;
    }

    // ---- Measurement ----
    // The first measurement after a (re)start comes from the software readings only: a loopback pulse captured
    // before the clock was set belongs to the old clock
    const bool loopback = s_ptp_loopback_active.load(std::memory_order_acquire) && st.initialized;
    double sw_offset_ns = 0.0;
    int64_t sw_bracket_ns = 0;
    const int sw_reads = ptp_measure_offset_sw(edge_sec, edge_local_us, st.free_ppb + st.cmd_ppb, loopback ? 4 : ptp_sw_reads_max,
                                               sw_offset_ns, sw_bracket_ns);
    if (s_ptp_epoch.load(std::memory_order_acquire) != epoch || !s_ptp_clock_ready.load(std::memory_order_acquire))
        return; // the clock was set or re-initialised meanwhile
    if (sw_reads == 0)
        return; // keep extrapolating with the last mapping

    const double sw_corrected_ns = sw_offset_ns - st.sw_bias_ns; // sw_bias_ns stays 0 without loopback
    double measurement_ns = sw_corrected_ns;
    bool from_loopback = false;
    if (loopback)
    {
        int32_t phase_ticks = 0;
        if (ptp_loopback_phase_ticks(gnss_cap_ticks, phase_ticks))
        {
            // The PTP pulse marks a whole PTP second. It came phase after the GNSS pulse, so at the GNSS edge the PTP
            // clock was phase short of a whole second; the whole seconds come from the software measurement.
            double hw_ns = -static_cast<double>(phase_ticks) * 1e9 / static_cast<double>(s_pps_capture_hz) +
                           static_cast<double>(PTP_PPS_LOOPBACK_DELAY_NS);
            hw_ns += std::round((sw_offset_ns - hw_ns) / 1e9) * 1e9;
            if (fabs(hw_ns - sw_corrected_ns) <= ptp_loopback_gate_ns)
            {
                measurement_ns = hw_ns;
                from_loopback = true;
                ++st.loopback_used;
            }
            else
                ++st.loopback_rejected;
        }
        else
            ++st.loopback_missing;

        static bool warned = false;
        if (!warned && st.loopback_used == 0 && st.loopback_missing + st.loopback_rejected >= 30)
        {
            warned = true;
            TIMING_LOGW(TAG, "PTP PPS loopback on GPIO %d: no usable pulses (%lu missing, %lu implausible); is the pin free and unconnected?",
                        PTPPPSLoopbackPin, static_cast<unsigned long>(st.loopback_missing), static_cast<unsigned long>(st.loopback_rejected));
        }
    }
    st.last_measurement_ns = measurement_ns;
    st.last_from_loopback = from_loopback;

    // ---- Filter ----
    const int64_t dt = edge_sec - st.last_edge_sec;
    if (st.initialized && (dt <= 0 || dt > 60))
        st.initialized = false; // edges missing for more than a minute (or time stepped): start again

    if (!st.initialized)
    {
        if (fabs(measurement_ns) > ptp_step_threshold_ns)
        {
            ptp_servo_step(edge_sec, edge_local_us, measurement_ns);
            return;
        }
        st.offset_ns = measurement_ns;
        st.noise_ns = 0.0;
        st.rejects = 0;
        st.last_innovation_ns = 0.0;
        st.settle = 3;
        st.initialized = true;
    }
    else
    {
        // Prediction: the old command until the last update, the current command after it
        const double pred_ns = st.offset_ns + (st.free_ppb + st.prev_cmd_ppb) * st.update_after_edge_s +
                               (st.free_ppb + st.cmd_ppb) * (static_cast<double>(dt) - st.update_after_edge_s);
        const double innovation_ns = measurement_ns - pred_ns;
        st.last_innovation_ns = innovation_ns;
        const double gate_ns = std::max(from_loopback ? 1000.0 : 3000.0, 8.0 * st.noise_ns);
        // Shortly after a start the first loopback measurement replaces the software start value (whose delay
        // error is not known yet)
        bool restart_from_loopback = false;
        if (st.settle > 0)
        {
            --st.settle;
            restart_from_loopback = from_loopback && fabs(innovation_ns) > gate_ns;
        }

        if (restart_from_loopback)
        {
            st.offset_ns = measurement_ns;
            st.noise_ns = 0.0;
            st.settle = 0;
        }
        else if (fabs(innovation_ns) > gate_ns && st.rejects < 4)
        {
            // Outlier: coast on the prediction (the published mapping stays valid, the command is unchanged)
            ++st.rejects;
            ++st.outliers;
            st.offset_ns = pred_ns;
            st.last_edge_sec = edge_sec;
            st.prev_cmd_ppb = st.cmd_ppb;
            st.update_after_edge_s = 0.0;
            return;
        }
        else if (fabs(innovation_ns) > gate_ns)
        {
            // Five in a row: the clock really moved, take the measurement as the new start (and the rate
            // estimate of the PPS task, in case the rate estimate was the problem)
#if DEBUG_ENABLED
            ESP_LOGW(TAG, "PTP servo: offset jumped by %.1f us, filter restarted", innovation_ns / 1000.0);
#endif
            st.offset_ns = measurement_ns;
            st.free_ppb = static_cast<double>(s_clock_ppb.load(std::memory_order_relaxed));
            st.noise_ns = 0.0;
        }
        else
        {
            st.offset_ns = pred_ns + ptp_filter_alpha * innovation_ns;
            st.free_ppb += ptp_filter_beta * innovation_ns / static_cast<double>(dt);
            st.noise_ns += (fabs(innovation_ns) - st.noise_ns) / 16.0;
        }
        st.rejects = 0;

        if (fabs(st.offset_ns) > ptp_step_threshold_ns)
        {
            ptp_servo_step(edge_sec, edge_local_us, st.offset_ns);
            return;
        }

        // Delay error of the software measurement, learned while the loopback works (running mean, weight 1/64)
        if (from_loopback && st.noise_ns < 200.0)
        {
            const double sample = sw_offset_ns - measurement_ns;
            st.sw_bias_ns = (st.sw_bias_samples < 64)
                                ? (st.sw_bias_ns * st.sw_bias_samples + sample) / (st.sw_bias_samples + 1)
                                : st.sw_bias_ns + (sample - st.sw_bias_ns) / 64.0;
            if (st.sw_bias_samples < UINT32_MAX)
                ++st.sw_bias_samples;
        }
    }
    st.last_edge_sec = edge_sec;

    // ---- Servo ----
    const double correction_ppb = std::clamp(-st.offset_ns / PTP_SERVO_TIME_CONSTANT_S, -ptp_servo_max_slew_ppb, ptp_servo_max_slew_ppb);
    const int32_t command = static_cast<int32_t>(std::clamp(llround(-st.free_ppb + correction_ppb),
                                                            static_cast<long long>(-ptp_max_command_ppb),
                                                            static_cast<long long>(ptp_max_command_ppb)));
    const int32_t old_command = st.cmd_ppb;
    ptp_apply_command(command);
    const double update_local_s = std::max(0.0, (static_cast<double>(local_now_ns()) - edge_local_us * 1000.0) * 1e-9);
    const double update_s = update_local_s - update_local_s * static_cast<double>(s_clock_ppb.load(std::memory_order_relaxed)) * 1e-9;
    st.prev_cmd_ppb = old_command;
    st.update_after_edge_s = update_s;

    // ---- Mapping, reference point = instant of this update ----
    const double ref_offset_ns = st.offset_ns + (st.free_ppb + old_command) * update_s;
    const int64_t ref_true_ns = edge_sec * 1000000000LL + llround(update_s * 1e9);
    ptp_map_publish(ref_true_ns + llround(ref_offset_ns), llround(ref_offset_ns),
                    llround((st.free_ppb + old_command) * 1000.0), llround((st.free_ppb + st.cmd_ppb) * 1000.0));
    s_ptp_comp_valid.store(true, std::memory_order_release);
}

// Called by the PPS task when the PPS is lost: stop the offset correction and keep the PTP clock on the estimated
// rate (holdover). The mapping continues from the current offset.
static void ptp_servo_holdover()
{
    ptp_servo_state_t &st = s_ptp_servo;
    if (!st.initialized || !s_ptp_clock_ready.load(std::memory_order_acquire) || !s_ptp_comp_valid.load(std::memory_order_acquire))
        return;
    int64_t raw_ns = 0;
    int64_t offset_ns = 0;
    if (!ptp_read_raw_ns(raw_ns) || !ptp_map_offset_at(raw_ns, offset_ns))
        return;
    const int32_t old_command = st.cmd_ppb;
    ptp_apply_command(static_cast<int32_t>(std::clamp(llround(-st.free_ppb), static_cast<long long>(-ptp_max_command_ppb),
                                                      static_cast<long long>(ptp_max_command_ppb))));
    ptp_map_publish(raw_ns, offset_ns, llround((st.free_ppb + old_command) * 1000.0), llround((st.free_ppb + st.cmd_ppb) * 1000.0));
    st.initialized = false; // the next PPS edge starts the filter again, the rate estimate is kept
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "PTP servo: holdover, offset %.1f ns, rate %.3f ppb, command %d ppb", static_cast<double>(offset_ns),
             st.free_ppb, static_cast<int>(st.cmd_ppb));
#endif
}

#if DEBUG_ENABLED
// Status line of the servo (debug output of the PPS task)
static void ptp_servo_log_status()
{
    const ptp_servo_state_t &st = s_ptp_servo;
    ESP_LOGI(TAG, "PTP: %s offset %.1f ns (meas %.1f, innov %.1f, noise %.1f) rate %.3f ppb cmd %d ppb | outliers %lu steps %lu cmd err %lu",
             st.last_from_loopback ? "loopback" : "software", st.offset_ns, st.last_measurement_ns, st.last_innovation_ns,
             st.noise_ns, st.free_ppb, static_cast<int>(st.cmd_ppb), static_cast<unsigned long>(st.outliers),
             static_cast<unsigned long>(st.steps), static_cast<unsigned long>(st.command_failures));
    if (s_ptp_loopback_active.load(std::memory_order_relaxed))
        ESP_LOGI(TAG, "PTP loopback: used %lu missing %lu rejected %lu | software - loopback %.1f ns (%lu)",
                 static_cast<unsigned long>(st.loopback_used), static_cast<unsigned long>(st.loopback_missing),
                 static_cast<unsigned long>(st.loopback_rejected), st.sw_bias_ns, static_cast<unsigned long>(st.sw_bias_samples));
}
#endif
#endif

static void pps_discipline_task(void *parameter)
{
    static constexpr int64_t ppsTimeoutUs = 3500000;            // 3.5 s: PPS missing timeout
    static constexpr int64_t maxPhaseErrorUsToCorrect = 250000; // do not correct if the phase error is huge
    static constexpr int64_t minAppliedCorrectionUs = 1;
    static constexpr double settledPhaseUs = 200.0; // below this the clock counts as settled
    static constexpr int freqCap = 17;              // frequency window: at most 16 s
    static constexpr int freqMinSeconds = 4;        // first estimate after 4 s
    static constexpr int anchorWindow = 64;         // edges used for the central value of the interrupt latency
    static constexpr double maxPlausiblePpm = 200.0;
    static constexpr double spikeLimitUs = 50.0; // tolerated deviation of a PPS interval from the expected one
    static constexpr int maxSpikes = 3;          // after this many spikes in a row the edges are trusted again

    struct TickSample
    {
        int64_t seq;
        uint64_t ticks;
    };
    TickSample hist[freqCap]{};
    int hist_count = 0;
    static double anchor_d[anchorWindow]; // static: the PPS task has little stack
    static double anchor_sorted[anchorWindow];
    int anchor_count = 0;
    int anchor_head = 0;

    bool have_last = false;
    uint32_t last_ticks = 0;
    uint64_t unwrapped = 0;
    int64_t seq = 0;
    double carry_us = 0.0;
    double est_ppm = 0.0;
    (void)est_ppm;          // only read by the debug log
    bool est_valid = false; // a frequency estimate exists (it is kept as holdover value when the PPS is lost)
    int spike_count = 0;    // PPS edges in a row that were ignored as spikes
    int64_t last_pps_us = 0;
    bool logged_active = false;
#if DEBUG_ENABLED
    uint32_t log_counter = 0;
#endif

    for (;;)
    {
        PpsCaptureEvent capture_event{};
        if (xQueueReceive(s_pps_timestamp_queue, &capture_event, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            const int64_t edge_us = capture_event.approximate_edge_us;                      // ISR time (noisy), whole microseconds
            const double isr_us = static_cast<double>(capture_event.isr_local_ns) / 1000.0; // the same, fine timebase
            const uint32_t ticks = capture_event.cap_ticks;                                 // hardware ticks (exact)
            const double hz = static_cast<double>(s_pps_capture_hz);

            // ---- 0. Spike rejection ----
            // Once the frequency is known, the interval to the previous edge is predictable to a few hundred ns.
            // An edge that is far off (glitch, spurious pulse) is ignored completely: it does not touch the estimators,
            // the system clock, the model or the sync state. After maxSpikes in a row the edges are trusted again
            // (the PPS itself may have moved) and the estimators start over.
            bool spike_accepted = false;
            if (have_last && est_valid)
            {
                const uint32_t gap_ticks = ticks - last_ticks;
                const int64_t gap_s = llround(static_cast<double>(gap_ticks) / hz);
                bool spike = (gap_s == 0); // two edges within half a second
                if (gap_s >= 1 && gap_s <= 3)
                {
                    const double expected_ticks = static_cast<double>(gap_s) * hz * (1.0 + est_ppm * 1e-6);
                    spike = fabs(static_cast<double>(gap_ticks) - expected_ticks) / hz * 1e6 > spikeLimitUs;
                }
                if (spike)
                {
                    if (spike_count < maxSpikes)
                    {
                        ++spike_count;
#if DEBUG_ENABLED
                        ESP_LOGW(TAG, "PPS spike ignored (%d of %d): interval %.6f s", spike_count, maxSpikes,
                                 static_cast<double>(gap_ticks) / hz);
#endif
                        continue;
                    }
                    spike_accepted = true;
                }
            }
            spike_count = 0;

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

            // ---- 1. Tick bookkeeping ----
            bool restart = !have_last || spike_accepted;
            if (have_last)
            {
                const uint32_t interval = ticks - last_ticks; // modulo arithmetic handles the 32 bit wrap
                const double secs = static_cast<double>(interval) / hz;
                const int64_t n = llround(secs);
                if (n >= 1 && n <= 3 && fabs(secs - static_cast<double>(n)) < 0.2)
                {
                    unwrapped += interval;
                    seq += n;
                }
                else
                {
#if DEBUG_ENABLED
                    ESP_LOGW(TAG, "PPS interval out of bounds (%.6f s); restarting estimators", secs);
#endif
                    restart = true;
                }
            }
            if (restart)
            {
                unwrapped = 0;
                seq = 0;
                hist_count = 0;
                anchor_count = 0;
                anchor_head = 0;
            }
            last_ticks = ticks;
            have_last = true;

            // ---- 2. Frequency estimate: sliding window of hardware ticks ----
            if (hist_count == freqCap)
            {
                memmove(&hist[0], &hist[1], sizeof(TickSample) * (freqCap - 1));
                hist_count--;
            }
            hist[hist_count++] = {seq, unwrapped};
            if (hist_count >= 2)
            {
                const TickSample &o = hist[0];
                const TickSample &nw = hist[hist_count - 1];
                const int64_t dseq = nw.seq - o.seq;
                if (dseq >= freqMinSeconds)
                {
                    const double ppm = (static_cast<double>(nw.ticks - o.ticks) /
                                            (static_cast<double>(dseq) * hz) -
                                        1.0) *
                                       1e6;
                    if (fabs(ppm) <= maxPlausiblePpm)
                    {
                        est_ppm = ppm; // >0 = oscillator / local clock runs TOO FAST
                        est_valid = true;
                        s_clock_ppb.store(static_cast<int32_t>(llround(ppm * 1000.0)), std::memory_order_relaxed);
                    }
                }
            }

            // ---- 3. Anchor of the edge in local time ----
            // d = ISR time - tick time = constant offset + interrupt latency (+ resolution of the local timebase).
            // The latency has outliers that are always late, so the central value is the mean of the
            // lowest 75 % of the last 64 values. The measured interrupt latency (see measure_pps_isr_latency_task)
            // is calculated with the same statistic and the same timebase, so the shape of the jitter cancels
            // out. A minimum would not do that: its bias depends on the noise.
            const double d = isr_us - static_cast<double>(unwrapped) * 1e6 / hz;
            anchor_d[anchor_head] = d;
            anchor_head = (anchor_head + 1) % anchorWindow;
            if (anchor_count < anchorWindow)
                anchor_count++;
            for (int i = 0; i < anchor_count; ++i)
                anchor_sorted[i] = anchor_d[i];
            std::sort(anchor_sorted, anchor_sorted + anchor_count);
            const int anchor_keep = std::max(1, (anchor_count * 3) / 4);
            double anchor_sum = 0.0;
            for (int i = 0; i < anchor_keep; ++i)
                anchor_sum += anchor_sorted[i];
            const double d_central = anchor_sum / anchor_keep;
            const double excess_us = d - d_central; // jitter of this edge relative to the central value
            const double edge_local_us = isr_us - excess_us -
                                         static_cast<double>(s_pps_isr_latency_ns.load(std::memory_order_relaxed)) / 1000.0;

            // ---- 4. Phase correction ----
            if (!s_time_has_been_set.load() || s_time_setting_in_progress.load())
                continue;
            if (xSemaphoreTake(s_time_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
                continue;

            // Several bracketed reads, keep the one with the smallest gap: the first read
            // (cold cache) is discarded and the mid-point assumption gets more precise.
            int64_t t1 = 0, t2 = 0; // local time in ns
            struct timeval now{};
            int64_t best_gap = INT64_MAX;
            for (int i = 0; i < 4; ++i)
            {
                struct timeval tv{};
                const int64_t a = local_now_ns();
                gettimeofday(&tv, nullptr);
                const int64_t b = local_now_ns();
                if ((b - a) < best_gap)
                {
                    best_gap = b - a;
                    t1 = a;
                    t2 = b;
                    now = tv;
                }
            }
            best_gap /= 1000; // us (debug output)

            const double sys_us = static_cast<double>(now.tv_sec) * 1e6 + static_cast<double>(now.tv_usec);
            const double mid_local_us = 0.5e-3 * (static_cast<double>(t1) + static_cast<double>(t2));
            const double edge_sys_us = sys_us - (mid_local_us - edge_local_us); // system clock value at the edge

            const int64_t edge_sec = static_cast<int64_t>(floor(edge_sys_us / 1e6 + 0.5));
            const double phase_error_us = edge_sys_us - static_cast<double>(edge_sec) * 1e6;

            // Residual error after drift compensation (the actual quality figure)
            double resid_us = phase_error_us;
            if (s_clock_comp_valid.load(std::memory_order_acquire))
                resid_us -= static_cast<double>(s_clock_ppb.load(std::memory_order_relaxed)) * 1e-9 *
                            (edge_local_us - static_cast<double>(s_clock_anchor_us.load(std::memory_order_relaxed)));

            bool ok = false;
            if (fabs(phase_error_us) <= static_cast<double>(maxPhaseErrorUsToCorrect))
            {
                const double total_us = -phase_error_us + carry_us;
                const int64_t apply_us = llround(total_us);
                ok = true;
                if (llabs(apply_us) >= minAppliedCorrectionUs)
                {
                    struct timeval delta{};
                    delta.tv_sec = static_cast<time_t>(apply_us / 1000000LL);
                    delta.tv_usec = static_cast<suseconds_t>(apply_us % 1000000LL);
                    if (delta.tv_usec < 0)
                    {
                        delta.tv_usec += 1000000;
                        delta.tv_sec -= 1;
                    }
                    ok = (adjtime(&delta, nullptr) == 0);
                }
                if (ok)
                {
                    carry_us = total_us - static_cast<double>(apply_us);
                    if (fabs(carry_us) > 2.0)
                        carry_us = 0.0;
                }
            }

            if (ok)
            {
                // Model clock: second of the edge + exact esp_timer time of the edge (hardware ticks)
                model_publish(edge_sec, llround(edge_local_us * 1000.0),
                              s_clock_ppb.load(std::memory_order_relaxed));
                s_model_rate_valid.store(est_valid, std::memory_order_release);

                // Set the system clock anchor only when the clock has settled
                if (fabs(phase_error_us) <= settledPhaseUs)
                {
                    s_clock_anchor_us.store(llround(edge_local_us), std::memory_order_relaxed);
                    s_clock_comp_valid.store(true, std::memory_order_release);
                }
                else
                {
                    s_clock_comp_valid.store(false, std::memory_order_release);
                }

#if PTP_FREE_RUNNING
                // Measure the PTP clock against this edge and steer its frequency (the clock is not written)
                ptp_servo_update(edge_sec, edge_local_us, ticks, est_valid);
#else
                // Set the PTP clock from the PPS second (not from the system clock, whose slew is still pending)
                set_hardware_clock_from_pps(edge_sec, edge_local_us);
#endif
                s_ntp_reference_time_64 = get_current_time_in_ntp64_format();
                s_ntp_reference_valid = true;
            }
            else
            {
                s_clock_comp_valid.store(false, std::memory_order_release);
            }

            xSemaphoreGive(s_time_mutex);

#if DEBUG_ENABLED
            if ((log_counter++ % 16) == 0)
            {
                ESP_LOGI(TAG, "PPS: ppm=%.4f raw_phase=%.2f us resid=%.2f us jitter_excess=%.2f us d_central=%.2f gap=%lld us",
                         est_ppm, phase_error_us, resid_us, excess_us, d_central, static_cast<long long>(best_gap));
#if PTP_FREE_RUNNING
                ptp_servo_log_status();
#else
                ESP_LOGI(TAG, "PTP: offset after write %d ns (%d reads)",
                         static_cast<int>(s_ptp_offset_dbg_ns.load(std::memory_order_relaxed)),
                         static_cast<int>(s_ptp_offset_dbg_n.load(std::memory_order_relaxed)));
#endif
#if NTP_BASIC_T3_TX_COMPENSATION && NTP_HW_TX_TIMESTAMPS
                ESP_LOGI(TAG, "Basic T3 TX delay: v4 %d ns (%lu) v6 %d ns (%lu)",
                         static_cast<int>(s_tx_delay[0][0].delay_ns.load(std::memory_order_relaxed)),
                         static_cast<unsigned long>(s_tx_delay[0][0].samples.load(std::memory_order_relaxed)),
                         static_cast<int>(s_tx_delay[1][0].delay_ns.load(std::memory_order_relaxed)),
                         static_cast<unsigned long>(s_tx_delay[1][0].samples.load(std::memory_order_relaxed)));
#endif
            }
#endif
            (void)resid_us; // only read by the debug log
        }
        else
        {
            const int64_t now_us = esp_timer_get_time();
            if (last_pps_us != 0 && (now_us - last_pps_us) > ppsTimeoutUs)
            {
#if DEBUG_ENABLED
                if (logged_active)
                    ESP_LOGW(TAG, "PPS discipline inactive (PPS signal unavailable).");
#endif
                if (logged_active)
                {
#if PTP_FREE_RUNNING
                    // Holdover: keep the PTP clock on its estimated rate, stop the offset correction
                    if (xSemaphoreTake(s_time_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
                    {
                        ptp_servo_holdover();
                        xSemaphoreGive(s_time_mutex);
                    }
#endif
                }
                logged_active = false;
                s_pps_discipline_active.store(false);
                sync_state_note_pps_timeout(now_us);

                // Restart the estimators but keep ppb as the holdover value;
                // the compensation keeps running with the last anchor.
                have_last = false;
                hist_count = 0;
                anchor_count = 0;
                anchor_head = 0;
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

#if RBG_LED_KY_016_MODULE_ENABLED || RBG_LED_WS2812_MODULE_ENABLED
    s_gnss_pps_startup_qualification_in_progress.store(true, std::memory_order_release);
#endif

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "Qualifying GNSS and PPS stability before startup.");
#endif

    for (;;)
    {
        int64_t now_us = esp_timer_get_time();
        if (valid_started_us > 0 &&
            now_us - valid_started_us >= static_cast<int64_t>(gnss_Startup_Qualification_Duration_Ms) * 1000LL - 1000000LL)
        {
            display_line(1, "Checking GNSS");
            display_line(2, "Confirming startup");
        }

        if (!current_gnss_timing_is_valid())
        {
            if (valid_started_us > 0)
                TIMING_LOGW(TAG, "GNSS startup qualification restarted: GNSS timing is invalid.");
            valid_started_us = 0;
            last_pps_us = 0;
            stable_pps_edges = 0;
            clear_pps_events();
            display_line(1, "Waiting for GNSS");
            display_line(2, "Timing invalid");
            vTaskDelay(pdMS_TO_TICKS(250));
            continue;
        }

        now_us = esp_timer_get_time();
        if (valid_started_us == 0)
        {
            valid_started_us = now_us;
            display_line(1, "GNSS Time Valid");
            display_line(2, "Qualifying PPS");
        }

        int64_t qualification_deadline_us = valid_started_us +
                                            static_cast<int64_t>(gnss_Startup_Qualification_Duration_Ms) * 1000LL;
        PpsCaptureEvent event{};
        if (wait_for_pps_capture_event(&event, 0))
        {
            // UBX polling can delay this task; qualify the captured edges, not the polling intervals.
            const int64_t edge_us = event.approximate_edge_us;
            if (last_pps_us == 0 ||
                ((edge_us - last_pps_us) >= gnss_Startup_Min_PPS_Interval_Us &&
                 (edge_us - last_pps_us) <= gnss_Startup_Max_PPS_Interval_Us))
            {
                stable_pps_edges++;
            }
            else
            {
                stable_pps_edges = 1;
            }
            last_pps_us = edge_us;
        }

        if (last_pps_us > 0 && now_us - last_pps_us > gnss_Startup_Max_PPS_Interval_Us)
        {
            last_pps_us = 0;
            stable_pps_edges = 0;
        }

        if (now_us >= qualification_deadline_us)
        {
            if (stable_pps_edges >= gnss_Startup_Qualification_PPS_Edges)
            {
                TIMING_LOGI(TAG, "GNSS and PPS startup qualification complete after %lu stable PPS edges.",
                            static_cast<unsigned long>(stable_pps_edges));
#if RBG_LED_KY_016_MODULE_ENABLED || RBG_LED_WS2812_MODULE_ENABLED
                s_gnss_pps_startup_qualification_in_progress.store(false, std::memory_order_release);
#endif
                return true;
            }

            TIMING_LOGW(TAG, "GNSS startup qualification restarted: only %lu stable PPS edges.",
                        static_cast<unsigned long>(stable_pps_edges));
            valid_started_us = now_us;
            last_pps_us = 0;
            stable_pps_edges = 0;
            clear_pps_events();
        }

        display_line(1, "PPS Stabilizing");
        if (stable_pps_edges >= gnss_Startup_Qualification_PPS_Edges)
        {
            now_us = esp_timer_get_time();
            int64_t remaining_us = qualification_deadline_us - now_us;
            if (remaining_us > 0)
            {
                char dwell_line[lcdColumns + 1];
                snprintf(dwell_line,
                         sizeof(dwell_line),
                         "Dwelling for %u secs",
                         static_cast<unsigned int>(std::min<int64_t>((remaining_us + 999999LL) / 1000000LL, 99)));
                display_line(2, dwell_line);
            }
        }
        else
        {
            char pps_edges_line[lcdColumns + 1];
            snprintf(pps_edges_line,
                     sizeof(pps_edges_line),
                     "Edges %u of %u",
                     static_cast<unsigned int>(std::min<uint32_t>(stable_pps_edges, 999u)),
                     static_cast<unsigned int>(std::min<uint32_t>(gnss_Startup_Qualification_PPS_Edges, 999u)));
            display_line(2, pps_edges_line);
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

#if DEBUG_ENABLED
    if (!uart1_output_set)
        ESP_LOGW(TAG, "GNSS timing output configuration failed. Continuing with the module's current output settings.");
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
    if (ntp_load_shedding_active() || s_mqtt_ntp_event_queue == nullptr ||
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
    bool too_large = false;
    while ((read = fread(buffer, 1, sizeof(buffer), file)) > 0)
    {
        if (payload.size() + read > MQTT_MAX_REPORT_SIZE)
        {
            too_large = true; // corrupted or foreign file: do not let it eat up the heap
            break;
        }
        payload.append(buffer, read);
    }
    bool read_failed = ferror(file) != 0;
    fclose(file);
    if (too_large)
    {
        ++s_mqtt_queued_messages_discarded;
        TIMING_LOGW(TAG, "MQTT queued report exceeds the report buffer; discarding it");
        if (unlink(files.front().c_str()) == 0)
            mqtt_tf_refresh_queue_count();
        else
            TIMING_LOGW(TAG, "MQTT oversized queued report could not be deleted: errno %d", errno);
        return;
    }
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

// Appends formatted text to a report buffer. Unlike "len += snprintf(...)" it never moves len past the end of the
// buffer (snprintf returns the length it would have written, which made the next call write out of bounds).
// A length equal to payload_size marks failure; callers must not publish the partial JSON.
static size_t json_append(char *payload, size_t payload_size, size_t len, const char *format, ...)
{
    if (len >= payload_size)
        return payload_size;
    va_list arguments;
    va_start(arguments, format);
    const int written = vsnprintf(payload + len, payload_size - len, format, arguments);
    va_end(arguments);
    if (written < 0)
        return payload_size;
    const size_t end = len + static_cast<size_t>(written);
    return end >= payload_size ? payload_size : end;
}

static bool mqtt_build_report(char *payload, size_t payload_size)
{
    uint32_t queued_messages = static_cast<uint32_t>(s_mqtt_queued_messages_count.load());
    uint32_t queued_messages_discarded = s_mqtt_queued_messages_discarded;
    s_mqtt_queued_messages_discarded = 0;
    uint32_t most_requests_per_second = s_ntp_most_requests_per_second.exchange(0, std::memory_order_relaxed);
    const bool load_shedding_this_period = s_ntp_shed_during_period.exchange(false, std::memory_order_relaxed);

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

    len = json_append(payload, payload_size, len, "{");

    len = json_append(payload, payload_size, len, "\"current\":{");
    len = json_append(payload, payload_size, len, "\"time\":\"%s\",", publishing_date_and_time);
    len = json_append(payload, payload_size, len, "\"uptime\":%lu,", (unsigned long)(esp_timer_get_time() / 1000000LL));
    len = json_append(payload, payload_size, len, "\"ethernet_up\":%s,", s_ethernet_connected.load() ? "true" : "false");

    // to make this easy on the user - the following have been simplified for reporting so the user sees true when things are ok and false when they are not
    bool current_gnss_locked = s_gnss_locked.load();                 // Indicates whether GNSS currently has a satellite lock
    bool current_gnss_timing_valid = s_ntp_gnss_timing_valid.load(); // Tracks if GNSS timing data is valid
    bool current_gnss_valid = !s_ntp_gnss_invalid.load();            // Signals GNSS timing missing, expired, or unusable
    bool current_sync_fresh = !s_ntp_sync_stale.load();              // Shows if last successful sync is too old
    bool current_sanity_matched = !s_ntp_sanity_mismatch.load();     // Marks detected mismatch between GNSS time and sanity checks (if the gnss time is outside the timeframe that it should be)
    bool current_gnss_ok = current_gnss_locked && current_gnss_timing_valid && current_gnss_valid && current_sync_fresh && current_sanity_matched;

    len = json_append(payload, payload_size, len, "\"gnss_synchronized\":%s,", current_gnss_ok ? "true" : "false");
    if (!current_gnss_ok)
    {
        len = json_append(payload, payload_size, len, "\"gnss_synchronized_indicators\":{");
        len = json_append(payload, payload_size, len, "\"locked\":%s,", current_gnss_locked ? "true" : "false");
        len = json_append(payload, payload_size, len, "\"timing\":%s,", current_gnss_timing_valid ? "true" : "false");
        len = json_append(payload, payload_size, len, "\"gnss_valid\":%s,", current_gnss_valid ? "true" : "false");
        len = json_append(payload, payload_size, len, "\"sync_fresh\":%s,", current_sync_fresh ? "true" : "false");
        len = json_append(payload, payload_size, len, "\"sanity_check_passed\":%s", current_sanity_matched ? "true" : "false");
        len = json_append(payload, payload_size, len, "},");
    }

    // to make this easy on the user - the following have been simplified for reporting so the user sees true when things are ok and false when they are not
    bool current_pps_active = s_ntp_pps_active.load();                   // Is PPS present right now?
    bool current_pps_discipline_active = s_pps_discipline_active.load(); // Is the PPS disciplining algorithm engaged?
    bool current_pps_synchronized = !s_ntp_pps_missing.load();           // Is PPS missing in a way that constitutes a synchronization fault?
    bool current_pps_ok = current_pps_active && current_pps_discipline_active && current_pps_synchronized;

    len = json_append(payload, payload_size, len, "\"pps_disciplined\":%s,", current_pps_ok ? "true" : "false");
    if (!current_pps_ok)
    {
        len = json_append(payload, payload_size, len, "\"pps_disciplined_indicators\":{");
        len = json_append(payload, payload_size, len, "\"pps_signals_present\":%s,", current_pps_active ? "true" : "false");
        len = json_append(payload, payload_size, len, "\"discipline_active\":%s,", current_pps_discipline_active ? "true" : "false");
        len = json_append(payload, payload_size, len, "\"pps_synchronized\":%s", current_pps_synchronized ? "true" : "false");
        len = json_append(payload, payload_size, len, "},");
    }

    len = json_append(payload, payload_size, len, "\"satellites\":%u", (unsigned int)s_satellite_count.load());

#if MQTT_MEMORY_REPORTING_ENABLED
    len = json_append(payload, payload_size, len, ",");
    len = json_append(payload, payload_size, len, "\"memory\":{");
    // len = json_append(payload, payload_size, len, "\"malloc_cap_8bit\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT));
    // len = json_append(payload, payload_size, len, "\"malloc_cap_32bit\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_32BIT));
    // len = json_append(payload, payload_size, len, "\"malloc_cap_internal\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    len = json_append(payload, payload_size, len, "\"malloc_cap_dma\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
    // len = json_append(payload, payload_size, len, "\"malloc_cap_spiram\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    // len = json_append(payload, payload_size, len, "\"malloc_cap_default\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    len = json_append(payload, payload_size, len, "\"free_heap\":%lu,", (unsigned long)esp_get_free_heap_size());
    len = json_append(payload, payload_size, len, "\"minimum_free_heap\":%lu,", (unsigned long)esp_get_minimum_free_heap_size());
    len = json_append(payload, payload_size, len, "\"largest_free_8bit_block\":%lu", (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    len = json_append(payload, payload_size, len, "}},");

#else
    len = json_append(payload, payload_size, len, "},");
#endif

    if (MQTT_QOS > 0)
    {
        len = json_append(payload, payload_size, len, "\"queued_messages\":{");
        len = json_append(payload, payload_size, len, "\"held\":%lu,", (unsigned long)queued_messages);
        len = json_append(payload, payload_size, len, "\"discarded\":%lu", (unsigned long)queued_messages_discarded);
        len = json_append(payload, payload_size, len, "},");
    };

    len = json_append(payload, payload_size, len, "\"this_period\":{");
    len = json_append(payload, payload_size, len, "\"ethernet_up_secs\":%lld,", (long long)(link_up_us / 1000000LL));
    len = json_append(payload, payload_size, len, "\"pps_pulses\":%lu,", (unsigned long)s_pps_pulses.exchange(0));
    len = json_append(payload, payload_size, len, "\"gnss_locked_secs\":%lu,", (unsigned long)gnss_locked_seconds_this_period());
    len = json_append(payload, payload_size, len, "\"satellites\":{");
    len = json_append(payload, payload_size, len, "\"min\":%u,", (unsigned int)satellite_min);
    len = json_append(payload, payload_size, len, "\"max\":%u", (unsigned int)satellite_max);
    len = json_append(payload, payload_size, len, "},");
    len = json_append(payload, payload_size, len, "\"ntp\":{");
    len = json_append(payload, payload_size, len, "\"requests\":{");
    const uint32_t authenticated_valid = s_ntp_authenticated_valid_requests.exchange(0);
    const uint32_t authenticated_invalid = s_ntp_authenticated_invalid_requests.exchange(0);
    const uint32_t unauthenticated_valid = s_ntp_unauthenticated_valid_requests.exchange(0);
    const uint32_t unauthenticated_invalid = s_ntp_unauthenticated_invalid_requests.exchange(0);
    s_ntp_valid_requests.exchange(0);
    s_ntp_invalid_requests.exchange(0);

#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED && MQTT_DISTINGUISH_AUTHENTICATED_REPORTING_ENABLED
    len = json_append(payload, payload_size, len, "\"authenticated\":{\"valid\":%lu,\"invalid\":%lu},", (unsigned long)authenticated_valid, (unsigned long)authenticated_invalid);
    len = json_append(payload, payload_size, len, "\"unauthenticated\":{\"valid\":%lu,\"invalid\":%lu},", (unsigned long)unauthenticated_valid, (unsigned long)unauthenticated_invalid);
#else
    len = json_append(payload, payload_size, len, "\"valid\":%lu,", (unsigned long)(authenticated_valid + unauthenticated_valid));
    len = json_append(payload, payload_size, len, "\"invalid\":%lu,", (unsigned long)(authenticated_invalid + unauthenticated_invalid));
#endif

    len = json_append(payload, payload_size, len, "\"load_shedding\":%s,", load_shedding_this_period ? "true" : "false");
    len = json_append(payload, payload_size, len, "\"telemetry_dropped\":%lu,", (unsigned long)s_ntp_telemetry_events_dropped.exchange(0));
    len = json_append(payload, payload_size, len, "\"max_per_second\":%lu", (unsigned long)most_requests_per_second);
    len = json_append(payload, payload_size, len, "},");
    len = json_append(payload, payload_size, len, "\"responses\":{");
    len = json_append(payload, payload_size, len, "\"synchronized_and_disciplined\":%lu,", (unsigned long)s_ntp_responses_synchronized_and_disciplined.exchange(0));
    len = json_append(payload, payload_size, len, "\"gnss_unsynchronized\":%lu,", (unsigned long)s_ntp_responses_gnss_unsynchronized.exchange(0));
    len = json_append(payload, payload_size, len, "\"pps_undisciplined\":%lu", (unsigned long)s_ntp_responses_pps_undisciplined.exchange(0));
    len = json_append(payload, payload_size, len, "}");

#if MQTT_CLIENT_REPORTING_ENABLED
    len = json_append(payload, payload_size, len, "},");
    len = json_append(payload, payload_size, len, "\"clients\":[");
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
            len = json_append(payload, payload_size, len, "%s{\"address\":\"%s\",\"requests\":%lu}",
                              first_client ? "" : ",", address_text,
                              static_cast<unsigned long>(s_mqtt_clients[index].requests));
            first_client = false;
        }
        s_mqtt_client_count = 0;
        xSemaphoreGive(s_mqtt_stats_mutex);
    }
    len = json_append(payload, payload_size, len, "],");
    len = json_append(payload, payload_size, len, "\"clients_overflown\":%s", s_mqtt_client_table_overflown.exchange(false) ? "true" : "false");
#else
    len = json_append(payload, payload_size, len, "}");
#endif

#if MQTT_HISTORICAL_REPORTING_ENABLED
    len = json_append(payload, payload_size, len, "},");
    len = json_append(payload, payload_size, len, "\"historical\":{");
    len = json_append(payload, payload_size, len, "\"gnss_receiver_last\":{");
    len = json_append(payload, payload_size, len, "\"synchronized_and_disciplined\":\"%s\",", last_synchronized_and_disciplined);
    len = json_append(payload, payload_size, len, "\"gnss_unsynchronized\":\"%s\",", last_gnss_unsynchronized);
    len = json_append(payload, payload_size, len, "\"pps_undisciplined\":\"%s\"", last_pps_undisciplined);
    len = json_append(payload, payload_size, len, "}");
#endif

    len = json_append(payload, payload_size, len, "}");
    len = json_append(payload, payload_size, len, "}");

    if (len >= payload_size)
    {
        if (payload_size != 0)
            payload[0] = '\0';
        TIMING_LOGW(TAG, "MQTT report exceeds the buffer or formatting failed; report not published");
        return false;
    }
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "Published:");
    ESP_LOGI(TAG, "%s", payload);
#endif
    return true;
}

static void mqtt_publish_final_report()
{
    if (!s_mqtt_connected.load())
        return;

    if (!mqtt_build_report(s_mqtt_final_payload, sizeof(s_mqtt_final_payload)))
        return;

    s_mqtt_restart_publish_id.store(-1);
    s_mqtt_restart_publish_completed.store(false);
    int message_id = esp_mqtt_client_publish(s_mqtt_client, s_mqtt_report_topic, s_mqtt_final_payload, 0, MQTT_QOS, MQTTBrokerRetain);
    if (message_id < 0)
        return;

    s_mqtt_restart_publish_id.store(message_id);
    TickType_t wait_started = xTaskGetTickCount();
    TickType_t wait_timeout = pdMS_TO_TICKS(MQTT_RESTART_PUBLISH_TIMEOUT_MS);
    while (!s_mqtt_restart_publish_completed.load() && (xTaskGetTickCount() - wait_started) < wait_timeout)
        vTaskDelay(pdMS_TO_TICKS(10));

    s_mqtt_restart_publish_id.store(-1);
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

    bool delivered_or_queued = false;
    if (MQTT_QOS == 0)
    {
        delivered_or_queued = esp_mqtt_client_publish(s_mqtt_client, s_mqtt_report_topic, payload, 0, MQTT_QOS, MQTTBrokerRetain) >= 0;
    }
    else
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
        }
        delivered_or_queued = s_mqtt_restart_publish_completed.load();
        s_mqtt_restart_publish_id.store(-1);
        if (!delivered_or_queued)
            delivered_or_queued = mqtt_enqueue_report(payload);
    }

    if (delivered_or_queued)
    {
        result = nvs_erase_key(handle, restart_NVS_KEY_PENDING);
        if (result == ESP_OK)
            result = nvs_commit(handle);
    }
    nvs_close(handle);
    return delivered_or_queued && result == ESP_OK;
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
            if (mqtt_build_report(s_mqtt_payload, sizeof(s_mqtt_payload)) && !mqtt_publish_report(s_mqtt_payload))
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

static void restart_deadline_callback(void *)
{
    esp_restart();
}

static void controlled_restart(const char *reason)
// Publish a controlled restart notification via MQTT or queue it if not connected.
{
    esp_timer_create_args_t deadline_args{};
    deadline_args.callback = restart_deadline_callback;
    deadline_args.name = "restart_deadline";
    esp_timer_handle_t deadline_timer = nullptr;
    esp_err_t deadline_result = esp_timer_create(&deadline_args, &deadline_timer);
    if (deadline_result == ESP_OK)
        deadline_result = esp_timer_start_once(deadline_timer, 5000000ULL);
    if (deadline_result != ESP_OK)
    {
        TIMING_LOGE(TAG, "Unable to arm restart deadline: %s", esp_err_to_name(deadline_result));
        esp_restart();
        return;
    }

    ESP_LOGW(TAG, "Controlled restart requested: %s", reason);
#if DEBUG_ENABLED
    vTaskDelay(pdMS_TO_TICKS(200));
#endif

#if MQTT_ENABLED

    mqtt_publish_final_report();

    char payload[128] = "";
    snprintf(payload, sizeof(payload), "{\"event\":\"controlled_restart\",\"reason\":\"%s\"}", reason);

    bool published = false;
    if (s_mqtt_connected.load())
    {
        if (MQTT_QOS == 0)
        {
            published = esp_mqtt_client_publish(s_mqtt_client, s_mqtt_report_topic, payload, 0, MQTT_QOS, MQTTBrokerRetain) >= 0;
            if (published)
                vTaskDelay(pdMS_TO_TICKS(MQTT_RESTART_PUBLISH_TIMEOUT_MS));
        }
        else
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
    }

    if (published)
    {
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "Published from controlled restart:");
        ESP_LOGI(TAG, "%s", payload);
        vTaskDelay(pdMS_TO_TICKS(200));
#endif
    }
    else
    {

        if (MQTT_QOS > 0)
        {
            if (!mqtt_enqueue_report(payload))
            {
#if DEBUG_ENABLED
                ESP_LOGW(TAG, "Failed to enqueue restart report, falling back to NVS storage");
                vTaskDelay(pdMS_TO_TICKS(200));
#endif

                nvs_handle_t handle = 0;
                esp_err_t result = nvs_open(restart_NVS_NAMESPACE, NVS_READWRITE, &handle);
                if (result == ESP_OK)
                {
                    result = nvs_set_str(handle, restart_NVS_KEY_PENDING, payload);
                    if (result == ESP_OK)
                        result = nvs_commit(handle);
                    nvs_close(handle);
                }

#if DEBUG_ENABLED

                if (result != ESP_OK)
                {
                    ESP_LOGE(TAG, "Restart notification could not be published or queued:");
                    ESP_LOGE(TAG, "%s", payload);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }

#endif
            }
        }
    }
#endif

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "Restarting now.");
    vTaskDelay(pdMS_TO_TICKS(200));
#endif
    esp_restart();

    return;
}

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

    // static assertions report the error if the condition is not true at compile time
    static_assert(MQTT_QOS >= 0, "MQTT_QOS must be 0, 1, or 2");
    static_assert(MQTT_QOS <= 2, "MQTT_QOS must be 0, 1, or 2");

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
    if (xTaskCreatePinnedToCore(mqtt_service_task, "mqtt_service", MQTT_Service_Task_Stack_Size, nullptr, 5, nullptr, 0) != pdPASS)
        s_mqtt_setup_failed.store(true);
#endif
}

static void ethernet_event_handler(void *, esp_event_base_t, int32_t event, void *event_data)
{
    if (event_data == nullptr || *static_cast<esp_eth_handle_t *>(event_data) != s_eth_handle)
        return;

    switch (event)
    {
    case ETHERNET_EVENT_START:
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "Ethernet driver started");
#endif
        display_line(1, "Ethernet started");
        break;
    case ETHERNET_EVENT_CONNECTED:
        s_ethernet_connected.store(true);
        if (IPV6_ENABLED)
        {
            const esp_err_t result = esp_netif_create_ip6_linklocal(s_eth_netif);
            if (result != ESP_OK)
                ESP_LOGW(TAG, "Unable to create Ethernet IPv6 link-local address: %s", esp_err_to_name(result));
            refresh_network_ipv6_addresses();
        }
#if MQTT_ENABLED
        s_eth_link_connected_us.store(esp_timer_get_time());
        if (s_mqtt_has_connected.load(std::memory_order_acquire))
            s_mqtt_disconnected_since_us.store(esp_timer_get_time(), std::memory_order_release);
#endif
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "Ethernet link connected");
#endif
        xEventGroupSetBits(s_net_event_group, ETH_CONNECTED_BIT);
        display_line(1, "Ethernet connected");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        if (!s_ethernet_recovery_in_progress.load(std::memory_order_acquire))
            s_last_physical_link_down_us.store(esp_timer_get_time(), std::memory_order_relaxed);
        [[fallthrough]];
    case ETHERNET_EVENT_STOP:
        s_ethernet_connected.store(false);
        s_first_ntp_input_us.store(0, std::memory_order_relaxed);
        s_last_ntp_response_us.store(0, std::memory_order_relaxed);
#if MQTT_ENABLED
        mqtt_note_ethernet_disconnected();
#endif
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "Ethernet %s", event == ETHERNET_EVENT_STOP ? "driver stopped" : "link disconnected");
#endif
        xEventGroupClearBits(s_net_event_group, ETH_CONNECTED_BIT | ETH_GOT_IP_BIT | ETH_GOT_IP6_BIT | ETH_GOT_ROUTABLE_IP6_BIT);
        clear_network_addresses();
        display_line(1, event == ETHERNET_EVENT_STOP ? "Ethernet stopped" : "Ethernet disconnect");
        display_line(3, "");
        break;
    default:
        break;
    }
}

static void ethernet_ip_event_handler(void *, esp_event_base_t, int32_t event, void *event_data)
{
    if (event_data == nullptr)
        return;

    switch (event)
    {
    case IP_EVENT_ETH_GOT_IP:
    {
        const auto &info = *static_cast<ip_event_got_ip_t *>(event_data);
        if (info.esp_netif != s_eth_netif)
            break;
        if (!IPV4_ENABLED)
            break;
        s_ipv4_ever_acquired.store(true, std::memory_order_relaxed);
        set_network_ipv4_address(info.ip_info.ip, info.ip_info.netmask);
#if MQTT_ENABLED
        if (s_mqtt_has_connected.load(std::memory_order_acquire) && !s_mqtt_connected.load(std::memory_order_acquire))
            s_mqtt_disconnected_since_us.store(esp_timer_get_time(), std::memory_order_release);
#endif
#if DEBUG_ENABLED
        ESP_LOGI(TAG,
                 "Ethernet IPv4 acquired: ip=" IPSTR ", mask=" IPSTR ", gw=" IPSTR,
                 IP2STR(&info.ip_info.ip),
                 IP2STR(&info.ip_info.netmask),
                 IP2STR(&info.ip_info.gw));
#endif
        xEventGroupSetBits(s_net_event_group, ETH_GOT_IP_BIT);
        display_selected_ip_address(static_cast<int>(time(nullptr) % 10));
        break;
    }
    case IP_EVENT_GOT_IP6:
    {
        const auto &info = *static_cast<ip_event_got_ip6_t *>(event_data);
        if (info.esp_netif != s_eth_netif || !IPV6_ENABLED)
            break;
        const bool routable_ipv6_acquired = set_network_ipv6_address(info.ip6_info.ip);
        refresh_network_ipv6_addresses();
#if MQTT_ENABLED
        if (s_mqtt_has_connected.load(std::memory_order_acquire) && !s_mqtt_connected.load(std::memory_order_acquire))
            s_mqtt_disconnected_since_us.store(esp_timer_get_time(), std::memory_order_release);
#endif
#if DEBUG_ENABLED
        const network_address_snapshot_t addresses = get_network_address_snapshot();
        ESP_LOGI(TAG,
                 "Ethernet IPv6 acquired: %s",
                 routable_ipv6_acquired ? addresses.ipv6 : "link-local address");
#endif
        xEventGroupSetBits(s_net_event_group, ETH_GOT_IP6_BIT);
        if (routable_ipv6_acquired)
            xEventGroupSetBits(s_net_event_group, ETH_GOT_ROUTABLE_IP6_BIT);
        display_selected_ip_address(static_cast<int>(time(nullptr) % 10));
        break;
    }
    case IP_EVENT_ETH_LOST_IP:
        if (static_cast<ip_event_got_ip_t *>(event_data)->esp_netif != s_eth_netif)
            break;
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "Ethernet lost IPv4 address");
#endif
        xEventGroupClearBits(s_net_event_group, ETH_GOT_IP_BIT);
        clear_network_ipv4_address();
        display_line(1, "Ethernet lost IP");
        display_selected_ip_address(static_cast<int>(time(nullptr) % 10));
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

    esp_netif_ip_info_t ip_info{};
    if (esp_netif_str_to_ip4(StaticIPAddress, &ip_info.ip) != ESP_OK)
    {
        ESP_LOGE(TAG, "Configured StaticIPAddress is invalid: %s", StaticIPAddress);
        return false;
    }
    if (esp_netif_str_to_ip4(Gateway, &ip_info.gw) != ESP_OK)
    {
        ESP_LOGE(TAG, "Configured Gateway is invalid: %s", Gateway);
        return false;
    }
    if (esp_netif_str_to_ip4(SubnetMask, &ip_info.netmask) != ESP_OK)
    {
        ESP_LOGE(TAG, "Configured SubnetMask is invalid: %s", SubnetMask);
        return false;
    }

    esp_netif_dns_info_t dns1{}, dns2{};
    dns1.ip.type = ESP_IPADDR_TYPE_V4;
    dns2.ip.type = ESP_IPADDR_TYPE_V4;
    if (PrimaryDNS[0] != '\0')
    {
        if (esp_netif_str_to_ip4(PrimaryDNS, &dns1.ip.u_addr.ip4) != ESP_OK)
        {
            ESP_LOGE(TAG, "Configured PrimaryDNS is invalid: %s", PrimaryDNS);
            return false;
        }
    }
    if (SecondaryDNS[0] != '\0')
    {
        if (esp_netif_str_to_ip4(SecondaryDNS, &dns2.ip.u_addr.ip4) != ESP_OK)
        {
            ESP_LOGE(TAG, "Configured SecondaryDNS is invalid: %s", SecondaryDNS);
            return false;
        }
    }

    const esp_err_t dhcp_result = esp_netif_dhcpc_stop(s_eth_netif);
    if (dhcp_result != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)
        ESP_ERROR_CHECK(dhcp_result);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_eth_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_set_dns_info(s_eth_netif, ESP_NETIF_DNS_MAIN, &dns1));
    ESP_ERROR_CHECK(esp_netif_set_dns_info(s_eth_netif, ESP_NETIF_DNS_BACKUP, &dns2));

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "Static IP configuration applied: ip=%s, mask=%s, gw=%s",
             StaticIPAddress, SubnetMask, Gateway);
#endif
    return true;
}

static void start_ethernet_driver()
{
    ESP_ERROR_CHECK(esp_netif_init());
    const esp_err_t event_loop_result = esp_event_loop_create_default();
    if (event_loop_result != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(event_loop_result);

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.sw_reset_timeout_ms = 1000;
    mac_config.rx_task_stack_size = 4096;
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = EthernetMDCGPIO;
    emac_config.smi_gpio.mdio_num = EthernetMDIOGPIO;
    emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio = EthernetRMIIClockGPIO;
    emac_config.emac_dataif_gpio.rmii.tx_en_num = EthernetRMIITXEnableGPIO;
    emac_config.emac_dataif_gpio.rmii.txd0_num = EthernetRMIITX0GPIO;
    emac_config.emac_dataif_gpio.rmii.txd1_num = EthernetRMIITX1GPIO;
    emac_config.emac_dataif_gpio.rmii.crs_dv_num = EthernetRMIICRSDVGPIO;
    emac_config.emac_dataif_gpio.rmii.rxd0_num = EthernetRMIIRX0GPIO;
    emac_config.emac_dataif_gpio.rmii.rxd1_num = EthernetRMIIRX1GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    ESP_ERROR_CHECK(mac != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = EthernetPHYAddress;
    phy_config.reset_gpio_num = EthernetPHYResetGPIO;
    // IDF 6 uses the IEEE 802.3 driver for IP101 (as did Arduino's IDF 6 alias).
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_config);
    ESP_ERROR_CHECK(phy != nullptr ? ESP_OK : ESP_ERR_NO_MEM);
    const esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &s_eth_handle));

    const esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_config);
    ESP_ERROR_CHECK(s_eth_netif != nullptr ? ESP_OK : ESP_ERR_NO_MEM);
    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(s_eth_handle);
    ESP_ERROR_CHECK(glue != nullptr ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, glue));
    ESP_ERROR_CHECK(esp_netif_set_hostname(s_eth_netif, DeviceName));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, ethernet_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, ethernet_ip_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_LOST_IP, ethernet_ip_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, ethernet_ip_event_handler, nullptr));

    // Configure static addressing before link-up so DHCP cannot race it.
    configure_static_ip();
    ESP_ERROR_CHECK(esp_eth_start(s_eth_handle));
}

static void wait_for_startup_ethernet_readiness()
{
    if (PreferIPvX == 1)
    {
        xEventGroupWaitBits(s_net_event_group, ETH_GOT_IP_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
        return;
    }

    const int64_t wait_started_us = esp_timer_get_time();
    for (;;)
    {
        refresh_network_ipv6_addresses();
        if ((xEventGroupGetBits(s_net_event_group) & ETH_GOT_ROUTABLE_IP6_BIT) != 0)
            return;

        const bool timeout_elapsed = esp_timer_get_time() - wait_started_us >=
                                     static_cast<int64_t>(ROUTABLE_IPV6_ACQUISITION_TIMEOUT_MS) * 1000;
        if (timeout_elapsed && s_pps_discipline_active.load(std::memory_order_acquire))
            return;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void setup_ethernet()
{
    if (s_net_event_group == nullptr)
        s_net_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(s_net_event_group != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    clear_network_addresses();
    xEventGroupClearBits(s_net_event_group, ETH_CONNECTED_BIT | ETH_GOT_IP_BIT | ETH_GOT_IP6_BIT | ETH_GOT_ROUTABLE_IP6_BIT);

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "Starting native Ethernet with phy_addr=%d, mdc=%d, mdio=%d, reset=%d",
             EthernetPHYAddress, EthernetMDCGPIO, EthernetMDIOGPIO, EthernetPHYResetGPIO);
#endif

    start_ethernet_driver();

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

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "Waiting for Ethernet %s address%s...",
             StaticIPAddress[0] != '\0' ? "static" : "DHCP",
             PreferIPvX == 1 ? "" : " and routable IPv6");
#endif
    wait_for_startup_ethernet_readiness();
#if DEBUG_ENABLED
    const network_address_snapshot_t addresses = get_network_address_snapshot();
    ESP_LOGI(TAG, "Ethernet setup complete, current address: %s", addresses.selected[0] == '\0' ? "<none>" : addresses.selected);
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

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "Debug: Enabled");
#else
    ESP_LOGW(TAG, "Debug: Disabled");
#endif

#if RBG_LED_KY_016_MODULE_ENABLED || RBG_LED_WS2812_MODULE_ENABLED
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

#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
    ESP_LOGI(TAG, "Symmetric-key authentication support: Enabled");
#else
    ESP_LOGW(TAG, "Symmetric-key authentication support: Disabled");
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

#if MQTT_DISTINGUISH_AUTHENTICATED_REPORTING_ENABLED
    ESP_LOGI(TAG, "MQTT distinguish authenticated and unauthenticated reporting: Enabled");
#else
    ESP_LOGW(TAG, "MQTT distinguish authenticated and unauthenticated reporting: Disabled");
#endif

    ESP_LOGI(TAG, "");
}

void setup_NVS_storage(void)
{
    if (!initialize_nvs_storage())
    {
        ESP_LOGE(TAG, "NVS initialization failed. GNSS module settings persistence is unavailable.");
        return;
    }

#if PPS_ISR_LATENCY_AUTO
    load_pps_isr_latency_from_nvs();
#endif
#if PTP_LATENCY_AUTO
    load_ptp_write_latency_from_nvs();
#endif
}

void setup_mutexes_and_semaphores(void)
{

    s_time_mutex = xSemaphoreCreateMutex();
    s_pps_semaphore = xSemaphoreCreateBinary();
    s_pps_timestamp_queue = xQueueCreate(1, sizeof(PpsCaptureEvent));
    s_pps_sync_timestamp_queue = xQueueCreate(1, sizeof(PpsCaptureEvent));
    s_ote_mutex = xSemaphoreCreateMutex();
    s_sync_state_mutex = xSemaphoreCreateMutex();
    s_network_address_mutex = xSemaphoreCreateMutex();

#if LIQUID_CRYSTAL_DISPLAY_ENABLED
    s_lcd_mutex = xSemaphoreCreateMutex();
#endif

#if CALCULATE_STACK_SIZES_ENABLED
    s_task_stack_usage_mutex = xSemaphoreCreateMutex();
#endif
}

static void setup_the_RGB_LED()
{

#if RBG_LED_KY_016_MODULE_ENABLED && RBG_LED_WS2812_MODULE_ENABLED
#error "KY-016 and WS2812 RGB LED modules cannot both be enabled at the same time"
#endif

#if RBG_LED_KY_016_MODULE_ENABLED
    ledc_timer_config_t timer_config{};
    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.duty_resolution = LEDC_TIMER_8_BIT;
    timer_config.timer_num = LEDC_TIMER_0;
    timer_config.freq_hz = 5000;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    const int led_pins[] = {LEDRedPin, LEDGreenPin, LEDBluePin};
    for (int channel = 0; channel < 3; ++channel)
    {
        ledc_channel_config_t channel_config{};
        channel_config.gpio_num = led_pins[channel];
        channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
        channel_config.channel = static_cast<ledc_channel_t>(channel);
        channel_config.intr_type = LEDC_INTR_DISABLE;
        channel_config.timer_sel = LEDC_TIMER_0;
        channel_config.duty = 0;
        channel_config.hpoint = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    }
#endif
#if RBG_LED_WS2812_MODULE_ENABLED
    static_assert(RGB_LED_Brightness_Percent >= 0 && RGB_LED_Brightness_Percent <= 100);
    s_ws2812_brightness = static_cast<uint8_t>((255 * RGB_LED_Brightness_Percent + 50) / 100);

    led_strip_config_t strip_config{};
    strip_config.strip_gpio_num = WS2812DataPin;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format.format.g_pos = 0;
    strip_config.color_component_format.format.r_pos = 1;
    strip_config.color_component_format.format.b_pos = 2;
    strip_config.color_component_format.format.num_components = 3;

    led_strip_rmt_config_t rmt_config{};
    rmt_config.resolution_hz = 10000000;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_ws2812_strip));
#endif
#if RBG_LED_KY_016_MODULE_ENABLED || RBG_LED_WS2812_MODULE_ENABLED
    control_RGB_LED(LED_startup, true);
#endif
}

void setup_the_LCD(void)
{

#if LIQUID_CRYSTAL_DISPLAY_ENABLED

    // static assertions report the error if the condition is not true at compile time
    static_assert(sizeof(lcdTopLineTitle) - 1 > 0, "lcdTopLineTitle is missing or not populated in settings, recommend value is 'ESP32 Time Server'");
    static_assert(sizeof(lcdTopLineTitle) - 1 <= 17, "lcdTopLineTitle must be at most 17 characters");

    if (setup_lcd() == ESP_OK)
    {
        display_line(0, lcdTopLineTitle);
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

static void setup_the_button()
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

    // std::string MACToBeUsed = configure_mac_address();
    MACToBeUsed = configure_mac_address();
    ESP_LOGI(TAG, "The MAC address that will be used for this device is: %s", MACToBeUsed.c_str());

    display_line(1, "Connecting Ethernet");
    display_line(2, "");
    setup_ethernet();
    display_selected_ip_address(static_cast<int>(time(nullptr) % 10));
}

static void ethernet_setup_task(void *parameter)
{
    setup_ethernet_connection();
    xTaskNotifyGive(static_cast<TaskHandle_t>(parameter));
    vTaskDelete(nullptr);
}

static bool start_ethernet_setup_task(TaskHandle_t startup_task)
{
    if (xTaskCreatePinnedToCore(ethernet_setup_task,
                                "ethernet_setup",
                                Ethernet_Setup_Task_Stack_Size,
                                startup_task,
                                16,
                                nullptr,
                                tskNO_AFFINITY) == pdPASS)
        return true;

    ESP_LOGE(TAG, "Unable to start Ethernet setup task");
    return false;
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

    for (;;)
    {
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
            controlled_restart("over_the_ethernet_update");

#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(OTE_Service);
#endif
    }
}

#endif

void setup_for_ote_updates()
{
#if OTE_UPDATES_ENABLED

    display_line(1, "Setup OTE");
    display_line(2, "");
    if (s_ote_mutex == nullptr ||
        xTaskCreatePinnedToCore(ote_service_task, "ote_service", OTE_Service_Task_Stack_Size, nullptr, 5, nullptr, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Unable to start OTE reboot service");
        display_line(2, "OTE setup failed");
        return;
    }
    const native_ote_callbacks_t callbacks{OTE_set_running_state, OTE_set_failure_state, OTE_set_success_state};
    const esp_err_t result = native_ote_start(s_eth_netif, callbacks);
    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to start native OTE: %s", esp_err_to_name(result));
        OTE_set_failure_state("OTE setup failed");
    }

#endif
}

// Reads the GNSS date and time together with the PPS edge it belongs to.
// The candidate time is the UTC second of the PPS edge at pps_release_time_us. The label is taken only from data that
// was received after this edge and well before the next one, so it cannot belong to another second:
//  - NMEA: the serial receive buffer is emptied right after the edge, then the first valid RMC sentence is used
//    (the sentences of a second follow its time pulse; old sentences in the buffer could describe an earlier second)
//  - UBX: pre-edge data is drained at the edge, then automatic NAV-PVT output is collected.
//    The receiver may service an on-demand poll only at its next navigation update, so polling
//    300 ms after the edge does not reliably label that edge.
// Data that is complete later than labelDeadlineUs after the edge is rejected.
static bool acquire_sync_candidate(sync_candidate_t *candidate)
{
    static constexpr int64_t labelDeadlineUs = 950000;
    static constexpr uint16_t ubxPvtTimeoutMs = 1500;

    if (candidate == nullptr)
        return false;

    *candidate = sync_candidate_t{};
    candidate->used_nmea_fallback = s_use_nmea_fallback;
    const int64_t attempt_start_us = esp_timer_get_time();
    const bool initial_sync = !s_time_has_been_set.load();

    if (initial_sync)
    {
        display_line(1, "Getting date & time");
        display_line(2, "Waiting for PPS");
    }
    TIMING_LOGI(TAG, "GNSS sync: waiting for a new PPS edge (%s).", s_use_nmea_fallback ? "NMEA" : "UBX");
    clear_pps_events();
    PpsCaptureEvent capture_event{};
    if (!wait_for_pps_capture_event(&capture_event, pdMS_TO_TICKS(1500)))
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "%s mode is running without PPS.", s_use_nmea_fallback ? "NMEA fallback" : "UBX");
#endif
        candidate->failures.pps_missing = true;
        if (initial_sync)
            display_line(2, "PPS wait timed out");
        return false;
    }
    const int64_t edge_us = capture_event.approximate_edge_us;
    if (!s_use_nmea_fallback)
    {
        // Drain pre-edge data before logging/display work can delay collection of the new solution.
        (void)s_gnss.checkUblox();
        s_gnss.flushPVT();
    }
    TIMING_LOGI(TAG, "GNSS sync: PPS captured at %lld us; age %lld us.",
                static_cast<long long>(edge_us),
                static_cast<long long>(esp_timer_get_time() - edge_us));

    if (initial_sync)
        display_line(2, "Reading GNSS time");
    if (s_use_nmea_fallback)
    {
        // Discard what was received before the edge
        for (int discarded = 0; discarded < 4096 && s_gnss_serial.available() > 0; ++discarded)
            (void)s_gnss_serial.read();

        nmea_rmc_time_t nmea_time{};
        const int64_t remaining_us = labelDeadlineUs - (esp_timer_get_time() - edge_us);
        if (remaining_us <= 20000 || !wait_for_nmea_rmc_time(&nmea_time, static_cast<uint32_t>(remaining_us / 1000)) ||
            (esp_timer_get_time() - edge_us) > labelDeadlineUs)
        {
            candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - no RMC sentence in time after the PPS edge.");
#endif
            return false;
        }

        candidate->candidate_time = epoch_from_utc(nmea_time.year, nmea_time.month, nmea_time.day, nmea_time.hour, nmea_time.minute, nmea_time.second);
    }
    else
    {
        const int64_t pvt_started_us = esp_timer_get_time();
        if (!wait_for_gnss_pvt(ubxPvtTimeoutMs) || s_gnss.packetUBXNAVPVT == nullptr)
        {
            candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - Position - Velocity - Time");
#endif
            TIMING_LOGW(TAG, "GNSS PVT wait failed after %lld us (limit %u ms); PPS age %lld us.",
                        static_cast<long long>(esp_timer_get_time() - pvt_started_us),
                        static_cast<unsigned int>(ubxPvtTimeoutMs),
                        static_cast<long long>(esp_timer_get_time() - edge_us));
            if (initial_sync)
                display_line(2, "GNSS PVT timeout");
            return false;
        }
        TIMING_LOGI(TAG, "GNSS sync: UBX PVT wait took %lld us; PPS age %lld us.",
                    static_cast<long long>(esp_timer_get_time() - pvt_started_us),
                    static_cast<long long>(esp_timer_get_time() - edge_us));

        const UBX_NAV_PVT_data_t pvt = s_gnss.packetUBXNAVPVT->data;
        TIMING_LOGI(TAG, "GNSS sync: PVT iTOW=%lu ms, calendar second=%u, nano=%ld.",
                    static_cast<unsigned long>(pvt.iTOW), static_cast<unsigned int>(pvt.sec),
                    static_cast<long>(pvt.nano));
        const uint8_t fix_type = pvt.fixType;
        if ((fix_type != 3 && fix_type != 4 && fix_type != 5) ||
            !pvt.flags.bits.gnssFixOK || !pvt.valid.bits.validDate || !pvt.valid.bits.validTime)
        {
            candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - Fix type.");
#endif
            return false;
        }

        int year = pvt.year;
        int month = pvt.month;
        int day = pvt.day;
        int hour = pvt.hour;
        int minute = pvt.min;
        int second = pvt.sec;

        if (year <= 2025 || month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60 ||
            pvt.nano < -1000000000 || pvt.nano > 1000000000)
        {
            candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - bad date or time.");
#endif
            return false;
        }

        if ((esp_timer_get_time() - edge_us) > labelDeadlineUs)
        {
            candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - navigation solution arrived too late after the PPS edge.");
#endif
            return false;
        }

        candidate->candidate_time = epoch_from_utc(year, month, day, hour, minute, second);
        // NAV-PVT UTC is the calendar second plus signed nano, which can straddle a second boundary.
        candidate->candidate_time += (pvt.nano > 500000000) - (pvt.nano < -500000000);
    }

    candidate->pps_release_time_us = edge_us;
    candidate->use_pps_alignment = true;

    if ((esp_timer_get_time() - attempt_start_us) > Max_Sync_Attempt_Us)
    {
        candidate->failures.gnss_invalid = true;
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "GNSS invalid checkpoint 2.");
#endif
        return false;
    }

    candidate->used_nmea_fallback = s_use_nmea_fallback;
    TIMING_LOGI(TAG, "GNSS sync: candidate UTC=%lld, PPS=%lld us, age=%lld us.",
                static_cast<long long>(candidate->candidate_time),
                static_cast<long long>(candidate->pps_release_time_us),
                static_cast<long long>(esp_timer_get_time() - edge_us));
    return true;
}

// Whole-second offset of the system clock at the PPS edge of a candidate: system time at the edge minus the GNSS
// second of the edge, rounded to whole seconds (0 = the clock shows the right second)
static time_t system_clock_second_offset(const sync_candidate_t &candidate)
{
    struct timeval now{};
    const int64_t before_us = esp_timer_get_time();
    gettimeofday(&now, nullptr);
    const int64_t after_us = esp_timer_get_time();
    const int64_t since_edge_us = (before_us + after_us) / 2 - candidate.pps_release_time_us;
    const double system_at_edge = static_cast<double>(now.tv_sec) + static_cast<double>(now.tv_usec) * 1e-6 -
                                  static_cast<double>(since_edge_us) * 1e-6;
    return static_cast<time_t>(llround(system_at_edge - static_cast<double>(candidate.candidate_time)));
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
    TIMING_LOGW(TAG, "GNSS sync failed (attempt %lu): PPS=%d, GNSS=%d, sanity=%d, stale=%d; retry in %lu ms.",
                static_cast<unsigned long>(failure_count),
                faults.pps_missing, faults.gnss_invalid, faults.sanity_mismatch, faults.sync_stale,
                static_cast<unsigned long>(retry_delay_ms));

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
        controlled_restart(("more_than_" + std::to_string(Sync_Failures_Before_Runtime_Recovery) + "_consecutive_time_sync_failures").c_str());

    vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
}

static void gnss_time_sync_task(void *parameter)
{
    (void)parameter;
    bool first_sync = true;
    time_t pending_step_delta = 0; // a whole-second difference seen once; it has to be confirmed before the clock is stepped

    for (;;)
    {
        // During the first synchronisation the time is not valid yet. Afterwards the flag is set only while the
        // clock is really stepped: the periodic GNSS check must not switch the responses to stratum 16 (that is
        // what get_ntp_reply_status does while a time setting is in progress).
        if (first_sync)
            s_time_setting_in_progress.store(true);
        sync_state_note_attempt();

        sync_candidate_t candidate{};
        if (!acquire_sync_candidate(&candidate))
        {
            handle_runtime_sync_failure(candidate.failures, 0, 1000);
            continue;
        }

        if (first_sync)
        {
            TIMING_LOGI(TAG, "GNSS initial sync: acquiring confirmation candidate.");
            sync_candidate_t confirmation_candidate{};
            if (!acquire_sync_candidate(&confirmation_candidate))
            {
                handle_runtime_sync_failure(confirmation_candidate.failures, 0, 1000);
                continue;
            }

            if (!first_sync_candidates_are_plausible(candidate, confirmation_candidate))
            {
                TIMING_LOGW(TAG, "GNSS initial sync: labels disagree: UTC delta=%lld s, PPS delta=%lld us.",
                            static_cast<long long>(confirmation_candidate.candidate_time - candidate.candidate_time),
                            static_cast<long long>(confirmation_candidate.pps_release_time_us - candidate.pps_release_time_us));
                display_line(2, "GNSS label mismatch");
                handle_runtime_sync_failure({false, true, false, false}, 0, 1000);
                continue;
            }

            candidate = confirmation_candidate;
        }

        time_t update_delta = 0;
        bool step_clock = true;
        if (!first_sync)
        {
            update_delta = system_clock_second_offset(candidate);
            bool sanity_check_passed = (update_delta >= -safeguardThresholdInSeconds) && (update_delta <= safeguardThresholdInSeconds);
            if (!sanity_check_passed)
            {
                pending_step_delta = 0;
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

                sync_state_note_failure({false, false, true, false}, update_delta);

                if (rebootIfSanityCheckFails)
                    controlled_restart("time_sync_failed_sanity_check");

                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            if (update_delta == 0)
            {
                pending_step_delta = 0;
                // The clock already shows the right second. While the PPS discipline is running a step would only
                // disturb it (model clock and PTP compensation are invalid until the next PPS edge), so skip it.
                step_clock = !(s_pps_discipline_active.load(std::memory_order_acquire) &&
                               s_model_valid.load(std::memory_order_acquire));
            }
            else if (update_delta != pending_step_delta)
            {
                // A whole second of difference: confirm it with a new reading before the clock is stepped
                pending_step_delta = update_delta;
#if DEBUG_ENABLED
                ESP_LOGW(TAG, "GNSS second differs from the clock by %lld s; confirming before the clock is stepped.",
                         static_cast<long long>(update_delta));
#endif
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }
            else
            {
                pending_step_delta = 0; // confirmed: step
            }
        }

        sync_state_clear_sanity_failures();

        if (step_clock)
        {
            s_time_setting_in_progress.store(true);
            if (first_sync)
                display_line(2, "Setting clocks");
            TIMING_LOGI(TAG, "GNSS sync: waiting for the time mutex.");
            if (xSemaphoreTake(s_time_mutex, portMAX_DELAY) == pdTRUE)
            {
                TIMING_LOGI(TAG, "GNSS sync: time mutex acquired.");
                int64_t elapsed_us = candidate.use_pps_alignment ? (esp_timer_get_time() - candidate.pps_release_time_us) : 0;
                if (elapsed_us < 0)
                    elapsed_us = 0;

                struct timeval tv{};
                tv.tv_sec = candidate.candidate_time + static_cast<time_t>(elapsed_us / 1000000LL);
                tv.tv_usec = static_cast<suseconds_t>(elapsed_us % 1000000LL);
                if (settimeofday(&tv, nullptr) != 0)
                {
                    const int error = errno;
                    xSemaphoreGive(s_time_mutex);
                    TIMING_LOGE(TAG, "GNSS sync: setting system time failed (errno=%d).", error);
                    if (first_sync)
                        display_line(2, "Clock setting failed");
                    handle_runtime_sync_failure({false, true, false, false}, 0, 1000);
                    continue;
                }
                TIMING_LOGI(TAG, "GNSS sync: system clock set.");
                s_clock_comp_valid.store(false, std::memory_order_release);
                model_invalidate();
                synchronize_hardware_clock();
                s_ntp_reference_time_64 = get_current_time_in_ntp64_format();
                s_ntp_reference_valid = true;

                xSemaphoreGive(s_time_mutex);
                TIMING_LOGI(TAG, "GNSS sync: clocks set and time mutex released.");
            }
        }

        s_time_setting_in_progress.store(false);
        s_time_has_been_set.store(true);
        sync_state_note_success(update_delta);

#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
        if (first_sync)
        {
            // Keep UART access in this task while PPS discipline proceeds independently.
            if (s_gnss.setDynamicModel(DYN_MODEL_STATIONARY, VAL_LAYER_RAM, 1500))
                ESP_LOGI(TAG, "GNSS receiver placed in stationary mode");
            else
                ESP_LOGW(TAG, "GNSS receiver not placed in stationary mode");
        }
#endif
        first_sync = false;

#if DEBUG_ENABLED
        char date_string[16] = "";
        char time_string[24] = "";
        time_t now_utc = time(nullptr);
        format_local_date_time(now_utc, date_string, sizeof(date_string), time_string, sizeof(time_string));

        if (candidate.used_nmea_fallback)
            ESP_LOGI(TAG, "GNSS time sync ( using NMEA fallback with PPS alignment ) on %s at %s%s", date_string, time_string,
                     step_clock ? "" : " (clock already correct, not stepped)");
        else
            ESP_LOGI(TAG, "GNSS time sync on %s at %s%s", date_string, time_string,
                     step_clock ? "" : " (clock already correct, not stepped)");
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

#if PPS_ISR_LATENCY_AUTO
    if (!s_pps_isr_latency_known)
    {
        // Run the measurement on the other core than the PPS interrupt, like a real PPS edge arrives
        const BaseType_t measure_core = (configNUM_CORES > 1 && s_pps_isr_core != tskNO_AFFINITY)
                                            ? static_cast<BaseType_t>(s_pps_isr_core == 0 ? 1 : 0)
                                            : static_cast<BaseType_t>(tskNO_AFFINITY);
        if (xTaskCreatePinnedToCore(measure_pps_isr_latency_task, "pps_isr_latency", 6144, nullptr, 21, nullptr, measure_core) != pdPASS)
            TIMING_LOGW(TAG, "PPS ISR latency measurement: task could not be started; using default latency");
    }
#endif

#if PTP_LATENCY_AUTO
    if (s_ptp_cal_required)
    {
        // Calibrate the PTP write latency during normal operation (low priority, it only looks at the samples
        // delivered by the PPS task once per second)
        s_ptp_cal_buf = static_cast<int32_t *>(malloc(sizeof(int32_t) * ptp_cal_block_samples));
        if (s_ptp_cal_buf != nullptr)
        {
            if (xTaskCreatePinnedToCore(ptp_latency_calibration_task, "ptp_lat_cal", 6144, nullptr, 5, nullptr, tskNO_AFFINITY) != pdPASS)
            {
                free(s_ptp_cal_buf);
                s_ptp_cal_buf = nullptr;
                TIMING_LOGW(TAG, "PTP write latency calibration: task could not be started, using %d ns",
                            static_cast<int>(s_ptp_write_latency_ns.load(std::memory_order_relaxed)));
            }
        }
        else
            TIMING_LOGW(TAG, "PTP write latency calibration: out of memory, using %d ns",
                        static_cast<int>(s_ptp_write_latency_ns.load(std::memory_order_relaxed)));
    }
#endif

    while (!s_time_has_been_set.load())
        vTaskDelay(pdMS_TO_TICKS(100));

    while (!s_pps_discipline_active.load(std::memory_order_acquire))
        vTaskDelay(pdMS_TO_TICKS(100));

    display_line(1, "Time Synchronized");
    display_line(2, "PPS Disciplined");
}

// Serves standard IPv6 NTP client requests and returns the device’s current synchronized time.
// Also servers IPv4 NTP client requests that the IPv4 NTP-related traffic the hardware interception path rejects
static void ntp_server_task(void *parameter)
{
    TaskHandle_t startup_task = static_cast<TaskHandle_t>(parameter);
    int ipv4_socket = IPV4_ENABLED ? socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP) : -1;
    int ipv6_socket = IPV6_ENABLED ? socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP) : -1;
    int ipv6_link_local_socket = IPV6_ENABLED ? socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP) : -1;
    if ((IPV4_ENABLED && ipv4_socket < 0) || (IPV6_ENABLED && ipv6_socket < 0))
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
    if ((IPV4_ENABLED && bind(ipv4_socket, reinterpret_cast<struct sockaddr *>(&ipv4_listen_addr), sizeof(ipv4_listen_addr)) != 0) ||
        (IPV6_ENABLED &&
         (setsockopt(ipv6_socket, IPPROTO_IPV6, IPV6_V6ONLY, &ipv6_only, sizeof(ipv6_only)) != 0 ||
          setsockopt(ipv6_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address)) != 0)))
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

    if (IPV6_ENABLED)
    {
        struct sockaddr_in6 ipv6_listen_addr{};
        ipv6_listen_addr.sin6_family = AF_INET6;
        ipv6_listen_addr.sin6_port = htons(NTP_PORT);
        ipv6_listen_addr.sin6_addr = in6addr_any;
        if (bind(ipv6_socket, reinterpret_cast<struct sockaddr *>(&ipv6_listen_addr), sizeof(ipv6_listen_addr)) != 0)
        {
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "Unable to bind IPv6 NTP UDP socket: errno %d", errno);
#endif
            if (ipv4_socket >= 0)
                closesocket(ipv4_socket);
            closesocket(ipv6_socket);
            if (ipv6_link_local_socket >= 0)
                closesocket(ipv6_link_local_socket);
            if (startup_task != nullptr)
                xTaskNotifyGive(startup_task);
            vTaskDelete(nullptr);
            return;
        }
    }

    esp_ip6_addr_t link_local_address{};
    if (!IPV6_ENABLED)
        ipv6_link_local_socket = -1;
    else if (ipv6_link_local_socket < 0 ||
             setsockopt(ipv6_link_local_socket, IPPROTO_IPV6, IPV6_V6ONLY, &ipv6_only, sizeof(ipv6_only)) != 0 ||
             setsockopt(ipv6_link_local_socket, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address)) != 0 ||
             esp_netif_get_ip6_linklocal(s_eth_netif, &link_local_address) != ESP_OK)
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
        ipv6_link_local_listen_addr.sin6_scope_id = esp_netif_get_netif_impl_index(s_eth_netif);
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
        if (ipv4_socket >= 0)
            FD_SET(ipv4_socket, &read_fds);
        if (ipv6_socket >= 0)
            FD_SET(ipv6_socket, &read_fds);
        if (ipv6_link_local_socket >= 0)
            FD_SET(ipv6_link_local_socket, &read_fds);
        struct timeval timeout{};
        timeout.tv_sec = 1;
        int max_socket = ipv4_socket >= 0 ? ipv4_socket : ipv6_socket;
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
                uint8_t request[NTP_MAX_PACKET_SIZE + 1];
                uint8_t reply[NTP_MAX_PACKET_SIZE] = {};
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

                uint64_t receive_time = get_disciplined_time_in_ntp64_format();
                if (receive_time == 0)
                    continue;
                if (len < static_cast<int>(NTP_PACKET_SIZE) || len > static_cast<int>(NTP_MAX_PACKET_SIZE))
                {
#if MQTT_ENABLED
                    s_ntp_invalid_requests.fetch_add(1, std::memory_order_relaxed);
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
                    if (len > static_cast<int>(NTP_PACKET_SIZE) && ((request[0] >> 3) & 0x07) == 4)
                        s_ntp_authenticated_invalid_requests.fetch_add(1, std::memory_order_relaxed);
                    else
#endif
                        s_ntp_unauthenticated_invalid_requests.fetch_add(1, std::memory_order_relaxed);
                    s_ntp_requests_this_second.fetch_add(1, std::memory_order_relaxed);
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
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
                    if (len > static_cast<int>(NTP_PACKET_SIZE) && ntp_version == 4)
                        s_ntp_authenticated_invalid_requests.fetch_add(1, std::memory_order_relaxed);
                    else
#endif
                        s_ntp_unauthenticated_invalid_requests.fetch_add(1, std::memory_order_relaxed);
                    s_ntp_requests_this_second.fetch_add(1, std::memory_order_relaxed);
#endif
#if CALCULATE_STACK_SIZES_ENABLED
                    report_current_task_stack_usage(NTP_Server);
#endif
                    continue;
                }

#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
                uint32_t authentication_key_id = 0;
                const ntp_auth_result_t authentication_result = ntp_auth_verify_request(request, static_cast<size_t>(len), ntp_version, &authentication_key_id);
                if (authentication_result != ntp_auth_result_t::unauthenticated && authentication_result != ntp_auth_result_t::valid)
                {
#if MQTT_ENABLED
                    s_ntp_invalid_requests.fetch_add(1, std::memory_order_relaxed);
                    if (ntp_version == 4)
                        s_ntp_authenticated_invalid_requests.fetch_add(1, std::memory_order_relaxed);
                    else
                        s_ntp_unauthenticated_invalid_requests.fetch_add(1, std::memory_order_relaxed);
                    s_ntp_requests_this_second.fetch_add(1, std::memory_order_relaxed);
#endif
                    continue;
                }
                const bool authenticated_request = authentication_result == ntp_auth_result_t::valid;
#else
                const bool authenticated_request = false;
#endif

                const bool internal_request = is_internal_ntp_client(source_addr);
                if (!s_ntp_external_responses_enabled.load(std::memory_order_acquire) && !internal_request)
                    continue;
                if (!internal_request && !admit_ntp_request())
                {
#if MQTT_ENABLED
                    s_ntp_requests_this_second.fetch_add(1, std::memory_order_relaxed);
#endif
                    continue;
                }

#if MQTT_ENABLED
                s_ntp_requests_this_second.fetch_add(1, std::memory_order_relaxed);
                s_ntp_valid_requests.fetch_add(1, std::memory_order_relaxed);
                if (authenticated_request)
                    s_ntp_authenticated_valid_requests.fetch_add(1, std::memory_order_relaxed);
                else
                    s_ntp_unauthenticated_valid_requests.fetch_add(1, std::memory_order_relaxed);
                mqtt_enqueue_ntp_request(source_addr);
#endif
                const struct sockaddr_in *source_ipv4 = nullptr;
                const struct in6_addr *client_ipv6 = nullptr;
                ntp_client_record_t client_ipv4_record{};
                ntp_client_record_ipv6_t client_ipv6_record{};
                const uint64_t origin_cookie = read_ntp_timestamp(request, 32) != read_ntp_timestamp(request, 40)
                                                   ? read_ntp_timestamp(request, 24)
                                                   : 0;
                if (source_addr.ss_family == AF_INET)
                {
                    source_ipv4 = reinterpret_cast<const struct sockaddr_in *>(&source_addr);
                    ntp_cache_find(ntohl(source_ipv4->sin_addr.s_addr), origin_cookie, &client_ipv4_record);
                }
                else if (source_addr.ss_family == AF_INET6)
                {
                    client_ipv6 = &reinterpret_cast<const struct sockaddr_in6 *>(&source_addr)->sin6_addr;
                    ntp_cache_find_ipv6(client_ipv6, origin_cookie, &client_ipv6_record);
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
                const bool ipv4_interleaved_reply = source_ipv4 != nullptr && previous_ipv4_t2 != 0 &&
                                                    client_receive_timestamp != client_transmit_timestamp &&
                                                    client_origin_timestamp == previous_ipv4_t2;
                const bool ipv6_interleaved_reply = client_ipv6 != nullptr && client_ipv6_record.prev_t2 != 0 &&
                                                    client_receive_timestamp != client_transmit_timestamp &&
                                                    client_origin_timestamp == client_ipv6_record.prev_t2;
                const bool interleaved_reply = ipv4_interleaved_reply || ipv6_interleaved_reply;
                const uint64_t transmit_time = get_disciplined_time_in_ntp64_format();
                if (transmit_time == 0)
                    continue;
                if (interleaved_reply)
                {
                    write_ntp_timestamp(reply, 24, client_receive_timestamp);
                    write_ntp_timestamp(reply, 40, ipv4_interleaved_reply ? previous_ipv4_t3 : client_ipv6_record.prev_t3);
                }
                else
                {
                    write_ntp_timestamp(reply, 40, transmit_time);
                }

                size_t reply_length = NTP_PACKET_SIZE;
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
                if (authenticated_request && !ntp_auth_append_response(reply, sizeof(reply), &reply_length, authentication_key_id))
                    continue;
#endif
                int sent = sendto(sock, reply, reply_length, 0, reinterpret_cast<struct sockaddr *>(&source_addr), source_addr_len);
                if (sent == static_cast<int>(reply_length))
                {
                    if (!internal_request)
                    {
                        s_last_ntp_response_us.store(esp_timer_get_time(), std::memory_order_relaxed);
                    }
                    if (source_ipv4 != nullptr)
                    {
                        ntp_cache_update(ntohl(source_ipv4->sin_addr.s_addr), receive_time, transmit_time);
                    }
                    else if (client_ipv6 != nullptr)
                    {
                        ntp_cache_update_ipv6(client_ipv6, receive_time, transmit_time);
                    }
                }
#if MQTT_ENABLED
                if (sent == static_cast<int>(reply_length))
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
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
    bool ntpv4_authenticated_standard = false;
    bool ntpv4_authenticated_interleaved = false;
    bool ntpv4_invalid_authentication_rejected = false;
    bool ntpv4_unknown_key_rejected = false;
    bool ntpv4_truncated_mac_rejected = false;
    bool ntpv4_altered_response_rejected = false;
#endif
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

enum class startup_health_mqtt_metrics_result_t : uint8_t
{
    passed,
    failed,
    could_not_determine,
};

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

static bool receive_startup_health_response(int socket_fd, uint8_t *reply, size_t reply_capacity, size_t *reply_length, uint64_t *arrival_time)
{
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);
    struct timeval timeout{};
    timeout.tv_sec = Startup_Health_Test_Response_Timeout_Ms / 1000;
    timeout.tv_usec = static_cast<suseconds_t>(Startup_Health_Test_Response_Timeout_Ms % 1000) * 1000;
    if (select(socket_fd + 1, &read_fds, nullptr, nullptr, &timeout) != 1)
        return false;

    int received_length = recvfrom(socket_fd, reply, reply_capacity, 0, nullptr, nullptr);
    if (received_length < 0)
        return false;

    *reply_length = static_cast<size_t>(received_length);
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

        size_t reply_length = 0;
        if (!receive_startup_health_response(socket_fd, reply, sizeof(reply), &reply_length, &exchange->t4) || reply_length != sizeof(reply))
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
    size_t reply_length = 0;
    uint64_t arrival_time = 0;
    request[0] = static_cast<uint8_t>(version << 3) | 3;
    write_ntp_timestamp(request, 24, previous_exchange.t2);
    write_ntp_timestamp(request, 32, previous_exchange.t4);
    write_ntp_timestamp(request, 40, previous_exchange.t1);

    if (sendto(socket_fd, request, sizeof(request), 0, destination, destination_length) != static_cast<int>(sizeof(request)) ||
        !receive_startup_health_response(socket_fd, reply, sizeof(reply), &reply_length, &arrival_time) || reply_length != sizeof(reply))
        return false;

    const uint64_t reply_transmit_time = read_ntp_timestamp(reply, 40);
    const uint64_t transmit_difference = reply_transmit_time >= previous_exchange.t3
                                             ? reply_transmit_time - previous_exchange.t3
                                             : previous_exchange.t3 - reply_transmit_time;
    return ((reply[0] >> 3) & 0x07) == version && (reply[0] & 0x07) == 4 &&
           read_ntp_timestamp(reply, 24) == previous_exchange.t4 &&
           transmit_difference <= Startup_Health_Test_interleaved_Tolerance;
}

#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
static bool run_authenticated_startup_health_test(int socket_fd, const struct sockaddr *destination, socklen_t destination_length,
                                                  startup_health_exchange_t *exchange, bool *altered_response_rejected)
{
    uint8_t request[NTP_MAX_PACKET_SIZE] = {};
    uint8_t reply[NTP_MAX_PACKET_SIZE] = {};
    size_t request_length = NTP_PACKET_SIZE;
    exchange->t1 = get_current_time_in_ntp64_format();
    request[0] = static_cast<uint8_t>(4U << 3) | 3U;
    write_ntp_timestamp(request, 40, exchange->t1);
    if (!ntp_auth_append_response(request, sizeof(request), &request_length, ntp_auth_default_key_id()))
        return false;

    for (uint32_t attempt = 0; attempt < Startup_Health_Test_Standard_Retries; ++attempt)
    {
        if (sendto(socket_fd, request, request_length, 0, destination, destination_length) != static_cast<int>(request_length))
            continue;
        size_t reply_length = 0;
        if (!receive_startup_health_response(socket_fd, reply, sizeof(reply), &reply_length, &exchange->t4) ||
            ntp_auth_verify_request(reply, reply_length, 4, nullptr) != ntp_auth_result_t::valid)
            continue;
        if (((reply[0] >> 3) & 0x07) != 4 || (reply[0] & 0x07) != 4 || read_ntp_timestamp(reply, 24) != exchange->t1)
            continue;
        reply[40] ^= 0x01;
        *altered_response_rejected = ntp_auth_verify_request(reply, reply_length, 4, nullptr) == ntp_auth_result_t::invalid_mac;
        reply[40] ^= 0x01;
        exchange->t2 = read_ntp_timestamp(reply, 32);
        exchange->t3 = read_ntp_timestamp(reply, 40);
        return exchange->t2 != 0 && exchange->t3 != 0;
    }
    return false;
}

enum class startup_health_invalid_auth_t : uint8_t
{
    wrong_mac,
    unknown_key,
    truncated_mac,
};

static bool run_invalid_authenticated_startup_health_test(int socket_fd, const struct sockaddr *destination, socklen_t destination_length,
                                                          startup_health_invalid_auth_t invalid_auth)
{
    uint8_t request[NTP_MAX_PACKET_SIZE] = {};
    size_t request_length = NTP_PACKET_SIZE;
    request[0] = static_cast<uint8_t>(4U << 3) | 3U;
    write_ntp_timestamp(request, 40, get_current_time_in_ntp64_format());
    if (!ntp_auth_append_response(request, sizeof(request), &request_length, ntp_auth_default_key_id()))
        return false;
    switch (invalid_auth)
    {
    case startup_health_invalid_auth_t::wrong_mac:
        request[request_length - 1] ^= 0x01;
        break;
    case startup_health_invalid_auth_t::unknown_key:
        memset(request + NTP_PACKET_SIZE, 0, NTP_AUTH_KEY_ID_SIZE);
        break;
    case startup_health_invalid_auth_t::truncated_mac:
        --request_length;
        break;
    }
    if (sendto(socket_fd, request, request_length, 0, destination, destination_length) != static_cast<int>(request_length))
        return false;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);
    struct timeval timeout{};
    timeout.tv_sec = Startup_Health_Test_Response_Timeout_Ms / 1000;
    timeout.tv_usec = static_cast<suseconds_t>(Startup_Health_Test_Response_Timeout_Ms % 1000) * 1000;
    return select(socket_fd + 1, &read_fds, nullptr, nullptr, &timeout) == 0;
}

static bool run_authenticated_interleaved_startup_health_test(int socket_fd, const struct sockaddr *destination, socklen_t destination_length,
                                                              const startup_health_exchange_t &previous_exchange)
{
    uint8_t request[NTP_MAX_PACKET_SIZE] = {};
    uint8_t reply[NTP_MAX_PACKET_SIZE] = {};
    size_t request_length = NTP_PACKET_SIZE;
    uint64_t arrival_time = 0;
    request[0] = static_cast<uint8_t>(4U << 3) | 3U;
    write_ntp_timestamp(request, 24, previous_exchange.t2);
    write_ntp_timestamp(request, 32, previous_exchange.t4);
    write_ntp_timestamp(request, 40, previous_exchange.t1);
    if (!ntp_auth_append_response(request, sizeof(request), &request_length, ntp_auth_default_key_id()) ||
        sendto(socket_fd, request, request_length, 0, destination, destination_length) != static_cast<int>(request_length))
        return false;
    size_t reply_length = 0;
    if (!receive_startup_health_response(socket_fd, reply, sizeof(reply), &reply_length, &arrival_time) ||
        ntp_auth_verify_request(reply, reply_length, 4, nullptr) != ntp_auth_result_t::valid)
        return false;
    const uint64_t reply_transmit_time = read_ntp_timestamp(reply, 40);
    const uint64_t transmit_difference = reply_transmit_time >= previous_exchange.t3
                                             ? reply_transmit_time - previous_exchange.t3
                                             : previous_exchange.t3 - reply_transmit_time;
    return (reply[0] & 0x07) == 4 && read_ntp_timestamp(reply, 24) == previous_exchange.t4 &&
           transmit_difference <= Startup_Health_Test_interleaved_Tolerance;
}
#endif

static startup_health_mqtt_metrics_result_t check_startup_health_mqtt_metrics(const startup_health_test_results_t &results)
{
#if MQTT_ENABLED && MQTT_CLIENT_REPORTING_ENABLED
    if (s_mqtt_ntp_event_queue == nullptr || s_mqtt_stats_mutex == nullptr)
        return startup_health_mqtt_metrics_result_t::could_not_determine;

    uint32_t expected_authenticated_valid = 0;
    uint32_t expected_unauthenticated_valid = 0;
    uint32_t expected_authenticated_invalid = 0;
    bool indeterminate = false;
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
    const bool authentication_available = ntp_auth_available();
#endif
    const auto count_endpoint = [&](const startup_health_endpoint_results_t &endpoint)
    {
        const bool unauthenticated_valid = endpoint.ntpv3_standard && endpoint.ntpv4_standard && endpoint.ntpv4_interleaved;
        if (unauthenticated_valid)
            expected_unauthenticated_valid += 3;
        else
            indeterminate = true;
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
        if (authentication_available)
        {
            const bool authenticated_valid = endpoint.ntpv4_authenticated_standard && endpoint.ntpv4_authenticated_interleaved;
            if (authenticated_valid)
                expected_authenticated_valid += 2;
            else
                indeterminate = true;

            const bool authenticated_invalid = endpoint.ntpv4_invalid_authentication_rejected && endpoint.ntpv4_unknown_key_rejected &&
                                               endpoint.ntpv4_truncated_mac_rejected;
            if (authenticated_invalid)
                expected_authenticated_invalid += 3;
            else
                indeterminate = true;
        }
#endif
    };

    const network_address_snapshot_t addresses = get_network_address_snapshot();
    if (results.prerequisites_ready)
    {
        if (IPV4_ENABLED && addresses.ipv4[0] != '\0')
        {
            count_endpoint(results.ipv4_loopback);
            count_endpoint(results.ipv4_assigned);
        }
        if (IPV6_ENABLED && addresses.ipv6[0] != '\0')
        {
            count_endpoint(results.ipv6_loopback);
            count_endpoint(results.ipv6_assigned);
        }
    }

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(MQTT_QUEUED_PUBLISH_DELAY_MS * 2U);
    while (uxQueueMessagesWaiting(s_mqtt_ntp_event_queue) != 0 && xTaskGetTickCount() < deadline)
        vTaskDelay(pdMS_TO_TICKS(1));

    if (uxQueueMessagesWaiting(s_mqtt_ntp_event_queue) != 0 || s_ntp_telemetry_events_dropped.load(std::memory_order_relaxed) != 0 ||
        s_mqtt_client_table_overflown.load(std::memory_order_relaxed))
        return startup_health_mqtt_metrics_result_t::could_not_determine;

    if (indeterminate || !results.prerequisites_ready)
        return startup_health_mqtt_metrics_result_t::could_not_determine;

    const uint32_t expected_valid = expected_authenticated_valid + expected_unauthenticated_valid;
    if (s_ntp_authenticated_valid_requests.load(std::memory_order_relaxed) < expected_authenticated_valid ||
        s_ntp_unauthenticated_valid_requests.load(std::memory_order_relaxed) < expected_unauthenticated_valid ||
        s_ntp_authenticated_invalid_requests.load(std::memory_order_relaxed) < expected_authenticated_invalid ||
        s_ntp_valid_requests.load(std::memory_order_relaxed) < expected_valid ||
        s_ntp_invalid_requests.load(std::memory_order_relaxed) < expected_authenticated_invalid ||
        s_ntp_responses_synchronized_and_disciplined.load(std::memory_order_relaxed) < expected_valid ||
        s_ntp_responses.load(std::memory_order_relaxed) < expected_valid)
        return startup_health_mqtt_metrics_result_t::failed;

    const auto endpoint_expected_valid_requests = [
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
                                                      authentication_available
#endif
    ](const startup_health_endpoint_results_t &endpoint)
    {
        uint32_t expected_requests = endpoint.ntpv3_standard && endpoint.ntpv4_standard && endpoint.ntpv4_interleaved ? 3U : 0U;
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
        if (authentication_available && endpoint.ntpv4_authenticated_standard && endpoint.ntpv4_authenticated_interleaved)
            expected_requests += 2;
#endif
        return expected_requests;
    };
    const auto has_expected_client_requests = [](const char *address_text, sa_family_t address_family, uint32_t expected_requests)
    {
        if (expected_requests == 0)
            return true;
        uint8_t expected_address[sizeof(struct in6_addr)] = {};
        const int address_size = address_family == AF_INET ? sizeof(struct in_addr) : sizeof(struct in6_addr);
        if (inet_pton(address_family, address_text, expected_address) != 1)
            return false;
        for (size_t index = 0; index < s_mqtt_client_count; ++index)
        {
            if (s_mqtt_clients[index].address_family == address_family &&
                memcmp(s_mqtt_clients[index].address, expected_address, address_size) == 0)
                return s_mqtt_clients[index].requests >= expected_requests;
        }
        return false;
    };

    if (xSemaphoreTake(s_mqtt_stats_mutex, portMAX_DELAY) != pdTRUE)
        return startup_health_mqtt_metrics_result_t::could_not_determine;
    const bool clients_match = (!IPV4_ENABLED ||
                                (has_expected_client_requests("127.0.0.1", AF_INET, endpoint_expected_valid_requests(results.ipv4_loopback)) &&
                                 has_expected_client_requests(addresses.ipv4, AF_INET, endpoint_expected_valid_requests(results.ipv4_assigned)))) &&
                               (!IPV6_ENABLED ||
                                (has_expected_client_requests("::1", AF_INET6, endpoint_expected_valid_requests(results.ipv6_loopback)) &&
                                 has_expected_client_requests(addresses.ipv6, AF_INET6, endpoint_expected_valid_requests(results.ipv6_assigned))));
    xSemaphoreGive(s_mqtt_stats_mutex);
    return clients_match ? startup_health_mqtt_metrics_result_t::passed : startup_health_mqtt_metrics_result_t::failed;
#else
    (void)results;
    return startup_health_mqtt_metrics_result_t::could_not_determine;
#endif
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
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
        if (ntp_auth_available())
        {
            queue_startup_health_test_result(name, "NTPv4 auth standard", false);
            queue_startup_health_test_result(name, "NTPv4 auth interlv ", false);
            queue_startup_health_test_result(name, "NTPv4 invalid auth ", false);
            queue_startup_health_test_result(name, "NTPv4 unknown key  ", false);
            queue_startup_health_test_result(name, "NTPv4 truncated MAC", false);
            queue_startup_health_test_result(name, "NTPv4 altered reply", false);
        }
#endif
        return results;
    }

    startup_health_exchange_t ntpv3_exchange{};
    startup_health_exchange_t ntpv4_exchange{};
    results.ntpv3_standard = run_standard_startup_health_test(socket_fd, destination, destination_length, 3, &ntpv3_exchange);
    results.ntpv4_standard = run_standard_startup_health_test(socket_fd, destination, destination_length, 4, &ntpv4_exchange);
    if (results.ntpv4_standard)
        results.ntpv4_interleaved = run_interleaved_startup_health_test(socket_fd, destination, destination_length, 4, ntpv4_exchange);
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
    if (ntp_auth_available())
    {
        startup_health_exchange_t authenticated_exchange{};
        results.ntpv4_authenticated_standard = run_authenticated_startup_health_test(socket_fd, destination, destination_length, &authenticated_exchange,
                                                                                     &results.ntpv4_altered_response_rejected);
        if (results.ntpv4_authenticated_standard)
            results.ntpv4_authenticated_interleaved = run_authenticated_interleaved_startup_health_test(socket_fd, destination, destination_length, authenticated_exchange);
        results.ntpv4_invalid_authentication_rejected = run_invalid_authenticated_startup_health_test(socket_fd, destination, destination_length,
                                                                                                      startup_health_invalid_auth_t::wrong_mac);
        results.ntpv4_unknown_key_rejected = run_invalid_authenticated_startup_health_test(socket_fd, destination, destination_length,
                                                                                           startup_health_invalid_auth_t::unknown_key);
        results.ntpv4_truncated_mac_rejected = run_invalid_authenticated_startup_health_test(socket_fd, destination, destination_length,
                                                                                             startup_health_invalid_auth_t::truncated_mac);
    }
#endif

    closesocket(socket_fd);
    queue_startup_health_test_result(name, "NTPv3 standard     ", results.ntpv3_standard);
    queue_startup_health_test_result(name, "NTPv4 standard     ", results.ntpv4_standard);
    queue_startup_health_test_result(name, "NTPv4 interleaved  ", results.ntpv4_interleaved);
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
    if (ntp_auth_available())
    {
        queue_startup_health_test_result(name, "NTPv4 auth standard", results.ntpv4_authenticated_standard);
        queue_startup_health_test_result(name, "NTPv4 auth interlv ", results.ntpv4_authenticated_interleaved);
        queue_startup_health_test_result(name, "NTPv4 invalid auth ", results.ntpv4_invalid_authentication_rejected);
        queue_startup_health_test_result(name, "NTPv4 unknown key  ", results.ntpv4_unknown_key_rejected);
        queue_startup_health_test_result(name, "NTPv4 truncated MAC", results.ntpv4_truncated_mac_rejected);
        queue_startup_health_test_result(name, "NTPv4 altered reply", results.ntpv4_altered_response_rejected);
    }
#endif
    return results;
}

static startup_health_test_results_t run_startup_health_tests()
{
    startup_health_test_results_t results{};
    const int64_t deadline_us = esp_timer_get_time() + static_cast<int64_t>(Startup_Health_Test_Ready_Timeout_Ms) * 1000;
    EventBits_t ready = 0;
    do
    {
        ready = xEventGroupGetBits(s_net_event_group);
        const bool required_ip_available = PreferIPvX == 1   ? (ready & ETH_GOT_IP_BIT) != 0
                                           : PreferIPvX == 2 ? (ready & ETH_GOT_ROUTABLE_IP6_BIT) != 0
                                                             : (ready & (ETH_GOT_IP_BIT | ETH_GOT_ROUTABLE_IP6_BIT)) != 0;
        if ((ready & ETH_CONNECTED_BIT) != 0 && required_ip_available)
            break;
        vTaskDelay(pdMS_TO_TICKS(100));
    } while (esp_timer_get_time() < deadline_us);

    const bool required_ip_available = PreferIPvX == 1   ? (ready & ETH_GOT_IP_BIT) != 0
                                       : PreferIPvX == 2 ? (ready & ETH_GOT_ROUTABLE_IP6_BIT) != 0
                                                         : (ready & (ETH_GOT_IP_BIT | ETH_GOT_ROUTABLE_IP6_BIT)) != 0;
    if ((ready & ETH_CONNECTED_BIT) == 0 || !required_ip_available || !s_time_has_been_set.load(std::memory_order_acquire) ||
        !s_pps_discipline_active.load(std::memory_order_acquire))
    {
        queue_startup_health_log(ESP_LOG_ERROR, "Health Check prerequisites were not ready before timeout");
        return results;
    }

    results.prerequisites_ready = true;
    const network_address_snapshot_t addresses = get_network_address_snapshot();
    if (IPV4_ENABLED && addresses.ipv4[0] != '\0')
    {
        struct sockaddr_in ipv4_loopback{};
        ipv4_loopback.sin_family = AF_INET;
        ipv4_loopback.sin_port = htons(NTP_PORT);
        ipv4_loopback.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        results.ipv4_loopback = run_startup_health_endpoint_test("IPv4 loopback", reinterpret_cast<const struct sockaddr *>(&ipv4_loopback), sizeof(ipv4_loopback), AF_INET);

        struct sockaddr_in ipv4_assigned{};
        ipv4_assigned.sin_family = AF_INET;
        ipv4_assigned.sin_port = htons(NTP_PORT);
        if (inet_pton(AF_INET, addresses.ipv4, &ipv4_assigned.sin_addr) == 1)
            results.ipv4_assigned = run_startup_health_endpoint_test("IPv4 assigned", reinterpret_cast<const struct sockaddr *>(&ipv4_assigned), sizeof(ipv4_assigned), AF_INET);
        else
            queue_startup_health_log(ESP_LOG_ERROR, "Health Check IPv4 assigned: invalid address");
    }

    if (IPV6_ENABLED && addresses.ipv6[0] != '\0')
    {
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
        ipv6_assigned.sin6_scope_id = esp_netif_get_netif_impl_index(s_eth_netif);
        if (inet_pton(AF_INET6, addresses.ipv6, &ipv6_assigned.sin6_addr) == 1)
            results.ipv6_assigned = run_startup_health_endpoint_test("IPv6 assigned", reinterpret_cast<const struct sockaddr *>(&ipv6_assigned), sizeof(ipv6_assigned), AF_INET6);
        else
            queue_startup_health_log(ESP_LOG_ERROR, "Health Check IPv6 assigned: invalid address");
    }

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
    const startup_health_mqtt_metrics_result_t mqtt_metrics_result = check_startup_health_mqtt_metrics(s_startup_health_test_results);
    ++s_startup_health_test_number;
    const char *mqtt_metrics_status = mqtt_metrics_result == startup_health_mqtt_metrics_result_t::passed
                                          ? "Passed"
                                      : mqtt_metrics_result == startup_health_mqtt_metrics_result_t::failed ? "Failed"
                                                                                                            : "Could not determine";
    queue_startup_health_log(mqtt_metrics_result == startup_health_mqtt_metrics_result_t::passed ? ESP_LOG_INFO : ESP_LOG_WARN,
                             "Health Check %02lu - MQTT metrics                       - %s", static_cast<unsigned long>(s_startup_health_test_number),
                             mqtt_metrics_status);
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

    if (get_ntp_reply_status().holdover)
        return 15;
    if (s_saved_gnss_baud_communication_failed.load())
        return 9;
    if (s_time_setting_in_progress.load())
        return 98; // 98 = time being set ('*' on the LCD); 99 is reserved for the button

    if (!s_ethernet_connected.load())
        required_top_line_message = 1;
    else if (s_ntp_external_responses_enabled.load() && s_net_event_group != nullptr &&
             !has_usable_network_address(xEventGroupGetBits(s_net_event_group)))
        required_top_line_message = 13;
#if MQTT_ENABLED
    else if (s_mqtt_queued_messages_count.load() > 0)
        required_top_line_message = 3;
    else if (s_mqtt_setup_failed.load())
        required_top_line_message = 2;
    else if (!s_mqtt_connected.load())
        required_top_line_message = 11;
#endif
    if (s_ntp_transport_health.load(std::memory_order_relaxed) == ntp_transport_health_t::stalled &&
        required_top_line_message != 1)
        required_top_line_message = 14;

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
    if (required_top_line_message == 0 && ntp_load_shedding_active())
        return 12;

    return required_top_line_message;
}

#if RBG_LED_KY_016_MODULE_ENABLED || RBG_LED_WS2812_MODULE_ENABLED
static void control_RGB_LED(RGB_LED_Colour Colour, bool enabled)
{
    uint8_t red_duty = 0;
    uint8_t green_duty = 0;
    uint8_t blue_duty = 0;

    if (enabled)
    {
        switch (Colour)
        {
        case RGB_LED_Colour::red:
            red_duty = 255;
            break;
        case RGB_LED_Colour::green:
            green_duty = 255;
            break;
        case RGB_LED_Colour::blue:
            blue_duty = 255;
            break;
        case RGB_LED_Colour::yellow:
            red_duty = 255;
            green_duty = 255;
            break;
        case RGB_LED_Colour::white:
            red_duty = 255;
            green_duty = 255;
            blue_duty = 255;
            break;
        case RGB_LED_Colour::orange:
            red_duty = 255;
            green_duty = 165; // 128
            break;
        case RGB_LED_Colour::pink:
            red_duty = 255;
            green_duty = 192;
            blue_duty = 203;
            break;
        case RGB_LED_Colour::off:
            break;
        }
    }

#if RBG_LED_KY_016_MODULE_ENABLED
    static_assert(RGB_LED_Brightness_Percent >= 0 && RGB_LED_Brightness_Percent <= 100);
    static constexpr uint8_t brightness = static_cast<uint8_t>((255 * RGB_LED_Brightness_Percent + 50) / 100);
    red_duty = static_cast<uint8_t>((red_duty * brightness + 127) / 255);
    green_duty = static_cast<uint8_t>((green_duty * brightness + 127) / 255);
    blue_duty = static_cast<uint8_t>((blue_duty * brightness + 127) / 255);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, red_duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, green_duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, blue_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2));
#endif
#if RBG_LED_WS2812_MODULE_ENABLED
    red_duty = static_cast<uint8_t>((red_duty * s_ws2812_brightness + 127) / 255);
    green_duty = static_cast<uint8_t>((green_duty * s_ws2812_brightness + 127) / 255);
    blue_duty = static_cast<uint8_t>((blue_duty * s_ws2812_brightness + 127) / 255);

    static uint32_t last_rgb = UINT32_MAX;
    const uint32_t rgb = (static_cast<uint32_t>(red_duty) << 16) |
                         (static_cast<uint32_t>(green_duty) << 8) | blue_duty;
    if (rgb != last_rgb)
    {
        ESP_ERROR_CHECK(led_strip_set_pixel(s_ws2812_strip, 0, red_duty, green_duty, blue_duty));
        ESP_ERROR_CHECK(led_strip_refresh(s_ws2812_strip));
        last_rgb = rgb;
    }
#endif
}

static void update_RGB_LED()
{
    RGB_LED_Colour Colour;
    bool flashing = false;

    if (s_open_for_business_message_written.load(std::memory_order_acquire))
    {
        // The codes are the ones returned by get_required_top_line_message()
        switch (get_required_top_line_message())
        {
        case 0: // The server is operating normally
            Colour = LED_normal;
            break;

        case 3: // There are queued MQTT items
            Colour = LED_normal;
            flashing = true;
            break;

        case 5: // PPS missing
        case 6: // GNSS missing or invalid
        case 7: // GNSS sync stale
        case 8: // GNSS has lost its satellite lock
            Colour = LED_warning;
            break;

        case 15: // Two-minute PPS holdover
            Colour = RGB_LED_Colour::yellow;
            flashing = true;
            break;

        case 11: // Not connected to the MQTT broker
            Colour = LED_critical;
            flashing = true;
            break;

        case 1:  // Ethernet not connected
        case 2:  // MQTT setup failed
        case 4:  // Sanity check mismatch
        case 9:  // Communication failure with the GNSS receiver
        case 14: // Transport layer stalled
            Colour = LED_critical;
            break;

        case 12: // Load shedding active
            Colour = LED_alert;
            flashing = true;
            break;

        case 13: // No usable IP address
            Colour = LED_alert;
            break;

        case 98: // The server is setting its time
            Colour = LED_sync;
            break;

        case 99: // The button is being pressed
            Colour = LED_button_push;
            break;

        default: // should not get here
            Colour = LED_normal;
            break;
        }
    }
    else
    {
        Colour = LED_startup;
        flashing = s_gnss_pps_startup_qualification_in_progress.load(std::memory_order_acquire) ||
                   s_pps_discipline_active.load();
    }

    const bool enabled = !flashing || ((esp_timer_get_time() / 1000000LL) % 2 == 0);
    control_RGB_LED(Colour, enabled);
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

#if OTE_UPDATES_ENABLED
        if (render_ote_display())
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
#endif

        time_t now_utc = time(nullptr);
        struct tm utc_tm{};
        gmtime_r(&now_utc, &utc_tm);

        if (utc_tm.tm_sec != previous_second)
        {
            previous_second = utc_tm.tm_sec;

#if UPTIME_RESTART_BUTTON_ENABLED
            if (check_uptime_request())
                display_uptime_seconds_counter = upTimeDisplayWillStayActiveForThisManySeconds;
#endif

            int required_top_line_message;

            if (s_open_for_business_message_written.load(std::memory_order_acquire))
                required_top_line_message = get_required_top_line_message();
            else
                required_top_line_message = 0;

            /*

            |  Code   | Top Line             | RGB LED         | Meaning                                                            |
            |-------- | -------------------- | --------------- | ------------------------------------------------------------------ |
            |   n/a   | ESP32 Time Server    | Blue            | The server is in startup mode                                      |
            |   n/a   | ESP32 Time Server    | Blue flashing   | The server is almost ready, but needs to complete PPS disciplining |
            |         |                      |                 |                                                                    |
            |    0    | ESP32 Time Server    | Green           | The server is operating normally                                   |
            |    3    | ESP32 Time Server[3] | Green flashing  | There are queued MQTT items                                        |
            |    5    | ESP32 Time Server[5] | Yellow          | PPS missing                                                        |
            |    6    | ESP32 Time Server[6] | Yellow          | GNSS missing or invalid                                            |
            |    7    | ESP32 Time Server[7] | Yellow          | GNSS sync stale                                                    |
            |    8    | ESP32 Time Server[8] | Yellow          | GNSS has lost its satellite lock                                   |
            |   11    | ESP32 Time Server[M] | Red flashing    | Not connected to the MQTT broker                                   |
            |    9    | ESP32 Time Server[9] | Red             | Communication failure with the GNSS receiver                       |
            |    1    | ESP32 Time Server[1] | Red             | Ethernet not connected                                             |
            |    2    | ESP32 Time Server[2] | Red             | MQTT setup failed                                                  |
            |   14    | ESP32 Time Server[T] | Red             | Transport layer stalled                                            |
            |    4    | ESP32 Time Server[4] | Red             | Sanity check mismatch                                              |
            |         |                      |                 |                                                                    |
            |   13    | ESP32 Time Server[I] | Orange          | No usable IP address                                               |
            |   12    | ESP32 Time Server[+] | Orange flashing | Load shedding active                                               |
            |   98    | ESP32 Time Server[*] | White           | The server is setting its time                                     |
            |   99    | ESP32 Time Server's  | Pink            | The button is being pressed                                        |


            During the two-minute PPS holdover, code 15 displays ESP32 Time Server[H] and flashes yellow.

            The above chart can be found in the folder: misc/status_indicators.md

            */

#if UPTIME_RESTART_BUTTON_ENABLED
            if (display_uptime_seconds_counter > 0)
                required_top_line_message = 99;
#endif

            // Update top line only if it has changed
            if (required_top_line_message != previous_top_line_message)
            {
                char top_line_message[21]; // 20 chars + null
                snprintf(top_line_message, sizeof(top_line_message), "%-20s", lcdTopLineTitle);

                if (required_top_line_message == 15)
                {
                    snprintf(top_line_message, sizeof(top_line_message), "%-20s", "ESP32 Time Server[H]");
                }
                else if (required_top_line_message == 98)
                {
                    top_line_message[18] = '*';
                }
                else if (required_top_line_message > 0 && required_top_line_message < 10)
                {
                    top_line_message[17] = '[';
                    top_line_message[18] = '0' + required_top_line_message;
                    top_line_message[19] = ']';
                }
                else if (required_top_line_message == 99)
                {
                    // when button pushed: "ESP32 Time Server's"
                    memcpy(top_line_message + sizeof(lcdTopLineTitle) - 1, "'s", 2);
                }
                else if (required_top_line_message >= 11 && required_top_line_message <= 14)
                {
                    top_line_message[17] = '[';
                    top_line_message[18] = required_top_line_message == 11   ? 'M'
                                           : required_top_line_message == 12 ? '+'
                                           : required_top_line_message == 13 ? 'I'
                                                                             : 'T';
                    top_line_message[19] = ']';
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

#if RBG_LED_KY_016_MODULE_ENABLED || RBG_LED_WS2812_MODULE_ENABLED
        update_RGB_LED();
#endif

        vTaskDelay(pdMS_TO_TICKS(50));

#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(LED_LCD_Button);
#endif
    }
#else
    for (;;)
    {
#if RBG_LED_KY_016_MODULE_ENABLED || RBG_LED_WS2812_MODULE_ENABLED
        update_RGB_LED();
#endif
#if UPTIME_RESTART_BUTTON_ENABLED
        (void)check_uptime_request();
#endif
        vTaskDelay(pdMS_TO_TICKS(50));
    }
#endif
}

void start_RGB_LED_LCD_and_Button_refresh()
{

    xTaskCreatePinnedToCore(update_LED_LCD_Button_task, "LED_LCD_Button_service", LED_LCD_Button_Task_Stack_Size, nullptr, 10, nullptr, tskNO_AFFINITY);
}

void setup_symmetric_key_authentication()
{
#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED
    if (ntp_auth_initialize())
        ESP_LOGI(TAG, "NTP symmetric-key authentication is available");
    else
        ESP_LOGW(TAG, "NTP symmetric-key authentication is unavailable");
#endif
}

void setup_ntp_server()
{

    // set the NTP load shedding threshold based on the ESP32-P4 chip version
    esp_chip_info_t chip_info{};
    esp_chip_info(&chip_info);
    NTP_Requests_Per_Second_Load_Shedding_Threshold = chip_info.revision <= 103
                                                          ? NTP_Requests_Per_Second_Load_Shedding_Threshold_ESP32P4_ChipVersion1_3
                                                          : NTP_Requests_Per_Second_Load_Shedding_Threshold_ESP32P4_ChipVersion_above_1_3;

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "NTP load shedding threshold set to %u requests per second", NTP_Requests_Per_Second_Load_Shedding_Threshold);
#endif

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
}

void setup_recovery_task()
{

    if (xTaskCreatePinnedToCore(ethernet_transport_recovery_task, "eth_recovery", Ethernet_Transport_Recovery_Task_Stack_Size,
                                nullptr, ETHERNET_RECOVERY_TASK_PRIORITY, nullptr, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "Unable to start NTP transport health monitoring");
        controlled_restart("ethernet_monitor_start_failed");
    }
}

void perform_health_check()
{
#if STARTUP_HEALTH_TEST_ENABLED
    if (xTaskCreatePinnedToCore(startup_health_test_task, "startup_health", Startup_Health_Test_Task_Stack_Size,
                                xTaskGetCurrentTaskHandle(), 5, nullptr, tskNO_AFFINITY) != pdPASS)
        ESP_LOGE(TAG, "Health Check could not start");
    else
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#endif
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
    //  - The IPv6 address acquired may be a Link-Local address (LLA) - which will be used as the least preferred address type.
    //    If afterwards a Unique Local Address (ULA) is acquired, it will be used as a more preferred address type over the LLA.
    //    If afterwards a Global Unicast Address (GUA) is acquired, it will be used as a more preferred address type over the LLA and ULA.
    //

#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED && DEBUG_ENABLED

    // The secret keys are only printed with DEBUG_ENABLED (a serial log is easily copied or recorded).
    // For the client configuration the ASCII keys of ESP32TimeServerSettingsSecrets.h can be used directly,
    // e.g. "1 SHA256 ASCII:<key>" in chrony.keys.
    if (ntp_auth_key_count() != 0)
    {
        char authentication_key_hex[NTP_AUTH_MAX_KEY_SIZE * 2 + 1] = "";
        uint32_t authentication_key_id = 0;
        ESP_LOGI(TAG, "Authorization key(s) for use with Meinberg (set in the client's ntp.keys file):");
        ESP_LOGI(TAG, " ");
        for (size_t index = 0; index < ntp_auth_key_count(); ++index)
        {
            if (ntp_auth_key_hex(index, &authentication_key_id, authentication_key_hex, sizeof(authentication_key_hex)))
                ESP_LOGI(TAG, "%" PRIu32 " SHA256 %s", authentication_key_id, authentication_key_hex);
        }
        ESP_LOGI(TAG, " ");
        ESP_LOGI(TAG, "Authorization key(s) for use with Chrony (set in the client's chrony.keys file):");
        ESP_LOGI(TAG, " ");
        for (size_t index = 0; index < ntp_auth_key_count(); ++index)
        {
            if (ntp_auth_key_hex(index, &authentication_key_id, authentication_key_hex, sizeof(authentication_key_hex)))
                ESP_LOGI(TAG, "%" PRIu32 " SHA256 HEX:%s", authentication_key_id, authentication_key_hex);
        }
        ESP_LOGI(TAG, " ");
    }
#endif

    refresh_network_ipv6_addresses();
    log_network_ipv6_addresses();
    const network_address_snapshot_t addresses = get_network_address_snapshot();
    if (PreferIPvX != 2) // If not IPv6 only, report IPv4 connection status
    {
        if (addresses.ipv4[0] != '\0')
            ESP_LOGI(TAG, "The IPv4 connection is up ( %s )", addresses.ipv4);
        else
            ESP_LOGW(TAG, "The IPv4 connection is down");
    }

    if (PreferIPvX != 1) // If not IPv4 only, report IPv6 connection status
    {
        if (addresses.ipv6[0] != '\0')
            ESP_LOGI(TAG, "The IPv6 connection is up ( %s )", addresses.ipv6);
        else
            ESP_LOGW(TAG, "The IPv6 connection is down");
    }

    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "MAC address: %s", MACToBeUsed.c_str());

    s_ntp_external_responses_enabled.store(true, std::memory_order_release);

    // Note: 'Open for business' message is purposefully not guarded by a DEBUG_ENABLE check - it should always be written to the console.
    char s_open_for_business_date_and_time[25] = "";
    format_time_to_ISO8601(time(nullptr), s_open_for_business_date_and_time, sizeof(s_open_for_business_date_and_time));
    ESP_LOGI(TAG, " ");
    ESP_LOGI(TAG, "***********************************************");
    ESP_LOGI(TAG, "* Open for business: %s *", s_open_for_business_date_and_time);
    ESP_LOGI(TAG, "***********************************************");
    ESP_LOGI(TAG, " ");

    s_open_for_business_message_written.store(true, std::memory_order_release);

#if !DEBUG_ENABLED
    ESP_LOGW(TAG, "DEBUG_ENABLED is disabled; routine main_cpp logging stops here (recovery/errors remain enabled)");
#endif
}

extern "C" void app_main()
{

    setup_pps_input(); // (keep this at the very beginning of the setup sequence)

    initArduino();

    write_opening_messages_to_the_console();

    setup_mqtt_tf_queue();

    setup_NVS_storage();

    setup_mutexes_and_semaphores();

    setup_the_RGB_LED();

    setup_the_LCD();

    setup_the_button();

    start_RGB_LED_LCD_and_Button_refresh();

    const bool ethernet_setup_started = start_ethernet_setup_task(xTaskGetCurrentTaskHandle());

    setup_the_gnss();

    if (ethernet_setup_started)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    setup_for_ote_updates();

    setup_mqtt();

    setup_symmetric_key_authentication();

    setup_ntp_server();

    setup_recovery_task();

    perform_health_check();

    write_open_for_business_messages_to_the_console();
}

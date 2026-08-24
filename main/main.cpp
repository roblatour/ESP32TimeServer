// ESP32 Time Server v2.5.2
// Copyright Rob Latour, 2026

//
// ESP32 Dev Board:     ESP32-P4-ETH https://www.waveshare.com/esp32-p4-eth.htm
//                                   https://www.waveshare.com/wiki/ESP32-P4-ETH?srsltid=AfmBOoo6nZm5hsPAhtpzT6lWSHd2zhWNPM_mqgbNvyoESbjvbO7uykcH
//
//                      NOTE: Powering the ESP32-P4_ETH by either a USB C cable or, with its optional POE had installed,
//                      a POE Ethernet cable is sufficient to power the ESP32-P4-ETH, GPS module and LCD screen.
//
//                      ************************************************************************************************
//                      * HOWEVER DO NOT POWER THE ESP32-P4_ETH VIA BOTH ITS USB C CONNECTION AND POE AT THE SAME TIME *
//                      ************************************************************************************************
//
// GPS (recommended):   SparkFun GNSS Receiver Breakout - MAX-M10S  https://www.sparkfun.com/sparkfun-gnss-receiver-breakout-max-m10s-qwiic.html
//
// LCD2004:             blue/green screen with HD44780 I2C serial interface adapter https://www.aliexpress.com/item/1005006829045609.html.
//
// Wiring:
//
// (mandatory) GPS module wiring to and from the ESP32-P4-ETH board:
// GPS GND            <- -> ESP32-P4-ETH GND
// GPS VCC            <- -> ESP32-P4-ETH 3V3
// GPS TXD            <- -> ESP32-P4-ETH GPIO17 (RX)
// GPS RXD            <- -> ESP32-P4-ETH GPIO16 (TX)
// GPS PPS            <- -> ESP32-P4-ETH GPIO18
//
// (optional) LCD204A V1.5 (HD44780 + PCF8574T I2C backpack) wiring to and from the ESP32-P4-ETH board:
// LCD GND            <- -> ESP32-P4-ETH GND
// LCD VCC            <- -> ESP32-P4-ETH 3V3
// LCD SDA            <- -> ESP32-P4-ETH GPIO8 (SDA)
// LCD SLC/SCL        <- -> ESP32-P4-ETH GPIO7 (SCL)
//
// (optional) Up time momentary button to and from teh ESP32-P4-ETH board:
// one terminal       <- -> ESP32-P4-ETH GPI03
// the other terminal <- -> ESP32-P4-ETH GND

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sys/time.h>
#include <vector>

#include "Arduino.h"
#include "ArduinoOTA.h"
#include "ETH.h"
#include "SparkFun_u-blox_GNSS_v3.h"
#include "ESP32TimeServerSettings.h"

extern "C"
{
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#if MQTT_ENABLED
#include "esp_heap_caps.h"
#include "mqtt_client.h"
#endif
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
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
static constexpr EventBits_t ETH_CONNECTED_BIT = BIT0;
static constexpr EventBits_t ETH_GOT_IP_BIT = BIT1;
static constexpr EventBits_t ETH_GOT_IP6_BIT = BIT2;
static constexpr size_t IP_ADDRESS_TEXT_SIZE = INET6_ADDRSTRLEN;
static constexpr uint32_t OTA_Failure_Display_Time_Ms = 10000;
static constexpr uint32_t OTA_Reboot_Delay_Ms = 5000;

static const char *TAG = "main_cpp";

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

static EventGroupHandle_t s_net_event_group = nullptr;
static SemaphoreHandle_t s_time_mutex = nullptr;
static SemaphoreHandle_t s_pps_semaphore = nullptr;
static QueueHandle_t s_pps_timestamp_queue = nullptr;
static SemaphoreHandle_t s_ota_mutex = nullptr;
#if LIQUID_CRYSTAL_DISPLAY_ENABLED
static SemaphoreHandle_t s_lcd_mutex = nullptr;
#endif
static SemaphoreHandle_t s_sync_state_mutex = nullptr;

#if OTE_UPDATES_ENABLED
static bool s_ota_in_progress = false;
static bool s_ota_failed = false;
static bool s_ota_success = false;
static int s_ota_progress_percent = -1;
static int64_t s_ota_failure_display_until_us = 0;
static int64_t s_ota_reboot_at_us = 0;
static char s_ota_error_reason[lcdColumns + 1] = "";
#endif

static std::atomic<bool> s_safe_guard_tripped{false};
static std::atomic<bool> s_time_setting_in_progress{false};
static std::atomic<bool> s_time_has_been_set{false};
static uint64_t s_ntp_reference_time_64 = 0;
static bool s_ntp_reference_valid = false;

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

static constexpr size_t MQTT_REPORT_QUEUE_DEPTH = 4; // be able to queue this many messages
static constexpr size_t MQTT_REPORT_SIZE = 4096;     // maximum message size
static constexpr size_t MQTT_CLIENT_LIMIT = 50;      // each message can contain request records for this many clients
static constexpr uint32_t MQTT_RESTART_PUBLISH_TIMEOUT_MS = 1000;

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
static esp_mqtt_client_handle_t s_mqtt_client = nullptr;
static std::atomic<bool> s_mqtt_setup_failed{false};
static std::atomic<bool> s_mqtt_connected{false};
static std::atomic<int> s_mqtt_restart_publish_id{-1};
static std::atomic<bool> s_mqtt_restart_publish_completed{false};
static std::atomic<uint32_t> s_pps_pulses{0};
static std::atomic<uint32_t> s_ntp_valid_requests{0};
static std::atomic<uint32_t> s_ntp_invalid_requests{0};
static std::atomic<uint32_t> s_ntp_responses{0};
static std::atomic<uint32_t> s_ntp_requests_with_gnss_lock{0};
static std::atomic<uint32_t> s_ntp_requests_without_gnss_lock{0};
static std::atomic<uint8_t> s_satellite_count{0};
static std::atomic<uint8_t> s_satellite_min{UINT8_MAX};
static std::atomic<uint8_t> s_satellite_max{0};
static std::atomic<int64_t> s_eth_link_connected_us{0};
static std::atomic<int64_t> s_eth_link_up_total_us{0};
#if MQTT_ENABLED && MQTT_HISTORICAL_REPORTING_ENABLED
static std::atomic<time_t> s_last_gnss_lock{0};
static std::atomic<time_t> s_last_gnss_unlock{0};
#endif

#if MQTT_ENABLED && MQTT_CLIENT_REPORTING_ENABLED
static std::atomic<bool> s_mqtt_client_table_overflown{false};
static char s_mqtt_clients_json[2048] = "";
static size_t s_mqtt_client_count = 0;
#endif

static int64_t s_mqtt_last_link_up_us = 0;
static mqtt_report_t s_mqtt_reports[MQTT_REPORT_QUEUE_DEPTH]{};
static char s_mqtt_payload[MQTT_REPORT_SIZE] = "";
static size_t s_mqtt_report_head = 0;
static std::atomic<size_t> s_mqtt_queued_messages_count{0};
static uint32_t s_mqtt_queued_messages_discarded = 0;
static mqtt_client_request_t s_mqtt_clients[MQTT_CLIENT_LIMIT]{};
static uint32_t s_ntp_requests_this_second = 0;
static uint32_t s_ntp_most_requests_per_second = 0;
static time_t s_ntp_request_second = 0;
static char s_mqtt_uri[64] = "";
static char s_mqtt_report_topic[128] = "";
static char s_mqtt_status_topic[128] = "";

static void mqtt_publish_final_report();
#endif

static HardwareSerial s_gps_serial(1);
static SFE_UBLOX_GNSS_SERIAL s_gps;
static uint32_t s_detected_gps_baud = 0;
static bool s_gps_required_assume_success = false;

// The following flag for National Marine Electronics Association fallback doesn't determine if it is allowed or not
// rather the program sets it to true if it is required (due to an older / ubox uncompliant hardware gps module being used)

static bool s_use_nmea_fallback = false;
static bool s_gps_is_max_m10s = false;
static std::atomic<bool> s_gnss_locked{false};
static std::atomic<int64_t> s_gnss_lock_started_us{0};
static std::atomic<int64_t> s_gnss_locked_total_us{0};
static std::atomic<bool> s_pps_discipline_active{false};

enum class sync_fault_t : uint8_t
{
    none = 0,
    pps_missing,
    gps_invalid,
    sanity_mismatch,
    sync_stale
};

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
    sync_fault_t fault = sync_fault_t::none;
};

struct sync_candidate_t
{
    time_t candidate_time = 0;
    bool use_pps_alignment = false;
    bool used_nmea_fallback = false;
    int64_t pps_release_time_us = 0;
    sync_fault_t failure = sync_fault_t::none;
};

static sync_state_t s_sync_state{};
static sync_state_t get_sync_state_snapshot();
static constexpr int64_t Sync_Stale_After_Us = static_cast<int64_t>(periodicGPSRefreshEveryThisNumberOfMinutes) * 60LL * 1000000LL * 3LL;
static constexpr int64_t Gnss_Validity_Timeout_Us = 3500000LL;
static constexpr int64_t Sync_Reboot_After_Us = 30LL * 60LL * 1000000LL;
static constexpr int64_t Max_Sync_Attempt_Us = 10000000LL;
static constexpr uint32_t Sync_Failures_Before_Runtime_Recovery = 3;
static constexpr uint32_t Sanity_Failures_Before_Fault = 2;

static constexpr char GPS_NVS_NAMESPACE[] = "gps_state";
static constexpr char GPS_NVS_KEY_ID_TYPE[] = "id_type";
static constexpr char GPS_NVS_KEY_ID_VALUE[] = "id_value";
static constexpr char GPS_NVS_KEY_MAX_BAUD[] = "max_baud";
static constexpr char GPS_NVS_KEY_ATTEMPT_NO_SIGNAL_RECOVERY[] = "no_sig_rcv";

static constexpr char GPS_ID_TYPE_UNIQID[] = "uniqid";
static constexpr char GPS_ID_TYPE_MODULE_FP[] = "module_fp";
static constexpr char GPS_ID_TYPE_GENERIC[] = "generic";
static constexpr char GPS_ID_VALUE_UNDETERMINED[] = "undetermined_gps_board";

struct gps_identity_t
{
    bool valid = false;
    char type[16] = "";
    char value[96] = "";
};

struct gps_nvs_data_t
{
    bool has_id_type = false;
    char id_type[16] = "";
    char id_value[96] = "";
    uint32_t max_baud = 0;
    bool has_attempt_no_signal_recovery = false;
    bool attempt_no_signal_recovery = true;
};

static uint32_t s_gps_target_baud = 0;
static bool s_attempt_no_signal_recovery = true;

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

    if (sync_stale)
    {
        state->fault = sync_fault_t::sync_stale;
    }
    else if (gnss_missing)
    {
        state->fault = sync_fault_t::gps_invalid;
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "GNSS invalid - GNSS missing.");
#endif
    }
    else if (pps_missing)
    {
        state->fault = sync_fault_t::pps_missing;
    }
    else if (state->fault == sync_fault_t::pps_missing || state->fault == sync_fault_t::gps_invalid || state->fault == sync_fault_t::sync_stale)
    {
        state->fault = sync_fault_t::none;
    }

    state->holdover_mode = sync_stale || pps_missing || gnss_missing || state->fault != sync_fault_t::none;
}

static void sync_state_note_attempt()
{
    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.last_sync_attempt_us = esp_timer_get_time();
        refresh_sync_state_locked(&s_sync_state, s_sync_state.last_sync_attempt_us);
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
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static void sync_state_note_pps_timeout(int64_t now_us)
{
    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.pps_active = false;
        refresh_sync_state_locked(&s_sync_state, now_us);
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
#if MQTT_ENABLED && MQTT_HISTORICAL_REPORTING_ENABLED
        s_last_gnss_lock.store(time(nullptr));
#endif
    }
    else
    {
        int64_t lock_started_us = s_gnss_lock_started_us.exchange(0);
        if (lock_started_us > 0 && now_us > lock_started_us)
            s_gnss_locked_total_us.fetch_add(now_us - lock_started_us);
#if MQTT_ENABLED && MQTT_HISTORICAL_REPORTING_ENABLED
        s_last_gnss_unlock.store(time(nullptr));
#endif
    }
}

static void sync_state_note_gnss_validity(bool valid)
{
    set_gnss_lock_state(valid);

    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        int64_t now_us = esp_timer_get_time();
        s_sync_state.gnss_timing_valid = valid;
        if (valid)
            s_sync_state.last_gnss_valid_us = now_us;
        refresh_sync_state_locked(&s_sync_state, now_us);
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static uint32_t sync_state_note_failure(sync_fault_t fault, time_t update_delta)
{
    uint32_t failure_count = 0;

    if (fault == sync_fault_t::gps_invalid)
        set_gnss_lock_state(false);

    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.last_sync_delta_seconds = update_delta;
        s_sync_state.consecutive_sync_failures++;
        s_sync_state.fault = fault;
        refresh_sync_state_locked(&s_sync_state, esp_timer_get_time());
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
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static void sync_state_note_success(time_t update_delta)
{
    set_gnss_lock_state(true);

    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_sync_state.last_successful_sync_us = esp_timer_get_time();
        s_sync_state.gnss_timing_valid = true;
        s_sync_state.last_gnss_valid_us = s_sync_state.last_successful_sync_us;
        s_sync_state.last_sync_delta_seconds = update_delta;
        s_sync_state.consecutive_sync_failures = 0;
        s_sync_state.consecutive_sanity_failures = 0;
        s_sync_state.fault = sync_fault_t::none;
        refresh_sync_state_locked(&s_sync_state, s_sync_state.last_successful_sync_us);
        xSemaphoreGive(s_sync_state_mutex);
    }
}

static sync_state_t get_sync_state_snapshot()
{
    sync_state_t snapshot{};

    if (xSemaphoreTake(s_sync_state_mutex, portMAX_DELAY) == pdTRUE)
    {
        refresh_sync_state_locked(&s_sync_state, esp_timer_get_time());
        snapshot = s_sync_state;
        xSemaphoreGive(s_sync_state_mutex);
    }

    return snapshot;
}

static const char *sync_fault_to_display_text(sync_fault_t fault)
{
    switch (fault)
    {
    case sync_fault_t::pps_missing:
        return "PPS missing";
    case sync_fault_t::gps_invalid:
        return "GPS invalid";
    case sync_fault_t::sanity_mismatch:
        return "Sanity check";
    case sync_fault_t::sync_stale:
        return "Sync stale";
    default:
        return "";
    }
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

static uint32_t get_highest_candidate_gps_baud()
{
    return 921600;
}

static void build_candidate_baud_rates(std::vector<uint32_t> &out, uint32_t preferred_baud)
{
    static constexpr uint32_t all_candidate_baud_rates[] = {921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600, 4800};

    out.clear();

    if (preferred_baud > 0)
        out.push_back(preferred_baud);

    for (auto br : all_candidate_baud_rates)
    {
        if (br != preferred_baud)
            out.push_back(br);
    }
}

static bool load_gps_nvs_data(gps_nvs_data_t *data)
{
    if (data == nullptr)
        return false;

    *data = gps_nvs_data_t{};

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(GPS_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
        return true;
    if (err != ESP_OK)
        return false;

    size_t type_length = 0;
    err = nvs_get_str(handle, GPS_NVS_KEY_ID_TYPE, nullptr, &type_length);
    if (err == ESP_OK && type_length > 0)
    {
        if (type_length > sizeof(data->id_type))
            type_length = sizeof(data->id_type);
        if (nvs_get_str(handle, GPS_NVS_KEY_ID_TYPE, data->id_type, &type_length) == ESP_OK)
            data->has_id_type = data->id_type[0] != '\0';
    }

    size_t value_length = 0;
    err = nvs_get_str(handle, GPS_NVS_KEY_ID_VALUE, nullptr, &value_length);
    if (err == ESP_OK && value_length > 0)
    {
        if (value_length > sizeof(data->id_value))
            value_length = sizeof(data->id_value);
        (void)nvs_get_str(handle, GPS_NVS_KEY_ID_VALUE, data->id_value, &value_length);
    }

    (void)nvs_get_u32(handle, GPS_NVS_KEY_MAX_BAUD, &data->max_baud);

    uint8_t attempt_no_signal_recovery = 0;
    err = nvs_get_u8(handle, GPS_NVS_KEY_ATTEMPT_NO_SIGNAL_RECOVERY, &attempt_no_signal_recovery);
    if (err == ESP_OK)
    {
        data->has_attempt_no_signal_recovery = true;
        data->attempt_no_signal_recovery = attempt_no_signal_recovery != 0;
    }

    nvs_close(handle);
    return true;
}

static bool save_gps_nvs_data(const gps_identity_t &identity, uint32_t max_baud)
{
    if (!identity.valid || identity.type[0] == '\0' || identity.value[0] == '\0' || max_baud == 0)
        return false;

    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(GPS_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return false;

    err = nvs_set_str(handle, GPS_NVS_KEY_ID_TYPE, identity.type);
    if (err == ESP_OK)
        err = nvs_set_str(handle, GPS_NVS_KEY_ID_VALUE, identity.value);
    if (err == ESP_OK)
        err = nvs_set_u32(handle, GPS_NVS_KEY_MAX_BAUD, max_baud);
    if (err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);
    return err == ESP_OK;
}

static bool save_attempt_no_signal_recovery_setting(bool enabled)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(GPS_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return false;

    err = nvs_set_u8(handle, GPS_NVS_KEY_ATTEMPT_NO_SIGNAL_RECOVERY, enabled ? 1U : 0U);
    if (err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);
    return err == ESP_OK;
}

static bool resolve_initial_attempt_no_signal_recovery(const gps_nvs_data_t &stored)
{
    if (stored.has_attempt_no_signal_recovery)
        return stored.attempt_no_signal_recovery;

    return !stored.has_id_type;
}

static void clear_gps_nvs_data()
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(GPS_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return;

    err = nvs_erase_key(handle, GPS_NVS_KEY_ID_TYPE);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return;
    }

    err = nvs_erase_key(handle, GPS_NVS_KEY_ID_VALUE);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return;
    }

    err = nvs_erase_key(handle, GPS_NVS_KEY_MAX_BAUD);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return;
    }

    err = nvs_erase_key(handle, GPS_NVS_KEY_ATTEMPT_NO_SIGNAL_RECOVERY);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(handle);
        return;
    }

    (void)nvs_commit(handle);
    nvs_close(handle);
}

static gps_identity_t query_gps_identity()
{
    gps_identity_t identity{};

    UBX_SEC_UNIQID_data_t unique_chip_data{};
    if (s_gps.getUniqueChipId(&unique_chip_data, 2000))
    {
        const char *unique_chip_id = s_gps.getUniqueChipIdStr(&unique_chip_data, 2000);
        if (unique_chip_id != nullptr && unique_chip_id[0] != '\0')
        {
            snprintf(identity.type, sizeof(identity.type), "%s", GPS_ID_TYPE_UNIQID);
            snprintf(identity.value, sizeof(identity.value), "%s", unique_chip_id);
            identity.valid = true;
            return identity;
        }
    }

    if (s_gps.getModuleInfo(2000))
    {
        const char *module_name = s_gps.getModuleName(2000);
        const char *firmware_type = s_gps.getFirmwareType(2000);
        uint8_t firmware_high = s_gps.getFirmwareVersionHigh(2000);
        uint8_t firmware_low = s_gps.getFirmwareVersionLow(2000);
        uint8_t protocol_high = s_gps.getProtocolVersionHigh(2000);
        uint8_t protocol_low = s_gps.getProtocolVersionLow(2000);

        if (module_name != nullptr && module_name[0] != '\0')
        {
            snprintf(identity.type, sizeof(identity.type), "%s", GPS_ID_TYPE_MODULE_FP);
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

    snprintf(identity.type, sizeof(identity.type), "%s", GPS_ID_TYPE_GENERIC);
    snprintf(identity.value, sizeof(identity.value), "%s", GPS_ID_VALUE_UNDETERMINED);
    identity.valid = true;
    return identity;
}

static bool gps_identity_matches(const gps_nvs_data_t &stored, const gps_identity_t &current)
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
        clear_gps_nvs_data();
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "Up time/reset button restart requested: cleared stored GPS NVS values.");
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
    uint64_t fraction = static_cast<uint64_t>(static_cast<double>(now.tv_usec) * 4294.967296);
    return (seconds << 32) | (fraction & 0xFFFFFFFFULL);
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
};

static ntp_reply_status_t get_ntp_reply_status()
{
    sync_state_t state = get_sync_state_snapshot();
    int64_t now_us = esp_timer_get_time();
    bool gnss_recent = state.gnss_timing_valid && state.last_gnss_valid_us > 0 &&
                       (now_us - state.last_gnss_valid_us) <= Gnss_Validity_Timeout_Us;
    bool stratum_one = s_time_has_been_set.load() && state.fault == sync_fault_t::none &&
                       state.pps_active && gnss_recent && s_ntp_reference_valid;

    if (stratum_one)
        return {0, 1, "GPS", true};

    return {};
}

static void build_ntp_reply(const uint8_t *request, uint8_t *reply, uint64_t receive_time, uint64_t transmit_time, const ntp_reply_status_t &status)
{
    memset(reply, 0, NTP_PACKET_SIZE);

    reply[0] = static_cast<uint8_t>((status.leap_indicator << 6) | 0b00100100);
    reply[1] = status.stratum;
    reply[2] = 4;
    reply[3] = 0xF7;
    reply[11] = 0x42;
    memcpy(reply + 12, status.reference_id, 4);

    if (status.reference_time_valid)
        write_ntp_timestamp(reply, 16, s_ntp_reference_time_64);

    memcpy(reply + 24, request + 40, 8);

    write_ntp_timestamp(reply, 32, receive_time);
    write_ntp_timestamp(reply, 40, transmit_time);
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
        while (s_gps_serial.available() > 0)
        {
            int value = s_gps_serial.read();
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

struct gps_probe_result_t
{
    bool saw_data = false;
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
    if (!s_gps.getPVT(status_query_timeout_ms))
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "getPVT failed");
#endif
        return false;
    };

    uint8_t fix_type = s_gps.getFixType(status_query_timeout_ms); // 0 - No fix; 1 - Dead reckoning only; 2 - 2D-fix; 3: 3D-fix; 4 - GPS + dead reckoning combined; 5 - Time only fix
#if MQTT_ENABLED
    uint8_t satellites = s_gps.getSIV(status_query_timeout_ms);
    s_satellite_count.store(satellites);
    if (satellites < s_satellite_min.load())
        s_satellite_min.store(satellites);
    if (satellites > s_satellite_max.load())
        s_satellite_max.store(satellites);
#endif
    bool gnss_fix_ok = s_gps.getGnssFixOk(status_query_timeout_ms);
    bool date_valid = s_gps.getDateValid(status_query_timeout_ms);
    bool time_valid = s_gps.getTimeValid(status_query_timeout_ms);

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

static gps_probe_result_t probe_gps_uart(uint32_t baud);

static gps_probe_result_t probe_gps_uart(uint32_t baud)
{
    static constexpr uint16_t gpsProbeListenTimeMs = 1500;
    gps_probe_result_t result{};
    size_t sample_len = 0;

    s_gps_serial.end();
    s_gps_serial.begin(baud, SERIAL_8N1, RXPin, TXPin);
    vTaskDelay(pdMS_TO_TICKS(150));

    uint32_t start_ms = millis();
    while ((millis() - start_ms) < gpsProbeListenTimeMs)
    {
        while (s_gps_serial.available() > 0)
        {
            int value = s_gps_serial.read();
            if (value < 0)
                break;

            result.saw_data = true;
            result.bytes_seen++;

            if (sample_len < (sizeof(result.sample) - 1))
            {
                char character = static_cast<char>(value);
                if (character >= 32 && character <= 126)
                {
                    result.sample[sample_len++] = character;
                }
                else if (character == '\r' || character == '\n' || character == '\t')
                {
                    result.sample[sample_len++] = ' ';
                }
                else
                {
                    result.sample[sample_len++] = '.';
                }
                result.sample[sample_len] = '\0';
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return result;
}

static bool try_gps_begin(uint32_t baud, bool assume_success, bool *saw_serial_data)
{

    static constexpr uint16_t gpsBeginMaxWaitMs = 2500;

    gps_probe_result_t probe_result = probe_gps_uart(baud);
    if (probe_result.saw_data && saw_serial_data != nullptr)
        *saw_serial_data = true;

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GPS probe at %lu baud: %u bytes seen, sample: %s",
             static_cast<unsigned long>(baud),
             static_cast<unsigned int>(probe_result.bytes_seen),
             probe_result.saw_data ? probe_result.sample : "<none>");
#endif

    s_gps_serial.end();
    s_gps_serial.begin(baud, SERIAL_8N1, RXPin, TXPin);
    vTaskDelay(pdMS_TO_TICKS(150));

    bool begin_result = s_gps.begin(s_gps_serial, gpsBeginMaxWaitMs, assume_success);
#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GPS begin at %lu baud with assume_success=%s -> %s",
             static_cast<unsigned long>(baud),
             assume_success ? "true" : "false",
             begin_result ? "success" : "failed");
#endif

    return begin_result;
}

static bool set_gps_baud_rate(uint32_t gps_baud, int max_attempts, uint32_t initial_probe_baud = 0)
{

    std::vector<uint32_t> candidate_baud_rates;
    uint32_t preferred_probe_baud = initial_probe_baud > 0 ? initial_probe_baud : gps_baud;
    build_candidate_baud_rates(candidate_baud_rates, preferred_probe_baud);

    bool saw_any_serial_data = false;
    s_detected_gps_baud = 0;
    s_gps_required_assume_success = false;

    char baud_line[lcdColumns + 1];

    for (int attempt = 0; attempt < max_attempts; ++attempt)
    {
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "GPS initialization attempt %d of %d", attempt + 1, max_attempts);
#endif

        for (uint32_t candidate_baud : candidate_baud_rates)
        {

            char dots[10] = "";
            for (int i = 0; i < (attempt % 4); i++)
                strcat(dots, ".");

            snprintf(baud_line, sizeof(baud_line), "Baud: %lu %s", static_cast<unsigned long>(candidate_baud), dots);
            display_line(2, baud_line);

            bool begin_without_assume = try_gps_begin(candidate_baud, false, &saw_any_serial_data);
            bool begin_with_assume = false;
            if (!begin_without_assume)
                begin_with_assume = try_gps_begin(candidate_baud, true, &saw_any_serial_data);

            if (begin_without_assume || begin_with_assume)
            {
                s_detected_gps_baud = candidate_baud;
                s_gps_required_assume_success = begin_with_assume;

                if (candidate_baud != gps_baud)
                {
#if DEBUG_ENABLED
                    ESP_LOGI(TAG,
                             "GPS responded at %lu baud. Attempting to switch to %lu baud.",
                             static_cast<unsigned long>(candidate_baud),
                             static_cast<unsigned long>(gps_baud));
#endif

                    bool baud_change_command_reported_success = s_gps.setSerialRate(gps_baud);

#if DEBUG_ENABLED
                    if (!baud_change_command_reported_success)
                        ESP_LOGW(TAG, "GPS baud-rate change command to %lu was not acknowledged. Probing the target baud anyway.", static_cast<unsigned long>(gps_baud));
#endif

                    vTaskDelay(pdMS_TO_TICKS(200));

                    bool reconnect_without_assume = try_gps_begin(gps_baud, false, &saw_any_serial_data);
                    bool reconnect_with_assume = false;
                    if (!reconnect_without_assume)
                        reconnect_with_assume = try_gps_begin(gps_baud, true, &saw_any_serial_data);

                    if (reconnect_without_assume || reconnect_with_assume)
                    {
                        s_detected_gps_baud = gps_baud;
                        s_gps_required_assume_success = reconnect_with_assume;

#if DEBUG_ENABLED
                        if (!baud_change_command_reported_success)
                            ESP_LOGW(TAG, "GPS baud-rate change command to %lu reported failure, but reconnect succeeded at the new baud.", static_cast<unsigned long>(gps_baud));
#endif
                    }
                    else
                    {
                        bool old_baud_without_assume = try_gps_begin(candidate_baud, false, &saw_any_serial_data);
                        bool old_baud_with_assume = false;
                        if (!old_baud_without_assume)
                            old_baud_with_assume = try_gps_begin(candidate_baud, true, &saw_any_serial_data);

                        if (old_baud_without_assume || old_baud_with_assume)
                        {
                            s_detected_gps_baud = candidate_baud;
                            s_gps_required_assume_success = old_baud_with_assume;

                            if (baud_change_command_reported_success && rebootIfGpsBaudChangeCommandSucceedsButImmediateReconnectFails)
                            {
#if DEBUG_ENABLED
                                ESP_LOGW(TAG, "GPS baud-rate change command to %lu was acknowledged, but immediate reconnect failed. Rebooting to complete transition.", static_cast<unsigned long>(gps_baud));
#endif
                                display_line(1, "GPS baud changed");
                                display_line(2, "Rebooting...");
                                vTaskDelay(pdMS_TO_TICKS(1000));
#if MQTT_ENABLED
                                mqtt_publish_final_report();
#endif
                                esp_restart();
                            }

#if DEBUG_ENABLED
                            ESP_LOGW(TAG, "GPS baud-rate change to %lu did not take effect immediately. Continuing at detected baud %lu.", static_cast<unsigned long>(gps_baud), static_cast<unsigned long>(candidate_baud));
#endif
                        }
                        else
                        {
#if DEBUG_ENABLED
                            ESP_LOGW(TAG, "GPS baud-rate change attempt left module unreachable at both %lu and %lu. Continuing with best-known state.", static_cast<unsigned long>(candidate_baud), static_cast<unsigned long>(gps_baud));
#endif
                        }
                    }
                }

#if DEBUG_ENABLED
                if (s_gps_required_assume_success)
                {
                    ESP_LOGW(TAG,
                             "GPS communication was established only with assume_success=true at %lu baud. The attached module may have limited u-blox compatibility.",
                             static_cast<unsigned long>(s_detected_gps_baud));
                };

                ESP_LOGI(TAG, "GPS initialization will continue at %lu baud.", static_cast<unsigned long>(s_detected_gps_baud));
#endif
                return true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

#if DEBUG_ENABLED
    if (saw_any_serial_data)
        ESP_LOGW(TAG, "GPS serial data was detected, but SparkFun u-blox GNSS v3 could not initialize the module. The module may be an older NEO-6/7/M8 variant.");
    else
        ESP_LOGW(TAG, "No GPS serial data was detected on GPIO%d/GPIO%d at any tested baud rate. Check power, TX/RX wiring, and signal levels.", RXPin, TXPin);
#endif

    return false;
}

static bool configure_gps_outputs(bool *uart1_output_set_result)
{
    bool i2c_output_disabled = s_gps.setI2COutput(0);
    bool uart1_output_set = s_gps.setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA);
    bool uart2_output_disabled = s_gps.setUART2Output(0);

    if (uart1_output_set_result != nullptr)
        *uart1_output_set_result = uart1_output_set;

#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GPS port config results: I2C off=%s, UART1 UBX=%s, UART2 off=%s",
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

static bool perform_extended_no_signal_recovery()
{
#if DEBUG_ENABLED
    ESP_LOGW(TAG, "Performing GPS extended no-signal recovery: factory-default + GNSS reset + reinitialization.");
#endif

    bool factory_default_applied = s_gps.factoryDefault();
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "GPS factoryDefault() -> %s", factory_default_applied ? "ok" : "failed");
#endif

    s_gps.hardReset();
    vTaskDelay(pdMS_TO_TICKS(1500));

    static constexpr int recoveryInitAttempts = 3;
    uint32_t recovery_target_baud = s_gps_target_baud > 0 ? s_gps_target_baud : get_highest_candidate_gps_baud();
    if (!set_gps_baud_rate(recovery_target_baud, recoveryInitAttempts))
    {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "GPS recovery reinitialization failed after reset.");
#endif
        return false;
    }

    bool uart1_output_set = false;
    bool gps_outputs_configured = configure_gps_outputs(&uart1_output_set);
    s_use_nmea_fallback = s_gps_required_assume_success || !uart1_output_set;

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "GPS recovery post-config: outputs=%s, fallback=%s, baud=%lu",
             gps_outputs_configured ? "ok" : "partial",
             s_use_nmea_fallback ? "enabled" : "disabled",
             static_cast<unsigned long>(s_detected_gps_baud));
#endif

    return true;
}

static void halt_with_display(const char *line1, const char *line2, const char *line3)
{
    display_line(1, line1);
    display_line(2, line2);
    display_line(3, line3);
    while (true)
        vTaskDelay(pdMS_TO_TICKS(1000));
}

static void IRAM_ATTR pps_isr_handler(void *arg)
{
    int64_t edge_us = esp_timer_get_time();
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_pps_semaphore != nullptr)
        xSemaphoreGiveFromISR(s_pps_semaphore, &higher_priority_task_woken);
    if (s_pps_timestamp_queue != nullptr)
        xQueueOverwriteFromISR(s_pps_timestamp_queue, &edge_us, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE)
        portYIELD_FROM_ISR();
}

static void setup_pps_input()
{
    gpio_config_t config{};
    config.pin_bit_mask = 1ULL << PPSPin;
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_POSEDGE;
    ESP_ERROR_CHECK(gpio_config(&config));

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(gpio_isr_handler_add(static_cast<gpio_num_t>(PPSPin), pps_isr_handler, nullptr));
}

static void clear_pps_events() // to do
{
    while (xSemaphoreTake(s_pps_semaphore, 0) == pdTRUE)
    {
    }
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
        int64_t edge_us = 0;
        if (xQueueReceive(s_pps_timestamp_queue, &edge_us, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            last_pps_us = edge_us;
            s_pps_discipline_active.store(true);
#if MQTT_ENABLED
            s_pps_pulses.fetch_add(1);
#endif
            sync_state_note_pps_edge(edge_us);

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

        /*

        // The following code is used to determine the ideal stack size for this method (pps_discipline_task)
        // it only needs to be setup and run once
        //
        // NOTE 1: Specific to pps_discipline_task to get a valid result the code below needed to be running
        //         with the Stack report usage display being examined when anPPS event happens (every second)
        //
        // NOTE 2: Results were taken after a minute to allow the numbers to settle in
        //
        // NOTE 3: The following is required to have the code directly below compile (build):
        //         From the ESP-IDF Terminal enter the command
        //            idf.py menuconfig
        //            (the menu screen may take 30 seconds or more to load)
        //         Navigate to:
        //            Component config → FreeRTOS → Kernel
        //         use the down arrow key to scroll down to
        //            configUSE_TRACE_FACILITY
        //            enable FreeTOS trace facility by pressing the space bar (puts a star in the option [*] to signify it is enabled)
        //         press 'S' (enter) to save the change
        //         press the ESC key
        //         press 'Q' to quit
        //         code should now build, flash and run ok
        //
        //         once the ideal stack size is known, the code can below can be commented out and the configUSE_TRACE_FACILITY disabled to save RAM
        //
        // NOTE 4: if in the future this program changes significantly the suggested stack size value generated in this testing may need to be redone
        //
        // Below is the code that has now already been used to calculate the ideal stack size:
        //

        // *** Hard‑coded stack size originally used when creating the task (bytes) ***
        const size_t allocated_bytes = 16384; // <-- set this to the value passed to xTaskCreatePinnedToCore

        // high watermark is returned in words
        UBaseType_t high_watermark_words = uxTaskGetStackHighWaterMark(NULL);
        size_t high_watermark_bytes = (size_t)high_watermark_words * sizeof(StackType_t);

        // compute peak usage and suggested size (25% margin)
        size_t peak_usage_bytes = (allocated_bytes > high_watermark_bytes) ? (allocated_bytes - high_watermark_bytes) : 0;
        size_t suggested_bytes = (size_t)((double)peak_usage_bytes * 1.25);

        ESP_LOGI("pps_discipline_task",
                 "Stack report: Allocated=%u bytes, HighWater=%u bytes unused, PeakUsage=%u bytes, Suggested=%u bytes",
                 (unsigned)allocated_bytes,
                 (unsigned)high_watermark_bytes,
                 (unsigned)peak_usage_bytes,
                 (unsigned)suggested_bytes);

        // Results of this testing (when a PPS event was underway):
        //
        // pps_discipline_task: Stack report: Allocated=16384 bytes, HighWater=540 bytes unused, PeakUsage=15844 bytes, Suggested=19805 bytes

        */
    }
}

static void setup_gps()
{

    static constexpr int maxAttemptsToInitializeGPS = 10;

    gps_nvs_data_t stored_gps_data{};
    bool gps_nvs_load_ok = load_gps_nvs_data(&stored_gps_data);

    bool first_time_initial_setup = !stored_gps_data.has_id_type;
    ESP_LOGI(TAG, "Startup mode: %s", first_time_initial_setup ? "First time initial setup" : "Not first time initial setup");

    s_attempt_no_signal_recovery = resolve_initial_attempt_no_signal_recovery(stored_gps_data);
    if (gps_nvs_load_ok && !stored_gps_data.has_attempt_no_signal_recovery)
    {
        bool save_recovery_setting_ok = save_attempt_no_signal_recovery_setting(s_attempt_no_signal_recovery);
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "GPS no-signal recovery default initialized to %s (save=%s)",
                 s_attempt_no_signal_recovery ? "true" : "false",
                 save_recovery_setting_ok ? "ok" : "failed");
#endif
    }

    uint32_t highest_candidate_baud = get_highest_candidate_gps_baud();
    uint32_t startup_target_baud = highest_candidate_baud;

    if (gps_nvs_load_ok && stored_gps_data.has_id_type && stored_gps_data.max_baud > 0)
        startup_target_baud = stored_gps_data.max_baud;

    bool known_module_fast_path = gps_nvs_load_ok && stored_gps_data.has_id_type && stored_gps_data.max_baud > 0;
#if DEBUG_ENABLED
    ESP_LOGI(TAG,
             "GPS startup path: %s (target baud=%lu)",
             known_module_fast_path ? "known module fast path" : "first-time scan path",
             static_cast<unsigned long>(startup_target_baud));
#endif

    s_gps_target_baud = startup_target_baud;
    uint32_t initial_probe_baud = first_time_initial_setup ? 9600 : s_gps_target_baud;

    if (!set_gps_baud_rate(s_gps_target_baud, maxAttemptsToInitializeGPS, initial_probe_baud))
    {
        ESP_LOGE(TAG, "GPS comms failed - check TX/RX + power");
        halt_with_display("GPS comms failed", "Check TX/RX + power", "See serial log");
    }

    gps_identity_t current_identity = query_gps_identity();
    bool should_retry_as_first_time = gps_nvs_load_ok && stored_gps_data.has_id_type;
    if (should_retry_as_first_time)
    {
        bool same_gps_module = gps_identity_matches(stored_gps_data, current_identity);
        if (!same_gps_module && startup_target_baud != highest_candidate_baud)
        {
            s_gps_target_baud = highest_candidate_baud;
            if (!set_gps_baud_rate(s_gps_target_baud, maxAttemptsToInitializeGPS, s_gps_target_baud))
            {
                ESP_LOGE(TAG, "GPS comms failed after module mismatch fallback");
                halt_with_display("GPS comms failed", "Check TX/RX + power", "See serial log");
            }
            current_identity = query_gps_identity();
        }
    }

    s_gps_is_max_m10s = strstr(current_identity.value, "MAX-M10S") != nullptr;
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "GPS module profile: %s", s_gps_is_max_m10s ? "MAX-M10S" : "generic/fallback");

    ESP_LOGI(TAG, "GPS startup mode: baud=%lu, initialization=%s",
             static_cast<unsigned long>(s_detected_gps_baud),
             s_gps_required_assume_success ? "assume_success" : "confirmed");
#endif

    if (s_detected_gps_baud > 0)
    {
        bool save_ok = save_gps_nvs_data(current_identity, s_detected_gps_baud);
#if DEBUG_ENABLED
        ESP_LOGI(TAG,
                 "GPS identity persistence: type=%s, value=%s, max_baud=%lu, save=%s",
                 current_identity.type,
                 current_identity.value,
                 static_cast<unsigned long>(s_detected_gps_baud),
                 save_ok ? "ok" : "failed");
#endif
    }

    bool uart1_output_set = false;
    configure_gps_outputs(&uart1_output_set);
#if DEBUG_ENABLED
    if (!uart1_output_set)
        ESP_LOGW(TAG, "GPS UART1 output configuration failed. Continuing with the module's current output settings.");
#endif

    s_use_nmea_fallback = s_gps_required_assume_success || !uart1_output_set;
    if (s_use_nmea_fallback && !allowFallbackProcessing)
    {
        ESP_LOGE(TAG, "NMEA fallback processing is required but not allowed - check the ESP32TimeServerSetting.h file.");
        halt_with_display("GPS fallback required", "Fallback not allowed", "Check settings");
    }
#if DEBUG_ENABLED
    if (s_use_nmea_fallback)
        ESP_LOGW(TAG, "NMEA fallback mode enabled for GPS fix/time acquisition.");
#endif

    display_line(1, "Waiting for GPS fix");
    display_line(2, "");

    static constexpr uint32_t gpsFixEscalationToNmeaFallbackMs = 120000UL;
    static constexpr uint32_t gpsNoSignalRecoveryMs = 180000UL;

    bool no_signal_recovery_attempted = false;
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
            fix_type = s_gps.getFixType();
            gnss_fix_ok = s_gps.getGnssFixOk();
            satellites_used = s_gps.getSIV();
            date_valid = s_gps.getDateValid();
            time_valid = s_gps.getTimeValid();
            ubx_fix_ready = (fix_type == 3 || fix_type == 4 || fix_type == 5) && gnss_fix_ok && date_valid && time_valid;

            char satellites_line[lcdColumns + 1];
            snprintf(satellites_line, sizeof(satellites_line), "Satellites: %u of 4", static_cast<unsigned int>(satellites_used));
            display_line(2, satellites_line);
        }

        uint32_t now_ms = millis();
        if (!ubx_fix_ready && !nmea_fallback_enabled && allowFallbackProcessing && (now_ms - wait_start_ms) >= gpsFixEscalationToNmeaFallbackMs)
        {
            nmea_fallback_enabled = true;
#if DEBUG_ENABLED
            ESP_LOGW(TAG, "No UBX fix after %lu ms. Enabling NMEA fallback for additional acquisition path.", static_cast<unsigned long>(now_ms - wait_start_ms));
#endif
        }

        if (!ubx_fix_ready && !s_use_nmea_fallback && !no_signal_recovery_attempted && s_attempt_no_signal_recovery && satellites_used == 0 && (now_ms - wait_start_ms) >= gpsNoSignalRecoveryMs)
        {
            no_signal_recovery_attempted = true;
            if (perform_extended_no_signal_recovery())
            {
                wait_start_ms = millis();
                last_status_log_ms = 0;
                last_long_wait_warning_ms = 0;
                continue;
            }
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
                ESP_LOGI(TAG, "GPS fix obtained after %lu ms: fix_type=%u (%s)",
                         static_cast<unsigned long>(millis() - wait_start_ms),
                         static_cast<unsigned int>(fix_type),
                         fix_type_to_text(fix_type));
            else
                ESP_LOGI(TAG, "GPS fix obtained after %lu ms via NMEA fallback.",
                         static_cast<unsigned long>(millis() - wait_start_ms));
#endif

            display_line(1, "GPS fix obtained");
            display_line(2, ubx_fix_ready ? fix_type_to_text(fix_type) : "NMEA fallback");
            vTaskDelay(pdMS_TO_TICKS(5000));
            return;
        }

        if (last_status_log_ms == 0 || (now_ms - last_status_log_ms) >= 5000UL)
        {
#if DEBUG_ENABLED

            if (s_use_nmea_fallback)

                ESP_LOGI(TAG, "Waiting for GPS fix: elapsed=%lu ms, NMEA fallback active",
                         static_cast<unsigned long>(now_ms - wait_start_ms));
            else
                ESP_LOGI(TAG, "Waiting for GPS fix: elapsed=%lu ms, fix_type=%u (%s), gnss_fix_ok=%s, date_valid=%s, time_valid=%s, SIV=%u",
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

        if ((now_ms - wait_start_ms) >= 60000UL && (last_long_wait_warning_ms == 0 || (now_ms - last_long_wait_warning_ms) >= 30000UL))
        {
#if DEBUG_ENABLED
            ESP_LOGW(TAG, "GPS has not produced a usable fix yet. Check antenna placement, sky view, and module compatibility.");
#endif
            last_long_wait_warning_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#if MQTT_ENABLED
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

    if (s_mqtt_stats_mutex == nullptr || xSemaphoreTake(s_mqtt_stats_mutex, 0) != pdTRUE)
        return;

    time_t now = time(nullptr);
    if (s_ntp_request_second != now)
    {
        s_ntp_request_second = now;
        s_ntp_requests_this_second = 0;
    }
    s_ntp_requests_this_second++;
    if (s_ntp_requests_this_second > s_ntp_most_requests_per_second)
        s_ntp_most_requests_per_second = s_ntp_requests_this_second;

#if MQTT_ENABLED && MQTT_CLIENT_REPORTING_ENABLED

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

    if (s_mqtt_client_count < MQTT_CLIENT_LIMIT)
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

static void mqtt_format_time(time_t value, char *output, size_t output_size)
{
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
        s_mqtt_connected.store(true);
    else if (event_id == MQTT_EVENT_DISCONNECTED || event_id == MQTT_EVENT_ERROR)
        s_mqtt_connected.store(false);
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

    return esp_mqtt_client_publish(s_mqtt_client, s_mqtt_report_topic, payload, 0, MQTT_QOS, 0) >= 0;
}

static void mqtt_enqueue_report(const char *payload)
{
    if (MQTT_QOS == 0)
        return;

    if (s_mqtt_queued_messages_count.load() == MQTT_REPORT_QUEUE_DEPTH)
    {
        s_mqtt_report_head = (s_mqtt_report_head + 1) % MQTT_REPORT_QUEUE_DEPTH;
        s_mqtt_queued_messages_count--;
        s_mqtt_queued_messages_discarded++;
    }

    size_t index = (s_mqtt_report_head + s_mqtt_queued_messages_count.load()) % MQTT_REPORT_QUEUE_DEPTH;
    snprintf(s_mqtt_reports[index].payload, sizeof(s_mqtt_reports[index].payload), "%s", payload);
    s_mqtt_queued_messages_count++;
}

static void mqtt_send_queued_messages()
{
    if (MQTT_QOS == 0)
        return;

    while (s_mqtt_queued_messages_count.load() > 0 && s_mqtt_connected.load())
    {
        if (!mqtt_publish_report(s_mqtt_reports[s_mqtt_report_head].payload))
            return;
        s_mqtt_report_head = (s_mqtt_report_head + 1) % MQTT_REPORT_QUEUE_DEPTH;
        s_mqtt_queued_messages_count--;
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
    uint32_t most_requests_per_second = 0;

#if MQTT_CLIENT_REPORTING_ENABLED
    s_mqtt_clients_json[0] = '\0';
    size_t clients_length = 0;

    if (xSemaphoreTake(s_mqtt_stats_mutex, portMAX_DELAY) == pdTRUE)
    {
        most_requests_per_second = s_ntp_most_requests_per_second;
        s_ntp_most_requests_per_second = s_ntp_requests_this_second;
        for (size_t index = 0; index < s_mqtt_client_count; index++)
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
            int written = snprintf(s_mqtt_clients_json + clients_length, sizeof(s_mqtt_clients_json) - clients_length,
                                   "%s{\"address\":\"%s\",\"requests\":%lu}",
                                   clients_length == 0 ? "" : ",", address_text,
                                   static_cast<unsigned long>(s_mqtt_clients[index].requests));
            if (written < 0 || static_cast<size_t>(written) >= sizeof(s_mqtt_clients_json) - clients_length)
                break;
            clients_length += static_cast<size_t>(written);
        }
        s_mqtt_client_count = 0;
        xSemaphoreGive(s_mqtt_stats_mutex);
    }

#else

    if (xSemaphoreTake(s_mqtt_stats_mutex, portMAX_DELAY) == pdTRUE)
    {
        most_requests_per_second = s_ntp_most_requests_per_second;
        s_ntp_most_requests_per_second = s_ntp_requests_this_second;

        xSemaphoreGive(s_mqtt_stats_mutex);
    }

#endif

    char publishing_date_and_time[25] = "";
    mqtt_format_time(time(nullptr), publishing_date_and_time, sizeof(publishing_date_and_time));

#if MQTT_HISTORICAL_REPORTING_ENABLED
    char last_lock[25] = "";
    char last_unlock[25] = "";
    mqtt_format_time(s_last_gnss_lock.load(), last_lock, sizeof(last_lock));
    mqtt_format_time(s_last_gnss_unlock.load(), last_unlock, sizeof(last_unlock));
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
        satellite_min = s_satellite_count.load();

    size_t len = 0;

    len += snprintf(payload + len, payload_size - len, "{");

    len += snprintf(payload + len, payload_size - len, "\"current\": {");
    len += snprintf(payload + len, payload_size - len, "\"time\":\"%s\",", publishing_date_and_time);
    len += snprintf(payload + len, payload_size - len, "\"uptime\":%lu,", (unsigned long)(esp_timer_get_time() / 1000000LL));
    len += snprintf(payload + len, payload_size - len, "\"ethernet_up\":%s,", s_ethernet_connected.load() ? "true" : "false");
    len += snprintf(payload + len, payload_size - len, "\"pps_active\":%s,", s_pps_discipline_active.load() ? "true" : "false");
    len += snprintf(payload + len, payload_size - len, "\"gnss_locked\":%s,", s_gnss_locked.load() ? "true" : "false");
    len += snprintf(payload + len, payload_size - len, "\"satellites\":%u", (unsigned int)s_satellite_count.load());

#if MQTT_MEMORY_REPORTING_ENABLED
    len += snprintf(payload + len, payload_size - len, ",");
    len += snprintf(payload + len, payload_size - len, "\"memory\":{");
    len += snprintf(payload + len, payload_size - len, "\"malloc_cap_8bit\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_8BIT));
    len += snprintf(payload + len, payload_size - len, "\"malloc_cap_32bit\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_32BIT));
    len += snprintf(payload + len, payload_size - len, "\"malloc_cap_internal\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    len += snprintf(payload + len, payload_size - len, "\"malloc_cap_dma\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA));
    len += snprintf(payload + len, payload_size - len, "\"malloc_cap_spiram\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    len += snprintf(payload + len, payload_size - len, "\"malloc_cap_default\":%lu,", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
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
    len += snprintf(payload + len, payload_size - len, "\"max_per_second\":%lu", (unsigned long)most_requests_per_second);
    len += snprintf(payload + len, payload_size - len, "},");
    len += snprintf(payload + len, payload_size - len, "\"responses\":{");
    len += snprintf(payload + len, payload_size - len, "\"gnss_lock\":%lu,", (unsigned long)s_ntp_requests_with_gnss_lock.exchange(0));
    len += snprintf(payload + len, payload_size - len, "\"gnss_unlock\":%lu", (unsigned long)s_ntp_requests_without_gnss_lock.exchange(0));
    len += snprintf(payload + len, payload_size - len, "}");

#if MQTT_CLIENT_REPORTING_ENABLED
    len += snprintf(payload + len, payload_size - len, "},");
    len += snprintf(payload + len, payload_size - len, "\"clients\":[%s],", s_mqtt_clients_json);
    len += snprintf(payload + len, payload_size - len, "\"clients_overflown\":%s", s_mqtt_client_table_overflown.exchange(false) ? "true" : "false");
#else
    len += snprintf(payload + len, payload_size - len, "}");
#endif

#if MQTT_HISTORICAL_REPORTING_ENABLED
    len += snprintf(payload + len, payload_size - len, "},");
    len += snprintf(payload + len, payload_size - len, "\"historical\":{");
    len += snprintf(payload + len, payload_size - len, "\"gnss\":{");
    len += snprintf(payload + len, payload_size - len, "\"last_lock\":\"%s\",", last_lock);
    len += snprintf(payload + len, payload_size - len, "\"last_unlock\":\"%s\"", last_unlock);
    len += snprintf(payload + len, payload_size - len, "}");
#endif

    len += snprintf(payload + len, payload_size - len, "}");
    len += snprintf(payload + len, payload_size - len, "}");

#if DEBUG_ENABLED
    ESP_LOGI(TAG, "Published: \n\r%s", payload);
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
    int message_id = esp_mqtt_client_publish(s_mqtt_client, s_mqtt_report_topic, payload, 0, MQTT_QOS, 0);
    if (message_id < 0)
        return;

    s_mqtt_restart_publish_id.store(message_id);
    TickType_t wait_started = xTaskGetTickCount();
    TickType_t wait_timeout = pdMS_TO_TICKS(MQTT_RESTART_PUBLISH_TIMEOUT_MS);
    while (!s_mqtt_restart_publish_completed.load() && (xTaskGetTickCount() - wait_started) < wait_timeout)
        vTaskDelay(pdMS_TO_TICKS(10));

    s_mqtt_restart_publish_id.store(-1);
}

static void mqtt_service_task(void *parameter)
{
    bool previously_connected = false;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(MQTTReportingPeriod * 1000UL));
        bool connected = s_mqtt_connected.load();
        if (connected && !previously_connected)
            esp_mqtt_client_publish(s_mqtt_client, s_mqtt_status_topic, "online", 0, MQTT_QOS, 1);
        previously_connected = connected;
        mqtt_send_queued_messages();
        mqtt_build_report(s_mqtt_payload, sizeof(s_mqtt_payload));
        if (!mqtt_publish_report(s_mqtt_payload))
            mqtt_enqueue_report(s_mqtt_payload);
    }
}

#endif

static void setup_mqtt()
{
#if MQTT_ENABLED

    if (MQTTServerIPAddress[0] == '\0' || MQTT_QOS < 0 || MQTT_QOS > 2)
    {
        s_mqtt_setup_failed.store(true);
        return;
    }

    s_mqtt_stats_mutex = xSemaphoreCreateMutex();
    if (s_mqtt_stats_mutex == nullptr)
    {
        s_mqtt_setup_failed.store(true);
        return;
    }
    snprintf(s_mqtt_uri, sizeof(s_mqtt_uri), "mqtt://%s:%u", MQTTServerIPAddress, static_cast<unsigned int>(MQTT_Port));
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
    if (xTaskCreatePinnedToCore(mqtt_service_task, "mqtt_service", 6144, nullptr, 5, nullptr, 0) != pdPASS)
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
#if MQTT_ENABLED
        s_ethernet_connected.store(true);
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
#if MQTT_ENABLED
        s_ethernet_connected.store(false);
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
#if MQTT_ENABLED
        s_ethernet_connected.store(false);
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

    if (!ETH.enableIPv6())
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Unable to enable Ethernet IPv6 support");
#endif
    }

    if (!ETH.begin(ETH_PHY_IP101,
                   ETH_PHY_ADDRESS,
                   static_cast<int>(ETH_MDC_GPIO),
                   static_cast<int>(ETH_MDIO_GPIO),
                   static_cast<int>(ETH_PHY_RST_GPIO),
                   EMAC_CLK_EXT_IN))
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "ETH.begin() failed");
#endif
        return;
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

    Serial.begin(serialMonitorSpeed);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "******************* Application Startup *******************");
    ESP_LOGI(TAG, "ESP32 Time Server v2.5.2");

#if !DEBUG_ENABLED
    ESP_LOGW(TAG, "DEBUG_ENABLED is turned off in the settings");
    ESP_LOGW(TAG, "No further application generate console displays will be generated");
    return;
#endif

#if UPTIME_RESTART_BUTTON_ENABLED
    ESP_LOGI(TAG, "Up time / Reset button support is enabled in the settings");
#else
    ESP_LOGW(TAG, "Up time / Reset button support is disabled in the settings");
#endif

#if LIQUID_CRYSTAL_DISPLAY_ENABLED
    ESP_LOGI(TAG, "LCD support is enabled in the settings");
#else
    ESP_LOGW(TAG, "LCD support is disabled in the settings");
#endif

#if OTE_UPDATES_ENABLED
    ESP_LOGI(TAG, "Over the Ethernet update support is enabled in the settings");
#else
    ESP_LOGW(TAG, "Over the Ethernet update support is disabled in the settings");
#endif
}

void setup_NVM_storage(void)
{

    if (!initialize_nvs_storage())
        ESP_LOGE(TAG, "NVS initialization failed. GPS module settings persistence is unavailable.");
}

void create_mutexes_and_semaphores(void)
{

#if LIQUID_CRYSTAL_DISPLAY_ENABLED
    s_lcd_mutex = xSemaphoreCreateMutex();
#endif
    s_time_mutex = xSemaphoreCreateMutex();
    s_pps_semaphore = xSemaphoreCreateBinary();
    s_pps_timestamp_queue = xQueueCreate(1, sizeof(int64_t));
    s_ota_mutex = xSemaphoreCreateMutex();
    s_sync_state_mutex = xSemaphoreCreateMutex();
}

void initialize_the_display(void)
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

static void setup_up_time_button()
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

static void configure_mac_address()
{

    uint8_t real_mac_address[6];
    char real_mac_address_str[18];

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

    if ((MACAddress[0] == '\0') || (std::strcmp(real_mac_address_str, MACAddress) == 0))
    {
#if DEBUG_ENABLED
        ESP_LOGI(TAG, "No need to change the MAC address.");
#endif
        return;
    }

    uint8_t mac[6] = {};
    if (!parse_mac_id_string(MACAddress, mac))
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "The MAC address is settings is invalid: %s - the MAC ID will not be changed", MACAddress);
#endif
        return;
    }

    esp_err_t err = esp_base_mac_addr_set(mac);
    if (err != ESP_OK)
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Failed to set MAC address to %s: %s", MACAddress, esp_err_to_name(err));
#endif
        return;
    }

    ESP_LOGI(TAG, "MAC address changed to : %s", MACAddress);
}

void setup_ethernet_connection()
{

    configure_mac_address();

    display_line(1, "Connecting Ethernet");
    display_line(2, "");
    setup_ethernet();
    display_selected_ip_address(static_cast<int>(time(nullptr) % 10));
}

#if OTE_UPDATES_ENABLED

static void ota_copy_reason(char *destination, size_t destination_size, const char *reason)
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

static void ota_set_running_state(unsigned int progress, unsigned int total)
{
    if (xSemaphoreTake(s_ota_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_ota_in_progress = true;
        s_ota_failed = false;
        s_ota_success = false;
        s_ota_failure_display_until_us = 0;
        s_ota_reboot_at_us = 0;
        s_ota_error_reason[0] = '\0';
        s_ota_progress_percent = total == 0 ? 0 : static_cast<int>((progress * 100U) / total);
        xSemaphoreGive(s_ota_mutex);
    }
}

static void ota_set_failure_state(const char *reason)
{
    if (xSemaphoreTake(s_ota_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_ota_in_progress = false;
        s_ota_failed = true;
        s_ota_success = false;
        s_ota_failure_display_until_us = esp_timer_get_time() + static_cast<int64_t>(OTA_Failure_Display_Time_Ms) * 1000LL;
        s_ota_reboot_at_us = 0;
        ota_copy_reason(s_ota_error_reason, sizeof(s_ota_error_reason), reason);
        xSemaphoreGive(s_ota_mutex);
    }
}

static void ota_set_success_state()
{
    if (xSemaphoreTake(s_ota_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_ota_in_progress = false;
        s_ota_failed = false;
        s_ota_success = true;
        s_ota_progress_percent = 100;
        s_ota_failure_display_until_us = 0;
        s_ota_reboot_at_us = esp_timer_get_time() + static_cast<int64_t>(OTA_Reboot_Delay_Ms) * 1000LL;
        s_ota_error_reason[0] = '\0';
        xSemaphoreGive(s_ota_mutex);
    }
}

static void format_ota_error_reason(ota_error_t error, char *buffer, size_t buffer_size)
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

static bool render_ota_display()
{
    bool ota_in_progress = false;
    bool ota_failed = false;
    bool ota_success = false;
    int ota_progress_percent = -1;
    int64_t ota_failure_display_until_us = 0;
    int64_t ota_reboot_at_us = 0;
    char ota_error_reason[lcdColumns + 1] = "";

    if (xSemaphoreTake(s_ota_mutex, portMAX_DELAY) != pdTRUE)
        return false;

    ota_in_progress = s_ota_in_progress;
    ota_failed = s_ota_failed;
    ota_success = s_ota_success;
    ota_progress_percent = s_ota_progress_percent;
    ota_failure_display_until_us = s_ota_failure_display_until_us;
    ota_reboot_at_us = s_ota_reboot_at_us;
    ota_copy_reason(ota_error_reason, sizeof(ota_error_reason), s_ota_error_reason);
    xSemaphoreGive(s_ota_mutex);

    int64_t now_us = esp_timer_get_time();
    if (!ota_in_progress && !ota_failed && !ota_success)
        return false;

    if (ota_failed && ota_failure_display_until_us > 0 && now_us >= ota_failure_display_until_us)
    {
        if (xSemaphoreTake(s_ota_mutex, portMAX_DELAY) == pdTRUE)
        {
            s_ota_failed = false;
            s_ota_progress_percent = -1;
            s_ota_error_reason[0] = '\0';
            s_ota_failure_display_until_us = 0;
            xSemaphoreGive(s_ota_mutex);
        }
        return false;
    }

    if (ota_in_progress)
    {
        char progress_line[lcdColumns + 1];
        snprintf(progress_line, sizeof(progress_line), "%d%% complete", ota_progress_percent < 0 ? 0 : ota_progress_percent);
        display_line(1, "OTA update started");
        display_line(2, progress_line);
        display_line(3, "Uploading firmware");
        return true;
    }

    if (ota_success)
    {
        (void)ota_reboot_at_us;
        display_line(1, "OTA successful");
        display_line(2, "100% complete");
        display_line(3, "Rebooting in 5 sec");
        return true;
    }

    display_line(1, "OTA failed");
    display_line(2, ota_error_reason[0] == '\0' ? "Unknown reason" : ota_error_reason);
    display_line(3, "Resuming in 10 sec");
    return true;
}

static void ota_service_task(void *parameter)
{
#if DEBUG_ENABLED
    ESP_LOGI(TAG, "OTA service task started on port %u", static_cast<unsigned int>(OTA_Port));
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
                         "OTA heartbeat: online=%s, ip=%s, in_progress=%s",
                         Network.isOnline() ? "true" : "false",
                         s_ip_address[0] == '\0' ? "<none>" : s_ip_address,
                         s_ota_in_progress ? "true" : "false");

            last_heartbeat_ms = now_ms;
        }
        */

        bool should_reboot = false;
        TickType_t loop_delay_ticks = pdMS_TO_TICKS(50);
        if (xSemaphoreTake(s_ota_mutex, portMAX_DELAY) == pdTRUE)
        {
            bool ota_idle = !s_ota_in_progress && !s_ota_failed && !s_ota_success;
            if (ota_idle)
                loop_delay_ticks = pdMS_TO_TICKS(200);

            if (s_ota_success && s_ota_reboot_at_us > 0 && esp_timer_get_time() >= s_ota_reboot_at_us)
                should_reboot = true;
            xSemaphoreGive(s_ota_mutex);
        }

        vTaskDelay(loop_delay_ticks);
        if (should_reboot)
        {
#if MQTT_ENABLED
            mqtt_publish_final_report();
#endif
            esp_restart();
        }

        /*

        // The following code is used to determine the ideal stack size for this method (ota_service_task)
        // it only needs to be setup and run once
        //
        // NOTE 1: Specific to ota_service_task to get a valid result the code below needed to be running
        //         with the Stack report usage display being examined when an OTA update was underway
        //
        // NOTE 2: The following is required to have the code directly below compile (build):
        //         From the ESP-IDF Terminal enter the command
        //            idf.py menuconfig
        //            (the menu screen may take 30 seconds or more to load)
        //         Navigate to:
        //            Component config → FreeRTOS → Kernel
        //         use the down arrow key to scroll down to
        //            configUSE_TRACE_FACILITY
        //            enable FreeTOS trace facility by pressing the space bar (puts a star in the option [*] to signify it is enabled)
        //         press 'S' (enter) to save the change
        //         press the ESC key
        //         press 'Q' to quit
        //         code should now build, flash and run ok
        //
        //         once the ideal stack size is known, the code can below can be commented out and the configUSE_TRACE_FACILITY disabled to save RAM
        //
        // NOTE 3: if in the future this program changes significantly the suggested stack size value generated in this testing may need to be redone
        //
        // Below is the code that has now already been used to calculate the ideal stack size:
        //

        // *** Hard‑coded stack size originally used when creating the task (bytes) ***
        const size_t allocated_bytes = 16384; // <-- set this to the value passed to xTaskCreatePinnedToCore

        // high watermark is returned in words
        UBaseType_t high_watermark_words = uxTaskGetStackHighWaterMark(NULL);
        size_t high_watermark_bytes = (size_t)high_watermark_words * sizeof(StackType_t);

        // compute peak usage and suggested size (25% margin)
        size_t peak_usage_bytes = (allocated_bytes > high_watermark_bytes) ? (allocated_bytes - high_watermark_bytes) : 0;
        size_t suggested_bytes = (size_t)((double)peak_usage_bytes * 1.25);

        ESP_LOGI("ota_service_task",
                 "Stack report: Allocated=%u bytes, HighWater=%u bytes unused, PeakUsage=%u bytes, Suggested=%u bytes",
                 (unsigned)allocated_bytes,
                 (unsigned)high_watermark_bytes,
                 (unsigned)peak_usage_bytes,
                 (unsigned)suggested_bytes);

        // Results of this testing (when an OTA update was underway):
        //
        // ota_service_task: Stack report: Allocated=16384 bytes, HighWater=13536 bytes unused, PeakUsage=2848 bytes, Suggested=3560 bytes

        */
    }
}

static void setup_ota()
{
    if (xSemaphoreTake(s_ota_mutex, portMAX_DELAY) == pdTRUE)
    {
        s_ota_in_progress = false;
        s_ota_failed = false;
        s_ota_success = false;
        s_ota_progress_percent = -1;
        s_ota_failure_display_until_us = 0;
        s_ota_reboot_at_us = 0;
        s_ota_error_reason[0] = '\0';
        xSemaphoreGive(s_ota_mutex);
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
             static_cast<unsigned int>(OTA_Port),
             static_cast<unsigned int>(strlen(OTAPassword)));
#endif

    ArduinoOTA.setPort(OTA_Port);
    ArduinoOTA.setHostname(DeviceName);
    ArduinoOTA.setPassword(OTAPassword);
    ArduinoOTA.setRebootOnSuccess(false);
    ArduinoOTA.onStart([]()
                       {
                           ota_set_running_state(0, 1);
#if DEBUG_ENABLED
                           ESP_LOGI(TAG, "OTA update started");
#endif
                       });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { ota_set_running_state(progress, total); });
    ArduinoOTA.onEnd([]()
                     {
                         ota_set_success_state();
#if DEBUG_ENABLED
                         ESP_LOGI(TAG, "OTA update completed successfully");
#endif
                     });
    ArduinoOTA.onError([](ota_error_t error)
                       {
                           char reason[lcdColumns + 1];
                           format_ota_error_reason(error, reason, sizeof(reason));
                           ota_set_failure_state(reason);
#if DEBUG_ENABLED
                           ESP_LOGE(TAG, "OTA update failed: %s", reason);
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
             static_cast<unsigned int>(OTA_Port));
#endif
}

#endif

void setup_for_ota_updates()
{
#if OTE_UPDATES_ENABLED

    display_line(1, "Setup OTA");
    display_line(2, "");
    setup_ota();

    // note: this task is intentionally pinned to core 0 (as opposed to tskNO_AFFINITY)
    xTaskCreatePinnedToCore(ota_service_task, "ota_service", 3560, nullptr, 5, nullptr, 0);
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
            candidate->failure = sync_fault_t::gps_invalid;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid checkpoint 1.");
#endif
            return false;
        }

        candidate->candidate_time = epoch_from_utc(nmea_time.year, nmea_time.month, nmea_time.day, nmea_time.hour, nmea_time.minute, nmea_time.second);

        clear_pps_events();
        if (xSemaphoreTake(s_pps_semaphore, pdMS_TO_TICKS(1500)) == pdTRUE)
        {
            candidate->use_pps_alignment = true;
            candidate->pps_release_time_us = esp_timer_get_time();
            candidate->candidate_time += 1;
        }
        else
        {
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "NMEA fallback is running without PPS.");
#endif
            if (!allowFallbackProcessingWithoutPPS)
            {
                candidate->failure = sync_fault_t::pps_missing;
                return false;
            }

            candidate->pps_release_time_us = esp_timer_get_time();
        }
    }
    else
    {
        clear_pps_events();
        if (xSemaphoreTake(s_pps_semaphore, pdMS_TO_TICKS(1500)) != pdTRUE)
        {
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "UBX mode is running without PPS.");
#endif

            candidate->failure = sync_fault_t::pps_missing;
            if (allowFallbackProcessingWithoutPPS)
            {
#if DEBUG_ENABLED
                ESP_LOGW(TAG, "Switching to NMEA fallback because PPS is unavailable.");
#endif
                s_use_nmea_fallback = true;
            }
            return false;
        }

        if (!s_gps.getPVT())
        {
            candidate->failure = sync_fault_t::gps_invalid;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - Position - Velocity - Time");
#endif
            return false;
        }

        uint8_t fix_type = s_gps.getFixType();
        if ((fix_type != 3 && fix_type != 4 && fix_type != 5) || !s_gps.getGnssFixOk() || !s_gps.getDateValid() || !s_gps.getTimeValid())
        {
            candidate->failure = sync_fault_t::gps_invalid;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - Fix type.");
#endif
            return false;
        }

        int year = s_gps.getYear();
        int month = s_gps.getMonth();
        int day = s_gps.getDay();
        int hour = s_gps.getHour();
        int minute = s_gps.getMinute();
        int second = s_gps.getSecond();

        if (year <= 2025 || month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60)
        {
            candidate->failure = sync_fault_t::gps_invalid;
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "GNSS invalid - bad date or time.");
#endif
            return false;
        }

        candidate->candidate_time = epoch_from_utc(year, month, day, hour, minute, second) + 1;

        vTaskDelay(pdMS_TO_TICKS(200));

        clear_pps_events();
        if (xSemaphoreTake(s_pps_semaphore, pdMS_TO_TICKS(1500)) != pdTRUE)
        {
#if DEBUG_ENABLED
            ESP_LOGE(TAG, "UBX mode lost PPS alignment pulse.");
#endif

            candidate->failure = sync_fault_t::pps_missing;
            if (allowFallbackProcessingWithoutPPS)
            {
#if DEBUG_ENABLED
                ESP_LOGW(TAG, "Switching to NMEA fallback because PPS is unavailable.");
#endif
                s_use_nmea_fallback = true;
            }
            return false;
        }

        candidate->pps_release_time_us = esp_timer_get_time();
        candidate->use_pps_alignment = true;
    }

    if ((esp_timer_get_time() - attempt_start_us) > Max_Sync_Attempt_Us)
    {
        candidate->failure = sync_fault_t::gps_invalid;
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "GNSS invalid checkpoint 2.");
#endif
        return false;
    }

    candidate->used_nmea_fallback = s_use_nmea_fallback;
    return true;
}

static void handle_runtime_sync_failure(sync_fault_t fault, time_t update_delta, uint32_t retry_delay_ms)
{
    uint32_t failure_count = sync_state_note_failure(fault, update_delta);
    sync_state_t snapshot = get_sync_state_snapshot();

    s_time_setting_in_progress.store(false);

    if (failure_count >= Sync_Failures_Before_Runtime_Recovery && (failure_count % Sync_Failures_Before_Runtime_Recovery) == 0)
    {
#if DEBUG_ENABLED
        ESP_LOGW(TAG, "Runtime GPS recovery attempt after %lu consecutive sync failures.", static_cast<unsigned long>(failure_count));
#endif
        setup_gps();
        sync_state_reset_failure_counters();
    }

    if (snapshot.last_successful_sync_us > 0 && (esp_timer_get_time() - snapshot.last_successful_sync_us) > Sync_Reboot_After_Us)
    {
#if DEBUG_ENABLED
        ESP_LOGE(TAG, "Rebooting after extended holdover without a successful GPS resync.");
#endif
        vTaskDelay(pdMS_TO_TICKS(200));
#if MQTT_ENABLED
        mqtt_publish_final_report();
#endif
        esp_restart();
    }

    vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
}

static void gps_time_sync_task(void *parameter)
{
    bool first_sync = true;

    for (;;)
    {

        s_time_setting_in_progress.store(true);
        sync_state_note_attempt();

        sync_candidate_t candidate{};
        if (!acquire_sync_candidate(&candidate))
        {
            handle_runtime_sync_failure(candidate.failure, 0, 1000);
            continue;
        }

        if (first_sync)
        {
            sync_candidate_t confirmation_candidate{};
            if (!acquire_sync_candidate(&confirmation_candidate))
            {
                handle_runtime_sync_failure(confirmation_candidate.failure, 0, 1000);
                continue;
            }

            if (!first_sync_candidates_are_plausible(candidate, confirmation_candidate))
            {
                handle_runtime_sync_failure(sync_fault_t::gps_invalid, 0, 1000);
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
                sync_state_note_failure(sync_fault_t::sanity_mismatch, update_delta);

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
            s_ntp_reference_time_64 = get_current_time_in_ntp64_format();
            s_ntp_reference_valid = true;

            xSemaphoreGive(s_time_mutex);

            s_safe_guard_tripped.store(false);
            s_time_setting_in_progress.store(false);
            s_time_has_been_set.store(true);
            first_sync = false;
            sync_state_note_success(update_delta);

            if (s_attempt_no_signal_recovery)
            {
                bool save_recovery_setting_ok = save_attempt_no_signal_recovery_setting(false);
                s_attempt_no_signal_recovery = false;
#if DEBUG_ENABLED
                ESP_LOGI(TAG, "GPS time set succeeded; no-signal recovery is now persisted as false (save=%s)", save_recovery_setting_ok ? "ok" : "failed");
#endif
            }

#if DEBUG_ENABLED
            char date_string[16] = "";
            char time_string[24] = "";
            time_t now_utc = time(nullptr);
            format_local_date_time(now_utc, date_string, sizeof(date_string), time_string, sizeof(time_string));

            if (candidate.used_nmea_fallback)
                ESP_LOGI(TAG, "GPS time sync ( using NMEA fallback %s ) on %s at %s", candidate.use_pps_alignment ? "with PPS alignment" : "without PPS alignment", date_string, time_string);
            else
                ESP_LOGI(TAG, "GPS time sync on %s at %s", date_string, time_string);
#endif

            uint32_t refresh_start_ms = millis();
            uint32_t refresh_interval_ms = periodicGPSRefreshEveryThisNumberOfMinutes * 60UL * 1000UL;
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
                    // provide for a two second tolerance to avoid false positive reporting on the loss of a gnss lock
                    if (invalid_started_us == 0)
                        invalid_started_us = esp_timer_get_time();

                    if ((esp_timer_get_time() - invalid_started_us) >= 2000000)
                    {
                        sync_state_note_gnss_validity(false);

#if DEBUG_ENABLED
                        ESP_LOGW(TAG,
                                 "GNSS timing invalid for more than 2 seconds; starting immediate reacquisition.");
#endif

                        break;
                    }
                }

                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
}

void setup_the_gps()
{

    display_line(1, "GPS setup underway");
    display_line(2, "");

    apply_timezone_settings();

    setup_gps();

    display_line(1, "Getting date & time");
    display_line(2, "");

    xTaskCreatePinnedToCore(gps_time_sync_task, "gps_time_sync", 16384, nullptr, 15, nullptr, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(pps_discipline_task, "pps_discipline", 19805, nullptr, 14, nullptr, tskNO_AFFINITY);

    while (!s_time_has_been_set.load())
        vTaskDelay(pdMS_TO_TICKS(100));
}

static void ntp_server_task(void *parameter)
{
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
            continue;
        }
        if (ready == 0)
            continue;

        static uint8_t next_socket = 0;
        const int sockets[] = {ipv4_socket, ipv6_socket, ipv6_link_local_socket};
        int sock = -1;
        for (size_t offset = 0; offset < sizeof(sockets) / sizeof(sockets[0]); ++offset)
        {
            size_t index = (next_socket + offset) % (sizeof(sockets) / sizeof(sockets[0]));
            if (sockets[index] >= 0 && FD_ISSET(sockets[index], &read_fds))
            {
                sock = sockets[index];
                next_socket = static_cast<uint8_t>((index + 1) % (sizeof(sockets) / sizeof(sockets[0])));
                break;
            }
        }
        if (sock < 0)
            continue;

        uint8_t request[NTP_PACKET_SIZE];
        uint8_t reply[NTP_PACKET_SIZE];
        struct sockaddr_storage source_addr{};
        socklen_t source_addr_len = sizeof(source_addr);

        int len = recvfrom(sock, request, sizeof(request), 0, reinterpret_cast<struct sockaddr *>(&source_addr), &source_addr_len);
        if (len < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
#if DEBUG_ENABLED
            ESP_LOGW(TAG, "recvfrom failed: errno %d", errno);
#endif
            continue;
        }

        if (len != static_cast<int>(NTP_PACKET_SIZE))
        {
#if MQTT_ENABLED
            s_ntp_invalid_requests.fetch_add(1);
#endif
            continue;
        }

        uint8_t ntp_version = (request[0] >> 3) & 0x07;
        uint8_t ntp_mode = request[0] & 0x07;
        if (ntp_version < 3 || ntp_version > 4 || ntp_mode != 3)
        {
#if MQTT_ENABLED
            s_ntp_invalid_requests.fetch_add(1);
#endif
            continue;
        }

#if MQTT_ENABLED
        s_ntp_valid_requests.fetch_add(1);
        mqtt_note_ntp_request(source_addr);
#endif
        ntp_reply_status_t status = get_ntp_reply_status();
#if MQTT_ENABLED
        if (status.stratum == 1)
            s_ntp_requests_with_gnss_lock.fetch_add(1);
        else
            s_ntp_requests_without_gnss_lock.fetch_add(1);
#endif
        bool reply_ready = false;
        if (xSemaphoreTake(s_time_mutex, portMAX_DELAY) == pdTRUE)
        {
            uint64_t receive_time = get_current_time_in_ntp64_format();
            uint64_t transmit_time = get_current_time_in_ntp64_format();
            build_ntp_reply(request, reply, receive_time, transmit_time, status);
            xSemaphoreGive(s_time_mutex);
            reply_ready = true;
        }

        if (reply_ready)
        {
            int sent = sendto(sock, reply, sizeof(reply), 0, reinterpret_cast<struct sockaddr *>(&source_addr), source_addr_len);
#if MQTT_ENABLED
            if (sent == static_cast<int>(sizeof(reply)))
                s_ntp_responses.fetch_add(1);
#else
            (void)sent;
#endif
#if DEBUG_ENABLED
            char source_address[IP_ADDRESS_TEXT_SIZE] = "";
            if (format_socket_address(source_addr, source_address, sizeof(source_address)))
                ESP_LOGI(TAG, "NTP -> %s", source_address);
#endif
        }
        /*

          // The following code is used to determine the ideal stack size for this method (ntp_server_task)
          // it only needs to be setup and run once
          //
          // NOTE 1: specific to ntp_server_task, the Stack report below will only be issued if an NTP request is received.
          //         accordingly, to generate a NTP request for testing the following can be entered via the Windows powershell command line:
          //              w32tm /stripchart /computer:192.168.1.24 /samples:5 /dataonly
          //
          // NOTE 2: The following is required to have the code directly below compile (build):
          //         From the ESP-IDF Terminal enter the command
          //            idf.py menuconfig
          //            (the menu screen may take 30 seconds or more to load)
          //         Navigate to:
          //            Component config → FreeRTOS → Kernel
          //         use the down arrow key to scroll down to
          //            configUSE_TRACE_FACILITY
          //            enable FreeTOS trace facility by pressing the space bar (puts a star in the option [*] to signify it is enabled)
          //         press 'S' (enter) to save the change
          //         press the ESC key
          //         press 'Q' to quit
          //         code should now build, flash and run ok
          //
          //         once the ideal stack size is known, the code can below can be commented out and the configUSE_TRACE_FACILITY disabled to save RAM
          //
          // NOTE 3: if in the future this program changes significantly the suggested stack size value generated in this testing may need to be redone
          //
          // Below is the code that has now already been used to calculate the ideal stack size:
          //

          // *** Hard‑coded stack size originally used when creating the task (bytes) ***
          const size_t allocated_bytes = 16384; // <-- set this to the value passed to xTaskCreatePinnedToCore

          // high watermark is returned in words
          UBaseType_t high_watermark_words = uxTaskGetStackHighWaterMark(NULL);
          size_t high_watermark_bytes = (size_t)high_watermark_words * sizeof(StackType_t);

          // compute peak usage and suggested size (25% margin)
          size_t peak_usage_bytes = (allocated_bytes > high_watermark_bytes) ? (allocated_bytes - high_watermark_bytes) : 0;
          size_t suggested_bytes = (size_t)((double)peak_usage_bytes * 1.25);

          ESP_LOGI("ntp_server_task",
                   "Stack report: Allocated=%u bytes, HighWater=%u bytes unused, PeakUsage=%u bytes, Suggested=%u bytes",
                   (unsigned)allocated_bytes,
                   (unsigned)high_watermark_bytes,
                   (unsigned)peak_usage_bytes,
                   (unsigned)suggested_bytes);

          vTaskDelay(pdMS_TO_TICKS(50));

          // Results of this testing:
          //
          //  ntp_server_task: Stack report: Allocated=19920 bytes, HighWater=17772 bytes unused, PeakUsage=2148 bytes, Suggested=2685 bytes
  */
    }
}

static void update_display_task(void *parameter)

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
        if (render_ota_display())
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

            sync_state_t sync_snapshot = get_sync_state_snapshot();
            bool sync_fault_active = s_safe_guard_tripped.load() || sync_snapshot.holdover_mode;

            int required_top_line_message = 0;

            // Determine message code for the first line:
            // 0 none;                                               "ESP32 Time Server   "
            // 1 Time sync underway (normal periodic behaviour);     "ESP32 Time Server * "
            // 2 MQTT setup failed;                                  "ESP32 Time Server[1]"
            // 3 MQTT there are queued items;                        "ESP32 Time Server[2]"
            // 4 Sanity check mismatch;                              "ESP32 Time Server[3]"
            // 5 PPS missing;                                        "ESP32 Time Server[4]"
            // 6 GNSS missing or invalid;                            "ESP32 Time Server[5]"
            // 7 GNSS snyc stale;                                    "ESP32 Time Server[6]"
            // 8 GNSS unlocked;                                      "ESP32 Time Server[7]"
            // 10  used for when the button is pressed;              "ESP32 Time Server's "

            if (s_time_setting_in_progress.load())
            {
                required_top_line_message = 1;
            }
            else
            {
#if MQTT_ENABLED
                if (s_mqtt_setup_failed.load())
                    required_top_line_message = 2;
                else if (s_mqtt_queued_messages_count.load() > 0)
                    required_top_line_message = 3;
#endif
                switch (sync_snapshot.fault)
                {
                case sync_fault_t::sanity_mismatch:
                    required_top_line_message = 4;
                    break;

                case sync_fault_t::pps_missing:
                    required_top_line_message = 5;
                    break;

                case sync_fault_t::gps_invalid:
                    required_top_line_message = 6;
                    break;

                case sync_fault_t::sync_stale:
                    required_top_line_message = 7;
                    break;

                default:
                    required_top_line_message = s_gnss_locked.load() ? 0 : 8;
                    break;
                }
            }

#if UPTIME_RESTART_BUTTON_ENABLED
            if (display_uptime_seconds_counter > 0)
                required_top_line_message = 10;
#endif

            // Update top line only if it has changed
            if (required_top_line_message != previous_top_line_message)
            {
                char top_line_message[21]; // 20 chars + null
                memset(top_line_message, ' ', sizeof(top_line_message));
                memcpy(top_line_message, "ESP32 Time Server", 17);

                if (required_top_line_message == 1)
                {
                    top_line_message[18] = '*';
                }
                else if (required_top_line_message > 1 && required_top_line_message < 10)
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

                display_line(1, "up time is");
                display_line(2, centered);
                display_line(3, " days hrs:mins:secs");
                display_uptime_seconds_counter--;
            }
            else
#endif
            {
                char date_line[16];
                char time_line[24];
                format_local_date_time(now_utc, date_line, sizeof(date_line), time_line, sizeof(time_line));
                display_line(1, date_line);
                display_line(2, time_line);
                display_selected_ip_address(utc_tm.tm_sec);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));

        /*

          // The following code is used to determine the ideal stack size for this method (update_display_task)
          // it only needs to be setup and run once
          //
          // NOTE 1: The following is required to have the code directly below compile (build):
          //         From the ESP-IDF Terminal enter the command
          //            idf.py menuconfig
          //            (the menu screen may take 30 seconds or more to load)
          //         Navigate to:
          //            Component config → FreeRTOS → Kernel
          //         use the down arrow key to scroll down to
          //            configUSE_TRACE_FACILITY
          //            enable FreeTOS trace facility by pressing the space bar (puts a star in the option [*] to signify it is enabled)
          //         press 'S' (enter) to save the change
          //         press the ESC key
          //         press 'Q' to quit
          //         code should now build, flash and run ok
          //
          //         once the ideal stack size is known, the code can below can be commented out and the configUSE_TRACE_FACILITY disabled to save RAM
          //
          // NOTE 2: if in the future this program changes significantly the suggested stack size value generated in this testing may need to be redone
          //
          // Below is the code that has now already been used to calculate the ideal stack size:

        // *** Hard‑coded stack size originally used when creating the task (bytes) ***
        const size_t allocated_bytes = 16384; // <-- set this to the value passed to xTaskCreatePinnedToCore

        // high watermark is returned in words
        UBaseType_t high_watermark_words = uxTaskGetStackHighWaterMark(NULL);
        size_t high_watermark_bytes = (size_t)high_watermark_words * sizeof(StackType_t);

        // compute peak usage and suggested size (25% margin)
        size_t peak_usage_bytes = (allocated_bytes > high_watermark_bytes) ? (allocated_bytes - high_watermark_bytes) : 0;
        size_t suggested_bytes = (size_t)((double)peak_usage_bytes * 1.25);

        ESP_LOGI("update_display_task",
                 "Stack report: Allocated=%u bytes, HighWater=%u bytes unused, PeakUsage=%u bytes, Suggested=%u bytes",
                 (unsigned)allocated_bytes,
                 (unsigned)high_watermark_bytes,
                 (unsigned)peak_usage_bytes,
                 (unsigned)suggested_bytes);

        // Results of this testing:
        //
        //   update_display_task: Stack report: Allocated=16384 bytes, HighWater=14416 bytes unused, PeakUsage=1968 bytes, Suggested=2460 byte
        */
    }
#else
    for (;;)
    {
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

    setup_NVM_storage();

    create_mutexes_and_semaphores();

    initialize_the_display();

    setup_up_time_button();

    setup_ethernet_connection();

    setup_for_ota_updates();

    setup_the_gps();

    setup_mqtt();

    xTaskCreatePinnedToCore(ntp_server_task, "ntp_server", 2685, nullptr, 20, nullptr, tskNO_AFFINITY);

    xTaskCreatePinnedToCore(update_display_task, "display_service", 2460, nullptr, 10, nullptr, tskNO_AFFINITY);
}

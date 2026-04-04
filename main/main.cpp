// ESP32 Time Server v2
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
// LCD2004:             blue/green screen with HD44780 I2C serial interface adapter https://www.aliexpress.com/item/1005006829045609.html?spm=a2g0o.order_list.order_list_main.11.29521802ixry0y
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
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hd44780.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "pcf8574.h"
}

static constexpr gpio_num_t ETH_MDC_GPIO = GPIO_NUM_31;
static constexpr gpio_num_t ETH_MDIO_GPIO = GPIO_NUM_52;
static constexpr gpio_num_t ETH_PHY_RST_GPIO = GPIO_NUM_51;
static constexpr int ETH_PHY_ADDRESS = 1;

static constexpr i2c_port_t LCD_I2C_PORT = I2C_NUM_0;
static constexpr gpio_num_t LCD_I2C_SDA_GPIO = GPIO_NUM_8;
static constexpr gpio_num_t LCD_I2C_SCL_GPIO = GPIO_NUM_7;

static constexpr uint16_t NTP_PORT = 123;
static constexpr size_t NTP_PACKET_SIZE = 48;
static constexpr uint64_t NTP_EPOCH_OFFSET = 2208988800ULL;
static constexpr EventBits_t ETH_CONNECTED_BIT = BIT0;
static constexpr EventBits_t ETH_GOT_IP_BIT = BIT1;
static constexpr uint32_t OTA_Failure_Display_Time_Ms = 10000;
static constexpr uint32_t OTA_Reboot_Delay_Ms = 5000;

static const char *TAG = "main_cpp";

static i2c_dev_t s_lcd_io{};
static uint8_t s_lcd_addr = 0;
static bool s_lcd_ready = false;
static hd44780_t s_lcd{};
static char s_lcd_last_lines[lcdRows][lcdColumns + 1] = {};
static bool s_lcd_line_cached[lcdRows] = {};

static EventGroupHandle_t s_net_event_group = nullptr;
static SemaphoreHandle_t s_time_mutex = nullptr;
static SemaphoreHandle_t s_pps_semaphore = nullptr;
static SemaphoreHandle_t s_pps_discipline_semaphore = nullptr;
static SemaphoreHandle_t s_ota_mutex = nullptr;
static SemaphoreHandle_t s_lcd_mutex = nullptr;

static bool s_ota_in_progress = false;
static bool s_ota_failed = false;
static bool s_ota_success = false;
static int s_ota_progress_percent = -1;
static int64_t s_ota_failure_display_until_us = 0;
static int64_t s_ota_reboot_at_us = 0;
static char s_ota_error_reason[lcdColumns + 1] = "";

static volatile bool s_safe_guard_tripped = false;
static volatile bool s_time_setting_in_progress = false;
static volatile bool s_time_has_been_set = false;
static uint64_t s_ntp_reference_time_64 = 0;
static bool s_ntp_reference_valid = false;

static char s_ip_address[16] = "";

static HardwareSerial s_gps_serial(1);
static SFE_UBLOX_GNSS_SERIAL s_gps;
static uint32_t s_detected_gps_baud = 0;
static bool s_gps_required_assume_success = false;

// The following flag for National Marine Electronics Association fallback doesn't determine if it is allowed or not
// rather the program sets it to true if it is required (due to an older / ubox uncompliant hardware gps module being used)

static bool s_use_nmea_fallback = false;
static volatile bool s_pps_discipline_active = false;

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

    if (!supportForLiquidCrystalDisplay)
        return;

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
        if (debugIsOn)
            ESP_LOGW(TAG, "LCD not ready for row %u", row);
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
        if (debugIsOn)
            ESP_LOGE(TAG, "hd44780_gotoxy failed: %s", esp_err_to_name(err));
        if (s_lcd_mutex != nullptr)
            xSemaphoreGive(s_lcd_mutex);
        return;
    }

    err = hd44780_puts(&s_lcd, padded);
    if (err != ESP_OK)
    {
        if (debugIsOn)
            ESP_LOGE(TAG, "hd44780_puts failed: %s", esp_err_to_name(err));
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
        if (debugIsOn)
            ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return false;
    }

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

static bool check_uptime_request()
{

    if (!supportForAnUpTimeRestartButton)
        return false;

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
        if (debugIsOn)
            ESP_LOGI(TAG, "Up time/reset button restart requested: cleared stored GPS NVS values.");
        esp_restart();
    }

    return true;
}

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

static void build_ntp_reply(const uint8_t *request, uint8_t *reply, uint64_t receive_time, uint64_t transmit_time)
{
    memset(reply, 0, NTP_PACKET_SIZE);

    reply[0] = 0b00011100;
    reply[1] = 0b00000001;
    reply[2] = 4;
    reply[3] = 0xF7;
    reply[11] = 0x50;
    reply[12] = 'G';
    reply[13] = 'P';
    reply[14] = 'S';

    uint64_t reference_time = s_ntp_reference_valid ? s_ntp_reference_time_64 : receive_time;
    write_ntp_timestamp(reply, 16, reference_time);

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

    if (debugIsOn)
        ESP_LOGI(TAG,
                 "GPS probe at %lu baud: %u bytes seen, sample: %s",
                 static_cast<unsigned long>(baud),
                 static_cast<unsigned int>(probe_result.bytes_seen),
                 probe_result.saw_data ? probe_result.sample : "<none>");

    s_gps_serial.end();
    s_gps_serial.begin(baud, SERIAL_8N1, RXPin, TXPin);
    vTaskDelay(pdMS_TO_TICKS(150));

    bool begin_result = s_gps.begin(s_gps_serial, gpsBeginMaxWaitMs, assume_success);
    if (debugIsOn)
        ESP_LOGI(TAG,
                 "GPS begin at %lu baud with assume_success=%s -> %s",
                 static_cast<unsigned long>(baud),
                 assume_success ? "true" : "false",
                 begin_result ? "success" : "failed");

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
        if (debugIsOn)
            ESP_LOGI(TAG, "GPS initialization attempt %d of %d", attempt + 1, max_attempts);

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
                    if (debugIsOn)
                        ESP_LOGI(TAG,
                                 "GPS responded at %lu baud. Attempting to switch to %lu baud.",
                                 static_cast<unsigned long>(candidate_baud),
                                 static_cast<unsigned long>(gps_baud));

                    bool baud_change_command_reported_success = s_gps.setSerialRate(gps_baud);
                    if (!baud_change_command_reported_success && debugIsOn)
                        ESP_LOGW(TAG, "GPS baud-rate change command to %lu was not acknowledged. Probing the target baud anyway.", static_cast<unsigned long>(gps_baud));

                    vTaskDelay(pdMS_TO_TICKS(200));

                    bool reconnect_without_assume = try_gps_begin(gps_baud, false, &saw_any_serial_data);
                    bool reconnect_with_assume = false;
                    if (!reconnect_without_assume)
                        reconnect_with_assume = try_gps_begin(gps_baud, true, &saw_any_serial_data);

                    if (reconnect_without_assume || reconnect_with_assume)
                    {
                        s_detected_gps_baud = gps_baud;
                        s_gps_required_assume_success = reconnect_with_assume;

                        if (!baud_change_command_reported_success && debugIsOn)
                            ESP_LOGW(TAG, "GPS baud-rate change command to %lu reported failure, but reconnect succeeded at the new baud.", static_cast<unsigned long>(gps_baud));
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
                                if (debugIsOn)
                                    ESP_LOGW(TAG, "GPS baud-rate change command to %lu was acknowledged, but immediate reconnect failed. Rebooting to complete transition.", static_cast<unsigned long>(gps_baud));
                                display_line(1, "GPS baud changed");
                                display_line(2, "Rebooting...");
                                vTaskDelay(pdMS_TO_TICKS(1000));
                                esp_restart();
                            }

                            if (debugIsOn)
                                ESP_LOGW(TAG, "GPS baud-rate change to %lu did not take effect immediately. Continuing at detected baud %lu.", static_cast<unsigned long>(gps_baud), static_cast<unsigned long>(candidate_baud));
                        }
                        else
                        {
                            if (debugIsOn)
                                ESP_LOGW(TAG, "GPS baud-rate change attempt left module unreachable at both %lu and %lu. Continuing with best-known state.", static_cast<unsigned long>(candidate_baud), static_cast<unsigned long>(gps_baud));
                        }
                    }
                }

                if (s_gps_required_assume_success)
                {
                    if (debugIsOn)
                        ESP_LOGW(TAG,
                                 "GPS communication was established only with assume_success=true at %lu baud. The attached module may have limited u-blox compatibility.",
                                 static_cast<unsigned long>(s_detected_gps_baud));
                }

                if (debugIsOn)
                    ESP_LOGI(TAG, "GPS initialization will continue at %lu baud.", static_cast<unsigned long>(s_detected_gps_baud));
                return true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    if (saw_any_serial_data)
    {
        if (debugIsOn)
            ESP_LOGW(TAG, "GPS serial data was detected, but SparkFun u-blox GNSS v3 could not initialize the module. The module may be an older NEO-6/7/M8 variant.");
    }
    else
    {
        if (debugIsOn)
            ESP_LOGW(TAG, "No GPS serial data was detected on GPIO%d/GPIO%d at any tested baud rate. Check power, TX/RX wiring, and signal levels.", RXPin, TXPin);
    }

    return false;
}

static bool configure_gps_outputs(bool *uart1_output_set_result)
{
    bool i2c_output_disabled = s_gps.setI2COutput(0);
    bool uart1_output_set = s_gps.setUART1Output(COM_TYPE_UBX | COM_TYPE_NMEA);
    bool uart2_output_disabled = s_gps.setUART2Output(0);

    if (uart1_output_set_result != nullptr)
        *uart1_output_set_result = uart1_output_set;

    if (debugIsOn)
        ESP_LOGI(TAG,
                 "GPS port config results: I2C off=%s, UART1 UBX=%s, UART2 off=%s",
                 i2c_output_disabled ? "ok" : "failed",
                 uart1_output_set ? "ok" : "failed",
                 uart2_output_disabled ? "ok" : "failed");

    if (debugIsOn && !uart2_output_disabled)
        ESP_LOGI(TAG, "UART2 output disable is not supported by this module/firmware. Continuing with UART1 configuration.");

    if (debugIsOn && !i2c_output_disabled)
        ESP_LOGI(TAG, "I2C output disable did not succeed. This is non-fatal for UART1 operation.");

    return uart1_output_set;
}

static bool perform_extended_no_signal_recovery()
{
    if (debugIsOn)
        ESP_LOGW(TAG, "Performing GPS extended no-signal recovery: factory-default + GNSS reset + reinitialization.");

    bool factory_default_applied = s_gps.factoryDefault();
    if (debugIsOn)
        ESP_LOGI(TAG, "GPS factoryDefault() -> %s", factory_default_applied ? "ok" : "failed");

    s_gps.hardReset();
    vTaskDelay(pdMS_TO_TICKS(1500));

    static constexpr int recoveryInitAttempts = 3;
    uint32_t recovery_target_baud = s_gps_target_baud > 0 ? s_gps_target_baud : get_highest_candidate_gps_baud();
    if (!set_gps_baud_rate(recovery_target_baud, recoveryInitAttempts))
    {
        if (debugIsOn)
            ESP_LOGW(TAG, "GPS recovery reinitialization failed after reset.");
        return false;
    }

    bool uart1_output_set = false;
    bool gps_outputs_configured = configure_gps_outputs(&uart1_output_set);
    s_use_nmea_fallback = s_gps_required_assume_success || !uart1_output_set;

    if (debugIsOn)
        ESP_LOGI(TAG, "GPS recovery post-config: outputs=%s, fallback=%s, baud=%lu",
                 gps_outputs_configured ? "ok" : "partial",
                 s_use_nmea_fallback ? "enabled" : "disabled",
                 static_cast<unsigned long>(s_detected_gps_baud));

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
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_pps_semaphore != nullptr)
        xSemaphoreGiveFromISR(s_pps_semaphore, &higher_priority_task_woken);
    if (s_pps_discipline_semaphore != nullptr)
        xSemaphoreGiveFromISR(s_pps_discipline_semaphore, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE)
        portYIELD_FROM_ISR();
}

static void setup_pps_input()
{
    gpio_config_t config{};
    config.pin_bit_mask = 1ULL << PPSPin;
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    config.intr_type = GPIO_INTR_POSEDGE;
    ESP_ERROR_CHECK(gpio_config(&config));

    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(gpio_isr_handler_add(static_cast<gpio_num_t>(PPSPin), pps_isr_handler, nullptr));
}

static void clear_pps_events()
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
        if (xSemaphoreTake(s_pps_discipline_semaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            last_pps_us = esp_timer_get_time();
            s_pps_discipline_active = true;

            if (!logged_active)
            {
                if (debugIsOn)
                    ESP_LOGI(TAG, "PPS discipline active.");
                logged_active = true;
            }

            if (!s_time_has_been_set || s_time_setting_in_progress)
                continue;

            if (xSemaphoreTake(s_time_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
                continue;

            struct timeval now{};
            gettimeofday(&now, nullptr);

            int64_t phase_error_us = static_cast<int64_t>(now.tv_usec);
            if (phase_error_us > 500000)
                phase_error_us -= 1000000;

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
                if (logged_active && debugIsOn)
                    ESP_LOGW(TAG, "PPS discipline inactive (PPS signal unavailable).");
                logged_active = false;
                s_pps_discipline_active = false;
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
        // pps_discipline_task: Stack report: Allocated=16384 bytes, HighWater=14556 bytes unused, PeakUsage=1828 bytes, Suggested=2385 bytes

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
        if (debugIsOn)
            ESP_LOGI(TAG, "GPS no-signal recovery default initialized to %s (save=%s)",
                     s_attempt_no_signal_recovery ? "true" : "false",
                     save_recovery_setting_ok ? "ok" : "failed");
    }

    uint32_t highest_candidate_baud = get_highest_candidate_gps_baud();
    uint32_t startup_target_baud = highest_candidate_baud;

    if (gps_nvs_load_ok && stored_gps_data.has_id_type && stored_gps_data.max_baud > 0)
        startup_target_baud = stored_gps_data.max_baud;

    bool known_module_fast_path = gps_nvs_load_ok && stored_gps_data.has_id_type && stored_gps_data.max_baud > 0;
    if (debugIsOn)
        ESP_LOGI(TAG,
                 "GPS startup path: %s (target baud=%lu)",
                 known_module_fast_path ? "known module fast path" : "first-time scan path",
                 static_cast<unsigned long>(startup_target_baud));

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

    if (debugIsOn)
        ESP_LOGI(TAG, "GPS startup mode: baud=%lu, initialization=%s",
                 static_cast<unsigned long>(s_detected_gps_baud),
                 s_gps_required_assume_success ? "assume_success" : "confirmed");

    if (s_detected_gps_baud > 0)
    {
        bool save_ok = save_gps_nvs_data(current_identity, s_detected_gps_baud);
        if (debugIsOn)
            ESP_LOGI(TAG,
                     "GPS identity persistence: type=%s, value=%s, max_baud=%lu, save=%s",
                     current_identity.type,
                     current_identity.value,
                     static_cast<unsigned long>(s_detected_gps_baud),
                     save_ok ? "ok" : "failed");
    }

    bool uart1_output_set = false;
    configure_gps_outputs(&uart1_output_set);
    if (!uart1_output_set)
    {
        if (debugIsOn)
            ESP_LOGW(TAG, "GPS UART1 output configuration failed. Continuing with the module's current output settings.");
    }

    s_use_nmea_fallback = s_gps_required_assume_success || !uart1_output_set;
    if (s_use_nmea_fallback && !allowFallbackProcessing)
    {
        ESP_LOGE(TAG, "NMEA fallback processing is required but not allowed - check the ESP32TimeServerSetting.h file.");
        halt_with_display("GPS fallback required", "Fallback not allowed", "Check settings");
    }

    if (s_use_nmea_fallback && debugIsOn)
        ESP_LOGW(TAG, "NMEA fallback mode enabled for GPS fix/time acquisition.");

    display_line(1, "Waiting for GPS fix");
    display_line(2, "");

    static constexpr uint32_t gpsFixEscalationToNmeaFallbackMs = 120000UL;
    static constexpr uint32_t gpsNoSignalRecoveryMs = 180000UL;

    bool no_signal_recovery_attempted = false;
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
            ubx_fix_ready = (fix_type > 0 && fix_type < 6) && gnss_fix_ok && date_valid && time_valid;

            char satellites_line[lcdColumns + 1];
            snprintf(satellites_line, sizeof(satellites_line), "Satellites: %u", static_cast<unsigned int>(satellites_used));
            display_line(2, satellites_line);
        }

        uint32_t now_ms = millis();
        if (!ubx_fix_ready && !s_use_nmea_fallback && allowFallbackProcessing && (now_ms - wait_start_ms) >= gpsFixEscalationToNmeaFallbackMs)
        {
            s_use_nmea_fallback = true;
            if (debugIsOn)
                ESP_LOGW(TAG, "No UBX fix after %lu ms. Enabling NMEA fallback for additional acquisition path.", static_cast<unsigned long>(now_ms - wait_start_ms));
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

        if (!ubx_fix_ready && s_use_nmea_fallback)
        {
            nmea_rmc_time_t nmea_time{};
            nmea_fix_ready = wait_for_nmea_rmc_time(&nmea_time, 1200UL);
        }

        if (ubx_fix_ready || nmea_fix_ready)
        {
            if (debugIsOn)
            {
                if (ubx_fix_ready)
                {
                    ESP_LOGI(TAG, "GPS fix obtained after %lu ms: fix_type=%u (%s)",
                             static_cast<unsigned long>(millis() - wait_start_ms),
                             static_cast<unsigned int>(fix_type),
                             fix_type_to_text(fix_type));
                }
                else
                {
                    ESP_LOGI(TAG, "GPS fix obtained after %lu ms via NMEA fallback.",
                             static_cast<unsigned long>(millis() - wait_start_ms));
                }
            }

            display_line(1, "GPS fix obtained");
            display_line(2, ubx_fix_ready ? fix_type_to_text(fix_type) : "NMEA fallback");
            vTaskDelay(pdMS_TO_TICKS(5000));
            return;
        }

        if (last_status_log_ms == 0 || (now_ms - last_status_log_ms) >= 5000UL)
        {
            if (debugIsOn)
            {
                if (s_use_nmea_fallback)
                {
                    ESP_LOGI(TAG, "Waiting for GPS fix: elapsed=%lu ms, NMEA fallback active",
                             static_cast<unsigned long>(now_ms - wait_start_ms));
                }
                else
                {
                    ESP_LOGI(TAG, "Waiting for GPS fix: elapsed=%lu ms, fix_type=%u (%s), gnss_fix_ok=%s, date_valid=%s, time_valid=%s, SIV=%u",
                             static_cast<unsigned long>(now_ms - wait_start_ms),
                             static_cast<unsigned int>(fix_type),
                             fix_type_to_text(fix_type),
                             gnss_fix_ok ? "true" : "false",
                             date_valid ? "true" : "false",
                             time_valid ? "true" : "false",
                             static_cast<unsigned int>(satellites_used));
                }
            }
            last_status_log_ms = now_ms;
        }

        if ((now_ms - wait_start_ms) >= 60000UL && (last_long_wait_warning_ms == 0 || (now_ms - last_long_wait_warning_ms) >= 30000UL))
        {
            if (debugIsOn)
                ESP_LOGW(TAG, "GPS has not produced a usable fix yet. Check antenna placement, sky view, and module compatibility.");
            last_long_wait_warning_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void arduino_eth_event_handler(arduino_event_id_t event, arduino_event_info_t info)
{
    switch (event)
    {
    case ARDUINO_EVENT_ETH_START:
        if (debugIsOn)
            ESP_LOGI(TAG, "Ethernet driver started");
        ETH.setHostname(DeviceName);
        display_line(1, "Ethernet started");
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        if (debugIsOn)
            ESP_LOGI(TAG, "Ethernet link connected");
        xEventGroupSetBits(s_net_event_group, ETH_CONNECTED_BIT);
        display_line(1, "Ethernet connected");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        snprintf(s_ip_address, sizeof(s_ip_address), IPSTR, IP2STR(&info.got_ip.ip_info.ip));
        if (debugIsOn)
            ESP_LOGI(TAG,
                     "Ethernet IP acquired: ip=" IPSTR ", mask=" IPSTR ", gw=" IPSTR,
                     IP2STR(&info.got_ip.ip_info.ip),
                     IP2STR(&info.got_ip.ip_info.netmask),
                     IP2STR(&info.got_ip.ip_info.gw));
        xEventGroupSetBits(s_net_event_group, ETH_GOT_IP_BIT);
        display_line(3, s_ip_address);
        break;
    case ARDUINO_EVENT_ETH_LOST_IP:
        if (debugIsOn)
            ESP_LOGW(TAG, "Ethernet lost IP address");
        xEventGroupClearBits(s_net_event_group, ETH_GOT_IP_BIT);
        s_ip_address[0] = '\0';
        display_line(1, "Ethernet lost IP");
        display_line(3, "");
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        if (debugIsOn)
            ESP_LOGW(TAG, "Ethernet link disconnected");
        xEventGroupClearBits(s_net_event_group, ETH_CONNECTED_BIT | ETH_GOT_IP_BIT);
        s_ip_address[0] = '\0';
        display_line(1, "Ethernet disconnect");
        display_line(3, "");
        break;
    case ARDUINO_EVENT_ETH_STOP:
        if (debugIsOn)
            ESP_LOGW(TAG, "Ethernet driver stopped");
        xEventGroupClearBits(s_net_event_group, ETH_CONNECTED_BIT | ETH_GOT_IP_BIT);
        s_ip_address[0] = '\0';
        display_line(1, "Ethernet stopped");
        display_line(3, "");
        break;
    default:
        break;
    }
}

static void setup_ethernet()
{
    if (s_net_event_group == nullptr)
        s_net_event_group = xEventGroupCreate();

    s_ip_address[0] = '\0';
    xEventGroupClearBits(s_net_event_group, ETH_CONNECTED_BIT | ETH_GOT_IP_BIT);

    Network.onEvent(arduino_eth_event_handler);

    if (debugIsOn)
        ESP_LOGI(TAG,
                 "Starting Arduino Ethernet with phy_addr=%d, mdc=%d, mdio=%d, power=%d",
                 ETH_PHY_ADDRESS,
                 static_cast<int>(ETH_MDC_GPIO),
                 static_cast<int>(ETH_MDIO_GPIO),
                 static_cast<int>(ETH_PHY_RST_GPIO));

    if (!ETH.begin(ETH_PHY_IP101,
                   ETH_PHY_ADDRESS,
                   static_cast<int>(ETH_MDC_GPIO),
                   static_cast<int>(ETH_MDIO_GPIO),
                   static_cast<int>(ETH_PHY_RST_GPIO),
                   EMAC_CLK_EXT_IN))
    {
        if (debugIsOn)
            ESP_LOGE(TAG, "ETH.begin() failed");
        return;
    }

    if (debugIsOn)
        ESP_LOGI(TAG, "Waiting for Ethernet DHCP address...");
    xEventGroupWaitBits(s_net_event_group, ETH_GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    if (debugIsOn)
        ESP_LOGI(TAG, "Ethernet setup complete, current IP: %s", s_ip_address[0] == '\0' ? "<none>" : s_ip_address);
}

void write_opening_messages_to_the_console()
{

    Serial.begin(serialMonitorSpeed);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "******************* Application Startup *******************");

    if (!debugIsOn)
    {
        ESP_LOGW(TAG, "debugIsOn is turned off in the settings");
        ESP_LOGW(TAG, "No further application generate console displays will be generated");
        return;
    };

    if (supportForAnUpTimeRestartButton)
        ESP_LOGI(TAG, "Up time / Reset button support is turned on in settings");
    else
        ESP_LOGW(TAG, "Up time / Reset button support is turned off in settings");

    if (supportForLiquidCrystalDisplay)
        ESP_LOGI(TAG, "LCD support is turned on in settings");
    else
        ESP_LOGW(TAG, "LCD support is turned off in settings");

    if (supportForOTEUpdates)
        ESP_LOGI(TAG, "Over the Ethernet update support is turned on in settings");
    else
        ESP_LOGW(TAG, "Over the Ethernet update support is turned off in settings");
}

void setup_NVM_storage(void)
{

    if (!initialize_nvs_storage())
        ESP_LOGE(TAG, "NVS initialization failed. GPS module persistence is unavailable.");
}

void create_mutexes_and_semaphores(void)
{

    s_lcd_mutex = xSemaphoreCreateMutex();
    s_time_mutex = xSemaphoreCreateMutex();
    s_pps_semaphore = xSemaphoreCreateBinary();
    s_pps_discipline_semaphore = xSemaphoreCreateBinary();
    s_ota_mutex = xSemaphoreCreateMutex();
}

void initialize_the_display(void)
{

    if (!supportForLiquidCrystalDisplay)
        return;

    if (setup_lcd() == ESP_OK)
    {
        display_line(0, "ESP32 Time Server");
        display_line(1, "");
        display_line(2, "");
        display_line(3, "");
    }
    else
    {
        if (debugIsOn)
            ESP_LOGE(TAG, "LCD setup failed");
    }
}

static void setup_up_time_button()
{

    if (supportForAnUpTimeRestartButton)
    {
        gpio_config_t config{};
        config.pin_bit_mask = 1ULL << upTimeRestartPin;
        config.mode = GPIO_MODE_INPUT;
        config.pull_up_en = GPIO_PULLUP_ENABLE;
        config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        config.intr_type = GPIO_INTR_DISABLE;
        ESP_ERROR_CHECK(gpio_config(&config));
    }
}

void setup_ethernet_connection()
{
    display_line(1, "Connecting Ethernet");
    display_line(2, "");
    setup_ethernet();
    display_line(3, s_ip_address);
}

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
    if (debugIsOn)
        ESP_LOGI(TAG, "OTA service task started on port %u", static_cast<unsigned int>(OTA_Port));

    // uint32_t last_heartbeat_ms = 0;   // uncomment this line and the block below if you want to see a heart beat message in the console log every 10 seconds

    for (;;)
    {
        ArduinoOTA.handle();

        // uncomment the following if you want to see a heart beat message in the console log every 10 seconds
        /*
        uint32_t now_ms = millis();
        if (last_heartbeat_ms == 0 || (now_ms - last_heartbeat_ms) >= 10000UL)
        {

            if (debugIsOn)
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
            esp_restart();

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
    }

    bool network_begin_ok = Network.begin();
    if (debugIsOn)
        ESP_LOGI(TAG,
                 "Arduino Network.begin()=%s, online=%s, current_ip=%s",
                 network_begin_ok ? "true" : "false",
                 Network.isOnline() ? "true" : "false",
                 s_ip_address[0] == '\0' ? "<none>" : s_ip_address);
    if (debugIsOn)
        ESP_LOGI(TAG,
                 "Configuring ArduinoOTA: host=%s, port=%u, password_length=%u",
                 DeviceName,
                 static_cast<unsigned int>(OTA_Port),
                 static_cast<unsigned int>(strlen(OTA_Password)));

    ArduinoOTA.setPort(OTA_Port);
    ArduinoOTA.setHostname(DeviceName);
    ArduinoOTA.setPassword(OTA_Password);
    ArduinoOTA.setRebootOnSuccess(false);
    ArduinoOTA.onStart([]()
                       {
        ota_set_running_state(0, 1);
        if (debugIsOn) ESP_LOGI(TAG, "OTA update started"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { ota_set_running_state(progress, total); });
    ArduinoOTA.onEnd([]()
                     {
        ota_set_success_state();
        if (debugIsOn) ESP_LOGI(TAG, "OTA update completed successfully"); });
    ArduinoOTA.onError([](ota_error_t error)
                       {
        char reason[lcdColumns + 1];
        format_ota_error_reason(error, reason, sizeof(reason));
        ota_set_failure_state(reason);
        if (debugIsOn) ESP_LOGE(TAG, "OTA update failed: %s", reason); });
    if (debugIsOn)
        ESP_LOGI(TAG, "Calling ArduinoOTA.begin()...");
    ArduinoOTA.begin();

    if (debugIsOn)
        ESP_LOGI(TAG,
                 "ArduinoOTA.begin() returned, listener should be available on %s:%u",
                 s_ip_address[0] == '\0' ? DeviceName : s_ip_address,
                 static_cast<unsigned int>(OTA_Port));
}

void setup_for_ota_updates()
{
    if (!supportForOTEUpdates)
        return;

    display_line(1, "Setup OTA");
    display_line(2, "");
    setup_ota();

    // note: this task is intentionally pinned to core 0 (as opposed to tskNO_AFFINITY)
    xTaskCreatePinnedToCore(ota_service_task, "ota_service", 3560, nullptr, 5, nullptr, 0);
}

static void gps_time_sync_task(void *parameter)
{
    bool first_sync = true;

    for (;;)
    {

        /*

        // The following code is used to determine the ideal stack size for this method (gps_time_sync_task)
        // it only needs to be setup and run once
        //
        //
        // NOTE 1: specific to gps_time_sync_task,
        //         in the other tasks the code to get a suggest stack size was added at the end of the method
        //         however as several continues are used in the for loop of this method this
        //         code block has been placed at the top of the loop
        //
        // NOTE 2: specific to gps_time_sync_task, the setting
        //         static constexpr uint32_t periodicGPSRefreshEveryThisNumberOfMinutes
        //         had its normal value of 5UL to resync with GPS every 5 minutes
        //         updated to 1UL so that the resync would happen every minute
        //         as the initial run of this task does not provide for a representative suggested stack value
        //         however subsequent runs did provide for a good value that could be used
        //         when testing was complete the periodicGPSRefreshEveryThisNumberOfMinutes was reset to its normal default value
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
        //         press 'Q' to quit
        //         code should now build, flash and run ok
        //
        //         once the ideal stack size is known, the code can below can be commented out and the configUSE_TRACE_FACILITY disabled to save RAM
        //
        // NOTE 4: if in the future this program changes significantly the suggested stack size value generated in this testing may need to be redone
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

        ESP_LOGI("gps_time_sync_task",
                 "Stack report: Allocated=%u bytes, HighWater=%u bytes unused, PeakUsage=%u bytes, Suggested=%u bytes",
                 (unsigned)allocated_bytes,
                 (unsigned)high_watermark_bytes,
                 (unsigned)peak_usage_bytes,
                 (unsigned)suggested_bytes);

        vTaskDelay(pdMS_TO_TICKS(50));

        // Results of this testing:
        //
        // gps_time_sync_task: Stack report: Allocated=16384 bytes, HighWater=14352 bytes unused, PeakUsage=2032 bytes, Suggested=2540 bytes

        */

        s_time_setting_in_progress = true;

        time_t candidate_time = 0;
        bool use_pps_alignment = false;
        int64_t pps_release_time_us = esp_timer_get_time();

        if (s_use_nmea_fallback)
        {
            nmea_rmc_time_t nmea_time{};
            if (!wait_for_nmea_rmc_time(&nmea_time, 3000UL))
            {
                s_time_setting_in_progress = false;
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }

            candidate_time = epoch_from_utc(nmea_time.year, nmea_time.month, nmea_time.day, nmea_time.hour, nmea_time.minute, nmea_time.second);

            clear_pps_events();
            if (xSemaphoreTake(s_pps_semaphore, pdMS_TO_TICKS(1500)) == pdTRUE)
            {
                use_pps_alignment = true;
                pps_release_time_us = esp_timer_get_time();
                candidate_time += 1;
            }
            else
            {
                ESP_LOGE(TAG, "NMEA fallback is running without PPS.");
                display_line(2, "NMEA fallback no PPS");

                if (!allowFallbackProcessingWithoutPPS)
                {
                    ESP_LOGE(TAG, "Fallback without PPS is not allowed - check the ESP32TimeServerSetting.h file.");
                    display_line(2, "GPS PPS unavailable");
                    s_time_setting_in_progress = false;
                    while (true)
                        vTaskDelay(pdMS_TO_TICKS(1000));
                }

                pps_release_time_us = esp_timer_get_time();
            }
        }
        else
        {
            clear_pps_events();
            if (xSemaphoreTake(s_pps_semaphore, pdMS_TO_TICKS(1500)) != pdTRUE)
            {
                ESP_LOGE(TAG, "UBX mode is running without PPS.");
                display_line(2, "UBX mode no PPS");

                if (!allowFallbackProcessingWithoutPPS)
                {
                    ESP_LOGE(TAG, "Fallback without PPS is not allowed - check the ESP32TimeServerSetting.h file.");
                    display_line(2, "Fallback not allowed");
                    s_time_setting_in_progress = false;
                    while (true)
                        vTaskDelay(pdMS_TO_TICKS(1000));
                }

                ESP_LOGW(TAG, "Switching to NMEA fallback because PPS is unavailable.");
                s_use_nmea_fallback = true;
                s_time_setting_in_progress = false;
                continue;
            }

            if (!s_gps.getPVT())
            {
                s_time_setting_in_progress = false;
                continue;
            }

            if (!s_gps.getDateValid() || !s_gps.getTimeValid())
            {
                s_time_setting_in_progress = false;
                continue;
            }

            int year = s_gps.getYear();
            int month = s_gps.getMonth();
            int day = s_gps.getDay();
            int hour = s_gps.getHour();
            int minute = s_gps.getMinute();
            int second = s_gps.getSecond();

            if (year <= 2022 || month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60)
            {
                s_time_setting_in_progress = false;
                continue;
            }

            candidate_time = epoch_from_utc(year, month, day, hour, minute, second) + 1;

            vTaskDelay(pdMS_TO_TICKS(200));

            clear_pps_events();
            if (xSemaphoreTake(s_pps_semaphore, pdMS_TO_TICKS(1500)) != pdTRUE)
            {
                ESP_LOGE(TAG, "UBX mode lost PPS alignment pulse.");
                display_line(2, "UBX mode no PPS");

                if (!allowFallbackProcessingWithoutPPS)
                {
                    ESP_LOGE(TAG, "Fallback without PPS is not allowed - check the ESP32TimeServerSetting.h file.");
                    display_line(2, "Fallback not allowed");
                    s_time_setting_in_progress = false;
                    while (true)
                        vTaskDelay(pdMS_TO_TICKS(1000));
                }

                ESP_LOGW(TAG, "Switching to NMEA fallback because PPS is unavailable.");
                s_use_nmea_fallback = true;
                s_time_setting_in_progress = false;
                continue;
            }

            pps_release_time_us = esp_timer_get_time();
            use_pps_alignment = true;
        }

        bool sanity_check_passed = first_sync;
        time_t update_delta = 0;
        if (!first_sync)
        {
            time_t current_time = time(nullptr);
            update_delta = current_time - candidate_time;
            sanity_check_passed = (update_delta >= -safeguardThresholdInSeconds) && (update_delta <= safeguardThresholdInSeconds);
        }

        if (!sanity_check_passed)
        {
            s_safe_guard_tripped = true;
            s_time_setting_in_progress = false;

            if (debugIsOn)
                ESP_LOGE(TAG, "Sanity check failed.");

            if (rebootIfSanityCheckFails)
            {
                if (debugIsOn)
                    ESP_LOGE(TAG, "Restarting according to settings.");
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            };
        }

        if (xSemaphoreTake(s_time_mutex, portMAX_DELAY) == pdTRUE)
        {
            int64_t elapsed_us = use_pps_alignment ? (esp_timer_get_time() - pps_release_time_us) : 0;
            if (elapsed_us < 0)
                elapsed_us = 0;

            struct timeval tv{};
            tv.tv_sec = candidate_time + static_cast<time_t>(elapsed_us / 1000000LL);
            tv.tv_usec = static_cast<suseconds_t>(elapsed_us % 1000000LL);
            settimeofday(&tv, nullptr);
            s_ntp_reference_time_64 = get_current_time_in_ntp64_format();
            s_ntp_reference_valid = true;

            xSemaphoreGive(s_time_mutex);

            s_safe_guard_tripped = false;
            s_time_setting_in_progress = false;
            s_time_has_been_set = true;
            first_sync = false;

            if (s_attempt_no_signal_recovery)
            {
                bool save_recovery_setting_ok = save_attempt_no_signal_recovery_setting(false);
                s_attempt_no_signal_recovery = false;
                if (debugIsOn)
                    ESP_LOGI(TAG, "GPS time set succeeded; no-signal recovery is now persisted as false (save=%s)", save_recovery_setting_ok ? "ok" : "failed");
            }

            if (debugIsOn)
            {
                char date_string[16] = "";
                char time_string[24] = "";
                time_t now_utc = time(nullptr);
                format_local_date_time(now_utc, date_string, sizeof(date_string), time_string, sizeof(time_string));

                if (s_use_nmea_fallback)
                    ESP_LOGI(TAG, "GPS time sync ( using NMEA fallback %s ) on %s at %s", use_pps_alignment ? "with PPS alignment" : "without PPS alignment", date_string, time_string);
                else
                    ESP_LOGI(TAG, "GPS time sync on %s at %s", date_string, time_string);
            };

            vTaskDelay(pdMS_TO_TICKS(periodicGPSRefreshEveryThisNumberOfMinutes * 60UL * 1000UL));
        }
    }
}

void setup_the_gps()
{

    display_line(1, "GPS setup underway");
    display_line(2, "");

    apply_timezone_settings();
    setup_pps_input();

    setup_gps();

    display_line(1, "Getting date & time");
    display_line(2, "");

    xTaskCreatePinnedToCore(gps_time_sync_task, "gps_time_sync", 2540, nullptr, 15, nullptr, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(pps_discipline_task, "pps_discipline", 2385, nullptr, 14, nullptr, tskNO_AFFINITY);

    while (!s_time_has_been_set)
        vTaskDelay(pdMS_TO_TICKS(100));
}

static void ntp_server_task(void *parameter)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        if (debugIsOn)
            ESP_LOGE(TAG, "Unable to create UDP socket: errno %d", errno);
        vTaskDelete(nullptr);
        return;
    }

    struct sockaddr_in listen_addr{};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(NTP_PORT);
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, reinterpret_cast<struct sockaddr *>(&listen_addr), sizeof(listen_addr)) != 0)
    {
        if (debugIsOn)
            ESP_LOGE(TAG, "Unable to bind UDP socket: errno %d", errno);
        closesocket(sock);
        vTaskDelete(nullptr);
        return;
    }

    struct timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    for (;;)
    {
        uint8_t request[NTP_PACKET_SIZE];
        uint8_t reply[NTP_PACKET_SIZE];
        struct sockaddr_in source_addr{};
        socklen_t source_addr_len = sizeof(source_addr);

        int len = recvfrom(sock, request, sizeof(request), 0, reinterpret_cast<struct sockaddr *>(&source_addr), &source_addr_len);
        if (len < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            if (debugIsOn)
                ESP_LOGW(TAG, "recvfrom failed: errno %d", errno);
            continue;
        }

        uint64_t receive_time = get_current_time_in_ntp64_format();

        if (len != static_cast<int>(NTP_PACKET_SIZE))
            continue;

        bool reply_ready = false;
        if (xSemaphoreTake(s_time_mutex, portMAX_DELAY) == pdTRUE)
        {
            uint64_t transmit_time = get_current_time_in_ntp64_format();
            build_ntp_reply(request, reply, receive_time, transmit_time);
            xSemaphoreGive(s_time_mutex);
            reply_ready = true;
        }

        if (reply_ready)
        {
            sendto(sock, reply, sizeof(reply), 0, reinterpret_cast<struct sockaddr *>(&source_addr), source_addr_len);
            if (debugIsOn)
            {
                char source_address[16] = "";
                char date_string[16] = "";
                char time_string[24] = "";
                time_t now_utc = time(nullptr);
                inet_ntoa_r(source_addr.sin_addr, source_address, sizeof(source_address));
                format_local_date_time(now_utc, date_string, sizeof(date_string), time_string, sizeof(time_string));
                ESP_LOGI(TAG, "NTP response provided to %s on %s at %s", source_address, date_string, time_string);
            }
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
       //  ntp_server_task: Stack report: Allocated=16384 bytes, HighWater=14372 bytes unused, PeakUsage=2012 bytes, Suggested=2515 bytes

       */
    }
}

static void update_display_task(void *parameter)
{
    int previous_top_line_message = -1;
    int previous_second = -1;
    int display_uptime_seconds_counter = 0;

    for (;;)
    {
        if (render_ota_display())
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        time_t now_utc = time(nullptr);
        struct tm utc_tm{};
        gmtime_r(&now_utc, &utc_tm);

        if (utc_tm.tm_sec != previous_second)
        {
            previous_second = utc_tm.tm_sec;

            if (check_uptime_request())
                display_uptime_seconds_counter = upTimeDisplayWillStayActiveForThisManySeconds;

            int required_top_line_message = 1;
            if (display_uptime_seconds_counter > 0)
            {
                required_top_line_message = 4;
            }
            else if (s_safe_guard_tripped)
            {
                required_top_line_message = 3;
            }
            else if (s_time_setting_in_progress)
            {
                required_top_line_message = 2;
            }

            if (required_top_line_message != previous_top_line_message)
            {
                const char *top_line_message = "ESP32 Time Server";
                if (required_top_line_message == 2)
                {
                    top_line_message = "ESP32 Time Server *";
                }
                else if (required_top_line_message == 3)
                {
                    top_line_message = "ESP32 Time Server **";
                }
                else if (required_top_line_message == 4)
                {
                    top_line_message = "ESP32 Time Server's";
                }

                display_line(0, top_line_message);
                previous_top_line_message = required_top_line_message;
            }

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
            {
                char date_line[16];
                char time_line[24];
                format_local_date_time(now_utc, date_line, sizeof(date_line), time_line, sizeof(time_line));
                display_line(1, date_line);
                display_line(2, time_line);
                display_line(3, s_ip_address);
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
        //  update_display_task: Stack report: Allocated=16384 bytes, HighWater=14492 bytes unused, PeakUsage=1892 bytes, Suggested=2365 bytes

        */
    }
}

extern "C" void app_main()
{
    initArduino();

    write_opening_messages_to_the_console();

    setup_NVM_storage();

    create_mutexes_and_semaphores();

    initialize_the_display();

    setup_up_time_button();

    setup_ethernet_connection();

    setup_for_ota_updates();

    setup_the_gps();

    xTaskCreatePinnedToCore(ntp_server_task, "ntp_server", 2515, nullptr, 20, nullptr, tskNO_AFFINITY);

    xTaskCreatePinnedToCore(update_display_task, "display_service", 2365, nullptr, 10, nullptr, tskNO_AFFINITY);
}

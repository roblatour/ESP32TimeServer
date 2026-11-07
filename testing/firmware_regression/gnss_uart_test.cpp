#include <algorithm>
#include <cassert>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <iterator>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "gnss_receiver.h"
#include "gnss_uart.h"
#include "soc/soc_caps.h"

#define DEBUG_ENABLED 0
#define MQTT_ENABLED 1
#define UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED 0

static constexpr const char *TAG = "gnss-test";
static constexpr int TXPin = 16, RXPin = 17;
static constexpr int upTimeRestartPin = 3;
static constexpr unsigned long holdUpTimeRestartButtonForThisManySecondsToTriggerAReset = 2;
static int64_t s_now_us = 0;
static bool s_driver_installed = false, s_foreign_driver = false;
static bool s_fail_config = false, s_fail_pin = false, s_fail_install = false;
static bool s_fail_delete = false, s_fail_read = false, s_fail_available = false, s_fail_write = false;
static bool s_fail_flush = false, s_fail_begin_available = false;
static bool s_probe_reply = false;
static bool s_emulate_receiver = false, s_drop_baud_ack = false;
static uint32_t s_receiver_baud = 0, s_receiver_max_baud = 0, s_blocked_probe_baud = 0;
static std::vector<uint32_t> s_requested_bauds;
static bool s_fail_pullup = false, s_fail_floating = false, s_fail_timeout = false, s_fail_fifo = false;
static unsigned s_installs = 0, s_deletes = 0, s_config_calls = 0, s_log_count = 0;
static unsigned s_restart_count = 0, s_clear_count = 0, s_satellite_count = 0;
static bool s_button_pressed = false;
static int s_tx_pin = -1, s_rx_pin = -1;
static int s_fifo_threshold = 0;
static uint8_t s_rx_timeout = 0;
static gpio_pull_mode_t s_pull_mode = GPIO_FLOATING;
static uart_config_t s_uart_config{};
static TickType_t s_last_read_timeout = 0, s_last_flush_timeout = 0;
static size_t s_write_limit = SIZE_MAX;
static std::deque<uint8_t> s_rx;
static std::deque<std::vector<uint8_t>> s_open_data;
static std::vector<uint8_t> s_tx;
static unsigned s_probe_requests = 0;
static gnss::Receiver s_gnss;
static gnss::TimeSolution s_gnss_time_solution{};
static uint32_t s_detected_gnss_baud = 0;
static bool s_gnss_required_assume_success = false;
static constexpr size_t lcdColumns = 20;
struct gnss_identity_t { bool valid = false; };
static gnss_identity_t query_gnss_identity() { return {true}; }
static void display_line(uint8_t, const char *) {}

int64_t esp_timer_get_time() { return s_now_us; }
void vTaskDelay(uint32_t ticks) { s_now_us += static_cast<int64_t>(ticks) * 1000; }
static int gpio_get_level(gpio_num_t) { return s_button_pressed ? 0 : 1; }
static void clear_gnss_nvs_data() { ++s_clear_count; }
static void controlled_restart(const char *reason)
{
    assert(std::strcmp(reason, "manual_restart") == 0);
    ++s_restart_count;
}
static void mqtt_note_satellite_count(uint8_t count) { s_satellite_count = count; }

const char *esp_err_to_name(esp_err_t) { return "mock error"; }
void fixture_gnss_log(const char *, const char *, ...) { ++s_log_count; }

bool uart_is_driver_installed(uart_port_t port)
{
    assert(port == UART_NUM_1);
    return s_driver_installed || s_foreign_driver;
}

esp_err_t uart_param_config(uart_port_t port, const uart_config_t *config)
{
    assert(port == UART_NUM_1 && config != nullptr && s_driver_installed);
    ++s_config_calls;
    if (s_fail_config)
        return ESP_FAIL;
    s_uart_config = *config;
    return ESP_OK;
}

esp_err_t uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts)
{
    assert(port == UART_NUM_1 && s_driver_installed && rts == UART_PIN_NO_CHANGE && cts == UART_PIN_NO_CHANGE);
    if (s_fail_pin)
        return ESP_FAIL;
    s_tx_pin = tx;
    s_rx_pin = rx;
    return ESP_OK;
}

esp_err_t gpio_set_pull_mode(gpio_num_t pin, gpio_pull_mode_t mode)
{
    assert(pin == s_rx_pin);
    if ((mode == GPIO_PULLUP_ONLY && s_fail_pullup) || (mode == GPIO_FLOATING && s_fail_floating))
        return ESP_FAIL;
    s_pull_mode = mode;
    return ESP_OK;
}

esp_err_t uart_set_rx_timeout(uart_port_t port, uint8_t timeout)
{
    assert(port == UART_NUM_1 && s_driver_installed);
    if (s_fail_timeout)
        return ESP_FAIL;
    s_rx_timeout = timeout;
    return ESP_OK;
}

esp_err_t uart_set_rx_full_threshold(uart_port_t port, int threshold)
{
    assert(port == UART_NUM_1 && s_driver_installed);
    if (s_fail_fifo)
        return ESP_FAIL;
    s_fifo_threshold = threshold;
    return ESP_OK;
}

esp_err_t uart_driver_install(uart_port_t port, int rx_size, int tx_size,
                              int queue_size, void *queue, int flags)
{
    assert(port == UART_NUM_1 && rx_size == 256 && tx_size == 0);
    assert(queue_size == 0 && queue == nullptr && flags == 0 && !s_driver_installed);
    if (s_fail_install)
        return ESP_FAIL;
    s_driver_installed = true;
    ++s_installs;
    if (!s_open_data.empty())
    {
        s_rx.assign(s_open_data.front().begin(), s_open_data.front().end());
        s_open_data.pop_front();
    }
    return ESP_OK;
}

esp_err_t uart_driver_delete(uart_port_t port)
{
    assert(port == UART_NUM_1 && s_driver_installed);
    if (s_fail_delete)
        return ESP_FAIL;
    s_driver_installed = false;
    s_rx.clear();
    ++s_deletes;
    return ESP_OK;
}

esp_err_t uart_get_buffered_data_len(uart_port_t port, size_t *available)
{
    assert(port == UART_NUM_1 && s_driver_installed);
    if (s_fail_available || s_fail_begin_available)
        return ESP_FAIL;
    *available = s_rx.size();
    return ESP_OK;
}

int uart_read_bytes(uart_port_t port, void *buffer, uint32_t length, TickType_t timeout)
{
    assert(port == UART_NUM_1 && s_driver_installed && buffer != nullptr);
    s_last_read_timeout = timeout;
    if (s_fail_read)
        return -1;
    auto *output = static_cast<uint8_t *>(buffer);
    const size_t count = std::min<size_t>(length, s_rx.size());
    for (size_t i = 0; i < count; ++i)
    {
        output[i] = s_rx.front();
        s_rx.pop_front();
    }
    if (count < length && timeout != portMAX_DELAY)
        s_now_us += static_cast<int64_t>(timeout) * 1000;
    return static_cast<int>(count);
}

static void queue_reply(uint8_t cls, uint8_t id, const std::vector<uint8_t> &payload)
{
    std::vector<uint8_t> response = {0xb5, 0x62, cls, id, static_cast<uint8_t>(payload.size()), 0};
    response.insert(response.end(), payload.begin(), payload.end());
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < response.size(); ++i)
    {
        a = static_cast<uint8_t>(a + response[i]);
        b = static_cast<uint8_t>(b + a);
    }
    response.push_back(a);
    response.push_back(b);
    s_rx.insert(s_rx.end(), response.begin(), response.end());
}

int uart_write_bytes(uart_port_t port, const void *buffer, size_t length)
{
    assert(port == UART_NUM_1 && s_driver_installed && buffer != nullptr);
    if (s_fail_write)
        return -1;
    const auto *bytes = static_cast<const uint8_t *>(buffer);
    const size_t count = std::min(length, s_write_limit);
    s_tx.insert(s_tx.end(), bytes, bytes + count);
    if (count == 16 && bytes[0] == 0xb5 && bytes[1] == 0x62 &&
        bytes[2] == 0x06 && bytes[3] == 0x8b)
    {
        ++s_probe_requests;
        if (s_probe_reply ||
            (s_emulate_receiver && static_cast<uint32_t>(s_uart_config.baud_rate) == s_receiver_baud &&
             s_receiver_baud != s_blocked_probe_baud))
            queue_reply(0x06, 0x8b, {1, 0, 0, 0, 1, 0, 0x73, 0x10, 1});
    }
    else if (s_emulate_receiver && count == 20 && bytes[0] == 0xb5 && bytes[1] == 0x62 &&
             bytes[2] == 0x06 && bytes[3] == 0x8a &&
             static_cast<uint32_t>(s_uart_config.baud_rate) == s_receiver_baud)
    {
        assert(bytes[10] == 1 && bytes[11] == 0 && bytes[12] == 0x52 && bytes[13] == 0x40);
        const uint32_t requested = static_cast<uint32_t>(bytes[14]) |
                                   (static_cast<uint32_t>(bytes[15]) << 8) |
                                   (static_cast<uint32_t>(bytes[16]) << 16) |
                                   (static_cast<uint32_t>(bytes[17]) << 24);
        s_requested_bauds.push_back(requested);
        const bool accepted = requested <= s_receiver_max_baud;
        if (!s_drop_baud_ack || !accepted)
            queue_reply(0x05, accepted ? 1 : 0, {0x06, 0x8a});
        if (accepted)
            s_receiver_baud = requested;
    }
    return static_cast<int>(count);
}

esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t timeout)
{
    assert(port == UART_NUM_1 && s_driver_installed);
    s_last_flush_timeout = timeout;
    return s_fail_flush ? ESP_FAIL : ESP_OK;
}

#include "gnss_app_under_test.h"

static void queue_rx(const std::string &text)
{
    s_rx.insert(s_rx.end(), text.begin(), text.end());
}

static void queue_open(const std::string &text)
{
    s_open_data.emplace_back(text.begin(), text.end());
}

static std::string checked_sentence(const char *body)
{
    uint8_t checksum = 0;
    for (const char *cursor = body; *cursor != '\0'; ++cursor)
        checksum ^= static_cast<uint8_t>(*cursor);
    char suffix[4];
    std::snprintf(suffix, sizeof(suffix), "*%02X", static_cast<unsigned>(checksum));
    return std::string("$") + body + suffix;
}

static void test_driver_lifecycle()
{
    assert(gnss_uart_close() == ESP_OK);
    assert(gnss_uart_open(TXPin, RXPin, 0) == ESP_ERR_INVALID_ARG);
    assert(gnss_uart_open(-1, RXPin, 9600) == ESP_ERR_INVALID_ARG);
    assert(gnss_uart_open(TXPin, -1, 9600) == ESP_ERR_INVALID_ARG);
    assert(gnss_uart_open(TXPin, TXPin, 9600) == ESP_ERR_INVALID_ARG);
    s_foreign_driver = true;
    const unsigned config_calls = s_config_calls;
    assert(gnss_uart_open(TXPin, RXPin, 9600) == ESP_ERR_INVALID_STATE);
    assert(s_config_calls == config_calls && s_deletes == 0);
    s_foreign_driver = false;
    s_fail_config = true;
    assert(gnss_uart_open(TXPin, RXPin, 9600) == ESP_FAIL && !s_driver_installed);
    s_fail_config = false;
    s_fail_pin = true;
    assert(gnss_uart_open(TXPin, RXPin, 9600) == ESP_FAIL && !s_driver_installed);
    s_fail_pin = false;
    s_fail_install = true;
    assert(gnss_uart_open(TXPin, RXPin, 9600) == ESP_FAIL && !s_driver_installed);
    s_fail_install = false;
    s_fail_pullup = true;
    assert(gnss_uart_open(TXPin, RXPin, 9600) == ESP_FAIL && !s_driver_installed && s_pull_mode == GPIO_FLOATING);
    s_fail_pullup = false;
    s_fail_timeout = true;
    assert(gnss_uart_open(TXPin, RXPin, 9600) == ESP_FAIL && !s_driver_installed && s_pull_mode == GPIO_FLOATING);
    s_fail_timeout = false;
    s_fail_fifo = true;
    assert(gnss_uart_open(TXPin, RXPin, 9600) == ESP_FAIL && !s_driver_installed && s_pull_mode == GPIO_FLOATING);
    s_fail_delete = true;
    const unsigned before_cleanup_logs = s_log_count;
    assert(gnss_uart_open(TXPin, RXPin, 9600) == ESP_FAIL && s_driver_installed);
    assert(s_log_count == before_cleanup_logs + 1);
    s_fail_delete = false;
    assert(gnss_uart_close() == ESP_OK && !s_driver_installed && s_pull_mode == GPIO_FLOATING);
    s_fail_fifo = false;
    assert(gnss_uart_open(TXPin, RXPin, 57600) == ESP_OK);
    assert(s_pull_mode == GPIO_PULLUP_ONLY && s_rx_timeout == 1 && s_fifo_threshold == 1);
    assert(gnss_uart_open(TXPin, RXPin, 57601) == ESP_OK);
    assert(s_rx_timeout == 1 && s_fifo_threshold == 120);
    assert(gnss_uart_open(TXPin, RXPin, 9600) == ESP_OK);
    assert(s_tx_pin == TXPin && s_rx_pin == RXPin && s_uart_config.baud_rate == 9600);
    assert(s_uart_config.data_bits == UART_DATA_8_BITS && s_uart_config.parity == UART_PARITY_DISABLE);
    assert(s_uart_config.stop_bits == UART_STOP_BITS_1 && s_uart_config.flow_ctrl == UART_HW_FLOWCTRL_DISABLE);
#if SOC_UART_SUPPORT_XTAL_CLK
    assert(s_uart_config.source_clk == UART_SCLK_XTAL);
#else
    assert(s_uart_config.source_clk == UART_SCLK_DEFAULT);
#endif
    queue_rx("old");
    const unsigned installs = s_installs, deletes = s_deletes;
    assert(gnss_uart_open(TXPin, RXPin, 115200) == ESP_OK);
    assert(s_installs == installs + 1 && s_deletes == deletes + 1 && s_rx.empty());
    assert(s_uart_config.baud_rate == 115200);
    s_fail_delete = true;
    assert(gnss_uart_open(TXPin, RXPin, 38400) == ESP_FAIL && s_driver_installed);
    s_fail_delete = false;
    s_fail_floating = true;
    assert(gnss_uart_close() == ESP_FAIL && !s_driver_installed);
    s_fail_floating = false;
    assert(gnss_uart_close() == ESP_OK && s_pull_mode == GPIO_FLOATING);
    assert(gnss_uart_close() == ESP_OK);
    size_t count = 0;
    uint8_t byte = 0;
    assert(gnss_uart_read(&byte, 1, 0, &count) == ESP_ERR_INVALID_STATE);
    assert(gnss_uart_open(TXPin, RXPin, 38400) == ESP_OK);
    std::puts("PASS: native UART ownership, 8N1 configuration, baud transitions, failure cleanup and reopen");
}

static void test_transport()
{
    queue_rx("ABC");
    size_t count = 0;
    uint8_t bytes[4]{};
    uint8_t peeked = 0;
    bool has_data = false;
    assert(gnss_uart_peek(&peeked, &has_data) == ESP_OK && has_data && peeked == 'A');
    assert(gnss_uart_peek(&peeked, &has_data) == ESP_OK && has_data && peeked == 'A');
    assert(gnss_uart_available(&count) == ESP_OK && count == 3);
    assert(gnss_uart_read(bytes, 1, 0, &count) == ESP_OK && count == 1 && bytes[0] == 'A');
    assert(gnss_uart_read(bytes, 2, 0, &count) == ESP_OK && count == 2 && bytes[0] == 'B' && bytes[1] == 'C');
    assert(gnss_uart_peek(&peeked, &has_data) == ESP_OK && !has_data);
    assert(gnss_uart_available(&count) == ESP_OK && count == 0);

    queue_rx("XY");
    const int64_t before = s_now_us;
    assert(gnss_uart_read(bytes, 3, 37, &count) == ESP_OK && count == 2);
    assert(bytes[0] == 'X' && bytes[1] == 'Y' && s_last_read_timeout == 37);
    assert(s_now_us - before == 37000);
    assert(gnss_uart_read(nullptr, 1, 0, &count) == ESP_ERR_INVALID_ARG);
    assert(gnss_uart_available(nullptr) == ESP_ERR_INVALID_ARG);

    uint8_t output[] = {1, 2, 3};
    s_write_limit = 2;
    assert(gnss_uart_write(output, 3, &count) == ESP_OK && count == 2);
    assert(s_tx.size() >= 2 && s_tx[s_tx.size() - 2] == 1 && s_tx.back() == 2);
    s_write_limit = SIZE_MAX;
    queue_rx("R");
    assert(gnss_uart_flush_tx(UINT32_MAX) == ESP_OK);
    assert(s_last_flush_timeout == portMAX_DELAY);
    assert(gnss_uart_read(bytes, 1, 0, &count) == ESP_OK && count == 1 && bytes[0] == 'R');
    s_fail_flush = true;
    assert(gnss_uart_flush_tx(37) == ESP_FAIL);
    s_fail_flush = false;

    queue_rx("peeked");
    assert(gnss_uart_peek(&peeked, &has_data) == ESP_OK && has_data && peeked == 'p');
    assert(gnss_uart_open(TXPin, RXPin, 921600) == ESP_OK);
    assert(gnss_uart_available(&count) == ESP_OK && count == 0);

    s_fail_read = true;
    assert(gnss_uart_read(bytes, 1, 0, &count) == ESP_FAIL && count == 0);
    s_fail_read = false;
    queue_rx("PQ");
    assert(gnss_uart_peek(&peeked, &has_data) == ESP_OK && has_data && peeked == 'P');
    s_fail_read = true;
    assert(gnss_uart_read(bytes, 2, 0, &count) == ESP_FAIL && count == 1 && bytes[0] == 'P');
    s_fail_read = false;
    assert(gnss_uart_read(bytes, 1, 0, &count) == ESP_OK && count == 1 && bytes[0] == 'Q');
    s_fail_write = true;
    assert(gnss_uart_write(output, sizeof(output), &count) == ESP_FAIL && count == 0);
    s_fail_write = false;
    s_fail_available = true;
    assert(gnss_uart_available(&count) == ESP_FAIL);
    s_fail_available = false;
    std::puts("PASS: native UART peek/read/write, partial I/O, timeout and TX-only flush");
}

static void test_application_paths()
{
    s_probe_reply = false;
    queue_open("$GPRMC,valid");
    queue_open("stale");
    bool saw_data = false;
    assert(try_gnss_begin(38400, true, &saw_data));
    assert(saw_data && s_probe_requests == 1 && s_rx.empty());

    queue_open("$GPRMC,valid");
    queue_open("stale");
    s_fail_begin_available = true;
    assert(!try_gnss_begin(38400, true, &saw_data));
    s_fail_begin_available = false;
    queue_open("noise");
    queue_open("");
    assert(!try_gnss_begin(38400, true, &saw_data));
    s_probe_reply = true;
    queue_open("");
    queue_open("stale");
    assert(try_gnss_begin(38400, false, &saw_data));
    s_probe_reply = false;

    s_now_us = (static_cast<int64_t>(UINT32_MAX) - 500) * 1000;
    queue_open("");
    const int64_t probe_start = s_now_us;
    const gnss_probe_result_t probe = probe_gnss_uart(9600);
    assert(!probe.saw_data && !probe.transport_error && s_now_us - probe_start == 1650000);

    const std::string located_rmc = "$GPRMC,123456.00,A,4916.4500,N,12311.1200,W,0.06,2.1,011126,,,A*00\r\n";
    nmea_sentence_buffer_t filtered{};
    bool complete = false;
    for (char byte : located_rmc)
    {
        complete = filtered.consume(byte) || complete;
        assert(std::strstr(filtered.sentence, "4916.4500") == nullptr);
        assert(std::strstr(filtered.sentence, "12311.1200") == nullptr);
        assert(std::strstr(filtered.sentence, "0.06") == nullptr);
    }
    assert(complete && filtered.kind == nmea_sentence_buffer_t::kind_t::rmc);
    nmea_rmc_time_t filtered_date{};
    assert(parse_nmea_rmc_sentence(filtered.sentence, &filtered_date));
    assert(filtered_date.year == 2026 && filtered_date.day == 1 && filtered_date.second == 56);

    queue_rx("$GPGGA,123456,4916.4500,N,12311.1200,W,1\r\n" + located_rmc);
    nmea_rmc_time_t date{};
    assert(wait_for_nmea_rmc_time(&date, 200));
    assert(date.year == 2026 && date.month == 11 && date.day == 1);
    assert(date.hour == 12 && date.minute == 34 && date.second == 56);
    s_now_us = (static_cast<int64_t>(UINT32_MAX) - 10) * 1000;
    const int64_t timeout_start = s_now_us;
    assert(!wait_for_nmea_rmc_time(&date, 50));
    assert(s_now_us - timeout_start == 60000);
    s_fail_read = true;
    assert(!wait_for_nmea_rmc_time(&date, 50));
    s_fail_read = false;

    s_now_us = (static_cast<int64_t>(UINT32_MAX) - 5000) * 1000;
    const std::string gsv = checked_sentence("GPGSV,1,1,01,01,00,000,00");
    process_nmea_gsv_sentence(gsv.c_str());
    const auto published = [] {
        return std::count_if(std::begin(s_nmea_gsv_groups), std::end(s_nmea_gsv_groups),
                             [](const nmea_gsv_group_t &group) { return group.published; });
    };
    assert(s_satellite_count == 1 && published() == 1);
    const uint32_t received_ms = uptime_ms();
    expire_nmea_gsv_groups(received_ms + 10000U);
    assert(published() == 1);
    expire_nmea_gsv_groups(received_ms + 10001U);
    assert(published() == 0);

    s_now_us = (static_cast<int64_t>(UINT32_MAX) - 1000) * 1000;
    s_button_pressed = true;
    assert(check_uptime_request());
    s_now_us += 1989000;
    assert(check_uptime_request() && s_restart_count == 0);
    s_now_us += 1000;
    assert(check_uptime_request() && s_restart_count == 1 && s_clear_count == 1);
    s_button_pressed = false;
    assert(!check_uptime_request());
    s_now_us = (static_cast<int64_t>(UINT32_MAX) + 1) * 1000 + 123456;
    assert(uptime_ms() == 123);
    std::puts("PASS: native application probe, NMEA fallback, transport errors and wraparound deadlines");
}

static void test_baud_recovery()
{
    s_open_data.clear();
    s_now_us = 0;
    s_emulate_receiver = true;
    s_drop_baud_ack = true;
    s_receiver_baud = 9600;
    s_receiver_max_baud = 115200;
    s_blocked_probe_baud = 0;
    s_requested_bauds.clear();
    assert(confirm_saved_gnss_baud_rate(9600));
    assert(change_gnss_baud_rate(9600, 115200));
    assert(s_detected_gnss_baud == 115200 && s_receiver_baud == 115200);
    assert(!s_gnss_required_assume_success && s_requested_bauds.back() == 115200);

    assert(!change_gnss_baud_rate(115200, 230400));
    assert(s_receiver_baud == 115200 && confirm_saved_gnss_baud_rate(115200));
    gnss_identity_t identity{};
    uint32_t initial_baud = 0;
    assert(set_gnss_baud_rate(1, &identity, &initial_baud));
    assert(identity.valid && initial_baud == 115200 && s_detected_gnss_baud == 115200);
    assert(std::find(s_requested_bauds.begin(), s_requested_bauds.end(), 921600U) != s_requested_bauds.end());

    s_drop_baud_ack = false;
    s_receiver_baud = 9600;
    s_receiver_max_baud = 115200;
    s_blocked_probe_baud = 115200;
    assert(confirm_saved_gnss_baud_rate(9600));
    assert(!change_gnss_baud_rate(9600, 115200));
    assert(s_receiver_baud == 115200 && !confirm_saved_gnss_baud_rate(9600));
    s_blocked_probe_baud = 0;
    assert(find_initial_gnss_baud_rate(1, &identity) && s_detected_gnss_baud == 115200);

    s_receiver_baud = 57600;
    assert(!confirm_saved_gnss_baud_rate(921600));
    assert(find_initial_gnss_baud_rate(1) && s_detected_gnss_baud == 57600);
    s_emulate_receiver = false;
    std::puts("PASS: baud ACK loss, reconnect failure, saved-rate recovery and highest-working-rate discovery");
}

int main()
{
    test_driver_lifecycle();
    test_transport();
    test_application_paths();
    test_baud_recovery();
    assert(gnss_uart_close() == ESP_OK);
}

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "driver/uart.h"
#include "esp_log.h"

#define DEBUG_ENABLED 0
#define CONFIG_ESP_CONSOLE_UART_NUM 0
#define pdTRUE 1

static constexpr const char *TAG = "startup-test";
static constexpr int serialMonitorSpeed = 57600;
static std::vector<std::string> s_events;
static esp_err_t s_nvs_error = ESP_OK, s_ota_error = ESP_OK, s_mark_error = ESP_OK, s_console_error = ESP_OK;
static esp_err_t s_psram_init_error = ESP_OK, s_psram_heap_error = ESP_OK;
static unsigned s_logs = 0, s_console_calls = 0, s_mark_calls = 0;
static unsigned s_psram_inits = 0, s_psram_tests = 0, s_psram_adds = 0, s_psram_malloc_enables = 0;
static bool s_nvs_passed = false, s_null_running = false;
static bool s_psram_initialized = false, s_psram_test_ok = true;
static uart_port_t s_console_port = UART_NUM_1;
static uint32_t s_console_baud = 0;

struct esp_partition_t {};
enum esp_ota_img_states_t
{
    ESP_OTA_IMG_VALID,
    ESP_OTA_IMG_PENDING_VERIFY,
};
static esp_partition_t s_running;
static esp_ota_img_states_t s_ota_state = ESP_OTA_IMG_VALID;

const char *esp_err_to_name(esp_err_t) { return "mock error"; }
void fixture_gnss_log(const char *, const char *, ...) { ++s_logs; }
esp_err_t nvs_flash_init()
{
    s_events.emplace_back("nvs-init");
    return s_nvs_error;
}
esp_err_t uart_set_baudrate(uart_port_t port, uint32_t baud)
{
    ++s_console_calls;
    s_console_port = port;
    s_console_baud = baud;
    return s_console_error;
}
bool esp_psram_is_initialized()
{
    s_events.emplace_back("psram");
    return s_psram_initialized;
}
esp_err_t esp_psram_init()
{
    ++s_psram_inits;
    if (s_psram_init_error == ESP_OK)
        s_psram_initialized = true;
    return s_psram_init_error;
}
bool esp_psram_extram_test()
{
    ++s_psram_tests;
    return s_psram_test_ok;
}
esp_err_t esp_psram_extram_add_to_heap_allocator()
{
    ++s_psram_adds;
    return s_psram_heap_error;
}
void heap_caps_malloc_extmem_enable(size_t threshold)
{
    assert(threshold == CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL);
    ++s_psram_malloc_enables;
}
const esp_partition_t *esp_ota_get_running_partition()
{
    s_events.emplace_back("ota");
    return s_null_running ? nullptr : &s_running;
}
esp_err_t esp_ota_get_state_partition(const esp_partition_t *partition, esp_ota_img_states_t *state)
{
    assert(partition == &s_running);
    *state = s_ota_state;
    return s_ota_error;
}
esp_err_t esp_ota_mark_app_valid_cancel_rollback()
{
    ++s_mark_calls;
    return s_mark_error;
}

static void check_NTP_and_PTP_settings() { s_events.emplace_back("settings"); }
static void validate_board_configuration() { s_events.emplace_back("board"); }
static void setup_pps_input() { s_events.emplace_back("pps"); }
static void write_opening_messages_to_the_console();
static void setup_mqtt_tf_queue() { s_events.emplace_back("tf"); }
static void setup_NVS_storage(bool initialized)
{
    s_events.emplace_back("nvs-storage");
    s_nvs_passed = initialized;
}
static void setup_mutexes_and_semaphores() {}
static void setup_the_RGB_LED() {}
static void setup_the_LCD() {}
static void setup_the_button() {}
static void start_RGB_LED_LCD_and_Button_refresh() {}
static void *xTaskGetCurrentTaskHandle() { return &s_running; }
static bool start_ethernet_setup_task(void *task) { return task == &s_running; }
static void setup_the_gnss() {}
static void ulTaskNotifyTake(int clear, TickType_t timeout)
{
    assert(clear == pdTRUE && timeout == portMAX_DELAY);
}
static void setup_for_ote_updates() {}
static void setup_mqtt() {}
static void setup_ntp_server() {}
static void setup_recovery_task() {}
static void perform_health_check() {}
static void write_open_for_business_messages_to_the_console() {}

#include "native_startup_under_test.h"

static void write_opening_messages_to_the_console()
{
    s_events.emplace_back("console");
    assert(configure_console() == ESP_OK);
}

static void test_nvs()
{
    for (const esp_err_t error : {ESP_ERR_NVS_NO_FREE_PAGES, ESP_ERR_NVS_NEW_VERSION_FOUND, ESP_FAIL})
    {
        s_nvs_error = error;
        const unsigned logs = s_logs;
        assert(!initialize_nvs_storage() && s_logs > logs);
    }
    s_nvs_error = ESP_OK;
    assert(initialize_nvs_storage());
    std::puts("PASS: explicit NVS initialization and non-destructive failure handling");
}

static void test_console()
{
    const unsigned calls = s_console_calls;
    assert(configure_console() == ESP_OK);
#if CONFIG_ESP_CONSOLE_UART
    assert(s_console_calls == calls + 1 && s_console_port == UART_NUM_0);
    assert(s_console_baud == static_cast<uint32_t>(serialMonitorSpeed));
    s_console_error = ESP_FAIL;
    assert(configure_console() == ESP_FAIL);
    s_console_error = ESP_OK;
#else
    assert(s_console_calls == calls);
#endif
    std::puts("PASS: configured native console selection does not use GNSS UART1");
}

static void test_psram()
{
#if CONFIG_SPIRAM && !CONFIG_SPIRAM_BOOT_INIT
#if CONFIG_SPIRAM_USE_MALLOC
    constexpr unsigned malloc_enables_per_add = 1;
#else
    constexpr unsigned malloc_enables_per_add = 0;
#endif
    s_psram_initialized = false;
    initialize_delayed_psram();
    assert(s_psram_inits == 1 && s_psram_tests == 1 && s_psram_adds == 1 &&
           s_psram_malloc_enables == malloc_enables_per_add);
    initialize_delayed_psram();
    assert(s_psram_inits == 1 && s_psram_tests == 2 && s_psram_adds == 2 &&
           s_psram_malloc_enables == 2 * malloc_enables_per_add);
    s_psram_initialized = false;
    s_psram_init_error = ESP_FAIL;
    unsigned logs = s_logs;
    initialize_delayed_psram();
    assert(s_logs == logs + 1 && s_psram_adds == 2);
    s_psram_init_error = ESP_OK;
    s_psram_test_ok = false;
    logs = s_logs;
    initialize_delayed_psram();
    assert(s_logs == logs + 1 && s_psram_adds == 2);
    logs = s_logs;
    initialize_delayed_psram();
    assert(s_logs == logs + 1 && s_psram_adds == 2);
    s_psram_test_ok = true;
    s_psram_heap_error = ESP_FAIL;
    logs = s_logs;
    initialize_delayed_psram();
    assert(s_logs == logs + 1 && s_psram_adds == 3 && s_psram_malloc_enables == 2 * malloc_enables_per_add);
    s_psram_heap_error = ESP_OK;
#else
    initialize_delayed_psram();
    assert(s_psram_inits == 0 && s_psram_adds == 0);
#endif
    std::puts("PASS: delayed PSRAM initialization and failure handling");
}

static void test_rollback()
{
#if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    s_ota_state = ESP_OTA_IMG_VALID;
    s_ota_error = ESP_ERR_NOT_SUPPORTED;
    confirm_running_ota_image();
    assert(s_mark_calls == 0);
    s_ota_error = ESP_OK;
    confirm_running_ota_image();
    assert(s_mark_calls == 0);
    s_ota_state = ESP_OTA_IMG_PENDING_VERIFY;
    confirm_running_ota_image();
    assert(s_mark_calls == 1);
    s_mark_error = ESP_FAIL;
    unsigned logs = s_logs;
    confirm_running_ota_image();
    assert(s_mark_calls == 2 && s_logs == logs + 1);
    s_mark_error = ESP_OK;
    s_ota_error = ESP_FAIL;
    logs = s_logs;
    confirm_running_ota_image();
    assert(s_mark_calls == 2 && s_logs == logs + 1);
    s_ota_error = ESP_OK;
    s_null_running = true;
    logs = s_logs;
    confirm_running_ota_image();
    assert(s_logs == logs + 1);
    s_null_running = false;
#else
    confirm_running_ota_image();
    assert(s_mark_calls == 0);
#endif
    std::puts("PASS: native OTA pending-image confirmation and explicit errors");
}

static void test_startup_order()
{
    s_events.clear();
    s_nvs_error = ESP_OK;
    s_ota_error = ESP_OK;
    s_ota_state = ESP_OTA_IMG_VALID;
    s_psram_initialized = false;
    app_main();
    assert(s_events.size() >= 7 && s_events[0] == "settings" && s_events[1] == "board" && s_events[2] == "pps");
    size_t index = 3;
#if CONFIG_SPIRAM && !CONFIG_SPIRAM_BOOT_INIT
    assert(s_events[index++] == "psram");
#endif
#if CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    assert(s_events[index++] == "ota");
#endif
    assert(s_events[index++] == "nvs-init");
    assert(s_events[index++] == "tf");
    assert(s_events[index++] == "console");
    assert(s_events[index] == "nvs-storage" && s_nvs_passed);

    s_nvs_error = ESP_ERR_NVS_NO_FREE_PAGES;
    app_main();
    assert(!s_nvs_passed);
    std::puts("PASS: early PPS registration and NVS-before-user startup ordering");
}

int main()
{
    test_nvs();
    test_console();
    test_psram();
    test_rollback();
    test_startup_order();
}

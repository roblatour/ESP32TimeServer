#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#define CONFIG_IDF_TARGET_ESP32P4 0
#define DEBUG_ENABLED 1
#define portMAX_DELAY 0xffffffffu

using esp_err_t = int;
static constexpr int ESP_OK = 0, ESP_FAIL = 1, ESP_ERR_NOT_FOUND = 2, ESP_ERR_NOT_SUPPORTED = 3;
static constexpr int LCD_I2C_PORT = 0, LCD_I2C_SDA_GPIO = 38, LCD_I2C_SCL_GPIO = 39;
static constexpr int lcdColumns = LCD_TEST_COLUMNS, lcdRows = LCD_TEST_ROWS;
static constexpr uint8_t lcdI2CAddressPrimary = 0x27, lcdI2CAddressSecondary = 0x3f;
static constexpr const char *TAG = "lcd-test";
enum hd44780_geometry_t
{
    HD44780_GEOMETRY_8X2, HD44780_GEOMETRY_16X2, HD44780_GEOMETRY_16X4,
    HD44780_GEOMETRY_20X2, HD44780_GEOMETRY_20X4, HD44780_GEOMETRY_40X2
};
struct hd44780_t {};
struct lcd_bus_hd44780_t {};
static hd44780_t lcd_object, *s_lcd = nullptr;
static lcd_bus_hd44780_t bus_object, *s_lcd_bus = nullptr;
static uint8_t s_lcd_addr = 0;
static bool s_lcd_ready = false;
static char s_lcd_last_lines[lcdRows][lcdColumns + 1]{};
static bool s_lcd_line_cached[lcdRows]{};
static int mutex, *s_lcd_mutex = &mutex;
static int bus_calls = 0, init_calls = 0, cleanup_calls = 0, control_calls = 0;
static int cursor_calls = 0, write_calls = 0, takes = 0, gives = 0, logs = 0;
static int failed_control = -1;
static bool primary_missing = false, secondary_missing = false, init_failure = false;
static bool cursor_failure = false, write_failure = false, cleanup_failure = false;
static std::string written;

static void record_log(const char *, const char *, ...) { ++logs; }
#define ESP_LOGE(tag, ...) record_log(tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) record_log(tag, __VA_ARGS__)
static const char *esp_err_to_name(esp_err_t) { return "injected error"; }
static int xSemaphoreTake(int *, unsigned) { ++takes; return 1; }
static int xSemaphoreGive(int *) { ++gives; return 1; }

static lcd_bus_hd44780_t *lcd_bus_pcf8574_i2c_create(int port, uint8_t address, int sda, int scl, uint32_t speed)
{
    assert(port == 0 && sda == 38 && scl == 39 && speed == 100000);
    ++bus_calls;
    if ((address == lcdI2CAddressPrimary && primary_missing) ||
        (address == lcdI2CAddressSecondary && secondary_missing))
        return nullptr;
    return &bus_object;
}
static hd44780_t *lcd_init(lcd_bus_hd44780_t *bus, hd44780_geometry_t geometry, bool owns_bus)
{
    assert(bus == &bus_object && owns_bus && geometry == HD44780_GEOMETRY_20X4);
    ++init_calls;
    return init_failure ? nullptr : &lcd_object;
}
static esp_err_t lcd_deinit(hd44780_t **lcd)
{
    assert(*lcd == &lcd_object);
    *lcd = nullptr;
    ++cleanup_calls;
    return cleanup_failure ? ESP_FAIL : ESP_OK;
}
static esp_err_t control(hd44780_t *lcd)
{
    assert(lcd == &lcd_object);
    return control_calls++ == failed_control ? ESP_FAIL : ESP_OK;
}
static esp_err_t lcd_try_backlight_on(hd44780_t *lcd) { return control(lcd); }
static esp_err_t lcd_try_display_on(hd44780_t *lcd) { return control(lcd); }
static esp_err_t lcd_try_cursor_off(hd44780_t *lcd) { return control(lcd); }
static esp_err_t lcd_try_blink_off(hd44780_t *lcd) { return control(lcd); }
static esp_err_t lcd_try_clear_screen(hd44780_t *lcd) { return control(lcd); }
static esp_err_t lcd_try_set_cursor(hd44780_t *lcd, uint8_t column, uint8_t row)
{
    assert(lcd == &lcd_object && column == 0 && row < lcdRows);
    ++cursor_calls;
    return cursor_failure ? ESP_FAIL : ESP_OK;
}
static esp_err_t lcd_try_write_str(hd44780_t *lcd, const char *text)
{
    assert(lcd == &lcd_object);
    ++write_calls;
    written = text;
    return write_failure ? ESP_FAIL : ESP_OK;
}

#include "s31_lcd_under_test.h"

static void reset()
{
    s_lcd = nullptr;
    s_lcd_bus = nullptr;
    s_lcd_addr = 0;
    s_lcd_ready = false;
    std::memset(s_lcd_last_lines, 0, sizeof(s_lcd_last_lines));
    std::memset(s_lcd_line_cached, 0, sizeof(s_lcd_line_cached));
    bus_calls = init_calls = cleanup_calls = control_calls = cursor_calls = write_calls = takes = gives = logs = 0;
    failed_control = -1;
    primary_missing = secondary_missing = init_failure = cursor_failure = write_failure = cleanup_failure = false;
    written.clear();
}

int main()
{
    reset();
    if (lcdColumns != 20 || lcdRows != 4)
    {
        assert(setup_lcd() == ESP_ERR_NOT_SUPPORTED && bus_calls == 0 && !s_lcd_ready && logs == 1);
        std::puts("PASS: unsupported S31 LCD geometries are reported explicitly");
        return 0;
    }

    primary_missing = true;
    assert(setup_lcd() == ESP_OK && bus_calls == 2 && s_lcd_addr == lcdI2CAddressSecondary);
    assert(s_lcd_ready && s_lcd == &lcd_object && s_lcd_bus == nullptr && control_calls == 5);
    display_line(1, "hello");
    assert(cursor_calls == 1 && write_calls == 1 && written == "hello               ");
    assert(takes == 1 && gives == 1 && s_lcd_line_cached[1]);
    display_line(1, "hello");
    assert(cursor_calls == 1 && write_calls == 1 && takes == 2 && gives == 2);
    cursor_failure = true;
    display_line(2, "failure");
    assert(!s_lcd_line_cached[2] && gives == takes && logs == 1);
    cursor_failure = false;
    write_failure = true;
    display_line(2, "failure");
    assert(!s_lcd_line_cached[2] && gives == takes && logs == 2);

    reset();
    primary_missing = secondary_missing = true;
    assert(setup_lcd() == ESP_ERR_NOT_FOUND && !s_lcd_ready && bus_calls == 2);
    reset();
    init_failure = true;
    assert(setup_lcd() == ESP_FAIL && !s_lcd_ready && init_calls == 1 && s_lcd_bus == nullptr);
    for (int failure = 0; failure < 5; ++failure)
    {
        reset();
        failed_control = failure;
        cleanup_failure = failure == 4;
        assert(setup_lcd() == ESP_FAIL && !s_lcd_ready && s_lcd == nullptr && cleanup_calls == 1);
        assert(control_calls == failure + 1 && logs == (cleanup_failure ? 1 : 0));
    }
    std::puts("PASS: S31 LCD address fallback, cached writes, failures and cleanup");
}

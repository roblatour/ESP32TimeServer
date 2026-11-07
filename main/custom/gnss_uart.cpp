// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#include "gnss_uart.h"

#include <climits>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "soc/soc_caps.h"

namespace
{
    constexpr uart_port_t GNSS_UART_PORT = UART_NUM_1;
    constexpr int GNSS_RX_BUFFER_SIZE = 256;
    constexpr int GNSS_RX_FIFO_HIGH_BAUD = SOC_UART_FIFO_LEN > 120 ? 120 : SOC_UART_FIFO_LEN - 6;
    constexpr const char *TAG = "gnss_uart";
    static_assert(SOC_UART_FIFO_LEN > 6, "GNSS UART FIFO must hold more than six bytes");
    bool s_driver_owned = false;
    int s_rx_pin = -1;
    int s_peeked_byte = -1;

    TickType_t timeout_ticks(uint32_t timeout_ms)
    {
        if (timeout_ms == UINT32_MAX)
            return portMAX_DELAY;
        const uint64_t ticks = (static_cast<uint64_t>(timeout_ms) * configTICK_RATE_HZ + 999) / 1000;
        return ticks >= portMAX_DELAY ? portMAX_DELAY : static_cast<TickType_t>(ticks);
    }
}

esp_err_t gnss_uart_open(int tx_pin, int rx_pin, uint32_t baud)
{
    if (tx_pin < 0 || rx_pin < 0 || tx_pin == rx_pin || baud == 0 || baud > INT_MAX)
        return ESP_ERR_INVALID_ARG;

    esp_err_t err = gnss_uart_close();
    if (err != ESP_OK)
        return err;
    if (uart_is_driver_installed(GNSS_UART_PORT))
        return ESP_ERR_INVALID_STATE;

    err = uart_driver_install(GNSS_UART_PORT, GNSS_RX_BUFFER_SIZE, 0, 0, nullptr, 0);
    if (err != ESP_OK)
        return err;
    s_driver_owned = true;

    uart_config_t config{};
    config.baud_rate = static_cast<int>(baud);
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if SOC_UART_SUPPORT_XTAL_CLK
    config.source_clk = UART_SCLK_XTAL;
#else
    config.source_clk = UART_SCLK_DEFAULT;
#endif

    err = uart_param_config(GNSS_UART_PORT, &config);
    if (err == ESP_OK)
        err = uart_set_pin(GNSS_UART_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err == ESP_OK)
    {
        s_rx_pin = rx_pin;
        err = gpio_set_pull_mode(static_cast<gpio_num_t>(rx_pin), GPIO_PULLUP_ONLY);
    }
    if (err == ESP_OK)
        err = uart_set_rx_timeout(GNSS_UART_PORT, 1);
    if (err == ESP_OK)
        err = uart_set_rx_full_threshold(GNSS_UART_PORT, baud > 57600 ? GNSS_RX_FIFO_HIGH_BAUD : 1);
    if (err != ESP_OK)
    {
        const esp_err_t cleanup_err = gnss_uart_close();
        if (cleanup_err != ESP_OK)
            ESP_LOGE(TAG, "Failed to clean up GNSS UART after setup error: %s", esp_err_to_name(cleanup_err));
        return err;
    }

    s_peeked_byte = -1;
    return ESP_OK;
}

esp_err_t gnss_uart_close()
{
    if (s_driver_owned)
    {
        const esp_err_t err = uart_driver_delete(GNSS_UART_PORT);
        if (err != ESP_OK)
            return err;
        s_driver_owned = false;
        s_peeked_byte = -1;
    }
    if (s_rx_pin >= 0)
    {
        const esp_err_t err = gpio_set_pull_mode(static_cast<gpio_num_t>(s_rx_pin), GPIO_FLOATING);
        if (err != ESP_OK)
            return err;
        s_rx_pin = -1;
    }
    return ESP_OK;
}

esp_err_t gnss_uart_available(size_t *available)
{
    if (available == nullptr)
        return ESP_ERR_INVALID_ARG;
    *available = 0;
    if (!s_driver_owned)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = uart_get_buffered_data_len(GNSS_UART_PORT, available);
    if (err == ESP_OK && s_peeked_byte >= 0)
        ++*available;
    return err;
}

esp_err_t gnss_uart_peek(uint8_t *value, bool *has_data)
{
    if (value == nullptr || has_data == nullptr)
        return ESP_ERR_INVALID_ARG;
    *has_data = false;
    if (!s_driver_owned)
        return ESP_ERR_INVALID_STATE;

    if (s_peeked_byte < 0)
    {
        uint8_t byte = 0;
        const int count = uart_read_bytes(GNSS_UART_PORT, &byte, 1, 0);
        if (count < 0)
            return ESP_FAIL;
        if (count == 0)
            return ESP_OK;
        s_peeked_byte = byte;
    }

    *value = static_cast<uint8_t>(s_peeked_byte);
    *has_data = true;
    return ESP_OK;
}

esp_err_t gnss_uart_read(uint8_t *buffer, size_t length, uint32_t timeout_ms, size_t *bytes_read)
{
    if (bytes_read == nullptr || (buffer == nullptr && length != 0) || length > INT_MAX)
        return ESP_ERR_INVALID_ARG;
    *bytes_read = 0;
    if (!s_driver_owned)
        return ESP_ERR_INVALID_STATE;
    if (length == 0)
        return ESP_OK;

    if (s_peeked_byte >= 0)
    {
        buffer[0] = static_cast<uint8_t>(s_peeked_byte);
        s_peeked_byte = -1;
        *bytes_read = 1;
    }

    if (*bytes_read < length)
    {
        const int count = uart_read_bytes(GNSS_UART_PORT, buffer + *bytes_read,
                                          static_cast<uint32_t>(length - *bytes_read), timeout_ticks(timeout_ms));
        if (count < 0)
            return ESP_FAIL;
        *bytes_read += static_cast<size_t>(count);
    }
    return ESP_OK;
}

esp_err_t gnss_uart_write(const uint8_t *buffer, size_t length, size_t *bytes_written)
{
    if (bytes_written == nullptr || (buffer == nullptr && length != 0) || length > INT_MAX)
        return ESP_ERR_INVALID_ARG;
    *bytes_written = 0;
    if (!s_driver_owned)
        return ESP_ERR_INVALID_STATE;
    if (length == 0)
        return ESP_OK;

    const int count = uart_write_bytes(GNSS_UART_PORT, buffer, length);
    if (count < 0)
        return ESP_FAIL;
    *bytes_written = static_cast<size_t>(count);
    return ESP_OK;
}

esp_err_t gnss_uart_flush_tx(uint32_t timeout_ms)
{
    if (!s_driver_owned)
        return ESP_ERR_INVALID_STATE;
    return uart_wait_tx_done(GNSS_UART_PORT, timeout_ticks(timeout_ms));
}

// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

// The GNSS setup, sync, and recovery tasks must transfer ownership sequentially.
esp_err_t gnss_uart_open(int tx_pin, int rx_pin, uint32_t baud);
esp_err_t gnss_uart_close();
esp_err_t gnss_uart_available(size_t *available);
esp_err_t gnss_uart_peek(uint8_t *value, bool *has_data);
esp_err_t gnss_uart_read(uint8_t *buffer, size_t length, uint32_t timeout_ms, size_t *bytes_read);
esp_err_t gnss_uart_write(const uint8_t *buffer, size_t length, size_t *bytes_written);
esp_err_t gnss_uart_flush_tx(uint32_t timeout_ms);

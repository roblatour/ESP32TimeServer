#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

enum uart_port_t
{
    UART_NUM_0,
    UART_NUM_1,
};

constexpr int UART_DATA_8_BITS = 8;
constexpr int UART_PARITY_DISABLE = 0;
constexpr int UART_STOP_BITS_1 = 1;
constexpr int UART_HW_FLOWCTRL_DISABLE = 0;
constexpr int UART_SCLK_DEFAULT = 0;
constexpr int UART_SCLK_XTAL = 1;
constexpr int UART_PIN_NO_CHANGE = -1;

struct uart_config_t
{
    int baud_rate = 0;
    int data_bits = 0;
    int parity = 0;
    int stop_bits = 0;
    int flow_ctrl = 0;
    int source_clk = 0;
};

bool uart_is_driver_installed(uart_port_t port);
esp_err_t uart_param_config(uart_port_t port, const uart_config_t *config);
esp_err_t uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts);
esp_err_t uart_driver_install(uart_port_t port, int rx_size, int tx_size,
                              int queue_size, void *queue, int flags);
esp_err_t uart_driver_delete(uart_port_t port);
esp_err_t uart_set_rx_timeout(uart_port_t port, uint8_t timeout);
esp_err_t uart_set_rx_full_threshold(uart_port_t port, int threshold);
esp_err_t uart_get_buffered_data_len(uart_port_t port, size_t *available);
int uart_read_bytes(uart_port_t port, void *buffer, uint32_t length, TickType_t timeout);
int uart_write_bytes(uart_port_t port, const void *buffer, size_t length);
esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t timeout);
esp_err_t uart_set_baudrate(uart_port_t port, uint32_t baud);

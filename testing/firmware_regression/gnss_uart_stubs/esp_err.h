#pragma once

using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_FAIL = -1;
constexpr esp_err_t ESP_ERR_INVALID_ARG = 0x102;
constexpr esp_err_t ESP_ERR_INVALID_STATE = 0x103;
constexpr esp_err_t ESP_ERR_NOT_SUPPORTED = 0x106;
constexpr esp_err_t ESP_ERR_NVS_NO_FREE_PAGES = 0x110d;
constexpr esp_err_t ESP_ERR_NVS_NEW_VERSION_FOUND = 0x1110;

const char *esp_err_to_name(esp_err_t error);

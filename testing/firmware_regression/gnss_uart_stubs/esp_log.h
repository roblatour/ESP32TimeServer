#pragma once

void fixture_gnss_log(const char *tag, const char *format, ...);

#define ESP_LOGE(tag, ...) fixture_gnss_log(tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) fixture_gnss_log(tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) fixture_gnss_log(tag, __VA_ARGS__)

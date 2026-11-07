#pragma once

#include <cstdint>

using TickType_t = uint32_t;
constexpr TickType_t portMAX_DELAY = UINT32_MAX;
constexpr uint32_t configTICK_RATE_HZ = 1000;
#define pdMS_TO_TICKS(ms) (ms)

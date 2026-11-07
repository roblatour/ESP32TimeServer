#pragma once

#include "esp_err.h"

using gpio_num_t = int;
enum gpio_pull_mode_t
{
    GPIO_PULLUP_ONLY,
    GPIO_FLOATING,
};

esp_err_t gpio_set_pull_mode(gpio_num_t pin, gpio_pull_mode_t mode);

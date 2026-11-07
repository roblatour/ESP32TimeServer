#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

#define MQTT_ENABLED 0
#define pdMS_TO_TICKS(ms) (ms)
using esp_err_t = int;
using esp_timer_handle_t = void *;
static constexpr esp_err_t ESP_OK = 0;
struct esp_timer_create_args_t
{
    void (*callback)(void *);
    const char *name;
};
static constexpr const char *TAG = "test";
static esp_err_t s_create_result = ESP_OK, s_start_result = ESP_OK;
static void (*s_callback)(void *) = nullptr;
static uint64_t s_deadline_us = 0;
static unsigned s_start_calls = 0, s_restarts = 0, s_logs = 0;
static int s_timer = 0;
static bool s_armed = false;
template <typename... Args>
static void test_log(const char *, const char *, Args...) { ++s_logs; }
#define ESP_LOGI(...) test_log(__VA_ARGS__)
#define ESP_LOGW(...) test_log(__VA_ARGS__)
#define ESP_LOGE(...) test_log(__VA_ARGS__)
static const char *esp_err_to_name(esp_err_t) { return "injected timer error"; }
static esp_err_t esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *timer)
{
    assert(std::strcmp(args->name, "restart_deadline") == 0);
    s_callback = args->callback;
    *timer = &s_timer;
    return s_create_result;
}
static esp_err_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us)
{
    assert(timer == &s_timer);
    ++s_start_calls;
    s_deadline_us = timeout_us;
    s_armed = s_start_result == ESP_OK;
    return s_start_result;
}
[[maybe_unused]] static void vTaskDelay(uint32_t) { assert(s_armed); }
struct Restart {};
static void esp_restart()
{
    ++s_restarts;
    throw Restart{};
}
#include "restart_under_test.h"

static void request_restart()
{
    try { controlled_restart("test"); }
    catch (const Restart &) { return; }
    assert(false);
}
int main()
{
    request_restart();
    assert(s_armed && s_deadline_us == 5000000ULL && s_start_calls == 1 && s_restarts == 1);
    assert(s_logs == (DEBUG_ENABLED ? 2U : 1U));
    try { s_callback(nullptr); }
    catch (const Restart &) {}
    assert(s_restarts == 2);
    s_create_result = 1;
    s_armed = false;
    request_restart();
    assert(s_start_calls == 1 && s_restarts == 3);
    s_create_result = ESP_OK;
    s_start_result = 1;
    request_restart();
    assert(s_start_calls == 2 && !s_armed && s_restarts == 4);
    std::puts("PASS: five-second restart deadline, independent reset callback, immediate reset on timer failure");
}

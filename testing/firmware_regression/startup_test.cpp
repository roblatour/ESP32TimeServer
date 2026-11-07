#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#define RBG_LED_KY_016_MODULE_ENABLED 0
#define RBG_LED_WS2812_MODULE_ENABLED 1
#define pdMS_TO_TICKS(ms) (ms)
#define TIMING_LOGI(...) do {} while (0)
#define TIMING_LOGW(...) do {} while (0)
#define ESP_LOGI(...) do {} while (0)

static constexpr int lcdColumns = 20;
static constexpr uint32_t gnss_Startup_Qualification_Duration_Ms = 15000;
static constexpr uint32_t gnss_Startup_Qualification_PPS_Edges = 10;
static constexpr int64_t gnss_Startup_Min_PPS_Interval_Us = 800000;
static constexpr int64_t gnss_Startup_Max_PPS_Interval_Us = 1200000;
static std::atomic<bool> s_gnss_pps_startup_qualification_in_progress{false};
struct PpsCaptureEvent
{
    int64_t approximate_edge_us = 0;
};

enum class Scenario { jitter, invalid_gnss, missing_pps, slow_queries };
static Scenario s_scenario;
static int64_t s_now_us;
static int64_t s_last_delivered_edge;
static uint32_t s_queries;
static std::vector<std::string> s_messages;

static int64_t esp_timer_get_time()
{
    assert(s_now_us < 90000000);
    return s_now_us;
}

static void display_line(uint8_t, const char *text)
{
    assert(std::string(text) != "Dwelling for 0 secs");
    s_messages.emplace_back(text);
}

static void vTaskDelay(uint32_t ms)
{
    s_now_us += static_cast<int64_t>(ms) * 1000;
}

static bool current_gnss_timing_is_valid()
{
    ++s_queries;
    s_now_us += (s_queries % 2 == 0) ? 300000 : 10000;
    if (s_scenario == Scenario::slow_queries && s_now_us < 20000000)
        s_now_us += 1500000;
    return s_scenario != Scenario::invalid_gnss ||
           s_now_us < 15500000 || s_now_us >= 20000000;
}

static void clear_pps_events()
{
    s_last_delivered_edge = s_now_us / 1000000 * 1000000;
}

static bool wait_for_pps_capture_event(PpsCaptureEvent *event, uint32_t)
{
    if (s_scenario == Scenario::missing_pps &&
        s_now_us >= 13000000 && s_now_us < 22000000)
        return false;
    const int64_t latest_edge = s_now_us / 1000000 * 1000000;
    if (latest_edge <= s_last_delivered_edge)
        return false;
    event->approximate_edge_us = latest_edge;
    s_last_delivered_edge = latest_edge;
    return true;
}

#include "startup_under_test.h"

int main()
{
    for (Scenario scenario : {Scenario::jitter, Scenario::invalid_gnss,
                              Scenario::missing_pps, Scenario::slow_queries})
    {
        s_scenario = scenario;
        s_now_us = 1000000;
        s_queries = 0;
        s_messages.clear();
        assert(wait_for_gnss_startup_qualification());
        assert(!s_gnss_pps_startup_qualification_in_progress.load());
        assert(s_now_us >= 16000000);
        if (scenario == Scenario::jitter)
            assert(s_now_us < 18000000);
        if (scenario == Scenario::invalid_gnss)
        {
            assert(s_now_us >= 35000000);
            assert(std::find(s_messages.begin(), s_messages.end(), "Timing invalid") != s_messages.end());
            assert(std::find(s_messages.begin(), s_messages.end(), "Checking GNSS") != s_messages.end());
        }
        if (scenario == Scenario::missing_pps)
            assert(s_now_us >= 31000000);
        if (scenario == Scenario::slow_queries)
            assert(s_now_us >= 20000000);
    }
    std::puts("PASS: GNSS startup polling jitter, invalid timing, missing PPS, slow queries and LCD progress");
}

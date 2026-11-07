#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

#define PTP_FREE_RUNNING 1
#define PPS_ISR_LATENCY_AUTO 0
#define DEBUG_ENABLED 0
#define TIMING_LOGI(...) do {} while (0)
#define TIMING_LOGW(...) do { ++warnings; } while (0)
#define TIMING_LOGE(...) do { ++warnings; } while (0)
#define ESP_ERROR_CHECK(expr) do { if ((expr) != ESP_OK) throw check_failure{}; } while (0)

using esp_err_t = int;
using gpio_num_t = int;
struct check_failure {};
static constexpr int ESP_OK = 0;
static constexpr int ESP_ERR_INVALID_STATE = 1;
static constexpr int CHIP_ESP32P4 = 4;
static constexpr int CHIP_ESP32S31 = 31;
static constexpr int MCPWM_CAPTURE_CLK_SRC_DEFAULT = 0;
static constexpr uint32_t PPS_CAPTURE_RESOLUTION_HZ = 80000000;
static constexpr int PPSPin = 33;
static int PTPPPSLoopbackPin = 54;
struct esp_chip_info_t { int model, revision; };
struct esp_eth_mac_t {};
struct Channel { bool registered = false; bool enabled = false; };
using mcpwm_cap_channel_handle_t = Channel *;
using mcpwm_cap_timer_handle_t = int *;
struct mcpwm_capture_timer_config_t
{
    int group_id = 0;
    int clk_src = 0;
    uint32_t resolution_hz = 0;
};
struct mcpwm_capture_channel_config_t
{
    int gpio_num = 0;
    int prescale = 0;
    struct { bool pos_edge = false; } flags;
};
struct mcpwm_capture_event_callbacks_t { void (*on_cap)() = nullptr; };
static void pps_capture_callback() {}
static void ptp_pps_capture_callback() {}
static void setup_ptp_pps_capture();

static mcpwm_cap_timer_handle_t s_pps_capture_timer;
static mcpwm_cap_channel_handle_t s_pps_capture_channel;
static mcpwm_cap_channel_handle_t s_ptp_pps_capture_channel;
static uint32_t s_pps_capture_hz;
static bool s_ptp_pps_capture_enabled;
static bool s_ptp_pps_output_configured;
static std::atomic<bool> s_ptp_loopback_active{false};
static Channel channels[2];
static int timer;
static int core, interrupt_core, model, revision, allocations, registrations, enables, deletions, warnings;
static int output_calls;
static uint32_t capture_resolution;
static bool reserved_output, pad_input, fail_allocation, fail_registration, fail_enable, fail_frequency;

static void esp_chip_info(esp_chip_info_t *info) { info->model = model; info->revision = revision; }
static esp_err_t mcpwm_new_capture_timer(const mcpwm_capture_timer_config_t *, mcpwm_cap_timer_handle_t *out)
{
    *out = &timer;
    return ESP_OK;
}
static esp_err_t mcpwm_capture_timer_get_resolution(mcpwm_cap_timer_handle_t, uint32_t *out)
{
    *out = capture_resolution;
    return ESP_OK;
}
static esp_err_t mcpwm_capture_timer_enable(mcpwm_cap_timer_handle_t) { return ESP_OK; }
static esp_err_t mcpwm_capture_timer_start(mcpwm_cap_timer_handle_t) { return ESP_OK; }
static esp_err_t mcpwm_new_capture_channel(mcpwm_cap_timer_handle_t, const mcpwm_capture_channel_config_t *config,
                                         mcpwm_cap_channel_handle_t *out)
{
    assert(allocations < 2);
    assert(config->prescale == 1 && config->flags.pos_edge);
    if (allocations == 1 && fail_allocation)
        return ESP_ERR_INVALID_STATE;
    *out = &channels[allocations++];
    pad_input = true;
    return ESP_OK;
}
static esp_err_t mcpwm_capture_channel_register_event_callbacks(mcpwm_cap_channel_handle_t channel,
                                                               const mcpwm_capture_event_callbacks_t *, void *)
{
    ++registrations;
    if (interrupt_core < 0)
        interrupt_core = core;
    if (core != interrupt_core || (channel == &channels[1] && fail_registration))
        return ESP_ERR_INVALID_STATE;
    channel->registered = true;
    return ESP_OK;
}
static esp_err_t mcpwm_capture_channel_enable(mcpwm_cap_channel_handle_t channel)
{
    assert(channel->registered && !channel->enabled);
    ++enables;
    if (channel == &channels[1] && fail_enable)
        return ESP_ERR_INVALID_STATE;
    channel->enabled = true;
    return ESP_OK;
}
static esp_err_t mcpwm_del_capture_channel(mcpwm_cap_channel_handle_t channel)
{
    assert(!channel->enabled);
    ++deletions;
    return ESP_OK;
}
static esp_err_t esp_eth_mac_set_pps_out_gpio(esp_eth_mac_t *, int)
{
    ++output_calls;
    if (reserved_output)
        return ESP_ERR_INVALID_STATE;
    reserved_output = true;
    pad_input = false;
    return ESP_OK;
}
static esp_err_t esp_eth_mac_set_pps_out_freq(esp_eth_mac_t *, int frequency)
{
    assert(frequency == 0);
    return fail_frequency ? ESP_ERR_INVALID_STATE : ESP_OK;
}
static esp_err_t gpio_input_enable(gpio_num_t) { pad_input = true; return ESP_OK; }

#include "pps_capture_under_test.h"

static void reset()
{
    for (auto &channel : channels)
        channel = {};
    s_pps_capture_timer = nullptr;
    s_pps_capture_channel = s_ptp_pps_capture_channel = nullptr;
    s_ptp_pps_capture_enabled = s_ptp_pps_output_configured = false;
    s_ptp_loopback_active.store(false);
    core = allocations = registrations = enables = deletions = warnings = output_calls = 0;
    interrupt_core = -1;
#if CONFIG_IDF_TARGET_ESP32P4
    model = CHIP_ESP32P4;
    revision = 300;
#else
    model = CHIP_ESP32S31;
    revision = 0;
#endif
    capture_resolution = PPS_CAPTURE_RESOLUTION_HZ;
    PTPPPSLoopbackPin = 54;
    reserved_output = pad_input = fail_allocation = fail_registration = fail_enable = fail_frequency = false;
}

int main()
{
    esp_eth_mac_t mac;
    reset();
    capture_resolution = 0;
    try
    {
        setup_pps_input();
        assert(false && "zero capture resolution must fail");
    }
    catch (const check_failure &)
    {
        assert(allocations == 0 && s_pps_capture_hz == 0);
    }

    reset();
#if CONFIG_IDF_TARGET_ESP32P4
    assert(!ptp_pps_output_available({CHIP_ESP32P4, 299}));
    assert(!ptp_pps_output_available({CHIP_ESP32S31, 0}));
    if (CONFIG_ESP32P4_REV_MIN_FULL < 300)
    {
        assert(!ptp_pps_output_available({CHIP_ESP32P4, 300}));
        setup_pps_input();
        assert(s_ptp_pps_capture_channel == nullptr && warnings == 1);
        std::puts("PASS: P4 pre-v3 build cannot activate loopback on a newer chip");
        return 0;
    }
    assert(ptp_pps_output_available({CHIP_ESP32P4, 300}));
#else
    assert(ptp_pps_output_available({CHIP_ESP32S31, 0}));
    assert(ptp_pps_output_available({CHIP_ESP32S31, 299}));
    assert(!ptp_pps_output_available({CHIP_ESP32P4, 300}));
#endif
    setup_pps_input();
    assert(registrations == 2 && interrupt_core == 0 && !channels[1].enabled);
    core = 1;
    setup_ptp_pps_loopback(&mac);
    assert(s_ptp_loopback_active.load() && pad_input && enables == 2 && warnings == 0);
    setup_ptp_pps_loopback(&mac);
    assert(output_calls == 1 && registrations == 2 && enables == 2 && warnings == 0);
    fail_frequency = true;
    setup_ptp_pps_loopback(&mac);
    assert(!s_ptp_loopback_active.load() && warnings == 1);
    fail_frequency = false;
    setup_ptp_pps_loopback(&mac);
    assert(s_ptp_loopback_active.load() && output_calls == 1 && enables == 2);

    for (int mode = 0; mode < 5; ++mode)
    {
        reset();
        if (mode == 0) PTPPPSLoopbackPin = -1;
        if (mode == 1)
        {
#if CONFIG_IDF_TARGET_ESP32P4
            revision = 299;
#else
            model = CHIP_ESP32P4;
#endif
        }
        if (mode == 2) PTPPPSLoopbackPin = PPSPin;
        if (mode == 3) fail_registration = true;
        if (mode == 4) fail_allocation = true;
        setup_pps_input();
        core = 1;
        setup_ptp_pps_loopback(&mac);
        assert(channels[0].enabled && !s_ptp_loopback_active.load());
        assert(s_ptp_pps_capture_channel == nullptr && output_calls == 0);
        assert(warnings == (mode == 0 ? 0 : 1));
        assert(deletions == (mode == 3 ? 1 : 0));
    }

    reset();
    setup_pps_input();
    reserved_output = true;
    setup_ptp_pps_loopback(&mac);
    assert(!s_ptp_loopback_active.load() && warnings == 1);
    reserved_output = false;
    setup_ptp_pps_loopback(&mac);
    assert(s_ptp_loopback_active.load());

    for (bool frequency_failure : {false, true})
    {
        reset();
        setup_pps_input();
        core = 1;
        fail_enable = !frequency_failure;
        fail_frequency = frequency_failure;
        setup_ptp_pps_loopback(&mac);
        assert(!s_ptp_loopback_active.load() && warnings == 1);
        fail_enable = fail_frequency = false;
        setup_ptp_pps_loopback(&mac);
        assert(s_ptp_loopback_active.load() && pad_input && output_calls == 1);
    }
    std::puts("PASS: PPS model/profile gating, zero-resolution rejection, same-core registration and recovery");
}

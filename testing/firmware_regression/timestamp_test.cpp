#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include "freertos/FreeRTOS.h"
#include "leap_fixture.h"

static int64_t esp_timer_get_time() { return 1000000; }

#define NTP_HW_TX_TIMESTAMPS 1
using esp_err_t = int;
using esp_eth_handle_t = void *;
struct eth_mac_time_t { uint32_t seconds; uint32_t nanoseconds; };
static constexpr int ESP_OK = 0, ESP_ERR_NO_MEM = 1, ESP_ERR_INVALID_STATE = 2, ESP_ERR_TIMEOUT = 3;
static constexpr int CLOCK_PTP_SYSTEM = 42;
static constexpr uint64_t NTP_EPOCH_OFFSET = 2208988800ULL;
static constexpr const char *TAG = "test";
static portMUX_TYPE s_clock_publish_mux = portMUX_INITIALIZER_UNLOCKED;
static std::atomic<bool> s_ptp_clock_ready{true}, s_ptp_comp_valid{true};
static std::atomic<int32_t> s_clock_ppb{0};
static std::atomic<uint32_t> s_ptp_comp_seq{0}, s_ptpmap_seq{0};
static std::atomic<int64_t> s_ptp_anchor_ns{0}, s_ptp_offset_ns{0};
static std::atomic<int64_t> s_ptpmap_ref_raw_ns{0}, s_ptpmap_ref_offset_ns{0};
static std::atomic<int64_t> s_ptpmap_rate_old_mppb{0}, s_ptpmap_rate_new_mppb{0};
static constexpr int64_t ptpmap_max_age_ns = 3600000000000LL, ptpmap_max_rate_mppb = 500000000LL;
static std::atomic<uint32_t> s_hw_tx_timestamp_failures{0};
static std::atomic<bool> s_hw_tx_timestamp_recovery_requested{false};
static unsigned s_logs = 0, s_hw_transmits = 0, s_plain_transmits = 0;
template <typename... Args>
static void test_log(const char *, const char *, Args...) { ++s_logs; }
#define ESP_LOGI(...) test_log(__VA_ARGS__)
#define ESP_LOGW(...) test_log(__VA_ARGS__)
#define ESP_LOGE(...) test_log(__VA_ARGS__)

static eth_mac_time_t s_timestamp{1700000000U, 500000000U};
static int s_read_error = 0;
static int64_t s_timestamp_age_ns = 10000;
static esp_err_t s_transmit_result = ESP_OK;
static int test_clock_gettime(int clock, timespec *now)
{
    assert(clock == CLOCK_PTP_SYSTEM);
    const int64_t ns = static_cast<int64_t>(s_timestamp.seconds) * 1000000000LL +
                       s_timestamp.nanoseconds + s_timestamp_age_ns;
    now->tv_sec = static_cast<time_t>(ns / 1000000000LL);
    now->tv_nsec = static_cast<long>(ns % 1000000000LL);
    return s_read_error;
}
#define clock_gettime test_clock_gettime
static esp_err_t esp_eth_transmit_ctrl_vargs(esp_eth_handle_t, void *ctrl, uint32_t argc, ...)
{
    assert(argc == 2);
    ++s_hw_transmits;
    *static_cast<eth_mac_time_t *>(ctrl) = s_timestamp;
    return s_transmit_result;
}
[[maybe_unused]] static esp_err_t esp_eth_transmit(esp_eth_handle_t, uint8_t *, size_t)
{
    ++s_plain_transmits;
    return ESP_OK;
}

#include "timestamp_under_test.h"

int main()
{
    set_test_leap(esp_timer_get_time());
    ptp_comp_publish(1700000000000000000LL, 1000);
    ptp_map_publish(1700000000500000000LL, 1000, 0, 0);
    int64_t anchor = 0, offset = 0;
    assert(ptp_comp_read(anchor, offset) && offset == 1000);
    assert(ptp_map_offset_at(1700000000500000000LL, offset) && offset == 1000);
    const uint64_t expected = ntp64_from_unix_ns(1699999963499999000LL);
    assert(ntp64_from_hardware_timestamp(s_timestamp) == expected);
    uint8_t ptp[10]{};
    assert(ptp_write_timestamp(ptp, s_timestamp));
    uint64_t ptp_seconds = 0;
    uint32_t ptp_nanoseconds = 0;
    for (unsigned i = 0; i < 6; ++i)
        ptp_seconds = (ptp_seconds << 8) | ptp[i];
    for (unsigned i = 6; i < 10; ++i)
        ptp_nanoseconds = (ptp_nanoseconds << 8) | ptp[i];
    assert(ptp_seconds == 1700000000ULL && ptp_nanoseconds == 499999000U);
    s_ptp_comp_valid = false;
    assert(ntp64_from_hardware_timestamp(s_timestamp) == 0);
    assert(!ptp_write_timestamp(ptp, s_timestamp));
    s_ptp_comp_valid = true;
    s_ptpmap_seq = 1;
    s_ptp_comp_seq = 1;
    assert(ntp64_from_hardware_timestamp(s_timestamp) == 0);
    assert(!ptp_write_timestamp(ptp, s_timestamp));
    s_ptpmap_seq = 2;
    s_ptp_comp_seq = 2;
    assert(!timestamp_is_valid(nullptr));
    const eth_mac_time_t empty{};
    assert(!timestamp_is_valid(&empty));
    const eth_mac_time_t invalid{1700000000U, 1000000000U};
    assert(!timestamp_is_valid(&invalid));
    assert(ntp64_from_hardware_timestamp(invalid) == 0);
    assert(!ptp_write_timestamp(ptp, invalid));

    uint8_t frame[48]{};
    uint64_t tx = 0;
    assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_OK && tx == expected);
    const eth_mac_time_t valid = s_timestamp;
    s_timestamp = {};
    for (int i = 0; i < 5; ++i)
    {
        tx = 123;
        assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_OK && tx == 0);
    }
    assert(s_hw_tx_timestamp_failures == 5 && s_hw_tx_timestamp_recovery_requested);
    s_timestamp = valid;
    assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_OK && tx == expected);
    assert(s_hw_tx_timestamp_failures == 0 && !s_hw_tx_timestamp_recovery_requested);

    s_timestamp_age_ns = 100000000;
    assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_OK && tx == 0);
    assert(s_hw_tx_timestamp_failures == 1);
    s_timestamp_age_ns = -2000000;
    assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_OK && tx == 0);
    s_timestamp_age_ns = 10000;
    s_read_error = -1;
    assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_OK && tx == 0);
    s_read_error = 0;
    s_transmit_result = ESP_ERR_NO_MEM;
    const uint32_t failures = s_hw_tx_timestamp_failures;
    assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_ERR_NO_MEM);
    assert(s_hw_tx_timestamp_failures == failures);
    s_transmit_result = ESP_ERR_TIMEOUT;
    for (int i = 0; i < 5; ++i)
        assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_ERR_TIMEOUT);
    assert(s_hw_tx_timestamp_recovery_requested);
    s_transmit_result = ESP_OK;
    assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_OK && tx == expected);
    assert(!s_hw_tx_timestamp_recovery_requested);
    s_ptp_comp_valid = false;
    assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_OK && tx == 0);
    s_ptp_comp_valid = true;
    s_ptp_clock_ready = false;
    const unsigned calls = s_hw_transmits;
    assert(transmit_with_hw_timestamp(nullptr, frame, sizeof(frame), &tx) == ESP_ERR_INVALID_STATE);
    assert(s_hw_transmits == calls);
    assert(s_plain_transmits == 0);
    assert(s_logs == (DEBUG_ENABLED ? 2U : 0U));
    for (const int8_t change : {int8_t{1}, int8_t{-1}})
    {
        const timescale::LeapSeconds leap{true, 37, change, 1893456000, esp_timer_get_time()};
        s_leap_state.publish(leap);
        const int64_t boundary_ns = leap.event_tai() * 1000000000LL;
        ptp_comp_publish(boundary_ns - 3000000000LL, 0);
        ptp_map_publish(boundary_ns - 3000000000LL, 0, 0, 0);
        for (const int64_t delta : {-2000000000LL, -1000000000LL, -1LL, 0LL, 1LL, 1000000000LL})
        {
            const int64_t tai_ns = boundary_ns + delta;
            const eth_mac_time_t stamp{static_cast<uint32_t>(tai_ns / 1000000000LL),
                                       static_cast<uint32_t>(tai_ns % 1000000000LL)};
            assert(ntp64_from_hardware_timestamp(stamp) == ntp64_from_unix_ns(leap.to_utc_ns(tai_ns)));
            assert(ptp_write_timestamp(ptp, stamp));
            uint64_t seconds = 0;
            uint32_t nanos = 0;
            for (unsigned i = 0; i < 6; ++i)
                seconds = (seconds << 8) | ptp[i];
            for (unsigned i = 6; i < 10; ++i)
                nanos = (nanos << 8) | ptp[i];
            assert(seconds == stamp.seconds && nanos == stamp.nanoseconds);
        }
    }
    s_leap_state.publish({});
    assert(ntp64_from_hardware_timestamp(s_timestamp) == 0);
    assert(!ptp_write_timestamp(ptp, s_timestamp));
    std::puts("PASS: hardware timestamps, missing/stale timestamps, recovery signaling, no software downgrade");
}

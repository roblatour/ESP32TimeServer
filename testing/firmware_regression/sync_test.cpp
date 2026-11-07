#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>
#include "gnss_receiver.h"
#include "leap_fixture.h"

#define pdMS_TO_TICKS(ms) (ms)
template <typename... Args>
static void test_log(Args...) {}
static constexpr const char *TAG = "sync-test";
#define TIMING_LOGI(...) test_log(__VA_ARGS__)
#define TIMING_LOGW(...) test_log(__VA_ARGS__)
#define TIMING_LOGE(...) test_log(__VA_ARGS__)
#define ESP_LOGE(...) do {} while (0)

static std::vector<std::string> s_gnss_info_logs, s_gnss_warning_logs;
template <typename... Args>
static void record_gnss_log(bool warning, const char *, const char *format, Args... args)
{
    char message[192];
    const int length = std::snprintf(message, sizeof(message), format, args...);
    assert(length >= 0 && static_cast<size_t>(length) < sizeof(message));
    (warning ? s_gnss_warning_logs : s_gnss_info_logs).emplace_back(message);
}
#define ESP_LOGI(...) record_gnss_log(false, __VA_ARGS__)
#define ESP_LOGW(...) record_gnss_log(true, __VA_ARGS__)
#define UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED 1
static constexpr uint8_t DYN_MODEL_STATIONARY = 2, DYN_MODEL_UNKNOWN = 255;

namespace gnss
{
    const char *status_name(Status) { return "mock status"; }
}

struct sync_faults_t
{
    bool pps_missing = false, gnss_invalid = false;
};
struct sync_candidate_t
{
    time_t candidate_time = 0;
    int64_t pps_release_time_us = 0;
    bool use_pps_alignment = false, used_nmea_fallback = false;
    sync_faults_t failures{};
};
struct PpsCaptureEvent { int64_t approximate_edge_us = 0; };
struct nmea_rmc_time_t { int year, month, day, hour, minute, second; };
static gnss::TimeSolution make_pvt()
{
    gnss::TimeSolution pvt{};
    pvt.year = 2026;
    pvt.month = 10;
    pvt.day = 20;
    pvt.hour = 17;
    pvt.fix_type = 3;
    pvt.fix_ok = pvt.date_valid = pvt.time_valid = pvt.fully_resolved = true;
    return pvt;
}

static gnss::TimeSolution s_gnss_time_solution{};

static int64_t s_now_us = 1000000;
static bool s_use_nmea_fallback = false;
static std::atomic<bool> s_time_has_been_set{false};
static constexpr int64_t Max_Sync_Attempt_Us = 10000000;
static bool s_have_pps = true, s_poll_ok = true, s_late = false;
static bool s_slow_receiver = false;
static bool s_nmea_valid = true, s_nmea_late = false;
static bool s_leap_poll_ok = true;
static int s_nmea_year = 2026, s_nmea_month = 10, s_nmea_day = 20;
static int s_nmea_second_override = -1;
static int s_fraction_case = 0;
static int s_calendar_case = 0;
static int64_t esp_timer_get_time() { return s_now_us; }
static void display_line(uint8_t, const char *) {}
static void clear_pps_events() {}
static bool wait_for_pps_capture_event(PpsCaptureEvent *event, uint32_t)
{
    s_now_us = (s_now_us / 1000000 + 1) * 1000000;
    event->approximate_edge_us = s_now_us;
    return s_have_pps;
}
static bool wait_for_nmea_rmc_time(nmea_rmc_time_t *data, uint32_t timeout_ms)
{
    if (!s_nmea_valid)
    {
        s_now_us += static_cast<int64_t>(timeout_ms) * 1000;
        return false;
    }
    *data = {s_nmea_year, s_nmea_month, s_nmea_day, 17,
             static_cast<int>((s_now_us / 60000000) % 60),
             s_nmea_second_override >= 0 ? s_nmea_second_override : static_cast<int>((s_now_us / 1000000) % 60)};
    s_now_us += s_nmea_late ? 1000000 : 100000;
    return true;
}
static bool s_uart_read_failed = false;
static bool read_gnss_uart_byte(uint8_t *, bool *has_data)
{
    *has_data = false;
    return !s_uart_read_failed;
}

static struct
{
    gnss::TimeSolution packet = make_pvt();
    gnss::LeapInformation leap_packet{0, 0, 3, 2, 2, 18, 0, 0};
    bool automatic = false;
    gnss::Status leap_information(gnss::LeapInformation &data, uint32_t)
    {
        if (!s_leap_poll_ok)
            return gnss::Status::timeout;
        leap_packet.itow = packet.itow;
        data = leap_packet;
        return gnss::Status::ok;
    }
    bool pending = true, flushed = false;
    unsigned polls = 0, drains = 0;
    bool stationary_ack = true;
    uint8_t dynamic_model = DYN_MODEL_STATIONARY;
    unsigned model_writes = 0, model_reads = 0;
    gnss::Status set_stationary_model(uint32_t max_wait_ms)
    {
        assert(max_wait_ms == 1500);
        ++model_writes;
        return stationary_ack ? gnss::Status::ok : gnss::Status::timeout;
    }
    gnss::Status read_dynamic_model(uint8_t &model, uint32_t max_wait_ms)
    {
        assert(model_writes == 1 && max_wait_ms == 1500);
        ++model_reads;
        if (dynamic_model == DYN_MODEL_UNKNOWN)
            return gnss::Status::timeout;
        model = dynamic_model;
        return gnss::Status::ok;
    }
    int64_t ready_us = 0;
    gnss::Status discard_pending_pvt()
    {
        assert(s_now_us % 1000000 < 1000); // Draining after the 300 ms delay would discard the current solution.
        pending = false;
        ++drains;
        flushed = true;
        ready_us = s_now_us + (s_slow_receiver ? 1100000 : 100000);
        return gnss::Status::ok;
    }
    gnss::Status next_pvt(gnss::TimeSolution &solution, uint32_t max_wait_ms)
    {
        if (!s_poll_ok)
        {
            s_now_us += static_cast<int64_t>(max_wait_ms) * 1000;
            return gnss::Status::timeout;
        }
        if (automatic && s_now_us < ready_us)
        {
            const int64_t wait_us = ready_us - s_now_us;
            if (wait_us > static_cast<int64_t>(max_wait_ms) * 1000)
            {
                s_now_us += static_cast<int64_t>(max_wait_ms) * 1000;
                return gnss::Status::timeout;
            }
            s_now_us += wait_us;
        }
        assert(!pending && flushed);
        flushed = false;
        ++polls;
        packet.second = static_cast<uint8_t>(s_now_us / 1000000);
        packet.itow = static_cast<uint32_t>(s_now_us / 1000);
        packet.nano = 0;
        if (s_fraction_case == 1)
        {
            --packet.second;
            packet.nano = 999999990;
        }
        else if (s_fraction_case == 2)
        {
            ++packet.second;
            packet.nano = -999999990;
        }
        if (s_calendar_case != 0)
        {
            packet.year = 2026;
            packet.month = 12;
            packet.day = 31;
            packet.hour = 23;
            packet.minute = 59;
            packet.second = s_calendar_case == 3 ? 58 : s_calendar_case == 4 || s_calendar_case == 6 ? 59 : 60;
            if (s_calendar_case == 2 || s_calendar_case == 7)
            {
                packet.year = 2027;
                packet.month = packet.day = 1;
                packet.hour = packet.minute = packet.second = 0;
            }
            if (s_calendar_case == 6)
                packet.nano = 999999990;
            if (s_calendar_case == 7)
                packet.nano = -999999990;
        }
        pending = true;
        if (s_slow_receiver)
        {
            assert(max_wait_ms >= 1100);
            if (!automatic)
                s_now_us += 1100000;
            solution = packet;
            return gnss::Status::ok;
        }
        s_now_us += s_late ? 1000000 : 5000;
        solution = packet;
        return gnss::Status::ok;
    }
} s_gnss;

#include "sync_under_test.h"

static void configure_stationary_mode_under_test(bool &first_sync)
{
#include "stationary_mode_under_test.h"
}

static void test_stationary_mode_readback()
{
    for (const bool acknowledged : {false, true})
    {
        for (const uint8_t model : {DYN_MODEL_STATIONARY, uint8_t{0}, DYN_MODEL_UNKNOWN})
        {
            s_gnss.stationary_ack = acknowledged;
            s_gnss.dynamic_model = model;
            s_gnss.model_writes = s_gnss.model_reads = 0;
            s_gnss_info_logs.clear();
            s_gnss_warning_logs.clear();

            bool first_sync = false;
            configure_stationary_mode_under_test(first_sync);
            assert(s_gnss.model_writes == 0 && s_gnss.model_reads == 0);
            assert(s_gnss_info_logs.empty() && s_gnss_warning_logs.empty());
            first_sync = true;
            configure_stationary_mode_under_test(first_sync);
            assert(s_gnss.model_writes == 1 && s_gnss.model_reads == 1);
            assert(!first_sync);
            configure_stationary_mode_under_test(first_sync);
            assert(s_gnss.model_writes == 1 && s_gnss.model_reads == 1);
            assert(s_gnss_info_logs.size() == (model == DYN_MODEL_STATIONARY ? 1U : 0U));
            assert(s_gnss_warning_logs.size() ==
                   (acknowledged ? 0U : 1U) + (model == DYN_MODEL_STATIONARY ? 0U : 1U));
            if (!acknowledged)
                assert(s_gnss_warning_logs.front().find("did not acknowledge") != std::string::npos);
            if (model == DYN_MODEL_STATIONARY)
                assert(s_gnss_info_logs.front().find("confirmed by RAM readback (dynamic model=2)") != std::string::npos);
            else if (model == DYN_MODEL_UNKNOWN)
                assert(s_gnss_warning_logs.back().find("readback failed") != std::string::npos);
            else
                assert(s_gnss_warning_logs.back().find("dynamic model=0 (expected 2)") != std::string::npos);
        }
    }
    std::puts("PASS: stationary-mode RAM readback, ACK-without-change, missing ACK, read failure and first-sync gating");
}

static void test_optional_timels()
{
    s_leap_state.publish({});
    gnss::TimeSolution pvt = make_pvt();
    pvt.itow = 1000;
    auto refresh = [&]() {
        s_now_us += 31000000;
        s_gnss.packet = pvt;
        return refresh_leap_information(pvt);
    };
    s_leap_poll_ok = false;
    assert(refresh() && s_leap_state.read().source == timescale::LeapSource::Bulletin);
    s_now_us += timescale::MAX_AGE_US + 1;
    assert(refresh() && s_leap_state.read().source == timescale::LeapSource::Bulletin);
    pvt.fully_resolved = false;
    assert(!refresh());
    pvt.fully_resolved = true;
    pvt.year = 2027;
    pvt.month = 5;
    pvt.day = 20;
    pvt.hour = pvt.minute = pvt.second = 0;
    s_timescale_requalification_required = false;
    assert(refresh() && s_leap_state.read().source == timescale::LeapSource::ContinuousUtc);
    assert(!s_timescale_requalification_required); // Changing policy source does not step or disqualify the clock.
    s_leap_poll_ok = true;
    auto &data = s_gnss.leap_packet;
    data.valid = 1;
    data.change_source = 0;
    assert(refresh() && s_leap_state.read().source == timescale::LeapSource::ContinuousUtc);
    data.current_leap_seconds = 19;
    assert(!refresh() && !s_leap_state.read().valid && s_timescale_requalification_required);
    s_leap_poll_ok = false;
    assert(!refresh()); // A timeout must not erase the receiver's contradictory offset.
    s_leap_poll_ok = true;
    data.valid = 3;
    data.change_source = 2;
    data.current_leap_seconds = 18;
    assert(refresh());
    data.leap_change = 1;
    data.time_to_event = 86401;
    assert(!refresh()); // A well-formed announcement after the abolition date is a conflict.
    s_leap_poll_ok = false;
    assert(!refresh());
    s_leap_poll_ok = true;
    data.leap_change = 0;
    assert(refresh());
    data.valid = 1;
    data.current_source = 0;
    data.current_leap_seconds = 17;
    assert(refresh() && s_leap_state.read().offset == 37); // Ignore firmware-default offsets.
    std::puts("PASS: optional/incomplete TIMELS, no expiry after continuous UTC, unresolved PVT, sticky offset/event conflicts and recovery");
}

int main()
{
    set_test_leap(s_now_us);
    test_stationary_mode_readback();
    sync_candidate_t first{}, second{};
    assert(acquire_sync_candidate(&first));
    assert(acquire_sync_candidate(&second));
    assert(first.use_pps_alignment && !first.used_nmea_fallback);
    assert(first_sync_candidates_are_plausible(first, second));
    second.candidate_time++;
    assert(!first_sync_candidates_are_plausible(first, second));
    assert(s_gnss.polls == 2 && s_gnss.drains == 2);

    s_fraction_case = 1;
    assert(acquire_sync_candidate(&first));
    s_fraction_case = 0;
    assert(acquire_sync_candidate(&second));
    assert(first_sync_candidates_are_plausible(first, second));
    s_fraction_case = 2;
    assert(acquire_sync_candidate(&first));
    s_fraction_case = 0;
    assert(acquire_sync_candidate(&second));
    assert(first_sync_candidates_are_plausible(first, second));

    s_poll_ok = false;
    assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    s_poll_ok = true;
    assert(acquire_sync_candidate(&first));
    s_late = true;
    assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    s_late = false;
    s_slow_receiver = true;
    assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    s_slow_receiver = false;
    s_gnss.packet.fix_ok = false;
    assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    s_gnss.packet.fix_ok = true;
    s_gnss.automatic = true;
    assert(acquire_sync_candidate(&first));
    assert(acquire_sync_candidate(&second));
    assert(first_sync_candidates_are_plausible(first, second));
    s_slow_receiver = true;
    assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    s_slow_receiver = false;
    s_poll_ok = false;
    const int64_t timeout_start_us = s_now_us;
    assert(!wait_for_gnss_pvt(50));
    assert(s_now_us - timeout_start_us == 50000);
    s_poll_ok = true;
    s_have_pps = false;
    assert(!acquire_sync_candidate(&first) && first.failures.pps_missing);

    s_have_pps = true;
    const timescale::LeapSeconds insertion{true, 37, 1, 1798761600, s_now_us};
    s_leap_state.publish(insertion);
    s_calendar_case = 1;
    assert(acquire_sync_candidate(&first) && first.candidate_time == 1798761637);
    s_calendar_case = 2;
    assert(acquire_sync_candidate(&second) && second.candidate_time == 1798761638);
    assert(first_sync_candidates_are_plausible(first, second));
    for (const int label : {6, 7})
    {
        s_calendar_case = label;
        assert(acquire_sync_candidate(&first) && first.candidate_time == 1798761637);
    }
    const timescale::LeapSeconds deletion{true, 37, -1, 1798761600, s_now_us};
    s_leap_state.publish(deletion);
    s_calendar_case = 3;
    assert(acquire_sync_candidate(&first) && first.candidate_time == 1798761635);
    s_calendar_case = 2;
    assert(acquire_sync_candidate(&second) && second.candidate_time == 1798761636);
    assert(first_sync_candidates_are_plausible(first, second));
    for (const int label : {4, 5})
    {
        s_calendar_case = label;
        assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    }
    s_gnss.packet.fully_resolved = false;
    assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    s_gnss.packet.fully_resolved = true;

    s_use_nmea_fallback = true;
    s_leap_state.publish({});
    const unsigned ubx_calls = s_gnss.polls;
    const unsigned ubx_drains = s_gnss.drains;
    assert(current_gnss_timing_is_valid());
    assert(s_leap_state.read().source == timescale::LeapSource::Bulletin);
    assert(acquire_sync_candidate(&first) && first.used_nmea_fallback);
    assert(acquire_sync_candidate(&second) && first_sync_candidates_are_plausible(first, second));
    assert(first.candidate_time == epoch_from_utc(2026, 10, 20, 17,
                                                static_cast<int>((first.pps_release_time_us / 60000000) % 60),
                                                static_cast<int>((first.pps_release_time_us / 1000000) % 60)) + 37);
    s_uart_read_failed = true;
    assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    s_uart_read_failed = false;
    s_nmea_valid = false;
    assert(!current_gnss_timing_is_valid());
    assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    s_nmea_valid = true;
    s_nmea_late = true;
    assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    s_nmea_late = false;
    s_have_pps = false;
    assert(!acquire_sync_candidate(&first) && first.failures.pps_missing);
    s_have_pps = true;
    s_nmea_year = 2027;
    s_nmea_month = 5;
    s_nmea_day = 20;
    assert(current_gnss_timing_is_valid());
    assert(s_leap_state.read().source == timescale::LeapSource::ContinuousUtc);
    assert(acquire_sync_candidate(&first) && first.used_nmea_fallback);
    s_nmea_second_override = 60;
    assert(!current_gnss_timing_is_valid());
    assert(!acquire_sync_candidate(&first) && first.failures.gnss_invalid);
    s_nmea_second_override = -1;
    assert(acquire_sync_candidate(&first));
    assert(s_gnss.polls == ubx_calls && s_gnss.drains == ubx_drains);
    test_optional_timels();
    std::puts("PASS: UBX and NMEA/PPS TAI acquisition, deadline/PPS failures, bulletin fallback and continuous-UTC qualification");
}

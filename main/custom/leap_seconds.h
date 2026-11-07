// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

#include <atomic>
#include <cstdint>
#include <limits>

namespace timescale
{
static constexpr int64_t SECOND_NS = 1000000000LL;
static constexpr int64_t MAX_AGE_US = 300000000LL;
static constexpr int64_t GPS_WEEK_MS = 604800000LL;
// IANA leap-seconds.list checked 2026-10-23; CGPM 2026 Resolution 5.
static constexpr int16_t BULLETIN_TAI_UTC = 37;
static constexpr int64_t BULLETIN_START_UTC = 1483228800LL; // 2017-01-01
static constexpr int64_t BULLETIN_EXPIRES_UTC = 1814140800LL; // 2027-06-28
static constexpr int64_t CONTINUOUS_UTC_START = 1810771200LL; // 2027-05-20
static_assert(BULLETIN_EXPIRES_UTC > CONTINUOUS_UTC_START,
              "The embedded bulletin must cover the continuous-UTC transition");

enum class LeapSource : uint8_t
{
    Gnss,
    Bulletin,
    ContinuousUtc
};

struct LeapSeconds
{
    bool valid = false;
    int16_t offset = 0; // TAI - UTC before event_utc, or the current offset when change == 0.
    int8_t change = 0;
    int64_t event_utc = 0;
    int64_t updated_us = 0;
    LeapSource source = LeapSource::Gnss;
    int64_t validity_us = MAX_AGE_US;

    bool fresh(int64_t now_us) const
    {
        return valid && now_us >= updated_us && now_us - updated_us <= validity_us;
    }

    int64_t event_tai() const
    {
        return event_utc + offset + change;
    }

    int16_t utc_offset(int64_t tai_seconds) const
    {
        return static_cast<int16_t>(offset + (change != 0 && tai_seconds >= event_tai() ? change : 0));
    }

    uint8_t leap_indicator(int64_t tai_seconds) const
    {
        const int64_t start = event_utc - 86400 + offset;
        return change != 0 && tai_seconds >= start && tai_seconds < event_tai()
                   ? (change > 0 ? 1 : 2) : 0;
    }

    int64_t to_utc_ns(int64_t tai_ns) const
    {
        const int64_t seconds = tai_ns / SECOND_NS;
        int16_t correction = utc_offset(seconds);
        // POSIX/NTP cannot label 23:59:60: repeat 23:59:59, without smearing TAI.
        if (change == 1 && seconds == event_tai() - 1)
            ++correction;
        return tai_ns - static_cast<int64_t>(correction) * SECOND_NS;
    }

    bool from_utc(int64_t utc_seconds, bool second_60, int64_t &tai_seconds) const
    {
        if (!valid)
            return false;
        if (second_60 && (change != 1 || utc_seconds != event_utc))
            return false;
        if (change == -1 && utc_seconds == event_utc - 1)
            return false;
        tai_seconds = utc_seconds + offset +
                      (!second_60 && change != 0 && utc_seconds >= event_utc ? change : 0);
        return true;
    }
};

// All fields are atomic: readers never race the single GNSS-task writer.
class LeapState
{
public:
    void publish(const LeapSeconds &value)
    {
        sequence.fetch_add(1, std::memory_order_acq_rel);
        valid.store(value.valid, std::memory_order_relaxed);
        offset.store(value.offset, std::memory_order_relaxed);
        change.store(value.change, std::memory_order_relaxed);
        event.store(value.event_utc, std::memory_order_relaxed);
        updated.store(value.updated_us, std::memory_order_relaxed);
        source.store(value.source, std::memory_order_relaxed);
        validity.store(value.validity_us, std::memory_order_relaxed);
        sequence.fetch_add(1, std::memory_order_release);
    }

    LeapSeconds read() const
    {
        for (unsigned attempt = 0; attempt < 3; ++attempt)
        {
            const uint32_t before = sequence.load(std::memory_order_acquire);
            if (before & 1U)
                continue;
            LeapSeconds value{valid.load(std::memory_order_relaxed),
                              offset.load(std::memory_order_relaxed),
                              change.load(std::memory_order_relaxed),
                              event.load(std::memory_order_relaxed),
                              updated.load(std::memory_order_relaxed),
                              source.load(std::memory_order_relaxed),
                              validity.load(std::memory_order_relaxed)};
            std::atomic_thread_fence(std::memory_order_acquire);
            if (before == sequence.load(std::memory_order_relaxed))
                return value;
        }
        return {};
    }

private:
    std::atomic<uint32_t> sequence{0};
    std::atomic<bool> valid{false};
    std::atomic<int16_t> offset{0};
    std::atomic<int8_t> change{0};
    std::atomic<int64_t> event{0}, updated{0};
    std::atomic<LeapSource> source{LeapSource::Gnss};
    std::atomic<int64_t> validity{MAX_AGE_US};
};

inline bool bulletin_leap_seconds(int64_t utc, int64_t now_us, LeapSeconds &result)
{
    if (utc < BULLETIN_START_UTC)
        return false;
    if (utc >= CONTINUOUS_UTC_START)
    {
        result = {true, BULLETIN_TAI_UTC, 0, 0, now_us, LeapSource::ContinuousUtc,
                  std::numeric_limits<int64_t>::max()};
        return true;
    }
    if (utc >= BULLETIN_EXPIRES_UTC)
        return false;
    result = {true, BULLETIN_TAI_UTC, 0, 0, now_us, LeapSource::Bulletin,
              (BULLETIN_EXPIRES_UTC - utc) * 1000000LL - 1};
    return true;
}

inline int16_t offset_at_utc(const LeapSeconds &leap, int64_t utc)
{
    return static_cast<int16_t>(leap.offset +
                               (leap.change != 0 && utc >= leap.event_utc ? leap.change : 0));
}

inline bool select_leap_seconds(int64_t utc, int64_t now_us, const LeapSeconds &previous,
                                const LeapSeconds &observed, LeapSeconds &result)
{
    result = observed.valid ? observed : previous;
    if (observed.valid && observed.change != 0 && observed.event_utc >= CONTINUOUS_UTC_START)
        return false;
    LeapSeconds bulletin{};
    const bool have_bulletin = bulletin_leap_seconds(utc, now_us, bulletin);
    if (utc >= CONTINUOUS_UTC_START)
    {
        if (result.offset != 0 &&
            (offset_at_utc(result, utc) != BULLETIN_TAI_UTC ||
             (result.change != 0 && result.event_utc >= CONTINUOUS_UTC_START)))
            return false;
        if (result.offset == 0 || result.change == 0)
            result = bulletin;
        else
        {
            // Retain a compatible historical transition for queued RX/TX timestamps.
            result.valid = true;
            result.updated_us = now_us;
            result.source = LeapSource::ContinuousUtc;
            result.validity_us = std::numeric_limits<int64_t>::max();
        }
        return have_bulletin;
    }
    if (observed.valid)
        return true;
    if (previous.source == LeapSource::Gnss && previous.offset != 0)
    {
        if (previous.fresh(now_us))
            return true;
        // Do not replace an expired, known event or changed offset with a no-event table.
        if (!have_bulletin || offset_at_utc(previous, utc) != bulletin.offset ||
            (previous.change != 0 && utc <= previous.event_utc))
            return false;
    }
    if (!have_bulletin)
        return false;
    result = bulletin;
    return true;
}

inline bool live_leap_source(uint8_t source)
{
    return (source >= 1 && source <= 5) || source == 8;
}

inline bool decode_leap_seconds(uint8_t version, uint8_t validity, uint8_t current_source,
                                uint8_t event_source, int8_t gps_utc, int8_t change,
                                int32_t time_to_event, uint32_t leap_itow, uint32_t pvt_itow,
                                int64_t pvt_utc, bool second_60, int64_t now_us,
                                const LeapSeconds &previous, LeapSeconds &result)
{
    if (version != 0 || (validity & 3U) != 3U || !live_leap_source(current_source) ||
        event_source < 2 || event_source > 7 || gps_utc < 0 || gps_utc > 100 ||
        change < -1 || change > 1 || leap_itow >= GPS_WEEK_MS || pvt_itow >= GPS_WEEK_MS)
        return false;
    int64_t age_ms = static_cast<int64_t>(pvt_itow) - leap_itow;
    if (age_ms > GPS_WEEK_MS / 2)
        age_ms -= GPS_WEEK_MS;
    if (age_ms < -GPS_WEEK_MS / 2)
        age_ms += GPS_WEEK_MS;
    if (age_ms < -2000 || age_ms > 2000 || age_ms % 1000 != 0)
        return false;

    const int16_t current_offset = static_cast<int16_t>(gps_utc + 19); // TAI - GPS is fixed.
    if (previous.fresh(now_us) && previous.source == LeapSource::Gnss && previous.change != 0 &&
        pvt_utc >= previous.event_utc - 2 && pvt_utc <= previous.event_utc + 2 &&
        (current_offset == previous.offset || current_offset == previous.offset + previous.change) &&
        (change == 0 || change == previous.change))
    {
        // A poll and PVT can straddle midnight. Keep the qualified schedule, not a mixed-epoch offset.
        result = previous;
        return true;
    }
    result = {true, current_offset, 0, 0, now_us};
    if (change != 0)
    {
        const int64_t approximate_event = pvt_utc + time_to_event - age_ms / 1000;
        const int64_t midnight = ((approximate_event + 43200) / 86400) * 86400;
        if (approximate_event < midnight - 2 || approximate_event > midnight + 2)
            return false;
        if (age_ms != 0 && time_to_event >= -3 && time_to_event <= 3)
            return false;
        const bool past = time_to_event - age_ms / 1000 <= 0 && !second_60 && pvt_utc >= midnight;
        result.offset = static_cast<int16_t>(current_offset - (past ? change : 0));
        result.change = change;
        result.event_utc = midnight;
    }
    else if (previous.valid && previous.source == LeapSource::Gnss && previous.change != 0 &&
             pvt_utc >= previous.event_utc &&
             current_offset == previous.offset + previous.change)
    {
        // Keep the last transition so queued RX/TX stamps still use their own offset.
        result.offset = previous.offset;
        result.change = previous.change;
        result.event_utc = previous.event_utc;
    }
    return true;
}
}

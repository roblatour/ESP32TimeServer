#include <cassert>
#include <cstdio>
#include <initializer_list>
#include "../../main/custom/leap_seconds.h"

static constexpr int64_t MIDNIGHT = 1893456000LL; // 2030-01-01; synthetic, not a real announcement.
static constexpr int64_t NS = timescale::SECOND_NS;

static void test_transition(int8_t change)
{
    const timescale::LeapSeconds leap{true, 37, change, MIDNIGHT, 1000000};
    const int64_t boundary = leap.event_tai();
    const int64_t last_utc = change > 0 ? MIDNIGHT - 1 : MIDNIGHT - 2;
    assert(leap.utc_offset(boundary - 1) == 37);
    assert(leap.utc_offset(boundary) == 37 + change);
    assert(leap.leap_indicator(MIDNIGHT - 86400 + 36) == 0);
    assert(leap.leap_indicator(MIDNIGHT - 86400 + 37) == (change > 0 ? 1 : 2));
    assert(leap.leap_indicator(boundary - 1) == (change > 0 ? 1 : 2));
    assert(leap.leap_indicator(boundary) == 0);
    assert(leap.to_utc_ns((boundary - 1) * NS + 123456789) == last_utc * NS + 123456789);
    assert(leap.to_utc_ns(boundary * NS) == MIDNIGHT * NS);
    assert(leap.to_utc_ns(boundary * NS - 1) == (last_utc + 1) * NS - 1);
    int64_t tai = 0;
    assert(leap.from_utc(MIDNIGHT, false, tai) && tai == boundary);
    if (change > 0)
    {
        assert(leap.from_utc(MIDNIGHT - 1, false, tai) && tai == boundary - 2);
        assert(leap.from_utc(MIDNIGHT, true, tai) && tai == boundary - 1);
        assert(leap.to_utc_ns((boundary - 2) * NS) == (MIDNIGHT - 1) * NS);
    }
    else
    {
        assert(!leap.from_utc(MIDNIGHT - 1, false, tai));
        assert(!leap.from_utc(MIDNIGHT, true, tai));
    }
    assert(!leap.from_utc(MIDNIGHT + 60, true, tai));
    assert(leap.fresh(1000000 + timescale::MAX_AGE_US));
    assert(!leap.fresh(1000001 + timescale::MAX_AGE_US));
    assert(!leap.fresh(999999));

    timescale::LeapSeconds decoded{};
    const timescale::LeapSeconds empty{};
    assert(timescale::decode_leap_seconds(0, 3, 2, 2, 18, change, 101,
                                          1000, 2000, MIDNIGHT - 100, false, 1, empty, decoded));
    assert(decoded.event_utc == MIDNIGHT && decoded.offset == 37 && decoded.change == change);
    // The leap packet can precede PVT across the GPS week rollover.
    assert(timescale::decode_leap_seconds(0, 3, 2, 2, 18, change, 101,
                                          604799000, 0, MIDNIGHT - 100, false, 1, empty, decoded));
    assert(decoded.event_tai() == boundary);
    // Once the receiver updates currLs and clears lsChange, retain the history for queued stamps.
    assert(timescale::decode_leap_seconds(0, 3, 2, 2, static_cast<int8_t>(18 + change), 0, -2,
                                          1000, 1000, MIDNIGHT + 2, false, 2, leap, decoded));
    assert(decoded.event_tai() == boundary && decoded.to_utc_ns((boundary - 1) * NS) == last_utc * NS);
    assert(timescale::decode_leap_seconds(0, 3, 2, 2, static_cast<int8_t>(18 + change), change, -2,
                                          1000, 1000, MIDNIGHT + 2, false, 2, empty, decoded));
    assert(decoded.offset == 37 && decoded.event_tai() == boundary);
    // Poll and PVT can report opposite sides of the transition. Never replace a known schedule with mixed epochs.
    for (const int8_t receiver_offset : {int8_t{18}, static_cast<int8_t>(18 + change)})
    {
        assert(timescale::decode_leap_seconds(0, 3, 2, 2, receiver_offset, change, 1,
                                              1000, 2000, MIDNIGHT, false, 1000001, leap, decoded));
        assert(decoded.offset == 37 && decoded.event_tai() == boundary);
    }
    assert(!timescale::decode_leap_seconds(0, 3, 2, 2, 18, change, 1,
                                           1000, 2000, MIDNIGHT, false, 1000001, empty, decoded));
}

static void test_authoritative_policy()
{
    using namespace timescale;
    constexpr int64_t now = 1000000;
    LeapSeconds selected{};
    assert(select_leap_seconds(CONTINUOUS_UTC_START - 1, now, {}, {}, selected));
    assert(selected.source == LeapSource::Bulletin && selected.offset == 37 && selected.change == 0);
    const LeapSeconds before = selected;
    assert(before.fresh(now + MAX_AGE_US + 1));
    const int64_t expiry_age = (BULLETIN_EXPIRES_UTC - (CONTINUOUS_UTC_START - 1)) * 1000000LL;
    assert(before.fresh(now + expiry_age - 1) && !before.fresh(now + expiry_age));
    assert(select_leap_seconds(CONTINUOUS_UTC_START, now + 1000000, before, {}, selected));
    assert(selected.source == LeapSource::ContinuousUtc && selected.offset == 37 && selected.change == 0);
    assert(selected.fresh(now + 10LL * 366 * 86400 * 1000000));
    int64_t tai = 0;
    assert(before.from_utc(CONTINUOUS_UTC_START - 1, false, tai));
    const int64_t last_tai = tai;
    assert(selected.from_utc(CONTINUOUS_UTC_START, false, tai) && tai == last_tai + 1);
    assert(selected.leap_indicator(tai) == 0);
    assert(!selected.from_utc(CONTINUOUS_UTC_START, true, tai));
    assert(select_leap_seconds(BULLETIN_EXPIRES_UTC + 86400, now, {}, {}, selected));
    assert(selected.source == LeapSource::ContinuousUtc);
    assert(!select_leap_seconds(BULLETIN_START_UTC - 1, now, {}, {}, selected));

    const LeapSeconds receiver{true, 37, 0, 0, now};
    assert(select_leap_seconds(CONTINUOUS_UTC_START - 1, now, {}, receiver, selected));
    assert(selected.source == LeapSource::Gnss);
    assert(select_leap_seconds(CONTINUOUS_UTC_START - 1, now + MAX_AGE_US + 1, receiver, {}, selected));
    assert(selected.source == LeapSource::Bulletin);
    const LeapSeconds pending{true, 37, 1, 1798761600, now};
    assert(select_leap_seconds(pending.event_utc - 100, now, {}, pending, selected));
    assert(selected.change == 1 && selected.source == LeapSource::Gnss);
    assert(!select_leap_seconds(pending.event_utc - 100, now + MAX_AGE_US + 1, pending, {}, selected));
    assert(!select_leap_seconds(pending.event_utc + 100, now + MAX_AGE_US + 1, pending, {}, selected));
    assert(!select_leap_seconds(CONTINUOUS_UTC_START, now, pending, {}, selected));
    // A fresh consistent receiver can clear an earlier conflict.
    assert(select_leap_seconds(CONTINUOUS_UTC_START, now, pending, receiver, selected));
    assert(selected.source == LeapSource::ContinuousUtc);
    const LeapSeconds forbidden{true, 37, 1, CONTINUOUS_UTC_START + 86400, now};
    assert(!select_leap_seconds(CONTINUOUS_UTC_START - 1, now, {}, forbidden, selected));
    assert(!select_leap_seconds(CONTINUOUS_UTC_START, now, {}, forbidden, selected));
    const LeapSeconds wrong_offset{true, 38, 0, 0, now};
    assert(!select_leap_seconds(CONTINUOUS_UTC_START, now, {}, wrong_offset, selected));
    const LeapSeconds history{true, 36, 1, BULLETIN_START_UTC, now};
    assert(select_leap_seconds(CONTINUOUS_UTC_START, now, {}, history, selected));
    assert(selected.source == LeapSource::ContinuousUtc && selected.change == 1);
    assert(selected.to_utc_ns((BULLETIN_START_UTC + 35) * SECOND_NS) ==
           (BULLETIN_START_UTC - 1) * SECOND_NS);
    LeapState state;
    state.publish(before);
    assert(state.read().source == LeapSource::Bulletin && state.read().validity_us == before.validity_us);
    state.publish(selected);
    assert(state.read().source == LeapSource::ContinuousUtc && state.read().fresh(now + expiry_age));
}

int main()
{
    test_transition(1);
    test_transition(-1);
    test_authoritative_policy();
    timescale::LeapState state;
    assert(!state.read().valid);
    const timescale::LeapSeconds normal{true, 37, 0, 0, 123};
    state.publish(normal);
    assert(state.read().fresh(123));
    assert(state.read().to_utc_ns((MIDNIGHT + 37) * NS) == MIDNIGHT * NS);
    timescale::LeapSeconds result{};
    for (uint8_t valid = 0; valid < 3; ++valid)
        assert(!timescale::decode_leap_seconds(0, valid, 2, 2, 18, 0, 0, 0, 0, MIDNIGHT, false, 1, {}, result));
    for (const uint8_t source : {uint8_t{0}, uint8_t{6}, uint8_t{7}, uint8_t{255}})
        assert(!timescale::decode_leap_seconds(0, 3, source, 2, 18, 0, 0, 0, 0, MIDNIGHT, false, 1, {}, result));
    assert(!timescale::decode_leap_seconds(1, 3, 2, 2, 18, 0, 0, 0, 0, MIDNIGHT, false, 1, {}, result));
    assert(!timescale::decode_leap_seconds(0, 3, 2, 0, 18, 0, 0, 0, 0, MIDNIGHT, false, 1, {}, result));
    assert(!timescale::decode_leap_seconds(0, 3, 2, 2, 18, 2, 0, 0, 0, MIDNIGHT, false, 1, {}, result));
    assert(!timescale::decode_leap_seconds(0, 3, 2, 2, -1, 0, 0, 0, 0, MIDNIGHT, false, 1, {}, result));
    assert(!timescale::decode_leap_seconds(0, 3, 2, 2, 18, 0, 0, 0, 3000, MIDNIGHT, false, 1, {}, result));
    assert(!timescale::decode_leap_seconds(0, 3, 2, 2, 18, 1, 100, 0, 0, MIDNIGHT, false, 1, {}, result));
    std::puts("PASS: leap transitions, bulletin expiry, 2027 continuous-UTC cutover, receiver precedence/conflicts, historical timestamps and atomic policy state");
}

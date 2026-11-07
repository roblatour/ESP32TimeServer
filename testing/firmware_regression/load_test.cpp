#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <deque>

static int64_t s_now_us = 1000000;
static int64_t esp_timer_get_time() { return s_now_us; }
static uint32_t NTP_Requests_Per_Second_Load_Shedding_Threshold = 0;
static std::atomic<int64_t> s_last_load_shedding_us{0}, s_last_ntp_input_us{0}, s_first_ntp_input_us{0};
static std::atomic<uint64_t> s_ntp_admission_deadline{0};
static std::atomic<uint32_t> s_ntp_input_count{0};
static std::atomic<bool> s_ntp_shed_during_period{false};
#include "load_under_test.h"

static void test_rate(uint32_t threshold, uint32_t offered, uint32_t expected)
{
    NTP_Requests_Per_Second_Load_Shedding_Threshold = threshold;
    s_ntp_admission_deadline = 0;
    s_ntp_input_count = 0;
    s_first_ntp_input_us = 0;
    s_last_load_shedding_us = 0;
    s_ntp_shed_during_period = false;
    s_now_us = 1000000;
    uint32_t admitted = 0;
    std::deque<int64_t> recent_admissions;
    for (uint32_t i = 0; i < offered; ++i)
    {
        s_now_us = 1000000 + static_cast<int64_t>(i) * 1000000 / offered;
        if (admit_ntp_request())
        {
            ++admitted;
            recent_admissions.push_back(s_now_us);
            while (recent_admissions.front() < s_now_us - 10000)
                recent_admissions.pop_front();
            assert(recent_admissions.size() <= threshold / 100 + NTP_ADMISSION_BURST_REQUESTS);
        }
    }
    assert(admitted == expected + (offered > expected ? NTP_ADMISSION_BURST_REQUESTS - 1 : 0));
    assert(s_ntp_input_count == offered && s_first_ntp_input_us == 1000000);
    assert(s_ntp_shed_during_period == (offered > expected));
}

static void test_burst_limit()
{
    NTP_Requests_Per_Second_Load_Shedding_Threshold = 10000;
    s_ntp_admission_deadline = 0;
    s_now_us = 1099999;
    for (uint32_t i = 0; i < NTP_ADMISSION_BURST_REQUESTS; ++i)
        assert(admit_ntp_request());
    assert(!admit_ntp_request());
    ++s_now_us;
    assert(!admit_ntp_request());
    s_now_us += 98;
    assert(!admit_ntp_request());
    ++s_now_us;
    assert(admit_ntp_request() && !admit_ntp_request());
    s_now_us += 600000000;
    for (uint32_t i = 0; i < NTP_ADMISSION_BURST_REQUESTS; ++i)
        assert(admit_ntp_request());
    assert(!admit_ntp_request());
}

int main()
{
    for (uint32_t threshold : {8500U, 10000U})
    {
        test_rate(threshold, 1000, 1000);
        test_rate(threshold, 11000, threshold);
        test_rate(threshold, 13000, threshold);
        test_rate(threshold, 50000, threshold);
    }
    test_burst_limit();
    s_now_us = 3000000;
    s_last_load_shedding_us = 0;
    assert(!ntp_load_shedding_active());
    s_last_load_shedding_us = s_now_us;
    assert(ntp_load_shedding_active());
    s_now_us += 1499999;
    assert(ntp_load_shedding_active());
    ++s_now_us;
    assert(!ntp_load_shedding_active());
    std::puts("PASS: sustained 8,500/10,000 rates at 11k/13k/50k offered load, bounded bursts, no window refill and indication expiry");
}

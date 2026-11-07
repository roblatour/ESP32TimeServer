#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include "ntp_cache.h"
#include "freertos/FreeRTOS.h"
#include "leap_fixture.h"

static int64_t s_now_us = 1000000000;
extern "C" int64_t esp_timer_get_time() { return s_now_us; }
static int64_t local_now_ns() { return s_now_us * 1000; }
static portMUX_TYPE s_clock_publish_mux = portMUX_INITIALIZER_UNLOCKED;
static unsigned s_logs = 0;
template <typename... Args>
static void test_log(const char *, const char *, Args...) { ++s_logs; }
#define ESP_LOGI(...) test_log(__VA_ARGS__)
#define ESP_LOGW(...) test_log(__VA_ARGS__)
#define ESP_LOGE(...) test_log(__VA_ARGS__)
static constexpr const char *TAG = "test";
static constexpr int CONFIG_LWIP_IPV6_NUM_ADDRESSES = 3;
static constexpr int IPPROTO_UDP = 17;
static constexpr uint16_t NTP_PORT = 123;
static constexpr size_t ETH_HEADER_SIZE = 14, IPV4_HEADER_SIZE = 20, IPV6_HEADER_SIZE = 40;
static constexpr size_t UDP_HEADER_SIZE = 8, NTP_PACKET_SIZE = 48, NTP_MAX_PACKET_SIZE = 72;
static constexpr uint64_t NTP_EPOCH_OFFSET = 2208988800ULL;
static constexpr int8_t NTP_PRECISION_EXPONENT = -24;
static constexpr uint32_t NTP_ROOT_DISPERSION = 2;
static constexpr uint32_t periodicGNSSRefreshEveryThisNumberOfMinutes = 5;
static constexpr int64_t ntpHoldoverDurationUs = 120000000;
static constexpr uint32_t ntpHoldoverDispersionPpm = 200;
static std::atomic<bool> s_time_has_been_set{true}, s_time_setting_in_progress{false};
static std::atomic<bool> s_ntp_reference_valid{true}, s_ptp_clock_ready{true}, s_ptp_comp_valid{true};
static std::atomic<uint64_t> s_ntp_reference_time_64{1234};
static std::atomic<uint32_t> s_own_ipv4_address_be{0}, s_own_ipv4_broadcast_be{0};
static std::atomic<uint32_t> s_own_ipv6_sequence{0}, s_own_ipv6_count{0};
static std::atomic<uint32_t> s_own_ipv6_addresses[CONFIG_LWIP_IPV6_NUM_ADDRESSES][4]{};
static std::atomic<uint32_t> s_ptpmap_seq{0};
static std::atomic<int64_t> s_ptpmap_ref_raw_ns{0}, s_ptpmap_ref_offset_ns{0};
static std::atomic<int64_t> s_ptpmap_rate_old_mppb{0}, s_ptpmap_rate_new_mppb{0};
static constexpr int64_t ptpmap_max_age_ns = 3600000000000LL;
static constexpr int64_t ptpmap_max_rate_mppb = 500000000LL;

using nvs_handle_t = int;
using esp_err_t = int;
static constexpr int ESP_OK = 0, ESP_ERR_NVS_NOT_FOUND = 1, ESP_ERR_NVS_INVALID_LENGTH = 2;
static constexpr int NVS_READONLY = 0, NVS_READWRITE = 1;
struct latency_calibration_t { uint32_t fingerprint; int32_t ns; };
static std::map<std::string, std::vector<uint8_t>> s_nvs;
static std::string s_namespace;
static unsigned s_nvs_sets = 0, s_nvs_commits = 0;
static esp_err_t s_nvs_error = ESP_OK;
static const char *esp_err_to_name(esp_err_t) { return "injected error"; }
static esp_err_t nvs_open(const char *name, int mode, nvs_handle_t *handle)
{
    if (s_nvs_error) return s_nvs_error;
    s_namespace = name;
    *handle = 1;
    return mode == NVS_READONLY && s_nvs.count(name) == 0 ? ESP_ERR_NVS_NOT_FOUND : ESP_OK;
}
static void nvs_close(nvs_handle_t) {}
static esp_err_t nvs_get_blob(nvs_handle_t, const char *, void *data, size_t *length)
{
    if (s_nvs.count(s_namespace) == 0) return ESP_ERR_NVS_NOT_FOUND;
    const auto &value = s_nvs.at(s_namespace);
    if (*length < value.size()) return ESP_ERR_NVS_INVALID_LENGTH;
    std::memcpy(data, value.data(), value.size());
    *length = value.size();
    return ESP_OK;
}
static esp_err_t nvs_set_blob(nvs_handle_t, const char *, const void *data, size_t length)
{
    ++s_nvs_sets;
    const auto *bytes = static_cast<const uint8_t *>(data);
    s_nvs[s_namespace] = std::vector<uint8_t>(bytes, bytes + length);
    return ESP_OK;
}
static esp_err_t nvs_commit(nvs_handle_t) { ++s_nvs_commits; return ESP_OK; }

#include "firmware_under_test.h"

static void test_cache()
{
    assert(ntp_cache_init());
    ntp_client_record_t v4{};
    in6_addr ip6{}, other6{};
    ip6.s6_addr[0] = 0x20;
    other6.s6_addr[0] = 0x30;
    ntp_client_record_ipv6_t v6{};
    assert(ntp_cache_update(1, 100, 101));
    assert(ntp_cache_update(1, 200, 201));
    assert(ntp_cache_find(1, 0, &v4) && v4.prev_t2 == 0);
    assert(ntp_cache_find(2, 100, &v4) && v4.prev_t2 == 0);
    assert(ntp_cache_find(1, 101, &v4) && v4.prev_t2 == 0);
    assert(ntp_cache_find(1, 100, &v4) && v4.prev_t3 == 101);
    assert(ntp_cache_find(1, 100, &v4) && v4.prev_t2 == 0);
    assert(ntp_cache_find(1, 200, &v4) && v4.prev_t3 == 201);
    assert(ntp_cache_update_ipv6(&ip6, 100, 301));
    assert(ntp_cache_update_ipv6(&ip6, 200, 401));
    assert(ntp_cache_find_ipv6(&other6, 100, &v6) && v6.prev_t2 == 0);
    assert(ntp_cache_find_ipv6(&ip6, 100, &v6) && v6.prev_t3 == 301);
    assert(ntp_cache_find_ipv6(&ip6, 100, &v6) && v6.prev_t2 == 0);
    assert(ntp_cache_find_ipv6(&ip6, 200, &v6) && v6.prev_t3 == 401);
    for (int poll = 1; poll < 5; ++poll)
    {
        assert(ntp_cache_update(1, poll * 100, poll * 100 + 1));
        assert(ntp_cache_find(1, poll * 100, &v4) && v4.prev_t3 == static_cast<uint64_t>(poll * 100 + 1));
    }
    assert(ntp_cache_update(1, 500, 501));
    assert(ntp_cache_update_ipv6(&ip6, 500, 501));
    s_now_us += 1024000000LL;
    assert(ntp_cache_find(1, 500, &v4) && v4.prev_t3 == 501);
    assert(ntp_cache_find_ipv6(&ip6, 500, &v6) && v6.prev_t3 == 501);
    assert(ntp_cache_update(1, 600, 601));
    assert(ntp_cache_update_ipv6(&ip6, 600, 601));
    s_now_us += 1800001000LL;
    assert(ntp_cache_find(1, 600, &v4) && v4.prev_t2 == 0);
    assert(ntp_cache_find_ipv6(&ip6, 600, &v6) && v6.prev_t2 == 0);
    ntp_cache_purge_expired();
    assert(ntp_cache_update(1, 700, 701) && ntp_cache_update(1, 700, 702));
    assert(ntp_cache_find(1, 700, &v4) && v4.prev_t2 == 0);
    assert(ntp_cache_update_ipv6(&ip6, 700, 701) && ntp_cache_update_ipv6(&ip6, 700, 702));
    assert(ntp_cache_find_ipv6(&ip6, 700, &v6) && v6.prev_t2 == 0);
    ntp_cache_deinit();
    assert(ntp_cache_init());
    for (uint64_t i = 1; i <= MAX_TRACKED_CLIENTS_IPV4 + 1; ++i)
    {
        ++s_now_us;
        assert(ntp_cache_update(1, i, i + 1000));
    }
    assert(ntp_cache_find(1, 1, &v4) && v4.prev_t2 == 0);
    assert(ntp_cache_find(1, MAX_TRACKED_CLIENTS_IPV4 + 1, &v4) && v4.prev_t3 == MAX_TRACKED_CLIENTS_IPV4 + 1001);
    ntp_cache_deinit();
}

static void test_holdover()
{
    s_now_us = 1000000000;
    set_test_leap(s_now_us);
    s_ntp_last_successful_sync_us = s_now_us;
    s_ntp_last_gnss_valid_us = s_now_us;
    s_ntp_last_pps_seen_us = s_now_us;
    s_ntp_pps_active = true;
    s_ntp_gnss_timing_valid = true;
    s_model_valid = true;
    s_model_rate_valid = true;
    assert(get_ntp_reply_status().stratum == 1);
    s_now_us += 3500001;
    assert(get_ntp_reply_status().holdover);
    const uint32_t early = get_ntp_reply_status().root_dispersion;
    s_ntp_pps_active = false;
    s_ntp_pps_missing = true;
    s_ntp_gnss_invalid = true;
    s_ntp_gnss_timing_valid = false;
    s_now_us = 1119999999;
    auto status = get_ntp_reply_status();
    assert(status.stratum == 1 && status.holdover && status.root_dispersion > early);
    uint8_t request[NTP_PACKET_SIZE]{}, reply[NTP_PACKET_SIZE]{};
    build_ntp_reply(request, reply, 4, 42, status);
    const uint32_t dispersion = (uint32_t(reply[8]) << 24) | (uint32_t(reply[9]) << 16) |
                                (uint32_t(reply[10]) << 8) | reply[11];
    assert(dispersion == status.root_dispersion && reply[1] == 1 && reply[0] >> 6 == 0);
    s_now_us = 1120000000;
    assert(get_ntp_reply_status().stratum == 16);
    s_now_us = 1060000000;
    s_model_rate_valid = false;
    assert(get_ntp_reply_status().stratum == 16);
    s_model_rate_valid = true;
    s_ntp_sanity_mismatch = true;
    assert(get_ntp_reply_status().stratum == 16);
    s_ntp_sanity_mismatch = false;
    s_time_setting_in_progress = true;
    assert(get_ntp_reply_status().stratum == 16);
    s_time_setting_in_progress = false;
    s_model_valid = false;
    assert(get_ntp_reply_status().stratum == 16);
    s_model_valid = true;
    s_ntp_last_pps_seen_us = s_now_us;
    s_ntp_last_gnss_valid_us = s_now_us;
    s_ntp_pps_active = true;
    s_ntp_gnss_timing_valid = true;
    s_ntp_pps_missing = false;
    s_ntp_gnss_invalid = false;
    assert(get_ntp_reply_status().stratum == 1 && !get_ntp_reply_status().holdover);
}

static void test_clock_mapping()
{
    model_publish(1700000000, 1000000000, 33000);
    const int64_t saved_now_us = s_now_us;
    s_now_us = 2000000;
    set_test_leap(s_now_us);
    assert(get_disciplined_time_in_ntp64_format() == ntp64_from_unix_ns(1699999963999967000LL));
    int64_t unix_ns = 0;
    assert(model_ns_at(2000000000, unix_ns));
    assert(unix_ns == 1700000000999967000LL);
    assert(model_ns_at(121000000000LL, unix_ns));
    assert(unix_ns == 1700000119996040000LL);
    s_model_seq.fetch_add(1);
    assert(!model_ns_at(2000000000, unix_ns));
    assert(get_disciplined_time_in_ntp64_format() == 0);
    s_model_seq.fetch_add(1);
    model_invalidate();
    assert(!model_ns_at(2000000000, unix_ns));
    assert(get_disciplined_time_in_ntp64_format() == 0);
    s_now_us = saved_now_us;
    ptp_map_publish(10000000000LL, 1000, 33000000, -1000000);
    int64_t offset = 0;
    assert(ptp_map_offset_at(9999900000LL, offset) && offset == 997);
    assert(ptp_map_offset_at(10000000000LL, offset) && offset == 1000);
    assert(ptp_map_offset_at(11000000000LL, offset) && offset == 0);
    s_ptpmap_seq.fetch_add(1);
    assert(!ptp_map_offset_at(11000000000LL, offset));
    s_ptpmap_seq.fetch_add(1);
    const uint64_t stamp = ntp64_from_unix_ns(1700000000500000000LL);
    assert((stamp & 0xffffffffULL) == 0x80000000ULL);
}

static void test_leap_replies()
{
    constexpr int64_t midnight = 1893456000LL;
    const int64_t saved_us = s_now_us;
    s_now_us = 1000000000;
    s_model_valid = s_model_rate_valid = true;
    s_ntp_last_successful_sync_us = s_ntp_last_gnss_valid_us = s_ntp_last_pps_seen_us = s_now_us;
    for (const int8_t change : {int8_t{1}, int8_t{-1}})
    {
        const timescale::LeapSeconds leap{true, 37, change, midnight, s_now_us};
        s_leap_state.publish(leap);
        s_tx_delay[0][0].samples = tx_delay_min_samples;
        for (const int64_t boundary : {leap.event_tai() - 1, leap.event_tai()})
        {
            s_tx_delay[0][0].delay_ns = 300000;
            const int64_t tai_ns = boundary * 1000000000LL - 200000;
            const uint64_t raw = ntp64_from_tai_ns(tai_ns);
            assert(compensate_basic_t3(raw, tai_ns, false, false) ==
                   ntp64_from_unix_ns(leap.to_utc_ns(tai_ns + 300000)));
            const int32_t old_delay = s_tx_delay[0][0].delay_ns;
            update_tx_delay_estimate(false, false, raw, ntp64_from_tai_ns(tai_ns + 300000));
            assert(s_tx_delay[0][0].delay_ns == old_delay ||
                   s_tx_delay[0][0].delay_ns == old_delay - 1);
        }
        s_test_tai_seconds = leap.event_tai() - 1;
        auto status = get_ntp_reply_status();
        assert(status.stratum == 1 && status.leap_indicator == (change > 0 ? 1 : 2));
        uint8_t request[NTP_PACKET_SIZE]{}, reply[NTP_PACKET_SIZE]{};
        build_ntp_reply(request, reply, 4, 42, status);
        assert((reply[0] >> 6) == status.leap_indicator && reply[1] == 1);
        assert(utc_now() == midnight - (change > 0 ? 1 : 2));
        s_now_us += 4000000;
        status = get_ntp_reply_status();
        assert(status.holdover && status.leap_indicator == (change > 0 ? 1 : 2));
        ++s_test_tai_seconds;
        assert(get_ntp_reply_status().leap_indicator == 0 && utc_now() == midnight);
        s_timescale_requalification_required = true;
        assert(get_ntp_reply_status().stratum == 16 && get_ntp_reply_status().leap_indicator == 3);
        s_timescale_requalification_required = false;
        s_now_us = leap.updated_us + timescale::MAX_AGE_US + 1;
        assert(get_ntp_reply_status().stratum == 16 && ntp64_from_tai_ns(leap.event_tai() * 1000000000LL) == 0);
        s_now_us = 1000000000;
    }
    set_test_leap(saved_us);
    s_now_us = saved_us;
}

static void test_frames()
{
    uint8_t frame[90]{};
    frame[0] = 2; frame[6] = 4; frame[12] = 8;
    auto *ip = frame + ETH_HEADER_SIZE;
    auto *udp = ip + IPV4_HEADER_SIZE;
    ip[0] = 0x45; ip[3] = 76; ip[9] = IPPROTO_UDP;
    ip[12] = 192; ip[13] = 168; ip[14] = 1; ip[15] = 2;
    ip[16] = 192; ip[17] = 168; ip[18] = 1; ip[19] = 3;
    uint32_t own = 0;
    std::memcpy(&own, ip + 16, sizeof(own));
    s_own_ipv4_address_be = own;
    udp[0] = 0xc0; udp[1] = 0x12; udp[3] = 123; udp[5] = 56;
    const auto checksum = internet_checksum(ip, 20);
    ip[10] = uint8_t(checksum >> 8); ip[11] = uint8_t(checksum);
    size_t ip_offset = 0, udp_offset = 0;
    assert(is_ipv4_ntp_request(frame, sizeof(frame), &ip_offset, &udp_offset));
    assert(hardware_ntp_frame_is_acceptable(frame, true, ip_offset, udp_offset));
    frame[0] = 1;
    assert(!hardware_ntp_frame_is_acceptable(frame, true, ip_offset, udp_offset));
    frame[0] = 2; ip[6] = 0x20;
    assert(!is_ipv4_ntp_request(frame, sizeof(frame), &ip_offset, &udp_offset));
    ip[6] = 0; ip[10] ^= 1;
    assert(!is_ipv4_ntp_request(frame, sizeof(frame), &ip_offset, &udp_offset));

    uint8_t frame6[110]{};
    frame6[0] = 2; frame6[6] = 4; frame6[12] = 0x86; frame6[13] = 0xdd;
    ip = frame6 + ETH_HEADER_SIZE; udp = ip + IPV6_HEADER_SIZE;
    ip[0] = 0x60; ip[5] = 56; ip[6] = IPPROTO_UDP;
    ip[8] = 0x20; ip[23] = 2; ip[24] = 0x20; ip[39] = 3;
    udp[0] = 0xc0; udp[1] = 0x12; udp[3] = 123; udp[5] = 56;
    uint32_t words[4]{};
    std::memcpy(words, ip + 24, sizeof(words));
    for (int i = 0; i < 4; ++i) s_own_ipv6_addresses[0][i] = words[i];
    s_own_ipv6_count = 1;
    const auto checksum6 = ipv6_udp_checksum(ip + 8, ip + 24, udp, 56);
    udp[6] = uint8_t(checksum6 >> 8); udp[7] = uint8_t(checksum6);
    assert(is_ipv6_ntp_request(frame6, sizeof(frame6), &ip_offset, &udp_offset));
    assert(hardware_ntp_frame_is_acceptable(frame6, false, ip_offset, udp_offset));
    ip[39] = 4;
    assert(!hardware_ntp_frame_is_acceptable(frame6, false, ip_offset, udp_offset));
    ip[39] = 3; udp[6] ^= 1;
    assert(!hardware_ntp_frame_is_acceptable(frame6, false, ip_offset, udp_offset));
}

static void test_nvs_and_logging()
{
    int32_t ns = 0;
    assert(!load_latency_calibration("isr_latency", 1, 500, 20000, ns));
    assert(save_latency_calibration("isr_latency", 1, 2500));
    assert(s_nvs_sets == 1 && s_nvs_commits == 1);
    assert(load_latency_calibration("isr_latency", 1, 500, 20000, ns) && ns == 2500);
    for (int boot = 0; boot < 1000; ++boot)
    {
        assert(load_latency_calibration("isr_latency", 1, 500, 20000, ns));
        assert(save_latency_calibration("isr_latency", 1, 2500));
    }
    assert(s_nvs_sets == 1 && s_nvs_commits == 1);
    assert(!load_latency_calibration("isr_latency", 2, 500, 20000, ns));
    assert(save_latency_calibration("isr_latency", 2, 2600));
    assert(s_nvs_sets == 2 && s_nvs_commits == 2);
    s_nvs_error = 99;
    assert(!save_latency_calibration("isr_latency", 2, 2600));
    assert(!load_latency_calibration("isr_latency", 2, 500, 20000, ns));
    s_nvs_error = ESP_OK;
    s_logs = 0;
    TIMING_LOGI(TAG, "info");
    TIMING_LOGW(TAG, "warning");
    TIMING_LOGE(TAG, "error");
    assert(s_logs == (DEBUG_ENABLED ? 3u : 0u));
    char payload[8]{};
    size_t length = json_append(payload, sizeof(payload), 0, "%s", "1234567");
    assert(length == 7 && payload[7] == '\0');
    length = json_append(payload, sizeof(payload), length, "%s", "8");
    assert(length == sizeof(payload));
    assert(json_append(payload, sizeof(payload), length, "more") == sizeof(payload));
    assert(json_append(nullptr, 0, 0, "empty") == 0);
}

int main()
{
    test_cache();
    test_holdover();
    test_clock_mapping();
    test_leap_replies();
    test_frames();
    test_nvs_and_logging();
    std::printf("PASS: cache/NAT/replay/expiry, holdover/boundary/dispersion, clocks, frames, NVS and DEBUG=%d\n", DEBUG_ENABLED);
}

#include <array>
#include <cassert>
#include <cstdio>
#include <cstring>
#include "auth_settings.h"
#include "auth_vectors.h"
#include "auth_crypto_fixture.h"
#include "esp_log.h"
#include "ntp_auth.h"
#include "ptp_l2_auth.h"
#include "ptp_l2_protocol.h"

[[maybe_unused]] static std::array<uint8_t, 128> message(uint8_t type, size_t length)
{
    std::array<uint8_t, 128> result{};
    result[0] = type;
    result[1] = 0x12;
    result[3] = static_cast<uint8_t>(length);
    result[20] = 1;
    result[29] = 1;
    result[31] = 7;
    result[32] = type == 1 ? 1 : 5;
    return result;
}

int main()
{
#if TEST_INVALID_PTP_KEYS || TEST_EMPTY_PTP_KEYS || TEST_PTP_INIT_FAILURE || TEST_PTP_IMPORT_FAILURE
    s_fixture_init_failure = TEST_PTP_INIT_FAILURE != 0;
    s_fixture_import_failure_at = TEST_PTP_IMPORT_FAILURE;
    const bool ready = TEST_EMPTY_PTP_KEYS && !PTP_L2_AUTHENTICATION_REQUIRED;
    assert(ptp_l2_auth_initialize() == ready && !ptp_l2_auth_available());
    const unsigned init_calls = s_fixture_init_calls;
    const unsigned import_calls = s_fixture_import_calls;
    s_fixture_init_failure = false;
    s_fixture_import_failure_at = 0;
    assert(ptp_l2_auth_initialize() == ready && !ptp_l2_auth_available());
    assert(s_fixture_init_calls == init_calls && s_fixture_import_calls == import_calls);
    assert(s_fixture_warning_logs == (ready ? 1U : 0U));
    assert(s_fixture_error_logs == (ready ? 0U : 1U));
    if (TEST_PTP_INIT_FAILURE)
    {
        assert(init_calls == 1 && import_calls == 0 && s_fixture_destroy_calls == 0);
        assert(std::strstr(s_fixture_last_log, "PSA crypto initialization failed: -1"));
    }
    else if (TEST_PTP_IMPORT_FAILURE)
    {
        assert(init_calls == 1 && import_calls == TEST_PTP_IMPORT_FAILURE);
        assert(s_fixture_destroy_calls == TEST_PTP_IMPORT_FAILURE - 1);
        assert(std::strstr(s_fixture_last_log, "HMAC key import failed: -1"));
    }
    else
    {
        assert(init_calls == 0 && import_calls == 0 && s_fixture_destroy_calls == 0);
        if (ready)
            assert(std::strstr(s_fixture_last_log, "outgoing time is unsigned; signed incoming packets will be rejected"));
        else if (TEST_EMPTY_PTP_KEYS)
            assert(std::strstr(s_fixture_last_log, "required but no PTP keys"));
        else
        {
            const char *causes[] = {"", "exactly 32 characters", "duplicate key ID", "active key ID",
                                    "zero key ID", "ASCII alphanumeric", "key count", "exactly 32 characters",
                                    "missing terminator", "ASCII alphanumeric"};
            assert(std::strstr(s_fixture_last_log, causes[TEST_INVALID_PTP_KEYS]));
        }
    }
    for (const auto &key : s_fixture_keys)
        assert(key.length == 0);
    auto packet = message(1, 44);
    size_t length = 44;
    assert(!ptp_l2_auth_append(packet.data(), packet.size(), &length) && length == 44);
    packet = message(1, 70);
    packet[44] = 0x80; packet[45] = 9; packet[47] = 22;
    packet[48] = PTP_L2_AUTHENTICATION_SPP; packet[53] = 1;
    assert(ptp_l2_auth_verify(packet.data(), 70) == ptp_l2_auth_result_t::unavailable);
    packet[48] ^= 1;
    assert(ptp_l2_auth_verify(packet.data(), 70) == ptp_l2_auth_result_t::malformed);
    std::puts("PASS: cached PTP policy readiness, one-time diagnostics, no unsigned crypto fallback and import cleanup");
#else
    auto packet = message(1, 44);
    assert(ptp_l2_auth_verify(nullptr, 44) == ptp_l2_auth_result_t::malformed);
    assert(ptp_l2_auth_verify(packet.data(), 44) == (PTP_L2_AUTHENTICATION_REQUIRED ?
           ptp_l2_auth_result_t::authentication_required : ptp_l2_auth_result_t::unauthenticated));
    assert(ptp_l2_auth_initialize() && ptp_l2_auth_initialize() && ptp_l2_auth_available());
    assert(s_fixture_init_calls == 1 && s_fixture_import_calls == 2);
    assert(s_fixture_warning_logs == 0 && s_fixture_error_logs == 0);
    size_t length = 44;
    assert(!ptp_l2_auth_append(packet.data(), 69, &length));
    assert(length == 44);
    assert(ptp_l2_auth_append(packet.data(), packet.size(), &length));
    assert(length == 70 && packet[3] == 70);
    assert(packet[44] == 0x80 && packet[45] == 9 && packet[47] == 22);
    assert(packet[48] == PTP_L2_AUTHENTICATION_SPP && packet[49] == 0 && packet[53] == 1);
    assert(std::memcmp(packet.data() + 54, EXPECTED_PTP_ICV, 16) == 0);
    assert(ptp_l2_auth_verify(packet.data(), length) == ptp_l2_auth_result_t::valid);
    for (size_t i = 0; i < length; ++i)
    {
        auto altered = packet;
        altered[i] ^= 1;
        assert(ptp_l2_auth_verify(altered.data(), length) != ptp_l2_auth_result_t::valid);
    }
    for (size_t truncated = 0; truncated < length; ++truncated)
        assert(ptp_l2_auth_verify(packet.data(), truncated) == ptp_l2_auth_result_t::malformed);
    auto unknown = packet;
    unknown[53] = 99;
    assert(ptp_l2_auth_verify(unknown.data(), length) == ptp_l2_auth_result_t::unknown_key);
    unknown = packet;
    unknown[49] = 2;
    assert(ptp_l2_auth_verify(unknown.data(), length) == ptp_l2_auth_result_t::malformed);
    unknown = packet;
    unknown[3] += 2;
    assert(ptp_l2_auth_verify(unknown.data(), length + 2) == ptp_l2_auth_result_t::malformed);
    s_fixture_crypto_failure = true;
    assert(ptp_l2_auth_verify(packet.data(), length) == ptp_l2_auth_result_t::invalid_mac);
    auto unsigned_packet = message(1, 44);
    size_t failed_length = 44;
    assert(!ptp_l2_auth_append(unsigned_packet.data(), unsigned_packet.size(), &failed_length));
    assert(failed_length == 44 && ptp_l2_auth_available() && ptp_l2_auth_initialize());
    s_fixture_crypto_failure = false;
    for (const auto &entry : {std::array<size_t, 2>{0, 44}, {1, 44}, {8, 44}, {9, 54}, {11, 64}})
    {
        auto outgoing = message(static_cast<uint8_t>(entry[0]), entry[1]);
        size_t size = entry[1];
        assert(ptp_l2_auth_append(outgoing.data(), outgoing.size(), &size));
        assert(size == entry[1] + PTP_L2_AUTH_TLV_SIZE);
        assert(ptp_l2_auth_verify(outgoing.data(), size) == ptp_l2_auth_result_t::valid);
    }
    // A second configured key is accepted for inbound rotation; TX stays on key 1.
    assert(ptp_l2_auth_verify(ROTATED_PTP_PACKET, sizeof(ROTATED_PTP_PACKET)) == ptp_l2_auth_result_t::valid);

    std::array<uint8_t, 128> frame{};
    const uint8_t multicast[] = {1, 0x1b, 0x19, 0, 0, 0};
    std::memcpy(frame.data(), multicast, sizeof(multicast));
    frame[6] = 2;
    frame[12] = 0x88;
    frame[13] = 0xf7;
    std::memcpy(frame.data() + 14, packet.data(), length);
    ptp::ParsedMessage parsed{};
    assert(ptp::parse_l2_frame(frame.data(), length + 14, 0, nullptr, parsed, true) == ptp::MessageKind::DelayRequest);
    assert(ptp::parse_l2_frame(frame.data(), length + 14, 0, nullptr, parsed) == ptp::MessageKind::Invalid);
    assert(ptp::parse_l2_frame(frame.data(), length + 13, 0, nullptr, parsed, true) == ptp::MessageKind::Invalid);
    assert(parsed.sequence_id == 7);

    assert(ntp_auth_initialize());
    std::array<uint8_t, 72> ntp{};
    ntp[0] = 0x23;
    assert(ntp_auth_verify_request(ntp.data(), 48, 4, nullptr) ==
           (NTP_AUTHENTICATION_REQUIRED ? ntp_auth_result_t::authentication_required : ntp_auth_result_t::unauthenticated));
    assert(ntp_auth_verify_request(ntp.data(), 48, 3, nullptr) ==
           (NTP_AUTHENTICATION_REQUIRED ? ntp_auth_result_t::authentication_required : ntp_auth_result_t::unauthenticated));
    size_t ntp_length = 48;
    assert(ntp_auth_append_response(ntp.data(), ntp.size(), &ntp_length, 1));
    assert(std::memcmp(ntp.data() + 52, EXPECTED_NTP_DIGEST, 20) == 0);
    uint32_t key_id = 0;
    assert(ntp_auth_verify_request(ntp.data(), ntp_length, 4, &key_id) == ntp_auth_result_t::valid && key_id == 1);
    ntp[71] ^= 1;
    assert(ntp_auth_verify_request(ntp.data(), ntp_length, 4, nullptr) == ntp_auth_result_t::invalid_mac);
    std::printf("PASS: independent NTP required=%d / PTP required=%d, HMAC vectors, all message types, key rotation, tampering and bounds\n",
                NTP_AUTHENTICATION_REQUIRED, PTP_L2_AUTHENTICATION_REQUIRED);
#endif
}

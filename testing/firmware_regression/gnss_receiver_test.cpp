#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gnss_receiver.h"
#include "gnss_uart.h"

static_assert(sizeof(gnss::Receiver) <= 640, "GNSS parser state must remain bounded");

namespace
{
    using Bytes = std::vector<uint8_t>;
    std::deque<uint8_t> rx;
    Bytes tx;
    std::vector<Bytes> commands;
    std::map<uint32_t, uint32_t> configuration;
    int64_t now_us = 0;
    bool no_reply = false;
    bool nak = false;
    bool nak_rtcm = false;
    bool fail_read = false;
    bool fail_write = false;
    bool fail_available = false;
    bool zero_write = false;
    bool inject_pvt = false;
    bool wrong_key_first = false;
    bool delayed_stale_ack = false;
    bool reject_set = false;
    bool ignore_set = false;
    bool drop_set_ack = false;
    bool drop_get_response = false;
    size_t write_limit = SIZE_MAX;
    size_t fail_next_writes = 0;
    size_t write_attempts = 0;
    Bytes unique_payload = {2, 0, 0, 0, 0x01, 0xab, 0x23, 0xcd, 0x45, 0xef};
    Bytes module_payload;
    Bytes leap_payload(24, 0);

#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
    const char *TAG = "gnss-test";
    std::vector<std::string> info_logs, warning_logs;

    template <typename... Args>
    void record_log(bool warning, const char *, const char *format, Args... args)
    {
        char message[192];
        const int length = std::snprintf(message, sizeof(message), format, args...);
        assert(length >= 0 && static_cast<size_t>(length) < sizeof(message));
        (warning ? warning_logs : info_logs).emplace_back(message);
    }
#define ESP_LOGI(...) record_log(false, __VA_ARGS__)
#define ESP_LOGW(...) record_log(true, __VA_ARGS__)
#endif

    uint32_t read_u32(const uint8_t *bytes)
    {
        return static_cast<uint32_t>(bytes[0]) |
               (static_cast<uint32_t>(bytes[1]) << 8) |
               (static_cast<uint32_t>(bytes[2]) << 16) |
               (static_cast<uint32_t>(bytes[3]) << 24);
    }

    void append_u32(Bytes &bytes, uint32_t value)
    {
        for (unsigned i = 0; i < 4; ++i)
            bytes.push_back(static_cast<uint8_t>(value >> (8 * i)));
    }

    void set_u32(Bytes &bytes, size_t offset, uint32_t value)
    {
        for (unsigned i = 0; i < 4; ++i)
            bytes[offset + i] = static_cast<uint8_t>(value >> (8 * i));
    }

    Bytes frame(uint8_t cls, uint8_t id, const Bytes &payload)
    {
        Bytes result = {0xb5, 0x62, cls, id, static_cast<uint8_t>(payload.size()),
                        static_cast<uint8_t>(payload.size() >> 8)};
        result.insert(result.end(), payload.begin(), payload.end());
        uint8_t a = 0, b = 0;
        for (size_t i = 2; i < result.size(); ++i)
        {
            a = static_cast<uint8_t>(a + result[i]);
            b = static_cast<uint8_t>(b + a);
        }
        result.push_back(a);
        result.push_back(b);
        return result;
    }

    void queue(const Bytes &bytes)
    {
        rx.insert(rx.end(), bytes.begin(), bytes.end());
    }

    Bytes pvt_payload()
    {
        Bytes payload(92, 0x5a);
        set_u32(payload, 0, 123456);
        payload[4] = 0xea;
        payload[5] = 0x07;
        payload[6] = 11;
        payload[7] = 1;
        payload[8] = 13;
        payload[9] = 2;
        payload[10] = 3;
        payload[11] = 7;
        set_u32(payload, 16, static_cast<uint32_t>(-123456789));
        payload[20] = 5;
        payload[21] = 1;
        payload[23] = 12;
        return payload;
    }

    void append_extension(Bytes &payload, const char *text)
    {
        const size_t start = payload.size();
        payload.resize(start + 30);
        const size_t length = std::min<size_t>(std::strlen(text), 30);
        std::memcpy(payload.data() + start, text, length);
    }

    void reply_config(uint32_t key, uint32_t value)
    {
        Bytes payload = {1, 0, 0, 0};
        append_u32(payload, key);
        if ((key >> 28) == 4)
            append_u32(payload, value);
        else
            payload.push_back(static_cast<uint8_t>(value));
        queue(frame(0x06, 0x8b, payload));
    }

    void respond(const Bytes &command)
    {
        const uint8_t cls = command[2], id = command[3];
        const size_t length = command[4] | (static_cast<size_t>(command[5]) << 8);
        const uint8_t *payload = command.data() + 6;
        assert(frame(cls, id, Bytes(payload, payload + length)) == command);
        commands.push_back(command);

        if (cls == 0x06 && id == 0x8a)
        {
            assert(length >= 9 && payload[0] == 0 && payload[2] == 0 && payload[3] == 0);
            bool unsupported = false;
            for (size_t offset = 4; offset < length;)
            {
                const uint32_t key = read_u32(payload + offset);
                offset += 4;
                const size_t width = (key >> 28) == 4 ? 4 : 1;
                assert(offset + width <= length);
                if (nak_rtcm && (key & 0xffff) == 4)
                    unsupported = true;
                if (!nak && !unsupported && !reject_set && !ignore_set)
                    configuration[key] = width == 4 ? read_u32(payload + offset) : payload[offset];
                offset += width;
            }
            if (delayed_stale_ack)
                queue(frame(0x05, 1, {cls, id}));
            if (inject_pvt)
            {
                queue(frame(0x01, 0x07, pvt_payload()));
                inject_pvt = false;
            }
            if (!no_reply && !drop_set_ack)
                queue(frame(0x05, nak || unsupported || reject_set ? 0 : 1, {cls, id}));
            return;
        }
        if (no_reply)
            return;
        if (nak)
        {
            queue(frame(0x05, 0, {cls, id}));
            return;
        }
        if (inject_pvt)
        {
            queue(frame(0x01, 0x07, pvt_payload()));
            inject_pvt = false;
        }
        if (cls == 0x06 && id == 0x8b)
        {
            assert(length == 8 && payload[0] == 0 && payload[1] == 0 && payload[2] == 0 && payload[3] == 0);
            if (drop_get_response)
                return;
            const uint32_t key = read_u32(payload + 4);
            if (wrong_key_first)
            {
                reply_config(key + 1, 99);
                wrong_key_first = false;
            }
            reply_config(key, configuration[key]);
        }
        else if (cls == 0x27 && id == 0x03)
            queue(frame(cls, id, unique_payload));
        else if (cls == 0x0a && id == 0x04)
            queue(frame(cls, id, module_payload));
        else if (cls == 0x01 && id == 0x26)
            queue(frame(cls, id, leap_payload));
        else if (cls == 0x01 && id == 0x07)
            queue(frame(cls, id, pvt_payload()));
        else
            assert(false);
    }

    void reset_fixture()
    {
        rx.clear();
        tx.clear();
        commands.clear();
        configuration.clear();
        configuration[0x10730001] = 1;
        now_us = 0;
        no_reply = nak = nak_rtcm = fail_read = fail_write = fail_available = zero_write = false;
        inject_pvt = wrong_key_first = delayed_stale_ack = reject_set = false;
        ignore_set = drop_set_ack = drop_get_response = false;
        fail_next_writes = write_attempts = 0;
#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
        info_logs.clear();
        warning_logs.clear();
#endif
        write_limit = SIZE_MAX;
        unique_payload = {2, 0, 0, 0, 0x01, 0xab, 0x23, 0xcd, 0x45, 0xef};
        module_payload.assign(40, 0);
        append_extension(module_payload, "FWVER=SPG 5.10");
        append_extension(module_payload, "PROTVER=34.10");
        append_extension(module_payload, "MOD=MAX-M10S");
        leap_payload.assign(24, 0);
        set_u32(leap_payload, 0, 123456);
        leap_payload[8] = 2;
        leap_payload[9] = 18;
        leap_payload[10] = 5;
        leap_payload[11] = 0xff;
        set_u32(leap_payload, 12, static_cast<uint32_t>(-123));
        leap_payload[23] = 3;
    }

    Bytes payload_of(const Bytes &command)
    {
        return Bytes(command.begin() + 6, command.end() - 2);
    }

    void configure_stationary_mode(gnss::Receiver &s_gnss, bool &first_sync)
    {
        (void)s_gnss;
#include "stationary_mode_under_test.h"
    }

    void test_stationary_application()
    {
        reset_fixture();
        gnss::Receiver receiver;
        bool first_sync = false;
        configure_stationary_mode(receiver, first_sync);
        assert(commands.empty() && !first_sync);

#if UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED
        for (int scenario = 0; scenario < 6; ++scenario)
        {
            reset_fixture();
            receiver.reset();
            configuration[0x20110021] = scenario == 2 ? 1 : 2;
            drop_set_ack = scenario == 1;
            ignore_set = scenario == 2;
            drop_get_response = scenario == 3;
            reject_set = scenario == 4;
            fail_next_writes = scenario == 5 ? 1 : 0;
            inject_pvt = scenario == 0;
            first_sync = true;
            configure_stationary_mode(receiver, first_sync);
            const size_t expected_commands = scenario == 5 ? 1 : 2;
            assert(!first_sync && commands.size() == expected_commands);
            if (scenario != 5)
                assert(commands[0] == frame(6, 0x8a, {0, 1, 0, 0, 0x21, 0, 0x11, 0x20, 2}));
            assert(commands.back() == frame(6, 0x8b, {0, 0, 0, 0, 0x21, 0, 0x11, 0x20}));
            assert(write_attempts == 2);
            assert(info_logs.size() == (scenario == 0 || scenario == 1 || scenario >= 4 ? 1U : 0U));
            assert(warning_logs.size() == (scenario == 0 ? 0U : 1U));
            if (scenario == 1 || scenario == 3)
                assert(now_us == 1500000);
            if (scenario == 1 || scenario >= 4)
                assert(warning_logs.front().find("did not acknowledge") != std::string::npos);
            if (scenario == 2)
                assert(warning_logs.front().find("dynamic model=1 (expected 2)") != std::string::npos);
            if (scenario == 3)
                assert(warning_logs.front().find("readback failed") != std::string::npos);
            configure_stationary_mode(receiver, first_sync);
            assert(commands.size() == expected_commands && write_attempts == 2);
            if (scenario == 0)
            {
                gnss::TimeSolution solution{};
                assert(receiver.next_pvt(solution, 100) == gnss::Status::ok && solution.itow == 123456);
                assert(commands.size() == 2);
            }
        }
#else
        first_sync = true;
        configure_stationary_mode(receiver, first_sync);
        assert(commands.empty() && !first_sync);
#endif
        std::puts("PASS: application stationary-mode gating and production receiver readback");
    }

    void test_configuration()
    {
        reset_fixture();
        gnss::Receiver receiver;
        assert(receiver.probe(100) == gnss::Status::ok);
        assert(commands[0] == frame(6, 0x8b, {0, 0, 0, 0, 1, 0, 0x73, 0x10}));
        assert(receiver.set_uart1_baud(115200, 100) == gnss::Status::ok);
        assert(commands[1] == frame(6, 0x8a, {0, 3, 0, 0, 1, 0, 0x52, 0x40, 0, 0xc2, 1, 0}));
        gnss::Status rtcm = gnss::Status::ok;
        nak_rtcm = true;
        assert(receiver.configure_output(gnss::OutputPort::uart1, true, true, &rtcm, 100) == gnss::Status::ok);
        assert(rtcm == gnss::Status::nak);
        assert(payload_of(commands[2]) == Bytes({0, 3, 0, 0, 1, 0, 0x74, 0x10, 1,
                                                  2, 0, 0x74, 0x10, 1}));
        assert(payload_of(commands[5]) == Bytes({0, 3, 0, 0, 4, 0, 0x74, 0x10, 0}));
        assert(configuration[0x10740001] == 1 && configuration[0x10740002] == 1);
        assert(receiver.enable_automatic_pvt(100) == gnss::Status::ok);
        assert(payload_of(commands[6]) == Bytes({0, 1, 0, 0, 7, 0, 0x91, 0x20, 1}));
        assert(configuration[0x20910007] == 1);
        assert(receiver.set_stationary_model(100) == gnss::Status::ok);
        assert(payload_of(commands[commands.size() - 1]) ==
               Bytes({0, 1, 0, 0, 0x21, 0, 0x11, 0x20, 2}));
        uint8_t model = 99;
        assert(receiver.read_dynamic_model(model, 100) == gnss::Status::ok && model == 2);
        assert(payload_of(commands.back()) == Bytes({0, 0, 0, 0, 0x21, 0, 0x11, 0x20}));

        no_reply = true;
        assert(receiver.set_stationary_model(5) == gnss::Status::timeout);
        no_reply = false;
        assert(receiver.read_dynamic_model(model, 100) == gnss::Status::ok && model == 2);
        nak = true;
        assert(receiver.configure_output(gnss::OutputPort::uart1, true, true, &rtcm, 100) == gnss::Status::nak);
        nak = false;
        wrong_key_first = true;
        assert(receiver.read_dynamic_model(model, 100) == gnss::Status::ok && model == 2);

        reset_fixture();
        queue(frame(0x05, 1, {0x06, 0x8a}));
        no_reply = true;
        assert(receiver.set_stationary_model(5) == gnss::Status::timeout);
        assert(rx.empty());

        reset_fixture();
        reject_set = true;
        delayed_stale_ack = true;
        assert(receiver.configure_output(gnss::OutputPort::uart1, true, true, &rtcm, 100) ==
               gnss::Status::malformed);
        // The stale ACK is accepted first; readback must still reject the unchanged setting.
        assert(commands.size() == 2 && configuration[0x10740001] == 0);

        reset_fixture();
        reject_set = true;
        delayed_stale_ack = true;
        assert(receiver.enable_automatic_pvt(100) == gnss::Status::malformed);
        assert(commands.size() == 2 && configuration[0x20910007] == 0);

        reset_fixture();
        reply_config(0x20110021, 2);
        no_reply = true;
        model = 99;
        assert(receiver.read_dynamic_model(model, 5) == gnss::Status::timeout && model == 99);

        for (const auto &port : {std::pair<gnss::OutputPort, uint8_t>{gnss::OutputPort::i2c, uint8_t{0x72}},
                                 {gnss::OutputPort::uart2, uint8_t{0x76}}})
        {
            reset_fixture();
            assert(receiver.configure_output(port.first, false, false, &rtcm, 100) == gnss::Status::ok);
            assert(payload_of(commands[0]) == Bytes({0, 3, 0, 0, 1, 0, port.second, 0x10, 0,
                                                     2, 0, port.second, 0x10, 0}));
            assert(payload_of(commands[3]) == Bytes({0, 3, 0, 0, 4, 0, port.second, 0x10, 0}));
        }
        std::puts("PASS: native UBX configuration bytes, RAM/BBR layers, readback, ACK loss and NAK");
    }

    void test_timing_frames()
    {
        reset_fixture();
        gnss::Receiver polling;
        gnss::TimeSolution polled{};
        assert(polling.next_pvt(polled, 100) == gnss::Status::ok && polled.itow == 123456);
        assert(commands.back() == frame(1, 7, {}));

        reset_fixture();
        gnss::Receiver receiver;
        assert(receiver.enable_automatic_pvt(100) == gnss::Status::ok);
        gnss::TimeSolution solution{};
        queue(frame(1, 7, pvt_payload()));
        assert(receiver.discard_pending_pvt() == gnss::Status::ok);
        assert(receiver.next_pvt(solution, 5) == gnss::Status::timeout);

        Bytes corrupt = frame(1, 7, pvt_payload());
        corrupt.back() ^= 1;
        Bytes ignored_payload(48);
        for (size_t i = 0; i < ignored_payload.size(); ++i)
            ignored_payload[i] = static_cast<uint8_t>(0x80 + i);
        queue({'$', 'G', 'P', 'R', 'M', 'C', '\r', '\n'});
        queue(corrupt);
        queue(frame(0x11, 0x22, ignored_payload));
        queue({0xb5, 0x62, 1, 7, 0xff, 0xff, 0x55});
        queue(frame(1, 7, pvt_payload()));
        queue(frame(1, 7, pvt_payload()));
        assert(receiver.next_pvt(solution, 100) == gnss::Status::ok);
        const auto *state = reinterpret_cast<const unsigned char *>(&receiver);
        const auto *state_end = state + sizeof(receiver);
        assert(std::search(state, state_end, ignored_payload.begin() + 24, ignored_payload.end()) == state_end);
        const Bytes excluded_pvt_fields(32, 0x5a);
        assert(std::search(state, state_end, excluded_pvt_fields.begin(), excluded_pvt_fields.end()) == state_end);
        assert(solution.itow == 123456 && solution.year == 2026 && solution.month == 11 && solution.day == 1);
        assert(solution.hour == 13 && solution.minute == 2 && solution.second == 3);
        assert(solution.nano == -123456789 && solution.fix_type == 5 && solution.satellites == 12);
        assert(solution.fix_ok && solution.date_valid && solution.time_valid && solution.fully_resolved);
        const uint64_t generation = solution.generation;
        assert(receiver.next_pvt(solution, 100) == gnss::Status::ok && solution.generation == generation + 1);
        assert(receiver.next_pvt(solution, 5) == gnss::Status::timeout);
        queue(frame(1, 7, {1, 2, 3}));
        assert(receiver.next_pvt(solution, 100) == gnss::Status::malformed);

        inject_pvt = true;
        uint8_t model = 0;
        assert(receiver.read_dynamic_model(model, 100) == gnss::Status::ok);
        assert(receiver.next_pvt(solution, 5) == gnss::Status::ok);
        assert(receiver.discard_pending_pvt() == gnss::Status::ok);
        assert(receiver.next_pvt(solution, 5) == gnss::Status::timeout);
        gnss::LeapInformation leap{};
        assert(receiver.leap_information(leap, 100) == gnss::Status::ok);
        assert(leap.itow == 123456 && leap.version == 0 && leap.valid == 3);
        assert(leap.current_source == 2 && leap.change_source == 5 && leap.current_leap_seconds == 18);
        assert(leap.leap_change == -1 && leap.time_to_event == -123);
        leap_payload[4] = 1;
        assert(receiver.leap_information(leap, 100) == gnss::Status::malformed);
        std::puts("PASS: native UBX framing, resynchronization, fresh PVT, signed time and leap fields");
    }

    void test_identity_and_errors()
    {
        reset_fixture();
        gnss::Receiver receiver;
        char id[13]{};
        assert(receiver.unique_id(id, 100) == gnss::Status::ok);
        assert(std::strcmp(id, "01AB23CD45EF") == 0);
        assert(commands.back() == frame(0x27, 0x03, {}));
        unique_payload = {1, 0, 0, 0, 0xde, 0xad, 0xbe, 0xef, 0x01};
        assert(receiver.unique_id(id, 100) == gnss::Status::ok);
        assert(std::strcmp(id, "DEADBEEF01") == 0);
        unique_payload.pop_back();
        assert(receiver.unique_id(id, 100) == gnss::Status::malformed && id[0] == '\0');
        gnss::ModuleInformation info{};
        assert(receiver.module_information(info, 100) == gnss::Status::ok);
        assert(commands.back() == frame(0x0a, 0x04, {}));
        assert(std::strcmp(info.module_name, "MAX-M10S") == 0 && std::strcmp(info.firmware_type, "SPG") == 0);
        assert(info.firmware_high == 5 && info.firmware_low == 10 && info.protocol_high == 34 && info.protocol_low == 10);
        const Bytes valid_module_payload = module_payload;
        module_payload.assign(40, 0);
        append_extension(module_payload, "FWVER=SPG +255.255");
        append_extension(module_payload, "PROTVER= 000.001");
        append_extension(module_payload, "MOD=MAX-M10S");
        assert(receiver.module_information(info, 100) == gnss::Status::ok);
        assert(info.firmware_high == 255 && info.firmware_low == 255 &&
               info.protocol_high == 0 && info.protocol_low == 1);
        module_payload = valid_module_payload;
        assert(receiver.module_information(info, 100) == gnss::Status::ok);
        for (const char *version : {"FWVER=SPG 256.10", "FWVER=SPG 4294967296.10",
                                    "FWVER=SPG 5.4294967296"})
        {
            module_payload.assign(40, 0);
            append_extension(module_payload, version);
            append_extension(module_payload, "PROTVER=34.10");
            append_extension(module_payload, "MOD=MAX-M10S");
            assert(receiver.module_information(info, 100) == gnss::Status::malformed);
            assert(info.firmware_high == 5 && info.firmware_low == 10);
        }
        module_payload.assign(40, 0);
        append_extension(module_payload, "FWVER=SPG 5.10");
        append_extension(module_payload, "PROTVER=4294967296.10");
        append_extension(module_payload, "MOD=MAX-M10S");
        assert(receiver.module_information(info, 100) == gnss::Status::malformed);
        assert(info.protocol_high == 34 && info.protocol_low == 10);
        module_payload = valid_module_payload;
        module_payload.pop_back();
        assert(receiver.module_information(info, 100) == gnss::Status::malformed);
        nak = true;
        assert(receiver.unique_id(id, 100) == gnss::Status::nak && id[0] == '\0');
        nak = false;

        fail_available = true;
        assert(receiver.probe(10) == gnss::Status::transport_error);
        fail_available = false;
        fail_write = true;
        assert(receiver.probe(10) == gnss::Status::transport_error);
        fail_write = false;
        zero_write = true;
        assert(receiver.probe(10) == gnss::Status::transport_error);
        zero_write = false;
        write_limit = 2;
        assert(receiver.probe(100) == gnss::Status::ok);
        fail_read = true;
        assert(receiver.probe(10) == gnss::Status::transport_error);
        fail_read = false;
        no_reply = true;
        const int64_t start = now_us;
        assert(receiver.probe(7) == gnss::Status::timeout && now_us - start == 7000);
        std::puts("PASS: native receiver identity compatibility, bounded fields and transport errors");
    }
}

int64_t esp_timer_get_time() { return now_us; }
void vTaskDelay(uint32_t ticks) { now_us += static_cast<int64_t>(ticks) * 1000; }

esp_err_t gnss_uart_available(size_t *available)
{
    if (fail_available)
        return ESP_FAIL;
    *available = rx.size();
    return ESP_OK;
}

esp_err_t gnss_uart_read(uint8_t *buffer, size_t length, uint32_t, size_t *bytes_read)
{
    if (fail_read)
        return ESP_FAIL;
    *bytes_read = 0;
    while (*bytes_read < length && !rx.empty())
    {
        buffer[(*bytes_read)++] = rx.front();
        rx.pop_front();
    }
    return ESP_OK;
}

esp_err_t gnss_uart_write(const uint8_t *buffer, size_t length, size_t *bytes_written)
{
    ++write_attempts;
    if (fail_next_writes > 0)
    {
        --fail_next_writes;
        return ESP_FAIL;
    }
    if (fail_write)
        return ESP_FAIL;
    *bytes_written = zero_write ? 0 : std::min(length, write_limit);
    tx.insert(tx.end(), buffer, buffer + *bytes_written);
    if (tx.size() >= 6)
    {
        const size_t message_length = 8 + tx[4] + (static_cast<size_t>(tx[5]) << 8);
        if (tx.size() == message_length)
        {
            const Bytes command = tx;
            tx.clear();
            respond(command);
        }
        else
            assert(tx.size() < message_length);
    }
    return ESP_OK;
}

int main()
{
    test_stationary_application();
    test_configuration();
    test_timing_frames();
    test_identity_and_errors();
    return 0;
}

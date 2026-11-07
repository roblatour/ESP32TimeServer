#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#define private public
#include "../../main/custom/ptp_followers.h"
#undef private
#include "../../main/custom/ptp_l2_protocol.h"

namespace
{
    int failures = 0;

    void check(bool condition, const char *expression, int line)
    {
        if (!condition)
        {
            std::fprintf(stderr, "Failure at line %d: %s\n", line, expression);
            ++failures;
        }
    }

#define CHECK(expression) check((expression), #expression, __LINE__)

    using Frame = std::vector<uint8_t>;

    void write_u16(uint8_t *data, uint16_t value)
    {
        data[0] = static_cast<uint8_t>(value >> 8);
        data[1] = static_cast<uint8_t>(value);
    }

    void write_u32(uint8_t *data, uint32_t value)
    {
        data[0] = static_cast<uint8_t>(value >> 24);
        data[1] = static_cast<uint8_t>(value >> 16);
        data[2] = static_cast<uint8_t>(value >> 8);
        data[3] = static_cast<uint8_t>(value);
    }

    Frame make_delay_request()
    {
        Frame frame(60, 0);
        const uint8_t destination[6] = {0x01, 0x1b, 0x19, 0x00, 0x00, 0x00};
        const uint8_t source[6] = {0x02, 0x10, 0x20, 0x30, 0x40, 0x50};
        std::memcpy(frame.data(), destination, sizeof(destination));
        std::memcpy(frame.data() + 6, source, sizeof(source));
        frame[12] = 0x88;
        frame[13] = 0xf7;
        uint8_t *message = frame.data() + 14;
        message[0] = 1;
        message[1] = 2;
        write_u16(message + 2, 44);
        message[4] = 7;
        for (size_t i = 0; i < 8; ++i)
            message[20 + i] = static_cast<uint8_t>(i + 1);
        message[28] = 0x12;
        message[29] = 0x34;
        write_u16(message + 30, 0x4567);
        message[32] = 1;
        write_u32(message + 40, 999999999U);
        return frame;
    }

    Frame make_announce()
    {
        Frame frame(78, 0);
        const Frame delay = make_delay_request();
        std::memcpy(frame.data(), delay.data(), 48);
        uint8_t *message = frame.data() + 14;
        message[0] = 11;
        write_u16(message + 2, 64);
        message[32] = 5;
        message[33] = 1;
        message[47] = 128;
        message[48] = 6;
        message[49] = 0x20;
        write_u16(message + 50, 0x1234);
        message[52] = 129;
        for (size_t i = 0; i < 8; ++i)
            message[53 + i] = static_cast<uint8_t>(0xa0 + i);
        write_u16(message + 61, 254);
        message[63] = 0x20;
        return frame;
    }

    ptp::MessageKind parse(const Frame &frame, size_t length, uint8_t domain = 7,
                           const uint8_t *local_identity = nullptr)
    {
        ptp::ParsedMessage parsed{};
        return ptp::parse_l2_frame(frame.data(), length, domain, local_identity, parsed);
    }

    void test_follower_lifecycle()
    {
        const uint8_t first[10] = {1, 2, 3, 4, 5, 6, 7, 8, 0, 1};
        const uint8_t other_port[10] = {1, 2, 3, 4, 5, 6, 7, 8, 0, 2};
        ptp::FollowerTable<3> table(100);
        CHECK(table.observe(1, first, 1000, first));
        CHECK(table.observe(1, first, 1050, first));
        CHECK(table.snapshot_and_reset(1050).current == 1);
        CHECK(table.observe(2, first, 1060, first));
        CHECK(table.observe(1, other_port, 1070, other_port));
        ptp::Snapshot current = table.snapshot_and_reset(1070);
        CHECK(current.current == 3);
        CHECK(current.min == 1);
        CHECK(current.max == 3);
        CHECK(table.snapshot_and_reset(1070).min == 3);
        CHECK(table.snapshot_and_reset(1070).max == 3);
        table.expire(1169);
        CHECK(table.snapshot_and_reset(1169).current == 1);
        table.expire(1170);
        CHECK(table.snapshot_and_reset(1170).current == 0);
    }

    void test_follower_rollover_snapshots_clear_and_capacity()
    {
        const uint8_t identity[10] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
        ptp::FollowerTable<1> signed_range_table(100);
        CHECK(signed_range_table.observe(0, identity, -50, identity));
        signed_range_table.expire(49);
        CHECK(signed_range_table.snapshot_and_reset(49).current == 1);
        signed_range_table.expire(50);
        CHECK(signed_range_table.snapshot_and_reset(50).current == 0);

        ptp::FollowerTable<2> table(1000);
        CHECK(table.observe(0, identity, 0, identity));
        const uint8_t second[10] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 1};
        CHECK(table.observe(0, second, 1, second));
        CHECK(table.observe(0, identity, 2, identity));
        const ptp::Snapshot frozen = table.snapshot_and_reset(2);
        CHECK(frozen.current == 2);
        table.clear();
        CHECK(table.snapshot_and_reset(2).current == 0);
        CHECK(frozen.current == 2);
        CHECK(table.observe(0, identity, 3, identity));
        CHECK(table.observe(0, second, 3, second));
        CHECK(!table.observe(1, second, 3, second));
        CHECK(table.snapshot_and_reset(3).overflow_drops == 1);

        ptp::FollowerTable<0> zero_capacity(100);
        CHECK(!zero_capacity.observe(0, identity, 0, identity));
        CHECK(zero_capacity.snapshot_and_reset(0).overflow_drops == 1);
        CHECK(!zero_capacity.observe(0, identity, 1, identity));
        CHECK(zero_capacity.snapshot_and_reset(1).overflow_drops == 1);

        ptp::FollowerTable<0> saturation(100);
        saturation.snapshot_.overflow_drops = std::numeric_limits<uint32_t>::max() - 1;
        CHECK(!saturation.observe(0, identity, 0, identity));
        CHECK(saturation.snapshot_.overflow_drops == std::numeric_limits<uint32_t>::max());
        CHECK(!saturation.observe(0, identity, 1, identity));
        CHECK(saturation.snapshot_.overflow_drops == std::numeric_limits<uint32_t>::max());
    }

    void test_follower_report_serialization()
    {
        char report[ptp::FOLLOWER_REPORT_CAPACITY]{};
        const ptp::Snapshot snapshot{2, 0, 3, 5};
        const int size = ptp::format_follower_report(report, sizeof(report), snapshot);
        CHECK(size > 0 && static_cast<size_t>(size) < sizeof(report));
        CHECK(std::string(report) == "\"ptp\":{\"followers\":{\"current\":2,\"min\":0,\"max\":3},\"clients\":[],\"follower_overflow_drops\":5}");
        const std::string frozen = report;
        const std::string combined = "{\"this_period\":{\"ntp\":{}," + frozen + "}}";
        CHECK(combined.find("\"ntp\":{},\"ptp\":") != std::string::npos);
        const std::string ptp_only = "{\"this_period\":{" + frozen + "}}";
        CHECK(ptp_only.find("\"ntp\"") == std::string::npos);
        ptp::FollowerTable<1> table(100);
        table.clear();
        CHECK(frozen == report);
        const auto maximum = std::numeric_limits<uint32_t>::max();
        const ptp::Snapshot worst_case{maximum, maximum, maximum, maximum};
        CHECK(ptp::format_follower_report(report, sizeof(report), worst_case) < static_cast<int>(sizeof(report)));
        char short_buffer[4]{};
        CHECK(ptp::format_follower_report(short_buffer, sizeof(short_buffer), snapshot) >= static_cast<int>(sizeof(short_buffer)));
        CHECK(short_buffer[3] == '\0');
    }

    void test_client_continuity_and_deduplication()
    {
        const uint8_t first[10] = {1};
        const uint8_t second[10] = {2};
        const uint8_t mac[6] = {0, 0xf1, 0xb2, 0xd2, 0xca, 0x9a};
        ptp::FollowerTable<2> table(100);
        CHECK(table.observe(0, first, 0, mac));
        CHECK(table.observe(0, second, 50, mac));
        auto snapshot = table.snapshot_and_reset(50);
        CHECK(snapshot.current == 2 && snapshot.client_count == 1);
        CHECK(!snapshot.clients[0].active_entire_period);
        table.expire(100);
        snapshot = table.snapshot_and_reset(100);
        CHECK(snapshot.current == 1 && snapshot.client_count == 1);
        CHECK(snapshot.clients[0].active_entire_period);
        table.expire(150);
        CHECK(table.observe(0, first, 151, mac));
        snapshot = table.snapshot_and_reset(151);
        CHECK(snapshot.client_count == 1 && !snapshot.clients[0].active_entire_period);
        CHECK(table.observe(0, first, 152, mac));
        CHECK(table.snapshot_and_reset(152).clients[0].active_entire_period);
        table.clear();
        CHECK(table.observe(0, first, 153, mac));
        snapshot = table.snapshot_and_reset(153);
        CHECK(snapshot.client_count == 1 && !snapshot.clients[0].active_entire_period);
        CHECK(std::memcmp(snapshot.clients[0].mac, mac, sizeof(mac)) == 0);
        char report[ptp::FOLLOWER_REPORT_CAPACITY]{};
        CHECK(ptp::format_follower_report(report, sizeof(report), snapshot) > 0);
        CHECK(std::string(report).find("\"mac\":\"00:f1:b2:d2:ca:9a\",\"active_entire_period\":false") != std::string::npos);
    }

    void test_client_report_limit_and_first_arrival_order()
    {
        ptp::FollowerTable<32> table(100);
        for (uint8_t i = 0; i < 32; ++i)
        {
            const uint8_t identity[10] = {i};
            CHECK(table.observe(0, identity, 0, identity));
        }
        CHECK(table.snapshot_and_reset(0).client_count == 32);
        auto snapshot = table.snapshot_and_reset(0);
        for (size_t i = 0; i < 32; ++i)
            CHECK(snapshot.clients[i].active_entire_period);
        int64_t now = 0;
        for (uint8_t i = 32; i < 150; ++i)
        {
            now += 50;
            for (uint8_t j = 0; j < 31; ++j)
            {
                const uint8_t identity[10] = {j};
                CHECK(table.observe(0, identity, now, identity));
            }
            const uint8_t identity[10] = {31};
            const uint8_t mac[6] = {i};
            CHECK(table.observe(0, identity, now, mac));
        }
        snapshot = table.snapshot_and_reset(now);
        constexpr size_t limit = ptp::DEFAULT_CLIENT_REPORT_LIMIT;
        CHECK(snapshot.client_count == limit);
        for (size_t i = 0; i < limit; ++i)
        {
            CHECK(snapshot.clients[i].mac[0] == i);
            CHECK(snapshot.clients[i].active_entire_period == (i < 31));
        }
        CHECK(table.snapshot_and_reset(now).client_count == 32);

        ptp::FollowerTable<2, 3> limited(100);
        for (uint8_t i = 0; i < 5; ++i)
        {
            const uint8_t identity[10] = {static_cast<uint8_t>(i % 2)};
            const uint8_t mac[6] = {i};
            (void)limited.observe(0, identity, i, mac);
        }
        const auto limited_snapshot = limited.snapshot_and_reset(5);
        CHECK(limited_snapshot.client_count == 3 && limited_snapshot.clients.size() == 3);

        snapshot.current = snapshot.min = snapshot.max = snapshot.overflow_drops = UINT32_MAX;
        for (auto &client : snapshot.clients)
        {
            std::memset(client.mac, 0xff, sizeof(client.mac));
            client.active_entire_period = false;
        }
        char report[ptp::FOLLOWER_REPORT_CAPACITY]{};
        const int size = ptp::format_follower_report(report, sizeof(report), snapshot);
        CHECK(size > 0 && static_cast<size_t>(size) < sizeof(report));
        CHECK(static_cast<size_t>(size) < ptp::follower_report_capacity(snapshot.client_count));
        std::vector<char> exact(static_cast<size_t>(size) + 1);
        CHECK(ptp::format_follower_report(exact.data(), exact.size(), snapshot) == size);
        CHECK(ptp::format_follower_report(exact.data(), exact.size() - 1, snapshot) == size);
        CHECK(ptp::format_follower_report(nullptr, 0, snapshot) == size);
    }

    void test_frozen_report_periods()
    {
        const uint8_t identity[10] = {1, 2, 3, 4, 5, 6, 7, 8, 0, 1};
        ptp::FollowerTable<1> table(100);
        std::vector<std::string> queued;
        char report[ptp::FOLLOWER_REPORT_CAPACITY]{};
        const auto generate = [&](int64_t now_us)
        {
            const auto snapshot = table.snapshot_and_reset(now_us);
            CHECK(snapshot.min <= snapshot.current && snapshot.current <= snapshot.max);
            CHECK(ptp::format_follower_report(report, sizeof(report), snapshot) > 0);
            queued.emplace_back(report);
        };
        generate(0);
        CHECK(table.observe(0, identity, 1, identity));
        generate(2);
        CHECK(table.observe(0, identity, 3, identity));
        generate(4);
        table.expire(103);
        generate(104);
        CHECK(queued[0].find("\"current\":0,\"min\":0,\"max\":0") != std::string::npos);
        CHECK(queued[1].find("\"current\":1,\"min\":0,\"max\":1") != std::string::npos);
        CHECK(queued[2].find("\"current\":1,\"min\":1,\"max\":1") != std::string::npos);
        CHECK(queued[3].find("\"current\":0,\"min\":0,\"max\":1") != std::string::npos);
        FILE *storage = nullptr;
#if defined(_MSC_VER)
        CHECK(tmpfile_s(&storage) == 0);
#else
        storage = std::tmpfile();
#endif
        CHECK(storage != nullptr);
        if (storage != nullptr)
        {
            const std::string frozen = queued[1];
            CHECK(std::fwrite(frozen.data(), 1, frozen.size(), storage) == frozen.size());
            table.clear();
            generate(105);
            std::rewind(storage);
            char restored[ptp::FOLLOWER_REPORT_CAPACITY]{};
            CHECK(std::fread(restored, 1, frozen.size(), storage) == frozen.size());
            CHECK(std::string(restored) == frozen);
            CHECK(queued[1] == frozen);
            std::fclose(storage);
        }
        const auto after_retry = table.snapshot_and_reset(106);
        CHECK(after_retry.current == 0 && after_retry.min == 0 && after_retry.max == 0);
    }

    void test_delay_request_validity_and_output()
    {
        Frame frame = make_delay_request();
        ptp::ParsedMessage parsed{};
        CHECK(ptp::parse_l2_frame(frame.data(), frame.size(), 7, nullptr, parsed) == ptp::MessageKind::DelayRequest);
        CHECK(parsed.domain == 7);
        CHECK(parsed.sequence_id == 0x4567);
        CHECK(std::memcmp(parsed.source_port_identity, frame.data() + 34, 10) == 0);
        write_u32(frame.data() + 14 + 40, 999999999U);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::DelayRequest);
        write_u32(frame.data() + 14 + 40, 1000000000U);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
    }

    void test_frame_ethernet_and_dispatch_validation()
    {
        Frame frame = make_delay_request();
        ptp::ParsedMessage parsed{};
        CHECK(ptp::parse_l2_frame(nullptr, 60, 7, nullptr, parsed) == ptp::MessageKind::Invalid);
        CHECK(parse(frame, 13) == ptp::MessageKind::Invalid);
        CHECK(parse(frame, 14) == ptp::MessageKind::Invalid);
        CHECK(parse(frame, 47) == ptp::MessageKind::Invalid);
        CHECK(parse(frame, 57) == ptp::MessageKind::Invalid);
        CHECK(parse(frame, 58) == ptp::MessageKind::DelayRequest);
        CHECK(parse(frame, 59) == ptp::MessageKind::DelayRequest);
        CHECK(parse(frame, 60) == ptp::MessageKind::DelayRequest);
        CHECK(parse(frame, 61) == ptp::MessageKind::Invalid);

        frame[12] = 0x08;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Ignored);
        frame = make_delay_request();
        frame[0] = 0x02;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Ignored);
        frame = make_delay_request();
        frame[6] = 0x01;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_delay_request();
        std::memset(frame.data() + 6, 0, 6);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_delay_request();
        frame[14] = 2;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Ignored);
        frame = make_delay_request();
        CHECK(parse(frame, frame.size(), 8) == ptp::MessageKind::Ignored);
        const uint8_t local_identity[8] = {1, 2, 3, 4, 5, 6, 7, 8};
        CHECK(parse(frame, frame.size(), 7, local_identity) == ptp::MessageKind::Ignored);
    }

    void test_delay_request_fields()
    {
        Frame frame = make_delay_request();
        uint8_t *message = frame.data() + 14;
        write_u16(message + 2, 43);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        write_u16(message + 2, 45);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);

        frame = make_delay_request();
        message = frame.data() + 14;
        message[0] |= 0x10;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_delay_request();
        message = frame.data() + 14;
        message[1] = 0x22;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_delay_request();
        message = frame.data() + 14;
        message[1] = 0x03;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_delay_request();
        message = frame.data() + 14;
        message[5] = 1;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);

        for (size_t i = 8; i < 16; ++i)
        {
            frame = make_delay_request();
            frame[14 + i] = 1;
            CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        }
        for (size_t i = 16; i < 20; ++i)
        {
            frame = make_delay_request();
            frame[14 + i] = 1;
            CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        }
        frame = make_delay_request();
        frame[14 + 28] = 0;
        frame[14 + 29] = 0;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_delay_request();
        frame[14 + 28] = 0xff;
        frame[14 + 29] = 0xff;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_delay_request();
        std::memset(frame.data() + 14 + 20, 0, 8);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_delay_request();
        std::memset(frame.data() + 14 + 20, 0xff, 8);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_delay_request();
        frame[14 + 32] = 0;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
    }

    void test_version_and_control_compatibility()
    {
        for (const uint8_t type : {uint8_t{1}, uint8_t{11}})
        {
            const uint8_t legacy_control = type == 1 ? 1 : 5;
            const auto expected = type == 1 ? ptp::MessageKind::DelayRequest : ptp::MessageKind::Announce;
            for (const bool allow_authentication : {false, true})
            {
                for (const uint8_t version : {uint8_t{2}, uint8_t{0x12}})
                {
                    for (const uint8_t control : {uint8_t{0}, uint8_t{1}, uint8_t{5}, uint8_t{0xff}})
                    {
                        Frame frame = type == 1 ? make_delay_request() : make_announce();
                        frame[15] = version;
                        frame[14 + 32] = control;
                        ptp::ParsedMessage parsed{};
                        const bool compatible = version == 0x12 || control == legacy_control;
                        CHECK(ptp::parse_l2_frame(frame.data(), frame.size(), 7, nullptr, parsed,
                                                  allow_authentication) ==
                              (compatible ? expected : ptp::MessageKind::Invalid));
                        if (type == 1)
                            CHECK(ptp::parse_l2_frame(frame.data(), 58, 7, nullptr, parsed,
                                                      allow_authentication) ==
                                  (compatible ? expected : ptp::MessageKind::Invalid));
                    }
                }
            }

            Frame frame = type == 1 ? make_delay_request() : make_announce();
            const size_t body = type == 1 ? 44 : 64;
            frame.resize(14 + body + PTP_L2_AUTH_TLV_SIZE);
            frame[15] = 0x12;
            frame[14 + 32] = 0;
            write_u16(frame.data() + 16, static_cast<uint16_t>(body + PTP_L2_AUTH_TLV_SIZE));
            ptp::ParsedMessage parsed{};
            CHECK(ptp::parse_l2_frame(frame.data(), frame.size(), 7, nullptr, parsed, true) == expected);
            CHECK(ptp::parse_l2_frame(frame.data(), frame.size(), 7, nullptr, parsed, false) ==
                  ptp::MessageKind::Invalid);
        }
    }

    void test_announce_validation()
    {
        Frame frame = make_announce();
        ptp::ParsedMessage parsed{};
        CHECK(ptp::parse_l2_frame(frame.data(), frame.size(), 7, nullptr, parsed) == ptp::MessageKind::Announce);
        CHECK(parsed.announce_dataset.priority1 == 128 && parsed.announce_dataset.clock_class == 6);
        CHECK(parsed.announce_dataset.clock_accuracy == 0x20 && parsed.announce_dataset.variance == 0x1234);
        CHECK(parsed.announce_dataset.priority2 == 129 && parsed.announce_dataset.identity[0] == 0xa0);
        CHECK(parsed.steps_removed == 254 && parsed.log_announce_interval == 1);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Announce);
        CHECK(parse(frame, frame.size() - 1) == ptp::MessageKind::Invalid);
        frame.push_back(0);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);

        frame = make_announce();
        uint8_t *message = frame.data() + 14;
        frame[14] |= 0x10;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        message[6] = 0x02;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        message[7] = 0x40;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        message[7] = 0x03;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        message[32] = 4;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        message[46] = 1;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        write_u16(message + 61, 255);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        std::memset(message + 53, 0, 8);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        std::memset(message + 53, 0xff, 8);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        message[48] = 0;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        message[48] = 0xff;
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
        frame = make_announce();
        message = frame.data() + 14;
        write_u32(message + 40, 1000000000U);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);

        frame = make_announce();
        message = frame.data() + 14;
        write_u16(message + 2, 65);
        frame.resize(79, 0);
        CHECK(parse(frame, frame.size()) == ptp::MessageKind::Invalid);
    }

    uint32_t next_random(uint32_t &state)
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }

    void test_deterministic_parser_fuzz()
    {
        uint32_t state = 0x7a39d4b1U;
        Frame frame(128);
        ptp::ParsedMessage parsed{};
        for (size_t iteration = 0; iteration < 30000; ++iteration)
        {
            const size_t actual_size = next_random(state) % (frame.size() + 1);
            for (uint8_t &byte : frame)
                byte = static_cast<uint8_t>(next_random(state));
            if (actual_size >= 14 && (iteration & 1U) == 0)
            {
                const Frame seed = (iteration & 2U) == 0 ? make_delay_request() : make_announce();
                std::memcpy(frame.data(), seed.data(), seed.size());
                frame[14 + (next_random(state) % 64)] ^= static_cast<uint8_t>(next_random(state));
                frame[12] = 0x88;
                frame[13] = 0xf7;
                std::memcpy(frame.data(), seed.data(), 6);
            }
            const size_t length = next_random(state) % (actual_size + 1);
            const ptp::MessageKind result = ptp::parse_l2_frame(frame.data(), length, 7, nullptr, parsed);
            CHECK(result == ptp::MessageKind::Ignored || result == ptp::MessageKind::Invalid ||
                  result == ptp::MessageKind::DelayRequest || result == ptp::MessageKind::Announce);
        }
    }
}

int main()
{
    test_follower_lifecycle();
    test_follower_rollover_snapshots_clear_and_capacity();
    test_follower_report_serialization();
    test_client_continuity_and_deduplication();
    test_client_report_limit_and_first_arrival_order();
    test_frozen_report_periods();
    test_delay_request_validity_and_output();
    test_frame_ethernet_and_dispatch_validation();
    test_delay_request_fields();
    test_version_and_control_compatibility();
    test_announce_validation();
    test_deterministic_parser_fuzz();
    if (failures != 0)
        std::fprintf(stderr, "%d test checks failed\n", failures);
    else
        std::printf("PTP host tests passed\n");
    return failures == 0 ? 0 : 1;
}

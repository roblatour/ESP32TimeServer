// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

// Adapted from https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3 (MIT License)
// Original copyright SparkFun Electronics, 2016

#pragma once

#include <cstddef>
#include <cstdint>

namespace gnss
{
    enum class Status
    {
        ok,
        timeout,
        nak,
        malformed,
        transport_error,
        invalid_argument
    };

    enum class OutputPort
    {
        i2c,
        uart1,
        uart2
    };

    struct TimeSolution
    {
        uint32_t itow = 0;
        uint16_t year = 0;
        uint8_t month = 0;
        uint8_t day = 0;
        uint8_t hour = 0;
        uint8_t minute = 0;
        uint8_t second = 0;
        int32_t nano = 0;
        uint8_t fix_type = 0;
        uint8_t satellites = 0;
        bool fix_ok = false;
        bool date_valid = false;
        bool time_valid = false;
        bool fully_resolved = false;
        uint64_t generation = 0;
        int64_t received_us = 0;
    };

    struct LeapInformation
    {
        uint32_t itow = 0;
        uint8_t version = 0;
        uint8_t valid = 0;
        uint8_t current_source = 0;
        uint8_t change_source = 0;
        int8_t current_leap_seconds = 0;
        int8_t leap_change = 0;
        int32_t time_to_event = 0;
    };

    struct ModuleInformation
    {
        char module_name[14]{};
        char firmware_type[8]{};
        uint8_t firmware_high = 0;
        uint8_t firmware_low = 0;
        uint8_t protocol_high = 0;
        uint8_t protocol_low = 0;
    };

    const char *status_name(Status status);

    // All operations are owned by the GNSS setup/synchronization task.
    class Receiver
    {
    public:
        void reset();
        Status probe(uint32_t timeout_ms);
        Status set_uart1_baud(uint32_t baud, uint32_t timeout_ms);
        Status configure_output(OutputPort port, bool ubx, bool nmea,
                                Status *rtcm_status, uint32_t timeout_ms);
        Status enable_automatic_pvt(uint32_t timeout_ms);
        Status set_stationary_model(uint32_t timeout_ms);
        Status read_dynamic_model(uint8_t &model, uint32_t timeout_ms);
        Status unique_id(char (&id)[13], uint32_t timeout_ms);
        Status module_information(ModuleInformation &info, uint32_t timeout_ms);
        Status leap_information(LeapInformation &info, uint32_t timeout_ms);
        Status discard_pending_pvt();
        Status next_pvt(TimeSolution &solution, uint32_t timeout_ms);

    private:
        static constexpr size_t MAX_PAYLOAD = 512;
        static constexpr size_t MAX_CONFIG_ENTRIES = 4;
        static constexpr size_t MAX_REQUEST_PAYLOAD = 4 + MAX_CONFIG_ENTRIES * 8;
        enum class ParseState : uint8_t
        {
            sync1,
            sync2,
            message_class,
            message_id,
            length_low,
            length_high,
            payload,
            checksum_a,
            checksum_b
        };

        struct Frame
        {
            uint8_t message_class = 0;
            uint8_t message_id = 0;
            uint16_t length = 0;
            uint8_t payload[MAX_PAYLOAD]{};
        };

        bool parse_byte(uint8_t byte);
        void reset_parser();
        void accept_pvt();
        Status drain(int64_t deadline_us);
        Status receive_frame(int64_t deadline_us);
        Status transact(uint8_t message_class, uint8_t message_id,
                        const uint8_t *payload, size_t length,
                        bool expect_ack, int64_t deadline_us,
                        uint32_t expected_key = 0);
        Status write_config(const uint32_t *keys, const uint32_t *values,
                            const uint8_t *widths, size_t count,
                            uint8_t layers, int64_t deadline_us);
        Status read_config(uint32_t key, uint8_t width, uint32_t &value,
                           int64_t deadline_us);

        ParseState state_ = ParseState::sync1;
        Frame frame_{};
        uint16_t offset_ = 0;
        uint8_t checksum_a_ = 0;
        uint8_t checksum_b_ = 0;
        uint8_t received_checksum_a_ = 0;
        TimeSolution pending_pvt_{};
        uint64_t generation_ = 0;
        bool pvt_pending_ = false;
        bool automatic_pvt_ = false;
    };
}

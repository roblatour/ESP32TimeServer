// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

// Adapted from https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3 (MIT License)
// Original copyright SparkFun Electronics, 2016

#include "gnss_receiver.h"

#include <cctype>
#include <cstring>

#include "gnss_uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace gnss
{
    namespace
    {
        // u-blox M10 SPG 5.10 interface description, UBX-21035062 R03.
        constexpr uint8_t CLASS_NAV = 0x01;
        constexpr uint8_t CLASS_ACK = 0x05;
        constexpr uint8_t CLASS_CFG = 0x06;
        constexpr uint8_t CLASS_MON = 0x0a;
        constexpr uint8_t CLASS_SEC = 0x27;
        constexpr uint8_t NAV_PVT = 0x07;
        constexpr uint8_t NAV_TIMELS = 0x26;
        constexpr uint8_t CFG_VALSET = 0x8a;
        constexpr uint8_t CFG_VALGET = 0x8b;
        constexpr uint8_t MON_VER = 0x04;
        constexpr uint8_t SEC_UNIQID = 0x03;
        constexpr uint32_t UART1_BAUD = 0x40520001;
        constexpr uint32_t UART1_INPUT_UBX = 0x10730001;
        constexpr uint32_t PVT_UART1_RATE = 0x20910007;
        constexpr uint32_t DYNAMIC_MODEL = 0x20110021;
        constexpr uint8_t LAYER_RAM = 0x01;
        constexpr uint8_t LAYER_RAM_BBR = 0x03;
        constexpr uint8_t STATIONARY_MODEL = 2;

        bool retain_payload_byte(uint8_t message_class, uint8_t message_id, uint16_t offset)
        {
            switch (message_class)
            {
            case CLASS_NAV:
                return (message_id == NAV_PVT || message_id == NAV_TIMELS) && offset < 24;
            case CLASS_ACK:
                return (message_id == 0 || message_id == 1) && offset < 2;
            case CLASS_CFG:
                return message_id == CFG_VALGET && offset < 12;
            case CLASS_MON:
                return message_id == MON_VER;
            case CLASS_SEC:
                return message_id == SEC_UNIQID && offset < 10;
            default:
                return false;
            }
        }

        uint16_t get_u16(const uint8_t *bytes)
        {
            return static_cast<uint16_t>(bytes[0]) |
                   static_cast<uint16_t>(static_cast<uint16_t>(bytes[1]) << 8);
        }

        uint32_t get_u32(const uint8_t *bytes)
        {
            return static_cast<uint32_t>(bytes[0]) |
                   (static_cast<uint32_t>(bytes[1]) << 8) |
                   (static_cast<uint32_t>(bytes[2]) << 16) |
                   (static_cast<uint32_t>(bytes[3]) << 24);
        }

        int32_t get_i32(const uint8_t *bytes)
        {
            const uint32_t value = get_u32(bytes);
            return static_cast<int32_t>(value <= INT32_MAX
                                            ? static_cast<int64_t>(value)
                                            : static_cast<int64_t>(value) - 0x100000000LL);
        }

        int8_t get_i8(uint8_t value)
        {
            return static_cast<int8_t>(value < 128 ? static_cast<int>(value) : static_cast<int>(value) - 256);
        }

        void put_u32(uint8_t *bytes, uint32_t value)
        {
            for (unsigned i = 0; i < 4; ++i)
                bytes[i] = static_cast<uint8_t>(value >> (8 * i));
        }

        int64_t deadline_after(uint32_t timeout_ms)
        {
            return esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000;
        }

        bool parse_version(const char *text, uint8_t &high, uint8_t &low)
        {
            const auto parse_component = [](const char *&cursor, uint8_t &component)
            {
                while (std::isspace(static_cast<unsigned char>(*cursor)))
                    ++cursor;
                if (*cursor == '+')
                    ++cursor;
                if (*cursor < '0' || *cursor > '9')
                    return false;

                unsigned value = 0;
                do
                {
                    const unsigned digit = static_cast<unsigned>(*cursor - '0');
                    if (value > (255U - digit) / 10U)
                        return false;
                    value = value * 10U + digit;
                    ++cursor;
                } while (*cursor >= '0' && *cursor <= '9');
                component = static_cast<uint8_t>(value);
                return true;
            };

            const char *cursor = text;
            uint8_t major = 0;
            uint8_t minor = 0;
            if (!parse_component(cursor, major) || *cursor != '.')
                return false;
            ++cursor;
            if (!parse_component(cursor, minor))
                return false;
            high = major;
            low = minor;
            return true;
        }
    }

    const char *status_name(Status status)
    {
        switch (status)
        {
        case Status::ok:
            return "ok";
        case Status::timeout:
            return "timeout";
        case Status::nak:
            return "NAK or unsupported";
        case Status::malformed:
            return "malformed response";
        case Status::transport_error:
            return "UART transport failure";
        case Status::invalid_argument:
            return "invalid argument";
        }
        return "unknown status";
    }

    void Receiver::reset_parser()
    {
        state_ = ParseState::sync1;
        offset_ = 0;
        checksum_a_ = 0;
        checksum_b_ = 0;
        received_checksum_a_ = 0;
    }

    void Receiver::reset()
    {
        reset_parser();
        pending_pvt_ = {};
        generation_ = 0;
        pvt_pending_ = false;
        automatic_pvt_ = false;
    }

    void Receiver::accept_pvt()
    {
        if (frame_.length != 92)
            return;

        const uint8_t *data = frame_.payload;
        TimeSolution solution{};
        solution.itow = get_u32(data);
        solution.year = get_u16(data + 4);
        solution.month = data[6];
        solution.day = data[7];
        solution.hour = data[8];
        solution.minute = data[9];
        solution.second = data[10];
        solution.date_valid = (data[11] & 0x01) != 0;
        solution.time_valid = (data[11] & 0x02) != 0;
        solution.fully_resolved = (data[11] & 0x04) != 0;
        solution.nano = get_i32(data + 16);
        solution.fix_type = data[20];
        solution.fix_ok = (data[21] & 0x01) != 0;
        solution.satellites = data[23];
        solution.generation = ++generation_;
        solution.received_us = esp_timer_get_time();
        pending_pvt_ = solution;
        pvt_pending_ = true;
    }

    bool Receiver::parse_byte(uint8_t byte)
    {
        const auto checksum = [this](uint8_t value)
        {
            checksum_a_ = static_cast<uint8_t>(checksum_a_ + value);
            checksum_b_ = static_cast<uint8_t>(checksum_b_ + checksum_a_);
        };
        switch (state_)
        {
        case ParseState::sync1:
            if (byte == 0xb5)
                state_ = ParseState::sync2;
            break;
        case ParseState::sync2:
            state_ = byte == 0x62   ? ParseState::message_class
                     : byte == 0xb5 ? ParseState::sync2
                                    : ParseState::sync1;
            break;
        case ParseState::message_class:
            checksum_a_ = checksum_b_ = 0;
            frame_.message_class = byte;
            checksum(byte);
            state_ = ParseState::message_id;
            break;
        case ParseState::message_id:
            frame_.message_id = byte;
            checksum(byte);
            state_ = ParseState::length_low;
            break;
        case ParseState::length_low:
            frame_.length = byte;
            checksum(byte);
            state_ = ParseState::length_high;
            break;
        case ParseState::length_high:
            frame_.length |= static_cast<uint16_t>(byte) << 8;
            checksum(byte);
            offset_ = 0;
            if (frame_.length > MAX_PAYLOAD)
                reset_parser();
            else
                state_ = frame_.length == 0 ? ParseState::checksum_a : ParseState::payload;
            break;
        case ParseState::payload:
            if (retain_payload_byte(frame_.message_class, frame_.message_id, offset_))
                frame_.payload[offset_] = byte;
            ++offset_;
            checksum(byte);
            if (offset_ == frame_.length)
                state_ = ParseState::checksum_a;
            break;
        case ParseState::checksum_a:
            received_checksum_a_ = byte;
            state_ = ParseState::checksum_b;
            break;
        case ParseState::checksum_b:
        {
            const bool valid = received_checksum_a_ == checksum_a_ && byte == checksum_b_;
            reset_parser();
            if (!valid)
            {
                if (byte == 0xb5)
                    state_ = ParseState::sync2;
                return false;
            }
            if (frame_.message_class == CLASS_NAV && frame_.message_id == NAV_PVT)
                accept_pvt();
            return true;
        }
        }
        return false;
    }

    Status Receiver::drain(int64_t deadline_us)
    {
        for (size_t count = 0; count < 4096; ++count)
        {
            if (esp_timer_get_time() >= deadline_us)
                return Status::timeout;
            size_t available = 0;
            if (gnss_uart_available(&available) != ESP_OK)
                return Status::transport_error;
            if (available == 0)
                return Status::ok;
            uint8_t byte = 0;
            size_t read = 0;
            if (gnss_uart_read(&byte, 1, 0, &read) != ESP_OK)
                return Status::transport_error;
            if (read == 0)
                return Status::ok;
            (void)parse_byte(byte);
        }
        return Status::timeout;
    }

    Status Receiver::receive_frame(int64_t deadline_us)
    {
        while (esp_timer_get_time() < deadline_us)
        {
            uint8_t byte = 0;
            size_t read = 0;
            if (gnss_uart_read(&byte, 1, 0, &read) != ESP_OK)
                return Status::transport_error;
            if (read == 0)
            {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }
            if (parse_byte(byte))
                return Status::ok;
        }
        return Status::timeout;
    }

    Status Receiver::transact(uint8_t message_class, uint8_t message_id,
                              const uint8_t *payload, size_t length,
                              bool expect_ack, int64_t deadline_us,
                              uint32_t expected_key)
    {
        if (length > MAX_REQUEST_PAYLOAD || (payload == nullptr && length != 0))
            return Status::invalid_argument;
        const Status drained = drain(deadline_us);
        if (drained != Status::ok)
            return drained;
        reset_parser();

        uint8_t message[MAX_REQUEST_PAYLOAD + 8]{};
        message[0] = 0xb5;
        message[1] = 0x62;
        message[2] = message_class;
        message[3] = message_id;
        message[4] = static_cast<uint8_t>(length);
        message[5] = static_cast<uint8_t>(length >> 8);
        if (length != 0)
            std::memcpy(message + 6, payload, length);
        uint8_t a = 0;
        uint8_t b = 0;
        for (size_t i = 2; i < length + 6; ++i)
        {
            a = static_cast<uint8_t>(a + message[i]);
            b = static_cast<uint8_t>(b + a);
        }
        message[length + 6] = a;
        message[length + 7] = b;

        size_t sent = 0;
        while (sent < length + 8)
        {
            if (esp_timer_get_time() >= deadline_us)
                return Status::timeout;
            size_t written = 0;
            if (gnss_uart_write(message + sent, length + 8 - sent, &written) != ESP_OK || written == 0)
                return Status::transport_error;
            sent += written;
        }

        for (;;)
        {
            const Status received = receive_frame(deadline_us);
            if (received != Status::ok)
                return received;
            if (frame_.message_class == CLASS_ACK &&
                (frame_.message_id == 0 || frame_.message_id == 1) &&
                frame_.length == 2 && frame_.payload[0] == message_class &&
                frame_.payload[1] == message_id)
            {
                if (frame_.message_id == 0)
                    return Status::nak;
                if (expect_ack)
                    return Status::ok;
                continue;
            }
            if (!expect_ack && frame_.message_class == message_class && frame_.message_id == message_id)
            {
                if (expected_key != 0 && frame_.length >= 8 &&
                    get_u32(frame_.payload + 4) != expected_key)
                    continue;
                return Status::ok;
            }
        }
    }

    Status Receiver::write_config(const uint32_t *keys, const uint32_t *values,
                                  const uint8_t *widths, size_t count,
                                  uint8_t layers, int64_t deadline_us)
    {
        if (keys == nullptr || values == nullptr || widths == nullptr || count == 0 || count > MAX_CONFIG_ENTRIES)
            return Status::invalid_argument;
        uint8_t payload[MAX_REQUEST_PAYLOAD]{};
        payload[1] = layers;
        size_t used = 4;
        for (size_t i = 0; i < count; ++i)
        {
            if (widths[i] != 1 && widths[i] != 4)
                return Status::invalid_argument;
            put_u32(payload + used, keys[i]);
            used += 4;
            for (uint8_t j = 0; j < widths[i]; ++j)
                payload[used++] = static_cast<uint8_t>(values[i] >> (8 * j));
        }
        return transact(CLASS_CFG, CFG_VALSET, payload, used, true, deadline_us);
    }

    Status Receiver::read_config(uint32_t key, uint8_t width, uint32_t &value, int64_t deadline_us)
    {
        if (width != 1 && width != 4)
            return Status::invalid_argument;
        uint8_t payload[8]{};
        put_u32(payload + 4, key);
        const Status result = transact(CLASS_CFG, CFG_VALGET, payload, sizeof(payload),
                                       false, deadline_us, key);
        if (result != Status::ok)
            return result;
        const Frame &response = frame_;
        if (response.length != 8 + width || response.payload[0] != 1 ||
            response.payload[1] != 0 || get_u16(response.payload + 2) != 0 ||
            get_u32(response.payload + 4) != key)
            return Status::malformed;
        value = width == 1 ? response.payload[8] : get_u32(response.payload + 8);
        return Status::ok;
    }

    Status Receiver::probe(uint32_t timeout_ms)
    {
        reset();
        uint32_t value = 0;
        return read_config(UART1_INPUT_UBX, 1, value, deadline_after(timeout_ms));
    }

    Status Receiver::set_uart1_baud(uint32_t baud, uint32_t timeout_ms)
    {
        if (baud == 0)
            return Status::invalid_argument;
        const uint8_t width = 4;
        return write_config(&UART1_BAUD, &baud, &width, 1, LAYER_RAM_BBR, deadline_after(timeout_ms));
    }

    Status Receiver::configure_output(OutputPort port, bool ubx, bool nmea,
                                      Status *rtcm_status, uint32_t timeout_ms)
    {
        uint32_t base = 0;
        switch (port)
        {
        case OutputPort::i2c:
            base = 0x10720000;
            break;
        case OutputPort::uart1:
            base = 0x10740000;
            break;
        case OutputPort::uart2:
            base = 0x10760000;
            break;
        default:
            return Status::invalid_argument;
        }
        if (rtcm_status != nullptr)
            *rtcm_status = Status::timeout;
        const int64_t deadline = deadline_after(timeout_ms);
        const uint32_t keys[] = {base | 1, base | 2};
        const uint32_t values[] = {ubx ? 1U : 0U, nmea ? 1U : 0U};
        const uint8_t widths[] = {1, 1};
        Status result = write_config(keys, values, widths, 2, LAYER_RAM_BBR, deadline);
        for (size_t i = 0; result == Status::ok && i < 2; ++i)
        {
            uint32_t actual = 0;
            result = read_config(keys[i], 1, actual, deadline);
            if (result == Status::ok && actual != values[i])
                result = Status::malformed;
        }
        if (result != Status::ok)
            return result;

        const uint32_t rtcm_key = base | 4;
        const uint32_t disabled = 0;
        const uint8_t width = 1;
        const Status optional = write_config(&rtcm_key, &disabled, &width, 1, LAYER_RAM_BBR, deadline);
        if (rtcm_status != nullptr)
            *rtcm_status = optional;
        return optional == Status::transport_error || optional == Status::invalid_argument ? optional : Status::ok;
    }

    Status Receiver::enable_automatic_pvt(uint32_t timeout_ms)
    {
        automatic_pvt_ = false;
        const int64_t deadline = deadline_after(timeout_ms);
        const uint32_t rate = 1;
        const uint8_t width = 1;
        Status result = write_config(&PVT_UART1_RATE, &rate, &width, 1, LAYER_RAM, deadline);
        uint32_t actual = 0;
        if (result == Status::ok)
            result = read_config(PVT_UART1_RATE, 1, actual, deadline);
        if (result == Status::ok && actual != rate)
            result = Status::malformed;
        automatic_pvt_ = result == Status::ok;
        return result;
    }

    Status Receiver::set_stationary_model(uint32_t timeout_ms)
    {
        const uint32_t model = STATIONARY_MODEL;
        const uint8_t width = 1;
        return write_config(&DYNAMIC_MODEL, &model, &width, 1, LAYER_RAM, deadline_after(timeout_ms));
    }

    Status Receiver::read_dynamic_model(uint8_t &model, uint32_t timeout_ms)
    {
        uint32_t value = 0;
        const Status result = read_config(DYNAMIC_MODEL, 1, value, deadline_after(timeout_ms));
        if (result == Status::ok)
            model = static_cast<uint8_t>(value);
        return result;
    }

    Status Receiver::unique_id(char (&id)[13], uint32_t timeout_ms)
    {
        id[0] = '\0';
        const Status result = transact(CLASS_SEC, SEC_UNIQID, nullptr, 0, false, deadline_after(timeout_ms));
        if (result != Status::ok)
            return result;
        const Frame &response = frame_;
        const size_t count = response.payload[0] == 1 && response.length == 9    ? 5
                             : response.payload[0] == 2 && response.length == 10 ? 6
                                                                                 : 0;
        if (count == 0)
            return Status::malformed;
        static constexpr char HEX[] = "0123456789ABCDEF";
        for (size_t i = 0; i < count; ++i)
        {
            id[2 * i] = HEX[response.payload[4 + i] >> 4];
            id[2 * i + 1] = HEX[response.payload[4 + i] & 0x0f];
        }
        id[2 * count] = '\0';
        return Status::ok;
    }

    Status Receiver::module_information(ModuleInformation &info, uint32_t timeout_ms)
    {
        const Status result = transact(CLASS_MON, MON_VER, nullptr, 0, false, deadline_after(timeout_ms));
        if (result != Status::ok)
            return result;
        const Frame &response = frame_;
        if (response.length < 40 || (response.length - 40) % 30 != 0)
            return Status::malformed;

        ModuleInformation parsed{};
        bool firmware_found = false;
        bool protocol_found = false;
        bool module_found = false;
        for (size_t offset = 40; offset < response.length; offset += 30)
        {
            char extension[31]{};
            std::memcpy(extension, response.payload + offset, 30);
            if (const char *entry = std::strstr(extension, "FWVER="))
            {
                entry += 6;
                size_t length = 0;
                while (length < sizeof(parsed.firmware_type) - 1 && entry[length] != '\0' && entry[length] != ' ')
                    ++length;
                std::memcpy(parsed.firmware_type, entry, length);
                parsed.firmware_type[length] = '\0';
                entry += length;
                if (*entry == ' ')
                    ++entry;
                firmware_found = parse_version(entry, parsed.firmware_high, parsed.firmware_low);
            }
            if (const char *entry = std::strstr(extension, "PROTVER="))
                protocol_found = parse_version(entry + 8, parsed.protocol_high, parsed.protocol_low);
            if (const char *entry = std::strstr(extension, "MOD="))
            {
                entry += 4;
                size_t length = 0;
                while (length < sizeof(parsed.module_name) - 1 && entry[length] != '\0' && entry[length] != ' ')
                    ++length;
                std::memcpy(parsed.module_name, entry, length);
                parsed.module_name[length] = '\0';
                module_found = length != 0;
            }
        }
        if (!module_found)
            std::memcpy(parsed.module_name, "NONE", sizeof("NONE"));
        if (!firmware_found || !protocol_found)
            return Status::malformed;
        info = parsed;
        return Status::ok;
    }

    Status Receiver::leap_information(LeapInformation &info, uint32_t timeout_ms)
    {
        const Status result = transact(CLASS_NAV, NAV_TIMELS, nullptr, 0, false, deadline_after(timeout_ms));
        if (result != Status::ok)
            return result;
        const Frame &response = frame_;
        if (response.length != 24 || response.payload[4] != 0)
            return Status::malformed;
        const uint8_t *data = response.payload;
        LeapInformation parsed{};
        parsed.itow = get_u32(data);
        parsed.version = data[4];
        parsed.current_source = data[8];
        parsed.current_leap_seconds = get_i8(data[9]);
        parsed.change_source = data[10];
        parsed.leap_change = get_i8(data[11]);
        parsed.time_to_event = get_i32(data + 12);
        parsed.valid = data[23];
        info = parsed;
        return Status::ok;
    }

    Status Receiver::discard_pending_pvt()
    {
        const Status result = drain(deadline_after(200));
        pvt_pending_ = false;
        reset_parser();
        return result;
    }

    Status Receiver::next_pvt(TimeSolution &solution, uint32_t timeout_ms)
    {
        const int64_t deadline = deadline_after(timeout_ms);
        if (pvt_pending_)
        {
            solution = pending_pvt_;
            pvt_pending_ = false;
            return Status::ok;
        }
        if (!automatic_pvt_)
        {
            const Status result = transact(CLASS_NAV, NAV_PVT, nullptr, 0, false, deadline);
            if (result != Status::ok)
                return result;
            if (frame_.length != 92 || !pvt_pending_)
                return Status::malformed;
        }
        else
        {
            while (!pvt_pending_)
            {
                const Status result = receive_frame(deadline);
                if (result != Status::ok)
                    return result;
                if (frame_.message_class == CLASS_NAV && frame_.message_id == NAV_PVT && frame_.length != 92)
                    return Status::malformed;
            }
        }
        solution = pending_pvt_;
        pvt_pending_ = false;
        return Status::ok;
    }
}

// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include "ptp_l2_auth.h"
#include "ptp_l2_master_selection.h"

namespace ptp
{
enum class MessageKind
{
    Ignored,
    Invalid,
    DelayRequest,
    Announce
};

struct ParsedMessage
{
    uint8_t domain;
    uint8_t source_port_identity[10];
    uint16_t sequence_id;
    MasterDataset announce_dataset;
    uint16_t steps_removed;
    int8_t log_announce_interval;
};

namespace detail
{
inline uint16_t read_u16(const uint8_t *data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

inline bool all_bytes_equal(const uint8_t *data, size_t length, uint8_t value)
{
    for (size_t i = 0; i < length; ++i)
    {
        if (data[i] != value)
            return false;
    }
    return true;
}

inline bool valid_clock_identity(const uint8_t *identity)
{
    return !all_bytes_equal(identity, 8, 0) && !all_bytes_equal(identity, 8, 0xff);
}
}

inline MessageKind parse_message(const uint8_t *message, size_t length, uint8_t domain,
                                 const uint8_t *local_clock_identity, ParsedMessage &parsed,
                                 bool allow_authentication = false)
{
    if (message == nullptr || length < 34)
        return MessageKind::Invalid;

    const uint8_t type = message[0] & 0x0f;
    if (type != 1 && type != 11)
        return MessageKind::Ignored;
    if (message[4] != domain ||
        (local_clock_identity != nullptr && std::memcmp(message + 20, local_clock_identity, 8) == 0))
        return MessageKind::Ignored;

    const uint16_t message_length = detail::read_u16(message + 2);
    const size_t expected_length = type == 1 ? 44 : 64;
    const bool signed_length = allow_authentication && message_length == expected_length + PTP_L2_AUTH_TLV_SIZE;
    if ((message_length != expected_length && !signed_length) || length != message_length)
        return MessageKind::Invalid;
    if ((message[0] & 0xf0) != 0 || (message[1] != 2 && message[1] != 0x12) || message[5] != 0 ||
        !detail::all_bytes_equal(message + 16, 4, 0))
        return MessageKind::Invalid;

    const uint16_t source_port = detail::read_u16(message + 28);
    if (!detail::valid_clock_identity(message + 20) || source_port == 0 || source_port == 0xffff)
        return MessageKind::Invalid;

    const uint32_t nanoseconds = (static_cast<uint32_t>(message[40]) << 24) |
                                 (static_cast<uint32_t>(message[41]) << 16) |
                                 (static_cast<uint32_t>(message[42]) << 8) | message[43];
    if (nanoseconds >= 1000000000U)
        return MessageKind::Invalid;

    // PTP v2.1 deprecates controlField; dispatch uses messageType instead.
    if (message[1] == 2 && message[32] != (type == 1 ? 1 : 5))
        return MessageKind::Invalid;

    if (type == 1)
    {
        if (message[6] != 0 || message[7] != 0 ||
            !detail::all_bytes_equal(message + 8, 8, 0))
            return MessageKind::Invalid;
    }
    else
    {
        if ((message[6] & 0xfe) != 0 || (message[7] & 0xc0) != 0 ||
            (message[7] & 0x03) == 0x03 || message[46] != 0 ||
            detail::read_u16(message + 61) >= 255 ||
            !detail::valid_clock_identity(message + 53) || message[48] == 0 || message[48] == 255)
            return MessageKind::Invalid;
    }

    parsed.domain = message[4];
    std::memcpy(parsed.source_port_identity, message + 20, sizeof(parsed.source_port_identity));
    parsed.sequence_id = detail::read_u16(message + 30);
    if (type == 11)
    {
        parsed.announce_dataset.priority1 = message[47];
        parsed.announce_dataset.clock_class = message[48];
        parsed.announce_dataset.clock_accuracy = message[49];
        parsed.announce_dataset.variance = detail::read_u16(message + 50);
        parsed.announce_dataset.priority2 = message[52];
        std::memcpy(parsed.announce_dataset.identity, message + 53, sizeof(parsed.announce_dataset.identity));
        parsed.steps_removed = detail::read_u16(message + 61);
        parsed.log_announce_interval = static_cast<int8_t>(message[33]);
    }
    return type == 1 ? MessageKind::DelayRequest : MessageKind::Announce;
}

inline MessageKind parse_l2_frame(const uint8_t *frame, size_t length, uint8_t domain,
                                  const uint8_t *local_clock_identity, ParsedMessage &parsed,
                                  bool allow_authentication = false)
{
    if (frame == nullptr || length < 14)
        return MessageKind::Invalid;
    if (frame[12] != 0x88 || frame[13] != 0xf7)
        return MessageKind::Ignored;
    static constexpr uint8_t multicast_mac[6] = {0x01, 0x1b, 0x19, 0, 0, 0};
    if (std::memcmp(frame, multicast_mac, 6) != 0)
        return MessageKind::Ignored;
    if (length < 48 || (frame[6] & 1) != 0 || detail::all_bytes_equal(frame + 6, 6, 0))
        return MessageKind::Invalid;
    const size_t message_length = detail::read_u16(frame + 16);
    const size_t frame_length = 14 + message_length;
    if (length < frame_length || length > (frame_length < 60 ? 60 : frame_length))
        return MessageKind::Invalid;
    return parse_message(frame + 14, message_length, domain, local_clock_identity, parsed, allow_authentication);
}
}

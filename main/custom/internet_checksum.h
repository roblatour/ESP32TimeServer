#pragma once

#include <cstddef>
#include <cstdint>

namespace net_packet
{
inline uint32_t add(uint32_t sum, const uint8_t *data, size_t length)
{
    while (length >= 2)
    {
        sum += static_cast<uint16_t>((data[0] << 8) | data[1]);
        data += 2;
        length -= 2;
    }
    return length == 0 ? sum : sum + static_cast<uint16_t>(data[0] << 8);
}

inline uint16_t fold(uint32_t sum)
{
    while (sum >> 16)
        sum = (sum & 0xffffU) + (sum >> 16);
    return static_cast<uint16_t>(sum);
}

inline uint16_t checksum(const uint8_t *data, size_t length)
{
    return static_cast<uint16_t>(~fold(add(0, data, length)));
}

inline uint32_t udp_pseudo_sum(const uint8_t *source, const uint8_t *destination, size_t address_length, size_t length)
{
    return add(add(0, source, address_length), destination, address_length) +
           17 + static_cast<uint32_t>(length & 0xffffU) + static_cast<uint32_t>(length >> 16);
}

inline uint32_t udp_sum(const uint8_t *source, const uint8_t *destination, size_t address_length,
                        const uint8_t *udp, size_t length)
{
    return add(udp_pseudo_sum(source, destination, address_length, length), udp, length);
}
}

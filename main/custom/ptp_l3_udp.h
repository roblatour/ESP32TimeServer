#pragma once

#include "internet_checksum.h"
#include "ptp_l2_protocol.h"

namespace ptp_l3_udp
{
static constexpr uint16_t EVENT_PORT = 319;
static constexpr uint16_t GENERAL_PORT = 320;
static constexpr size_t MAX_FRAME_SIZE = 14 + 40 + 8 + 64;
static constexpr uint8_t IPV4_GROUP[4] = {224, 0, 1, 129};
static constexpr uint8_t IPV6_GROUP[16] = {0xff, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0x81};
static constexpr uint8_t IPV4_MAC[6] = {1, 0, 0x5e, 0, 1, 0x81};
static constexpr uint8_t IPV6_MAC[6] = {0x33, 0x33, 0, 0, 1, 0x81};

inline void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value >> 8);
    data[1] = static_cast<uint8_t>(value);
}

struct Datagram
{
    const uint8_t *message = nullptr;
    size_t length = 0;
    bool ipv6 = false;
    uint16_t port = 0;
};

// Only this multicast profile is intercepted. Other UDP traffic remains owned by lwIP.
inline ptp::MessageKind decode(const uint8_t *frame, size_t length, uint8_t domain,
                              const uint8_t *identity, Datagram &datagram, ptp::ParsedMessage &parsed)
{
    if (frame == nullptr || length < 14)
        return ptp::MessageKind::Ignored;
    const bool ipv6 = frame[12] == 0x86 && frame[13] == 0xdd;
    const bool ipv4 = frame[12] == 8 && frame[13] == 0;
    if ((!ipv4 && !ipv6) || length < (ipv6 ? 62U : 42U))
        return ptp::MessageKind::Ignored;
    const uint8_t *ip = frame + 14;
    const size_t ip_header = ipv6 ? 40 : (ip[0] & 15) * 4U;
    if ((ip[0] >> 4) != (ipv6 ? 6 : 4) || ip_header < (ipv6 ? 40U : 20U) ||
        length < 14 + ip_header + 8 ||
        (ipv6 ? ip[6] : ip[9]) != 17 ||
        std::memcmp(ip + (ipv6 ? 24 : 16), ipv6 ? IPV6_GROUP : IPV4_GROUP, ipv6 ? 16 : 4) != 0)
        return ptp::MessageKind::Ignored;
    const uint8_t *udp = ip + ip_header;
    const uint16_t port = ptp::detail::read_u16(udp + 2);
    if (port != EVENT_PORT && port != GENERAL_PORT)
        return ptp::MessageKind::Ignored;
    const size_t ip_length = ipv6 ? 40 + ptp::detail::read_u16(ip + 4) : ptp::detail::read_u16(ip + 2);
    const size_t udp_length = ptp::detail::read_u16(udp + 4);
    if (ip_length > length - 14 || udp_length < 8 || ip_length != ip_header + udp_length ||
        std::memcmp(frame, ipv6 ? IPV6_MAC : IPV4_MAC, 6) != 0 ||
        (frame[6] & 1) != 0 || ptp::detail::all_bytes_equal(frame + 6, 6, 0) ||
        (ipv6 ? (ip[7] == 0 || ip[8] == 0xff || ptp::detail::all_bytes_equal(ip + 8, 16, 0))
              : ((ptp::detail::read_u16(ip + 6) & 0xbfff) != 0 ||
                 net_packet::checksum(ip, ip_header) != 0 || ip[12] == 0 || ip[12] >= 224)) ||
        (ipv6 && udp[6] == 0 && udp[7] == 0) ||
        ((ipv6 || udp[6] != 0 || udp[7] != 0) &&
         net_packet::fold(net_packet::udp_sum(ip + (ipv6 ? 8 : 12), ip + (ipv6 ? 24 : 16),
                                             ipv6 ? 16 : 4, udp, udp_length)) != 0xffff))
        return ptp::MessageKind::Invalid;
    datagram = {udp + 8, udp_length - 8, ipv6, port};
    const auto kind = ptp::parse_message(datagram.message, datagram.length, domain, identity, parsed);
    if ((kind == ptp::MessageKind::DelayRequest && port != EVENT_PORT) ||
        (kind == ptp::MessageKind::Announce && port != GENERAL_PORT))
        return ptp::MessageKind::Invalid;
    return kind;
}

inline size_t encapsulate(uint8_t *frame, size_t capacity, const uint8_t *source_mac,
                          const uint8_t *source_ip, bool ipv6, const uint8_t *message, size_t message_length)
{
    const size_t ip_header = ipv6 ? 40 : 20;
    const size_t udp_length = 8 + message_length;
    const size_t frame_length = 14 + ip_header + udp_length;
    if (frame == nullptr || source_mac == nullptr || source_ip == nullptr || message == nullptr ||
        message_length < 34 || message_length > 64 || capacity < frame_length)
        return 0;
    std::memset(frame, 0, frame_length);
    std::memcpy(frame, ipv6 ? IPV6_MAC : IPV4_MAC, 6);
    std::memcpy(frame + 6, source_mac, 6);
    write_u16(frame + 12, ipv6 ? 0x86dd : 0x0800);
    uint8_t *ip = frame + 14;
    ip[0] = ipv6 ? 0x60 : 0x45;
    write_u16(ip + (ipv6 ? 4 : 2), static_cast<uint16_t>(ipv6 ? udp_length : ip_header + udp_length));
    ip[ipv6 ? 6 : 9] = 17;
    ip[ipv6 ? 7 : 8] = 1;
    std::memcpy(ip + (ipv6 ? 8 : 12), source_ip, ipv6 ? 16 : 4);
    std::memcpy(ip + (ipv6 ? 24 : 16), ipv6 ? IPV6_GROUP : IPV4_GROUP, ipv6 ? 16 : 4);
    if (!ipv6)
        write_u16(ip + 10, net_packet::checksum(ip, ip_header));
    uint8_t *udp = ip + ip_header;
    const uint16_t port = (message[0] & 15) == 0 || (message[0] & 15) == 1 ? EVENT_PORT : GENERAL_PORT;
    write_u16(udp, port);
    write_u16(udp + 2, port);
    write_u16(udp + 4, static_cast<uint16_t>(udp_length));
    std::memcpy(udp + 8, message, message_length);
    const uint16_t checksum = static_cast<uint16_t>(~net_packet::fold(
        net_packet::udp_sum(ip + (ipv6 ? 8 : 12), ip + (ipv6 ? 24 : 16), ipv6 ? 16 : 4, udp, udp_length)));
    write_u16(udp + 6, checksum == 0 ? 0xffff : checksum);
    return frame_length;
}
}

// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

namespace ptp
{
    static constexpr size_t DEFAULT_CLIENT_REPORT_LIMIT = 64;

    // Upper bounds used to size MQTT report buffers. The section header with four maximum
    // counters needs under 140 bytes; each MAC record needs at most 57 bytes including its separator.
    static constexpr size_t FOLLOWER_REPORT_HEADER_BYTES = 160;
    static constexpr size_t FOLLOWER_REPORT_RECORD_BYTES = 64;

    constexpr size_t follower_report_capacity(size_t client_limit)
    {
        return FOLLOWER_REPORT_HEADER_BYTES + client_limit * FOLLOWER_REPORT_RECORD_BYTES;
    }

    static constexpr size_t FOLLOWER_REPORT_CAPACITY = follower_report_capacity(DEFAULT_CLIENT_REPORT_LIMIT);

    struct Client
    {
        uint8_t mac[6]{};
        bool active_entire_period = false;
    };

    template <size_t ClientLimit = DEFAULT_CLIENT_REPORT_LIMIT>
    struct BasicSnapshot
    {
        static constexpr size_t client_limit = ClientLimit;

        uint32_t current;
        uint32_t min;
        uint32_t max;
        uint32_t overflow_drops;
        std::array<Client, ClientLimit> clients{};
        size_t client_count = 0;
    };

    using Snapshot = BasicSnapshot<>;

    template <size_t ClientLimit>
    int format_follower_report(char *output, size_t capacity, const BasicSnapshot<ClientLimit> &snapshot)
    {
        int length = 0;
        bool failed = false;
        const auto append = [&](const char *format, auto... arguments)
        {
            const size_t offset = static_cast<size_t>(length);
            const int written = offset < capacity
                                    ? std::snprintf(output + offset, capacity - offset, format, arguments...)
                                    : std::snprintf(nullptr, 0, format, arguments...);
            if (written < 0)
                failed = true;
            else
                length += written;
        };
        append("\"ptp\":{\"followers\":{\"current\":%u,\"min\":%u,\"max\":%u},\"clients\":[",
               static_cast<unsigned>(snapshot.current), static_cast<unsigned>(snapshot.min),
               static_cast<unsigned>(snapshot.max));
        for (size_t i = 0; i < snapshot.client_count; ++i)
        {
            const auto &client = snapshot.clients[i];
            append("%s{\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\",\"active_entire_period\":%s}",
                   i == 0 ? "" : ",",
                   static_cast<unsigned>(client.mac[0]), static_cast<unsigned>(client.mac[1]),
                   static_cast<unsigned>(client.mac[2]), static_cast<unsigned>(client.mac[3]),
                   static_cast<unsigned>(client.mac[4]), static_cast<unsigned>(client.mac[5]),
                   client.active_entire_period ? "true" : "false");
        }
        append("],\"follower_overflow_drops\":%u}", static_cast<unsigned>(snapshot.overflow_drops));
        return failed ? -1 : length;
    }

    template <size_t Capacity, size_t ClientLimit = DEFAULT_CLIENT_REPORT_LIMIT>
    class FollowerTable
    {
        static_assert(Capacity <= std::numeric_limits<uint32_t>::max(), "Follower capacity exceeds counter range");
        static_assert(Capacity <= ClientLimit, "Continuous followers must fit in the client report");

    public:
        using Snapshot = BasicSnapshot<ClientLimit>;

        explicit FollowerTable(int64_t timeout_us)
            : timeout_us_(timeout_us > 0 ? static_cast<uint64_t>(timeout_us) : 0)
        {
        }

        bool observe(uint8_t domain, const uint8_t *source_port_identity, int64_t now_us,
                     const uint8_t *source_mac, uint8_t transport = 0, int64_t timeout_us = 0,
                     size_t transport_capacity = Capacity)
        {
            expire(now_us);
            if (source_port_identity == nullptr || source_mac == nullptr)
                return false;

            Entry *available = nullptr;
            size_t transport_count = 0;
            for (auto &entry : entries_)
            {
                if (entry.active && entry.transport == transport)
                    ++transport_count;
                if (!entry.active)
                {
                    if (available == nullptr)
                        available = &entry;
                }
                else if (entry.transport == transport && entry.domain == domain &&
                         std::memcmp(entry.identity, source_port_identity, sizeof(entry.identity)) == 0)
                {
                    if (std::memcmp(entry.mac, source_mac, sizeof(entry.mac)) != 0)
                    {
                        entry.active = false;
                        note_departure(entry.mac);
                        std::memcpy(entry.mac, source_mac, sizeof(entry.mac));
                        entry.active = true;
                        note_client(source_mac, false);
                    }
                    entry.last_seen_us = now_us;
                    entry.timeout_us = timeout_us > 0 ? static_cast<uint64_t>(timeout_us) : timeout_us_;
                    return true;
                }
            }

            if (available == nullptr || transport_count >= transport_capacity)
            {
                if (snapshot_.overflow_drops < std::numeric_limits<uint32_t>::max())
                    ++snapshot_.overflow_drops;
                return false;
            }

            available->domain = domain;
            available->transport = transport;
            available->timeout_us = timeout_us > 0 ? static_cast<uint64_t>(timeout_us) : timeout_us_;
            std::memcpy(available->identity, source_port_identity, sizeof(available->identity));
            std::memcpy(available->mac, source_mac, sizeof(available->mac));
            available->last_seen_us = now_us;
            available->active = true;
            ++snapshot_.current;
            note_client(source_mac, false);
            update_extrema();
            return true;
        }

        void expire(int64_t now_us)
        {
            for (auto &entry : entries_)
            {
                if (entry.active && now_us >= entry.last_seen_us &&
                    static_cast<uint64_t>(now_us) - static_cast<uint64_t>(entry.last_seen_us) >= entry.timeout_us)
                {
                    entry.active = false;
                    note_departure(entry.mac);
                    --snapshot_.current;
                }
            }
            update_extrema();
        }

        void clear()
        {
            for (auto &entry : entries_)
                entry.active = false;
            for (size_t i = 0; i < snapshot_.client_count; ++i)
                snapshot_.clients[i].active_entire_period = false;
            snapshot_.current = 0;
            update_extrema();
        }

        void clear_transport(uint8_t transport)
        {
            for (auto &entry : entries_)
            {
                if (entry.active && entry.transport == transport)
                {
                    entry.active = false;
                    note_departure(entry.mac);
                    --snapshot_.current;
                }
            }
            update_extrema();
        }

        Snapshot snapshot_and_reset(int64_t now_us)
        {
            expire(now_us);
            const Snapshot result = snapshot_;
            snapshot_.min = snapshot_.current;
            snapshot_.max = snapshot_.current;
            snapshot_.overflow_drops = 0;
            snapshot_.client_count = 0;
            // Seed all boundary-active MACs before admitting transient arrivals.
            for (const auto &entry : entries_)
                if (entry.active)
                    note_client(entry.mac, true);
            return result;
        }

    private:
        struct Entry
        {
            uint8_t domain = 0;
            uint8_t transport = 0;
            uint64_t timeout_us = 0;
            uint8_t identity[10]{};
            uint8_t mac[6]{};
            int64_t last_seen_us = 0;
            bool active = false;
        };

        void note_client(const uint8_t *mac, bool entire_period)
        {
            for (size_t i = 0; i < snapshot_.client_count; ++i)
                if (std::memcmp(snapshot_.clients[i].mac, mac, 6) == 0)
                    return;
            if (snapshot_.client_count == ClientLimit)
                return;
            auto &client = snapshot_.clients[snapshot_.client_count++];
            std::memcpy(client.mac, mac, sizeof(client.mac));
            client.active_entire_period = entire_period;
        }

        void note_departure(const uint8_t *mac)
        {
            for (const auto &entry : entries_)
                if (entry.active && std::memcmp(entry.mac, mac, sizeof(entry.mac)) == 0)
                    return;
            for (size_t i = 0; i < snapshot_.client_count; ++i)
                if (std::memcmp(snapshot_.clients[i].mac, mac, 6) == 0)
                    snapshot_.clients[i].active_entire_period = false;
        }

        void update_extrema()
        {
            if (snapshot_.current < snapshot_.min)
                snapshot_.min = snapshot_.current;
            if (snapshot_.current > snapshot_.max)
                snapshot_.max = snapshot_.current;
        }

        std::array<Entry, Capacity> entries_{};
        uint64_t timeout_us_;
        Snapshot snapshot_{};
    };
}

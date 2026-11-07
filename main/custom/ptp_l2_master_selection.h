// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace ptp
{
struct MasterDataset
{
    uint8_t priority1 = 128;
    uint8_t clock_class = 248;
    uint8_t clock_accuracy = 0xfe;
    uint16_t variance = 0xffff;
    uint8_t priority2 = 128;
    uint8_t identity[8]{};
};

inline int compare_master_datasets(const MasterDataset &left, const MasterDataset &right)
{
    if (left.priority1 != right.priority1)
        return left.priority1 < right.priority1 ? -1 : 1;
    if (left.clock_class != right.clock_class)
        return left.clock_class < right.clock_class ? -1 : 1;
    if (left.clock_accuracy != right.clock_accuracy)
        return left.clock_accuracy < right.clock_accuracy ? -1 : 1;
    if (left.variance != right.variance)
        return left.variance < right.variance ? -1 : 1;
    if (left.priority2 != right.priority2)
        return left.priority2 < right.priority2 ? -1 : 1;
    return std::memcmp(left.identity, right.identity, sizeof(left.identity));
}

enum class SelectionRole
{
    Listening,
    Master,
    Standby
};

template <size_t Capacity>
class MasterSelection
{
public:
    MasterSelection(bool enabled, int64_t timeout_us, int64_t startup_wait_us)
        : enabled_(enabled), timeout_us_(static_cast<uint64_t>(timeout_us)),
          startup_wait_us_(static_cast<uint64_t>(startup_wait_us))
    {
    }

    void reset(const MasterDataset &local, int64_t now_us)
    {
        local_ = local;
        entries_ = {};
        started_us_ = now_us;
        legacy_seen_ = false;
        initialized_ = true;
    }

    void observe(const MasterDataset &dataset, const uint8_t *source, uint16_t sequence,
                 int8_t log_interval, int64_t now_us)
    {
        if (!initialized_ || source == nullptr)
        {
            increment(unsupported_announces_);
            return;
        }
        if (!enabled_)
        {
            legacy_seen_ = true;
            legacy_last_us_ = now_us;
            return;
        }
        expire(now_us);
        Entry *existing = nullptr;
        Entry *available = nullptr;
        Entry *replaceable = nullptr;
        for (auto &entry : entries_)
        {
            if (!entry.active)
                available = &entry;
            else
            {
                if (std::memcmp(entry.source, source, sizeof(entry.source)) == 0)
                    existing = &entry;
                if (entry.count < 2 && (replaceable == nullptr ||
                    compare_master_datasets(entry.dataset, replaceable->dataset) > 0))
                    replaceable = &entry;
            }
        }
        if (log_interval < -7 || log_interval > 4)
        {
            increment(unsupported_announces_);
            return;
        }
        if (existing != nullptr)
        {
            const uint16_t advance = static_cast<uint16_t>(sequence - existing->sequence);
            if (advance == 0 || advance >= 0x8000)
                return;
        }
        // Echoes of our grandmaster identity must not make us stand aside.
        if (std::memcmp(dataset.identity, local_.identity, sizeof(dataset.identity)) == 0 ||
            compare_master_datasets(dataset, local_) >= 0)
        {
            if (existing != nullptr)
                existing->active = false;
            return;
        }
        Entry *entry = existing != nullptr ? existing : available;
        if (entry == nullptr)
        {
            increment(overflow_drops_);
            // Never evict a qualified better master for a single new Announce.
            if (replaceable == nullptr || compare_master_datasets(dataset, replaceable->dataset) >= 0)
                return;
            entry = replaceable;
            entry->active = false;
        }
        if (!entry->active || std::memcmp(entry->dataset.identity, dataset.identity, sizeof(dataset.identity)) != 0)
            entry->count = 0;
        if (entry->count < 2)
            ++entry->count;
        entry->dataset = dataset;
        std::memcpy(entry->source, source, sizeof(entry->source));
        entry->sequence = sequence;
        entry->last_seen_us = now_us;
        const uint64_t interval_us = log_interval >= 0
                                         ? uint64_t{1000000} << log_interval
                                         : (uint64_t{1000000} + (uint64_t{1} << -log_interval) - 1) >> -log_interval;
        entry->timeout_us = interval_us * 3 > timeout_us_ ? interval_us * 3 : timeout_us_;
        entry->active = true;
    }

    SelectionRole role(int64_t now_us)
    {
        if (!initialized_)
            return SelectionRole::Listening;
        if (!enabled_)
            return legacy_seen_ && (now_us < legacy_last_us_ ||
                   static_cast<uint64_t>(now_us) - static_cast<uint64_t>(legacy_last_us_) <= timeout_us_)
                       ? SelectionRole::Standby : SelectionRole::Master;
        expire(now_us);
        if (now_us < started_us_ ||
            static_cast<uint64_t>(now_us) - static_cast<uint64_t>(started_us_) < startup_wait_us_)
            return SelectionRole::Listening;
        for (const auto &entry : entries_)
            if (entry.active && entry.count >= 2)
                return SelectionRole::Standby;
        return SelectionRole::Master;
    }

    uint32_t overflow_drops() const { return overflow_drops_; }
    uint32_t unsupported_announces() const { return unsupported_announces_; }

private:
    struct Entry
    {
        MasterDataset dataset{};
        uint8_t source[10]{};
        uint16_t sequence = 0;
        uint8_t count = 0;
        int64_t last_seen_us = 0;
        uint64_t timeout_us = 0;
        bool active = false;
    };

    static void increment(uint32_t &counter)
    {
        if (counter < std::numeric_limits<uint32_t>::max())
            ++counter;
    }

    void expire(int64_t now_us)
    {
        for (auto &entry : entries_)
            if (entry.active && now_us >= entry.last_seen_us &&
                static_cast<uint64_t>(now_us) - static_cast<uint64_t>(entry.last_seen_us) >= entry.timeout_us)
                entry.active = false;
    }

    std::array<Entry, Capacity> entries_{};
    MasterDataset local_{};
    bool enabled_;
    uint64_t timeout_us_;
    uint64_t startup_wait_us_;
    bool initialized_ = false;
    int64_t started_us_ = 0;
    bool legacy_seen_ = false;
    int64_t legacy_last_us_ = 0;
    uint32_t overflow_drops_ = 0;
    uint32_t unsupported_announces_ = 0;
};
}

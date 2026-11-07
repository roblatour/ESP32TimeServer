// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace mqtt_queue
{
    // Fixed-size circular byte queue holding variable-length reports. Each record is a 4-byte length
    // followed by the report bytes (no terminator), padded to 4 bytes, and is never split across the
    // end of the buffer, so a record can be published in place. When a new record does not fit, the
    // oldest records are dropped until it does. Not thread-safe; callers serialize access.
    template <size_t Capacity>
    class ReportQueue
    {
        static_assert(Capacity >= 8 && Capacity % 4 == 0, "Queue capacity must be a multiple of 4 bytes");

    public:
        static constexpr size_t HEADER_BYTES = sizeof(uint32_t);
        static constexpr uint32_t WRAP_MARKER = UINT32_MAX;

        static constexpr size_t record_bytes(size_t length)
        {
            return (HEADER_BYTES + length + 3) & ~static_cast<size_t>(3);
        }

        static constexpr size_t max_payload_bytes()
        {
            return Capacity - HEADER_BYTES;
        }

        // Returns false, leaving the queue unchanged, only if the report can never fit.
        bool push(const char *data, size_t length, uint32_t &discarded)
        {
            if (data == nullptr || length > max_payload_bytes())
                return false;
            const size_t needed = record_bytes(length);
            size_t offset = 0;
            while (!find_space(needed, offset))
            {
                pop();
                ++discarded;
            }
            if (offset == 0 && count_ > 0 && tail_ != 0 && Capacity - tail_ >= HEADER_BYTES)
                write_length(tail_, WRAP_MARKER);
            write_length(offset, static_cast<uint32_t>(length));
            std::memcpy(buffer_ + offset + HEADER_BYTES, data, length);
            tail_ = offset + needed;
            used_ += needed;
            ++count_;
            return true;
        }

        bool front(const char *&data, size_t &length) const
        {
            if (count_ == 0)
                return false;
            length = read_length(head_);
            data = reinterpret_cast<const char *>(buffer_ + head_ + HEADER_BYTES);
            return true;
        }

        void pop()
        {
            if (count_ == 0)
                return;
            const size_t bytes = record_bytes(read_length(head_));
            head_ += bytes;
            used_ -= bytes;
            if (--count_ == 0)
            {
                head_ = tail_ = 0;
                return;
            }
            if (Capacity - head_ < HEADER_BYTES || read_length(head_) == WRAP_MARKER)
                head_ = 0;
        }

        size_t count() const { return count_; }
        size_t used_bytes() const { return used_; }
        static constexpr size_t capacity() { return Capacity; }

    private:
        bool find_space(size_t needed, size_t &offset) const
        {
            if (count_ == 0)
            {
                offset = 0;
                return needed <= Capacity;
            }
            if (tail_ > head_)
            {
                if (needed <= Capacity - tail_)
                {
                    offset = tail_;
                    return true;
                }
                offset = 0;
                return needed <= head_;
            }
            offset = tail_;
            return needed <= head_ - tail_;
        }

        uint32_t read_length(size_t offset) const
        {
            uint32_t value = 0;
            std::memcpy(&value, buffer_ + offset, sizeof(value));
            return value;
        }

        void write_length(size_t offset, uint32_t value)
        {
            std::memcpy(buffer_ + offset, &value, sizeof(value));
        }

        alignas(4) uint8_t buffer_[Capacity]{};
        size_t head_ = 0;
        size_t tail_ = 0;
        size_t count_ = 0;
        size_t used_ = 0;
    };
}

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <random>
#include <string>
#include "mqtt_report_queue.h"

template <size_t Capacity>
static std::string front_text(const mqtt_queue::ReportQueue<Capacity> &queue)
{
    const char *data = nullptr;
    size_t length = 0;
    assert(queue.front(data, length));
    return std::string(data, length);
}

template <size_t Capacity>
static void check_matches(mqtt_queue::ReportQueue<Capacity> queue, const std::deque<std::string> &expected)
{
    assert(queue.count() == expected.size());
    for (const std::string &text : expected)
    {
        assert(front_text(queue) == text);
        queue.pop();
    }
    const char *data = nullptr;
    size_t length = 0;
    assert(!queue.front(data, length) && queue.used_bytes() == 0);
}

static void test_basic_order_and_reset()
{
    mqtt_queue::ReportQueue<64> queue;
    uint32_t discarded = 0;
    const char *data = nullptr;
    size_t length = 0;
    assert(!queue.front(data, length));
    queue.pop();
    assert(queue.push("a", 1, discarded) && queue.push("bcdef", 5, discarded) && queue.push("", 0, discarded));
    assert(queue.count() == 3 && queue.used_bytes() == 8 + 12 + 4 && discarded == 0);
    check_matches(queue, {"a", "bcdef", ""});
}

static void test_wrap_marker_and_implicit_wrap()
{
    mqtt_queue::ReportQueue<32> queue;
    uint32_t discarded = 0;
    assert(queue.push("12345678", 8, discarded));  // [0,12)
    assert(queue.push("abcdefgh", 8, discarded));  // [12,24)
    queue.pop();                                   // head 12
    assert(queue.push("ABCDEFGH", 8, discarded));  // 8 bytes left at the end; wraps to [0,12) with a marker at 24
    assert(discarded == 0);
    check_matches(queue, {"abcdefgh", "ABCDEFGH"});
    queue.pop();
    assert(queue.push("wxyz", 4, discarded));        // [12,20)
    assert(queue.push("0123456789", 10, discarded)); // needs 16 contiguous bytes: neither gap fits until both go
    assert(discarded == 2);
    check_matches(queue, {"0123456789"});

    mqtt_queue::ReportQueue<24> exact;
    assert(exact.push("12345678", 8, discarded) && exact.push("abcdefgh", 8, discarded)); // ends exactly at 24
    exact.pop();
    assert(exact.push("XY", 2, discarded)); // implicit wrap: no room for a marker
    check_matches(exact, {"abcdefgh", "XY"});
    exact.pop();
    assert(front_text(exact) == "XY");
}

static void test_drop_oldest_and_oversize()
{
    mqtt_queue::ReportQueue<64> queue;
    uint32_t discarded = 0;
    assert(queue.push("aaaa", 4, discarded) && queue.push("bbbb", 4, discarded));
    const std::string largest(queue.max_payload_bytes(), 'z');
    assert(!queue.push(largest.data(), largest.size() + 1, discarded));
    assert(discarded == 0);
    check_matches(queue, {"aaaa", "bbbb"});
    assert(!queue.push(nullptr, 0, discarded));
    assert(queue.push(largest.data(), largest.size(), discarded));
    assert(discarded == 2 && queue.count() == 1 && queue.used_bytes() == 64);
    check_matches(queue, {largest});
}

static void test_randomized_against_model()
{
    constexpr size_t capacity = 512;
    mqtt_queue::ReportQueue<capacity> queue;
    std::deque<std::string> model;
    std::mt19937 random(12345);
    uint32_t discarded = 0;
    uint32_t sequence = 0;
    for (int step = 0; step < 200000; ++step)
    {
        if (random() % 3 == 0)
        {
            if (!model.empty())
            {
                assert(front_text(queue) == model.front());
                model.pop_front();
            }
            queue.pop();
        }
        else
        {
            std::string text = std::to_string(sequence++) + ":";
            text.append(random() % 160, static_cast<char>('a' + random() % 26));
            const uint32_t before = discarded;
            assert(queue.push(text.data(), text.size(), discarded));
            model.push_back(text);
            for (uint32_t i = before; i < discarded; ++i)
                model.pop_front();
            assert(queue.count() == model.size() && queue.used_bytes() <= capacity);
            size_t bytes = 0;
            for (const std::string &item : model)
                bytes += queue.record_bytes(item.size());
            assert(bytes == queue.used_bytes());
        }
        if (step % 997 == 0)
            check_matches(queue, model);
    }
    check_matches(queue, model);
}

int main()
{
    test_basic_order_and_reset();
    test_wrap_marker_and_implicit_wrap();
    test_drop_oldest_and_oversize();
    test_randomized_against_model();
    std::puts("PASS: MQTT report queue order, wrap, drop-oldest and oversize handling");
}

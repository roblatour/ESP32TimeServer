#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>
#include <vector>
#include "mqtt_report_queue.h"

static constexpr int MQTT_QOS = TEST_MQTT_QOS;
static constexpr size_t MQTT_RAM_QUEUE_BYTES = 64 * 1024;
static constexpr size_t MQTT_TF_REPORT_READ_LIMIT = 48 * 1024;
static constexpr const char *TAG = "storage-test";
static constexpr char TF_QUEUE_DIRECTORY[] = "/tfcard/Queue";
static char s_mqtt_payload[1024];
static std::atomic<bool> s_tf_queue_available{false}, s_mqtt_connected{true};
static std::atomic<size_t> s_mqtt_queued_messages_count{0};
static uint32_t s_mqtt_queued_messages_discarded = 0;
static uint64_t s_tf_queue_next_sequence = 0;
static unsigned logs = 0;
static void record_log(const char *, const char *, ...) { ++logs; }
#define TIMING_LOGE(...) do { if (DEBUG_ENABLED) record_log(__VA_ARGS__); } while (0)
#define TIMING_LOGW(...) TIMING_LOGE(__VA_ARGS__)
#define TIMING_LOGI(...) TIMING_LOGE(__VA_ARGS__)
#define ESP_LOGI(...) record_log(__VA_ARGS__)
#define ESP_LOGW(...) record_log(__VA_ARGS__)

static constexpr unsigned MALLOC_CAP_INTERNAL = 1, MALLOC_CAP_8BIT = 2;
static size_t free_heap = 256 * 1024, largest_block = 128 * 1024;
static bool fail_allocation = false, fragment_after_allocation = false;
static unsigned allocations = 0, frees = 0;
static size_t allocated_bytes = 0;
static size_t heap_caps_get_free_size(unsigned caps)
{
    assert(caps == (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    return free_heap - allocated_bytes;
}
static size_t heap_caps_get_largest_free_block(unsigned caps)
{
    assert(caps == (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    return allocated_bytes && fragment_after_allocation ? 16 * 1024 - 1 : largest_block;
}
static void *heap_caps_malloc(size_t bytes, unsigned caps)
{
    assert(caps == (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    ++allocations;
    if (fail_allocation)
        return nullptr;
    allocated_bytes += bytes;
    void *result = std::malloc(bytes);
    assert(result != nullptr);
    return result;
}
static void heap_caps_free(void *storage)
{
    ++frees;
    allocated_bytes = 0;
    std::free(storage);
}

enum class failure_t
{
    none, directory_open, directory_read, directory_close, open, write, flush,
    sync, close, rename, remove, seek, tell, read, disk_full_once, disk_full
};
static failure_t failure = failure_t::none;
static std::vector<std::string> names;
static size_t directory_index = 0;
static unsigned writes = 0, removes = 0, publishes = 0;
static bool publish_ok = true;
static std::string stored_payload = "stored";
static std::string published;
struct DIR {};
struct dirent { char d_name[32]; };
static DIR mock_directory;
static DIR *opendir(const char *path)
{
    assert(std::strcmp(path, TF_QUEUE_DIRECTORY) == 0);
    directory_index = 0;
    if (failure == failure_t::directory_open) { errno = EIO; return nullptr; }
    return &mock_directory;
}
static dirent *readdir(DIR *)
{
    static dirent entry;
    if (failure == failure_t::directory_read) { errno = EIO; return nullptr; }
    if (directory_index == names.size())
        return nullptr;
    std::snprintf(entry.d_name, sizeof(entry.d_name), "%s", names[directory_index++].c_str());
    return &entry;
}
static int closedir(DIR *) { return failure == failure_t::directory_close ? -1 : 0; }
static FILE *test_fopen(const char *, const char *mode)
{
    if (failure == failure_t::open) { errno = EIO; return nullptr; }
    FILE *file = std::tmpfile();
    assert(file != nullptr);
    if (std::strcmp(mode, "rb") == 0)
    {
        assert(std::fwrite(stored_payload.data(), 1, stored_payload.size(), file) == stored_payload.size());
        std::rewind(file);
    }
    return file;
}
static size_t test_fwrite(const void *data, size_t size, size_t count, FILE *file)
{
    ++writes;
    if (failure == failure_t::disk_full ||
        (failure == failure_t::disk_full_once && writes == 1))
    {
        errno = ENOSPC;
        return 0;
    }
    if (failure == failure_t::write) { errno = EIO; return 0; }
    return std::fwrite(data, size, count, file);
}
static int test_fflush(FILE *file)
{
    if (failure == failure_t::flush) { errno = EIO; return -1; }
    return std::fflush(file);
}
static int fsync(int) { if (failure == failure_t::sync) { errno = EIO; return -1; } return 0; }
static int test_fileno(FILE *) { return 1; }
static int test_fclose(FILE *file)
{
    const int result = std::fclose(file);
    if (failure == failure_t::close) { errno = EIO; return -1; }
    return result;
}
static int test_rename(const char *, const char *)
{
    if (failure == failure_t::rename) { errno = EIO; return -1; }
    names.push_back("R0000002.003");
    return 0;
}
static int test_unlink(const char *path)
{
    if (std::strstr(path, ".TMP"))
        return 0;
    ++removes;
    if (failure == failure_t::remove) { errno = EIO; return -1; }
    if (!names.empty())
        names.erase(names.begin());
    return 0;
}
static int test_fseek(FILE *file, long offset, int origin)
{
    if (failure == failure_t::seek) { errno = EIO; return -1; }
    return std::fseek(file, offset, origin);
}
static long test_ftell(FILE *file) { return failure == failure_t::tell ? -1 : std::ftell(file); }
static size_t test_fread(void *data, size_t size, size_t count, FILE *file)
{
    return failure == failure_t::read ? 0 : std::fread(data, size, count, file);
}
static bool mqtt_publish_queued_report(const char *payload, size_t length)
{
    ++publishes;
    published.assign(payload, length);
    return publish_ok;
}
#define fopen test_fopen
#define fileno test_fileno
#define unlink test_unlink
#define fwrite test_fwrite
#define fflush test_fflush
#define fclose test_fclose
#define rename test_rename
#define fseek test_fseek
#define ftell test_ftell
#define fread test_fread
#include "mqtt_storage_under_test.h"
#undef fopen
#undef fwrite
#undef fflush
#undef fclose
#undef rename
#undef fseek
#undef ftell
#undef fread
#undef fileno
#undef unlink

#define MQTT_ENABLED 1
#define NTP_SERVER_ENABLED 1
#define RBG_LED_KY_016_MODULE_ENABLED 0
#define RBG_LED_WS2812_MODULE_ENABLED 1
#include "setting_types.c"
struct sync_state_t
{
    struct { bool sanity_mismatch = false, pps_missing = false, gnss_invalid = false, sync_stale = false; } faults;
};
static sync_state_t sync_state;
static sync_state_t get_sync_state_snapshot() { return sync_state; }
static bool holdover = false;
struct ntp_reply_status_t { bool holdover; };
static ntp_reply_status_t get_ntp_reply_status() { return {holdover}; }
static std::atomic<bool> s_saved_gnss_baud_communication_failed{false}, s_time_setting_in_progress{false};
static std::atomic<bool> s_ethernet_connected{true}, s_ntp_external_responses_enabled{true};
static std::atomic<bool> s_mqtt_setup_failed{false}, s_gnss_locked{true};
static std::atomic<bool> s_open_for_business_message_written{true};
static std::atomic<bool> s_gnss_pps_startup_qualification_in_progress{false}, s_pps_discipline_active{false};
static void *s_net_event_group = nullptr;
static unsigned xEventGroupGetBits(void *) { return 1; }
static bool has_usable_network_address(unsigned bits) { return bits != 0; }
enum class ntp_transport_health_t { healthy, stalled };
static std::atomic<ntp_transport_health_t> s_ntp_transport_health{ntp_transport_health_t::healthy};
static bool ntp_load_shedding_active() { return false; }
static constexpr RGB_LED_Colour LED_normal = RGB_LED_Colour::green, LED_warning = RGB_LED_Colour::yellow,
    LED_critical = RGB_LED_Colour::red, LED_alert = RGB_LED_Colour::orange,
    LED_sync = RGB_LED_Colour::white, LED_button_push = RGB_LED_Colour::pink,
    LED_startup = RGB_LED_Colour::blue;
static constexpr char lcdTopLineTitle[] = "ESP32 Time Server";
static int64_t now_us = 0;
static int64_t esp_timer_get_time() { return now_us; }
static constexpr unsigned s_ws2812_brightness = 255;
static void *s_ws2812_strip = nullptr;
static unsigned rgb_red = 0, rgb_green = 0, rgb_blue = 0;
static int led_strip_set_pixel(void *, int, unsigned red, unsigned green, unsigned blue)
{
    rgb_red = red;
    rgb_green = green;
    rgb_blue = blue;
    return 0;
}
static int led_strip_refresh(void *) { return 0; }
#define ESP_ERROR_CHECK(result) assert((result) == 0)
#include "mqtt_storage_status_under_test.h"

static void check_status()
{
    s_mqtt_connected.store(false);
    for (const int code : {16, 17})
    {
        s_mqtt_queue_fallback_status.store(code);
        assert(get_required_top_line_message() == code);
        assert(lcd_top_line(code) == (code == 16 ? "ESP32 Time Server[Q]" : "ESP32 Time Server[q]"));
        now_us = 0;
        update_RGB_LED();
        assert(rgb_red == 0 && rgb_green == 128 && rgb_blue == 128);
        now_us = 1000000;
        update_RGB_LED();
        assert(rgb_red == 0 && rgb_green == (code == 16 ? 128U : 0U) &&
               rgb_blue == (code == 16 ? 128U : 0U));
        s_ethernet_connected.store(false);
        assert(get_required_top_line_message() == 1);
        s_ethernet_connected.store(true);
        sync_state.faults.pps_missing = true;
        assert(get_required_top_line_message() == 5);
        sync_state.faults.pps_missing = false;
        holdover = true;
        assert(get_required_top_line_message() == 15);
        holdover = false;
    }
    s_mqtt_queue_fallback_status.store(0);
    s_mqtt_connected.store(true);
}

static void reset()
{
    if (s_mqtt_ram_queue != nullptr)
    {
        s_mqtt_ram_queue->~mqtt_ram_queue_t();
        heap_caps_free(s_mqtt_ram_queue);
    }
    s_mqtt_ram_queue = nullptr;
    s_mqtt_ram_queue_allocation_attempted = false;
    s_mqtt_queue_fallback_status.store(0);
    s_tf_queue_available.store(true);
    s_mqtt_connected.store(true);
    s_mqtt_queued_messages_count.store(2);
    s_mqtt_queued_messages_discarded = 0;
    s_tf_queue_next_sequence = 0x2003;
    names = {"R0000001.002", "R0000000.001"};
    failure = failure_t::none;
    fail_allocation = fragment_after_allocation = false;
    free_heap = 256 * 1024;
    largest_block = 128 * 1024;
    allocations = frees = writes = removes = publishes = logs = 0;
    publish_ok = true;
    stored_payload = "stored";
    published.clear();
}

int main()
{
    reset();
    check_status();
    mqtt_tf_queue_snapshot_t snapshot;
    assert(mqtt_tf_queue_files(snapshot));
    assert(snapshot.count == 2 && snapshot.next_sequence == 0x1003);
    assert(std::strcmp(snapshot.oldest_path, "/tfcard/Queue/R0000000.001") == 0);
    assert(allocations == 0);
    if (MQTT_QOS == 0)
    {
        assert(!mqtt_allocate_ram_queue() && !mqtt_enqueue_report("new"));
        mqtt_send_queued_messages();
        assert(allocations == 0 && writes == 0 && publishes == 0);
        std::puts("PASS: QoS 0 has no RAM allocation or queue I/O");
        return 0;
    }
#if CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS
    assert(!mqtt_allocate_ram_queue() && allocations == 0);
    failure = failure_t::open;
    assert(!mqtt_enqueue_report("lost"));
    assert(s_mqtt_queue_fallback_status.load() == 17 && allocations == 0);
    std::puts("PASS: Q/q LCD and teal LED status; abort-on-OOM builds never risk optional queue allocation");
    return 0;
#else

    assert(mqtt_enqueue_report("new") && allocations == 0);
    assert(s_tf_queue_available.load() && s_mqtt_queued_messages_count.load() == 3);
    for (const failure_t injected : {failure_t::open, failure_t::write, failure_t::flush,
                                    failure_t::sync, failure_t::close, failure_t::rename,
                                    failure_t::directory_open, failure_t::directory_read,
                                    failure_t::directory_close})
    {
        reset();
        failure = injected;
        assert(mqtt_enqueue_report("new"));
        assert(!s_tf_queue_available.load() && s_mqtt_queue_fallback_status.load() == 16);
        assert(allocations == 1 && removes == 0);
        assert(s_mqtt_queued_messages_count.load() == 1 && s_mqtt_queued_messages_discarded == 0);
        const char *front = nullptr;
        size_t length = 0;
        assert(s_mqtt_ram_queue->front(front, length) && std::string(front, length) == "new");
        mqtt_send_queued_messages();
        assert(published == "new" && s_mqtt_queued_messages_count.load() == 0);
        assert(s_mqtt_queue_fallback_status.load() == 16);
        assert((logs > 0) == bool(DEBUG_ENABLED));
    }
    for (const failure_t injected : {failure_t::directory_open, failure_t::directory_read,
                                    failure_t::directory_close, failure_t::open, failure_t::seek,
                                    failure_t::tell, failure_t::read, failure_t::close, failure_t::remove})
    {
        reset();
        failure = injected;
        mqtt_send_queued_messages();
        assert(!s_tf_queue_available.load() && s_mqtt_queue_fallback_status.load() == 16);
        assert(allocations == 1 && s_mqtt_queued_messages_count.load() == 0);
        assert(mqtt_enqueue_report("after read failure"));
        assert(s_mqtt_queued_messages_count.load() == 1);
    }
    reset();
    publish_ok = false;
    mqtt_send_queued_messages();
    assert(s_tf_queue_available.load() && allocations == 0 && removes == 0);
    reset();
    failure = failure_t::disk_full_once;
    assert(mqtt_enqueue_report("new") && s_tf_queue_available.load());
    assert(removes == 1 && s_mqtt_queued_messages_discarded == 1 && allocations == 0);
    reset();
    failure = failure_t::disk_full;
    names.assign(30, "R0000000.001");
    assert(mqtt_enqueue_report("new") && !s_tf_queue_available.load());
    assert(removes == 16 && writes == 17 && s_mqtt_queued_messages_discarded == 16);

    for (int condition = 0; condition < 4; ++condition)
    {
        reset();
        failure = failure_t::open;
        if (condition == 0) free_heap = sizeof(mqtt_ram_queue_t) + 64 * 1024 - 1;
        if (condition == 1) largest_block = sizeof(mqtt_ram_queue_t) - 1;
        if (condition == 2) fail_allocation = true;
        if (condition == 3) fragment_after_allocation = true;
        assert(!mqtt_enqueue_report("lost"));
        assert(s_mqtt_ram_queue == nullptr && s_mqtt_queue_fallback_status.load() == 17);
        assert(s_mqtt_queued_messages_count.load() == 0 && s_mqtt_queued_messages_discarded == 1);
        assert(allocations == (condition >= 2 ? 1U : 0U));
        assert(frees == (condition == 3 ? 1U : 0U));
        assert(!mqtt_enqueue_report("also lost"));
        assert(allocations == (condition >= 2 ? 1U : 0U));
        s_mqtt_queued_messages_count.store(1);
        mqtt_send_queued_messages();
        assert(publishes == 0);
    }
    reset();
    free_heap = sizeof(mqtt_ram_queue_t) + 64 * 1024;
    assert(mqtt_allocate_ram_queue());
    assert(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) == 64 * 1024);
    assert(mqtt_allocate_ram_queue() && allocations == 1);
    reset();
    stored_payload.assign(2048, 'L');
    mqtt_send_queued_messages();
    assert(published == stored_payload && allocations == 1 && frees == 1);
    reset();
    stored_payload.assign(2048, 'L');
    fail_allocation = true;
    mqtt_send_queued_messages();
    assert(s_tf_queue_available.load() && publishes == 0 && removes == 0);
    reset();
    stored_payload.assign(MQTT_TF_REPORT_READ_LIMIT + 1, 'X');
    mqtt_send_queued_messages();
    assert(s_tf_queue_available.load() && allocations == 0 && removes == 1);
    assert(s_mqtt_queued_messages_discarded == 1);
    reset();
    std::puts("PASS: Q/q LCD and teal LED status; dynamic RAM queue, safe heap thresholds, TF write/read/scan failures, bounded FIFO eviction and one-shot fallback");
#endif
}

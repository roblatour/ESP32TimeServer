#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

using esp_err_t = int;
constexpr int ESP_OK = 0, ESP_FAIL = -1, ESP_ERR_TIMEOUT = 1;
constexpr int ESP_ERR_INVALID_STATE = 2, ESP_ERR_INVALID_ARG = 3, ESP_ERR_NOT_SUPPORTED = 4;
constexpr int HTTPD_RESP_USE_STRLEN = -1, HTTP_POST = 1, HTTPD_SOCK_ERR_TIMEOUT = -3;
constexpr int ESP_PARTITION_TYPE_APP = 0, ESP_PARTITION_SUBTYPE_APP_OTA_MIN = 16;
constexpr int ESP_PARTITION_SUBTYPE_APP_OTA_MAX = 32;
constexpr uint32_t OTA_WITH_SEQUENTIAL_WRITES = 0xfffffffe;
constexpr uint8_t ESP_IMAGE_HEADER_MAGIC = 0xe9;
constexpr uint32_t ESP_APP_DESC_MAGIC_WORD = 0xabcd5432;
constexpr int ESP_IMAGE_MAX_SEGMENTS = 16, ESP_IMAGE_APPLICATION = 1;
constexpr int AF_INET = 2, AF_INET6 = 10, SOL_SOCKET = 1, SO_RCVTIMEO = 2;
using socklen_t = unsigned int;
struct timeval { long tv_sec; long tv_usec; };
struct sockaddr { unsigned short sa_family; };
struct sockaddr_storage { unsigned short ss_family; unsigned char padding[126]; };
struct in6_addr { unsigned char bytes[16]; };
struct sockaddr_in6 { unsigned short sin6_family; in6_addr sin6_addr; };
struct ifreq { char ifr_name[16]; };
inline bool IN6_IS_ADDR_V4MAPPED(const in6_addr *address)
{
    const unsigned char prefix[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255};
    return std::memcmp(address->bytes, prefix, sizeof(prefix)) == 0;
}
struct esp_netif_t {};
struct esp_partition_t { uint32_t address; size_t size; int type; int subtype; };
#pragma pack(push, 1)
struct esp_image_header_t
{
    uint8_t magic;
    uint8_t segment_count;
    uint8_t spi_mode;
    uint8_t spi_speed_size;
    uint32_t entry_addr;
    uint8_t wp_pin;
    uint8_t spi_pin_drv[3];
    uint16_t chip_id;
    uint8_t min_chip_rev;
    uint16_t min_chip_rev_full;
    uint16_t max_chip_rev_full;
    uint8_t reserved[4];
    uint8_t hash_appended;
};
#pragma pack(pop)
struct esp_image_segment_header_t { uint32_t load_addr; uint32_t data_len; };
struct esp_app_desc_t { uint32_t magic_word; unsigned char prefix[44]; char project_name[32]; unsigned char padding[176]; };
static_assert(sizeof(esp_image_header_t) == 24 && sizeof(esp_app_desc_t) == 256);
static_assert(offsetof(esp_image_header_t, chip_id) == 12 && offsetof(esp_image_header_t, min_chip_rev_full) == 15);
static_assert(offsetof(esp_app_desc_t, project_name) == 48);
using esp_ota_handle_t = unsigned int;
using httpd_handle_t = void *;
struct httpd_req_t
{
    const char *uri = "/ote";
    size_t content_len = 0;
    std::map<std::string, std::string> headers;
    std::vector<char> body;
    size_t offset = 0;
    std::string status, response;
};
struct httpd_config_t
{
    uint16_t server_port = 80;
    size_t stack_size = 4096;
    int core_id = 0;
    unsigned int max_open_sockets = 7, max_uri_handlers = 8;
    uint16_t recv_wait_timeout = 5, send_wait_timeout = 5;
    ifreq *if_name = nullptr;
    esp_err_t (*open_fn)(httpd_handle_t, int) = nullptr;
};
#define HTTPD_DEFAULT_CONFIG() httpd_config_t{}
struct httpd_uri_t
{
    const char *uri = nullptr;
    int method = 0;
    esp_err_t (*handler)(httpd_req_t *) = nullptr;
};
struct mdns_txt_item_t { const char *key; const char *value; };

struct FakeState
{
    esp_partition_t running{0x10000, 8192, ESP_PARTITION_TYPE_APP, 16};
    esp_partition_t inactive{0x20000, 8192, ESP_PARTITION_TYPE_APP, 17};
    esp_app_desc_t description{};
    bool no_partition = false;
    uint16_t chip_id = 42, chip_revision = 0;
    int begin_error = 0, end_error = 0, boot_error = 0, chip_error = 0;
    int fail_write = 0, fail_receive = 0, receive_error = -1;
    int begin_count = 0, writes = 0, receives = 0, aborts = 0, ends = 0, boots = 0, chip_checks = 0;
    int failures = 0, successes = 0, socket_error = 0, peer_error = 0, reply_error = 0;
    int starts = 0, stops = 0, service_error = 0, register_error = 0;
    int64_t clock_us = 0, receive_advance_us = 0, begin_advance_us = 0;
    int64_t write_advance_us = 0, end_advance_us = 0;
    size_t receive_limit = 1024;
    sockaddr_storage peer{};
    std::vector<int64_t> socket_timeouts;
    std::vector<char> written;
    std::vector<std::pair<unsigned int, unsigned int>> progress;
    std::vector<std::string> events;
    std::string hostname, service;
};
inline FakeState fake;
inline const char *esp_err_to_name(esp_err_t) { return "fake error"; }
inline void fake_log(const char *, const char *, ...) {}
#define ESP_LOGE(...) fake_log(__VA_ARGS__)
#define ESP_LOGW(...) fake_log(__VA_ARGS__)
inline int64_t esp_timer_get_time() { return fake.clock_us; }
inline const esp_app_desc_t *esp_app_get_description() { return &fake.description; }
inline const esp_partition_t *esp_ota_get_running_partition() { return &fake.running; }
inline const esp_partition_t *esp_ota_get_next_update_partition(const esp_partition_t *)
{
    return fake.no_partition ? nullptr : &fake.inactive;
}
inline esp_err_t bootloader_common_check_chip_validity(const esp_image_header_t *image, int type)
{
    ++fake.chip_checks;
    return type == ESP_IMAGE_APPLICATION && image->chip_id == fake.chip_id &&
                   fake.chip_revision >= image->min_chip_rev_full &&
                   (image->max_chip_rev_full == 0 || image->max_chip_rev_full == UINT16_MAX ||
                    fake.chip_revision <= image->max_chip_rev_full)
               ? fake.chip_error
               : ESP_FAIL;
}
inline esp_err_t esp_ota_begin(const esp_partition_t *, size_t size, esp_ota_handle_t *handle)
{
    fake.events.push_back("begin");
    ++fake.begin_count;
    fake.clock_us += fake.begin_advance_us;
    if (size != OTA_WITH_SEQUENTIAL_WRITES) return ESP_FAIL;
    *handle = 1;
    return fake.begin_error;
}
inline esp_err_t esp_ota_write(esp_ota_handle_t, const void *data, size_t count)
{
    fake.events.push_back("write");
    fake.clock_us += fake.write_advance_us;
    if (++fake.writes == fake.fail_write) return ESP_FAIL;
    const char *bytes = static_cast<const char *>(data);
    fake.written.insert(fake.written.end(), bytes, bytes + count);
    return ESP_OK;
}
inline esp_err_t esp_ota_abort(esp_ota_handle_t)
{
    ++fake.aborts; fake.events.push_back("abort"); return ESP_OK;
}
inline esp_err_t esp_ota_end(esp_ota_handle_t)
{
    ++fake.ends; fake.clock_us += fake.end_advance_us;
    fake.events.push_back("end"); return fake.end_error;
}
inline esp_err_t esp_ota_set_boot_partition(const esp_partition_t *)
{
    ++fake.boots; fake.events.push_back("boot"); return fake.boot_error;
}
inline size_t httpd_req_get_hdr_value_len(httpd_req_t *request, const char *name)
{
    auto item = request->headers.find(name);
    return item == request->headers.end() ? 0 : item->second.size();
}
inline esp_err_t httpd_req_get_hdr_value_str(httpd_req_t *request, const char *name, char *buffer, size_t size)
{
    auto item = request->headers.find(name);
    if (item == request->headers.end() || item->second.size() >= size) return ESP_FAIL;
    std::memcpy(buffer, item->second.c_str(), item->second.size() + 1);
    return ESP_OK;
}
inline int httpd_req_to_sockfd(httpd_req_t *) { return 7; }
inline int httpd_req_recv(httpd_req_t *request, char *buffer, size_t count)
{
    fake.clock_us += fake.receive_advance_us;
    if (++fake.receives == fake.fail_receive) return fake.receive_error;
    count = std::min({count, fake.receive_limit, request->body.size() - request->offset});
    std::memcpy(buffer, request->body.data() + request->offset, count);
    request->offset += count;
    return static_cast<int>(count);
}
inline esp_err_t httpd_resp_set_status(httpd_req_t *request, const char *value)
{
    request->status = value; return ESP_OK;
}
inline esp_err_t httpd_resp_set_type(httpd_req_t *, const char *) { return ESP_OK; }
inline esp_err_t httpd_resp_set_hdr(httpd_req_t *, const char *, const char *) { return ESP_OK; }
inline esp_err_t httpd_resp_send(httpd_req_t *request, const char *message, int)
{
    request->response = message; fake.events.push_back("reply"); return fake.reply_error;
}
inline int setsockopt(int, int, int, const void *value, socklen_t)
{
    const timeval &timeout = *static_cast<const timeval *>(value);
    fake.socket_timeouts.push_back(timeout.tv_sec * 1000000LL + timeout.tv_usec);
    return fake.socket_error;
}
inline int getpeername(int, sockaddr *address, socklen_t *size)
{
    std::memcpy(address, &fake.peer, std::min<size_t>(*size, sizeof(fake.peer)));
    return fake.peer_error;
}
inline esp_err_t esp_netif_get_netif_impl_name(esp_netif_t *, char *name)
{
    std::memcpy(name, "en0", sizeof("en0")); return ESP_OK;
}
inline esp_err_t httpd_start(httpd_handle_t *server, const httpd_config_t *)
{
    ++fake.starts; *server = reinterpret_cast<void *>(1); return ESP_OK;
}
inline esp_err_t httpd_stop(httpd_handle_t) { ++fake.stops; return ESP_OK; }
inline esp_err_t httpd_register_uri_handler(httpd_handle_t, const httpd_uri_t *)
{
    return fake.register_error;
}
inline esp_err_t mdns_init() { return ESP_OK; }
inline esp_err_t mdns_hostname_set(const char *name) { fake.hostname = name; return ESP_OK; }
inline esp_err_t mdns_service_add(const char *, const char *service, const char *,
                                 uint16_t, mdns_txt_item_t *, size_t)
{
    fake.service = service; return fake.service_error;
}
inline esp_err_t mdns_service_remove(const char *, const char *) { return ESP_OK; }

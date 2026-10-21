#include "ESP32TimeServerSettings.h"
#include "native_ote.h"

#if OTE_UPDATES_ENABLED

#include <algorithm>
#include <cstring>
#include <limits>

#include "bootloader_common.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "mdns.h"

namespace
{
static constexpr char TAG[] = "native_ote";
static constexpr bool IPV4_ENABLED = PreferIPvX != 2;
static constexpr bool IPV6_ENABLED = PreferIPvX != 1;
static constexpr size_t IMAGE_PREFIX_SIZE =
    sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
static constexpr size_t BUFFER_SIZE = 1024;
static_assert(IMAGE_PREFIX_SIZE <= BUFFER_SIZE, "OTA prefix must fit the receive buffer");

static httpd_handle_t s_server = nullptr;
static native_ote_callbacks_t s_callbacks{};
// The single HTTP server task owns this latch; never clear it after boot selection.
static bool s_boot_selected = false;

static esp_err_t send_response(httpd_req_t *request, const char *status, const char *message)
{
    httpd_resp_set_status(request, status);
    httpd_resp_set_type(request, "text/plain");
    httpd_resp_set_hdr(request, "Connection", "close");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    (void)httpd_resp_send(request, message, HTTPD_RESP_USE_STRLEN);
    // Returning an error closes the session, including any unread upload body.
    return ESP_FAIL;
}

static esp_err_t upload_failure(httpd_req_t *request, const char *status, const char *reason,
                                esp_err_t error = ESP_FAIL)
{
    if (s_callbacks.failure != nullptr)
        s_callbacks.failure(reason);
    ESP_LOGE(TAG, "%s (%s)", reason, esp_err_to_name(error));
    return send_response(request, status, reason);
}

static void report_progress(size_t written, size_t total)
{
    if (s_callbacks.progress != nullptr)
        s_callbacks.progress(static_cast<unsigned int>(written), static_cast<unsigned int>(total));
}

static bool is_authenticated(httpd_req_t *request)
{
    char authorization[sizeof(OTEPassword) + 7]{};
    const size_t password_length = strlen(OTEPassword);
    if (password_length == 0 ||
        httpd_req_get_hdr_value_len(request, "Authorization") != password_length + 7 ||
        httpd_req_get_hdr_value_str(request, "Authorization", authorization, sizeof(authorization)) != ESP_OK ||
        memcmp(authorization, "Bearer ", 7) != 0)
        return false;

    unsigned int difference = 0;
    for (size_t index = 0; index < password_length; ++index)
        difference |= static_cast<unsigned char>(authorization[index + 7]) ^
                      static_cast<unsigned char>(OTEPassword[index]);
    return difference == 0;
}

static bool has_valid_content_length(httpd_req_t *request)
{
    char value[24]{};
    if (httpd_req_get_hdr_value_str(request, "Content-Length", value, sizeof(value)) != ESP_OK ||
        value[0] == '\0' || httpd_req_get_hdr_value_len(request, "Transfer-Encoding") != 0)
        return false;

    size_t length = 0;
    for (const char *cursor = value; *cursor != '\0'; ++cursor)
    {
        if (*cursor < '0' || *cursor > '9')
            return false;
        const size_t digit = static_cast<size_t>(*cursor - '0');
        if (length > (std::numeric_limits<size_t>::max() - digit) / 10)
            return false;
        length = length * 10 + digit;
    }
    return length == request->content_len && length >= IMAGE_PREFIX_SIZE;
}

static esp_err_t receive_bytes(httpd_req_t *request, char *buffer, size_t length, int64_t deadline_us)
{
    size_t received = 0;
    while (received < length)
    {
        const int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0)
            return ESP_ERR_TIMEOUT;

        // Bound every blocking receive by both the idle and whole-upload deadlines.
        const int64_t wait_us = std::min<int64_t>(remaining_us,
                                                  static_cast<int64_t>(OTEReceiveTimeoutSeconds) * 1000000LL);
        // lwIP rounds SO_RCVTIMEO down to milliseconds; zero would mean no timeout.
        const int64_t wait_ms = (wait_us + 999) / 1000;
        timeval timeout{};
        timeout.tv_sec = static_cast<decltype(timeout.tv_sec)>(wait_ms / 1000);
        timeout.tv_usec = static_cast<decltype(timeout.tv_usec)>((wait_ms % 1000) * 1000);
        if (setsockopt(httpd_req_to_sockfd(request), SOL_SOCKET, SO_RCVTIMEO,
                       &timeout, sizeof(timeout)) != 0)
            return ESP_FAIL;

        const int count = httpd_req_recv(request, buffer + received, length - received);
        if (count <= 0)
            return count == HTTPD_SOCK_ERR_TIMEOUT ? ESP_ERR_TIMEOUT : ESP_FAIL;
        received += static_cast<size_t>(count);
        if (esp_timer_get_time() >= deadline_us)
            return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static esp_err_t receive_failure(httpd_req_t *request, esp_err_t error)
{
    return error == ESP_ERR_TIMEOUT
               ? upload_failure(request, "408 Request Timeout", "OTE receive/upload timeout\n", error)
               : upload_failure(request, "400 Bad Request", "OTE receive failed or upload truncated\n", error);
}

static esp_err_t upload_handler(httpd_req_t *request)
{
    if (!is_authenticated(request))
    {
        httpd_resp_set_hdr(request, "WWW-Authenticate", "Bearer");
        return send_response(request, "401 Unauthorized", "OTE authentication required\n");
    }
    if (s_boot_selected)
        return send_response(request, "409 Conflict", "OTE image already accepted; wait for restart\n");
    if (strcmp(request->uri, "/ote") != 0)
        return send_response(request, "404 Not Found", "Use POST /ote\n");

    char content_type[32]{};
    if (httpd_req_get_hdr_value_str(request, "Content-Type", content_type, sizeof(content_type)) != ESP_OK ||
        strcmp(content_type, "application/octet-stream") != 0)
        return send_response(request, "415 Unsupported Media Type",
                             "Send a raw application/octet-stream application .bin\n");
    if (!has_valid_content_length(request))
        return send_response(request, "400 Bad Request", "A valid firmware Content-Length is required\n");

    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
    if (running == nullptr || partition == nullptr || partition->address == running->address ||
        partition->type != ESP_PARTITION_TYPE_APP ||
        partition->subtype < ESP_PARTITION_SUBTYPE_APP_OTA_MIN ||
        partition->subtype >= ESP_PARTITION_SUBTYPE_APP_OTA_MAX)
        return upload_failure(request, "500 Internal Server Error", "No inactive OTA application partition\n");
    if (request->content_len > partition->size ||
        request->content_len > std::numeric_limits<unsigned int>::max())
        return send_response(request, "413 Content Too Large", "Firmware exceeds the inactive OTA partition\n");

    const int64_t deadline_us = esp_timer_get_time() +
                               static_cast<int64_t>(OTEMaxUploadTimeSeconds) * 1000000LL;
    report_progress(0, request->content_len);
    char buffer[BUFFER_SIZE];
    esp_err_t result = receive_bytes(request, buffer, IMAGE_PREFIX_SIZE, deadline_us);
    if (result != ESP_OK)
        return receive_failure(request, result);

    esp_image_header_t image_header{};
    esp_image_segment_header_t segment_header{};
    esp_app_desc_t description{};
    memcpy(&image_header, buffer, sizeof(image_header));
    memcpy(&segment_header, buffer + sizeof(image_header), sizeof(segment_header));
    memcpy(&description, buffer + sizeof(image_header) + sizeof(segment_header), sizeof(description));
    const esp_app_desc_t *running_description = esp_app_get_description();
    if (image_header.magic != ESP_IMAGE_HEADER_MAGIC || image_header.segment_count == 0 ||
        image_header.segment_count > ESP_IMAGE_MAX_SEGMENTS ||
        segment_header.data_len < sizeof(esp_app_desc_t) ||
        segment_header.data_len > request->content_len - sizeof(image_header) - sizeof(segment_header) ||
        description.magic_word != ESP_APP_DESC_MAGIC_WORD ||
        strncmp(description.project_name, running_description->project_name, sizeof(description.project_name)) != 0)
        return upload_failure(request, "400 Bad Request", "Wrong application firmware image/project\n");
    result = bootloader_common_check_chip_validity(&image_header, ESP_IMAGE_APPLICATION);
    if (result != ESP_OK)
        return upload_failure(request, "400 Bad Request", "Firmware chip or revision is unsuitable\n", result);
    if (esp_timer_get_time() >= deadline_us)
        return receive_failure(request, ESP_ERR_TIMEOUT);

    esp_ota_handle_t handle = 0;
    result = esp_ota_begin(partition, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (result != ESP_OK)
        return upload_failure(request, "500 Internal Server Error", "OTA begin failed\n", result);

    size_t written = 0;
    size_t count = IMAGE_PREFIX_SIZE;
    for (;;)
    {
        if (esp_timer_get_time() >= deadline_us)
        {
            (void)esp_ota_abort(handle);
            return receive_failure(request, ESP_ERR_TIMEOUT);
        }
        result = esp_ota_write(handle, buffer, count);
        if (result != ESP_OK)
        {
            (void)esp_ota_abort(handle);
            return upload_failure(request, "400 Bad Request", "OTA write failed\n", result);
        }
        written += count;
        report_progress(written, request->content_len);
        if (written == request->content_len)
            break;
        count = std::min(sizeof(buffer), request->content_len - written);
        result = receive_bytes(request, buffer, count, deadline_us);
        if (result != ESP_OK)
        {
            (void)esp_ota_abort(handle);
            return receive_failure(request, result);
        }
    }
    if (esp_timer_get_time() >= deadline_us)
    {
        (void)esp_ota_abort(handle);
        return receive_failure(request, ESP_ERR_TIMEOUT);
    }

    // esp_ota_end consumes the handle even on failure; do not abort it afterwards.
    result = esp_ota_end(handle);
    if (result != ESP_OK)
        return upload_failure(request, "400 Bad Request", "Firmware validation failed\n", result);
    if (esp_timer_get_time() >= deadline_us)
        return receive_failure(request, ESP_ERR_TIMEOUT);
    result = esp_ota_set_boot_partition(partition);
    if (result != ESP_OK)
        return upload_failure(request, "500 Internal Server Error", "OTA boot selection failed\n", result);

    s_boot_selected = true;
    // Boot selection is committed even if the uploader disconnects before this reply.
    if (s_callbacks.success != nullptr)
        s_callbacks.success();
    return send_response(request, "200 OK", "OTE image validated; restarting in 5 seconds\n");
}

static esp_err_t open_session(httpd_handle_t server, int socket_fd)
{
    (void)server;
    sockaddr_storage peer{};
    socklen_t length = sizeof(peer);
    if (getpeername(socket_fd, reinterpret_cast<sockaddr *>(&peer), &length) != 0)
        return ESP_FAIL;
    bool ipv4 = false;
    if (peer.ss_family == AF_INET)
        ipv4 = true;
#if CONFIG_LWIP_IPV6
    else if (peer.ss_family == AF_INET6)
        ipv4 = IN6_IS_ADDR_V4MAPPED(&reinterpret_cast<const sockaddr_in6 *>(&peer)->sin6_addr);
#endif
    else
        return ESP_FAIL;
    return (ipv4 ? IPV4_ENABLED : IPV6_ENABLED) ? ESP_OK : ESP_FAIL;
}
} // namespace

esp_err_t native_ote_start(esp_netif_t *netif, const native_ote_callbacks_t &callbacks)
{
    if (s_server != nullptr)
        return ESP_ERR_INVALID_STATE;
    if (netif == nullptr || OTEPassword[0] == '\0' || DeviceName[0] == '\0' || OTEPort == 0 ||
        OTEReceiveTimeoutSeconds == 0 ||
        OTEReceiveTimeoutSeconds > std::numeric_limits<uint16_t>::max() || OTEMaxUploadTimeSeconds == 0)
        return ESP_ERR_INVALID_ARG;
#if !CONFIG_LWIP_IPV6
    if (PreferIPvX == 2)
        return ESP_ERR_NOT_SUPPORTED;
#endif

    struct ifreq ethernet_interface{};
    esp_err_t result = esp_netif_get_netif_impl_name(netif, ethernet_interface.ifr_name);
    if (result != ESP_OK)
        return result;
    if (ethernet_interface.ifr_name[0] == '\0')
        return ESP_ERR_INVALID_ARG;

    httpd_config_t configuration = HTTPD_DEFAULT_CONFIG();
    configuration.server_port = OTEPort;
    configuration.stack_size = 6144;
    configuration.core_id = 0;
    configuration.max_open_sockets = 2;
    configuration.max_uri_handlers = 1;
    configuration.recv_wait_timeout = OTEReceiveTimeoutSeconds;
    configuration.send_wait_timeout = OTEReceiveTimeoutSeconds;
    configuration.if_name = &ethernet_interface;
    configuration.open_fn = open_session;

    s_callbacks = callbacks;
    httpd_handle_t server = nullptr;
    result = httpd_start(&server, &configuration);
    if (result != ESP_OK)
        return result;

    result = mdns_init();
    if (result == ESP_OK)
        result = mdns_hostname_set(DeviceName);
    if (result == ESP_OK)
    {
        mdns_txt_item_t text[] = {{"path", "/ote"}};
        result = mdns_service_add(DeviceName, "_http", "_tcp", OTEPort, text, 1);
    }
    if (result != ESP_OK)
    {
        httpd_stop(server);
        return result;
    }

    httpd_uri_t upload{};
    upload.uri = "/ote";
    upload.method = HTTP_POST;
    upload.handler = upload_handler;
    result = httpd_register_uri_handler(server, &upload);
    if (result != ESP_OK)
    {
        (void)mdns_service_remove("_http", "_tcp");
        httpd_stop(server);
        return result;
    }
    s_server = server;
    ESP_LOGW(TAG, "HTTP OTE on Ethernet TCP port %u at %s.local; plaintext, trusted LAN/VPN only",
             static_cast<unsigned int>(OTEPort), DeviceName);
    return ESP_OK;
}

#endif

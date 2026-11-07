#include <iostream>
#include <stdexcept>
#include <functional>

// Compile the production handler unchanged, replacing only its SDK boundary.
#include "..\..\..\main\custom\native_ote.cpp"

#define CHECK(expression)                          \
    do                                             \
    {                                              \
        if (!(expression))                         \
            throw std::runtime_error(#expression); \
    } while (0)

static void progress_callback(unsigned int value, unsigned int total)
{
    fake.progress.emplace_back(value, total);
    fake.events.push_back("progress:" + std::to_string(value));
}
static void failure_callback(const char *)
{
    ++fake.failures;
    fake.events.push_back("failure");
}
static void success_callback()
{
    ++fake.successes;
    fake.events.push_back("success");
}

static httpd_req_t fresh_request()
{
    fake = FakeState{};
    s_server = nullptr;
    s_boot_selected = false;
    s_callbacks = {progress_callback, failure_callback, success_callback};
    fake.description.magic_word = ESP_APP_DESC_MAGIC_WORD;
    std::memcpy(fake.description.project_name, "TimeServer", sizeof("TimeServer"));
    fake.peer.ss_family = AF_INET;
    httpd_req_t request;
    request.content_len = 2500;
    request.body.resize(request.content_len, 'x');
    request.headers = {{"Authorization", "Bearer test-password"},
                       {"Content-Type", "application/octet-stream"},
                       {"Content-Length", std::to_string(request.content_len)}};
    esp_image_header_t image{};
    image.magic = ESP_IMAGE_HEADER_MAGIC;
    image.segment_count = 1;
    image.chip_id = 42;
    image.max_chip_rev_full = UINT16_MAX;
    esp_image_segment_header_t segment{};
    segment.data_len = static_cast<uint32_t>(request.content_len - sizeof(image) - sizeof(segment));
    std::memcpy(request.body.data(), &image, sizeof(image));
    std::memcpy(request.body.data() + sizeof(image), &segment, sizeof(segment));
    std::memcpy(request.body.data() + sizeof(image) + sizeof(segment), &fake.description, sizeof(fake.description));
    return request;
}

static void rejected_before_writes()
{
    for (int scenario = 0; scenario < 10; ++scenario)
    {
        auto request = fresh_request();
        std::string status = "400";
        if (scenario == 0)
        {
            request.headers["Authorization"] = "Bearer wrong";
            status = "401";
        }
        if (scenario == 1)
        {
            request.headers["Content-Type"] = "multipart/form-data";
            status = "415";
        }
        if (scenario == 2)
        {
            fake.inactive.size = 100;
            status = "413";
        }
        if (scenario == 3)
            request.body[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) +
                         offsetof(esp_app_desc_t, project_name)] = '!';
        if (scenario == 4)
            fake.chip_error = ESP_FAIL;
        if (scenario == 5)
            request.headers.erase("Content-Length");
        if (scenario == 6)
            request.headers["Content-Length"] = "999999999999999999999999999";
        if (scenario == 7)
            request.headers["Transfer-Encoding"] = "chunked";
        if (scenario == 8)
        {
            fake.inactive.address = fake.running.address;
            status = "500";
        }
        if (scenario == 9)
        {
            fake.no_partition = true;
            status = "500";
        }
        CHECK(upload_handler(&request) == ESP_FAIL);
        CHECK(request.status.find(status) == 0);
        CHECK(fake.begin_count == 0 && fake.writes == 0 && fake.boots == 0 && fake.aborts == 0);
        CHECK(!s_boot_selected && fake.successes == 0);
    }
}

static void chip_and_revision_rejection()
{
    constexpr uint16_t P4 = 0x0012, S31 = 0x0020;
    for (const auto &chips : {std::pair<uint16_t, uint16_t>{P4, S31}, {S31, P4}})
    {
        auto request = fresh_request();
        fake.chip_id = chips.first;
        esp_image_header_t image{};
        std::memcpy(&image, request.body.data(), sizeof(image));
        image.chip_id = chips.second;
        std::memcpy(request.body.data(), &image, sizeof(image));
        CHECK(upload_handler(&request) == ESP_FAIL);
        CHECK(request.status.find("400") == 0);
        CHECK(fake.chip_checks == 1);
        CHECK(fake.begin_count == 0 && fake.writes == 0 && fake.boots == 0 && !s_boot_selected);
    }

    for (bool minimum_mismatch : {false, true})
    {
        auto request = fresh_request();
        fake.chip_id = P4;
        fake.chip_revision = 300;
        esp_image_header_t image{};
        std::memcpy(&image, request.body.data(), sizeof(image));
        image.chip_id = P4;
        image.min_chip_rev_full = minimum_mismatch ? 301 : 0;
        image.max_chip_rev_full = minimum_mismatch ? UINT16_MAX : 299;
        std::memcpy(request.body.data(), &image, sizeof(image));
        CHECK(upload_handler(&request) == ESP_FAIL);
        CHECK(request.status.find("400") == 0);
        CHECK(fake.chip_checks == 1);
        CHECK(fake.begin_count == 0 && fake.writes == 0 && fake.boots == 0 && !s_boot_selected);
    }

    auto compatible = fresh_request();
    fake.chip_id = P4;
    fake.chip_revision = 301;
    esp_image_header_t compatible_image{};
    std::memcpy(&compatible_image, compatible.body.data(), sizeof(compatible_image));
    compatible_image.chip_id = P4;
    compatible_image.min_chip_rev_full = 301;
    compatible_image.max_chip_rev_full = 0;
    std::memcpy(compatible.body.data(), &compatible_image, sizeof(compatible_image));
    CHECK(upload_handler(&compatible) == ESP_FAIL);
    CHECK(compatible.status == "200 OK" && fake.chip_checks == 1);
    CHECK(fake.writes > 0 && fake.boots == 1 && s_boot_selected);

    auto request = fresh_request();
    request.body.resize(IMAGE_PREFIX_SIZE - 1);
    CHECK(upload_handler(&request) == ESP_FAIL);
    CHECK(fake.chip_checks == 0);
    CHECK(fake.begin_count == 0 && fake.writes == 0 && fake.boots == 0 && !s_boot_selected);
}

static void failures_release_only_live_handles()
{
    for (int scenario = 0; scenario < 9; ++scenario)
    {
        auto request = fresh_request();
        int aborts = 0, ends = 0, boots = 0;
        if (scenario == 0)
            fake.begin_error = ESP_FAIL;
        if (scenario == 1)
        {
            fake.fail_write = 1;
            aborts = 1;
        }
        if (scenario == 2)
        {
            fake.fail_write = 2;
            aborts = 1;
        }
        if (scenario == 3)
        {
            fake.fail_receive = 1;
        }
        if (scenario == 4)
        {
            fake.fail_receive = 2;
            aborts = 1;
        }
        if (scenario == 5)
        {
            fake.end_error = ESP_FAIL;
            ends = 1;
        }
        if (scenario == 6)
        {
            fake.boot_error = ESP_FAIL;
            ends = 1;
            boots = 1;
        }
        if (scenario == 7)
        {
            request.body.resize(IMAGE_PREFIX_SIZE + 100);
            aborts = 1;
        }
        if (scenario == 8)
        {
            fake.fail_receive = 2;
            fake.receive_error = HTTPD_SOCK_ERR_TIMEOUT;
            aborts = 1;
        }
        CHECK(upload_handler(&request) == ESP_FAIL);
        CHECK(fake.aborts == aborts && fake.ends == ends && fake.boots == boots);
        CHECK(fake.failures == 1 && fake.successes == 0 && !s_boot_selected);
        CHECK(request.status.find("200") != 0);
    }
}

static void successful_order_and_lockout()
{
    auto request = fresh_request();
    fake.receive_limit = 73;
    CHECK(upload_handler(&request) == ESP_FAIL);
    CHECK(request.status == "200 OK" && s_boot_selected);
    CHECK(fake.written == request.body);
    CHECK(fake.begin_count == 1 && fake.ends == 1 && fake.boots == 1 && fake.aborts == 0);
    CHECK(fake.successes == 1 && fake.failures == 0);
    CHECK(fake.progress.front().first == 0 && fake.progress.back().first == request.content_len);
    for (size_t index = 1; index < fake.progress.size(); ++index)
        CHECK(fake.progress[index].first > fake.progress[index - 1].first);
    for (const auto &entry : fake.progress)
        CHECK(entry.second == request.content_len);
    CHECK(fake.events.front() == "progress:0");
    CHECK(fake.events[1] == "begin" && fake.events[2] == "write");
    const auto &events = fake.events;
    CHECK(events[events.size() - 4] == "end" && events[events.size() - 3] == "boot");
    CHECK(events[events.size() - 2] == "success" && events.back() == "reply");
    const int writes = fake.writes;
    request.offset = 0;
    CHECK(upload_handler(&request) == ESP_FAIL && request.status.find("409") == 0);
    CHECK(fake.writes == writes && fake.boots == 1 && fake.successes == 1 && fake.failures == 0);

    request = fresh_request();
    fake.reply_error = ESP_FAIL;
    upload_handler(&request);
    CHECK(s_boot_selected && fake.successes == 1);
}

static void deadlines_and_socket_failures()
{
    for (int scenario = 0; scenario < 6; ++scenario)
    {
        auto request = fresh_request();
        int aborts = 0;
        if (scenario == 0)
        {
            fake.fail_receive = 1;
            fake.receive_error = HTTPD_SOCK_ERR_TIMEOUT;
        }
        if (scenario == 1)
            fake.receive_advance_us = 300000000;
        if (scenario == 2)
        {
            fake.begin_advance_us = 300000000;
            aborts = 1;
        }
        if (scenario == 3)
        {
            fake.write_advance_us = 300000000;
            aborts = 1;
        }
        if (scenario == 4)
            fake.end_advance_us = 300000000;
        if (scenario == 5)
            fake.socket_error = -1;
        upload_handler(&request);
        CHECK(request.status.find(scenario == 5 ? "400" : "408") == 0);
        CHECK(fake.aborts == aborts && fake.boots == 0 && !s_boot_selected && fake.failures == 1);
    }
    auto request = fresh_request();
    char bytes[1];
    CHECK(receive_bytes(&request, bytes, 1, 1) == ESP_OK);
    CHECK(fake.socket_timeouts.back() == 1000);
    CHECK(receive_bytes(&request, bytes, 1, 0) == ESP_ERR_TIMEOUT);
    request = fresh_request();
    CHECK(receive_bytes(&request, bytes, 1, 300000000) == ESP_OK);
    CHECK(fake.socket_timeouts.back() == 15000000);
    CHECK(receive_bytes(&request, bytes, 1, 2000000) == ESP_OK);
    CHECK(fake.socket_timeouts.back() == 2000000);
}

static void address_family_rules()
{
    fresh_request();
    fake.peer.ss_family = AF_INET;
    CHECK((open_session(nullptr, 7) == ESP_OK) == (PreferIPvX != 2));
    fake.peer.ss_family = AF_INET6;
    CHECK((open_session(nullptr, 7) == ESP_OK) == (PreferIPvX != 1));
    auto *peer = reinterpret_cast<sockaddr_in6 *>(&fake.peer);
    peer->sin6_addr.bytes[10] = 255;
    peer->sin6_addr.bytes[11] = 255;
    CHECK((open_session(nullptr, 7) == ESP_OK) == (PreferIPvX != 2));
    fake.peer.ss_family = 99;
    CHECK(open_session(nullptr, 7) == ESP_FAIL);
    fake.peer_error = -1;
    CHECK(open_session(nullptr, 7) == ESP_FAIL);
}

static void startup_and_mdns()
{
    fresh_request();
    esp_netif_t netif;
    CHECK(native_ote_start(nullptr, s_callbacks) == ESP_ERR_INVALID_ARG);
    CHECK(native_ote_start(&netif, s_callbacks) == ESP_OK);
    CHECK(fake.hostname == HostName && fake.service == "_http");
    CHECK(native_ote_start(&netif, s_callbacks) == ESP_ERR_INVALID_STATE);
    fresh_request();
    fake.service_error = ESP_FAIL;
    CHECK(native_ote_start(&netif, s_callbacks) == ESP_FAIL);
    CHECK(fake.stops == 1 && s_server == nullptr);
    fresh_request();
    fake.register_error = ESP_FAIL;
    CHECK(native_ote_start(&netif, s_callbacks) == ESP_FAIL);
    CHECK(fake.stops == 1 && s_server == nullptr);
}

int main()
{
    const std::pair<const char *, std::function<void()>> tests[] = {
        {"prewrite rejection", rejected_before_writes},
        {"chip/revision and truncated-image rejection", chip_and_revision_rejection},
        {"OTA failure cleanup", failures_release_only_live_handles},
        {"success ordering and lockout", successful_order_and_lockout},
        {"deadlines and sockets", deadlines_and_socket_failures},
        {"address family gating", address_family_rules},
        {"startup and mDNS", startup_and_mdns},
    };
    try
    {
        for (const auto &test : tests)
        {
            test.second();
            std::cout << "PASS " << test.first << '\n';
        }
        std::cout << "7 groups passed; PreferIPvX=" << PreferIPvX << '\n';
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}

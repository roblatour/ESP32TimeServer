#pragma once

// Included by main.cpp after the shared PHC mapping and PTP helpers.
struct ptp_l3_udp_received_t
{
    uint8_t message[64];
    uint8_t mac[6];
    uint8_t length;
    uint8_t family;
    eth_mac_time_t timestamp;
    uint32_t generation;
    uint32_t network_generation;
};

struct ptp_l3_udp_family_t
{
    int sockets[2] = {-1, -1};
    uint8_t source[16]{};
    bool ready = false;
    bool healthy = true;
    uint16_t sync_sequence = 0;
    uint16_t announce_sequence = 0;
    int64_t last_sync_us = 0;
    int64_t last_announce_us = 0;
    int64_t retry_us = 0;
    ptp::MasterSelection<PTP_L3_UDP_FOREIGN_MASTER_CAPACITY> selection{
        PTP_L3_UDP_MASTER_SELECTION_ENABLED != 0, PTP_L3_UDP_FOREIGN_ANNOUNCE_TIMEOUT_SECONDS * 1000000LL,
        2000000LL << PTP_L3_UDP_LOG_ANNOUNCE_INTERVAL};
};

static ptp_l3_udp_family_t s_ptp_l3_udp_families[2];
static QueueHandle_t s_ptp_l3_udp_queue = nullptr;
static std::atomic<bool> s_ptp_l3_udp_worker_ready{false};
static std::atomic<bool> s_ptp_l3_udp_timestamp_fault{false};
static std::atomic<uint32_t> s_ptp_l3_udp_network_generation[2]{};

static void ptp_l3_udp_clear_followers(unsigned family)
{
    portENTER_CRITICAL(&s_ptp_followers_mux);
    s_ptp_followers.clear_transport(static_cast<uint8_t>(family + 1));
    portEXIT_CRITICAL(&s_ptp_followers_mux);
}

static ptp::MasterDataset ptp_l3_udp_dataset()
{
    ptp::MasterDataset dataset{};
    dataset.priority1 = PTP_L3_UDP_PRIORITY1;
    dataset.priority2 = PTP_L3_UDP_PRIORITY2;
    memcpy(dataset.identity, s_ptp_identity, sizeof(dataset.identity));
    return dataset;
}

static void ptp_l3_udp_close(ptp_l3_udp_family_t &state)
{
    state.ready = false;
    for (int &fd : state.sockets)
    {
        if (fd >= 0)
        {
            if (close(fd) != 0)
                ESP_LOGE(TAG, "PTP UDP socket close failed: errno=%d", errno);
            fd = -1;
        }
    }
}

static bool ptp_l3_udp_source(unsigned family, uint8_t *source)
{
    if (!s_ethernet_connected.load(std::memory_order_acquire) ||
        s_ethernet_recovery_in_progress.load(std::memory_order_acquire) || s_eth_netif == nullptr)
        return false;
    if (family == 0)
    {
        const uint32_t address = s_own_ipv4_address_be.load(std::memory_order_acquire);
        memcpy(source, &address, 4);
        return IPV4_ENABLED && address != 0;
    }
    esp_ip6_addr_t address{};
    if (!IPV6_ENABLED || esp_netif_get_ip6_linklocal(s_eth_netif, &address) != ESP_OK)
        return false;
    memcpy(source, address.addr, 16);
    return source[0] == 0xfe && (source[1] & 0xc0) == 0x80;
}

static bool ptp_l3_udp_open(unsigned family, ptp_l3_udp_family_t &state)
{
    const int interface_index = esp_netif_get_netif_impl_index(s_eth_netif);
    if (interface_index <= 0)
    {
        ESP_LOGE(TAG, "PTP UDP Ethernet interface index unavailable");
        return false;
    }
    for (unsigned port_index = 0; port_index < 2; ++port_index)
    {
        const int fd = socket(family == 0 ? AF_INET : AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
        state.sockets[port_index] = fd;
        bool ok = fd >= 0;
        if (ok && family == 0)
        {
            sockaddr_in local{};
            local.sin_family = AF_INET;
            local.sin_port = htons(port_index == 0 ? ptp_l3_udp::EVENT_PORT : ptp_l3_udp::GENERAL_PORT);
            ip_mreq membership{};
            memcpy(&membership.imr_multiaddr, ptp_l3_udp::IPV4_GROUP, 4);
            memcpy(&membership.imr_interface, state.source, 4);
            ok = bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) == 0 &&
                 setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &membership, sizeof(membership)) == 0;
        }
        else if (ok)
        {
            const int v6_only = 1;
            sockaddr_in6 local{};
            local.sin6_family = AF_INET6;
            local.sin6_port = htons(port_index == 0 ? ptp_l3_udp::EVENT_PORT : ptp_l3_udp::GENERAL_PORT);
            local.sin6_scope_id = static_cast<uint32_t>(interface_index);
            ipv6_mreq membership{};
            memcpy(&membership.ipv6mr_multiaddr, ptp_l3_udp::IPV6_GROUP, 16);
            membership.ipv6mr_interface = static_cast<unsigned>(interface_index);
            ok = setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &v6_only, sizeof(v6_only)) == 0 &&
                 bind(fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) == 0 &&
                 setsockopt(fd, IPPROTO_IPV6, IPV6_JOIN_GROUP, &membership, sizeof(membership)) == 0;
        }
        if (!ok)
        {
            ESP_LOGE(TAG, "PTP UDP/IPv%u bind/join port %u failed: errno=%d",
                     family == 0 ? 4U : 6U, port_index == 0 ? 319U : 320U, errno);
            ptp_l3_udp_close(state);
            return false;
        }
    }
    state.ready = true;
    state.healthy = true;
    state.last_sync_us = state.last_announce_us = 0;
    state.selection.reset(ptp_l3_udp_dataset(), esp_timer_get_time());
    ESP_LOGI(TAG, "PTP UDP/IPv%u multicast joined on Ethernet (319/320, E2E, unsigned)", family == 0 ? 4U : 6U);
    return true;
}

static bool ptp_l3_udp_receive_frame(const uint8_t *frame, uint32_t length, const eth_mac_time_t *timestamp)
{
    if (!s_ptp_l3_udp_worker_ready.load(std::memory_order_acquire))
        return false;
    ptp_l3_udp::Datagram datagram{};
    ptp::ParsedMessage parsed{};
    const auto kind = ptp_l3_udp::decode(frame, length, PTP_L3_UDP_DOMAIN, s_ptp_identity, datagram, parsed);
    if (kind == ptp::MessageKind::Ignored && datagram.message == nullptr)
        return false;
    if ((datagram.ipv6 && !IPV6_ENABLED) || (!datagram.ipv6 && !IPV4_ENABLED))
        return true;
    if (kind == ptp::MessageKind::Invalid)
    {
        s_ptp_invalid_requests.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    if (kind != ptp::MessageKind::DelayRequest && kind != ptp::MessageKind::Announce)
        return true;
    const uint32_t generation = s_ptp_clock_generation.load(std::memory_order_acquire);
    if ((generation & 1U) != 0 || !s_ptp_clock_ready.load(std::memory_order_acquire) ||
        s_ethernet_recovery_in_progress.load(std::memory_order_acquire))
        return true;
    ptp_l3_udp_received_t received{};
    memcpy(received.message, datagram.message, datagram.length);
    memcpy(received.mac, frame + 6, 6);
    received.length = static_cast<uint8_t>(datagram.length);
    received.family = datagram.ipv6 ? 1 : 0;
    received.generation = generation;
    received.network_generation = s_ptp_l3_udp_network_generation[received.family].load(std::memory_order_acquire);
    if (timestamp != nullptr)
        received.timestamp = *timestamp;
    if (xQueueSend(s_ptp_l3_udp_queue, &received, 0) != pdTRUE)
        s_ptp_queue_drops.fetch_add(1, std::memory_order_relaxed);
    return true;
}

static bool ptp_l3_udp_transmit(unsigned family, const uint8_t *message, size_t length, eth_mac_time_t *timestamp = nullptr)
{
    auto &state = s_ptp_l3_udp_families[family];
    uint8_t frame[ptp_l3_udp::MAX_FRAME_SIZE]{};
    const size_t frame_length = ptp_l3_udp::encapsulate(frame, sizeof(frame), s_ptp_source_mac, state.source,
                                                       family != 0, message, length);
    if (frame_length == 0)
    {
        ESP_LOGE(TAG, "Invalid PTP UDP transmit frame");
        return false;
    }
    return state.ready && ptp_transmit_raw(frame, static_cast<uint32_t>(frame_length), timestamp);
}

static bool ptp_l3_udp_qualified(unsigned family, int64_t now_us)
{
    const auto &state = s_ptp_l3_udp_families[family];
    return state.ready && state.healthy && ptp_time_qualified() &&
           s_ptp_l3_udp_families[family].selection.role(now_us) == ptp::SelectionRole::Master;
}

static void ptp_l3_udp_build_message(uint8_t *message, uint8_t type, uint16_t sequence, uint16_t size, unsigned family)
{
    ptp_build_message(message, type, sequence, size, PTP_L3_UDP_DOMAIN, PTP_L3_UDP_LOG_SYNC_INTERVAL,
                      PTP_L3_UDP_LOG_ANNOUNCE_INTERVAL, ptp_l3_udp_dataset(), false);
    ptp_write_u16(message + 28, static_cast<uint16_t>(family + 2));
    if (type == 9)
        message[33] = 0; // E2E logMinDelayReqInterval: one second.
}

static void ptp_l3_udp_service_task(void *)
{
    uint32_t last_generation = UINT32_MAX;
    for (;;)
    {
        ptp_l3_udp_received_t received{};
        const bool available = xQueueReceive(s_ptp_l3_udp_queue, &received, pdMS_TO_TICKS(20)) == pdTRUE;
        const int64_t now_us = esp_timer_get_time();
        const uint32_t generation = s_ptp_clock_generation.load(std::memory_order_acquire);
        if (generation != last_generation)
            s_ptp_l3_udp_timestamp_fault.store(false, std::memory_order_release);
        bool active = false;
        for (unsigned family = 0; family < 2; ++family)
        {
            if (family == 0 ? !IPV4_ENABLED : !IPV6_ENABLED)
                continue;
            auto &state = s_ptp_l3_udp_families[family];
            uint8_t source[16]{};
            const bool source_ready = ptp_l3_udp_source(family, source);
            if (generation != last_generation || !source_ready ||
                memcmp(source, state.source, family == 0 ? 4 : 16) != 0)
            {
                s_ptp_l3_udp_network_generation[family].fetch_add(1, std::memory_order_acq_rel);
                ptp_l3_udp_close(state);
                ptp_l3_udp_clear_followers(family);
                memcpy(state.source, source, sizeof(source));
                state.retry_us = 0;
            }
            if (!state.ready && source_ready && now_us >= state.retry_us)
            {
                state.retry_us = now_us + 5000000LL;
                ptp_l3_udp_open(family, state);
            }
            if (available && received.family == family && received.generation == generation && state.ready &&
                received.network_generation == s_ptp_l3_udp_network_generation[family].load(std::memory_order_acquire))
            {
                ptp::ParsedMessage parsed{};
                const auto kind = ptp::parse_message(received.message, received.length, PTP_L3_UDP_DOMAIN, s_ptp_identity, parsed);
                if (kind == ptp::MessageKind::Announce)
                    state.selection.observe(parsed.announce_dataset, parsed.source_port_identity, parsed.sequence_id,
                                            parsed.log_announce_interval, now_us);
                else if (kind == ptp::MessageKind::DelayRequest)
                {
                    s_ptp_valid_requests.fetch_add(1, std::memory_order_relaxed);
                    if (!timestamp_is_valid(&received.timestamp))
                    {
                        s_ptp_rx_timestamp_misses.fetch_add(1, std::memory_order_relaxed);
                        state.healthy = false;
                    }
                    else if (ptp_l3_udp_qualified(family, now_us))
                    {
                        ptp_clock_lock_t lock;
                        if (lock.locked && ptp_l3_udp_qualified(family, esp_timer_get_time()) &&
                            generation == s_ptp_clock_generation.load(std::memory_order_acquire))
                        {
                            uint8_t message[64]{};
                            ptp_l3_udp_build_message(message, 9, parsed.sequence_id, 54, family);
                            memcpy(message + 44, parsed.source_port_identity, 10);
                            if (ptp_write_timestamp(message + 34, received.timestamp))
                            {
                                if (ptp_l3_udp_transmit(family, message, 54))
                                {
                                    s_ptp_delay_responses.fetch_add(1, std::memory_order_relaxed);
                                    portENTER_CRITICAL(&s_ptp_followers_mux);
                                    if (generation == s_ptp_clock_generation.load(std::memory_order_acquire))
                                        s_ptp_followers.observe(parsed.domain, parsed.source_port_identity, now_us, received.mac,
                                            static_cast<uint8_t>(family + 1), PTP_L3_UDP_FOLLOWER_TIMEOUT_SECONDS * 1000000LL,
                                            PTP_L3_UDP_FOLLOWER_CAPACITY);
                                    portEXIT_CRITICAL(&s_ptp_followers_mux);
                                }
                                else
                                    state.healthy = false;
                            }
                        }
                    }
                }
            }
            bool qualified = ptp_l3_udp_qualified(family, now_us);
            if (qualified && now_us - state.last_sync_us >= (1000000LL << PTP_L3_UDP_LOG_SYNC_INTERVAL))
            {
                ptp_clock_lock_t lock;
                if (lock.locked && ptp_l3_udp_qualified(family, esp_timer_get_time()) &&
                    generation == s_ptp_clock_generation.load(std::memory_order_acquire))
                {
                    uint8_t message[64]{};
                    const uint16_t sequence = state.sync_sequence++;
                    ptp_l3_udp_build_message(message, 0, sequence, 44, family);
                    ptp_write_sync_origin(message + 34);
                    eth_mac_time_t egress{};
                    const bool sent = ptp_l3_udp_transmit(family, message, 44, &egress);
                    timespec raw{};
                    bool recent = false;
                    if (sent && timestamp_is_valid(&egress) && clock_gettime(CLOCK_PTP_SYSTEM, &raw) == 0)
                    {
                        const int64_t age_ns = static_cast<int64_t>(raw.tv_sec) * 1000000000LL + raw.tv_nsec -
                            (static_cast<int64_t>(egress.seconds) * 1000000000LL + egress.nanoseconds);
                        recent = age_ns > -2000000LL && age_ns < 100000000LL;
                    }
                    ptp_l3_udp_build_message(message, 8, sequence, 44, family);
                    if (!recent || generation != s_ptp_clock_generation.load(std::memory_order_acquire) ||
                        !ptp_l3_udp_qualified(family, esp_timer_get_time()) ||
                        !ptp_write_timestamp(message + 34, egress) || !ptp_l3_udp_transmit(family, message, 44))
                    {
                        s_ptp_tx_timestamp_misses.fetch_add(1, std::memory_order_relaxed);
                        state.healthy = false;
                    }
                    state.last_sync_us = now_us;
                }
            }
            if (qualified && state.healthy &&
                now_us - state.last_announce_us >= (1000000LL << PTP_L3_UDP_LOG_ANNOUNCE_INTERVAL))
            {
                ptp_clock_lock_t lock;
                if (lock.locked && ptp_l3_udp_qualified(family, esp_timer_get_time()) &&
                    generation == s_ptp_clock_generation.load(std::memory_order_acquire))
                {
                    uint8_t message[64]{};
                    ptp_l3_udp_build_message(message, 11, state.announce_sequence++, 64, family);
                    if (!ptp_l3_udp_transmit(family, message, 64))
                        state.healthy = false;
                    state.last_announce_us = now_us;
                }
            }
            qualified = ptp_l3_udp_qualified(family, esp_timer_get_time());
            if (!state.healthy)
                s_ptp_l3_udp_timestamp_fault.store(true, std::memory_order_release);
            if (!qualified)
                ptp_l3_udp_clear_followers(family);
            active = active || qualified;
        }
        last_generation = generation;
        if (active)
            s_ptp_service_active.fetch_or(2U, std::memory_order_acq_rel);
        else
            s_ptp_service_active.fetch_and(static_cast<uint8_t>(~2U), std::memory_order_acq_rel);
#if CALCULATE_STACK_SIZES_ENABLED
        report_current_task_stack_usage(PTP_L3_UDP_Service);
#endif
    }
}

static bool start_ptp_l3_udp_service()
{
#if PTP_L2_SERVER_ENABLED
    if (!s_ptp_l2_worker_ready.load(std::memory_order_acquire))
#endif
    {
        if (esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, s_ptp_source_mac) != ESP_OK)
            return false;
        memcpy(s_ptp_identity, s_ptp_source_mac, 3);
        s_ptp_identity[3] = 0xff;
        s_ptp_identity[4] = 0xfe;
        memcpy(s_ptp_identity + 5, s_ptp_source_mac + 3, 3);
    }
    s_ptp_l3_udp_queue = xQueueCreate(PTP_L3_UDP_REQUEST_QUEUE_DEPTH, sizeof(ptp_l3_udp_received_t));
    if (s_ptp_l3_udp_queue == nullptr)
        return false;
    if (xTaskCreatePinnedToCore(ptp_l3_udp_service_task, "ptp_l3_udp", PTP_L3_UDP_Service_Task_Stack_Size,
                                nullptr, 12, nullptr, 0) != pdPASS)
    {
        vQueueDelete(s_ptp_l3_udp_queue);
        s_ptp_l3_udp_queue = nullptr;
        return false;
    }
    s_ptp_l3_udp_worker_ready.store(true, std::memory_order_release);
    return true;
}

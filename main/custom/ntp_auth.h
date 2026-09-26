// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

#include <cstddef>
#include <cstdint>

static constexpr size_t NTP_AUTH_KEY_ID_SIZE = 4;
static constexpr size_t NTP_AUTH_DIGEST_SIZE = 20;
static constexpr size_t NTP_AUTH_TRAILER_SIZE = NTP_AUTH_KEY_ID_SIZE + NTP_AUTH_DIGEST_SIZE;
static constexpr size_t NTP_AUTH_MAX_KEYS = 4;
static constexpr size_t NTP_AUTH_MAX_KEY_SIZE = 64;

enum class ntp_auth_result_t : uint8_t
{
    unauthenticated,
    valid,
    malformed,
    unavailable,
    unknown_key,
    invalid_mac,
};

struct ntp_auth_key_t
{
    uint32_t key_id;
    uint8_t algorithm;
    uint8_t key_length;
    uint8_t lifecycle;
    uint8_t key[NTP_AUTH_MAX_KEY_SIZE];
};

bool ntp_auth_initialize();
bool ntp_auth_available();
size_t ntp_auth_key_count();
bool ntp_auth_key_hex(size_t index, uint32_t *key_id, char *output, size_t output_size);
uint32_t ntp_auth_default_key_id();
bool ntp_auth_default_key_hex(char *output, size_t output_size);
bool ntp_auth_default_key_ascii(char *output, size_t output_size);
ntp_auth_result_t ntp_auth_verify_request(const uint8_t *packet, size_t packet_length, uint8_t version, uint32_t *key_id);
bool ntp_auth_append_response(uint8_t *packet, size_t packet_capacity, size_t *packet_length, uint32_t key_id);

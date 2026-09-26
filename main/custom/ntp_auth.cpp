// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#include "ntp_auth.h"
#include "ESP32TimeServerSettings.h"

#include <cinttypes>
#include <cstring>
#include <iterator>

#include "esp_log.h"
#include "psa/crypto.h"

namespace
{
    static constexpr uint8_t NTP_AUTH_ALGORITHM_SHA256_160 = 1;
    static constexpr uint8_t NTP_AUTH_KEY_ACTIVE = 1;
    static constexpr size_t SYMMETRIC_KEY_LENGTH = 32;
    static const char *TAG = "ntp_auth";

    static constexpr unsigned int number_of_symmetric_keys = std::size(symmetric_keys);

    static ntp_auth_key_t s_keys[NTP_AUTH_MAX_KEYS]{};
    static size_t s_key_count = 0;
    static bool s_initialized = false;
    static bool s_available = false;

    static uint32_t read_u32_be(const uint8_t *value)
    {
        return (static_cast<uint32_t>(value[0]) << 24) | (static_cast<uint32_t>(value[1]) << 16) |
               (static_cast<uint32_t>(value[2]) << 8) | static_cast<uint32_t>(value[3]);
    }

    static void write_u32_be(uint8_t *value, uint32_t number)
    {
        value[0] = static_cast<uint8_t>(number >> 24);
        value[1] = static_cast<uint8_t>(number >> 16);
        value[2] = static_cast<uint8_t>(number >> 8);
        value[3] = static_cast<uint8_t>(number);
    }

    static bool constant_time_equal(const uint8_t *left, const uint8_t *right, size_t length)
    {
        uint8_t difference = 0;
        for (size_t index = 0; index < length; ++index)
            difference |= left[index] ^ right[index];
        return difference == 0;
    }

    static const ntp_auth_key_t *find_key(uint32_t key_id)
    {
        for (size_t index = 0; index < s_key_count; ++index)
        {
            if (s_keys[index].key_id == key_id && s_keys[index].lifecycle == NTP_AUTH_KEY_ACTIVE)
                return &s_keys[index];
        }
        return nullptr;
    }

    static bool calculate_digest(const ntp_auth_key_t &key, const uint8_t *input, size_t input_length, uint8_t digest[NTP_AUTH_DIGEST_SIZE])
    {
        uint8_t full_digest[32] = {};
        size_t full_digest_length = 0;
        psa_hash_operation_t operation = PSA_HASH_OPERATION_INIT;
        const psa_status_t status = psa_hash_setup(&operation, PSA_ALG_SHA_256) == PSA_SUCCESS &&
                                    psa_hash_update(&operation, key.key, key.key_length) == PSA_SUCCESS &&
                                    psa_hash_update(&operation, input, input_length) == PSA_SUCCESS &&
                                    psa_hash_finish(&operation, full_digest, sizeof(full_digest), &full_digest_length);
        psa_hash_abort(&operation);
        if (status != PSA_SUCCESS || full_digest_length != sizeof(full_digest))
            return false;
        memcpy(digest, full_digest, NTP_AUTH_DIGEST_SIZE);
        return true;
    }
} // namespace

bool ntp_auth_initialize()
{
    if (s_initialized)
        return s_available;

    s_initialized = true;

#if SYMMETRIC_KEY_AUTHENTICATION_ENABLED

    // number_of_symmetric_keys = std::size(symmetric_keys);

    if (number_of_symmetric_keys < 1 || number_of_symmetric_keys > NTP_AUTH_MAX_KEYS)
    {
        ESP_LOGE(TAG, "Invalid number of symmetric keys: %u", number_of_symmetric_keys);
        return false;
    }

    for (size_t index = 0; index < number_of_symmetric_keys; ++index)
    {
        const symmetric_key_t &source = symmetric_keys[index];
        bool valid = true;
        if (number_of_symmetric_keys < 1 || number_of_symmetric_keys > 4)
        {
            ESP_LOGE(TAG, "The number of symmetric keys %zu: key_id must be between 1 and 4 (inclusive) (got %" PRIu32 ")", number_of_symmetric_keys);
            valid = false;
        }
        if (source.key_id < 1 || source.key_id > 65535)
        {
            ESP_LOGE(TAG, "Symmetric key %zu: key_id must be between 1 and 65535 (inclusive)(got %" PRIu32 ")", index + 1, source.key_id);
            valid = false;
        }
        size_t actual_length = 0;
        bool alphanumeric = true;
        while (actual_length < sizeof(source.key_value) && source.key_value[actual_length] != '\0')
        {
            const char character = source.key_value[actual_length++];
            if (!((character >= '0' && character <= '9') ||
                  (character >= 'A' && character <= 'Z') ||
                  (character >= 'a' && character <= 'z')))
                alphanumeric = false;
        }
        if (!alphanumeric)
        {
            ESP_LOGE(TAG, "Symmetric key %zu: key_value must contain only alphanumeric characters", index + 1);
            valid = false;
        }
        if (actual_length != SYMMETRIC_KEY_LENGTH || source.key_value[SYMMETRIC_KEY_LENGTH] != '\0')
        {
            ESP_LOGE(TAG, "Symmetric key %zu: key_value must contain exactly %zu characters (got %zu)",
                     index + 1, SYMMETRIC_KEY_LENGTH, actual_length);
            valid = false;
        }
        for (size_t other = 0; other < s_key_count; ++other)
        {
            if (s_keys[other].key_id == source.key_id)
            {
                ESP_LOGE(TAG, "Symmetric key %zu: duplicate key_id %" PRIu32, index + 1, source.key_id);
                valid = false;
                break;
            }
        }
        if (!valid)
            continue;

        s_keys[s_key_count++] = {source.key_id, NTP_AUTH_ALGORITHM_SHA256_160, static_cast<uint8_t>(SYMMETRIC_KEY_LENGTH), NTP_AUTH_KEY_ACTIVE, {}};
        memcpy(s_keys[s_key_count - 1].key, source.key_value, SYMMETRIC_KEY_LENGTH);
    }
#endif
    if (psa_crypto_init() != PSA_SUCCESS)
        return false;
    s_available = s_key_count != 0;
    return s_available;
}

bool ntp_auth_available()
{
    return s_available;
}

size_t ntp_auth_key_count()
{
    return s_available ? s_key_count : 0;
}

uint32_t ntp_auth_default_key_id()
{
    return s_available ? s_keys[0].key_id : 0;
}

bool ntp_auth_key_hex(size_t index, uint32_t *key_id, char *output, size_t output_size)
{
    if (!s_available || index >= s_key_count || key_id == nullptr || output == nullptr ||
        output_size < s_keys[index].key_length * 2 + 1)
        return false;

    const ntp_auth_key_t &key = s_keys[index];
    static constexpr char hex_digits[] = "0123456789ABCDEF";
    for (size_t character_index = 0; character_index < key.key_length; ++character_index)
    {
        output[character_index * 2] = hex_digits[key.key[character_index] >> 4];
        output[character_index * 2 + 1] = hex_digits[key.key[character_index] & 0x0F];
    }
    output[key.key_length * 2] = '\0';
    *key_id = key.key_id;
    return true;
}

bool ntp_auth_default_key_hex(char *output, size_t output_size)
{
    uint32_t key_id = 0;
    return ntp_auth_key_hex(0, &key_id, output, output_size);
}

bool ntp_auth_default_key_ascii(char *output, size_t output_size)
{
    if (!s_available || output == nullptr || output_size < s_keys[0].key_length + 1)
        return false;

    for (size_t index = 0; index < s_keys[0].key_length; ++index)
    {
        const uint8_t character = s_keys[0].key[index];
        if (character < 0x21 || character > 0x7E)
            return false;
        output[index] = static_cast<char>(character);
    }
    output[s_keys[0].key_length] = '\0';
    return true;
}

ntp_auth_result_t ntp_auth_verify_request(const uint8_t *packet, size_t packet_length, uint8_t version, uint32_t *key_id)
{
    if (key_id != nullptr)
        *key_id = 0;
    if (packet == nullptr || packet_length < 48)
        return ntp_auth_result_t::malformed;
    if (packet_length == 48)
        return ntp_auth_result_t::unauthenticated;
    if (version != 4 || packet_length != 48 + NTP_AUTH_TRAILER_SIZE)
        return ntp_auth_result_t::malformed;
    if (!s_available)
        return ntp_auth_result_t::unavailable;

    const size_t key_id_offset = packet_length - NTP_AUTH_TRAILER_SIZE;
    const uint32_t requested_key_id = read_u32_be(packet + key_id_offset);
    const ntp_auth_key_t *key = find_key(requested_key_id);
    if (key == nullptr)
        return ntp_auth_result_t::unknown_key;

    uint8_t digest[NTP_AUTH_DIGEST_SIZE] = {};
    if (!calculate_digest(*key, packet, key_id_offset, digest) ||
        !constant_time_equal(digest, packet + key_id_offset + NTP_AUTH_KEY_ID_SIZE, NTP_AUTH_DIGEST_SIZE))
        return ntp_auth_result_t::invalid_mac;

    if (key_id != nullptr)
        *key_id = requested_key_id;
    return ntp_auth_result_t::valid;
}

bool ntp_auth_append_response(uint8_t *packet, size_t packet_capacity, size_t *packet_length, uint32_t key_id)
{
    if (packet == nullptr || packet_length == nullptr || *packet_length != 48 || packet_capacity < 48 + NTP_AUTH_TRAILER_SIZE || !s_available)
        return false;

    const ntp_auth_key_t *key = find_key(key_id);
    if (key == nullptr)
        return false;

    write_u32_be(packet + 48, key_id);
    if (!calculate_digest(*key, packet, 48, packet + 48 + NTP_AUTH_KEY_ID_SIZE))
        return false;
    *packet_length = 48 + NTP_AUTH_TRAILER_SIZE;
    return true;
}

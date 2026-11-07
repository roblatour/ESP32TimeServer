// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT

#include "ptp_l2_auth.h"
#include "ESP32TimeServerSettings.h"

#include <cstring>
#include <iterator>

#include "esp_log.h"
#include "psa/crypto.h"

namespace
{
static const char *TAG = "ptp_l2_auth";
static constexpr psa_algorithm_t MAC_ALGORITHM = PSA_ALG_TRUNCATED_MAC(PSA_ALG_HMAC(PSA_ALG_SHA_256), PTP_L2_AUTH_ICV_SIZE);
struct ptp_l2_auth_key_t
{
    uint32_t id;
    psa_key_id_t handle;
};
static ptp_l2_auth_key_t s_keys[PTP_L2_AUTH_MAX_KEYS]{};
static size_t s_key_count = 0;
static bool s_initialized = false;
static bool s_policy_ready = false;
static bool s_available = false;

static uint16_t read_u16(const uint8_t *data)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

static const ptp_l2_auth_key_t *find_key(uint32_t id)
{
    for (size_t i = 0; i < s_key_count; ++i)
        if (s_keys[i].id == id)
            return &s_keys[i];
    return nullptr;
}

static size_t body_size(uint8_t type)
{
    switch (type & 0x0f)
    {
    case 0:
    case 1:
    case 8:
        return 44;
    case 9:
        return 54;
    case 11:
        return 64;
    default:
        return 0;
    }
}

static void release_keys()
{
    for (size_t i = 0; i < s_key_count; ++i)
    {
        const psa_status_t status = psa_destroy_key(s_keys[i].handle);
        if (status != PSA_SUCCESS)
            ESP_LOGE(TAG, "PTP key cleanup failed: %ld", static_cast<long>(status));
        s_keys[i] = {};
    }
    s_key_count = 0;
}
}

bool ptp_l2_auth_initialize()
{
    if (s_initialized)
        return s_policy_ready;
    s_initialized = true;
    const size_t count = std::size(ptp_l2_symmetric_keys);
    if (count == 0)
    {
        if (PTP_L2_AUTHENTICATION_REQUIRED)
        {
            ESP_LOGE(TAG, "PTP authentication required but no PTP keys are configured");
            return false;
        }
        ESP_LOGW(TAG, "No PTP keys configured: outgoing time is unsigned; signed incoming packets will be rejected");
        s_policy_ready = true;
        return true;
    }
    if (count > PTP_L2_AUTH_MAX_KEYS)
    {
        ESP_LOGE(TAG, "Invalid PTP key count: %u (maximum %u)", static_cast<unsigned>(count),
                 static_cast<unsigned>(PTP_L2_AUTH_MAX_KEYS));
        return false;
    }
    bool active_key_found = false;
    size_t index = 0;
    for (const symmetric_key_t &source : ptp_l2_symmetric_keys)
    {
        if (source.key_id == 0)
        {
            ESP_LOGE(TAG, "Invalid PTP key record %u: zero key ID", static_cast<unsigned>(index + 1));
            return false;
        }
        for (size_t previous = 0; previous < index; ++previous)
        {
            if (ptp_l2_symmetric_keys[previous].key_id == source.key_id)
            {
                ESP_LOGE(TAG, "Invalid PTP key record %u: duplicate key ID", static_cast<unsigned>(index + 1));
                return false;
            }
        }
        if (source.key_value[32] != '\0')
        {
            ESP_LOGE(TAG, "Invalid PTP key record %u: missing terminator after 32 bytes", static_cast<unsigned>(index + 1));
            return false;
        }
        for (size_t i = 0; i < 32; ++i)
        {
            const char c = source.key_value[i];
            if (c == '\0')
            {
                ESP_LOGE(TAG, "Invalid PTP key record %u: key must contain exactly 32 characters", static_cast<unsigned>(index + 1));
                return false;
            }
            if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')))
            {
                ESP_LOGE(TAG, "Invalid PTP key record %u: key must contain only ASCII alphanumeric characters", static_cast<unsigned>(index + 1));
                return false;
            }
        }
        active_key_found = active_key_found || source.key_id == PTP_L2_AUTHENTICATION_ACTIVE_KEY_ID;
        ++index;
    }
    if (!active_key_found)
    {
        ESP_LOGE(TAG, "PTP active key ID is not configured");
        return false;
    }
    const psa_status_t init_status = psa_crypto_init();
    if (init_status != PSA_SUCCESS)
    {
        ESP_LOGE(TAG, "PTP PSA crypto initialization failed: %ld", static_cast<long>(init_status));
        return false;
    }
    for (const symmetric_key_t &source : ptp_l2_symmetric_keys)
    {
        psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
        psa_set_key_type(&attributes, PSA_KEY_TYPE_HMAC);
        psa_set_key_bits(&attributes, 256);
        psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE | PSA_KEY_USAGE_VERIFY_MESSAGE);
        psa_set_key_algorithm(&attributes, MAC_ALGORITHM);
        psa_key_id_t handle = 0;
        const psa_status_t status = psa_import_key(&attributes, reinterpret_cast<const uint8_t *>(source.key_value), 32, &handle);
        psa_reset_key_attributes(&attributes);
        if (status != PSA_SUCCESS)
        {
            ESP_LOGE(TAG, "PTP HMAC key import failed: %ld", static_cast<long>(status));
            release_keys();
            return false;
        }
        s_keys[s_key_count++] = {source.key_id, handle};
    }
    s_available = true;
    s_policy_ready = true;
    return true;
}

bool ptp_l2_auth_available()
{
    return s_available;
}

ptp_l2_auth_result_t ptp_l2_auth_verify(const uint8_t *message, size_t length)
{
    if (message == nullptr || length < 34 || read_u16(message + 2) != length)
        return ptp_l2_auth_result_t::malformed;
    const size_t body = body_size(message[0]);
    if (body == 0 || length < body)
        return ptp_l2_auth_result_t::malformed;
    if (length == body)
        return PTP_L2_AUTHENTICATION_REQUIRED ? ptp_l2_auth_result_t::authentication_required : ptp_l2_auth_result_t::unauthenticated;
    if (length != body + PTP_L2_AUTH_TLV_SIZE || message[1] != 0x12)
        return ptp_l2_auth_result_t::malformed;
    const uint8_t *tlv = message + body;
    if (read_u16(tlv) != 0x8009 || read_u16(tlv + 2) != PTP_L2_AUTH_TLV_SIZE - 4 ||
        tlv[4] != PTP_L2_AUTHENTICATION_SPP || tlv[5] != 0)
        return ptp_l2_auth_result_t::malformed;
    if (!s_available)
        return ptp_l2_auth_result_t::unavailable;
    const uint32_t id = (static_cast<uint32_t>(tlv[6]) << 24) | (static_cast<uint32_t>(tlv[7]) << 16) |
                        (static_cast<uint32_t>(tlv[8]) << 8) | tlv[9];
    const ptp_l2_auth_key_t *key = find_key(id);
    if (key == nullptr)
        return ptp_l2_auth_result_t::unknown_key;
    const psa_status_t status = psa_mac_verify(key->handle, MAC_ALGORITHM, message, length - PTP_L2_AUTH_ICV_SIZE,
                                               tlv + 10, PTP_L2_AUTH_ICV_SIZE);
    if (status == PSA_SUCCESS)
        return ptp_l2_auth_result_t::valid;
    if (status != PSA_ERROR_INVALID_SIGNATURE)
        ESP_LOGE(TAG, "PTP HMAC verification failed: %ld", static_cast<long>(status));
    return ptp_l2_auth_result_t::invalid_mac;
}

bool ptp_l2_auth_append(uint8_t *message, size_t capacity, size_t *length)
{
    if (message == nullptr || length == nullptr || *length < 34 || *length > capacity ||
        capacity - *length < PTP_L2_AUTH_TLV_SIZE || body_size(message[0]) != *length ||
        read_u16(message + 2) != *length || message[1] != 0x12 || !s_available)
    {
        ESP_LOGE(TAG, "Cannot sign PTP message: invalid buffer, header, or authentication state");
        return false;
    }
    const ptp_l2_auth_key_t *key = find_key(PTP_L2_AUTHENTICATION_ACTIVE_KEY_ID);
    if (key == nullptr)
    {
        ESP_LOGE(TAG, "Cannot sign PTP message: active key unavailable");
        return false;
    }
    const size_t total = *length + PTP_L2_AUTH_TLV_SIZE;
    uint8_t *tlv = message + *length;
    tlv[0] = 0x80;
    tlv[1] = 0x09;
    tlv[2] = 0;
    tlv[3] = PTP_L2_AUTH_TLV_SIZE - 4;
    tlv[4] = PTP_L2_AUTHENTICATION_SPP;
    tlv[5] = 0; // Immediate authentication; no optional sequence/reserved fields.
    for (size_t i = 0; i < 4; ++i)
        tlv[6 + i] = static_cast<uint8_t>(key->id >> (24 - 8 * i));
    message[2] = static_cast<uint8_t>(total >> 8);
    message[3] = static_cast<uint8_t>(total);
    size_t mac_length = 0;
    const psa_status_t status = psa_mac_compute(key->handle, MAC_ALGORITHM, message, total - PTP_L2_AUTH_ICV_SIZE,
                                                tlv + 10, PTP_L2_AUTH_ICV_SIZE, &mac_length);
    if (status != PSA_SUCCESS || mac_length != PTP_L2_AUTH_ICV_SIZE)
    {
        ESP_LOGE(TAG, "PTP HMAC signing failed: %ld", static_cast<long>(status));
        return false;
    }
    *length = total;
    return true;
}

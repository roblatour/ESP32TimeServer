#pragma once

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <bcrypt.h>
#include <cstddef>
#include <cstdint>
#include <cstring>
#pragma comment(lib, "bcrypt.lib")

// Host adapter only: run the firmware's PSA call sites against Windows crypto.
using psa_status_t = int;
using psa_key_id_t = unsigned;
using psa_algorithm_t = unsigned;
static constexpr int PSA_SUCCESS = 0, PSA_ERROR_INVALID_SIGNATURE = -149;
static constexpr unsigned PSA_ALG_SHA_256 = 1, PSA_KEY_TYPE_HMAC = 1;
static constexpr unsigned PSA_KEY_USAGE_SIGN_MESSAGE = 1, PSA_KEY_USAGE_VERIFY_MESSAGE = 2;
#define PSA_ALG_HMAC(algorithm) (0x100u | (algorithm))
#define PSA_ALG_TRUNCATED_MAC(algorithm, length) ((algorithm) | (static_cast<unsigned>(length) << 16))
struct psa_key_attributes_t { unsigned unused = 0; };
#define PSA_KEY_ATTRIBUTES_INIT {}
struct psa_hash_operation_t { BCRYPT_ALG_HANDLE algorithm = nullptr; BCRYPT_HASH_HANDLE hash = nullptr; };
#define PSA_HASH_OPERATION_INIT {}
struct fixture_key_t { uint8_t data[64]{}; size_t length = 0; };
inline fixture_key_t s_fixture_keys[8]{};
inline bool s_fixture_crypto_failure = false;
inline bool s_fixture_init_failure = false;
inline unsigned s_fixture_init_calls = 0, s_fixture_import_calls = 0, s_fixture_destroy_calls = 0;
inline unsigned s_fixture_import_failure_at = 0;
inline psa_status_t psa_crypto_init()
{
    ++s_fixture_init_calls;
    return s_fixture_crypto_failure || s_fixture_init_failure ? -1 : PSA_SUCCESS;
}
inline void psa_set_key_type(psa_key_attributes_t *, unsigned) {}
inline void psa_set_key_bits(psa_key_attributes_t *, unsigned) {}
inline void psa_set_key_usage_flags(psa_key_attributes_t *, unsigned) {}
inline void psa_set_key_algorithm(psa_key_attributes_t *, unsigned) {}
inline void psa_reset_key_attributes(psa_key_attributes_t *) {}
inline psa_status_t psa_import_key(const psa_key_attributes_t *, const uint8_t *key, size_t length, psa_key_id_t *id)
{
    ++s_fixture_import_calls;
    if (s_fixture_crypto_failure || length > 64 || s_fixture_import_calls == s_fixture_import_failure_at)
        return -1;
    for (unsigned i = 1; i < 8; ++i)
    {
        if (s_fixture_keys[i].length == 0)
        {
            std::memcpy(s_fixture_keys[i].data, key, length);
            s_fixture_keys[i].length = length;
            *id = i;
            return PSA_SUCCESS;
        }
    }
    return -1;
}
inline psa_status_t psa_destroy_key(psa_key_id_t id)
{
    ++s_fixture_destroy_calls;
    if (id < 8)
        s_fixture_keys[id] = {};
    return PSA_SUCCESS;
}
inline psa_status_t psa_hash_abort(psa_hash_operation_t *operation)
{
    if (operation->hash)
        BCryptDestroyHash(operation->hash);
    if (operation->algorithm)
        BCryptCloseAlgorithmProvider(operation->algorithm, 0);
    *operation = {};
    return PSA_SUCCESS;
}
inline psa_status_t psa_hash_setup(psa_hash_operation_t *operation, psa_algorithm_t)
{
    if (s_fixture_crypto_failure ||
        BCryptOpenAlgorithmProvider(&operation->algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0) < 0 ||
        BCryptCreateHash(operation->algorithm, &operation->hash, nullptr, 0, nullptr, 0, 0) < 0)
        return -1;
    return PSA_SUCCESS;
}
inline psa_status_t psa_hash_update(psa_hash_operation_t *operation, const uint8_t *data, size_t length)
{
    return BCryptHashData(operation->hash, const_cast<PUCHAR>(data), static_cast<ULONG>(length), 0) < 0 ? -1 : PSA_SUCCESS;
}
inline psa_status_t psa_hash_finish(psa_hash_operation_t *operation, uint8_t *output, size_t capacity, size_t *length)
{
    if (capacity < 32 || BCryptFinishHash(operation->hash, output, 32, 0) < 0)
        return -1;
    *length = 32;
    return PSA_SUCCESS;
}
inline psa_status_t psa_mac_compute(psa_key_id_t id, psa_algorithm_t, const uint8_t *data, size_t size,
                                    uint8_t *output, size_t capacity, size_t *length)
{
    if (s_fixture_crypto_failure || id >= 8 || s_fixture_keys[id].length == 0 || capacity < 16)
        return -1;
    psa_hash_operation_t operation{};
    uint8_t full[32]{};
    const auto &key = s_fixture_keys[id];
    const bool ok = BCryptOpenAlgorithmProvider(&operation.algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, BCRYPT_ALG_HANDLE_HMAC_FLAG) >= 0 &&
                    BCryptCreateHash(operation.algorithm, &operation.hash, nullptr, 0, const_cast<PUCHAR>(key.data), static_cast<ULONG>(key.length), 0) >= 0 &&
                    BCryptHashData(operation.hash, const_cast<PUCHAR>(data), static_cast<ULONG>(size), 0) >= 0 &&
                    BCryptFinishHash(operation.hash, full, sizeof(full), 0) >= 0;
    psa_hash_abort(&operation);
    if (!ok)
        return -1;
    std::memcpy(output, full, 16);
    *length = 16;
    return PSA_SUCCESS;
}
inline psa_status_t psa_mac_verify(psa_key_id_t id, psa_algorithm_t algorithm, const uint8_t *data, size_t size,
                                   const uint8_t *mac, size_t length)
{
    uint8_t computed[16]{};
    size_t computed_length = 0;
    if (length != sizeof(computed) || psa_mac_compute(id, algorithm, data, size, computed, sizeof(computed), &computed_length) != PSA_SUCCESS)
        return -1;
    uint8_t difference = 0;
    for (size_t i = 0; i < length; ++i)
        difference |= computed[i] ^ mac[i];
    return difference == 0 ? PSA_SUCCESS : PSA_ERROR_INVALID_SIGNATURE;
}

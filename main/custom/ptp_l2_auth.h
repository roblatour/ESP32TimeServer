// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT

#pragma once

#include <cstddef>
#include <cstdint>

static constexpr size_t PTP_L2_AUTH_TLV_SIZE = 26;
static constexpr size_t PTP_L2_AUTH_ICV_SIZE = 16;
static constexpr size_t PTP_L2_AUTH_MAX_KEYS = 4;

enum class ptp_l2_auth_result_t : uint8_t
{
    unauthenticated,
    valid,
    authentication_required,
    malformed,
    unavailable,
    unknown_key,
    invalid_mac,
};

// Call before publishing PTP tasks. Returns cached policy readiness, not crypto
// availability: optional unprovisioned service is ready but remains unsigned.
bool ptp_l2_auth_initialize();
// True only after the complete key ring has been validated and imported.
bool ptp_l2_auth_available();
ptp_l2_auth_result_t ptp_l2_auth_verify(const uint8_t *message, size_t message_length);
bool ptp_l2_auth_append(uint8_t *message, size_t capacity, size_t *message_length);

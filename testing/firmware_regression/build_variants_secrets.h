#pragma once

// Synthetic build-only values; never use these in a deployed server.
static constexpr char OTEPassword[] = "build-test-only";
static constexpr char MQTTUsername[] = "build-test-only";
static constexpr char MQTTPassword[] = "build-test-only";
static constexpr symmetric_key_t symmetric_keys[] = {{1, "0123456789abcdef0123456789abcdef"}};
static constexpr symmetric_key_t ptp_l2_symmetric_keys[] = {{1, "0123456789abcdef0123456789abcdef"}};

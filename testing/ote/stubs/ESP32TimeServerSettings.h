#pragma once
#include <cstdint>
#define OTE_UPDATES_ENABLED 1
#define CONFIG_LWIP_IPV6 1
#ifndef TEST_PREFER_IPVX
#define TEST_PREFER_IPVX 0
#endif
static constexpr int PreferIPvX = TEST_PREFER_IPVX;
static constexpr char OTEPassword[] = "test-password";
static constexpr char HostName[] = "TestTimeServer";
static constexpr uint16_t OTEPort = 3232;
static constexpr unsigned int OTEReceiveTimeoutSeconds = 15;
static constexpr unsigned int OTEMaxUploadTimeSeconds = 300;

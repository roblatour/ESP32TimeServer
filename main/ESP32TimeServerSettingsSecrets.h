// ESP32 Time Server v4
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#include <array>

// Over The Ethernet password for the ESP32 Time Server
static constexpr char OTEPassword[] = "ESP32TimeServerpw";

// Credentials for the ESP32 Time Server as an MQTT client
static constexpr char MQTTUsername[] = "";
static constexpr char MQTTPassword[] = "";

// Symmetric keys for NTPv4 authentication
// NTP use and authentication must be properly enabled in the ESP32TimeServerSettings.h file to make use of these keys
// NTP uses 32-character alphanumeric keys; these values may be different from the PTP L2 symmetric keys below.
// For more information on setting up NTP symmetric keys, please see the 'NTP server and authenticated clients' section in misc/setup.md 
static constexpr symmetric_key_t symmetric_keys[] = {
    {1, "abcdefghijklmnopqrstuvwxyz123456"},
};

// Symmetric keys for PTP L2 authentication
// PTP L2 use and authentication must be properly enabled in the ESP32TimeServerSettings.h file to make use of these keys
// PTP L2 uses 32-character alphanumeric keys; these values may be different from the NTP symmetric keys above.
// For more information on setting up PTP L2 symmetric keys, please see the 'Layer 2 PTP server and LinuxPTP client' section in misc/setup.md 
static constexpr symmetric_key_t ptp_l2_symmetric_keys[] = {
    {1, "ABCDEFGHIJKLMNOPQRSTUVWXYZ123456"},
};

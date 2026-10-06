// ESP32 Time Server v3.0.3
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

// Over The Ethernet password for the ESP32 Time Server
static constexpr char OTEPassword[] = "ESP32TimeServerpw";

// Credentials for the ESP32 Time Server as an MQTT client
static constexpr char MQTTUsername[] = "";
static constexpr char MQTTPassword[] = "";

// Symmetric keys for NTPv4 packet authentication
// Please see misc/symmetric_key_authentication.md for instructions on generating and managing symmetric keys.
static constexpr symmetric_key_t symmetric_keys[] = {
    {1, "abcdefghijklmnopqrstuvwxyz123456"},
};
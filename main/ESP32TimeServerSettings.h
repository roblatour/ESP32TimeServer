// ESP32 Time Server v4.0
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//
// Note: some additional settings are defined in the ESP32TimeServerSettingsSecrets.h file
//

#pragma once
#include <cstddef>
#include <cstdint>
#include <ctime>
#include "sdkconfig.h"
#include "custom/setting_types.c"
#include "ESP32TimeServerSettingsSecrets.h"

// (Required) Hostname for this device on the network - used by OTE updates when enabled
static constexpr char HostName[] = "ESP32TimeServer";

// Enable any combination of time services, but not none.
#define NTP_SERVER_ENABLED 1    // 0 = Disabled; 1 = Enabled
#define PTP_L2_SERVER_ENABLED 0 // 0 = Disabled; 1 = Enabled
#define PTP_L3_SERVER_ENABLED 0 //  0 = Disabled; 1 = Enabled; also set PreferIPvX below to support dual-stack - for example PreferIPv4 = 0

// (Required when NTP is enabled) NTPv4 symmetric-key packet authentication.
// When enabled, only authenticated NTPv4 requests with a valid configured key receive signed replies.
// NTP_AUTHENTICATION_REQUIRED selects mixed (0) or authenticated-only (1) service.
// Please also see ESP32TimeServerSettingsSecretes.h for the configuration of symmetric keys.
#define NTP_AUTHENTICATION_ENABLED 1  // 0 = Disabled; 1 = Enabled
#define NTP_AUTHENTICATION_REQUIRED 0 // 0 = Mixed; 1 = Reject unsigned NTP (including NTPv3)

// (Required when PTP L2 is enabled)
#define PTP_L2_DOMAIN 0
#define PTP_L2_LOG_SYNC_INTERVAL 0
#define PTP_L2_LOG_ANNOUNCE_INTERVAL 1
#define PTP_L2_PRIORITY1 128
#define PTP_L2_PRIORITY2 128
#define PTP_L2_REQUEST_QUEUE_DEPTH 8
#define PTP_L2_FOLLOWER_CAPACITY 32
#define PTP_L2_FOLLOWER_TIMEOUT_SECONDS 30
#define PTP_L2_FOREIGN_ANNOUNCE_TIMEOUT_SECONDS 10
#define PTP_L2_MASTER_SELECTION_ENABLED 1 // 0 = Legacy any-Announce suppression; 1 = GNSS-only ranked election
#define PTP_L2_FOREIGN_MASTER_CAPACITY 8  // 1..32 better-master records; two fresh Announce messages qualify a peer

// (optional when PTP is enabled) IEEE 1588-2019 PTP AUTHENTICATION TLV, HMAC-SHA256-128.
// Independent of NTP: configure ptp_l2_symmetric_keys in the included secrets header.
// With keys, enabled PTP signs all outgoing messages; optional mode accepts unsigned input.
// An intentionally empty PTP ring permits unsigned-only service when not required.
#define PTP_L2_AUTHENTICATION_ENABLED 1       // 0 = Legacy unsigned PTP; 1 = Enable authentication
#define PTP_L2_AUTHENTICATION_REQUIRED 0      // 0 = Mixed; 1 = Reject unsigned PTP
#define PTP_L2_AUTHENTICATION_SPP 1           // Security parameter pointer: 0..255
#define PTP_L2_AUTHENTICATION_ACTIVE_KEY_ID 1 // Outgoing key; must exist in ptp_l2_symmetric_keys

// (Required when PTP L2 is enabled)
#define PTP_L3_UDP_DOMAIN 0
#define PTP_L3_UDP_LOG_SYNC_INTERVAL 0
#define PTP_L3_UDP_LOG_ANNOUNCE_INTERVAL 1
#define PTP_L3_UDP_PRIORITY1 128
#define PTP_L3_UDP_PRIORITY2 128
#define PTP_L3_UDP_REQUEST_QUEUE_DEPTH 8
#define PTP_L3_UDP_FOLLOWER_CAPACITY 32
#define PTP_L3_UDP_FOLLOWER_TIMEOUT_SECONDS 30
#define PTP_L3_UDP_FOREIGN_ANNOUNCE_TIMEOUT_SECONDS 10
#define PTP_L3_UDP_MASTER_SELECTION_ENABLED 1
#define PTP_L3_UDP_FOREIGN_MASTER_CAPACITY 8

// (Optional) debug support
// NOTE: setting DEBUG_ENABLED to 1 (Enabled) will degrade performance during periods of high volume ntp requests
#define DEBUG_ENABLED 0 // 0 = Disabled; 1 = Enabled
// Baud for an ESP-IDF UART console; USB Serial/JTAG ignores this setting.
// UART1 is reserved for GNSS and cannot also be used as the console.
static constexpr int serialMonitorSpeed = 115200;

// (optional) RGB LED module support; enable support for one module type below at most
#define RBG_LED_KY_016_MODULE_ENABLED 0 // 0 = Disabled; 1 = Enabled
#define RBG_LED_WS2812_MODULE_ENABLED 0 // 0 = Disabled; 1 = Enabled

static constexpr int RGB_LED_Brightness_Percent = 60;

// The LED colours below may be changed to suit your preferences, but the descriptors should remain unchanged
// For example: you can change LED_normal to blue but leave the descriptor 'LED_normal' unchanged
// For more information, please see ./misc/status_indicators.md
static constexpr RGB_LED_Colour LED_startup = RGB_LED_Colour::blue;
static constexpr RGB_LED_Colour LED_normal = RGB_LED_Colour::green;
static constexpr RGB_LED_Colour LED_sync = RGB_LED_Colour::white;
static constexpr RGB_LED_Colour LED_warning = RGB_LED_Colour::yellow;
static constexpr RGB_LED_Colour LED_alert = RGB_LED_Colour::orange;
static constexpr RGB_LED_Colour LED_critical = RGB_LED_Colour::red;
static constexpr RGB_LED_Colour LED_button_push = RGB_LED_Colour::pink;

// (Optional) uptime / reset momentary button support
#define UPTIME_RESTART_BUTTON_ENABLED 0 // 0 = Disabled; 1 = Enabled
static constexpr unsigned long holdUpTimeRestartButtonForThisManySecondsToTriggerAReset = 10UL;
static constexpr int upTimeDisplayWillStayActiveForThisManySeconds = 10;

// (Optional) attached LCD support
// IMPORTANT NOTE: ****************************************************************************************
// Some LCD modules operate at 3.3v logic levels, while others operate at 5v logic levels.                *
// Only connect a 3.3v LCD.                                                                               *
// Never power the LCD by 5v (as it can damage the ESP32 GPIO pins)                                       *
// ********************************************************************************************************
#define LIQUID_CRYSTAL_DISPLAY_ENABLED 1 // 0 = Disabled; 1 = Enabled
static constexpr int lcdColumns = 20;
static constexpr int lcdRows = 4;
static constexpr char lcdTopLineTitle[] = "ESP32 Time Server"; // text on the LCD's top line (max 17 characters; the last 3 columns are reserved for status indicators)
static constexpr bool display24HourFormat = false;             // set to true for 24-hour format, false for 12-hour format
static constexpr bool displayHoursWithPaddedZero = false;      // set to true to pad the hour with a leading zero when less than 10, false not to pad
static constexpr bool displayTimeZone = false;
static constexpr int lcdI2CAddressPrimary = 0x27;
static constexpr int lcdI2CAddressSecondary = 0x3F;

// (Required) Preference for network connection (even if LIQUID_CRYSTAL_DISPLAY_ENABLED is set to 0)
static constexpr int PreferIPvX = 0; // 0 - dual-stack, LCD rotates IPv4, IPv6, and MAC addresses as appropriate based on enabled settings
                                     // 1 - use IPv4 only, LCD shows IPv4 address only
                                     // 2 - use IPv6 only, LCD shows IPv6 address only
                                     // 4 - dual-stack, but LCD shows IPv4 address only
                                     // 6 - dual-stack, but LCD shows IPv6 address only
                                     // 7 - Layer 2 (L2) Ethernet only (no IPv4/IPv6); LCD shows the MAC ("AA:BB:CC:DD:EE:FF")
                                     // 8 - dual-stack, but LCD shows IPv4 and IPv6 address status only ("IPv4: OK/N/A  IPv6: GUA/ULA/LLA/N/A")
                                     // 9 - dual-stack, but leave the LCD bottom line blank for privacy/security reasons
                                     //
                                     // Notes:
                                     // - Dual-stack provides for both IPv4 and IPv6 support, but also inherently includes L2 Ethernet support.
                                     // - With prefer IPv4: if an IPv6 address is offered it will be temporarily accepted,
                                     //   until an IPv4 offer comes in at which time the program will automatically switch over
                                     //   to an IPv4 address.
                                     // - The same is true, in reverse, for prefer IPv6.
                                     // - With a value of 0, 4, 6, 8, or 9 NTP requests from both IPv4 and IPv6 clients are accepted.
                                     // - With values 1 and 2, NTP and UDP PTP use only the selected IP version.
                                     // - For IPv6 addresses, the LCD will display the appropriate status based on highest scope address type
                                     //   (Global-Unicast address (GUA), Unique-Local address (ULA), or Link-Local address (LLA)) assigned:
                                     //   i.e. if a GUA is assigned it will be shown, otherwise if a ULA is assigned it will be shown,
                                     //   otherwise if a LLA is assigned it will be shown.
                                     // - option 7 requires NTP, UDP PTP, OTE, and MQTT to be disabled.

// (Optional) Over The Ethernet updates support
// Please also see ESP32TimeServerSettingsSecretes.h for the configuration of the OTE password.
#define OTE_UPDATES_ENABLED 1 // 0 = Disabled; 1 = Enabled
static constexpr uint16_t OTEPort = 3232;
static constexpr unsigned int OTEReceiveTimeoutSeconds = 15;
static constexpr unsigned int OTEMaxUploadTimeSeconds = 300;

// (Optional) MQTT reporting support
// MQTT_ENABLED provides for current system status and activity reporting on information such as:
// uptime, number of active satellites, ntp request counts, etc.
// Additional detailed related to client activity, memory usage, and historical information can be optionally included
// When symmetric-key authentication is enabled, MQTT reporting can distinguish between authenticated and unauthenticated requests as well
// Please also see ESP32TimeServerSettingsSecretes.h for the configuration of the MQTT credentials.
#define MQTT_ENABLED 1                                     // 0 = Disabled; 1 = Enabled
#define MQTT_CLIENT_REPORTING_ENABLED 1                    // 0 = Disabled; 1 = Enabled
#define MQTT_MEMORY_REPORTING_ENABLED 1                    // 0 = Disabled; 1 = Enabled
#define MQTT_HISTORICAL_REPORTING_ENABLED 1                // 0 = Disabled; 1 = Enabled
#define MQTT_DISTINGUISH_AUTHENTICATED_REPORTING_ENABLED 0 // 0 = Disabled; 1 = Enabled

static constexpr char MQTTServerIPAddress[] = ""; // For example 192.168.1.15
static constexpr char MQTTTopic[] = "ESP32TimeServer";
static constexpr uint16_t MQTTPort = 1883;
static constexpr uint16_t MQTTBrokerRetain = 1;      // 0 = tell the broker not to retain the most current message; 1 = tell the broker to retain the most current message
static constexpr uint32_t MQTTReportingPeriod = 900; // in seconds - 900 for production

// QoS 0 delivers at most once and does not retain reports while disconnected
// QoS 1 delivers at least once and queues reports while disconnected
// QoS 2 delivers exactly once and queues reports while disconnected

static constexpr int MQTT_QOS = 0;

// Notes:
// For MQTT_QOS > 0
//
//   If TF is present and supports read/write operations then report queuing will be done using the TF card
//   (an empty 16GB TF card can potentially hold more than 36,000 queued messages).  Regardless, if queued reports do
//   exceed the TF storage capacity they will be managed on a FIFO basis.
//
//   If in processing there is a problem with reading or writing to the TF card, the affected reports will be queued
//   in RAM instead (if possible), subject to MQTT_RAM_QUEUE_BYTES limits (below).
//
//   If a TF card reader is not enabled, or if a TF card, formatted using FAT32, is not present in the TF reader, or
//   if read/write operations to it fail then reports that cannot be published are queued in RAM (MQTT_RAM_QUEUE_BYTES below).
//   Reports vary in size, so the number held depends on how many clients each one lists; when the queue is full the oldest reports
//   are dropped to make room for the newest.  RAM-queued reports are lost if power is lost or there is an unexpected system crash;
//   flash ram is not used so as to not degrade it over time.
//
//   However, if you never want the TF card reader to be used, set MQTT_USE_TF_CARD to 0 (below) and
//   RAM will be used for queuing regardless of event if a TF card reader and card are present.
//
//   MQTT report client limits (these apply with or without a TF card)
//   MQTT_NTP_CLIENT_REPORT_LIMIT is the most unique NTP client addresses (IPv4 or IPv6) listed in one report;
//   further clients are counted in the request totals and flagged by "clients_overflown".
//   MQTT_PTP_CLIENT_REPORT_LIMIT is the most unique PTP follower MACs listed in one report; it must be at least
//   the sum of enabled Layer 2 and per-family UDP capacities so every continuous follower is listed.
//   The report buffer is sized automatically from these limits; no other values need recalculating.
//
//   WARNING: raising these limits (or MQTT_RAM_QUEUE_BYTES) uses more RAM and could cause the system to run out of
//            memory, either at startup or later while it is running.  Before and after a change, enable
//            MQTT_MEMORY_REPORTING_ENABLED above and check "current" -> "memory" -> "minimum_free_heap"
//            in the published report: it is the lowest free RAM seen since startup, so it is the amount of spare room.
//            Keep a comfortable margin (at least 64 KB is suggested) for future features and network bursts, and also
//            check that "largest_free_8bit_block" stays well above 16 KB.
//
//            PSRAM is not used in this solution as it introduces hardware-specific configuration complexities,
//            slower boot times, and potential bus-contention timing issues that complicate system reporting and
//            validation under load.
//
//   RAM cost of each change (approximate):
//
//     +1 MQTT_NTP_CLIENT_REPORT_LIMIT = +106 bytes (82 bytes of report buffer + 24 bytes of client table)
//     +1 MQTT_PTP_CLIENT_REPORT_LIMIT = +71 bytes  (64 bytes of report buffer + 7 bytes of follower table),
//                                       plus 14 bytes of MQTT task stack while a report is built
//
//     +1 KB of MQTT_RAM_QUEUE_BYTES   = +1,024 bytes
//
//   The RAM queue must hold at least one full-size report; the build fails with a message if it cannot.
//   With the defaults one full-size report is about 26 KB, but typical reports are far smaller so the
//   64 KB queue usually holds many reporting periods.

#define MQTT_USE_TF_CARD 1 // 0 = Disabled (never use TF card even if available); 1 = Enabled (use TF card if available)

static constexpr size_t MQTT_NTP_CLIENT_REPORT_LIMIT = 256;

// The MQTT_PTP_CLIENT_REPORT_LIMIT is automatically calculated based on the enabled PTP server types and their configured capacities (defined above).  Do not change the line of code below:
static constexpr size_t MQTT_PTP_CLIENT_REPORT_LIMIT = (PTP_L2_SERVER_ENABLED ? PTP_L2_FOLLOWER_CAPACITY : 0) + (PTP_L3_SERVER_ENABLED ? PTP_L3_UDP_FOLLOWER_CAPACITY * ((PreferIPvX != 2) + (PreferIPvX != 1)) : 0);

static constexpr size_t MQTT_RAM_QUEUE_BYTES = 64 * 1024; // total RAM queue size in bytes; default total value 64KB; the total must be a multiple of 4

// (Optional) setting of the ESP32's MAC address support
// set the value below to "" to use the ESP32's default MAC address, or
// set the value below to a desired MAC address, such as "80:f1:b2:d1:d9:18"

static constexpr char MACAddress[] = "";

// (Optional) setting a static IP address for the ESP32-P4 support
// set StaticIPAddress to "" to use DHCP (the default behaviour), which allows the router to
// auto-assign an IP address, or set it to a desired static IP address such as "192.168.1.100".
// when a static IP address is set, Gateway and SubnetMask must also be provided.
// PrimaryDNS and SecondaryDNS are optional; leave them as "" to omit them.

static constexpr char StaticIPAddress[] = ""; // optional - leave as "" to omit, or for example: 192.168.1.100
static constexpr char Gateway[] = "";         // optional - leave as "" to omit, or for example: : 192.168.1.1
static constexpr char SubnetMask[] = "";      // optional - leave as "" to omit, or for example: 255.255.255.0
static constexpr char PrimaryDNS[] = "";      // optional - leave as "" to omit, or for example: 8.8.8.8
static constexpr char SecondaryDNS[] = "";    // optional - leave as "" to omit, or for example: 1.1.1.1

// (Required) Time zone setting for your region - for more information see https://gist.github.com/alwynallan/24d96091655391107939
static constexpr const char *timeZoneSpec = "EST5EDT,M3.2.0/2,M11.1.0/2";

// (Required) GNSS support

// IMPORTANT NOTES: ***************************************************************************************
// Some GNSS receivers and breakout boards provide a 3.3v PPS (Pulse-Per-Second) output pin               *
// while others provide a 5v PPS output pin.                                                              *
// 1. Only connect a 3.3v PPS pin to an ESP32 GPIO pin, never connect a 5v PPS pin to an ESP32 GPIO pin   *
// 2. For the wire between the ESP32-P4's GPIO pin and the GNSS Receiver's PPS pin                        *
//    use as short and as well connect wire as possible; a standard copper Dupont wire                    *
//    will be fine assuming its gauge is between 24 AWG and 28AWG                                         *
// 3. The GNSS GND must be connected to the ESP32 GND                                                     *
// ********************************************************************************************************

#define UBLOX_COMPLIANT_GNSS_RECEIVER_ENABLED 1 // 0 = Disabled; 1 = Enabled

// The following only needs to be set if your receiver does not use a U-Blox or U-Block clone GNSS chip

static constexpr uint32_t baudRateForUbloxNonCompliantGNSSReceiver = 115200;

// Unless you know what you are doing, the options below should be left as is

static constexpr int64_t ntpHoldoverDurationUs = 180LL * 1000000LL; // Holdover duration in microseconds (180 seconds)
static constexpr uint32_t ntpHoldoverDispersionPpm = 200;           // Conservative uncertainty growth while coasting on the learned oscillator rate.

// This project was designed and tested to work with a SparkFun GNSS Receiver Breakout board which uses a u-blox - MAX-M10S module.
// ( https://www.sparkfun.com/sparkfun-gnss-receiver-breakout-max-m10s-qwiic.html )
// However, the code has fallback logic for non/cloned/older u-blox gnss modules and has been tested with one such device as well.
//
// The setting below determines if the code should provide processing for other (than the MAX-M10S) gnss module - even if they are less capable/potentially less accurate.

// Note: even if fallback processing is set to true below, accuracy should still be fine as long as PPS is disciplined.
// For more information here are some detailed timing accuracy notes:
// - This firmware timestamps NTP responses using gettimeofday() system call, which is backed by the ESP32-P4's high-resolution timer
//   driven by the built-in 40 MHz external XTAL via the APB clock (~10 ppm). This is the most accurate clock available
//   on this chip; no external 32.768 kHz RTC crystal is needed or beneficial for NTP timestamping purposes.
// - The PPS discipline task applies continuous sub-second corrections on every GNSS PPS pulse.
//   This reduces inter-sync error to well under 1 ms, limited mainly by interrupt latency (~10-100 us).
// - Preferred GNSS (MAX-M10S) connects at 921600 baud, minimizing serial latency and enabling faster, more precise time
//   message processing. Fallback modules may be limited to 9600 baud.

static constexpr bool allowFallbackProcessing = true;

static constexpr uint32_t periodicGNSSRefreshEveryThisNumberOfMinutes = 5UL; // resync with GNSS every 5 minutes (recommended)

// a reboot during setup may be required to facilitate initial setup of non/clone/older u-blox gnss modules;
// recommend leaving it set to true unless code continually restarts when this setting is used

static constexpr bool rebootIfGNSSBaudChangeCommandSucceedsButImmediateReconnectFails = true;

static constexpr time_t safeguardThresholdInSeconds = 1; // When a new GNSS reading is taken the difference between it and the last reading
                                                         // should be sub-second, if not a sanity check safeguard flag is tripped

static constexpr bool rebootIfSanityCheckFails = true; // Further to the above,
                                                       // if the sanity check fails, the system will either:
                                                       // - automatically reboot if this setting is set to true, or
                                                       // - display double asterisks'**' on the LCD screen to indicate an issue with the GNSS
                                                       //   and then will providing the esp32's time, unsynced by the GNSS, moving forward.

// Do not change the next line of code

#define PTP_ENABLED (PTP_L2_SERVER_ENABLED || PTP_L3_SERVER_ENABLED)

// Do not change the next seven lines of code

#if CONFIG_IDF_TARGET_ESP32P4
#define ESP32_CHIP_TYPE "P4"
#elif CONFIG_IDF_TARGET_ESP32S31
#define ESP32_CHIP_TYPE "S31"
#else
#error "ESP32TimeServer supports only ESP32-P4 and ESP32-S31"
#endif

// Based on the ESP32 chip type, the GPIO pin definitions for the Waveshare ESP32-P4 Ethernet board or the ESP32-S31-Function-CoreBoard-1 will be used

#if CONFIG_IDF_TARGET_ESP32P4
#define TIME_SERVER_BOARD_HAS_TF_CARD 1
// Waveshare P4 Ethernet board wiring.
static constexpr int LEDBluePin = 4;
static constexpr int LEDGreenPin = 5;
static constexpr int LEDRedPin = 6;
static constexpr int WS2812DataPin = 4;
static constexpr int LCDI2CSDAPin = 8;
static constexpr int LCDI2CSCLPin = 7;
static constexpr int upTimeRestartPin = 3;

// (required) GPIO pin definitions for the GNSS module
static constexpr int TXPin = 22;  // note: prior to release 2.4 pin 16 was used for TX (connects to RX on the GNSS receiver)
static constexpr int RXPin = 21;  // note: prior to release 2.4 pin 17 was used for RX (connects to TX on the GNSS receiver)
static constexpr int PPSPin = 20; // note: prior to release 2.4 pin 18 was used for PPS (connects to PPS on the GNSS receiver)

// Optional internal PTP PPS loopback. Requires ESP32-P4 chip revision v3.0 or later
// and a free, unconnected GPIO.
// The pin is driven by the EMAC and captured by MCPWM; no external wire is needed.
// Set to the number of an unused GPIO pin for the internal PTP PPS loopback; set to -1 if not used
static constexpr int PTPPPSLoopbackPin = 54; // -1;

// ESP32-P4 IP101 PHY and external 50 MHz RMII clock (Waveshare Ethernet boards).
static constexpr int EthernetMDCGPIO = 31;
static constexpr int EthernetMDIOGPIO = 52;
static constexpr int EthernetPHYResetGPIO = 51;
static constexpr int EthernetPHYAddress = 1;
static constexpr int EthernetRMIIClockGPIO = 50;
static constexpr int EthernetRMIITXEnableGPIO = 49;
static constexpr int EthernetRMIITX0GPIO = 34;
static constexpr int EthernetRMIITX1GPIO = 35;
static constexpr int EthernetRMIICRSDVGPIO = 28;
static constexpr int EthernetRMIIRX0GPIO = 29;
static constexpr int EthernetRMIIRX1GPIO = 30;

// (required) TF Card GPIO pin definitions for the Waveshare:
// ESP32-P4-ETH, ESP32-P4-POE-ETH, ESP32-P4-WIFI-ETH, ESP32-P4-WIFI-POE-ETH
// if your using this board do not change these
static constexpr int TFCardCommandPin = 44;
static constexpr int TFCardClockPin = 43;
static constexpr int TFCardData3Pin = 42;
static constexpr int TFCardData2Pin = 41;
static constexpr int TFCardData1Pin = 40;
static constexpr int TFCardData0Pin = 39;
#elif CONFIG_IDF_TARGET_ESP32S31
// ESP32-S31-Function-CoreBoard-1 wiring. GPIO23 is for internal loopback only;
// leave its external J2 pin (29, labelled SD_D3) unconnected.
static constexpr int LEDBluePin = 46;
static constexpr int LEDGreenPin = 47;
static constexpr int LEDRedPin = 48;
static constexpr int WS2812DataPin = 60;
static constexpr int LCDI2CSDAPin = 38;
static constexpr int LCDI2CSCLPin = 39;
static constexpr int upTimeRestartPin = 40;
static constexpr int TXPin = 22;             // J2 pin 28, labelled SD_D2 on the ESP32-S31 Function CoreBoard-1 (connects to RX on the GNSS receiver)
static constexpr int RXPin = 21;             // J2 pin 27, labelled SD_D1 on the ESP32-S31 Function CoreBoard-1 (connects to TX on the GNSS receiver)
static constexpr int PPSPin = 20;            // J2 pin 26, labelled SD_D0 on the ESP32-S31 Function CoreBoard-1 (connects to PPS on the GNSS receiver)
static constexpr int PTPPPSLoopbackPin = 23; // J2 pin 29, labelled SD_D3; -1 disables loopback

// Optional 3.3 V SPI TF reader for the MQTT queue (MQTT_QOS > 0).
#define S31_EXTERNAL_TF_CARD_ENABLED 1 // 0 = Disabled (use RAM queue); 1 = Enabled (External TF card reader)
#define TIME_SERVER_BOARD_HAS_TF_CARD S31_EXTERNAL_TF_CARD_ENABLED
// GPIO0/1 require the board's optional 32.768 kHz crystal to remain unpopulated.
static constexpr int TFCardChipSelectPin = 0;      // CS
static constexpr int TFCardMOSIPin = 1;            // MOSI
static constexpr int TFCardClockPin = 2;           // CLK
static constexpr int TFCardMISOPin = 3;            // MISO (sometimes labelled MOSO)
static constexpr int TFCardSPIFrequencyKHz = 4000; // conservative speed for short jumper wires

static constexpr int EthernetMDCGPIO = 5;
static constexpr int EthernetMDIOGPIO = 6;
static constexpr int EthernetPHYResetGPIO = 7;
static constexpr int EthernetPHYAddress = -1;
static constexpr int EthernetRGMIIRXClockGPIO = 14;
static constexpr int EthernetRGMIITXClockGPIO = 13;
static constexpr int EthernetRGMIIPHYRefClockGPIO = -1;
static constexpr int EthernetRGMIITXControlGPIO = 12;
static constexpr int EthernetRGMIITX0GPIO = 8;
static constexpr int EthernetRGMIITX1GPIO = 9;
static constexpr int EthernetRGMIITX2GPIO = 10;
static constexpr int EthernetRGMIITX3GPIO = 11;
static constexpr int EthernetRGMIIRXControlGPIO = 15;
static constexpr int EthernetRGMIIRX0GPIO = 19;
static constexpr int EthernetRGMIIRX1GPIO = 18;
static constexpr int EthernetRGMIIRX2GPIO = 17;
static constexpr int EthernetRGMIIRX3GPIO = 16;
#endif

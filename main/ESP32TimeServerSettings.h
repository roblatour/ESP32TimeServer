#pragma once
#include <cstdint>
#include <ctime>

// (optional) debug support
// NOTE: setting DEBUG_ENABLED to 1 (Enabled) will degrade accuracy and performance during periods of high volume ntp requests
#define DEBUG_ENABLED 0 // 0 = Disabled; 1 = Enabled
static constexpr int serialMonitorSpeed = 115200;

// (optional) up time / reset momentary button support
#define UPTIME_RESTART_BUTTON_ENABLED 1 // 0 = Disabled; 1 = Enabled
static constexpr int upTimeRestartPin = 3;
static constexpr unsigned long holdUpTimeRestartButtonForThisManySecondsToTriggerAReset = 10UL;
static constexpr int upTimeDisplayWillStayActiveForThisManySeconds = 10;

// (optional) attached LCD support
#define LIQUID_CRYSTAL_DISPLAY_ENABLED 1 // 0 = Disabled; 1 = Enabled
static constexpr int lcdI2CAddressPrimary = 0x27;
static constexpr int lcdI2CAddressSecondary = 0x3F;
static constexpr int lcdColumns = 20;
static constexpr int lcdRows = 4;
static constexpr bool displayTimeZone = false;

// (optional) Over The Ethernet updates support
#define OTE_UPDATES_ENABLED 1 // 0 = Disabled; 1 = Enabled
static constexpr char DeviceName[] = "ESP32TimeServer";
static constexpr char OTAPassword[] = "ESP32TimeServerpw";
static constexpr uint16_t OTA_Port = 3232;

// (optional) MQTT reporting support
// MQTT_ENABLED provides for current system status and activity reporting on information such as: uptime, number of active satellites, ntp request counts, etc.
// Additional detailed related to client activity, memory usage, and historical information can be optionally included
#define MQTT_ENABLED 1                                        // 0 = Disabled; 1 = Enabled
#define MQTT_CLIENT_REPORTING_ENABLED 1                       // 0 = Disabled; 1 = Enabled
#define MQTT_MEMORY_REPORTING_ENABLED 1                       // 0 = Disabled; 1 = Enabled
#define MQTT_HISTORICAL_REPORTING_ENABLED 1                   // 0 = Disabled; 1 = Enabled
static constexpr char MQTTServerIPAddress[] = "";             // For example 192.168.1.15
static constexpr uint16_t MQTT_Port = 1883;
static constexpr char MQTTUsername[] = "";
static constexpr char MQTTPassword[] = "";  
static constexpr char MQTTTopic[] = "ESP32TimeServer";
static constexpr uint32_t MQTTReportingPeriod = 900;            // in seconds
static constexpr uint32_t MQTTFrequencyOfKeepAliveRequest = 60; // in seconds
// QoS 0 delivers at most once and does not retain reports while disconnected
// QoS 1 delivers at least once and queues reports while disconnected
// QoS 2 delivers exactly once and queues reports while disconnected
// Notes: - only limit message queuing is provided for MQTT_QOS 1 and 2 (up to 4 prior periods, each detailing the usage of a maximum of 50 unique clients)
//        - message queuing is held in volatile RAM and will be lost if power is lost or there is a unexpected system crash - flash ram is not used so as to not degrade it over time
//          a future release may include using a TF card to improve queuing
static constexpr int MQTT_QOS = 0;

// (optional) setting of the ESP32-P4's MAC address support
// set the value below to "" to use the ESP32-P4's default MAC address, or
// set the value below to a desired MAC address, such as "80:f1:b2:d1:d9:18"
static constexpr char MACAddress[] = ""; 

// (optional) setting a static IP address for the ESP32-P4 support
// set StaticIPAddress to "" to use DHCP (the default behaviour), which allows the router to
// auto-assign an IP address, or set it to a desired static IP address such as "192.168.1.100".
// when a static IP address is set, Gateway and SubnetMask must also be provided.
// PrimaryDNS and SecondaryDNS are optional; leave them as "" to omit them.
static constexpr char StaticIPAddress[] = ""; // optional - leave as "" to omit, or for example: 192.168.1.100
static constexpr char Gateway[] = "";         // optional - leave as "" to omit, or for example: : 192.168.1.1
static constexpr char SubnetMask[] = "";      // optional - leave as "" to omit, or for example: 255.255.255.0
static constexpr char PrimaryDNS[] = "";      // optional - leave as "" to omit, or for example: 8.8.8.8
static constexpr char SecondaryDNS[] = "";    // optional - leave as "" to omit, or for example: 1.1.1.1
static constexpr int PreferIPvX = 4;          // Preference for network connection
                                              // 0 - no preference between IPv4 and IPv6
                                              // 4 - prefer IPv4
                                              // 6 - prefer IPv6
                                              // Note: with prefer IPv4: if an IPv6 address is offered it will be temporarily accepted,
                                              // until an IPv4 offer comes in at which time the program will automatically switch over
                                              // to an IPv4 address.
                                              // The same is true, in reverse, for prefer IPv6.
                                              // Additionally, regardless of the value selected ntp requests from either IPv4 and IPv6
                                              // clients will be accepted

// (required) GPS support
static constexpr int TXPin = 22;  // note: prior to release 2.4 pin 16 was used for TX
static constexpr int RXPin = 21;  // note: prior to release 2.4 pin 17 was used for RX
static constexpr int PPSPin = 20; // note: prior to release 2.4 pin 18 was used for PPS

// This code was designed and tested to work with a SparkFun GNSS Receiver Breakout board which uses a u-blox - MAX-M10S module.
// ( https://www.sparkfun.com/sparkfun-gnss-receiver-breakout-max-m10s-qwiic.html )
// However, the code has fallback logic for non/cloned/older u-blox gps modules and has been tested with one such device as well.
//
// The setting below determines if the code should provide processing for other (than the MAX-M10S) gps module - even if they are less capable/potentially less accurate.

// Note: even if fallback processing is set to true below, accuracy should  still be fine as long as PPS is also supported by the hardware and used.
// For more information here are some detailed timing accuracy notes:
// - This firmware timestamps NTP responses using gettimeofday() system call, which is backed by the ESP32-P4's high-resolution timer
//   driven by the built-in 40 MHz external XTAL via the APB clock (~10 ppm). This is the most accurate clock available
//   on this chip; no external 32.768 kHz RTC crystal is needed or beneficial for NTP timestamping purposes.
// - Without PPS: time is corrected only at each GPS resync (every 5 min by default). The ~10 ppm APB drift yields up to ~3 ms of
//   accumulated error between syncs; temperature variation can push this toward ~6-9 ms worst case.
// - With PPS: the PPS discipline task applies continuous sub-second corrections via adjtime() on every GPS pulse.
//   This reduces inter-sync error to well under 1 ms, limited mainly by interrupt latency (~10-100 us).
//   Accordingly, the use of PPS is highly recommended
// - Preferred GPS (MAX-M10S) connects at 921600 baud, minimizing serial latency and enabling faster, more precise time
//   message processing. Fallback modules may be limited to 9600 baud, introducing additional parsing delay and reducing
//   the accuracy of the time set at each GPS resync.

static constexpr bool allowFallbackProcessing = true;

// Determines if processing should proceed without PPS support; processing without PPS will be less accurate (as described above)
static constexpr bool allowFallbackProcessingWithoutPPS = false;

static constexpr uint32_t periodicGPSRefreshEveryThisNumberOfMinutes = 5UL; // resync with GPS every 5 minutes (recommended)

// a reboot during setup may be required to facilitate initial setup of non/clone/older u-blox gps modules;
// recommend leaving it set to true unless code continually restarts when this setting is used
static constexpr bool rebootIfGpsBaudChangeCommandSucceedsButImmediateReconnectFails = true;

static constexpr time_t safeguardThresholdInSeconds = 1; // When a new GPS reading is taken the difference between it and the last reading
                                                         // should be sub-second, if not a sanity check safeguard flag is tripped

static constexpr bool rebootIfSanityCheckFails = true; // Further to the above,
                                                       // if the sanity check fails, the system will either:
                                                       // - automatically reboot if this setting is set to true, or
                                                       // - display double asterisks'**' on the LCD screen to indicate an issue with the GPS
                                                       //   and then will providing the esp32's time, unsynced by the GPS, moving forward.

// (required) Time zone setting for your region - for more information see https://gist.github.com/alwynallan/24d96091655391107939
static constexpr const char *timeZoneSpec = "EST5EDT,M3.2.0/2,M11.1.0/2";

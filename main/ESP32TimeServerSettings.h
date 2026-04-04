#pragma once

#include <cstdint>
#include <ctime>

// debug displays
static constexpr bool debugIsOn = true;
static constexpr int serialMonitorSpeed = 115200;

// support for an (optional) up time / reset momentary button
static constexpr bool supportForAnUpTimeRestartButton = true;
static constexpr int upTimeRestartPin = 3;
static constexpr unsigned long holdUpTimeRestartButtonForThisManySecondsToTriggerAReset = 10UL;
static constexpr int upTimeDisplayWillStayActiveForThisManySeconds = 10;

// support for an (optional) attached LCD
static constexpr bool supportForLiquidCrystalDisplay = true;
static constexpr int lcdI2CAddressPrimary = 0x27;
static constexpr int lcdI2CAddressSecondary = 0x3F;
static constexpr int lcdColumns = 20;
static constexpr int lcdRows = 4;
static constexpr bool displayTimeZone = false;

// support for (optional) over the ethernet updates
static constexpr bool supportForOTEUpdates = true;
static constexpr char DeviceName[] = "ESP32TimeServer";
static constexpr char OTA_Password[] = "ESP32TimeServerpw";
static constexpr uint16_t OTA_Port = 3232;

// GPS support (required)
static constexpr int TXPin = 16;
static constexpr int RXPin = 17;
static constexpr int PPSPin = 18;

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

// Time zone setting for your region - for more information see https://gist.github.com/alwynallan/24d96091655391107939
static constexpr const char *timeZoneSpec = "EST5EDT,M3.2.0/2,M11.1.0/2";

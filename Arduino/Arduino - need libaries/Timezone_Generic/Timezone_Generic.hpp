/*********************************************************************************************************************************
  Timezone_Generic.hpp
  
  For AVR, ESP8266/ESP32, SAMD21/SAMD51, nRF52, STM32, WT32_ETH01 boards

  Based on and modified from Arduino Timezone Library (https://github.com/JChristensen/Timezone)
  to support other boards such as ESP8266/ESP32, SAMD21, SAMD51, Adafruit's nRF52 boards, etc.

  Copyright (C) 2018 by Jack Christensen and licensed under GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html

  Built by Khoi Hoang https://github.com/khoih-prog/Timezone_Generic
  Licensed under MIT license
  
  Version: 1.10.1

  Version Modified By  Date      Comments
  ------- -----------  ---------- -----------
  1.2.4   K Hoang      17/10/2020 Initial porting to support SAM DUE, SAMD21, SAMD51, nRF52, ESP32/ESP8266, STM32, etc. boards
                                  using SPIFFS, LittleFS, EEPROM, FlashStorage, DueFlashStorage.
  1.2.5   K Hoang      28/10/2020 Add examples to use STM32 Built-In RTC.
  1.2.6   K Hoang      01/11/2020 Allow un-initialized TZ then use begin() method to set the actual TZ (Credit of 6v6gt)
  1.3.0   K Hoang      09/01/2021 Add support to ESP32/ESP8266 using LittleFS/SPIFFS, and to AVR, UNO WiFi Rev2, etc.
                                  Fix compiler warnings.
  1.4.0   K Hoang      04/06/2021 Add support to RP2040-based boards using RP2040 Arduino-mbed or arduino-pico core
  1.5.0   K Hoang      13/06/2021 Add support to ESP32-S2 and ESP32-C3. Fix bug
  1.6.0   K Hoang      16/07/2021 Add support to WT32_ETH01
  1.7.0   K Hoang      10/08/2021 Add support to Ameba Realtek RTL8720DN, RTL8722DM and RTM8722CSM
  1.7.1   K Hoang      10/10/2021 Update `platform.ini` and `library.json`
  1.7.2   K Hoang      02/11/2021 Fix crashing issue for new cleared flash
  1.7.3   K Hoang      01/12/2021 Auto detect ESP32 core for LittleFS. Fix bug in examples for WT32_ETH01
  1.8.0   K Hoang      31/12/2021 Fix `multiple-definitions` linker error
  1.9.0   K Hoang      20/01/2022 Make compatible to old code
  1.9.1   K Hoang      26/01/2022 Update to be compatible with new FlashStorage libraries. Add support to more SAMD/STM32 boards
  1.10.0  K Hoang      06/04/2022 Use Ethernet_Generic library as default. Add support to Portenta_H7 Ethernet and WiFi
  1.10.1  K Hoang      25/09/2022 Add support to `RP2040W` using `CYW43439 WiFi` with `arduino-pico` core
 **********************************************************************************************************************************/

#pragma once

#ifndef TIMEZONE_GENERIC_HPP
#define TIMEZONE_GENERIC_HPP


// For AVR, Teensy, STM32 boards, use EEPROM
// For SAM DUE, use DueFlashStorage. 
// For SAMD, use FlashStorage_SAMD
// For nRF52, use LittleFS/InternalFS.
// For ESP8266, use LitteFS, SPIFFS or EEPROM.
// For ESP32, use SPIFFS or EEPROM.

#define TIMEZONE_GENERIC_VERSION            "Timezone_Generic v1.10.1"

#define TIMEZONE_GENERIC_VERSION_MAJOR      1
#define TIMEZONE_GENERIC_VERSION_MINOR      10
#define TIMEZONE_GENERIC_VERSION_PATCH      1

#define TIMEZONE_GENERIC_VERSION_INT        1010001


#if defined(ARDUINO) && (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <TimeLib.h>    // https://github.com/PaulStoffregen/Time

#include "Timezone_Generic_Debug.h"

// convenient constants for TimeChangeRules
enum week_t   {Last, First, Second, Third, Fourth};
enum dow_t    {Sun = 1, Mon, Tue, Wed, Thu, Fri, Sat};
enum month_t  {Jan = 1, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec};

// structure to describe rules for when daylight/summer time begins,
// or when standard time begins.
typedef struct 
{
  char    abbrev[6];    // five chars max
  uint8_t week;         // First, Second, Third, Fourth, or Last week of the month
  uint8_t dow;          // day of week, 1=Sun, 2=Mon, ... 7=Sat
  uint8_t month;        // 1=Jan, 2=Feb, ... 12=Dec
  uint8_t hour;         // 0-23
  int     offset;       // offset from UTC in minutes
} TimeChangeRule;

#define TZ_DATA_SIZE    sizeof(TimeChangeRule)

class Timezone
{
  public:
    Timezone(const TimeChangeRule& dstStart, const TimeChangeRule& stdStart, uint32_t address = 0);
    Timezone(const TimeChangeRule& stdTime, uint32_t address = 0);
    
    
    // Allow a "blank" TZ object then use begin() method to set the actual TZ.
    // Feature added by 6v6gt (https://forum.arduino.cc/index.php?topic=711259)
    Timezone(uint32_t address = 0);
    
		void init(const TimeChangeRule& dstStart, const TimeChangeRule& stdStart);
		//////
    
    time_t  toLocal(const time_t& utc);
    time_t  toLocal(const time_t& utc, TimeChangeRule **tcr);
    time_t  toUTC(const time_t& local);
    bool    utcIsDST(const time_t& utc);
    bool    locIsDST(const time_t& local);
    void    setRules(const TimeChangeRule& dstStart, const TimeChangeRule& stdStart);
    
    //void    readRules(int address);
    void    readRules();
    void    writeRules(int address = 0);
    
    TimeChangeRule* read_DST_Rule()
    {
      return &m_dst;
    }
    
    TimeChangeRule* read_STD_Rule()
    {
      return &m_std;
    }
    
    void display_DST_Rule();  
    void display_STD_Rule();

  private:
  
    void    initStorage(uint32_t address);
    
    void    calcTimeChanges(int yr);
    void    initTimeChanges();
    time_t  toTime_t(const TimeChangeRule& r, int yr);
    
    TimeChangeRule m_dst;   // rule for start of dst or summer time for any year
    TimeChangeRule m_std;   // rule for start of standard time for any year
    
    time_t m_dstUTC;        // dst start for given/current year, given in UTC
    time_t m_stdUTC;        // std time start for given/current year, given in UTC
    time_t m_dstLoc;        // dst start for given/current year, given in local time
    time_t m_stdLoc;        // std time start for given/current year, given in local time
    
    void      readTZData();
    void      writeTZData(int address);
    
    uint16_t  TZ_DATA_START     = 0;
    
    bool      storageSystemInit = false;
};

#endif    // TIMEZONE_GENERIC_HPP

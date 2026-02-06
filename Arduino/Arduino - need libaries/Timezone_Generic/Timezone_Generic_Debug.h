/*********************************************************************************************************************************
  Timezone_Generic_Debug.h
 
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

#ifndef TIMEZONE_GENERIC_DEBUG_H
#define TIMEZONE_GENERIC_DEBUG_H

#ifdef TZ_DEBUG_PORT
  #define TZ_DBG_PORT      TZ_DEBUG_PORT
#else
  #define TZ_DBG_PORT      Serial
#endif

// Change _TZ_LOGLEVEL_ to set tracing and logging verbosity
// 0: DISABLED: no logging
// 1: ERROR: errors
// 2: WARN: errors and warnings
// 3: INFO: errors, warnings and informational (default)
// 4: DEBUG: errors, warnings, informational and debug

#ifndef _TZ_LOGLEVEL_
  #define _TZ_LOGLEVEL_       1
#endif

///////////////////////////////////////

const char TZ_MARK[]  = "[TZ] ";
const char TZ_SPACE[] = " ";

#define TZ_PRINT        TZ_DBG_PORT.print
#define TZ_PRINTLN      TZ_DBG_PORT.println

#define TZ_PRINT_MARK   TZ_PRINT(TZ_MARK)
#define TZ_PRINT_SP     TZ_PRINT(TZ_SPACE)
#define TZ_PRINT_LINE   TZ_PRINT(TZ_LINE)

///////////////////////////////////////

#define TZ_LOGERROR(x)         if(_TZ_LOGLEVEL_>0) { TZ_PRINT_MARK; TZ_PRINTLN(x); }
#define TZ_LOGERROR0(x)        if(_TZ_LOGLEVEL_>0) { TZ_PRINT(x); }
#define TZ_LOGERROR1(x,y)      if(_TZ_LOGLEVEL_>0) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINTLN(y); }
#define TZ_HEXLOGERROR1(x,y)   if(_TZ_LOGLEVEL_>0) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINTLN(y, HEX); }
#define TZ_LOGERROR2(x,y,z)    if(_TZ_LOGLEVEL_>0) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINT(y); TZ_PRINT_SP; TZ_PRINTLN(z); }
#define TZ_LOGERROR3(x,y,z,w)  if(_TZ_LOGLEVEL_>0) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINT(y); TZ_PRINT_SP; TZ_PRINT(z); TZ_PRINT_SP; TZ_PRINTLN(w); }

///////////////////////////////////////

#define TZ_LOGWARN(x)          if(_TZ_LOGLEVEL_>1) { TZ_PRINT_MARK; TZ_PRINTLN(x); }
#define TZ_LOGWARN0(x)         if(_TZ_LOGLEVEL_>1) { TZ_PRINT(x); }
#define TZ_LOGWARN1(x,y)       if(_TZ_LOGLEVEL_>1) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINTLN(y); }
#define TZ_HEXLOGWARN1(x,y)    if(_TZ_LOGLEVEL_>1) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINTLN(y, HEX); }
#define TZ_LOGWARN2(x,y,z)     if(_TZ_LOGLEVEL_>1) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINT(y); TZ_PRINT_SP; TZ_PRINTLN(z); }
#define TZ_LOGWARN3(x,y,z,w)   if(_TZ_LOGLEVEL_>1) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINT(y); TZ_PRINT_SP; TZ_PRINT(z); TZ_PRINT_SP; TZ_PRINTLN(w); }

///////////////////////////////////////

#define TZ_LOGINFO(x)          if(_TZ_LOGLEVEL_>2) { TZ_PRINT_MARK; TZ_PRINTLN(x); }
#define TZ_LOGINFO0(x)         if(_TZ_LOGLEVEL_>2) { TZ_PRINT(x); }
#define TZ_LOGINFO1(x,y)       if(_TZ_LOGLEVEL_>2) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINTLN(y); }
#define TZ_HEXLOGINFO1(x,y)    if(_TZ_LOGLEVEL_>2) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINTLN(y, HEX); }
#define TZ_LOGINFO2(x,y,z)     if(_TZ_LOGLEVEL_>2) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINT(y); TZ_PRINT_SP; TZ_PRINTLN(z); }
#define TZ_LOGINFO3(x,y,z,w)   if(_TZ_LOGLEVEL_>2) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINT(y); TZ_PRINT_SP; TZ_PRINT(z); TZ_PRINT_SP; TZ_PRINTLN(w); }

///////////////////////////////////////

#define TZ_LOGDEBUG(x)         if(_TZ_LOGLEVEL_>3) { TZ_PRINT_MARK; TZ_PRINTLN(x); }
#define TZ_LOGDEBUG0(x)        if(_TZ_LOGLEVEL_>3) { TZ_PRINT(x); }
#define TZ_LOGDEBUG1(x,y)      if(_TZ_LOGLEVEL_>3) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINTLN(y); }
#define TZ_HEXLOGDEBUG1(x,y)   if(_TZ_LOGLEVEL_>3) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINTLN(y, HEX); }
#define TZ_LOGDEBUG2(x,y,z)    if(_TZ_LOGLEVEL_>3) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINT(y); TZ_PRINT_SP; TZ_PRINTLN(z); }
#define TZ_LOGDEBUG3(x,y,z,w)  if(_TZ_LOGLEVEL_>3) { TZ_PRINT_MARK; TZ_PRINT(x); TZ_PRINT_SP; TZ_PRINT(y); TZ_PRINT_SP; TZ_PRINT(z); TZ_PRINT_SP; TZ_PRINTLN(w); }

///////////////////////////////////////

#endif    // TIMEZONE_GENERIC_DEBUG_H

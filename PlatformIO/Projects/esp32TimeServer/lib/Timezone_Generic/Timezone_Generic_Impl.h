/*********************************************************************************************************************************
  Timezone_Generic_Impl.h
  
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

#ifndef TIMEZONE_GENERIC_IMPL_H
#define TIMEZONE_GENERIC_IMPL_H

#define  TZ_FILENAME      "/timezone.dat"
#define  TZ_DATA_OFFSET   0

#define TZ_USE_EEPROM      true

/////////////////////////////

// To eliminate warnings with [-Wundef]
#define TZ_USE_ESP32        false
#define TZ_USE_ESP8266      false
#define TZ_USE_SAMD         false
#define TZ_USE_SAM_DUE      false
#define TZ_USE_NRF52        false
#define TZ_USE_STM32        false
#define TZ_USE_RP2040       false
#define TZ_USE_MBED_RP2040  false


/////////////////////////////

#if defined(ESP32)

  #if ( ARDUINO_ESP32C3_DEV )
    // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-gpio.c
    #warning ESP32-C3 boards not fully supported yet. Only SPIFFS and EEPROM OK. Tempo esp32_adc2gpio to be replaced
    const int8_t esp32_adc2gpio[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    #warning Using ESP32-C3
  #endif

  #if ( ARDUINO_ESP32C3_DEV )
    // Currently, ESP32-C3 only supporting SPIFFS and EEPROM. Will fix to support LittleFS
    #ifdef USE_LITTLEFS
      #undef USE_LITTLEFS
    #endif
    #define USE_LITTLEFS          false
  #endif

  #if defined(TZ_USE_ESP32)
    #undef TZ_USE_ESP32
  #endif
  #define TZ_USE_ESP32     true
  
  #if defined(TZ_USE_EEPROM)
    #undef TZ_USE_EEPROM
  #endif
  #define TZ_USE_EEPROM    false
  
  #ifndef USE_LITTLEFS
    #define USE_LITTLEFS   true
  #endif
  
  #if USE_LITTLEFS
    // Use LittleFS
    #include "FS.h"
    
    // Check cores/esp32/esp_arduino_version.h and cores/esp32/core_version.h
    //#if ( ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(2, 0, 0) )  //(ESP_ARDUINO_VERSION_MAJOR >= 2)
    #if ( defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 2) )
      #if (_TZ_LOGLEVEL_ > 2)
        #warning Using ESP32 Core 1.0.6 or 2.0.0+
      #endif
      
      // The library has been merged into esp32 core from release 1.0.6
      #include <LittleFS.h>
      
      #define FileFS        LittleFS
    #else
      #if (_TZ_LOGLEVEL_ > 2)
        #warning Using ESP32 Core 1.0.5-. You must install LITTLEFS library
      #endif
      
      // The library has been merged into esp32 core from release 1.0.6
      #include <LITTLEFS.h>             // https://github.com/lorol/LITTLEFS
      
      #define FileFS        LITTLEFS
    #endif

  #else
    // Use SPIFFS
    #include "FS.h"
    #include <SPIFFS.h>

    #define FileFS        SPIFFS
  #endif

//////////////////////////////////////////////////////////////

#elif defined(ESP8266)
  #if defined(TZ_USE_ESP8266)
    #undef TZ_USE_ESP8266
  #endif
  #define TZ_USE_ESP8266     true
  
  #if defined(TZ_USE_EEPROM)
    #undef TZ_USE_EEPROM
  #endif
  #define TZ_USE_EEPROM    false
  
  #ifndef USE_LITTLEFS
    #define USE_LITTLEFS   true
  #endif
  
  #include <FS.h>
  #include <LittleFS.h>
  
  #if USE_LITTLEFS   
    #define FileFS        LittleFS 
  #else
    #define FileFS        SPIFFS
  #endif
  
#elif ( defined(ARDUINO_SAM_DUE) || defined(__SAM3X8E__) )
  #if defined(TZ_USE_SAM_DUE)
    #undef TZ_USE_SAM_DUE
  #endif
  #define TZ_USE_SAM_DUE     true
  
  #if defined(TZ_USE_EEPROM)
    #undef TZ_USE_EEPROM
  #endif
  #define TZ_USE_EEPROM    false
  
  #warning Use SAM-DUE and DueFlashStorage

//////////////////////////////////////////////////////////////
  
#elif ( defined(ARDUINO_SAMD_ZERO) || defined(ARDUINO_SAMD_MKR1000) || defined(ARDUINO_SAMD_MKRWIFI1010) \
     || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_SAMD_MKRFox1200) || defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310) \
     || defined(ARDUINO_SAMD_MKRGSM1400) || defined(ARDUINO_SAMD_MKRNB1500) || defined(ARDUINO_SAMD_MKRVIDOR4000) \
     || defined(ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS) || defined(__SAMD51__) || defined(__SAMD51J20A__) \
     || defined(__SAMD51J19A__) || defined(__SAMD51G19A__) || defined(__SAMD51P19A__)  \
     || defined(__SAMD21E15A__) || defined(__SAMD21E16A__) || defined(__SAMD21E17A__) || defined(__SAMD21E18A__) \
     || defined(__SAMD21G15A__) || defined(__SAMD21G16A__) || defined(__SAMD21G17A__) || defined(__SAMD21G18A__) \
     || defined(__SAMD21J15A__) || defined(__SAMD21J16A__) || defined(__SAMD21J17A__) || defined(__SAMD21J18A__) )
  #if defined(TZ_USE_SAMD)
    #undef TZ_USE_SAMD
  #endif
  #define TZ_USE_SAMD      true
  
  #if defined(TZ_USE_EEPROM)
    #undef TZ_USE_EEPROM
  #endif
  #define TZ_USE_EEPROM    false
  
  #warning Use SAMD and FlashStorage

//////////////////////////////////////////////////////////////
  
#elif ( defined(NRF52840_FEATHER) || defined(NRF52832_FEATHER) || defined(NRF52_SERIES) || defined(ARDUINO_NRF52_ADAFRUIT) || \
        defined(NRF52840_FEATHER_SENSE) || defined(NRF52840_ITSYBITSY) || defined(NRF52840_CIRCUITPLAY) || defined(NRF52840_CLUE) || \
        defined(NRF52840_METRO) || defined(NRF52840_PCA10056) || defined(PARTICLE_XENON) || defined(NINA_B302_ublox) || defined(NINA_B112_ublox) )   

  #if defined(TZ_USE_NRF52)
    #undef TZ_USE_NRF52
  #endif
  #define TZ_USE_NRF52      true
  
  #if defined(TZ_USE_EEPROM)
    #undef TZ_USE_EEPROM
  #endif
  #define TZ_USE_EEPROM    false
  
  #warning Use NRF52 and LittleFS / InternalFS


//////////////////////////////////////////////////////////////

#elif ( ( defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_PORTENTA_H7_M4) ) && defined(ARDUINO_ARCH_MBED) )

  #if MBED_PORTENTA_H7_INITIALIZED
    #define MBED_PORTENTA_H7_TO_BE_INITIALIZED     false
    #warning MBED_PORTENTA_H7_INITIALIZED in another place
  #else
    // Better to delay until init done
    #if defined(MBED_PORTENTA_H7_INITIALIZED)
      #undef MBED_PORTENTA_H7_INITIALIZED
    #endif
    #define MBED_PORTENTA_H7_INITIALIZED           true
    
    #define MBED_PORTENTA_H7_TO_BE_INITIALIZED     true
    //#warning MBED_PORTENTA_H7_TO_BE_INITIALIZED here
  #endif
    
  #if defined(TZ_USE_MBED_PORTENTA)
    #undef TZ_USE_MBED_PORTENTA
  #endif
  #define TZ_USE_MBED_PORTENTA      true
  
  #if defined(TZ_USE_EEPROM)
    #undef TZ_USE_EEPROM
  #endif
  #define TZ_USE_EEPROM    false
  
  #warning Use MBED PORTENTA_H7 and LittleFS
//////////////////////////////////////////////////////////////

#elif ( defined(STM32F0) || defined(STM32F1) || defined(STM32F2) || defined(STM32F3)  ||defined(STM32F4) || defined(STM32F7) || \
       defined(STM32L0) || defined(STM32L1) || defined(STM32L4) || defined(STM32H7)  ||defined(STM32G0) || defined(STM32G4) || \
       defined(STM32WB) || defined(STM32MP1) )

  #if defined(TZ_USE_STM32)
    #undef TZ_USE_STM32
  #endif
  #define TZ_USE_STM32      true
  
  #if defined(TZ_USE_EEPROM)
    #undef TZ_USE_EEPROM
  #endif
  
  #define TZ_USE_EEPROM    false
  #warning Use STM32 and FlashStorage
  //#define TZ_USE_EEPROM    true
  //#warning Use STM32 and EEPROM

//////////////////////////////////////////////////////////////

#elif ( defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED) )

  // For Earle' arduino-pico core
  #if defined(TZ_USE_RP2040)
    #undef TZ_USE_RP2040
  #endif
  #define TZ_USE_RP2040      true
  
  #if defined(TZ_USE_EEPROM)
    #undef TZ_USE_EEPROM
  #endif 
  #define TZ_USE_EEPROM    false
  
  #warning Use RP2040 (such as RASPBERRY_PI_PICO) and LittleFS

//////////////////////////////////////////////////////////////

#elif ( defined(ARDUINO_ARCH_RP2040) && defined(ARDUINO_ARCH_MBED) )

  // For Arduino' arduino-mbed core
  // To check and determine if we need to init LittleFS here
  #if MBED_RP2040_INITIALIZED
    #define MBED_RP2040_TO_BE_INITIALIZED     false
    #warning MBED_RP2040_INITIALIZED in another place
  #else
    // Better to delay until init done
    #if defined(MBED_RP2040_INITIALIZED)
      #undef MBED_RP2040_INITIALIZED
    #endif
    #define MBED_RP2040_INITIALIZED           true
    
    #define MBED_RP2040_TO_BE_INITIALIZED     true
    //#warning MBED_RP2040_TO_BE_INITIALIZED here
  #endif
  
  #if defined(TZ_USE_MBED_RP2040)
    #undef TZ_USE_MBED_RP2040
  #endif
  #define TZ_USE_MBED_RP2040      true
  
  #if defined(TZ_USE_EEPROM)
    #undef TZ_USE_EEPROM
  #endif
  #define TZ_USE_EEPROM    false
  
  #warning Use MBED RP2040 (such as NANO_RP2040_CONNECT, RASPBERRY_PI_PICO) and LittleFS

//////////////////////////////////////////////////////////////

#elif ( defined(CONFIG_PLATFORM_8721D) || defined(BOARD_RTL8722D) || defined(BOARD_RTL8722DM_MINI) || defined(BOARD_RTL8720DN_BW16) )
  #if defined(TZ_USE_RTL8720)
    #undef TZ_USE_RTL8720
  #endif
  #define TZ_USE_RTL8720      true
  
  #if defined(TZ_USE_EEPROM)
    #undef TZ_USE_EEPROM
  #endif
  #define TZ_USE_EEPROM    false
  
  #warning Use TZ_USE_RTL8720 and FlashStorage_TZ_USE_RTL8720

//////////////////////////////////////////////////////////////
            
#else
  #if defined(CORE_TEENSY)
    #define TZ_USE_EENSY      true
    #warning Use TEENSY and EEPROM
    
  #elif ( defined(ARDUINO_AVR_ADK) || defined(ARDUINO_AVR_BT) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_ESPLORA) \
      || defined(ARDUINO_AVR_ETHERNET) || defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_GEMMA) || defined(ARDUINO_AVR_LEONARDO) \
      || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_LILYPAD_USB) || defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560) \
      || defined(ARDUINO_AVR_MICRO) || defined(ARDUINO_AVR_MINI) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_NG) \
      || defined(ARDUINO_AVR_PRO) || defined(ARDUINO_AVR_ROBOT_CONTROL) || defined(ARDUINO_AVR_ROBOT_MOTOR) || defined(ARDUINO_AVR_UNO) \
      || defined(ARDUINO_AVR_YUN) || defined(__AVR__) )        
    #warning Use AVR and EEPROM
    #define TZ_USE_AVR        true
    //#include <avr/eeprom.h>
    
  #else
    #warning Use Unknown board and EEPROM
  #endif  
#endif

//default to use EEPROM, otherwise, use DueFlashStorage or FlashStorage_SAMD
#if TZ_USE_EEPROM
  #include <EEPROM.h>

  #ifndef TZ_EEPROM_SIZE
    // Please change according to your application to avoid overwriting or being overwritten
    #define TZ_EEPROM_SIZE     (E2END + 1)
  #endif

/////////////////////////////
#elif TZ_USE_SAMD
  // Include EEPROM-like API for FlashStorage
  #include <FlashStorage_SAMD.h>             //https://github.com/khoih-prog/FlashStorage_SAMD
  
#elif TZ_USE_STM32
  // Include EEPROM-like API for FlashStorage
  //#include <FlashStorage_STM32.h>             //https://github.com/khoih-prog/FlashStorage_STM32 
  /**
   Most STM32 devices don't have an integrated EEPROM. To emulate a EEPROM, the STM32 Arduino core emulated
   the operation of an EEPROM with the help of the embedded flash.
   Writing to a flash is very expensive operation, since a whole flash page needs to be written, even if you only
   want to access the flash byte-wise.
   The STM32 Arduino core provides a buffered access API to the emulated EEPROM. The library has allocated the
   buffer even if you don't use the buffered API, so it's strongly suggested to use the buffered API anyhow.
   */
  #if ( defined(STM32F1xx) || defined(STM32F3xx) )
    #include <FlashStorage_STM32F1.h>       // https://github.com/khoih-prog/FlashStorage_STM32F1
    #warning STM32F1/F3 devices have no integrated EEPROM. Using buffered API with FlashStorage_STM32F1 library
  #else
    #include <FlashStorage_STM32.h>       // https://github.com/khoih-prog/FlashStorage_STM32
    #warning STM32 devices have no integrated EEPROM. Using buffered API with FlashStorage_STM32 library
  #endif
  
/////////////////////////////  
#elif TZ_USE_SAM_DUE
  //Use DueFlashStorage to simulate EEPROM
  #include <DueFlashStorage.h>                 //https://github.com/sebnil/DueFlashStorage
  DueFlashStorage dueFlashStorage;

/////////////////////////////  
#elif TZ_USE_NRF52
  // Include LittleFS similar to SPIFFS
  #include <Adafruit_LittleFS.h>
  #include <InternalFileSystem.h>
  using namespace Adafruit_LittleFS_Namespace;
  
  File TZ_file(InternalFS);

/////////////////////////////
#elif TZ_USE_RP2040

  //Use LittleFS for RPI Pico
  #include <FS.h>
  #include <LittleFS.h>

  FS* filesystem =      &LittleFS;
  #define FileFS        LittleFS

/////////////////////////////
#elif (TZ_USE_MBED_RP2040 && MBED_RP2040_TO_BE_INITIALIZED)

  //Use LittleFS for MBED RPI Pico
  #include "FlashIAPBlockDevice.h"
  #include "LittleFileSystem.h"
  #include "mbed.h"

  #include <stdio.h>
  #include <errno.h>
  #include <functional>

  #include "BlockDevice.h"

  #if !defined(RP2040_FLASH_SIZE)
    #define RP2040_FLASH_SIZE         (2 * 1024 * 1024)
  #endif

  #if !defined(RP2040_FS_LOCATION_END)
  #define RP2040_FS_LOCATION_END    RP2040_FLASH_SIZE
  #endif

  #if !defined(RP2040_FS_SIZE_KB)
    // Using default 64KB for LittleFS
    #define RP2040_FS_SIZE_KB       (64)
  #endif

  #if !defined(RP2040_FS_START)
    #define RP2040_FS_START           (RP2040_FLASH_SIZE - (RP2040_FS_SIZE_KB * 1024))
  #endif

  #if !defined(FORCE_REFORMAT)
    #define FORCE_REFORMAT            false
  #endif

  FlashIAPBlockDevice bd(XIP_BASE + RP2040_FS_START, (RP2040_FS_SIZE_KB * 1024));

  mbed::LittleFileSystem fs("fs");
  
  #if defined(TZ_FILENAME)
    #undef TZ_FILENAME
  #endif
  #define  TZ_FILENAME     "/fs/timezone.dat"
  
  #warning MBED_RP2040_TO_BE_INITIALIZED locally in Timezone_Generic

/////////////////////////////

#elif (TZ_USE_MBED_PORTENTA && MBED_PORTENTA_H7_TO_BE_INITIALIZED)

  //Use LittleFS for MBED RPI Pico
  #include "FlashIAPBlockDevice.h"
  #include "LittleFileSystem.h"
  #include "mbed.h"

  #include <stdio.h>
  #include <errno.h>
  #include <functional>

  #include "BlockDevice.h"
  
  #include "mbed_portenta/FlashIAPLimits.h"
  
  #if !defined(FORCE_REFORMAT)
    #define FORCE_REFORMAT            false
  #elif FORCE_REFORMAT
    #warning FORCE_REFORMAT enable. Are you sure ?
  #endif

  static FlashIAPBlockDevice* blockDevicePtr = nullptr;

  mbed::LittleFileSystem fs("littlefs");
  
  struct FlashIAPLimits _flashIAPLimits;
  
  #if defined(TZ_FILENAME)
    #undef TZ_FILENAME
  #endif
  #define  TZ_FILENAME     "/littlefs/timezone.dat"
  
  #warning MBED_PORTENTA_H7_TO_BE_INITIALIZED locally in Timezone_Generic


/////////////////////////////
#elif TZ_USE_RTL8720
  // Include FlashStorage
  #include <FlashStorage_RTL8720.h>             //https://github.com/khoih-prog/FlashStorage_RTL8720
    
/////////////////////////////
  
#endif    //#if TZ_USE_EEPROM
/////////////////////////////

#ifndef TZ_DEBUG
  #define TZ_DEBUG       false
#endif


/*----------------------------------------------------------------------*
   Create a Timezone object from the given time change rules.
  ----------------------------------------------------------------------*/
Timezone::Timezone(const TimeChangeRule& dstStart, const TimeChangeRule& stdStart, uint32_t address)
  : m_dst(dstStart), m_std(stdStart), TZ_DATA_START(address)
{ 
  initStorage(address);
  initTimeChanges();
}

/*----------------------------------------------------------------------*
   Create a Timezone object for a zone that does not observe
   daylight time.
  ----------------------------------------------------------------------*/
Timezone::Timezone(const TimeChangeRule& stdTime, uint32_t address)
  : m_dst(stdTime), m_std(stdTime), TZ_DATA_START(address)
{ 
  initStorage(address);
  initTimeChanges();
}

//////

/*----------------------------------------------------------------------*
   initialise (used where void constructor is called)  6v6gt
  ----------------------------------------------------------------------*/
void Timezone::init(const TimeChangeRule& dstStart, const TimeChangeRule& stdStart)
{
  //m_dst = dstStart;
  //m_std = stdStart;
  memcpy(&m_dst, &dstStart, sizeof(m_dst));
  memcpy(&m_std, &stdStart, sizeof(m_std));
}

//////

// Allow a "blank" TZ object then use begin() method to set the actual TZ.
// Feature added by 6v6gt (https://forum.arduino.cc/index.php?topic=711259)
/*----------------------------------------------------------------------*
   Create a Timezone object from time change rules stored in EEPROM
   at the given address.
  ----------------------------------------------------------------------*/
Timezone::Timezone(uint32_t address)
{
  initStorage(address);
  
  initTimeChanges();

  readRules();
}

void Timezone::initStorage(uint32_t address)
{
  this->TZ_DATA_START = address;
  
#if TZ_USE_EEPROM
  EEPROM.begin();

  TZ_LOGDEBUG3("Read from EEPROM, size = ", TZ_EEPROM_SIZE, ", offset = ", TZ_DATA_START);

/////////////////////////////    
#elif TZ_USE_SAMD
  // Do something to init FlashStorage
  
/////////////////////////////    
#elif TZ_USE_STM32
  // Do something to init FlashStorage  
  

/////////////////////////////    
#elif TZ_USE_SAM_DUE
  // Do something to init DueFlashStorage

/////////////////////////////  
#elif TZ_USE_NRF52
  // Do something to init LittleFS / InternalFS
  // Initialize Internal File System
  InternalFS.begin();
  
/////////////////////////////    
#elif TZ_USE_ESP32
  // Do something to init LittleFS / InternalFS
  // Initialize Internal File System
  if (!FileFS.begin(true))
  {
    TZ_LOGDEBUG("SPIFFS/LittleFS failed! Already tried formatting.");
  
    if (!FileFS.begin())
    {     
      // prevents debug info from the library to hide err message.
      delay(100);
      
      TZ_LOGERROR("LittleFS failed!");
    }
  }
  
/////////////////////////////    
#elif TZ_USE_ESP8266
  // Do something to init LittleFS / InternalFS
  // Initialize Internal File System
  // Do something to init LittleFS / InternalFS
  // Initialize Internal File System
  FileFS.format();
   
  if (!FileFS.begin())
  {
    TZ_LOGDEBUG("SPIFFS/LittleFS failed! Already tried formatting.");
  
    if (!FileFS.begin())
    {     
      // prevents debug info from the library to hide err message.
      delay(100);
      
      TZ_LOGERROR("LittleFS failed!");
    }
  }
  
/////////////////////////////
#elif TZ_USE_RP2040

      bool beginOK = FileFS.begin();
  
      if (!beginOK)
      {
        TZ_LOGERROR("LittleFS error");
      }
  
/////////////////////////////

#elif TZ_USE_MBED_RP2040
 
      TZ_LOGDEBUG1("LittleFS size (KB) = ", RP2040_FS_SIZE_KB);
      
      int err = fs.mount(&bd);
      
      if (err)
      {       
        // Reformat if we can't mount the filesystem
        TZ_LOGERROR("LittleFS Mount Fail. Formatting... ");
 
        err = fs.reformat(&bd);
      }
      else
      {
        TZ_LOGDEBUG("LittleFS Mount OK");
      }
     
      if (err)
      {
        TZ_LOGERROR("LittleFS error");
      }

/////////////////////////////

#elif TZ_USE_MBED_PORTENTA

      if (blockDevicePtr != nullptr)
        return;

      // Get limits of the the internal flash of the microcontroller
      _flashIAPLimits = getFlashIAPLimits();
      
      TZ_LOGDEBUG1("Flash Size: (KB) = ", _flashIAPLimits.flash_size / 1024.0);
      TZ_HEXLOGDEBUG1("FlashIAP Start Address: = 0x", _flashIAPLimits.start_address);
      TZ_LOGDEBUG1("LittleFS size (KB) = ", _flashIAPLimits.available_size / 1024.0);
      
      blockDevicePtr = new FlashIAPBlockDevice(_flashIAPLimits.start_address, _flashIAPLimits.available_size);
      
      if (!blockDevicePtr)
      {    
        TZ_LOGERROR("Error init FlashIAPBlockDevice");

        return;
      }
      
  #if FORCE_REFORMAT
      fs.reformat(blockDevicePtr);
  #endif

      int err = fs.mount(blockDevicePtr);
      
      TZ_LOGDEBUG(err ? "LittleFS Mount Fail" : "LittleFS Mount OK");
  
      if (err)
      {
        // Reformat if we can't mount the filesystem
        TZ_LOGDEBUG("Formatting... ");
  
        err = fs.reformat(blockDevicePtr);
      }
  
      bool beginOK = (err == 0);
  
      if (!beginOK)
      {
        TZ_LOGERROR("\nLittleFS error");
      }
      
/////////////////////////////  
 
#elif TZ_USE_RTL8720
  // Do something to init FlashStorage_RTL8720
    
/////////////////////////////  
#else
  #error Un-identifiable board selected. Please check your Tools->Board setting.
#endif
/////////////////////////////  

  storageSystemInit = true;
}

/*----------------------------------------------------------------------*
   Convert the given UTC time to local time, standard or
   daylight time, as appropriate.
  ----------------------------------------------------------------------*/
time_t Timezone::toLocal(const time_t& utc)
{
  // recalculate the time change points if needed
  if (year(utc) != year(m_dstUTC)) 
    calcTimeChanges(year(utc));

  if (utcIsDST(utc))
    return utc + m_dst.offset * SECS_PER_MIN;
  else
    return utc + m_std.offset * SECS_PER_MIN;
}

/*----------------------------------------------------------------------*
   Convert the given UTC time to local time, standard or
   daylight time, as appropriate, and return a pointer to the time
   change rule used to do the conversion. The caller must take care
   not to alter this rule.
  ----------------------------------------------------------------------*/
time_t Timezone::toLocal(const time_t& utc, TimeChangeRule **tcr)
{
  // recalculate the time change points if needed
  if (year(utc) != year(m_dstUTC)) 
    calcTimeChanges(year(utc));

  if (utcIsDST(utc)) 
  {
    *tcr = &m_dst;
    return utc + m_dst.offset * SECS_PER_MIN;
  }
  else 
  {
    *tcr = &m_std;
    return utc + m_std.offset * SECS_PER_MIN;
  }
}

/*----------------------------------------------------------------------*
   Convert the given local time to UTC time.
 *                                                                      *
   WARNING:
   This function is provided for completeness, but should seldom be
   needed and should be used sparingly and carefully.
 *                                                                      *
   Ambiguous situations occur after the Standard-to-DST and the
   DST-to-Standard time transitions. When changing to DST, there is
   one hour of local time that does not exist, since the clock moves
   forward one hour. Similarly, when changing to standard time, there
   is one hour of local times that occur twice since the clock moves
   back one hour.
 *                                                                      *
   This function does not test whether it is passed an erroneous time
   value during the Local -> DST transition that does not exist.
   If passed such a time, an incorrect UTC time value will be returned.
 *                                                                      *
   If passed a local time value during the DST -> Local transition
   that occurs twice, it will be treated as the earlier time, i.e.
   the time that occurs before the transistion.
 *                                                                      *
   Calling this function with local times during a transition interval
   should be avoided!
  ----------------------------------------------------------------------*/
time_t Timezone::toUTC(const time_t& local)
{
  // recalculate the time change points if needed
  if (year(local) != year(m_dstLoc)) 
    calcTimeChanges(year(local));

  if (locIsDST(local))
    return local - m_dst.offset * SECS_PER_MIN;
  else
    return local - m_std.offset * SECS_PER_MIN;
}

/*----------------------------------------------------------------------*
   Determine whether the given UTC time_t is within the DST interval
   or the Standard time interval.
  ----------------------------------------------------------------------*/
bool Timezone::utcIsDST(const time_t& utc)
{
  // recalculate the time change points if needed
  if (year(utc) != year(m_dstUTC)) 
    calcTimeChanges(year(utc));

  if (m_stdUTC == m_dstUTC)       // daylight time not observed in this tz
    return false;
  else if (m_stdUTC > m_dstUTC)   // northern hemisphere
    return (utc >= m_dstUTC && utc < m_stdUTC);
  else                            // southern hemisphere
    return !(utc >= m_stdUTC && utc < m_dstUTC);
}

/*----------------------------------------------------------------------*
   Determine whether the given Local time_t is within the DST interval
   or the Standard time interval.
  ----------------------------------------------------------------------*/
bool Timezone::locIsDST(const time_t& local)
{
  // recalculate the time change points if needed
  if (year(local) != year(m_dstLoc)) 
    calcTimeChanges(year(local));

  if (m_stdUTC == m_dstUTC)       // daylight time not observed in this tz
    return false;
  else if (m_stdLoc > m_dstLoc)   // northern hemisphere
    return (local >= m_dstLoc && local < m_stdLoc);
  else                            // southern hemisphere
    return !(local >= m_stdLoc && local < m_dstLoc);
}

/*----------------------------------------------------------------------*
   Calculate the DST and standard time change points for the given
   given year as local and UTC time_t values.
  ----------------------------------------------------------------------*/
void Timezone::calcTimeChanges(int yr)
{
  m_dstLoc = toTime_t(m_dst, yr);
  m_stdLoc = toTime_t(m_std, yr);
  m_dstUTC = m_dstLoc - m_std.offset * SECS_PER_MIN;
  m_stdUTC = m_stdLoc - m_dst.offset * SECS_PER_MIN;
}

/*----------------------------------------------------------------------*
   Initialize the DST and standard time change points.
  ----------------------------------------------------------------------*/
void Timezone::initTimeChanges()
{
  m_dstLoc = 0;
  m_stdLoc = 0;
  m_dstUTC = 0;
  m_stdUTC = 0;
}

/*----------------------------------------------------------------------*
   Convert the given time change rule to a time_t value
   for the given year.
  ----------------------------------------------------------------------*/
time_t Timezone::toTime_t(const TimeChangeRule& r, int yr)
{
  uint8_t m = r.month;     // temp copies of r.month and r.week
  uint8_t w = r.week;
  
  if (w == 0)              // is this a "Last week" rule?
  {
    if (++m > 12)        // yes, for "Last", go to the next month
    {
      m = 1;
      ++yr;
    }
    
    w = 1;               // and treat as first week of next month, subtract 7 days later
  }

  // calculate first day of the month, or for "Last" rules, first day of the next month
  tmElements_t tm;
  
  tm.Hour   = r.hour;
  tm.Minute = 0;
  tm.Second = 0;
  tm.Day    = 1;
  tm.Month  = m;
  tm.Year   = yr - 1970;
  time_t t  = makeTime(tm);

  // add offset from the first of the month to r.dow, and offset for the given week
  t += ( (r.dow - weekday(t) + 7) % 7 + (w - 1) * 7 ) * SECS_PER_DAY;
  
  // back up a week if this is a "Last" rule
  if (r.week == 0) 
    t -= 7 * SECS_PER_DAY;
    
  return t;
}

/*----------------------------------------------------------------------*
   Read or update the daylight and standard time rules from RAM.
  ----------------------------------------------------------------------*/
void Timezone::setRules(const TimeChangeRule& dstStart, const TimeChangeRule& stdStart)
{
  m_dst = dstStart;
  m_std = stdStart;
  initTimeChanges();  // force calcTimeChanges() at next conversion call
}

void Timezone::display_DST_Rule()
{
  TZ_LOGERROR("DST rule");
  TZ_LOGERROR3("abbrev :",  m_dst.abbrev, ", week :",   m_dst.week);
  TZ_LOGERROR3("dow :",     m_dst.dow,    ", month :",  m_dst.month);
  TZ_LOGERROR3("hour :",    m_dst.hour,   ", offset :", m_dst.offset);
}

void Timezone::display_STD_Rule()
{
  TZ_LOGERROR("DST rule");
  TZ_LOGERROR3("abbrev :",  m_std.abbrev, ", week :",   m_std.week);
  TZ_LOGERROR3("dow :",     m_std.dow,    ", month :",  m_std.month);
  TZ_LOGERROR3("hour :",    m_std.hour,   ", offset :", m_std.offset);
}


/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from EEPROM/storage at
   the TZ_DATA_START offset.
  ----------------------------------------------------------------------*/
void Timezone::readRules()
{
  readTZData();
  initTimeChanges();  // force calcTimeChanges() at next conversion call
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to EEPROM/storage at
   the TZ_DATA_START offset.
  ----------------------------------------------------------------------*/
void Timezone::writeRules(int address)
{
  this->TZ_DATA_START = address;
  
  writeTZData(address);
  initTimeChanges();  // force calcTimeChanges() at next conversion call
}

/////////////////////////////////////////////

#if (TZ_USE_ESP32)

#if USE_LITTLEFS
  #warning Using ESP32 LittleFS in Timezone_Generic
#else
  #warning Using ESP32 SPIFFS in Timezone_Generic
#endif
  
  // ESP32 code    
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{
  if (!storageSystemInit)
  {
    // Format SPIFFS/LittleFS if not yet
    if (!FileFS.begin(true))
    {
      TZ_LOGERROR(F("SPIFFS/LittleFS failed! Formatting."));
      
      if (!FileFS.begin())
      {
        TZ_LOGERROR(F("SPIFFS/LittleFS failed! Pls use EEPROM."));
        return;
      }
    }
    
    storageSystemInit = true;
  }
  
  // ESP32 code
  File file = FileFS.open(TZ_FILENAME, "r");
  
  TZ_LOGDEBUG3(F("Reading m_dst & m_std from TZ_file :"), TZ_FILENAME, F(", data offset ="), TZ_DATA_OFFSET);

  if (file)
  {
    memset(&m_dst, 0, TZ_DATA_SIZE);
    memset(&m_std, 0, TZ_DATA_SIZE);
    
    file.seek(TZ_DATA_OFFSET);
    
    file.readBytes((char *) &m_dst, TZ_DATA_SIZE);
    
    // Seek to be sure
    file.seek(TZ_DATA_OFFSET + TZ_DATA_SIZE);
    file.readBytes((char *) &m_std, TZ_DATA_SIZE);

    TZ_LOGDEBUG(F("Reading from TZ_file OK"));

    file.close(); 
  }
  else
  {
    TZ_LOGERROR(F("Reading from TZ_file failed"));
  }
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{ 
  (void) address;
  
  if (!storageSystemInit)
  {
    // Format SPIFFS/LittleFS if not yet
    if (!FileFS.begin(true))
    {
      TZ_LOGERROR(F("SPIFFS/LittleFS failed! Formatting."));
      
      if (!FileFS.begin())
      {
        TZ_LOGERROR(F("SPIFFS/LittleFS failed!"));
        return;
      }
    }
    
    storageSystemInit = true;
  }
  
  // ESP32 code
  File file = FileFS.open(TZ_FILENAME, "w");

  TZ_LOGDEBUG3(F("Saving m_dst & m_std to TZ_file :"), TZ_FILENAME, F(", data offset ="), TZ_DATA_OFFSET);

  if (file)
  {
    file.seek(TZ_DATA_OFFSET);
    file.write((uint8_t *) &m_dst, TZ_DATA_SIZE);

    // Seek to be sure
    file.seek(TZ_DATA_OFFSET + TZ_DATA_SIZE);
    file.write((uint8_t *) &m_std, TZ_DATA_SIZE);
    
    file.close();

    TZ_LOGDEBUG("Saving to TZ_file OK");
  }
  else
  {
    TZ_LOGERROR("Saving to TZ_file failed");
  }   
}

/////////////////////////////////////////////

#elif (TZ_USE_ESP8266)

#if USE_LITTLEFS
  #warning Using ESP8266 LittleFS in Timezone_Generic
#else
  #warning Using ESP8266 SPIFFS in Timezone_Generic
#endif

  // ESP8266 code    
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{
  if (!storageSystemInit)
  {
    // Format SPIFFS/LittleFS if not yet
    if (!FileFS.begin())
    {
      TZ_LOGERROR(F("SPIFFS/LittleFS failed! Formatting."));
      FileFS.format();
      
      if (!FileFS.begin())
      {
        TZ_LOGERROR(F("SPIFFS/LittleFS failed"));
        return;
      }
    }
    
    storageSystemInit = true;
  }
  
  // ESP8266 code
  File file = FileFS.open(TZ_FILENAME, "r");
  
  TZ_LOGDEBUG3(F("Reading m_dst & m_std from TZ_file :"), TZ_FILENAME, F(", data offset ="), TZ_DATA_OFFSET);

  if (file)
  {
    memset(&m_dst, 0, TZ_DATA_SIZE);
    memset(&m_std, 0, TZ_DATA_SIZE);
    
    file.seek(TZ_DATA_OFFSET);
    
    file.readBytes((char *) &m_dst, TZ_DATA_SIZE);
    
    // Seek to be sure
    file.seek(TZ_DATA_OFFSET + TZ_DATA_SIZE);
    file.readBytes((char *) &m_std, TZ_DATA_SIZE);

    TZ_LOGDEBUG(F("Reading from TZ_file OK"));

    file.close(); 
  }
  else
  {
    TZ_LOGERROR(F("Reading from TZ_file failed"));
  }
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{ 
  (void) address;
  
  if (!storageSystemInit)
  {
    // Format SPIFFS/LittleFS if not yet
    if (!FileFS.begin())
    {
      TZ_LOGERROR(F("SPIFFS/LittleFS failed! Formatting."));
      FileFS.format();
      
      if (!FileFS.begin())
      {
        TZ_LOGERROR(F("SPIFFS/LittleFS failed! Pls use EEPROM."));
        return;
      }
    }
    
    storageSystemInit = true;
  }
  
  // ESP8266 code
  File file = FileFS.open(TZ_FILENAME, "w");

  TZ_LOGDEBUG3(F("Saving m_dst & m_std to TZ_file :"), TZ_FILENAME, F(", data offset ="), TZ_DATA_OFFSET);

  if (file)
  {
    file.seek(TZ_DATA_OFFSET);
    file.write((uint8_t *) &m_dst, TZ_DATA_SIZE);

    // Seek to be sure
    file.seek(TZ_DATA_OFFSET + TZ_DATA_SIZE);
    file.write((uint8_t *) &m_std, TZ_DATA_SIZE);
    
    file.close();

    TZ_LOGDEBUG("Saving to TZ_file OK");
  }
  else
  {
    TZ_LOGERROR("Saving to TZ_file failed");
  }   
}

/////////////////////////////////////////////

#elif (TZ_USE_SAMD)

  #warning Using SAMD FlashStorage in Timezone_Generic
  
  // SAMD code  
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from EEPROM at
   the given address.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  // It's too bad that emulate EEPROM.read()/write() can only deal with bytes. 
  // Have to read/write each byte. To rewrite the library
  
  memset(&m_dst, 0, TZ_DATA_SIZE);
  memset(&m_std, 0, TZ_DATA_SIZE);
  
  uint16_t offset   = TZ_DATA_START;               
  uint8_t* _pointer = (uint8_t *) &m_dst;

  for (uint16_t i = 0; i < TZ_DATA_SIZE; i++, _pointer++, offset++)
  {              
    *_pointer = EEPROM.read(offset);
  }
            
  _pointer = (uint8_t *) &m_std;

  for (uint16_t i = 0; i < TZ_DATA_SIZE; i++, _pointer++, offset++)
  {              
    *_pointer = EEPROM.read(offset);
  }

  return;  
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to EEPROM at
   the given address.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  // It's too bad that emulate EEPROM.read()/write() can only deal with bytes. 
  // Have to read/write each byte. To rewrite the library
  
  uint16_t offset   = address;               
  uint8_t* _pointer = (uint8_t *) &m_dst;

  for (uint16_t i = 0; i < TZ_DATA_SIZE; i++, _pointer++, offset++)
  {              
    EEPROM.write(offset, *_pointer);
  }
            
  _pointer = (uint8_t *) &m_std;

  for (uint16_t i = 0; i < TZ_DATA_SIZE; i++, _pointer++, offset++)
  {              
    EEPROM.write(offset, *_pointer);
  }

  return;  
}


/////////////////////////////////////////////

#elif (TZ_USE_STM32)

  #warning Using STM32 FlashStorage in Timezone_Generic
  
  // SAMD code  
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from EEPROM at
   the given address.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  // It's too bad that emulate EEPROM.read()/write() can only deal with bytes. 
  // Have to read/write each byte. To rewrite the library
  
  memset(&m_dst, 0, TZ_DATA_SIZE);
  memset(&m_std, 0, TZ_DATA_SIZE);
  
  uint16_t offset   = TZ_DATA_START;               
  uint8_t* _pointer = (uint8_t *) &m_dst;

  for (uint16_t i = 0; i < TZ_DATA_SIZE; i++, _pointer++, offset++)
  {              
    *_pointer = EEPROM.read(offset);
  }
            
  _pointer = (uint8_t *) &m_std;

  for (uint16_t i = 0; i < TZ_DATA_SIZE; i++, _pointer++, offset++)
  {              
    *_pointer = EEPROM.read(offset);
  }

  return;  
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to EEPROM at
   the given address.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  // It's too bad that emulate EEPROM.read()/write() can only deal with bytes. 
  // Have to read/write each byte. To rewrite the library
  
  uint16_t offset   = address;               
  uint8_t* _pointer = (uint8_t *) &m_dst;

  for (uint16_t i = 0; i < TZ_DATA_SIZE; i++, _pointer++, offset++)
  {              
    EEPROM.write(offset, *_pointer);
  }
            
  _pointer = (uint8_t *) &m_std;

  for (uint16_t i = 0; i < TZ_DATA_SIZE; i++, _pointer++, offset++)
  {              
    EEPROM.write(offset, *_pointer);
  }

  return;  
}

/////////////////////////////////////////////
         
#elif (TZ_USE_SAM_DUE)

  #warning Using SAM DUE dueFlashStorage in Timezone_Generic
  // SAM DUE code    
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from dueFlashStorage at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  memset(&m_dst, 0, TZ_DATA_SIZE);
  memset(&m_std, 0, TZ_DATA_SIZE);
  
  // SAM DUE code
  byte* dataPointer = (byte* ) dueFlashStorage.readAddress(TZ_DATA_START);

  memcpy(&m_dst, dataPointer, TZ_DATA_SIZE); 
  
  dataPointer += TZ_DATA_SIZE;
  
  memcpy(&m_std, dataPointer, TZ_DATA_SIZE); 
  
  TZ_LOGDEBUG("Reading from dueFlashStorage OK");
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to dueFlashStorage at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{ 
  (void) address;
  
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  // SAM DUE code     
  dueFlashStorage.write(TZ_DATA_START, (byte *) &m_dst, TZ_DATA_SIZE);
  dueFlashStorage.write(TZ_DATA_START + TZ_DATA_SIZE, (byte *) &m_std, TZ_DATA_SIZE);
  
  TZ_LOGDEBUG("Writing to dueFlashStorage OK");
}

/////////////////////////////////////////////
      
#elif (TZ_USE_NRF52)

  #warning Using NRF52 LittleFS in Timezone_Generic
  
  // nRF52 code    
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{
  if (!storageSystemInit)
  {
    InternalFS.begin();
    storageSystemInit = true;
  }
  
  // nRF52 code
  TZ_file.open(TZ_FILENAME, FILE_O_READ);
  
  TZ_LOGDEBUG3("Reading m_dst & m_std from TZ_file :", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);

  if (TZ_file)
  {
    memset(&m_dst, 0, TZ_DATA_SIZE);
    memset(&m_std, 0, TZ_DATA_SIZE);
    
    TZ_file.seek(TZ_DATA_OFFSET);
    
    TZ_file.read((char *) &m_dst, TZ_DATA_SIZE);
    // Seek to be sure
    TZ_file.seek(TZ_DATA_OFFSET + TZ_DATA_SIZE);
    TZ_file.read((char *) &m_std, TZ_DATA_SIZE);

    TZ_file.close();
    
    TZ_LOGDEBUG("Reading from TZ_file OK");
  }
  else
  {
    TZ_LOGERROR("Reading from TZ_file failed");
  }
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{ 
  (void) address;
  
  if (!storageSystemInit)
  {
    InternalFS.begin();
    storageSystemInit = true;
  }
  
  // nRF52 code
  TZ_file.open(TZ_FILENAME, FILE_O_WRITE);

  TZ_LOGDEBUG3("Saving m_dst & m_std to TZ_file :", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);

  if (TZ_file)
  {
    TZ_file.seek(TZ_DATA_OFFSET);
    TZ_file.write((uint8_t *) &m_dst, TZ_DATA_SIZE);
    // Seek to be sure
    TZ_file.seek(TZ_DATA_OFFSET + TZ_DATA_SIZE);
    TZ_file.write((uint8_t *) &m_std, TZ_DATA_SIZE);
    
    TZ_file.close();

    TZ_LOGDEBUG("Saving to TZ_file OK");
  }
  else
  {
    TZ_LOGERROR("Saving to TZ_file failed");
  }   
}

/////////////////////////////////////////////

#elif (TZ_USE_STM32)

  #if 1
  
  #if defined(DATA_EEPROM_BASE)
    // For STM32 devices having integrated EEPROM.
    #include <EEPROM.h>
    #warning STM32 devices have integrated EEPROM. Not using buffered API.   
  #else
    /**
     Most STM32 devices don't have an integrated EEPROM. To emulate a EEPROM, the STM32 Arduino core emulated
     the operation of an EEPROM with the help of the embedded flash.
     Writing to a flash is very expensive operation, since a whole flash page needs to be written, even if you only
     want to access the flash byte-wise.
     The STM32 Arduino core provides a buffered access API to the emulated EEPROM. The library has allocated the
     buffer even if you don't use the buffered API, so it's strongly suggested to use the buffered API anyhow.
     */
    #if ( defined(STM32F1xx) || defined(STM32F3xx) )
      #include <FlashStorage_STM32F1.h>       // https://github.com/khoih-prog/FlashStorage_STM32F1
      #warning STM32F1/F3 devices have no integrated EEPROM. Using buffered API with FlashStorage_STM32F1 library
    #else
      #include <FlashStorage_STM32.h>       // https://github.com/khoih-prog/FlashStorage_STM32
      #warning STM32 devices have no integrated EEPROM. Using buffered API with FlashStorage_STM32 library
    #endif
  #endif    // #if defined(DATA_EEPROM_BASE)  
  
  #else
  #warning Using STM32 EEPROM in Timezone_Generic
  #endif

  // STM32 code    
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from EEPROM at
   the given address.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{ 
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  TZ_LOGDEBUG3("Read from EEPROM, size = ", TZ_EEPROM_SIZE, ", offset = ", TZ_DATA_START);
  
  memset(&m_dst, 0, TZ_DATA_SIZE);
  memset(&m_std, 0, TZ_DATA_SIZE);
  
  EEPROM.get(TZ_DATA_START, m_dst);
  EEPROM.get(TZ_DATA_START + TZ_DATA_SIZE, m_std);
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to EEPROM at
   the given address.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{
  (void) address;
  
  if (!storageSystemInit)
  {   
    EEPROM.begin();
    storageSystemInit = true;
  }
  
  TZ_LOGERROR3("Write to EEPROM, size = ", TZ_EEPROM_SIZE, ", offset = ", TZ_DATA_START);
   
  EEPROM.put(TZ_DATA_START, m_dst);
  EEPROM.put(TZ_DATA_START + TZ_DATA_SIZE, m_std);
}

/////////////////////////////////////////////
#elif (TZ_USE_RP2040)

  #warning Using RP2040 LittleFS in Timezone_Generic
  
  // RP2040 code    
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  // RP2040 code
  File file = FileFS.open(TZ_FILENAME, "r");
  
  TZ_LOGDEBUG3("Reading m_dst & m_std from TZ_file :", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);

  if (file)
  {
    memset(&m_dst, 0, TZ_DATA_SIZE);
    memset(&m_std, 0, TZ_DATA_SIZE);
       
    file.seek(TZ_DATA_OFFSET);
    file.read((uint8_t *) &m_dst, TZ_DATA_SIZE);
        
    // Seek to be sure   
    file.seek(TZ_DATA_OFFSET + TZ_DATA_SIZE);
    file.read((uint8_t *) &m_std, TZ_DATA_SIZE);

    TZ_LOGDEBUG("Reading from TZ_file OK");

    file.close(); 
  }
  else
  {
    TZ_LOGERROR("Reading from TZ_file failed");
  }
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{ 
  (void) address;
  
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  // RP2040 code
  File file = FileFS.open(TZ_FILENAME, "w");

  TZ_LOGDEBUG3("Saving m_dst & m_std to TZ_file :", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);

  if (file)
  {
    file.seek(TZ_DATA_OFFSET);
    file.write((uint8_t *) &m_dst, TZ_DATA_SIZE);
        
    // Seek to be sure
    file.seek(TZ_DATA_OFFSET + TZ_DATA_SIZE);
    file.write((uint8_t *) &m_std, TZ_DATA_SIZE);
    
    file.close();

    TZ_LOGDEBUG("Saving to TZ_file OK");
  }
  else
  {
    TZ_LOGERROR("Saving to TZ_file failed");
  }   
}

/////////////////////////////////////////////

#elif (TZ_USE_MBED_RP2040)

  #warning Using MBED RP2040 LittleFS in Timezone_Generic
  
  // MBED RP2040 code  
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{
  TZ_LOGDEBUG3("Start readTZData from ", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);
  
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  // MBED RP2040 code
  FILE *file = fopen(TZ_FILENAME, "r");
  
  TZ_LOGDEBUG3("Reading m_dst & m_std from TZ_file :", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);

  if (file)
  {
    memset(&m_dst, 0, TZ_DATA_SIZE);
    memset(&m_std, 0, TZ_DATA_SIZE);
    
    fseek(file, TZ_DATA_OFFSET, SEEK_SET);
    fread((uint8_t *) &m_dst, 1, TZ_DATA_SIZE, file);
        
    // Seek to be sure
    fseek(file, TZ_DATA_OFFSET + TZ_DATA_SIZE, SEEK_SET);
    fread((uint8_t *) &m_std, 1, TZ_DATA_SIZE, file);

    TZ_LOGDEBUG("Reading from TZ_file OK");

    fclose(file);
  }
  else
  {
    TZ_LOGERROR("Reading from TZ_file failed");
  }
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{ 

  (void) address;
  
  TZ_LOGDEBUG3("Start writeTZData to ", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);
  
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  // MBED RP2040 code
  FILE *file = fopen(TZ_FILENAME, "w");

  TZ_LOGDEBUG3("Saving m_dst & m_std to TZ_file :", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);

  if (file)
  {   
    fseek(file, TZ_DATA_OFFSET, SEEK_SET);
    fwrite((uint8_t *) &m_dst, 1, sizeof(m_dst) /*TZ_DATA_SIZE*/, file);
    
    // Seek to be sure
    fseek(file, TZ_DATA_OFFSET + TZ_DATA_SIZE, SEEK_SET);
    fwrite((uint8_t *) &m_std, 1, sizeof(m_std) /*TZ_DATA_SIZE*/, file);
    
    fclose(file);

    TZ_LOGDEBUG("Saving to TZ_file OK");
  }
  else
  {
    TZ_LOGERROR("Saving to TZ_file failed");
  }
}

/////////////////////////////////////////////

#elif (TZ_USE_MBED_PORTENTA)

  #warning Using MBED PORTENTA LittleFS in Timezone_Generic
  
  // MBED PORTENTA code  
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{
  TZ_LOGDEBUG3("Start readTZData from ", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);
   
  // MBED PORTENTA code
  FILE *file = fopen(TZ_FILENAME, "r");
  
  TZ_LOGDEBUG3("Reading m_dst & m_std from TZ_file :", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);

  if (file)
  {
    memset(&m_dst, 0, TZ_DATA_SIZE);
    memset(&m_std, 0, TZ_DATA_SIZE);
    
    fseek(file, TZ_DATA_OFFSET, SEEK_SET);
    fread((uint8_t *) &m_dst, 1, TZ_DATA_SIZE, file);
        
    // Seek to be sure
    fseek(file, TZ_DATA_OFFSET + TZ_DATA_SIZE, SEEK_SET);
    fread((uint8_t *) &m_std, 1, TZ_DATA_SIZE, file);

    TZ_LOGDEBUG("Reading from TZ_file OK");

    fclose(file);
  }
  else
  {
    TZ_LOGERROR("Reading from TZ_file failed");
    
    //fs.reformat(blockDevicePtr);
  }
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to LittleFS at
   the given offset.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{ 

  (void) address;
  
  TZ_LOGDEBUG3("Start writeTZData to ", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);
  
 
  // MBED PORTENTA code
  FILE *file = fopen(TZ_FILENAME, "w");

  TZ_LOGDEBUG3("Saving m_dst & m_std to TZ_file :", TZ_FILENAME, ", data offset =", TZ_DATA_OFFSET);

  if (file)
  {   
    fseek(file, TZ_DATA_OFFSET, SEEK_SET);
    fwrite((uint8_t *) &m_dst, 1, sizeof(m_dst) /*TZ_DATA_SIZE*/, file);
    
    // Seek to be sure
    fseek(file, TZ_DATA_OFFSET + TZ_DATA_SIZE, SEEK_SET);
    fwrite((uint8_t *) &m_std, 1, sizeof(m_std) /*TZ_DATA_SIZE*/, file);
    
    fclose(file);

    TZ_LOGDEBUG("Saving to TZ_file OK");
  }
  else
  {
    TZ_LOGERROR("Saving to TZ_file failed");
    
    //fs.reformat(blockDevicePtr);
  }
}

/////////////////////////////////////////////

#elif (TZ_USE_RTL8720)

  #warning Using RTL8720 FlashStorage in Timezone_Generic

  // RTL8720 code    
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from FlashStorage at
   the given address.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{ 
  if (!storageSystemInit)
  {
    storageSystemInit = true;
  }
  
  TZ_LOGDEBUG3("Read from FlashStorage, size = ", FlashStorage.length(), ", offset = ", TZ_DATA_START);
  
  memset(&m_dst, 0, TZ_DATA_SIZE);
  memset(&m_std, 0, TZ_DATA_SIZE);
  
  FlashStorage.get(TZ_DATA_START, m_dst);
  FlashStorage.get(TZ_DATA_START + TZ_DATA_SIZE, m_std);
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to EEPROM at
   the given address.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{
  (void) address;
  
  if (!storageSystemInit)
  {   
    storageSystemInit = true;
  }
  
  TZ_LOGDEBUG3("Read from FlashStorage, size = ", FlashStorage.length(), ", offset = ", TZ_DATA_START);
   
  FlashStorage.setCommitASAP(false); 
  FlashStorage.put(TZ_DATA_START, m_dst);
  FlashStorage.put(TZ_DATA_START + TZ_DATA_SIZE, m_std);
  FlashStorage.commit();
  FlashStorage.setCommitASAP(true); 
}

        
/////////////////////////////////////////////        
#elif TZ_USE_EEPROM
/*----------------------------------------------------------------------*
   Read the daylight and standard time rules from EEPROM at
   the given address.
  ----------------------------------------------------------------------*/
void Timezone::readTZData()
{ 
  if (!storageSystemInit)
  {
    EEPROM.begin();
    storageSystemInit = true;
  }
  
  TZ_LOGERROR3("Read from EEPROM, size = ", TZ_EEPROM_SIZE, ", offset = ", TZ_DATA_START);
  
  memset(&m_dst, 0, TZ_DATA_SIZE);
  memset(&m_std, 0, TZ_DATA_SIZE);
    
  EEPROM.get(TZ_DATA_START, m_dst);
  EEPROM.get(TZ_DATA_START + TZ_DATA_SIZE, m_std);
}

/*----------------------------------------------------------------------*
   Write the daylight and standard time rules to EEPROM at
   the given address.
  ----------------------------------------------------------------------*/
void Timezone::writeTZData(int address)
{
  (void) address;
  
  if (!storageSystemInit)
  {
    EEPROM.begin();
    storageSystemInit = true;
  }
  
  TZ_LOGERROR3("Write to EEPROM, size = ", TZ_EEPROM_SIZE, ", offset = ", TZ_DATA_START);
   
  EEPROM.put(TZ_DATA_START, m_dst);
  EEPROM.put(TZ_DATA_START + TZ_DATA_SIZE, m_std);
}
        
#endif

#endif    // TIMEZONE_GENERIC_IMPL_H

// Rob Latour, 2023
// License: MIT
// https://github.com/roblatour
//
// This program's key setting can be viewed/updated in the file:
//      ESP32TimeServerKeySettings.h
//
// https://time.is may be used a reference point to confirm your computer's date/time accuracy
//

// board: Olimex ESP32 POE ISO

#include <ETH.h>
#include <Timezone.h> // https://github.com/khoih-prog/Timezone_Generic
#include <ESP32Time.h>
#include <SoftwareSerial.h>
#include <SparkFun_u-blox_GNSS_v3.h> // https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3
#include <LiquidCrystal_I2C.h>       // https://github.com/johnrickman/LiquidCrystal_I2C
#include "ESP32TimeServerKeySettings.h"

// ESP32Time real time clock
ESP32Time rtc(0);

// Ethernet
bool eth_connected = false;
bool eth_got_IP = false;
String ip = "";

// GPS
SFE_UBLOX_GNSS_SERIAL gps;
SoftwareSerial GPSDevice(RXPin, TXPin); 
volatile bool ppsFlag;  // GPS one-pulse-per-second flag

// LCD Display
LiquidCrystal_I2C lcd(lcdI2CAddress, lcdColumns, lcdRows);
String fullyBlankLine = "";

// TimeZone
TimeChangeRule *tcr;

// NTP port and packet buffer
#define NTP_PORT 123
#define NTP_PACKET_SIZE 48
byte packetBuffer[NTP_PACKET_SIZE];
WiFiUDP Udp;

// Constants and global variables
const unsigned long oneSecond_inMilliseconds = 1000;                              // one second in milliseconds
const unsigned long oneMinute_inMilliseconds = 60 * oneSecond_inMilliseconds;     // one minute in milliseconds
const unsigned long thirtyMinutes_inMilliseconds = 30 * oneMinute_inMilliseconds; // 30 minutes in milliseconds
const long oneSecond_inMicroseconds_L = 1000000;                                  // one second in microseconds (signed long)
const double oneSecond_inMicroseconds_D = 1000000.0;                              // one second in microseconds (double)
                                                                                  //
const unsigned long periodicTimeRefreshPeriod = thirtyMinutes_inMilliseconds;     // how often the system's real time clock is refreshed with GPS data
const time_t safeguardThresholdInSeconds = 1;                                     // used to ensure a GPS time refresh is only performed if the difference between the old and new times is this many seconds or less
volatile bool SafeGuardTripped = false;                                           // used to ensure the time isn't changed beyond that which would reasonably be expected within the periodicTimeRefreshPeriod
                                                                                  //
volatile bool theTimeSettingProcessIsUnderway;                                    // signifies when the time is being set / refreshed
                                                                                  //
SemaphoreHandle_t mutex;                                                          // used to ensure an NTP request results are not impacted by the process that refreshes the time
                                                                                  //
TaskHandle_t taskHandle0 = NULL;                                                  // task handle for updating the display
TaskHandle_t taskHandle1 = NULL;                                                  // task handle for setting/refreshing the time 

                                               
//**************************************************************************************************************************

void setupSerial()
{

  if (debugIsOn)
  {

    Serial.begin(SerialMonitorSpeed);

    for (int i = 0; i < 10; i++)
      Serial.println("");
  };
}

void turnOffWifiAndBluetooth()
{

  // wifi and bluetooth aren't needed so turn them off
  WiFi.mode(WIFI_OFF);
  btStop();
}

void setupButton()
{

  pinMode(UpTimePin, INPUT_PULLUP);
};

void display(uint8_t row, String msg, bool writeToSerialMonitor = true)
{

  String displayLine = msg + fullyBlankLine; // adding the fully blank line here clears the remnants of previously displayed information
  displayLine = displayLine.substring(0, 20);

  lcd.setCursor(0, row);
  lcd.print(displayLine);

  if (debugIsOn && writeToSerialMonitor)
    Serial.println(displayLine);
}

void setupDisplay()
{

  lcd.begin(lcdColumns, lcdRows);
  lcd.init();
  lcd.backlight();
  lcd.noAutoscroll();
  lcd.noCursor();
  lcd.clear();
  lcd.home();

  for (int i = 0; i < lcdColumns; i++)
    fullyBlankLine.concat(" ");

  display(0, "ESP32 Time Server", false);
}

void GetAdjustedDateAndTimeStrings(time_t UTC_Time, String &dateString, String &timeString)
{

  // adjust utc time to local time
  time_t now_Local_Time = usEastern.toLocal(UTC_Time, &tcr);

  // format dateLine

  dateString = String(year(now_Local_Time));

  dateString.concat("-");

  if (month(now_Local_Time) < 10)
    dateString.concat("0");

  dateString.concat(String(month(now_Local_Time)));

  dateString.concat("-");

  if (day(now_Local_Time) < 10)
    dateString.concat("0");

  dateString.concat(String(day(now_Local_Time)));

  // format timeLine

  timeString = String(hourFormat12(now_Local_Time));

  timeString.concat(":");

  if (minute(now_Local_Time) < 10)
    timeString.concat("0");

  timeString.concat(String(minute(now_Local_Time)));

  timeString.concat(":");

  if (second(now_Local_Time) < 10)
    timeString.concat("0");

  timeString.concat(String(second(now_Local_Time)));

  if (isAM(now_Local_Time))
    timeString.concat(" AM");
  else
    timeString.concat(" PM");
};

String GetUpTime()
{

  unsigned long ms = millis();

  const int oneSecond = 1000;
  const int oneMinute = oneSecond * 60;
  const int oneHour = oneMinute * 60;
  const int oneDay = oneHour * 24;

  int numberOfDays = ms / oneDay;
  ms = ms - numberOfDays * oneDay;

  int numberOfHours = ms / oneHour;
  ms = ms - numberOfHours * oneHour;

  int numberOfMinutes = ms / oneMinute;
  ms = ms - numberOfMinutes * oneMinute;

  int numberOfSeconds = ms / oneSecond;

  String returnValue = "";

  char buffer[21];

  sprintf(buffer, "%d %02d:%02d:%02d", numberOfDays, numberOfHours, numberOfMinutes, numberOfSeconds);

  returnValue = String(buffer);
  return returnValue;
}

bool checkUpTimeRequest()
{

  bool returnValue = false;

  if (digitalRead(UpTimePin) == 0)
  {

    delay(10); // weed out false positives caused by debounce

    if (digitalRead(UpTimePin) == 0)
    {

      returnValue = true;
    };
  };

  return returnValue;
}

void updateTheDisplay(void *parameter)
{

  const int programNameRow = 0;   // the LCD row on which the program name will be displayed
                                  //
                                  // when an NTP request is not being processed:
  const int dateRow = 1;          // the LCD row on which the current date in local time will be displayed
  const int timeRow = 2;          // the LCD row on which the current time in local time will be displayed
  const int ipAddressRow = 3;     // the LCD row on which the IP Address of this device will be displayed
                                  //
                                  // when an NTP request is being processed:
  const int upTimeRequestRow = 1; // the LCD row on which the up time request message will be displayed
  const int upTimeResultsRow = 2; // the LCD row on which the up time will be displayed
  const int upTimeExplainRow = 3; // the LCD row on which a blank line will appear when displaying the up time

  int previouseTopLineMessage = -1;
  int previousSecond = -1;

  bool uptimeRequestBeingMade = false;
  bool anUptimeRequestHadBeenMade = false;

  while (true)
  {

    if (second() != previousSecond)
    {

      uptimeRequestBeingMade = checkUpTimeRequest();

      // Display the top line message (when it changes)
      //
      // Meanings of the top line message are:
      //
      //   the server is running                                             ESP32 Time Server
      //
      //   the server is running and a time refresh is underway              ESP32 Time Server *
      //
      //   the server is running and a time refresh is underway              ESP32 Time Server **
      //   but the server denied the last time refresh as it
      //   was outside the safeguard range
      //
      //   the user has pressed the button to request the server's up time   ESP32 Time Server's

      int RequiredTopLineMessage;

      if (uptimeRequestBeingMade)
        RequiredTopLineMessage = 4;
      else if (SafeGuardTripped)
        RequiredTopLineMessage = 3;
      else if (theTimeSettingProcessIsUnderway)
        RequiredTopLineMessage = 2;
      else
        RequiredTopLineMessage = 1;

      if (RequiredTopLineMessage != previouseTopLineMessage)
      {

        String TopLineMessage;
        if (RequiredTopLineMessage == 1)
          TopLineMessage = "ESP32 Time Server";
        else if (RequiredTopLineMessage == 2)
          TopLineMessage = "ESP32 Time Server *";
        else if (RequiredTopLineMessage == 3)
          TopLineMessage = "ESP32 Time Server **";
        else if (RequiredTopLineMessage == 4)
          TopLineMessage = "ESP32 Time Server's";

        display(programNameRow, TopLineMessage, false);

        previouseTopLineMessage = RequiredTopLineMessage;
      };

      // If an NTP request is underway

      // Display the IP address of the request
      // also show system uptime

      if (uptimeRequestBeingMade)
      {
        display(upTimeRequestRow, "up time is", false);

        // centre up time results on the display
        String ws = GetUpTime();
        int padLeftSpacesNeeded = (lcdColumns - ws.length()) / 2;
        String padLeft = fullyBlankLine.substring(0, padLeftSpacesNeeded);
        ws = padLeft + ws;
        display(upTimeResultsRow, ws, false);

        display(upTimeExplainRow, " days hrs:mins:secs", false);
        anUptimeRequestHadBeenMade = true;
      }
      else
      {

        // Dislay the date and time

        static int lastSecond = -1;
        static int lastDay = -1;

        // Get the time
        time_t now_UTC_Time = rtc.getEpoch();

        // if an update request had previously been made, then both the date and time lines will need to be refreshed,
        // otherwise:
        //   if the second hasn't changed since the last time it was displayed, then the display doesn't need to be updated further

        int thisSecond = second(now_UTC_Time);
        if ((thisSecond != lastSecond) || anUptimeRequestHadBeenMade)
        {

          lastSecond = thisSecond;

          // get the formated date and time as seperate strings

          String dateLine = "";
          String timeLine = "";
          GetAdjustedDateAndTimeStrings(now_UTC_Time, dateLine, timeLine);

          // update the time line on the display
          display(timeRow, timeLine, false);

          // if the date hasn't changed  since the last time it was displayed, then the display doesn't need to be updated

          int thisDay = day(now_UTC_Time);
          if ((thisDay != lastDay) || anUptimeRequestHadBeenMade)
          {

            // update the date line on the display
            display(dateRow, dateLine, false);
            lastDay = thisDay;
          };

          // if an uptime request had been made restore the ip address line
          if (anUptimeRequestHadBeenMade)
            display(ipAddressRow, ip);

          // clear the uptimeRequestBeingMade flag
          uptimeRequestBeingMade = false;
        };

        previousSecond = second();
        vTaskDelay(975 / portTICK_PERIOD_MS);
      };
    };
  };
}

void startAnOngoingTaskToUpdateTheDisplayEverySecond()
{

  xTaskCreatePinnedToCore(
      updateTheDisplay,     // Function that should be called
      "Update the display", // Name of the task (for debugging)
      3000,                 // Stack size (bytes)
      NULL,                 // Parameter to pass
      10,                   // Task priority
      &taskHandle0,         // Task handle
      0                     // use core 0 to split the load with setDateAndTimeFromGPS
  );
}

bool setTheGPSBaudRate(int gpsBaud, int maxAattemptsToChangeTheBaudRate)
{

  bool buadRateNeedsToBeSet = true;
  int attemptsToChangeTheBaudRate = 0;

  while ((buadRateNeedsToBeSet) && (attemptsToChangeTheBaudRate < maxAattemptsToChangeTheBaudRate))
  {

    if (debugIsOn)
    {
      Serial.println("Attempt " + String(int(attemptsToChangeTheBaudRate + 1)) + " of " + String(maxAattemptsToChangeTheBaudRate) + ":");
      Serial.println("  Set baud rate to " + String(gpsBaud));
    }

    GPSDevice.begin(gpsBaud);
    delay(100);
    if (gps.begin(GPSDevice))
    {
      buadRateNeedsToBeSet = false;
    }
    else
    {

      if (debugIsOn)
        Serial.println("  Could not connect at a buad rate of " + String(gpsBaud) + ", now trying 9600 baud");

      GPSDevice.begin(9600);

      if (gps.begin(GPSDevice))
      {
        if (debugIsOn)
          Serial.println("  Connected at 9600 baud, switching to " + String(gpsBaud) + " baud");
        gps.setSerialRate(gpsBaud);
        delay(100);
      }
      else
      {
        if (debugIsOn)
        {
          Serial.println("  Could not connect at a buad rate of 9600 baud");
          if ((attemptsToChangeTheBaudRate + 1) != maxAattemptsToChangeTheBaudRate)
            Serial.println("  will try again");
        };
        // gps.factoryDefault();
        delay(2000); // Wait a bit before trying again to limit the Serial output
      };

      attemptsToChangeTheBaudRate++;
    };
  };

  //   gps.saveConfiguration();   // Optionally: if when you are testing this sketch the buad rate is constantly being 
  //                              // successfully set at the desired rate (38400) then you can uncomment this line of code
  //                              // so that the GPS's configuration will default to starting at 34800 rather than 9600.  
  //                              // However, as this will only need to be done once this line of code should then be 
  //                              // commented back out for subsequent runs.  This procedure is entirely optional.

  if (debugIsOn)
  {
    Serial.println("");
    if (buadRateNeedsToBeSet)
    {
      Serial.println("The baud rate on the GPS could not be set to " + String(gpsBaud));
    }
    else
    {
      Serial.println("  Baud rate set to " + String(gpsBaud));
    };
    Serial.println("");
  };

  return !buadRateNeedsToBeSet;
};

void setupGPS()
{

  if (!setTheGPSBaudRate(GPSBaud, 10))
  {
    display(1, "Something is wrong");
    display(2, "couldn't communicate");
    display(3, "with the GPS device.");
    // Freeze here
    while (true)
      ;
  };

  gps.setI2COutput(0);
  gps.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
  gps.setUART2Output(0);

  display(1, "Waiting for GPS fix");

  unsigned long nextCheck = millis() + oneSecond_inMilliseconds;
  byte fixType;

  bool continueWaitingForAFix = true;
  while (continueWaitingForAFix)
  {

    // only check once a second
    if (millis() > nextCheck)
    {

      nextCheck = millis() + oneSecond_inMilliseconds;

      fixType = gps.getFixType();
      if ((fixType > 0) && (fixType < 6))
      {

        String msg;
        switch (int(fixType))
        {
        case 0:
          msg = "No fix";
          break;
        case 1:
          msg = "Dead reckoning";
          break;
        case 2:
          msg = "2D";
          break;
        case 3:
          msg = "3D";
          break;
        case 4:
          msg = "GNSS + Dead reckoning";
          break;
        case 5:
          msg = "Date and time";
          break;
        };
        display(1, "GPS fix obtained");
        display(2, msg);
        delay(5000);

        continueWaitingForAFix = false;
      };
    };
  };
}

void setDateAndTimeFromGPS(void *parameter)
{

  static bool thisIsTheFirstTimeSetBeingMadeAtStartup = true;

  /// used below to ensure a GPS time refresh if is only performed if the difference between the old and new times is reasonable for the periodicTimeRefreshPeriod
  const time_t safeguardThresholdHigh = safeguardThresholdInSeconds;
  const time_t safeguardThresholdLow = -1 * safeguardThresholdInSeconds;

  time_t candidateDateAndTime;

  if (debugIsOn)
    Serial.println("Start setDateAndTimeFromGPS task");

  while (true)
  {

    theTimeSettingProcessIsUnderway = true;

    // wait for the ppsFlag to be raised at the start of the 1st second
    ppsFlag = false;
    while (!ppsFlag)
      ;

    if (gps.getPVT()) // get latest time data (to reflect the start of the next second)
    {

      if (gps.getDateValid() && gps.getTimeValid()) // make sure the date and time are valid (in that values are populated)
      {

        struct tm wt;
        wt.tm_year = gps.getYear();
        wt.tm_mon = gps.getMonth();
        wt.tm_mday = gps.getDay();
        wt.tm_hour = gps.getHour();
        wt.tm_min = gps.getMinute();
        wt.tm_sec = gps.getSecond();

        if ((wt.tm_year > 2022) && (wt.tm_mon > 0) && (wt.tm_mon < 13) && (wt.tm_mday > 0) && (wt.tm_mday < 32) && (wt.tm_hour < 24) && (wt.tm_min < 60) && (wt.tm_sec < 61)) // make sure the values are within reason
        {

          // set candidate time according the gps (this will be effective when the PPS flag is next raised)
          wt.tm_year -= 1900;                     // adjust year (see you again in 2036)
          wt.tm_mon -= 1;                         // adjust month (January is month 0)
          candidateDateAndTime = mktime(&wt) + 1; // not sure why the + 1 but it is

          if (debugIsOn)
            Serial.println("Candidate date and time " + String(wt.tm_year) + " " + String(wt.tm_mon) + " " + String(wt.tm_mday) + " " + String(wt.tm_hour) + " " + String(wt.tm_min) + " " + String(wt.tm_sec));

          time_t wt = candidateDateAndTime;
          time_t candiateDateAndTime_t = time(&wt);

          // give some time to ensure the PPS pin is reset
          vTaskDelay(200 / portTICK_PERIOD_MS);

          // wait for the PPS flag to be raised (signifying the true start of the candidate time)
          ppsFlag = false;
          while (!ppsFlag)
            ;

          unsigned long pegProcessingAdjustmentStartTime = micros();

          // at this point:
          // apply a sanity check; the current rtc time and the candidate time just taken from the gps readings which will be used to refresh the current rtc should be within a second of each other (safeguardThresholdInSeconds)
          // if the sanity check fails, do not set the time and raise a Safeguard flag which be used to update the display to show the user the latest time refresh failed
          // if the sanity check passes, proceed with refreshing the time and if the Safeguard flag been previously been raised then lower it

          bool SanityCheckPassed;
          time_t updateDelta;

          if (thisIsTheFirstTimeSetBeingMadeAtStartup)
          {
            SanityCheckPassed = true;
          }
          else
          {
            time_t currentRTC_t = rtc.getEpoch();
            time_t currentRTCDateAndTime_t = time(&currentRTC_t);
            updateDelta = currentRTCDateAndTime_t - candiateDateAndTime_t;
            bool SanityCheckPassed = (((updateDelta >= safeguardThresholdLow) && (updateDelta <= safeguardThresholdHigh)));
          };

          if (SanityCheckPassed)
          {

            // place a hold on (the date and time) so if an NTP request is underway in the fraction of a second this code will take, the time and date values don't change mid way through that request.
            if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
            {

              // set the date and time

              unsigned long pegProcessingAdjustmentEndTime = micros();
              unsigned long ProcessingAdjustment = pegProcessingAdjustmentEndTime - pegProcessingAdjustmentStartTime;

              // set the real time clock
              rtc.setTime((unsigned long)candidateDateAndTime, (int)ProcessingAdjustment);

              // release the hold
              xSemaphoreGive(mutex);

              if (debugIsOn)
              {
                Serial.print("Date and time set to ");
                String ws = rtc.getDateTime(true);
                ws.trim();
                Serial.println(ws + " (UTC)");
              };

              SafeGuardTripped = false;
              theTimeSettingProcessIsUnderway = false;
              thisIsTheFirstTimeSetBeingMadeAtStartup = false;

              // whew that was hard work but fun, lets take a break and then do it all again
              vTaskDelay(periodicTimeRefreshPeriod / portTICK_PERIOD_MS);
            }
            else
            {
              if (debugIsOn)
              {
                Serial.println("Could not refresh the time as a NTP request was underway");
                Serial.println("Will try again");
              };
            }
          }
          else
          {
            if (debugIsOn)
            {
              Serial.println("This date and time refresh failed its sanity check with a time delta of " + String(updateDelta) + " seconds");
              Serial.println("The time was not refreshed.");
              Serial.print("Date and time are ");
              String ws = rtc.getDateTime(true);
              ws.trim();
              Serial.println(ws + " (UTC)");
              Serial.println("Will try again");
            };

            SafeGuardTripped = true;
          };
        };
      };
    };
  };
};

void startAnOngoingTaskToRefreshTheDateAndTimeFromTheGPS()
{

  xTaskCreatePinnedToCore(
      setDateAndTimeFromGPS,
      "Set Date and Time from GPS",
      3000,
      NULL,
      20, // task priority must be reasonably high or the queues from which the gps data is drawn will not be adequately replenished
      &taskHandle1,
      1 // use core 1 to split the load with updateTheDisplay
  );
};

void EthEvent(WiFiEvent_t event)
{

  const int rowOfDisplayToShowStatus = 1;
  const int rowOfDisplayToShowIP = 3;

  switch (event)
  {
  case ARDUINO_EVENT_ETH_START:
    ETH.setHostname("MasterClock");
    display(rowOfDisplayToShowStatus, "Ethernet started");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    display(rowOfDisplayToShowStatus, "Ethernet connected");
    eth_connected = true;
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    ip = ETH.localIP().toString();
    display(rowOfDisplayToShowIP, ip);
    eth_got_IP = true;
    eth_connected = true;
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    display(rowOfDisplayToShowStatus, "Ethernet disconnect");
    eth_connected = false;
    break;
  case ARDUINO_EVENT_ETH_STOP:
    display(rowOfDisplayToShowStatus, "Ethernet stopped");
    eth_connected = false;
    break;
  default:
    break;
  }
}

void setupEitherNet()
{

  WiFi.onEvent(EthEvent);
  ETH.begin();

  while (!eth_got_IP)
    delay(1);
}

void startUDPSever()
{

  Udp.begin(NTP_PORT);
}

uint64_t getCurrentTimeInNTP64BitFormat()
{

  const uint64_t numberOfSecondsBetween1900and1970 = 2208988800;

  uint64_t clockSecondsSinceEpoch = numberOfSecondsBetween1900and1970 + (uint64_t)rtc.getEpoch();
  long clockMicroSeconds = (long)rtc.getMicros();

  // as one might infer clockMicroSeconds is in microseconds (i.e. 1 second = 1,000,000 microseconds)
  //
  // accordingly, if the clockMicroSeconds is greater than one million ...
  //   for every million that is over:
  //     add 1 (second) to clockSecondsSinceEpoch, and
  //     reduce the clockMicroSeconds by one million (microseconds)
  //
  // likewise ...
  //
  // if the clockMicroSeconds is less than zero:
  //   for every million that is under zero:
  //     subtract (second) from clockSecondsSinceEpoch, and
  //     increase the clockMicroSeconds by one million (microseconds)

  while (clockMicroSeconds > oneSecond_inMicroseconds_L)
  {
    clockSecondsSinceEpoch++;
    clockMicroSeconds -= oneSecond_inMicroseconds_L;
  };

  while (clockMicroSeconds < 0L)
  {
    clockSecondsSinceEpoch--;
    clockMicroSeconds += oneSecond_inMicroseconds_L;
  };

  // for the next two lines to be clear, please see: https://tickelton.gitlab.io/articles/ntp-timestamps/

  double clockMicroSeconds_D = (double)clockMicroSeconds * (double)(4294.967296);
  uint64_t ntpts = ((uint64_t)clockSecondsSinceEpoch << 32) | (uint64_t)(clockMicroSeconds_D);

  return ntpts;
}

// send NTP reply
void sendNTPpacket(IPAddress remoteIP, int remotePort)
{

  // set the receive time to the current time
  uint64_t receiveTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  // Initialize values needed to form NTP request

  // LI: 0, Version: 4, Mode: 4 (server)
  // packetBuffer[0] = 0b00100100;
  // LI: 0, Version: 3, Mode: 4 (server)
  packetBuffer[0] = 0b00011100;

  // Stratum, or type of clock
  packetBuffer[1] = 0b00000001;

  // Polling Interval
  packetBuffer[2] = 4;

  // Peer Clock Precision
  // log2(sec)
  // 0xF6 <--> -10 <--> 0.0009765625 s
  // 0xF7 <--> -9 <--> 0.001953125 s
  // 0xF8 <--> -8 <--> 0.00390625 s
  // 0xF9 <--> -7 <--> 0.0078125 s
  // 0xFA <--> -6 <--> 0.0156250 s
  // 0xFB <--> -5 <--> 0.0312500 s
  packetBuffer[3] = 0xF7;

  // 8 bytes for Root Delay & Root Dispersion
  // root delay
  packetBuffer[4] = 0;
  packetBuffer[5] = 0;
  packetBuffer[6] = 0;
  packetBuffer[7] = 0;

  // root dispersion
  packetBuffer[8] = 0;
  packetBuffer[9] = 0;
  packetBuffer[10] = 0;
  packetBuffer[11] = 0x50;

  // time source (namestring)
  packetBuffer[12] = 71; // G
  packetBuffer[13] = 80; // P
  packetBuffer[14] = 83; // S
  packetBuffer[15] = 0;

  // get the current time and write it out as the reference time to bytes 16 to 23 of the response packet
  uint64_t referenceTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  packetBuffer[16] = (int)((referenceTime_uint64_t >> 56) & 0xFF);
  packetBuffer[17] = (int)((referenceTime_uint64_t >> 48) & 0xFF);
  packetBuffer[18] = (int)((referenceTime_uint64_t >> 40) & 0xFF);
  packetBuffer[19] = (int)((referenceTime_uint64_t >> 32) & 0xFF);
  packetBuffer[20] = (int)((referenceTime_uint64_t >> 24) & 0xFF);
  packetBuffer[21] = (int)((referenceTime_uint64_t >> 16) & 0xFF);
  packetBuffer[22] = (int)((referenceTime_uint64_t >> 8) & 0xFF);
  packetBuffer[23] = (int)(referenceTime_uint64_t & 0xFF);

  // copy transmit time from the NTP original request to bytes 24 to 31 of the response packet
  packetBuffer[24] = packetBuffer[40];
  packetBuffer[25] = packetBuffer[41];
  packetBuffer[26] = packetBuffer[42];
  packetBuffer[27] = packetBuffer[43];
  packetBuffer[28] = packetBuffer[44];
  packetBuffer[29] = packetBuffer[45];
  packetBuffer[30] = packetBuffer[46];
  packetBuffer[31] = packetBuffer[47];

  // write out the receive time (it was set above) to bytes 32 to 39 of the response packet
  packetBuffer[32] = (int)((receiveTime_uint64_t >> 56) & 0xFF);
  packetBuffer[33] = (int)((receiveTime_uint64_t >> 48) & 0xFF);
  packetBuffer[34] = (int)((receiveTime_uint64_t >> 40) & 0xFF);
  packetBuffer[35] = (int)((receiveTime_uint64_t >> 32) & 0xFF);
  packetBuffer[36] = (int)((receiveTime_uint64_t >> 24) & 0xFF);
  packetBuffer[37] = (int)((receiveTime_uint64_t >> 16) & 0xFF);
  packetBuffer[38] = (int)((receiveTime_uint64_t >> 8) & 0xFF);
  packetBuffer[39] = (int)(receiveTime_uint64_t & 0xFF);

  // get the current time and write it out as the transmit time to bytes 40 to 47 of the response packet
  uint64_t transmitTime_uint64_t = getCurrentTimeInNTP64BitFormat();

  packetBuffer[40] = (int)((transmitTime_uint64_t >> 56) & 0xFF);
  packetBuffer[41] = (int)((transmitTime_uint64_t >> 48) & 0xFF);
  packetBuffer[42] = (int)((transmitTime_uint64_t >> 40) & 0xFF);
  packetBuffer[43] = (int)((transmitTime_uint64_t >> 32) & 0xFF);
  packetBuffer[44] = (int)((transmitTime_uint64_t >> 24) & 0xFF);
  packetBuffer[45] = (int)((transmitTime_uint64_t >> 16) & 0xFF);
  packetBuffer[46] = (int)((transmitTime_uint64_t >> 8) & 0xFF);
  packetBuffer[47] = (int)(transmitTime_uint64_t & 0xFF);

  // send the reply
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void processNTPRequests()
{

  unsigned long replyStartTime = micros();

  int packetSize = Udp.parsePacket();

  if (packetSize == NTP_PACKET_SIZE) // an NTP request has arrived
  {

    // store sender ip for later use
    IPAddress remoteIP = Udp.remoteIP();

    // read the data from the packet into the buffer for later use
    Udp.read(packetBuffer, NTP_PACKET_SIZE);

    // hold here if and while the date and time are being refreshed
    // when ok to proceed place a hold on using the mutex to prevent the date and time from being refreshed while the reply packet is being built
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
      // send NTP reply
      sendNTPpacket(remoteIP, Udp.remotePort());
      xSemaphoreGive(mutex);
    };

    // report query in serial monitor
    // note: unlike other serial monitor writes in this sketch, this particular write is on the critical path for processing NTP requests.
    // while it does not delay the response to an initial NTP request, if subsequent NTP requests are queued up to run directly afterward
    // this serial monitor write will delay responding to the queued request by approximately 1 milli second.
    if (debugIsOn)
    {

      String dateLine = "";
      String timeLine = "";
      GetAdjustedDateAndTimeStrings(rtc.getEpoch(), dateLine, timeLine);
      String updatemessage = "Query from " + remoteIP.toString() + " on " + dateLine + " at " + timeLine;
      Serial.println(updatemessage);
    };
  }
  else
  {
    if (packetSize > 0)
    {
      Udp.flush(); // not sure what this incoming packet is, but it is not an ntp request so get rid of it
      if (debugIsOn)
        Serial.println("Invalid request received on port " + String(NTP_PORT) + ", length =" + String(packetSize));
    };
  };
}

void ppsHandlerRising()
{                 // PPS interrupt handler
  ppsFlag = true; // raise the flag that signals the start of the next second
}

void setup()
{

  setupSerial();

  if (debugIsOn)
    Serial.println("ESP32 Time Server starting setup ...");

  turnOffWifiAndBluetooth();

  setupButton();

  setupDisplay();

  display(1, "GPS setup underway");
  setupGPS();

  display(1, "Getting date & time", false);
  display(2, " ", false);
  display(3, " ", false);

  // create a mutex to be used to ensure an NTP request results are not impacted by the process that refreshes the time
   mutex = xSemaphoreCreateMutex();

  // setup for the use of the pulse-per-second pin
  pinMode(PPSPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPSPin), ppsHandlerRising, RISING);

  startAnOngoingTaskToRefreshTheDateAndTimeFromTheGPS();

  // wait until the time is actually set
  while (theTimeSettingProcessIsUnderway)
    delay(10);

  display(1, "Connecting Ethernet", false);
  display(2, " ", false);
  display(3, " ", false);
  setupEitherNet();
  startUDPSever();

  startAnOngoingTaskToUpdateTheDisplayEverySecond();

  if (debugIsOn)
    Serial.println("ESP32 Time Server setup complete - listening for NTP requests now");
}

void loop()
{
  processNTPRequests();
}
// Rob Latour, 2023
// License: MIT
// https://github.com/roblatour

#include <TimeLib.h>

// Key settings for the ESP32 Time Server project
  
// GPS  
const int RXPin = 36, TXPin = 4;                  // RX and TX pins on the ESP32 board (The ESP32's RX pin is connected to the GPS's TX pin, and the ESP32's TX pin is connected to the GPS's RX pin)
const int PPSPin = 2;                             // PPS pin on the ESP32 board
const uint32_t GPSBaud = 38400;                   // desired GPS baud 

// LCD  
const int lcdI2CAddress = 0x27;                   // lcd I2C address; if you don't know your display address, run an I2C scanner sketch
const int lcdColumns = 20;                        // number of columns on your LCD display (typically 16 or 20)
const int lcdRows = 4;                            // number of rows on your LCD display (typically 2 or 4)

// Button                                         // the button is used to show up time
const int UpTimePin = 34;                         // pin on the ESP32 board which the uptime button is connected
const int displayUpTimeSecondsToStayActive = 10;  // number of seconds the Up Time display will stay active

// Serial monitor  
int SerialMonitorSpeed = 115200;                  // Serial monitor bps
bool debugIsOn = false;                           // set to true to see progress results in the serial monitor, set to false to suppress these

// Time zone                                      // Time is displayed on the LCD display in keeping with your local time zone                                             
                                                  // please see https://github.com/khoih-prog/Timezone_Generic#timechangerules-struct for more information
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};  
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};   
Timezone myTZ(myDST, mySTD);  
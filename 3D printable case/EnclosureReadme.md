**3D Print models for the ESP32 Time Server project**<br>
https://github.com/roblatour/ESP32TimeServer <br>
Copyright Rob Latour, 2026


**Fusion 380 Source file:** <br>

The Fusion 360 source code containing the models for all 3D print files identified below:<br>
ESP32 Time Server enclosure.f3z

**3D Print Files included:**<br>

The following 3D print files are include for each of the enclosure parts listed below:<br>
.stl file, <br>
(PRUSA Slicer) .3mf file, and <br>
(PRUSA Slicer) GCode file (using PTEG).

**Case:**<br>
The case (filename Case) is designed to accommodate the following:<br>

- Waveshare ESP32-P4-ETH development board (with or without a POE Hat)
  https://www.waveshare.com/esp32-p4-eth.htm
  
  or 
  
  Olimex ESP32-ETH-POE (Revision B)
  (discontinued)

- Sparkfun Max-M10s GPS module:<br>
  https://www.sparkfun.com/sparkfun-gnss-receiver-breakout-max-m10s-qwiic.html

- a LCD 2004 screen<br>
  https://www.aliexpress.com/item/1005006829045609.html

- an optional USB C cable connector<br>
  https://www.aliexpress.com/item/1005006584965187.html  (right angle connector)

- button<br>
  https://www.aliexpress.com/item/1005004066257419.html


**Backing**<br>

The particular case back needed depends on the configuration choosen.

The Backing file naming convention is as follows:

Fileaname contains: Back

Filename contains either: ESP32-P4-ETH or OLIMEX to indicate which development board it accommodates.

Filename contains either: USB or NoUSB to indicate if an opening for the USB connector is included in the print (see note below).

Filename contains PCB to indicate the back is designed to accommodate the PCB.

**Brace**

The Olimex enclosure requires the printing of the Brace Olimex file.<br>
The ETH-P4-ETH enclosure does not require a separate brace file as the brace is built into the back.


Notes:

1. The ESP32-P4-ETH may be powered either by POE or USB-C - BUT MUST NOT BE POWERED BY BOTH AT THE SAME TIME.  

2. If using the ESP32-P4-ETH with the POE hat the USB cable should be connected only when the Ethernet cable is not coming from an POE powering device (such as a POE switch or POE injector).

3. The USB-C cable may be use to get console log outputs from the ESP32-P4-ETH board, or to flash it.  However, flashing may also be achieved via an Over The Ethernet update.

4. The Olimex related models above were designed to accommodate the Olimex revision B board - newer revisions may not work4

5. The Olimex board also requires the printing of the Olimex brace file.

6. A USB-C cable connector option is not available for the Olimex board configuration.


























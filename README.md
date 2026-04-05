# ESP32 NTP Stratum 1 Time Server (version 2)

**ESP32 NTP Stratum 1 Time Server for your home network**

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

---

## Overview

This project was born with a simple goal: create a low-budget, **Stratum 1 NTP time server** solution with no need for periodic Linux OS updates. Rather just set it up and let it do its job.

![ESP32 Time Server](./PCB/photo01.jpg)

It uses a **GNSS North America & Europe) receiver** as its time source, making it a true Stratum 1 server — meaning it gets its time directly from satellites rather than from another NTP server. Every device on your home network can then synchronize to it for highly accurate local time.

A full write-up of the original (version 1) project is available on [Hackaday.io](https://hackaday.io/project/189309-esp32-ntp-time-server-stratum-1).

---

## What's New in Version 2

Version 2 is a significant upgrade over [Version 1](https://github.com/roblatour/ESP32TimeServer/releases/tag/v1.0.0.0):

- **New microcontroller board** — now built around the [WaveShare ESP32-P4-ETH](https://www.waveshare.com/esp32-p4-eth.htm) instead of the Olimex ESP32-PoE-ISO. The ESP32-P4 CPU is much faster than the ESP32-WROOM-32/32E used in v1.
- **More accurate NTP responses** — PPS (Pulse Per Second) pin support is now fully exploited to discipline the time reference, delivering sub-millisecond accuracy when using a PPS-capable GPS module.
- **Over-the-Ethernet OTA updates** — firmware can now be updated over the Ethernet connection directly from VS Code, with on-screen progress shown on the LCD.
- **LCD is now optional** — in v1 an LCD 2004 screen was required; in v2 it is entirely optional.
- **Optional up time / reset button** — carry-over from v1, still supported.
- **Updated 3D printed case** — the enclosure files have been updated for the new WaveShare ESP32-P4-ETH board.
- **Built with ESP-IDF** — rewritten from the ground up in C++ on ESP-IDF v5.5.3 using FreeRTOS tasks.
- **NVS-backed GPS state** — the GPS module identity and baud rate are persisted across reboots so following the initial setup startup speed is quick.
- **Broader GNSS module support** — still works great with the SparkFun MAX-M10S (recommended), but now also supports lower-cost, generic GNSS modules, even those (although not recommended) that do not expose a PPS pin.

> The source code for **Version 1** (Arduino / PlatformIO) remains available at:
> [https://github.com/roblatour/ESP32TimeServer/releases/tag/v1.0.0.0](https://github.com/roblatour/ESP32TimeServer/releases/tag/v1.0.0.0)

---

## Hardware

| Qty | Item                                                                                                                                                                                                                                                                                                                                                                        |
| --- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | [WaveShare ESP32-P4-ETH](https://www.waveshare.com/esp32-p4-eth.htm) development board (with or without optional PoE Hat) <sup>1</sup>                                                                                                                                                                                                                                      |
| 1   | GPS/GNSS module [Recommeded: SparkFun GNSS Receiver Breakout - MAX-M10S (Qwiic)](https://www.sparkfun.com/sparkfun-gnss-receiver-breakout-max-m10s-qwiic.html) <sup>1</sup>. Note: while some lower-cost generic modules ([AliExpress - NEO8M](https://www.aliexpress.com/item/1005003721844881.html)) are also supported, those including a PPS pin are strongly preferred |
| 1   | GPS/GNSS antenna with SMA connector ([SparkFun GPS/GNSS Magnetic Mount Antenna - 3m (SMA)](https://www.sparkfun.com/products/14986)) <sup>1</sup>                                                                                                                                                                                                                           |
| 1   | *(Optional)* 4×20 I²C LCD display with HD44780 controller with PCF8574 I²C backpack ([AliExpress](https://www.aliexpress.com/item/1005006829045609)) <sup>1</sup>                                                                                                                                                                                                           |
| 1   | *(Optional)* Momentary push button for displaying up time and triggering a reset ([AliExpress](https://www.aliexpress.com/item/1005004066257419.html)) <sup>1</sup>                                                                                                                                                                                                         |
| 1   | *(Optional)* USB C extension cable (with right angle end) ([AliExpress](https://www.aliexpress.com/item/1005006584965187.html)) <sup>1</sup> + two M3*8                                                                                                                                                                                                                     |
| —   | Miscellaneous: Ethernet cable, female dupont connection wires, small 4" .1" zip ties, solder <sup>2</sup>                                                                                                                                                                                                                                                                   |
| —   | A PoE-capable switch, PoE injector, **or** USB-C power supply and USB-C cable                                                                                                                                                                                                                                                                                               |
> ⚠️ **WARNING — Do NOT power the ESP32-P4-ETH via both its USB-C connector and a PoE powered Ethernet cable at the same time.**
>  Powering from both simultaneously may damage the ESP32-P4-ETH board, POE switch, or device providing USB power. Either power source alone is sufficient to power the board, GPS module, and LCD screen.
> 

<sup>1</sup> *(Optional)* 3D Printable Case designed with these specific components in mind (for more information see below)<br>

<sup>2</sup>  *(Optional)* To cut down on wiring a PCB can be used.  One designed to work within this project's 3D printed enclosure is available in the [`PCB`](./PCB) folder. If your interested, please review the `Readme.md` file in that folder for more information.<br>


### Wiring

**GPS module → ESP32-P4-ETH** *(mandatory)*

| GPS pin | ESP32-P4-ETH pin |
| ------- | ---------------- |
| GND     | GND              |
| VCC     | 3V3              |
| TXD     | GPIO 17 (RX)     |
| RXD     | GPIO 16 (TX)     |
| PPS     | GPIO 18          |

**LCD 2004 (HD44780 + PCF8574 I²C backpack) → ESP32-P4-ETH** *(optional)*

| LCD pin | ESP32-P4-ETH pin |
| ------- | ---------------- |
| GND     | GND              |
| VCC     | 3V3              |
| SDA     | GPIO 8           |
| SLC/SCL | GPIO 7           |

**Up time / Reset button → ESP32-P4-ETH** *(optional)*
| Button             | ESP32-P4-ETH pin |
| ------------------ | ---------------- |
| One terminal       | GND              |
| The other terminal | GPIO 3           |

Refer to the pin definitions in [`main/ESP32TimeServerSettings.h`](./main/ESP32TimeServerSettings.h).


---

## 3D Printed Case

An downloadable 3D printable enclosure designed for use the WaveShare ESP32-P4-ETH (with or without the POE hat) + Sparkfun GPS boards is included in the [`3D printable case/`](./3D%20printable%20case/) folder. Both `.stl` (print-ready) and `.f3z` (Fusion 360 editable source) files are provided so you can tweak the design to suit your needs.

> **Note:** the 3D printable enclosure files also include the needed models for version one of this project which used the Olimex ESP32-PoE-ISO Rev. B.

---

## Software

### Development Environment

This release is built using:

- **[Visual Studio Code](https://code.visualstudio.com/)**
- **[Espressif ESP-IDF Extension for VS Code](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)**
- **ESP-IDF v5.5.3**

### Dependencies

Dependencies are managed automatically via the ESP-IDF Component Manager:

- [`SparkFun u-blox GNSS Arduino Library v3`](https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3) — GPS communication
- `esp-idf-lib/hd44780` — LCD driver
- `esp-idf-lib/pcf8574` — I²C LCD backpack driver
- `espressif/arduino-esp32` — Arduino compatibility layer (used for OTA and serial)

### Configuration

All user-configurable settings — GPIO pins, GPS options, LCD options, button support, time zone, and safeguard threshold — are centralised in:

```
main/ESP32TimeServerSettings.h
```

Edit this file before building to match your hardware setup.

### Build & Flash

Load the ESP-IDF environment, then build and flash:

```cmd
call C:\esp\v5.5.3\esp-idf\export.bat
idf.py build
idf.py -p COMx flash monitor
```

*(Replace `COMx` with your actual serial port.)*

### OTA Updates (Over Ethernet)

A VS Code task **"ESP-IDF: OTA Upload over Ethernet"** is included in `.vscode/tasks.json`. It builds the firmware and deploys it to the device over Ethernet using `espota.py` — no USB cable needed after the first flash.  Alternatively, there is a terminal command that can be used, it is found in the `misc` folder, in the file [`Useful Power Shell Commands.txt`](./misc/usefull%20powershell%20commands.txt).

---

## Validating Your Time Server

Once running, and with your Network and System changes setup, you can validate accuracy at [https://time.is](https://time.is). 

---
### Setting up your Network / Systems to make use of the ESP32 Time Server

Please see [`Setup.md`](Setup.md)

---

## License

This project is released under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

## Supporting This Project

To help support this project, or to just say thanks, you're welcome to 'buy me a coffee'

[<img alt="buy me a coffee" width="200px" src="https://cdn.buymeacoffee.com/buttons/v2/default-blue.png" />](https://www.buymeacoffee.com/roblatour)

---

Copyright © 2026 Rob Latour

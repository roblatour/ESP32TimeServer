# ESP32 NTP Stratum 1 Time Server (version 2.7)

An ESP32 NTP Stratum 1 Time Server for your home network

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

---

## Overview

This project was born with a simple goal: create a low-budget, **Stratum 1 NTP
time server** solution with no need for periodic Linux OS updates. Rather just
set it up and let it do its job.

![ESP32 Time Server](./PCB/photo01.jpg)

It uses a **GNSS (North America & Europe) receiver** as its time source, making
it a true Stratum 1 server — meaning it gets its time directly from satellites
rather than from another NTP server. Every device on your home network can then
synchronize to it for highly accurate local time.

A full write-up of the original (version 1) project is available on
[Hackaday.io](https://hackaday.io/project/189309-esp32-ntp-time-server-stratum-1).

---

## What's New in

[Version 2](https://github.com/roblatour/ESP32TimeServer/releases/tag/v2.0.0.0)

- **New microcontroller board** — now built around the
  [WaveShare ESP32-P4-ETH](https://www.waveshare.com/esp32-p4-eth.htm) instead
  of the Olimex ESP32-PoE-ISO. The ESP32-P4 is more modern and faster than the
  ESP32-WROOM-32/32E used in v1.

- **More accurate NTP responses** — PPS (Pulse Per Second) pin support is now
  fully exploited to discipline the time reference, delivering sub-millisecond
  accuracy when using a PPS-capable GPS module.
- **Over-the-Ethernet OTA updates** — firmware can now be updated over the
  Ethernet connection directly from VS Code, with on-screen progress shown on
  the LCD.
- **LCD is now optional** — in v1 an LCD 2004 screen was required; in v2 it is
  entirely optional.
- **Optional up time / reset button** — carry-over from v1, still supported.
- **Updated 3D printed case** — the enclosure files have been updated for the
  new WaveShare ESP32-P4-ETH board.
- **Built with ESP-IDF** — rewritten from the ground up in C++ on ESP-IDF v5.5.3
  using FreeRTOS tasks.
- **NVS-backed GPS state** — the GPS module identity and baud rate are persisted
  across reboots so following the initial setup startup speed is quick.
- **Broader GNSS module support** — still works great with the SparkFun MAX-M10S
  (recommended), but now also supports lower-cost, generic GNSS modules, even
  those (although not recommended) that do not expose a PPS pin.
- **Ability to set a custom MAC Address** — (version 2.2) allows the use of the
  default ESP32-P4's MAC address or for a custom MAC address to be set.
- **Ability to set a Static IP Address** — (version 2.3) allows the use of a
  DHCP assigned or static IP address.
- **Handles greater throughput and number of concurrent requests** (version
  2.4.1).
- **IPv6 support** - (version 2.5) IPv6 support has been added.
- **MQTT publishing** — (version 2.5) optional MQTT publishing of time server
  stats is now available. For more information see
  [misc/esp32timeserver_json_doc.md](misc/esp32timeserver_json_doc.md)
- **Improved GNSS Satellite lock and PPS discipline tracking and recovery** -
  (version 2.5) with returned results being tagged as Stratum 16 (undefined)
  until a lost lock and/or failed PPS discipline is recovered.
- **Conditional compilation** - (version 2.5) when optional features are
  disabled in the settings their associated code will now be excluded from the
  executable.
- **TF card support** — (version 2.6) enabling queueing of vastly greater
  amounts of MQTT reporting data should broker communications be lost
- **Home Assistant** - (version 2.6) dashboard card setup documentation added

<!-- markdownlint-disable MD059 -->

[here](./Homeassistant/README.md)
<!-- markdownlint-disable MD059 -->

> The source code for **Version 1** (Arduino / PlatformIO) remains available at:
> [https://github.com/roblatour/ESP32TimeServer/releases/tag/v1.0.0.0](https://github.com/roblatour/ESP32TimeServer/releases/tag/v1.0.0.0)

---

## Hardware

<!-- markdownlint-disable line-length table-column-style -->

| Qty | Item                                                                                                                                                                                                                                                                                                                                                                         |
| --- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | [WaveShare ESP32-P4-ETH](https://www.waveshare.com/esp32-p4-eth.htm) development board (with or without optional PoE Hat) <sup>1</sup>                                                                                                                                                                                                                                       |
| 1   | GPS/GNSS module [Recommended: SparkFun GNSS Receiver Breakout - MAX-M10S (Qwiic)](https://www.sparkfun.com/sparkfun-gnss-receiver-breakout-max-m10s-qwiic.html) <sup>1</sup>. Note: while some lower-cost generic modules ([AliExpress - NEO8M](https://www.aliexpress.com/item/1005003721844881.html)) are also supported, those including a PPS pin are strongly preferred |
| 1   | GPS/GNSS antenna with SMA connector ([SparkFun GPS/GNSS Magnetic Mount Antenna - 3m (SMA)](https://www.sparkfun.com/products/14986)) <sup>1</sup>                                                                                                                                                                                                                            |
| 1   | _(Optional)_ TF Card, formatted as FAT32, if you're using MQTT and want more than four messages queued should broker communications be down) <sup>1</sup>                                                                                                                                                                                                                    |
| 1   | _(Optional)_ 4×20 I²C LCD display with HD44780 controller with PCF8574 I²C backpack ([AliExpress](https://www.aliexpress.com/item/1005006829045609)) <sup>1</sup>                                                                                                                                                                                                            |
| 1   | _(Optional)_ Momentary push button for displaying up time and triggering a reset ([AliExpress](https://www.aliexpress.com/item/1005004066257419.html)) <sup>1</sup>                                                                                                                                                                                                          |
| 1   | _(Optional)_ USB C extension cable (with right angle end) ([AliExpress](https://www.aliexpress.com/item/1005006584965187.html)) <sup>1</sup> + two M3*8                                                                                                                                                                                                                      |
| —   | Miscellaneous: Ethernet cable, female dupont connection wires, small 4" .1" zip ties, solder <sup>2</sup>                                                                                                                                                                                                                                                                    |
| —   | A PoE-capable switch, PoE injector, **or** USB-C power supply and USB-C cable                                                                                                                                                                                                                                                                                                |

<!-- markdownlint-enable line-length table-column-style -->

> ⚠️ **WARNING — Do NOT power the ESP32-P4-ETH via both its USB-C connector and
> a PoE powered Ethernet cable at the same time.** Powering from both
> simultaneously may damage the ESP32-P4-ETH board, POE switch, or device
> providing USB power. Either power source alone is sufficient to power the
> board, GPS module, and LCD screen.

<sup>1</sup> _(Optional)_ 3D Printable Case designed with these specific
components in mind (for more information see below)

<sup>2</sup> _(Optional)_ To cut down on wiring a PCB can be used. One designed
to work within this project's 3D printed enclosure is available in the
[`PCB`](./PCB) folder. If your interested, please review the `Readme.md` file in
that folder for more information.

### Wiring

**GPS module → ESP32-P4-ETH** _(mandatory)_

| GPS pin | ESP32-P4-ETH pin |
| ------- | ---------------- |
| GND     | GND              |
| VCC     | 3V3              |
| PPS     | GPIO 20          |
| TXD     | GPIO 21 (RX)     |
| RXD     | GPIO 22 (TX)     |

> **Important:** the above pin selections have changed in version 2.4 for better
> forward and backward compatibility between revisions of the ESP32-P4 chip.
> However, if an earlier version of the software has been working fine for you
> then you may be able to use the old pin specifications if you want to avoid
> rewiring your project. Please see the ESP32TimeServerSettings.h file for more
> information.

**LCD 2004 (HD44780 + PCF8574 I²C backpack) → ESP32-P4-ETH** _(optional)_

| LCD pin | ESP32-P4-ETH pin |
| ------- | ---------------- |
| GND     | GND              |
| VCC     | 3V3              |
| SDA     | GPIO 8           |
| SLC/SCL | GPIO 7           |

**Up time / Reset button → ESP32-P4-ETH** _(optional)_

| Button             | ESP32-P4-ETH pin |
| ------------------ | ---------------- |
| One terminal       | GND              |
| The other terminal | GPIO 3           |

Refer to the pin definitions in
[`main/ESP32TimeServerSettings.h`](./main/ESP32TimeServerSettings.h).

---

## 3D Printed Case

A downloadable 3D printable enclosure designed for use the WaveShare
ESP32-P4-ETH (with or without the POE hat) + Sparkfun GPS boards is included in
the [`3D printable case/`](./3D%20printable%20case/) folder. Both `.stl`
(print-ready) and `.f3z` (Fusion 360 editable source) files are provided so you
can tweak the design to suit your needs.

> **Note:** the 3D printable enclosure files also include the needed models for
> version 1 of this project which used the Olimex ESP32-PoE-ISO Rev. B.

---

## Software

### Development Environment

This release is built using:

- **[Visual Studio Code](https://code.visualstudio.com/)**
- **[Espressif ESP-IDF Extension for VS Code](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)**

(It has been tested to compile on 5.5.3+ and 6.0.2+)

### Dependencies

The
[`SparkFun u-blox GNSS Arduino Library v3`](https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3)
(v3.1.14) is included as a **git submodule** in
[`3rdparty/SparkFun_u-blox_GNSS_v3`](./3rdparty/SparkFun_u-blox_GNSS_v3). Since
it is an Arduino library and is not published to the ESP-IDF Component Registry,
it is manually created during Setup Step 1.

The remaining dependencies are managed automatically via the ESP-IDF Component
Manager (declared in [`main/idf_component.yml`](./main/idf_component.yml)):

- `esp-idf-lib/hd44780` — LCD driver
- `esp-idf-lib/pcf8574` — I²C LCD backpack driver
- `espressif/arduino-esp32` — Arduino compatibility layer for OTA and serial

> **Note:** The ESP-IDF Component Manager resolves these dependencies
> dynamically at build time. A `dependencies.lock` file is generated locally on
> first build but is intentionally not committed because it is git-ignored.

## Setup

### Setup Step 1 - Download

Create a folder for this project, clone this repository into it, and update the
SparkFun library within it. For example:

```cmd
c:
```

```cmd
mkdir c:\temp\ESP32TimeServerProject
```

```cmd
cd \temp\ESP32TimeserverProject
```

```cmd
git clone --recursive https://github.com/roblatour/ESP32TimeServer
```

```cmd
cd ESP32TimeServer
```

```cmd
git submodule update --init --recursive
```

### Setup Step 2 - Configuration

All user-configurable settings — GPIO pins, GPS options, LCD options, button
support, time zone, etc. — are centralized in:

```plaintext
main/ESP32TimeServerSettings.h
```

Edit this file to match your desired hardware setup before building.

> **Note:** The settings file indicates whether certain features are enabled.
> These include support for an Uptime/Restart Button, a Liquid Crystal Display,
> MQTT reporting, and Over the Ethernet Updates. Please also note, that if a
> Liquid Crystal Display is **not** connected, `LIQUID_CRYSTAL_DISPLAY_ENABLED`
> **must** be `0` (disabled) or a critical runtime error will occur.

### Setup Step 3 - Build

The ESP32-P4 has different revisions; the build steps are detailed below for
each.

```cmd
del sdkconfig
```

<!-- markdownlint-disable MD013 -->

- For current ESP32-P4 modules at revision 3.1 and above:

  ```cmd
  idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;config/esp32p4_rev_v3_1.defaults" set-target esp32p4 build
  ```

- For ESP32-P4 modules at revision 3.0:

  ```cmd
  idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;config/esp32p4_rev_v3_0.defaults" set-target esp32p4 build
  ```

- For older ESP32-P4 modules with revisions prior to version 3.0:

  ```cmd
  idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;config/esp32p4_rev_pre_v3.defaults" set-target esp32p4 build
  ```

```cmd
call C:\esp\v5.5.3\esp-idf\export.bat
```

<!-- markdownlint-enable MD013 -->

### Setup Step 4 - Flash

```cmd
idf.py -p COMx flash monitor
```

> Note: Replace `COMx` with your actual serial port. For example: COM6
>
> **Important:** Do not use `--force` to flash an image to an earlier revision
> ESP32-P4.

### Over The Ethernet (OTE) Updates

Following the initial flash, OTE updates can be performed if
`OTE_UPDATES_ENABLED` was set to `1` (Enabled) for the initial build and flash.

A VS Code task **"ESP-IDF: OTE Upload over Ethernet"** is included in
`.vscode/tasks.json`. It builds the firmware and deploys it to the device over
Ethernet using `espota.py`, so no USB cable is needed after the first flash.
Alternatively, use the terminal command in
[`UsefulPowerShellCommands.md`](./misc/UsefullPowerShellCommands.md) in the
`misc` folder.

---

## Setting up your Network / Systems to make use of the ESP32 Time Server

Please see [`Setup.md`](Setup.md)

---

## Validating Your Time Server

Once running, and with your Network and System changes setup, you can validate
accuracy at [https://time.is](https://time.is).

You're also welcome to use this open source software (also developed by me) to
stress test your time server:

<https://github.com/roblatour/TimeServerStressTest>

---

## License

This project is released under the **MIT License** — see the [LICENSE](LICENSE)
file for details.

---

## Supporting This Project

To help support this project, or to just say thanks, you're welcome to 'buy me a
coffee'.

[<img alt="buy me a coffee" width="200px" src="https://cdn.buymeacoffee.com/buttons/v2/default-blue.png" />](https://www.buymeacoffee.com/roblatour)

---

Copyright © 2026 Rob Latour

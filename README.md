# ESP32 NTP Stratum 1 Time Server (version 2.9)

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

## What's New in [Version 2](https://github.com/roblatour/ESP32TimeServer/releases)

- **New microcontroller board** — built around the
  [WaveShare ESP32-P4-ETH](https://www.waveshare.com/esp32-p4-eth.htm) instead
  of the Olimex ESP32-PoE-ISO. The ESP32-P4 is more modern and faster than the
  ESP32-WROOM-32/32E used in v1.

  Also (as of version 2.9) the [ESP32-P4-WIFI6-POE-ETH](https://www.waveshare.com/esp32-p4-wifi6-poe-eth.htm?sku=32832) 
  is supported. The WaveShare ESP32-P4-WIFI6-POE-ETH uses an ESP32-P4 v3.2 chip, 
  while the WaveShare ESP32-P4-ETH uses a ESP32-P4 v1.3 chip. Although the 1.3 
  chip works very well for this project it is not recommended by Espressif for
  new designs. The v3.2 chip works even better for this project, 
  but in ways unlikely to be a differentiator for most homelabs.   
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
- **Built with ESP-IDF** — rewritten from the ground up in C++ on ESP-IDF.
- **NVS-backed GPS state** — the GPS module identity and baud rate are persisted
  across reboots so following the initial setup startup speed is quick.
- **Broader GNSS module support** — still works great with the SparkFun MAX-M10S
  (recommended), but now also supports lower-cost, generic GNSS modules, even
  those (although not recommended) that do not expose a PPS pin.
- **Ability to set a custom MAC Address** — (version 2.2) allows the use of the
  default ESP32-P4's MAC address or for a custom MAC address to be set.
- **Ability to set a Static IP Address** — (version 2.3) allows the use of a
  DHCP assigned or static IP address.
- **Handles greater throughput and number of concurrent requests** - (progressively 
  through versions 2.4, 2.7, 2.8, and 2.9).  
- **IPv6 support** - (version 2.5) IPv6 support has been added.
- **MQTT publishing** — (version 2.5) optional MQTT publishing of time server
  stats is now available. For more information see
  [misc/esp32timeserver_json_doc.md](misc/esp32timeserver_json_doc.md)
- **Improved GNSS Satellite lock and PPS discipline tracking and recovery** -
  (version 2.5) with returned results being tagged as Stratum 16 (undefined)
  until a lost lock and/or failed PPS discipline is recovered.
- **TF card support** — (version 2.6) enabling queueing of vastly greater
  amounts of MQTT reporting data should broker communications be lost
- **Home Assistant** - (version 2.6) added Home Assistant entity and 
  dashboard setup instructions [here](https://github.com/roblatour/ESP32TimeServer/blob/main/HomeAssistant/README.md).
- **Improved accuracy** - (version 2.7) greater accuracy setting the 
  precise time every second.
- **Improved throughput** - (versions 2.7 & 2.8) increased maximum 
  requests per second.
- **Support for RFC 9769-compatible interleaved responses** - (version 2.8)
  for NTPv4 requests over IPv4 and IPv6.
- **Hardware time stamping** (version 2.8) added for both NTPv3 and NTPv4 
  requests over IPv4 and (version 2.9) IPv6. This drastically reduces jitter
  (the variation in successive clock offset measurements) between a client
  and server using RFC 9769-compliant NTP requests.
- **Testing instructions, tools, and links added** - (versions 2.8) added
  instructions, tools, and links for determining / testing: jitter,
  drift, RFC 9769 compliance, memory, and server performance under stress.  
- **(optional) Startup Health Check** - (version 2.9) allowing the program 
  to selftest its core functionality at startup.
- **Experimental support for non UBlox compliant GNSS receivers** - (version 
  2.9) working with the [GT-U16](https://www.aliexpress.com/item/1005008288311771.html)
  which has better reception than some other receivers at the same, or higher, price points
  (see release notes for more information).
- **WaveShare's ESP32-P4-WIFI6-POE-ETH (ESP32-P4 v3.2 chip)** (version 2.9) fully
  tested and working exceptionally well.

> The source code for **Version 1** (Arduino / PlatformIO) remains available at:
> [https://github.com/roblatour/ESP32TimeServer/releases/tag/v1.0.0.0](https://github.com/roblatour/ESP32TimeServer/releases/tag/v1.0.0.0)

---

## Hardware



| Qty | Item                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| --- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | WaveShare [ESP32-P4-ETH](https://www.waveshare.com/esp32-p4-eth.htm?sku=32086) or [ESP32-P4-WIFI6-ETH](https://www.waveshare.com/esp32-p4-wifi6-poe-eth.htm?sku=32832) development board (with or without optional PoE) <sup>1 2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 1   | GPS/GNSS module [Recommended: SparkFun GNSS Receiver Breakout - MAX-M10S (Qwiic)](https://www.sparkfun.com/sparkfun-gnss-receiver-breakout-max-m10s-qwiic.html) <sup>2</sup>. Additionally, experimental support for the [GT U16](https://www.aliexpress.com/item/1005008288311771.html) has been added in version 2.9 - this receiver offers superior reception when coupled with the right antenna (see notes below).  Additionally, while some lower-cost generic modules UBlox compliant ([AliExpress - NEO8M](https://www.aliexpress.com/item/1005003721844881.html)) receivers are supported, those without a PPS pin are no longer supported. |
| 1   | GPS/GNSS antenna with SMA connector ([SparkFun GPS/GNSS Magnetic Mount Antenna - 3m (SMA)](https://www.sparkfun.com/products/14986)) <sup>2</sup>. Alternatively the GT U16 with an L1/L5 antenna offers superior reception.  However, in my testing I used the [3M SMA W70C](https://www.aliexpress.com/item/1005008421771962.html?spm=a2g0o.order_list.order_list_main.11.7a7f1802GCXbvZ) L1/L2/L5 antenna an [IPEX to SMA adapter](https://www.aliexpress.com/item/1005009047225776.html) and got very good results (I have the antenna bundled with the GT U16 (link above) on order and when I get it and test it I will update this page).                                                            |
| 1   | _(Optional)_ TF Card, formatted as FAT32, if you're using MQTT and want more than four messages queued should broker communications be down) <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 1   | _(Optional)_ 4×20 I²C LCD display with HD44780 controller with PCF8574 I²C backpack ([AliExpress](https://www.aliexpress.com/item/1005006829045609)) <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 1   | _(Optional)_ Momentary push button for displaying up time and triggering a reset ([AliExpress](https://www.aliexpress.com/item/1005004066257419.html)) <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 1   | _(Optional)_ USB C extension cable (with right angle end) ([AliExpress](https://www.aliexpress.com/item/1005006584965187.html)) <sup>2</sup> + two M3*8                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| —   | Miscellaneous: Ethernet cable, female dupont connection wires, small 4" .1" zip ties, solder <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| —   | A PoE-capable switch, PoE injector, **or** USB-C power supply and USB-C cable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |



> ⚠️ **WARNING — Do NOT power the ESP32-P4-ETH via both its USB-C connector and
> a PoE powered Ethernet cable at the same time.** Powering from both
> simultaneously may damage the ESP32-P4-ETH board, POE switch, or device
> providing USB power. Either power source alone is sufficient to power the
> board, GPS module, and LCD screen.

<sup>1</sup> See notes in the 'What's New in Version 2' section above.

<sup>2</sup> _(Optional)_ there are 3D Printable Case designed for use
with ESP32-P4_ETH and these specific components (for more information see below).

_(Also Optional)_ To cut down on wiring a PCB can be used. One designed
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

> **Important:** 
> 
> Some GNSS receivers and breakout boards provide a 3.3v PPS (Pulse-Per-Second) output pin
while others provide a 5v PPS output pin.
>
>  1. Only connect a 3.3v PPS pin to an ESP32 GPIO pin, never connect a 5v PPS pin to an ESP32 GPIO pin
>
>  2. For the wire between the ESP32-P4's GPIO pin and the GNSS Receiver's PPS pin
use as short and as well connect wire as possible; a standard copper Dupont wire
will be fine assuming its gauge is between 24 AWG and 28 AWG
>
>  3. The GNSS GND must be connected to the ESP32 GND
> 
> 
> Also of note, the above PPS, TX, and RX pin selections were changed in version 2.4 for better
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

> **Note:** the 3D printable enclosure files also includes the needed models for
> version 1 (used an Olimex ESP32-PoE-ISO Rev. B dev board).

---

## Software

### Build and development Environment


**To build and flash ESP32TimeServer** you will need **Espressif's ESP-IDF v6.1** or above. 
If you don't already have this software installed, just follow the install instructions
directly below:

https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html

which include these links to get it:

[Installation of ESP-IDF and Tools on Windows](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/windows-setup.html)

[Installation of ESP-IDF and Tools on Linux](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-setup.html)

[Installation of ESP-IDF and Tools on macOS](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/macos-setup.html)


If you want to further develop the project, Visual Studio Code with the ESP-IDF extension is recommended.
However, they are not needed to build and flash the project - the instructions for that are below.

Both Visual Studio Code (Community Edition) and the ESP-IDF extension are free to download and use.
Here is where you can find more information about them and get your copies:

- **[Visual Studio Code](https://code.visualstudio.com/)**
- **[Espressif ESP-IDF Extension for VS Code](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)**


### Dependencies

The
[`SparkFun u-blox GNSS Arduino Library v3`](https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3)
(v3.1.14) is included as a **git submodule** in
[`3rdparty/SparkFun_u-blox_GNSS_v3`](./3rdparty/SparkFun_u-blox_GNSS_v3). The
local component wrapper in `components/SparkFun_u-blox_GNSS_v3` builds it from
that checked-out submodule.

The remaining dependencies are managed automatically via the ESP-IDF Component
Manager (declared in [`main/idf_component.yml`](./main/idf_component.yml)):

- `esp-idf-lib/hd44780` — LCD driver
- `esp-idf-lib/pcf8574` — I²C LCD backpack driver
- `espressif/arduino-esp32` — Arduino compatibility layer for OTA and serial
- `espressif/mqtt` — MQTT client used for optional reporting

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

Open a Command Prompt, load the ESP-IDF 6.1 environment, and remove any
existing generated configuration before building. The first build downloads the
Component Manager dependencies into `managed_components`; this generated folder
is not part of the repository.

```cmd
call C:\esp\v6.1\esp-idf\export.bat
```
```cmd
if exist sdkconfig del sdkconfig
```

The ESP32-P4 has different revisions; use the command matching your module.



- For older ESP32-P4 modules with revisions prior to version 3.0 (**including the
  Waveshare ESP32-P4-ETH**):

  ```cmd
  idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;config/esp32p4_rev_pre_v3.defaults" set-target esp32p4 build
  ```

- For ESP32-P4 modules at revision 3.0:

  ```cmd
  idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;config/esp32p4_rev_v3_0.defaults" set-target esp32p4 build
  ```
- For ESP32-P4 modules at revision 3.1 and above (**including the 
  WaveShare ESP32-P4-WIFI6-POE-ETH**):

  ```cmd
  idf.py -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;config/esp32p4_rev_v3_1.defaults" set-target esp32p4 build
  ```




### Setup Step 4 - Flash

In the same Command Prompt where ESP-IDF 6.1 was loaded:

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

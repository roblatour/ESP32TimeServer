# ESP32 NTP and PTP Stratum 1 Time Server (version 4.0)

An ESP32 NTP and PTP Stratum 1 Time Server for home networks

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

---

## Overview

This project was born with a simple goal: create a low-budget, **Stratum 1
Network Time Protocol (NTP) time server** with no need for periodic
Linux OS updates. Rather just set it up and let it do its job.  Recently
the project has been enhanced to additionally support 
**L2 and L3 UDP Multicast Precision Time Protocol (PTP)** as well.

![ESP32 Time Server](./PCB/photo01.jpg)

---
## Key Features

- **Stratum 1 time source** — ESP32TimeServer uses a **GNSS (North America & Europe) receiver** as its time source, 
making it a true Stratum 1 server — meaning it gets its time directly from satellites
rather than from another server. Every device on your home network can then
synchronize to it for highly accurate local time.
- **NTP and PTP in one server** — the program relays time, individually or in any combination, using 
Network Time, Precision Time Layer 2 (L2), and/or Precision Time Layer 3 (L3) UDP Multicast protocols
- **Sub-millisecond accuracy** — Pulse per second hardware events are used to discipline the time reference 
- **Hardware timestamping** - hardware time stamping is used to greatly improve precision reporting across a network
- **Authentication support** — ESP32TimeServer supports NTPv4 symmetric-key and PTP L2 authentication - as 
well as mixed modes (i.e. enforced or not)
- **Modern hardware** — fully vetted on the ESP32-S31 Function CoreBoard-1 (preview) with Gigabit Ethernet, and on the 
Waveshare ESP32-P4 Ethernet board series with 100 Megabit Ethernet
- **Flexible GNSS module support** — it works best with the ublox MAX-M10S GNSS receiver, but also supports the non-ublox
 GT-U16 GNSS receiver
- **Easy network setup** — the program supports a DHCP-assigned or static IP address, and the default or 
customized MAC address
- **Monitoring and smart-home integration** — the program provides for MQTT reporting and the queuing of messages
when the broker is down, with [Home Assistant setup instructions](./HomeAssistant/README.md) also available
- **Over-the-Ethernet firmware updates** — program updates may be managed over your network so the hardware 
can be easily deployed in hard to get at places
- **Optional LCD, RGB LED, and button** — a 4×20 I²C LCD, WS2812/KY-016 RGB LED, and up time / reset button can, 
but don't have to be, included in the hardware setup (in any combination)
- **Extensive testing tools** - health check, firmware regression, memory leak, drift, jitter, server alignment,
and RFC9769 compliance testing write-ups / tools are included  
- **Companion tools** for configuring a Windows w32time client for NTP or PTP L3, and for Over the Ethernet updates
are included 
- **Companion projects** — companion Github projects include: an [ESP32-P4 hardware based NTP and PTP testing tool](https://github.com/roblatour/ESP32TimeServerTester), example ESP32-P4/S31 PTP Followers for
[PTP L2](https://github.com/roblatour/ESP32PTPL2Follower) and 
[PTP L3 UDP Multicast](https://github.com/roblatour/ESP32PTPL3Follower), and a
[Windows based NTP stress testing tool](https://github.com/roblatour/TimeServerStressTest)
- **Open and maintainable** — all code for this project and the companion projects above are licensed as MIT.  The source for this project is written in C++ on ESP-IDF using FreeRTOS tasks (with no Arduino dependencies) and fully included in this repository 
- **3D-printable files** - for a variety of cases configurations also included in this repository and/or linked 
to at [printables.com](https://www.printables.com/@RobLatour_54648/models)
  

([Detailed release history](https://github.com/roblatour/ESP32TimeServer/releases))

---

## Hardware



| Qty | Item                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| --- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | WaveShare [ESP32-P4-ETH](https://www.waveshare.com/esp32-p4-eth.htm?sku=32086) or [ESP32-P4-WIFI6-ETH](https://www.waveshare.com/esp32-p4-wifi6-poe-eth.htm?sku=32832) development board (with or without optional PoE) <sup>1 2</sup>, or the experimental [ESP32-S31 Function CoreBoard-1](https://www.aliexpress.com/item/1005013174212520.html).                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 1   | GPS/GNSS module Recommened: [SparkFun GNSS Receiver Breakout - MAX-M10S (Qwiic)](https://www.sparkfun.com/sparkfun-gnss-receiver-breakout-max-m10s-qwiic.html?affiliate_code=dW1O7LlPwh&referring_service=link) <sup>2</sup>.  Additionally, one project user reported using this [MAX-M10N board](https://canada.newark.com/mikroelektronika/mikroe-6786/add-on-brd-microbus-compatible/dp/09AN2199) just fine.  Beyond that, support for the [GT U16](https://www.aliexpress.com/item/1005008288311771.html) has been added in version 2.9 - this receiver offers superior reception when coupled with the right antenna (see notes below).  Additionally, while some lower-cost generic modules UBlox compliant receivers are supported, those without a PPS pin are not. |
| 1   | GPS/GNSS antenna with SMA connector ([SparkFun GPS/GNSS Magnetic Mount Antenna - 3m (SMA)](https://www.sparkfun.com/products/14986?affiliate_code=dW1O7LlPwh&referring_service=link)) <sup>2</sup>. Alternatively the GT U16 with an L1/L5 antenna offers superior reception.  However, in my testing I used the [3M SMA W70C](https://www.aliexpress.com/item/1005008421771962.html?spm=a2g0o.order_list.order_list_main.11.7a7f1802GCXbvZ) L1/L2/L5 antenna an [IPEX to SMA adapter](https://s.click.aliexpress.com/e/_c31hthKJ) and got very good results (I have the antenna bundled with the GT U16 (link above) on order and when I get it and test it I will update this page).                                                                                       |
| 1   | _(Optional)_ RGB LED: KY-016 ([AliExpress](https://s.click.aliexpress.com/e/_c3Jceyex)) or WS2812 ([AliExpress](https://s.click.aliexpress.com/e/_c3rjPVub)) with a 470&Omega; resistor and optional lens cap ([AliExpress](https://www.aliexpress.com/item/1005003854908953.html))                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 1   | _(Optional)_ 4×20 I²C LCD display with HD44780 controller with PCF8574 I²C backpack ([AliExpress](https://www.aliexpress.com/item/1005006829045609.html)) <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 1   | _(Optional)_ Momentary push button for displaying up time and triggering a reset ([AliExpress](https://s.click.aliexpress.com/e/_c4UbVuTN)) <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 1   | _(Optional)_ USB C extension cable (with right angle end) ([AliExpress](https://s.click.aliexpress.com/e/_c3C5c6Kx)) <sup>2</sup> + two M3*8                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 1   | _(Optional)_ TF Card Reader (only needed for the ESP32-S31 Functional Coreboard-1; the WaveShare ESP32-P4-ETH series boards have a built in TF Card Reader) ([AliExpress](https://s.click.aliexpress.com/e/_c34w6E9D))                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 1   | _(Optional)_ TF Card, formatted as FAT32, if you're using MQTT and want a larger queue, kept through power loss, should broker communications be down                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| —   | Miscellaneous: Ethernet cable, [female dupont connection wires](https://s.click.aliexpress.com/e/_c3UwFX2R), small 4" .1" zip ties, solder <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| —   | A PoE-capable switch, PoE injector, **or** USB-C power supply and USB-C cable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |



> ⚠️ **WARNING — Do NOT power the ESP32-P4-POE-ETH or the ESP32-P4-WIFI6-POE-ETH
> via both its USB-C connector and a PoE powered Ethernet cable at the same time.**
> Powering from both simultaneously may damage the ESP32-P4-ETH board, PoE switch,
> or device providing USB power. Either power-supply alone is generally sufficient to
> power the board, GNSS module, LCD screen, etc.  However, if after flashing the 
> firmware and a power recycle the LCD screen blinks on and off for several minutes
> before coming online its a good indication a power-supply (USB charger or 
> PoE injector) issue. In this case try a different USB charger or PoE injector, 
> being careful to always use only one power-supply at a time.

<sup>1</sup> See notes in the 'What's New in Version 2' section above.

<sup>2</sup> _(Optional)_ there are 3D Printable Case designed for use
with ESP32-P4_ETH and these specific components (for more information see below).

_(Also Optional)_ To cut down on wiring a PCB can be used. One designed
to work within this project's 3D printed enclosure is available in the
[`PCB`](./PCB) folder. If your interested, please review the `Readme.md` file in
that folder for more information.

### Wiring

#### GPS module → ESP32-P4-ETH _(mandatory)_

| GPS pin | ESP32-P4-ETH pin |
| ------- | ---------------- |
| GND     | GND              |
| VCC     | 3V3              |
| PPS     | GPIO 20          |
| TXD     | GPIO 21 (RX)     |
| RXD     | GPIO 22 (TX)     |

**Pro Tip:** (Optionally) Adding a capacitor between the GNSS module's Vcc and GND power pins,
placed as close as possible to those pins, can improve GNSS power supply stability. Use a
100nf (aka 0.1μf, aka 104) ceramic capacitor, rated at least 50v, 
such as [this one](https://www.aliexpress.com/item/1005004469852680.html).

**Important:**
 
Some GNSS receivers and breakout boards provide a 3.3v PPS (Pulse-Per-Second) output pin
while others provide a 5v PPS output pin.

1. Only connect a 3.3v PPS pin to an ESP32 GPIO pin, never connect a 5v PPS pin to an ESP32 GPIO pin

2. For the wire between the ESP32-P4's GPIO pin and the GNSS Receiver's PPS pin
use as short and as well connect wire as possible; a standard copper Dupont wire
will be fine assuming its gauge is between 24 AWG and 28 AWG

3. The GNSS GND must be connected to the ESP32 GND
 
 
Also of note, the above PPS, TX, and RX pin selections were changed in version 2.4 for better
forward and backward compatibility between revisions of the ESP32-P4 chip.
However, if an earlier version of the software has been working fine for you
then you may be able to use the old pin specifications if you want to avoid
rewiring your project. Please see the ESP32TimeServerSettings.h file for more
information.

#### Up time / Reset button → ESP32-P4-ETH _(optional)_

| Button             | ESP32-P4-ETH pin |
| ------------------ | ---------------- |
| One terminal       | GND              |
| The other terminal | GPIO 3           |


#### WS2812 RGB LED _(optional)_

| LCD pin |                     | ESP32-P4-ETH pin |
| ------- | ------------------- | ---------------- |
| GND     |                     | GND              |
| DI      | 470&Omega; resistor | GPIO 4           |
| 5v      |                     | V5 / VBUS        |
> **Important** with the WS2812 a 470&Omega; resistor is require between the DI and GPIO 4 pin

#### KY-016 RGB LED _(optional)_

| LCD pin | ESP32-P4-ETH pin |
| ------- | ---------------- |
| GND     | GND              |
| BLUE    | GPIO 4           |
| GREEN   | GPIO 5           |
| RED     | GPIO 6           |

#### LCD 2004 (HD44780 + PCF8574 I²C backpack) → ESP32-P4-ETH _(optional)_

| LCD pin | ESP32-P4-ETH pin |
| ------- | ---------------- |
| GND     | GND              |
| VCC     | 3V3              |
| SLC/SCL | GPIO 7           |
| SDA     | GPIO 8           |

Refer to the pin definitions in
[`main/ESP32TimeServerSettings.h`](./main/ESP32TimeServerSettings.h).

#### ESP32-S31 Function CoreBoard 1 wiring

The following are GPIO numbers and the board's corresponding J2 header pin numbers,
**Note board silk screen numbering (J2 pins) do not correspond with the GPIO pin numbering**:

| Peripheral connection              | S31 GPIO     | J2 pin       |
| ---------------------------------- | ------------ | ------------ |
| GNSS receiver RX (MCU TX)          | 22           | D2           |
| GNSS receiver TX (MCU RX)          | 21           | D1           |
| GNSS PPS input                     | 20           | D0           |
| LCD SDA                            | 38           | 22           |
| LCD SCL                            | 39           | 21           |
| Uptime/reset button                | 40           | 19           |
| Onboard addressable LED            | 60           | 6            |
| Optional KY-016 blue / green / red | 46 / 47 / 48 | 16 / 13 / 14 |

Use a common ground and **3.3 V** GNSS UART and PPS signals. Keep the PPS wire
short. 
**Only ever connect the LCD to a 3.3v power pin - never 5v.**  

#### Optional external SPI TF card reader (S31)

Use a reader designed for **3.3 V power and 3.3 V SPI signals**, with a FAT32
microSD/TF card. This add-on provides the same persistent MQTT backlog as the
P4's onboard socket; it is not required for time serving. The GNSS, LCD, button,
LED options, Ethernet, and internal PPS loopback retain their existing pins.

| Reader pin | S31 connection |
| ---------- | -------------- |
| 3.3V       | 3V3 power      |
| CS         | GPIO0          |
| MOSI       | GPIO1          |
| CLK        | GPIO2          |
| MISO/MOSO  | GPIO3          |
| GND        | GND            |

**Only ever connect the Card Reader to a 3.3v power pin - never 5v.**

**Power sharing:** If you are designing a setup with an LCD and external
TF card reader and only one 3V3 header pin is available, after using 
the first for the GNSS receiver, then you can used a connecting 
wire from the remaining 3v3 header pin that splits and goes to the 
power in for both the LCD and TF card reader.

`MOSO` is sometimes a misprint for **MISO** (reader output, MCU input);
confirm this in the reader's documentation. 

Keep the connecting wires to the TF Card reader short.

Insert the card before power-on; mounting is attempted once at startup.
An absent, non-FAT32, or unusable card falls back to the existing RAM queue
without formatting the card.

---

## 3D Printed Case

Various downloadable 3D printable enclosures, designed for use with either the 
WaveShare ESP32-P4 series of boards (with our without a POE hat) or the Espressif
ESP32-S31 Functional CoreBoard-1, are included in the
[`3D printable case/`](./3D%20printable%20case/) folder.  Different models
are designed to work with different hardware combinations. Additional user
contributions are welcome.

> **Note:** the 3D printable enclosure files also includes the needed models for
> version 1 (used an Olimex ESP32-PoE-ISO Rev. B dev board).

---

## Software

### Build and development environment


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

#### **For Linux users:**
A kind user of this project has shared more detailed environment setup, build and flash instructions specific to Linux (Debian 13) - these can be found [here](./misc/Linux_specific_setup_instructions.md)


### Dependencies

Dependencies are resolved by the ESP-IDF Component Manager from
[`main/idf_component.yml`](./main/idf_component.yml):

- P4: `esp-idf-lib/hd44780` and `esp-idf-lib/pcf8574`.
- S31: `jef-sure/hd44780`, which supports the existing PCF8574 wiring.
- Both: `espressif/mqtt`, `espressif/mdns`, and `espressif/led_strip`.

> **Note:** The ESP-IDF Component Manager resolves these dependencies
> dynamically at build time. Each build directory has its own generated
> `dependencies.lock`; it is not a shared cross-target lock.

## Setup

### Setup Step 1 - Download

Create a folder for this project and clone this repository into it. For example:

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
git clone https://github.com/roblatour/ESP32TimeServer
```

```cmd
cd ESP32TimeServer
```

### Setup Step 2 - Configuration

All user-configurable settings — GPIO pins, GPS options, LCD options, button
support, time zone, etc. — are centralized in:

```plaintext
main/ESP32TimeServerSettings.h
```

Edit this file to match your desired hardware setup before building. The
ESP-IDF target selected by the build profile chooses the P4 or S31 GPIO branch
automatically; no separate board-name string selects the compiler target.
For a UART console, `serialMonitorSpeed` sets its baud rate. USB Serial/JTAG
does not use this value; UART1 is reserved for the GNSS receiver and cannot be
selected as the console.

> **Note:** The settings file indicates whether certain features are enabled.
> These include support for an Uptime/Restart Button, a Liquid Crystal Display,
> MQTT reporting, and Over the Ethernet Updates. Please also note, that if a
> Liquid Crystal Display is **not** connected, `LIQUID_CRYSTAL_DISPLAY_ENABLED`
> **must** be `0` (disabled) or a critical runtime error will occur.

### Setup Step 3 - Build

Open a Command Prompt at the repository root and load the ESP-IDF 6.1
environment. The first build downloads the Component Manager dependencies into
`managed_components`.

```cmd
call C:\esp\v6.1\esp-idf\export.bat
```

Connect the board, then identify the chip model and revision:

```cmd
python -m esptool -p COMx chip-id
```

Replace `COMx` with the actual port. Select the model first; an S31 revision
number must not be interpreted as a P4 revision. For ESP32-P4, choose the
matching profile:v

| Chip revision                  | Profile  | Build directory |
| ------------------------------ | -------- | --------------- |
| P4 before v3.0                 | `pre_v3` | `build_pre_v3`  |
| P4 v3.0                        | `v3_0`   | `build_v3_0`    |
| P4 v3.1 or later               | `v3_1`   | `build_v3_1`    |
| ESP32-S31 Function CoreBoard 1 | `s31`    | `build_s31`     |

For example:

```cmd
python .\tools\build.py --profile v3_1
```

Use `--profile v3_0` or `--profile pre_v3` for the other P4 profiles, or
`--profile s31` for the S31. The helper supplies `--preview` for S31. Each
profile keeps its generated `sdkconfig` inside its own build directory. The
helper does not flash, delete the existing root `sdkconfig`, or run
`set-target` on incremental builds. It rejects a build directory configured
for a different target or revision. Use `--build-dir .\my_build` to choose a
different workspace-local directory, `--define NAME=VALUE` to add a CMake
definition, and `--reconfigure` when reconfiguration is intentional.

For an equivalent initial raw v3.1 build, use a fresh directory:

```cmd
idf.py -B .\build_v3_1 ^
  -D IDF_TARGET=esp32p4 ^
  -D SDKCONFIG=.\build_v3_1\sdkconfig ^
  -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;config\esp32p4_rev_v3_1.defaults" ^
  build
```

For the other revisions, substitute their profile directory, generated
`sdkconfig`, and matching file under `config`. ESP-IDF also applies
`sdkconfig.defaults.esp32p4`, including the P4's 32 MB flash. The equivalent
fresh S31 build is:

```cmd
idf.py --preview -B .\build_s31 ^
  -D IDF_TARGET=esp32s31 ^
  -D SDKCONFIG=.\build_s31\sdkconfig ^
  -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;config\esp32s31_function_coreboard_1.defaults" ^
  build
```

### Setup Step 4 - Flash

In the same Command Prompt where ESP-IDF 6.1 was loaded:

```cmd
idf.py -B .\build_v3_1 -p COMx flash monitor
```

For S31, use its matching output directory and preview flag:

```cmd
idf.py --preview -B .\build_s31 -p COMx flash monitor
```

> Note: Replace `COMx` with your actual serial port. For example: COM6
>
> Use the directory for the profile you built. Do not use `--force` to flash an
> image to the wrong chip or an earlier ESP32-P4 revision.

### Over The Ethernet (OTE) Updates

Following the initial flash, OTE updates can be performed if
`OTE_UPDATES_ENABLED` was set to `1` (Enabled) for the initial build and flash.
For more information, please see [`UsefulPowerShellCommands.md`](./misc/UsefullPowerShellCommands.md).
OTE checks the image chip and revision before starting the update. Verify
cross-target and incompatible-revision rejection on the actual device; never
bypass those checks.

---

## Setting up your network / systems to make use of ESP32 Time Server

To make use of ESP32TimeServer on your network /systems please see [`here`](misc/setup.md)

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

# ESP32 NTP and PTP Stratum 1 Time Server (version 4.0)

An ESP32 NTP and PTP Stratum 1 Time Server for home networks

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

---

## Overview

This project was born with a simple goal: create a low-budget, **Stratum 1
Network Time Protocol (NTP) time server** with no need for periodic
Linux OS updates. Rather just set it up and let it do its job.

More recently Precision Time Protocol (PTP) support has been added.

ESP32TimeServer can serve NTP, PTP Layer 2 PTP, and PTP Layer 3
UDP, individually or in any combination.

![ESP32 Time Server](./PCB/photo01.jpg)

ESP32TimeServer uses a **GNSS (North America & Europe) receiver** as its time source, 
making it a true Stratum 1 server — meaning it gets its time directly from satellites
rather than from another server. Every device on your home network can then
synchronize to it for highly accurate local time.

ESP32TimeServer supports Waveshare ESP32-P4 Ethernet dev boards and the 
ESP32-S31 Function CoreBoard-1.

---

## What's New

**Version 4** brings the following changes:

- Support for the ESP32-S31 (preview) chip which provides for Gigabit Ethernet.
  Performance of the ESP32-S31 (preview) chip surpasses  the ESP32-P4 rel 1.3 chip,
  but is below the ESP32-P4 rel 3.2 chip. 
- Added Precision Time Protocol (PTP) support. The server can now operate
  as an NTP, PTP Layer 2 (L2), PTP Layer 3 (L3) UDP, or any and all 
  combinations simultaneously.
- Both the NTP and PTP L2 protocols support operation with or without 
  authentication. NTP and PTP L2 have independent mixed/authenticated-only 
  policy settings, each defaulting to mixed mode. 
  Additional notes: 
  1. PTP servers sync time with their clients with a degree of precision measured in 
  Nanoseconds (ns) as opposed to a Network Time Protocol (NTP) server with a degree of
  precision measured in Microseconds (µs) (1 second = 1 million µs = 1 billion ns).
  2. PTP servers and their followers, will not work in virtual environments nor 
  across router (OPNSense/pfSense) interfaces - they do however work
  within the same interface. 
  3. Linux chrony implementations provides for NTP time and or 
  PTP L2 time with a fall back to NTP time.  Windows implementations
  provide for either NTP time or PTP L3 UDP time but with not
  PTP L3 UDP with NTP fallback.
- added/updated the following companion programs:

  (added) ESP32PTPL2Follower ( https://github.com/roblatour/ESP32PTPL2Follower )
  an example PTP L2 Follower for the ESP32-P4 and ESP32-S31 platforms
  
  
  (added) ESP32PTPL3Follower ( https://github.com/roblatour/ESP32PTPL3Follower )
  an example PTP L3 UDP Follower for the ESP32-P4 and ESP32-S31 platforms
  
  (added) tools\configTimeSettings.ps1 a python script to automate the setup of native
  Windows w32time client for use with ESP32TimeServer in either NTP or PTP L3 UDP modes
    
  (updated) ESP32TimeServerTester ( https://github.com/roblatour/ESP32TimeServerTester )
  to test PTP L2 and L3 UDP servers 
  
  
- removed use of the Arduino Sparkfun GNSS library (adapting its needed code for use within 
  ESP-IDF) as well as the last vestiges of Arduino code.  This was done to support the
  ESP32-S31 chip which while supported in the ESP-IDF environment is 
  not supported in the Arduino environment (although may be in the future)
- added automated regression testing 
  for more information please see [here](./testing/README.md)
- updates to MQTT messaging for new PTP reporting and to provide for, in most cases, more saved 
  messages in memory when a TF card is not available
- updates to Home Assistant configurations for new PTP reporting
- added external TF card reader support for the ESP32-S31 as the Espressif Functional CoreBoard - 1
  does not have a built in TF card reader
- revised leap-second processing aligned to the CGPM's decision to do away with leap-second processing in May 2027 
  for more information please see [here](./misc/leap_second_handling.md)
- added read back confirmation of Ublox receiver going in to stationary mode 
- several other updates and improvements

**Version 3** further improvements:
- **NTPv4 Symmetric-key authentication** added for Windows 
  clients using Meinberg and Linux clients using chrony. 
- **WS2182** RGB LED support added.
- **Stationary GNSS mode** — with UBLOX compliant GNSS receivers the program enables
  stationary-mode for improved time reporting. 
- updated to use Espressif's Ethernet and OTE update (TCP 3232) processes in place of 
  the Arduino Ethernet and OTE update (UDP 3232) processes
- improved accuracy when using the ESP32-P4 release 3.0 or above chip

**Version 2** was a ground-up rewrite which came with a faster, more capable board 
support and a long list of accuracy, reliability, and convenience improvements over
version 1, including:

- **Higher accuracy, lower jitter** — full use of the GPS module's PPS
  (Pulse Per Second) pin disciplines the clock for sub-millisecond accuracy,
  with hardware time stamping and RFC 9769-compatible interleaved responses
  further reducing jitter for both NTPv3 and NTPv4 requests.
- **Faster, more modern hardware** — built around the WaveShare ESP32-P4-ETH
  (or the newer ESP32-P4-WIFI6-ETH), a significant step up from the
  ESP32-WROOM-32/32E used in v1.
- **Broader GNSS module support** — still works great with the recommended
  SparkFun MAX-M10S, but also supports lower-cost generic GNSS modules and now
  has experimental support for the GT-U16, which offers better reception than
  some competing receivers at the same or higher price points.
- **Wired firmware updates** — Over-the-Ethernet updates let you push
  new firmware directly from VS Code, with on-screen progress shown on the
  LCD, so you rarely need to plug in a cable after the first flash.
- **Handles more traffic** — greatly increased throughput and concurrent
  request handling, so the server keeps up even on busy networks.
- **IPv6 support** — the server now responds to NTP requests over IPv6 as
  well as IPv4.
- **Flexible networking** — set a custom MAC address or a static IP, instead
  of always relying on the device's default address and DHCP.
- **MAC address selection** — `MACAddress` is a base-MAC override and is applied
  before Ethernet starts. For PTP master selection, use the active six-byte
  address reported as `Active Ethernet MAC` at startup; the SDK may derive a
  different Ethernet source MAC from the base address.
- **Optional LCD support** — a 4x20 LCD screen was required in v1; 
  in v2 it's entirely optional, along with an optional up time / reset button.
- **Optional KY-016 RGB LED support added** — [here](misc/status_indicators.md)
  is how that is used.
- **Smart GNSS lock tracking** — improved satellite lock and PPS discipline
  tracking, with results correctly reported as Stratum 16 (undefined) if lock
  or discipline is temporarily lost, and automatic recovery once it's
  restored.
- **Quick restarts** — GPS module identity and baud rate are saved to
  non-volatile storage, so the server starts up quickly after the initial
  setup.
- **MQTT publishing** — optionally publish time server stats to an MQTT
  broker, with TF card support for queueing far more messages if the broker
  connection drops.
- **Home Assistant integration** — ready-made entity and dashboard setup
  instructions for pulling time server stats into Home Assistant.
- **Built-in self-test** — an optional startup health check lets the server
  verify its own core functionality every time it boots.
- **Testing tools and guidance** — instructions, tools, and links for
  measuring jitter, drift, RFC 9769 compliance, memory usage, and server
  performance under stress.
- **Updated 3D printed case** — enclosure files refreshed for the new
  WaveShare ESP32-P4-ETH board.

 **Version 1** described in detail [here](https://hackaday.io/project/189309-esp32-ntp-time-server-stratum-1) was built for Arduino / PlatformIO and remains available
 [here](https://github.com/roblatour/ESP32TimeServer/releases/tag/v1.0.0.0).
>
([Detailed release history](https://github.com/roblatour/ESP32TimeServer/releases))

---

## Hardware



| Qty | Item                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| --- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1   | WaveShare [ESP32-P4-ETH](https://www.waveshare.com/esp32-p4-eth.htm?sku=32086) or [ESP32-P4-WIFI6-ETH](https://www.waveshare.com/esp32-p4-wifi6-poe-eth.htm?sku=32832) development board (with or without optional PoE) <sup>1 2</sup>, or the experimental [ESP32-S31 Function CoreBoard-1](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s31/esp32-s31-function-coreboard-1/user_guide.html).                                                                                                                                                                                                                                                                                                            |
| 1   | GPS/GNSS module [Recommended: SparkFun GNSS Receiver Breakout - MAX-M10S (Qwiic)](https://www.sparkfun.com/sparkfun-gnss-receiver-breakout-max-m10s-qwiic.html) <sup>2</sup>.  Additionally, one project user reported using this [MAX-M10N board](https://canada.newark.com/mikroelektronika/mikroe-6786/add-on-brd-microbus-compatible/dp/09AN2199) just fine.  Beyond that, support for the [GT U16](https://www.aliexpress.com/item/1005008288311771.html) has been added in version 2.9 - this receiver offers superior reception when coupled with the right antenna (see notes below).  Additionally, while some lower-cost generic modules UBlox compliant receivers are supported, those without a PPS pin are not. |
| 1   | GPS/GNSS antenna with SMA connector ([SparkFun GPS/GNSS Magnetic Mount Antenna - 3m (SMA)](https://www.sparkfun.com/products/14986)) <sup>2</sup>. Alternatively the GT U16 with an L1/L5 antenna offers superior reception.  However, in my testing I used the [3M SMA W70C](https://www.aliexpress.com/item/1005008421771962.html?spm=a2g0o.order_list.order_list_main.11.7a7f1802GCXbvZ) L1/L2/L5 antenna an [IPEX to SMA adapter](https://www.aliexpress.com/item/1005009047225776.html) and got very good results (I have the antenna bundled with the GT U16 (link above) on order and when I get it and test it I will update this page).                                                                             |
| 1   | _(Optional)_ TF Card, formatted as FAT32, if you're using MQTT and want a larger queue, kept through power loss, should broker communications be down <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 1   | _(Optional)_ RGB LED: KY-016 ([AliExpress](https://www.aliexpress.com/item/32763280158.html)) or WS2812 ([AliExpress](https://www.aliexpress.com/item/1005009109749689.html)) with a 470&Omega; resistor and optional lens cap ([AliExpress](https://www.aliexpress.com/store/625859))                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 1   | _(Optional)_ 4×20 I²C LCD display with HD44780 controller with PCF8574 I²C backpack ([AliExpress](https://www.aliexpress.com/item/1005006829045609.html)) <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 1   | _(Optional)_ Momentary push button for displaying up time and triggering a reset ([AliExpress](https://www.aliexpress.com/item/1005004066257419.html)) <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 1   | _(Optional)_ USB C extension cable (with right angle end) ([AliExpress](https://www.aliexpress.com/item/1005006584965187.html)) <sup>2</sup> + two M3*8                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| —   | Miscellaneous: Ethernet cable, female dupont connection wires, small 4" .1" zip ties, solder <sup>2</sup>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| —   | A PoE-capable switch, PoE injector, **or** USB-C power supply and USB-C cable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |



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

`MOSO` is sometimes a misprint for **MISO** (reader output, MCU input);
confirm this in the reader's documentation. 

Keep the connecting wires to the TF Card reader short.

Insert the card before power-on; mounting is attempted once at startup.
An absent, non-FAT32, or unusable card falls back to the existing RAM queue
without formatting the card.

**Power sharing:** J2 pins 35 and 36 are connections to the same 3.3 V rail,
not two independent supplies and not GPIO signal pins. The LCD and TF reader
can share one 3V3 header connection using a suitable power distribution
connector, with common ground, provided the board regulator, supply, and wiring
can handle the total load, including the GNSS and card write-current peaks.
The number of power pins does not establish available current; check the
reader/card and LCD specifications and the board's remaining current budget.
A 100 nF ceramic capacitor plus a suitable bulk capacitor (for example 10–47 uF)
near the reader can help with transients, but cannot compensate for an
undersized supply.

**Do not connect a reader's 3.3V pin to 5V, directly or through a resistor or
resistor divider.** 

---

## 3D Printed Case

A downloadable 3D printable enclosure designed for use the WaveShare
ESP32-P4-ETH (with or without the POE hat) + Sparkfun GPS boards is included in
the [`3D printable case/`](./3D%20printable%20case/) folder. Both `.stl`
(print-ready) and `.f3z` (Fusion 360 editable source) files are provided so you
can tweak the design to suit your needs.

> **Note:** the 3D printable enclosure files also includes the needed models for
> version 1 (used an Olimex ESP32-PoE-ISO Rev. B dev board).

The existing PCB and cases were designed for the Waveshare boards; they are not
claimed to fit the S31 Function CoreBoard.

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

The initial S31 build used ESP-IDF 6.1.0, `jef-sure/hd44780` 0.1.3,
`espressif/mqtt` 1.1.0, `espressif/mdns` 1.14.1, and
`espressif/led_strip` 3.1.1. Component versions are resolved independently for
each build directory and may change within the declared version constraints.

If PSRAM is enabled but boot initialization is disabled, set
`CONFIG_SPIRAM_BOOT_HW_INIT=y` and
`CONFIG_SPIRAM_PRE_CONFIGURE_MEMORY_PROTECTION=y` for delayed initialization.
The native path follows `CONFIG_SPIRAM_USE_MALLOC` even if the former Arduino
`CONFIG_ARDUINO_ISR_IRAM` option is enabled. This differs from Arduino's
initializer; the optional path has not yet been validated on hardware.

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

For a PTP-capable retrofit build, see the
[PTP settings](misc/PTP_transports.md).
These are compile-time choices, not MQTT commands. Keep your existing pins,
GNSS, addressing, secrets, display, and native OTE configuration rather than
copying the donor settings file.

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

Preserve existing customized configurations; defaults do not
override options already selected in them. 
Performance optimization must remain explicitly selected in an existing `sdkconfig`.


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

On S31, select the port for the board's USB Serial/JTAG or USB-to-UART
connection. Keep programming pins separate from the GNSS UART pins. The
application images are `build_<profile>\ESP32TimeServer.bin`; P4 and S31
images, and images with different P4 revision requirements, are not
interchangeable.

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

This is only a basic connectivity check. Native GNSS, S31 Ethernet and
accessories, NTP/PTP hardware timestamps, and timing accuracy require the
physical acceptance and load measurements listed in
[the S31 plan](./adding_s31_support.md). They remain pending for Phase 2.

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

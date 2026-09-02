# Useful Command Prompt Commands

Below, in more detail, are various Command Prompt commands that will allow you
to:

- Provide for Over The Ethernet (OTE) updates
- Test for NTP responses
- Reset the Meinberg drift file (to recreate a new baseline for the use of a
  different time server)

## Notes

In the examples below:

- The device name, `ESP32TimeServer`, is set as `DeviceName` in
  `ESP32TimeServerSettings.h`.
- `x.x.x.x` is the IP address of the ESP32-P4 running the ESP32 time server
  software.

## 1. Over The Ethernet (OTE) Updates

Here are a couple example command line commands which may be used, following a
build, to provide for an Over The Ethernet (OTE) update. The path name which
precedes `ESP32TimeServer.bin` may vary.

<!-- markdownlint-disable MD013 -->

```cmd
python tools\espota.py -i ESP32TimeServer.local -p 3232 -P 3232 -a ESP32TimeServerpw -f build-esp32p4-rev-1-3\ESP32TimeServer.bin -r -d
```

or

```cmd
python tools\espota.py -i x.x.x.x -p 3232 -P 3232 -a ESP32TimeServerpw -f build-esp32p4-rev-1-3\ESP32TimeServer.bin -r -d
```

## 2. NTP Test Request (ntpdate) - Linux or Windows (with Meinberg)

Here is a command which may be used for sending a NTP test request on Linux or
Windows (with the Meinberg Network Time Protocol Daemon service for Windows
installed and running):

```cmd
ntpdate -q ESP32TimeServer.local
```

or

```cmd
ntpdate -q x.x.x.x
```

With the output showing, for example:

<!-- markdownlint-disable MD013 -->

```text
28 Aug 10:46:21 ntpdate[51120]: Raised to realtime priority class
server 192.168.7.24, stratum 1, offset +0.006410, delay 0.04124
28 Aug 10:46:24 ntpdate[51120]: adjust time server 192.168.7.24 offset +0.006410 sec
```

<!-- markdownlint-enable MD013 -->

The `stratum 1` value indicates that the server is a Stratum 1 time source
(directly connected to a reference clock, in this case GNSS). The `offset` value
is the clock offset between the machine issuing the command and the
`ESP32TimeServer.local` device (in the case above approximately 6/1000's of a
second). The `delay` value is the round-trip network delay.

## 3. NTP Test Request (w32tm) - Windows only

If you are using Windows and running the default Windows Time service, here is a
command which may be used for sending NTP test requests (will not work if you
are not using Windows or using the Meinberg Network Time Protocol Daemon service
for Windows):

```cmd
w32tm /stripchart /computer:ESP32TimeServer.local /samples:5 /dataonly
```

or

```cmd
w32tm /stripchart /computer:x.x.x.x /samples:5 /dataonly
```

<!-- markdownlint-enable MD013 -->

With the output showing, for example:

```text
Tracking 192.168.7.24 [192.168.7.24:123].
Collecting 5 samples.
The current time is 2026-08-24 8:23:44 AM.
08:23:44, +00.0022650s
08:23:46, +00.0016820s
08:23:48, +00.0016135s
08:23:50, +00.0017035s
08:23:52, +00.0018610s
```

The first number above is the current time. The second number above is the clock
offset between the machine issuing the command and the `ESP32TimeServer.local`
device (in the case above approximately 2/1000's of a second).

## 4. NTP Test Request (sntp) - MacOS

If you are using MacOS, here is a command which may be used for sending an NTP
test request (the `sntp` utility is included with MacOS by default):

```cmd
sntp -d ESP32TimeServer.local
```

or

```cmd
sntp -d x.x.x.x
```

With the output showing, for example:

<!-- markdownlint-disable MD013 -->

```text
sntp 4.2.8p15@1.37-o
request 2026-08-24T08:24:01
rec:1715006401.000000 frac:0.000000000
refid: .GNSS.
leap: 0 stratum: 1 rootdelay: 0.000000
response from ESP32TimeServer.local: offset 0.002154, delay 0.040680
```

<!-- markdownlint-enable MD013 -->

The offset value shown above is the clock offset between the machine issuing the
command and the `ESP32TimeServer.local` device (in the case above approximately
2/1000's of a second).

## 5. Resetting the Meinberg Drift File

If you are using the
[Meinberg NTP Time Server](https://www.meinbergglobal.com/english/sw/ntp.htm)
and you want to create a new baseline as you are now using a new time server,
then from the Windows Command Prompt with Administrative Privileges you can use
these commands:

```cmd
net stop ntp
del "C:\Program Files (x86)\NTP\etc\ntp.drift"
net start ntp
```

Between 15 minutes to an hour later a new `ntp.drift` file will be created by
the service.

If you don't do this the Meinberg software will automatically recalibrate the
drift file within a few hours.

## 6. NTP Stress Testing Tool

Here is an open source Windows app, written by me, which can be used to stress
test your NTP server.

<!-- markdownlint-disable MD013 -->

[TimeServerStressTest](https://github.com/roblatour/TimeServerStressTest)

![Screenshot](https://raw.githubusercontent.com/roblatour/TimeServerStressTest/main/Misc/screenshot.jpg)

<!-- markdownlint-enable MD013 -->

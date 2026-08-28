# Useful Windows Command Prompt Commands

Below, in more detail, are Windows Command Prompt commands that will allow you
to:

- Provide for Over The Ethernet (OTE) updates
- Test for NTP responses
- Reset the Meinberg drift file (to recreate a new baseline for the use of a
  different time server)

## Notes

In the examples below:

- The device name, `ESP32TimeServer`, is set as `DeviceName` in
  `ESP32TimeServerSettings.h`.
- `xx.xx.xx.xx` is the IP address of the ESP32-P4 running the ESP32 time server
  software.

## 1. Over The Ethernet (OTE) Updates

Here are a couple example command line commands which may be used, following a
build, to provide for an Over The Ethernet (OTE) update. The path name which
precedes `ESP32TimeServer.bin` may vary.

```cmd
python tools\espota.py -i ESP32TimeServer.local -p 3232 -P 3232 -a ESP32TimeServerpw -f build-esp32p4-rev-1-3\ESP32TimeServer.bin -r -d
```

or

```cmd
python tools\espota.py -i x.x.x.x -p 3232 -P 3232 -a ESP32TimeServerpw -f build-esp32p4-rev-1-3\ESP32TimeServer.bin -r -d
```

## 2. NTP Test Request (w32tm)

Here is a command which may be used for sending a NTP test request:

```cmd
w32tm /stripchart /computer:ESP32TimeServer.local /samples:5 /dataonly
```

or

```cmd
w32tm /stripchart /computer:xx.xx.xx.xx /samples:5 /dataonly
```

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
device (in the case above approximately 4/10000's of a second).

## 3. NTP Test Request (ntpdate)

Here is an alternative command which may be used for sending a NTP test request:

```cmd
ntpdate -q 192.168.7.25
```

With the output showing, for example:

```text
24 Aug 08:20:01 ntpdate[48144]: Raised to high priority class, realtime requires Increase Scheduling Priority privilege (enabled with secpol.msc).
server 192.168.7.25, stratum 1, offset +0.003054, delay 0.04068
24 Aug 08:20:01 ntpdate[48144]: adjust time server 192.168.7.25 offset +0.003054 sec
```

## 4. Resetting the Meinberg Drift File

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

## 5. NTP Stress Testing Tool

Here is open source project Windows app, written by me, which allows you to
stress test your NTP server.

[TimeServerStressTest](https://github.com/roblatour/TimeServerStressTest)

![Screenshot](https://raw.githubusercontent.com/roblatour/TimeServerStressTest/main/Misc/screenshot.jpg)

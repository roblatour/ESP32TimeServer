# Setup and usage fo the rfc976_test.py script:

## Install Python: (if not already installed)

  Windows:
  ```cmd
     winget install --id Python.Python.3.14 -e
   ```

  Linux:
     look up the command for your distribution, but for example:

   ```bash
     sudo apt install python3
   ```

## Run the python script from the command line:

```cmd
py rfc9769_test.py --target x.x.x.x
```
( where x.x.x.x is the IP address for your esp32TimeServer, for example 192.168.1.24 )

Additional CLI options include: --timeout, --retries, --verbose

Here is a sample report run with `py rfc9769_test.py --target 192.168.1.24 --verbose`

```
==============================================================================

                     NTP RFC 9769 Protocol Validation Test

                           Testing IP: 192.168.1.24
                      Local Time: 2026-09-07 08:26:29 AM


------------------------------------------------------------------------------

[+] Running Standard NTPv3 Validation Test...
 |-- Sending Standard Request packet...
 |-- Attempt 1/3: sending packet to ('192.168.1.24', 123)
 |-- Received Payload metadata: NTPv3, Mode=4, Stratum=1, LI=0
 |-- Server Reference TS: 3997772789.000037
 |-- [PASS] Standard loopback handshake validation verified (raw bytes match).

[+] Running Interleaved NTPv3 Validation Test...
 |-- Sending Interleaved Handshake Request packet...
 |-- Attempt 1/3: sending packet to ('192.168.1.24', 123)
 |-- Server Echoed Origin TS:   3997772789.5198298 (expected prior client T4)
 |-- Server Receive TS (T2):     3997772790.5283313
 |-- Server Transmit TS (T3):    3997772789.5181813 (expected prior server T3)
 |-- [PASS] RFC 9769 interleaved timestamp validation verified.

[+] Running Standard NTPv4 Validation Test...
 |-- Sending Standard Request packet...
 |-- Attempt 1/3: sending packet to ('192.168.1.24', 123)
 |-- Received Payload metadata: NTPv4, Mode=4, Stratum=1, LI=0
 |-- Server Reference TS: 3997772791.000033
 |-- [PASS] Standard loopback handshake validation verified (raw bytes match).

[+] Running Interleaved NTPv4 Validation Test...
 |-- Sending Interleaved Handshake Request packet...
 |-- Attempt 1/3: sending packet to ('192.168.1.24', 123)
 |-- Server Echoed Origin TS:   3997772791.531306 (expected prior client T4)
 |-- Server Receive TS (T2):     3997772792.5343633
 |-- Server Transmit TS (T3):    3997772791.5296874 (expected prior server T3)
 |-- [PASS] RFC 9769 interleaved timestamp validation verified.

------------------------------------------------------------------------------

                NTP RFC 9769 Protocol Validation Test Complete

                          Copyright Rob Latour, 2026
                                License: MIT
              Website: https://github.com/roblatour/ESP32TimeServer


==============================================================================
```

# NTP Server Alignment Test

## Purpose

This test determines alignment of NTP servers on the same internal network.

It is a useful way to help quality assure new builds and/or identify potential network issues.

Two or more servers can be tested at a time.

It queries each server repeatedly, calculates the average offset for each server, and compares those averages to show how closely the servers align to each other. 

Once done it prepares a report detailing per-server offset statistics, pairwise average differences, the maximum spread between averages, and an alignment assessment. 
Here is an [example](results.txt).

## Setup and usage

### Install Python

**Windows:**

```bash
winget install --id Python.Python.3 -e
```

**Linux:**

Install Python 3 using your distribution's package manager. For example:

```bash
sudo apt install python3
```

### Install `ntpdate`

Install `ntpdate` and ensure the `ntpdate` command is available on your system `PATH`. The script uses `ntpdate -q`, which queries a server without changing the computer's clock.

No Python packages are required; the script uses only the Python standard library.

---

## Usage notes

1. Run the test from the directory in which the `ntp_server_alignment.py` is located, supplying at least two unique NTP server IPv4
   addresses for example:

   ```bash
   py ntp_server_alignment.py --servers 192.168.7.24 192.168.1.24 192.168.1.214
   ```

   On systems where `py` is unavailable, use:

   ```bash
   python3 ntp_server_alignment.py --servers 192.168.7.24 192.168.1.24 192.168.1.214
   ```
 
   Before collection starts, the script pings every supplied server once. If one or
   more servers do not respond, it identifies each unresponsive server and exits
   without checking or changing the output file or running `ntpdate`.

   If every server responds, the script checks whether `results.txt` already exists.
   If it does, enter `y` to delete it and proceed, or `n` (or press Enter) to leave
   it unchanged and exit without running tests. Any other response is requested again.

2. By default the script uses a default of 20 samples from each server, to use a different number of samples:

   ```bash
   py ntp_server_alignment.py --samples 50 --servers 192.168.7.24 192.168.1.24 192.168.1.214
   ```

3. The server addresses are mandatory and must be unique IPv4 addresses in one of
   these ranges (indicating only internal servers are being tested):

   - `10.0.0.0` to `10.255.255.255`
   - `172.16.0.0` to `172.31.255.255`
   - `192.168.0.0` to `192.168.255.255`
   - `169.254.0.0` to `169.254.255.255`

   If any supplied address is invalid or outside these ranges, the script reports
   every invalid address and performs no testing.

4. Command options are case-insensitive. Run the script with no arguments, `?`,
   `-HELP`, or `--help` to display usage information without running tests.

5. Review `results.txt` after the test completes. 

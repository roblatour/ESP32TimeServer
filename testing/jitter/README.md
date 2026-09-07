# Determining NTP Jitter on Linux, Windows, and macOS

NTP jitter is a measure of the variation in network delay and time offset between a client and its NTP server(s). Lower jitter generally indicates a more stable and accurate time source.

---

# 1. Linux

## Recommended Method: Chrony

Most modern Linux distributions use **Chrony** rather than the older `ntpd`.

### Verify Installation

```bash
chronyc tracking
```

Key metrics:

- **RMS offset**: Overall time synchronization stability.
- **Root delay**: Total network delay to the reference clock.
- **Root dispersion**: Estimated clock error.

For source-specific jitter information:

```bash
chronyc sourcestats
```

The **Std Dev** column provides the best approximation of NTP jitter for each source.

### Installation

**RHEL / Rocky / AlmaLinux**

```bash
sudo dnf install chrony
```
```bash
sudo systemctl enable --now chronyd
```

**Ubuntu / Debian**

```bash
sudo apt update
```
```bash
sudo apt install chrony
```
```bash
sudo systemctl enable --now chrony
```

More information: https://chrony-project.org/documentation.html

## Alternative Method: NTPD

```bash
ntpq -c rv
```

Look for:

```text
jitter=0.345
```

The **jitter** variable is reported directly by NTP.

---

# 2. Windows

Windows includes the **Windows Time Service (W32Time)** by default.

## Native Windows Method (W32Time)

### Check Current Status

```cmd
w32tm /query /status
```

### Estimate Jitter Using Stripchart

```cmd
w32tm /stripchart /computer:time.windows.com /samples:20 /dataonly
```

Example:

```text
10:00:01, +0.0012345s
10:00:03, +0.0013450s
10:00:05, +0.0012100s
```

Review the variation between offset measurements. Greater variation indicates higher jitter.

### Installation

No installation is required. `w32tm.exe` is included with supported Windows releases.

## Alternative Method: Meinberg NTP

Meinberg provides a Windows distribution of the reference NTP implementation, including tools such as **ntpd** and **ntpq**. The package also includes monitoring utilities for viewing NTP performance. citeturn3search26turn3search28

### Check Jitter

Open an elevated command prompt and run:

```cmd
ntpq -c rv
```

Look for a line similar to:

```text
jitter=0.182
```

This value is the NTP daemon's calculated jitter and is generally more useful than estimating jitter from stripchart output.

### View Peer Statistics

```cmd
ntpq -p
```

This displays reachability, delay, offset, and peer statistics for configured NTP servers.

### Installation

Download and install the Meinberg NTP package for Windows:

https://www.meinbergglobal.com/english/sw/ntp.htm

Installation is typically a standard Next → Next → Finish process. The installer deploys the NTP service and associated utilities. citeturn3search26

---

# 3. macOS

macOS includes the **sntp** utility by default.

## Query an NTP Server

```bash
sntp -d pool.ntp.org
```

or

```bash
sntp -d time.apple.com
```

The `-d` option displays detailed timing information, including offset measurements.

### Basic Query

```bash
sntp pool.ntp.org
```

## More Detailed Analysis with Chrony

Install Chrony:

```bash
brew install chrony
```

Then run:

```bash
chronyc sourcestats
```

Review the **Std Dev** column as the jitter indicator.

### Homebrew

If Homebrew is not installed, see:

https://brew.sh

---

# Quick Reference

| OS             | Tool                  | Command               | Jitter Indicator |
| -------------- | --------------------- | --------------------- | ---------------- |
| Linux          | Chrony                | `chronyc sourcestats` | `Std Dev`        |
| Linux          | NTPD                  | `ntpq -c rv`          | `jitter=`        |
| Windows        | W32Time               | `w32tm /stripchart`   | Offset variation |
| Windows        | Meinberg NTP          | `ntpq -c rv`          | `jitter=`        |
| macOS          | sntp                  | `sntp -d server`      | Offset variation |
| macOS + Chrony | `chronyc sourcestats` | `Std Dev`             |

**Best Practice:** Collect at least 50–100 samples over several minutes and use the standard deviation of the measured offsets as the jitter value. This provides a more representative measurement than a single snapshot.

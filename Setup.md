# Using the ESP32 Time Server on Your Network

This guide explains how to configure your network or individual devices to use the **ESP32 NTP Stratum 1 Time Server** as their time source.

---

## Step 1 — Assign a Static IP Address

Before pointing any client at the ESP32 Time Server, it should have a **fixed, reserved IP address** on your network so that its address never changes after a DHCP lease renewal.

The easiest way to do this is through a **DHCP reservation** (sometimes called a static DHCP binding) on your router or firewall, keyed on the ESP32's MAC address.

> **Finding the IP address**
> Once the ESP32 Time Server boots and connects to the network:
> - If an **LCD screen is connected**, the assigned IP address is displayed on the screen.
> - If **no LCD is configured**, the IP address is printed to the serial console log (visible via `idf.py monitor` or any serial terminal at 115200 baud).

Note this IP address — it is needed for all of the configuration steps below.

---

## Option A — Network-Wide Setup (Recommended)

Configuring time at the router/firewall level means **every device on the network** automatically benefits without any per-device changes.

### OPNsense

OPNsense includes a built-in NTP service that can be pointed at the ESP32 Time Server so that OPNsense itself syncs to it and then acts as an NTP server for the rest of the LAN.

1. Log in to the OPNsense web interface.
2. Navigate to **Services → Network Time → General**.
3. In the **Time servers** field, replace any existing entries with the IP address of the ESP32 Time Server (e.g., `192.168.1.50`).
4. Set **NTP Orphan Mode** stratum to a value higher than 1 (e.g., `12`) so that OPNsense only falls back to its own clock as a last resort.
5. Under the **Interface** section, ensure the NTP service is listening on the **LAN** interface (and any other internal interfaces you want to serve).
6. Click **Save** and then **Apply**.
7. Navigate to **Services → Network Time → Status** to confirm OPNsense has synchronised to the ESP32 Time Server (it should appear as the active peer at stratum 1).

> **Optional — Force all LAN NTP traffic to OPNsense:** Create a firewall rule on the LAN interface that redirects UDP port 123 traffic (destination any) to OPNsense's own LAN IP. This ensures every device on the network uses OPNsense (backed by the ESP32) regardless of what NTP server it has configured.

---

### pfSense

pfSense likewise ships with an NTP daemon that can be configured to use the ESP32 Time Server as its upstream reference.

1. Log in to the pfSense web interface.
2. Navigate to **Services → NTP**.
3. In the **Time Server** list, remove any existing entries and add the IP address of the ESP32 Time Server (e.g., `192.168.1.50`). Select **Prefer** for this entry.
4. Under **NTP Interface(s)**, check the **LAN** interface (and any additional internal interfaces).
5. Set the **Orphan Tier** (under the Clock discipline section) to a value higher than 1 (e.g., `12`).
6. Click **Save**.
7. Navigate to **Status → NTP** to confirm pfSense has synchronised with the ESP32 Time Server at stratum 1.

> **Optional — Force all LAN NTP traffic to pfSense:** Under **Firewall → NAT → Port Forward**, create a rule that redirects UDP port 123 from any LAN source to pfSense's LAN IP. This intercepts hard-coded NTP addresses in devices and satisfies them from pfSense, which is backed by the ESP32 Time Server.

---

## Option B — Per-Device Setup

If a network-wide change is not possible or desired, each device can be configured individually to point directly at the ESP32 Time Server's IP address.

---

### Windows — Meinberg NTP Daemon (Recommended)

Windows ships with its own time sync service (`W32tm`), but its accuracy and reliability are limited. The **Meinberg NTP package** installs a proper `ntpd` service — the same reference implementation used on servers worldwide — and is the recommended approach for Windows systems.

#### Download

1. Open a browser and go to: [https://www.meinbergglobal.com/english/sw/ntp.htm#ntp_stable](https://www.meinbergglobal.com/english/sw/ntp.htm#ntp_stable)
2. Download the current stable installer — look for the section **"NTP for Current Windows Versions (Windows XP and later), with IPv6 support"** and download `ntp-4.2.8p18a2-win32-setup.exe` (or the latest version shown on the page).
3. Optionally verify the SHA256 checksum of the downloaded file against the `.sha256sum` file linked on the same page.

#### Install

1. Right-click the downloaded installer and select **Run as administrator**.
2. **Step 1 — Program folder:** Accept the default or choose a custom installation path, then click **Next**.
3. **Step 2 — Components:** Leave all components checked (NTP Service, NTP Documentation, etc.) and click **Next**.
4. **Step 3 — Initial configuration:** Select **"Use a public NTP time server"** (you will replace this with the ESP32 server address after installation) and click **Next**.
5. **Step 4 — Service settings:** Select **"Use System Account"** (recommended) and click **Next**.
6. Click **Install** to complete the installation. The NTP service will be installed and started automatically.

#### Configure to Use the ESP32 Time Server

After installation, edit the NTP configuration file to point to the ESP32 Time Server:

1. Open **Notepad** (or any text editor) **as Administrator**.
2. Open the file `C:\Program Files (x86)\NTP\etc\ntp.conf` (or `C:\Program Files\NTP\etc\ntp.conf` depending on your system).
3. Find any existing `server` or `pool` lines and replace them with the IP address of your ESP32 Time Server. For example:

   ```
   server 192.168.1.50 iburst prefer
   ```

   Replace `192.168.1.50` with the actual IP address of your ESP32 Time Server.

4. Save the file.
5. Restart the NTP service: open **Services** (`services.msc`), find **NTP Daemon**, right-click it, and select **Restart**. Alternatively, run the following in an **Administrator Command Prompt**:

   ```cmd
   net stop ntpd
   net start ntpd
   ```

#### Verify

Open an **Administrator Command Prompt** and run:

```cmd
ntpq -p
```

After a minute or two the output should show the ESP32 Time Server's IP address with an asterisk (`*`) next to it, indicating it is the active synchronisation source. The `stratum` column should read `1`.

You can also use the **NTP Time Server Monitor** (installed alongside the daemon) to view synchronisation status graphically.

---

### Linux

Most Linux distributions use either `systemd-timesyncd` (simple, default on many distros) or a full `ntpd` / `chrony` installation.

#### Option 1 — systemd-timesyncd (simple, most desktop distros)

1. Edit the timesyncd configuration file as root:

   ```bash
   sudo nano /etc/systemd/timesyncd.conf
   ```

2. Add or update the `NTP=` line under `[Time]` with the ESP32 Time Server's IP address:

   ```ini
   [Time]
   NTP=192.168.1.50
   ```

3. Save the file, then restart the service:

   ```bash
   sudo systemctl restart systemd-timesyncd
   ```

4. Verify synchronisation:

   ```bash
   timedatectl status
   ```

   The output should show `NTP service: active` and `System clock synchronized: yes`.

#### Option 2 — chrony (more accurate, recommended for servers)

1. Install chrony if not already present:

   ```bash
   # Debian / Ubuntu
   sudo apt install chrony

   # Fedora / RHEL / CentOS
   sudo dnf install chrony
   ```

2. Edit the chrony configuration:

   ```bash
   sudo nano /etc/chrony.conf
   ```

3. Comment out or remove existing `pool` / `server` lines and add:

   ```
   server 192.168.1.50 iburst prefer
   ```

4. Save and restart chrony:

   ```bash
   sudo systemctl restart chronyd
   ```

5. Verify:

   ```bash
   chronyc tracking
   chronyc sources -v
   ```

   The ESP32 Time Server should appear with a `*` indicating it is the selected source at stratum 1.

#### Option 3 — ntpd

1. Install ntp if not already present:

   ```bash
   # Debian / Ubuntu
   sudo apt install ntp

   # Fedora / RHEL / CentOS
   sudo dnf install ntp
   ```

2. Edit `/etc/ntp.conf`:

   ```bash
   sudo nano /etc/ntp.conf
   ```

3. Replace existing `pool` / `server` lines with:

   ```
   server 192.168.1.50 iburst prefer
   ```

4. Restart ntpd:

   ```bash
   sudo systemctl restart ntp
   ```

5. Verify after a minute:

   ```bash
   ntpq -p
   ```

   The ESP32 Time Server should show with an asterisk (`*`) at stratum 1.

---

### macOS (Apple)

macOS uses its own NTP client, configurable via the **Date & Time** system settings or the command line.

#### Graphical (System Settings)

1. Open **System Settings** (macOS Ventura and later) or **System Preferences** (macOS Monterey and earlier).
2. Navigate to **General → Date & Time** (Ventura/Sonoma) or **Date & Time** (Monterey and earlier).
3. Unlock the pane by clicking the padlock and entering your administrator password.
4. Ensure **Set date and time automatically** is enabled.
5. In the NTP server field, replace the existing value (e.g., `time.apple.com`) with the IP address of the ESP32 Time Server, for example:

   ```
   192.168.1.50
   ```

6. Press **Enter** / **Return** to confirm.

macOS will now synchronise to the ESP32 Time Server.

#### Command Line

You can also configure the NTP server from the terminal:

```bash
# Set the NTP server
sudo systemsetup -setnetworktimeserver 192.168.1.50

# Enable network time sync
sudo systemsetup -setusingnetworktime on

# Verify
sudo systemsetup -getnetworktimeserver
```

To force an immediate sync:

```bash
sudo sntp -sS 192.168.1.50
```

---

## Validating Accuracy

Once configured, you can cross-check the accuracy of your system clock against internet references using [https://time.is](https://time.is). A well-configured Stratum 1 source backed by GPS with PPS should keep your systems within a few milliseconds of true UTC.

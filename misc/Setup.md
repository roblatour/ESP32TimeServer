# ESP32TimeServer homelab setup

This guide brings together the network, server, and client setup for NTP and
PTP. It covers **configuration of an already built and flashed
ESP32TimeServer**, not firmware build or flashing instructions. Replace example
addresses and interface names with your own.

## Contents

- [ESP32TimeServer homelab setup](#esp32timeserver-homelab-setup)
  - [Contents](#contents)
  - [Overview and choosing a protocol](#overview-and-choosing-a-protocol)
  - [Prepare the homelab network](#prepare-the-homelab-network)
  - [Configure ESP32TimeServer](#configure-esp32timeserver)
  - [Linux and Ubuntu: NTP](#linux-and-ubuntu-ntp)
    - [Simple client: systemd-timesyncd](#simple-client-systemd-timesyncd)
    - [Chrony: NTP first, then optional PTP](#chrony-ntp-first-then-optional-ptp)
  - [Linux and Ubuntu: firewall rules](#linux-and-ubuntu-firewall-rules)
  - [Linux and Ubuntu: Layer 2 PTP with Chrony](#linux-and-ubuntu-layer-2-ptp-with-chrony)
    - [Prepare the interface and NTP](#prepare-the-interface-and-ntp)
    - [Hardware-timestamping branch](#hardware-timestamping-branch)
    - [Software-timestamping branch](#software-timestamping-branch)
    - [Connect LinuxPTP to Chrony and start services](#connect-linuxptp-to-chrony-and-start-services)
  - [Windows: NTP](#windows-ntp)
  - [Windows: Layer 3 UDP PTP](#windows-layer-3-udp-ptp)
  - [macOS: NTP and PTP limitations](#macos-ntp-and-ptp-limitations)
  - [Optional authentication and key management](#optional-authentication-and-key-management)
    - [NTP server and authenticated clients](#ntp-server-and-authenticated-clients)
    - [Layer 2 PTP server and LinuxPTP client](#layer-2-ptp-server-and-linuxptp-client)
  - [Validation and troubleshooting](#validation-and-troubleshooting)
  - [Further reading](#further-reading)
  - [Appendix 1 - Third Party Firewall Rules for Windows](#appendix-1---third-party-firewall-rules-for-windows)
    - [Windows NTP: W32Time or Meinberg](#windows-ntp-w32time-or-meinberg)
    - [Windows Layer 3 UDP PTP: W32Time](#windows-layer-3-udp-ptp-w32time)
    - [Bitdefender example for UDP PTP](#bitdefender-example-for-udp-ptp)

## Overview and choosing a protocol

A homelab is easier to troubleshoot when its computers agree on the time: logs
line up, scheduled jobs run when expected, and distributed applications have a
useful common timeline. ESP32TimeServer uses a GNSS reference and PPS to serve
local **stratum 1 NTP** and **PTP**. Its PTP service offers untagged Ethernet
Layer 2 and multicast UDP over IPv4 and/or IPv6. Services can be enabled
independently; NTP and both PTP transports can run together.

**Start with NTP unless you have a reason to need PTP.** NTP works across
ordinary IP networks and is the simplest option for a mixed-OS homelab. PTP uses
a grandmaster and followers, and can provide tighter synchronization on a
controlled wired LAN with suitable timestamping hardware. It is not
_automatically_ more accurate: NIC drivers, switches, network asymmetry,
congestion, clock quality, and client configuration matter. NTP can also use
hardware timestamps; a well-configured NTP client can outperform poorly
configured or software-timestamped PTP. A synchronized NIC hardware clock (PHC)
is not necessarily the computer's system clock.

| Protocol   | Strengths                       | Main limitations            |
| ---------- | ------------------------------- | --------------------------- |
| NTP        | Broad OS support; easy fallback | Accuracy varies by client   |
| PTP L2     | Optional authentication         | Same untagged LAN required  |
| PTP L3 UDP | Native Windows PTP support      | Always unsigned; local only |

- **NTP:** IP/UDP port `123`, routable across ordinary networks. Linux Chrony,
  Windows W32Time or Meinberg, and macOS can use it. Multiple sources and
  fallback are practical, but symmetric-key authentication needs a compatible
  client. With a learned rate, server-side PPS holdover lasts at most 180
  seconds after the last accepted PPS and advertises increasing dispersion. NTP
  works across IP router (for example OPNSense/pfSesne) interfaces.
- **PTP Layer 2:** untagged Ethernet EtherType `0x88f7`, multicast MAC
  `01:1b:19:00:00:00`. LinuxPTP can use it, with optional shared-key
  authentication, and Chrony can use its measurements to set the system clock.
  It does not work across an IP router interfaces only within them; switches
  must pass multicast. VLAN-tagged PTP, gPTP/802.1AS, and unicast negotiation
  are unsupported.
- **PTP Layer 3 UDP:** multicast UDP ports `319/320`, IPv4 group `224.0.1.129`
  or IPv6 group `ff02::181`. Native Windows W32Time can use the IPv4 service; It
  is **always unsigned**. Traffic is local to the multicast-capable network:
  outgoing IPv4 TTL and IPv6 hop limit are 1, and the IPv6 group is link-local.
  Like PTP Layer 2 it does not work across IP router interfaces only within
  them. It is not a substitute for the Layer 2 transport.

NTP supports holdover (where the Server continues to maintain accurate time
using its local internal clock after losing its external reference
synchronization source). **However, both PTP transports have no holdover.** They
withdraw service when GNSS/PPS is not qualified. Linux can retain NTP as a
fallback; the Windows native PTP configuration below does not automatically fall
back to NTP.

Both PTP services use **domain 0**, two-step Sync/Follow_Up, and end-to-end
(E2E) delay measurement by default. Domain, transport, delay mechanism, and
timescale must match between server and follower. A regular Ethernet switch may
forward PTP but does not become a PTP-aware switch by doing so. Do not confuse
this service with gPTP/IEEE 802.1AS used by some AVB devices. Successful
software configuration is not proof of hardware-verified accuracy or full IEEE
1588 conformance.

## Prepare the homelab network

1. Connect the ESP32TimeServer by wired Ethernet, as directly to the primary
   router or the clients' switch as practical. For PTP, put follower and server
   on a multicast-capable wired segment; avoid VLAN-tagged PTP and do not assume
   that routing between firewall interfaces forwards it. Virtual-machine time
   synchronization can interfere with physical PTP testing.
2. Give the server a **stable/static IP address**, preferably a DHCP reservation
   on the router/firewall keyed to its MAC address. This is needed for NTP and
   for the Windows UDP PTP setup; Linux Layer 2 PTP itself does not use an IP
   address. Find the assigned address on the optional LCD or in the serial log
   at 115200 baud. The examples below use `192.168.1.24`.
3. Permit only the protocols you need on trusted networks: UDP **123** for NTP;
   multicast and UDP **319/320** for UDP PTP; or EtherType **`0x88f7`** for
   Layer 2 PTP. Enabling UDP ports does not make a Layer 2 follower work. IPv6
   UDP PTP uses the link-local multicast group `ff02::181`, not a router, ULA,
   or global IPv6 address. Keep Internet exposure and broad firewall exceptions
   out of this setup.
4. Choose how NTP clients reach the server:
   - **Per-device:** point each client's NTP service to ESP32TimeServer. This
     makes each client visible in the server's MQTT reporting and permits
     different fallback sources per client.
   - **Router NTP service:** use ESP32TimeServer as an upstream source on
     OPNsense or pfSense, optionally alongside Internet NTP sources. The router
     then serves LAN clients and can provide fallback and status information,
     but the ESP32TimeServer sees only the router in its client report. In
     OPNsense, use **Services → Network Time → General**, add the ESP32 under
     **Time servers**, enable the LAN interface, set orphan stratum above 1 (for
     example 12), apply, and verify under **Services → Network Time → Status**.
     In pfSense, use **Services → NTP**, add the ESP32 under **Time Server**
     (optionally **Prefer**), serve the LAN under **NTP Interface(s)**, set
     **Orphan Tier** above 1, and check **Status → NTP**. Keep other working
     sources if fallback is desired rather than removing them all.
   - **Router redirection:** some routers can redirect LAN UDP 123 requests
     toward the ESP32. Check the router's actual LAN redirection/NAT behavior;
     ordinary WAN port forwarding is not equivalent. This makes the ESP32 a
     single point of failure and prevents clients from choosing their own
     upstream NTP servers.
   - **Hybrid:** let the router serve ordinary clients and point selected
     machines directly to ESP32TimeServer.

A router using ESP32TimeServer as its upstream normally serves one stratum below
the ESP32's NTP stratum 1. Additional sources improve resilience; pointing both
NTP and PTP at the same ESP32 provides **protocol fallback**, not an independent
reference if that device fails.

## Configure ESP32TimeServer

In `../main/ESP32TimeServerSettings.h`, retain your board's existing hardware,
GNSS, and network settings. Enable the services needed by your clients:

```cpp
#define NTP_SERVER_ENABLED 1
#define PTP_L2_SERVER_ENABLED 1
#define PTP_L3_SERVER_ENABLED 1
```

You may disable any unneeded service with `0`; at least one must remain enabled.
For Linux Layer 2 followers use `PTP_L2_SERVER_ENABLED=1`. For the Windows
recipe use `PTP_L3_SERVER_ENABLED=1`, keep `PTP_L3_UDP_DOMAIN=0`, and select
IPv4 with `PreferIPvX=1` or an IP-enabled dual-stack setting such as `0`.
`PreferIPvX=2` is IPv6-only and does not suit the Windows IPv4 recipe. UDP PTP
can serve IPv4 and IPv6 together; `PreferIPvX=0`, `4`, `6`, `8`, and `9` allow
both, while `1` is IPv4-only and `2` IPv6-only. Mode `7` is Layer 2 only and
requires disabling IP-based services. Existing local configurations enabling all
services should set `CONFIG_LWIP_MAX_SOCKETS=16`.

If the running firmware already has these settings, no rebuild is needed.
Otherwise apply the settings to your normal board profile and deploy the updated
firmware using the project's existing build/flash procedure; this guide does not
cover that procedure. Do not change the board profile merely to enable a
protocol.

Allow GNSS/PPS to qualify before testing. A green status indicator signals
normal operation; blue flashing indicates PPS disciplining is not yet complete.
Yellow flashing with `[H]` indicates NTP PPS holdover, **not** PTP holdover. The
server can also yield PTP to a better qualified master. For other LCD/LED states
see [status indicators](status_indicators.md).

Authentication is optional and **separate for NTP and Layer 2 PTP**; UDP PTP
remains unsigned regardless. Set up any keys and required policies under
[Optional authentication](#optional-authentication-and-key-management) before
configuring clients to require signed packets.

## Linux and Ubuntu: NTP

Use either the simple `systemd-timesyncd` client or Chrony; avoid having two
services discipline the system clock. The examples use Ubuntu paths; other Linux
distributions may have different paths and service names.

### Simple client: systemd-timesyncd

In `/etc/systemd/timesyncd.conf`, set:

```ini
[Time]
NTP=192.168.1.24
```

Then run `sudo systemctl restart systemd-timesyncd` and `timedatectl status`.
Look for `NTP service: active` and `System clock synchronized: yes`.

### Chrony: NTP first, then optional PTP

On Ubuntu install Chrony (`sudo apt install chrony`). Back up your existing
Chrony configuration, stop any other active system-clock controller, and edit
`/etc/chrony/chrony.conf` or an included file. Keep existing working fallback
sources if desired; do not add the same ESP32 twice. Add:

```text
server 192.168.1.24 iburst xleave
```

`xleave` enables interleaved NTP with a supporting server, including this
firmware. Keep Ubuntu's ordinary `driftfile`, `makestep`, and `rtcsync`
settings. Check syntax and restart:

```bash
sudo chronyd -p -f /etc/chrony/chrony.conf
sudo systemctl enable --now chrony.service
sudo systemctl restart chrony.service
chronyc sources -v
chronyc tracking
chronyc ntpdata 192.168.1.24
```

The ESP32 should become reachable (reach grows toward octal `377`), appear as
`^*` if selected, and report NTP stratum 1. Tracking should show
`Leap status: Normal`. Fix NTP reachability or authentication before adding PTP.
On an NTP-only client with a suitable NIC, Chrony's `hwtimestamp` option can
improve accuracy; verify **both** RX and TX timestamp modes with
`chronyc ntpdata`. Do **not** simultaneously discipline the same NIC PHC with
PTP and use it for Chrony NTP hardware timestamping in the combined recipe
below. Leave NTP fallback using software timestamps there.

For a separate `ntpd` installation, use `server 192.168.1.24 iburst prefer` in
`/etc/ntp.conf`, restart the distribution's NTP service, and check `ntpq -p`. Do
not run it alongside Chrony or timesyncd as a competing clock controller.

See [Linux firewall rules](#linux-and-ubuntu-firewall-rules) if the client has
restrictive local network policy.

## Linux and Ubuntu: firewall rules

**A Linux NTP client usually needs no new firewall rule.** With ordinary
stateful firewall defaults, it can send a request to the ESP32's UDP port `123`
and receive the reply as established traffic. If outbound UDP is blocked, allow
**outbound UDP to the server's port 123**, restricted to the ESP32's address.
Only a Linux machine **serving** NTP to other clients needs an inbound UDP 123
rule for those clients (for example, a router or a Chrony server with `allow`
configured). Do not open inbound UDP 123 merely to use Chrony or timesyncd as a
client.

On an Ubuntu client using **UFW**, inspect the policy first. If outbound NTP is
blocked, this example allows only the ESP32; substitute your server address:

```bash
sudo ufw status verbose
sudo ufw allow out to 192.168.1.24 port 123 proto udp
sudo ufw status numbered
```

For the **Layer 2 PTP** setup below, PTP uses Ethernet EtherType `0x88f7`, not
IP/UDP. UFW's ordinary IP port rules do **not** govern this traffic: opening UDP
`319/320` does not fix a blocked Layer 2 PTP link. A normal Linux IP firewall
needs no PTP port exception for this recipe. If you have explicitly configured
Ethernet/bridge filtering (such as nftables `bridge` or `netdev` hooks), inspect
that policy and permit PTP Ethernet frames on the trusted wired interface in
both directions, scoped to the relevant peers/multicast MAC. Also check that
switches forward the multicast group. Do not disable the host firewall or
AppArmor to work around a transport mismatch; AppArmor permissions for `ptp4l`
and `phc2sys` are covered below and are **not** firewall rules.

A Linux client using the **optional UDP PTP** transport instead needs multicast
UDP `319/320` permitted both ways on its PTP interface if its policy blocks that
traffic. Scope IPv4 rules to `224.0.1.129` and the ESP32 peer; for IPv6, use
link-local `ff02::181` and the correct interface. This does **not** apply to the
Layer 2 configuration below. Check the actual firewall implementation and
interface before adding rules.

## Linux and Ubuntu: Layer 2 PTP with Chrony

This recipe uses **LinuxPTP** (`ptp4l` and, with a hardware-timestamping NIC,
`phc2sys`) to feed PTP measurements to Chrony. **Chrony alone disciplines the
system clock** and keeps its NTP sources available for fallback:

- Hardware timestamps: ESP32 → `ptp4l` → NIC PHC → `phc2sys` (SHM producer) →
  Chrony → system clock.
- Software timestamps: ESP32 → `ptp4l` (SHM producer) → Chrony → system clock.

Use the same untagged Layer 2 network as the ESP32. Its IP address is needed for
NTP, not Layer 2 PTP. `ptp4l` otherwise defaults to UDP; specify `L2`. For a
Linux **UDP** PTP client instead, use `network_transport UDPv4` or `UDPv6`, with
E2E and matching domain; IPv6 additionally needs `udp6_scope 0x02`. UDP PTP has
no authentication. The steps below specifically configure **Layer 2**. For host
firewall requirements, see
[Linux firewall rules](#linux-and-ubuntu-firewall-rules); opening UDP ports does
not help Layer 2 PTP.

### Prepare the interface and NTP

1. Identify the wired interface with `ip link show`; use the interface name,
   such as `eno1`, not its numeric index, Wi-Fi, or a virtual interface. Verify
   that it and the ESP32 share the same untagged segment.
2. Install the tools: `sudo apt update` then
   `sudo apt install chrony linuxptp ethtool tcpdump apparmor-utils`.
3. Back up existing `/etc/chrony`, `/etc/linuxptp`, relevant AppArmor local
   overrides and any existing `ptp4l.service`/`phc2sys.service` units and
   drop-ins. Inspect and stop competing time services and old PTP instances;
   keep Chrony enabled. Complete
   [Chrony NTP setup](#chrony-ntp-first-then-optional-ptp) and verify NTP first.
   Leave any additional usable NTP sources for fallback.
4. Run `ethtool -T eno1`. For the hardware branch, check `hardware-transmit`,
   `hardware-receive`, `hardware-raw-clock`, a nonnegative **PTP Hardware
   Clock** index belonging to this NIC, transmit mode `on`, and an appropriate
   PTPv2 Layer 2 receive filter. A `/dev/ptp0` device alone does not prove this
   NIC supports it. Otherwise use software timestamps. Software mode works but
   has more latency uncertainty and jitter.

Choose **one** of the following two branches (described in detail below):
**Hardware-timestamping** or **Software-timestamping** (as determined in 4.
above).

Substitute your interface (as determined in 1. above) for `eno1` everywhere
below.

Create the folder `/etc/linuxptp` if needed with
`sudo install -d -m 755 /etc/linuxptp`.

### Hardware-timestamping branch

Create `/etc/linuxptp/ptp4l-chrony.conf`:

```ini
[global]
network_transport L2
delay_mechanism E2E
time_stamping hardware
clientOnly 1
domainNumber 0
clock_servo pi
```

Create `/etc/systemd/system/phc2sys.service`:

```ini
[Unit]
Description=PTP hardware clock measurements to Chrony SHM
Requires=ptp4l.service
After=ptp4l.service

[Service]
Type=simple
ExecStart=/usr/sbin/phc2sys -a -r -E ntpshm -M 0 -m -q
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Here `ptp4l` disciplines the NIC PHC. `phc2sys -E ntpshm -M 0` **publishes
measurements to SHM 0** rather than running its usual system-clock servo. Use
one `-r`, not two, and do not set a fixed UTC/TAI offset such as `-O -37`.
Disable Chrony's `hwtimestamp` on this same PHC before running the bridge; NTP
fallback can still use software timestamps.

### Software-timestamping branch

Create `/etc/linuxptp/ptp4l-chrony.conf` instead with:

```ini
[global]
network_transport L2
delay_mechanism E2E
time_stamping software
clientOnly 1
domainNumber 0
clock_servo ntpshm
ntpshm_segment 0
```

Leave `phc2sys` disabled. `ptp4l` publishes SHM 0 directly; changing its servo
to `pi` would compete with Chrony for the system clock. Do not install two SHM
producers on the same segment. Neither branch needs a hard-coded 37-second
Chrony correction.

### Connect LinuxPTP to Chrony and start services

Create `/etc/systemd/system/ptp4l.service` (replace `eno1`):

```ini
[Unit]
Description=ESP32 Layer 2 PTP client for Chrony
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l-chrony.conf -i eno1 -m -q
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Add **one** PTP reference clock to Chrony's configuration, alongside the NTP
server from the earlier section:

```text
server 192.168.1.24 iburst xleave
refclock SHM 0 poll 3 refid PTP prefer
```

Avoid duplicate source lines and remove `prefer` from other time sources. Do not
set `trust`, unrealistic delays, or an artificial UTC offset to make an
unqualified source appear healthy. Verify
`sudo chronyd -p -f /etc/chrony/chrony.conf`; inspect
`systemctl cat ptp4l.service` and any old drop-ins before enabling. SHM segment
0 (key `0x4e545030`) should be owned by root with mode `600`; investigate
unexpected ownership with `ipcs -m`.

Ubuntu AppArmor may restrict `ptp4l` even under `sudo`. If its packaged profile
includes `<local/usr.sbin.ptp4l>`, preserve other entries in
`/etc/apparmor.d/local/usr.sbin.ptp4l` and add only the needed rules:

```text
capability sys_time,
/etc/linuxptp/*.conf r,
/etc/linuxptp/*.cfg r,
```

Reload with `sudo apparmor_parser -r /etc/apparmor.d/usr.sbin.ptp4l`. For the
**hardware branch**, inspect the `phc2sys` profile's actual local hook (it may
include `local/usr.sbin.ptp4l` rather than `local/usr.sbin.phc2sys`) and permit
`@{run}/ptp4l rw,` and `/etc/linuxptp/*.cfg r,` through that hook, then reload
its profile. Add permissions only when appropriate to the installed profile and
audit denial; do not disable AppArmor globally.

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now chrony.service ptp4l.service
sudo systemctl restart chrony.service ptp4l.service
```

On hardware clients **also** run `sudo systemctl enable --now phc2sys.service`;
software clients must leave it disabled. Check
`systemctl show phc2sys.service -p ExecStart` on hardware clients: it must still
use `-a -r -E ntpshm -M 0`, not a system-clock PI servo.

Allow time for discovery and Chrony's filtering; a running process alone does
not mean synchronized time. Check `journalctl -u ptp4l.service`, and on hardware
clients `journalctl -u phc2sys.service`. Run `chronyc sources -v` and
`chronyc tracking`. A working preferred PTP source appears as `#* PTP` with
recent samples and nonzero reach; usable NTP remains available, often `^-`.
`#? PTP` with reach zero means no usable PTP samples. The PTP SHM source's
reported **stratum 0** is Chrony's reference-clock representation, _not_ a PTP
grandmaster stratum; tracking normally reports the client as stratum 1 when PTP
is selected. Check for `Leap status: Normal`. Hardware `ptp4l` should reach
`SLAVE` with recurring offsets; in software SHM mode `s0` or `UNCALIBRATED` can
be normal because Chrony, not `ptp4l`, adjusts the clock.

During a maintenance window, stop `ptp4l.service`, wait for PTP samples to
expire, and confirm a usable NTP source becomes `^*`; restart `ptp4l` and, if
needed, the hardware bridge, then confirm PTP recovers. This fallback is not
instantaneous and does not help if NTP is unreachable. Test real link and master
loss separately, then check boot persistence.

## Windows: NTP

**NTP is the safer default on Windows**, especially if additional NTP servers
are needed for resilience. Native W32Time can use unsigned NTP; it cannot use
this project's symmetric-key NTP authentication. The repository's
[`configTimeSettings.ps1`](../tools/configTimeSettings.ps1) can configure
W32Time NTP from **Windows PowerShell as Administrator**, using literal IPv4 or
IPv6 addresses:

```powershell
.\configTimeSettings.ps1 -set NTP -serveraddress1 192.168.1.24
w32tm /query /peers
w32tm /query /status /verbose
```

Run the script from the repository's `tools` directory; use `-help` for its
backup, other peers, and restore options. Server 1 is required; optional servers
2–4 must be supplied without gaps. For link-local IPv6 supply an interface scope
(for example `%23`, using the adapter's `ifIndex`). The script's second NTP peer
may be fallback-only; this is **NTP peer fallback**, not PTP-to-NTP failover.
For restrictive or third-party firewall policy, see
[Appendix 1: Windows NTP rules](#windows-ntp-w32time-or-meinberg).

For authenticated NTPv4 or a full `ntpd` implementation, install the
[Meinberg NTP package](https://www.meinbergglobal.com/english/sw/ntp.htm#ntp_stable)
and edit `C:\Program Files (x86)\NTP\etc\ntp.conf` (or
`C:\Program Files\NTP\etc\ntp.conf`) as Administrator:

```text
server 192.168.1.24 iburst prefer
```

Replace conflicting `server`/`pool` lines as appropriate, restart **NTP Daemon**
in Services, and check `ntpq -p` after a minute or two. The ESP32 should show as
the selected `*` source at stratum 1. Follow the separate
[Meinberg key steps](#ntp-server-and-authenticated-clients) if authentication is
required. Do not run two competing Windows clock services against each other;
check which provider owns the system clock before switching implementations.

## Windows: Layer 3 UDP PTP

Native W32Time PTP is available on Windows 10 version 1809 and Windows Server
2019 and later; use a supported, updated release. It needs the firmware's
**unsigned UDP/IPv4 PTP**, not Layer 2, on domain 0 with two-step Sync, E2E
delay, and valid PTP-timescale/UTC-offset announcements. Keep the PC and server
on the same multicast-capable wired IPv4 subnet. The Windows NIC driver may
offer hardware timestamps (improving precision), or Windows may use software
timestamps instead; Linux support on the same card does not imply Windows driver
support. The Windows recipe does not target VMs or domain-managed time policy.

**Important:** native Windows PTP cannot verify the project's Layer 2 PTP
authentication. Its PTP recipe disables the NTP input provider, so loss of PTP
has **no automatic NTP fallback**. A firewall allowlist is not authentication.
If no compatible, qualified UDP PTP source is reachable, keep NTP rather than
disabling working time synchronization.

From the repository's `tools` folder in **64-bit Windows PowerShell 5.1**, run
the helper with your ESP32 IPv4 address. Readiness and inventory do not need
Administrator privileges; `-set` and `-restore` do:

```powershell
.\configTimeSettings.ps1 -readiness 192.168.1.24
.\configTimeSettings.ps1 -inventory 192.168.1.24
.\configTimeSettings.ps1 -set PTP 192.168.1.24
```

The readiness report requires a wired adapter on the ESP32TimeServer's subnet
with a Private network profile and a usable Windows PTP provider. The script
refuses unsupported situations before changing settings. On confirmation it
saves a `configtimebackup_YYYY_MM_DD_HH_MM_SS.bak` file in the current
directory, configures W32Time's PTP provider, disables its NTP input provider,
applies a higher-accuracy clock-discipline profile, installs **Windows Defender
Firewall** rules for wired adapters/UDP 319–320, restarts Windows Time if
requested, and waits for fresh synchronization. **Keep the backup**; the script
does not configure third-party firewalls. Follow
[Appendix 1: Windows UDP PTP rules](#windows-layer-3-udp-ptp-w32time) for the
needed inbound/outbound traffic and a tested Bitdefender example. Do not use
`ping 224.0.1.129` as a PTP test: multicast peers normally do not answer it.

> **Note:** the configTimeSettings.ps1 script may identify a Network Adapter as
> not supporting hardware timestamping even though the same Network Adapter
> supports hardware timestamping in Linux. If so use Windows Start → Device
> Manager → Network Adapters → `the network adapter in question` → Properties →
> Advanced to look for and enable a property such as `PTP Hardware Timestamp`
> which is currently disabled. Also, be aware the adapter may support hardware
> timestamping in Linux but not in Windows as its manufacture did not have it
> certified by Microsoft for that purpose.

Verify with:

```powershell
w32tm /query /status /verbose
```

Look for `ReferenceId: 0x4D505450` (`MPTP`), the ESP32 as source,
`Last Sync Error: 0`, and an advancing **Last Successful Sync Time**. The phase
offset should shrink as Windows slews its clock; this can take minutes. Windows
may report stratum 3 for its PTP client: **PTP itself has no stratum**. In Event
Viewer, **Applications and Services Logs → Microsoft → Windows →
Time-Service-PTP-Provider → PTP-Operational**, events 512 and 513 indicate
Announce reception and selected master; System Time-Service event 34 indicates
rejected samples. If the clock is wrong by whole seconds, inspect UTC/TAI
announcements rather than adding a fixed offset.

To return to your prior configuration, run as Administrator:

```powershell
.\configTimeSettings.ps1 -restore "<path printed by -set PTP>"
```

Alternatively use `-set NTP -serveraddress1 192.168.1.24` as above. A manual
return is necessary after PTP loss; the firmware has no PTP holdover. The
script's restore does not undo third-party firewall changes.

## macOS: NTP and PTP limitations

macOS can use the ESP32's **unsigned NTP** service for its system clock. In
**System Settings → General → Date & Time** on recent macOS (or **System
Preferences → Date & Time** on older versions), enable **Set date and time
automatically** and replace the network time server with the ESP32's IP address.
Interface names and permission prompts vary by macOS release. Alternatively, in
Terminal:

```bash
sudo systemsetup -setnetworktimeserver 192.168.1.24
sudo systemsetup -setusingnetworktime on
sudo systemsetup -getnetworktimeserver
```

An optional immediate check is `sudo sntp -sS 192.168.1.24`. The built-in macOS
time service does **not** support this project's symmetric-key NTPv4
authentication. Leave NTP authentication in mixed mode for this client or use a
different supported client.

**PTP is not supported here for synchronizing the Mac's main system clock.**
macOS has PTP-related facilities for subsystems such as AVB audio and AirPlay,
but ESP32TimeServer interoperability with those subsystems has **not been tested
and is not supported in this release**. Do not treat Apple's AVB device browser
as a system-clock PTP client.

## Optional authentication and key management

Authentication checks possession of a shared secret and packet integrity; it
does **not** encrypt traffic, prevent packet delay or loss, or guarantee a
correctly configured reference. Keys in firmware, build outputs, logs, and
client files are recoverable by anyone with access. Protect them and keep them
out of public source control and support captures. NTP and PTP policies and keys
are independent; do not reuse their secrets.

### NTP server and authenticated clients

In `../main/ESP32TimeServerSettings.h`, set `NTP_AUTHENTICATION_ENABLED=1` and
choose `NTP_AUTHENTICATION_REQUIRED=0` for mixed signed/unsigned service or `1`
to reject unsigned requests (including NTPv3) across IPv4, IPv6, and loopback.
Required mode needs authentication enabled. In the private secrets header
included by the settings, provision **one to four** unique NTP key IDs
`1..65535`, each paired with an **exactly 32-character alphanumeric** secret.
Generate a new secret for each key, for example with:

```bash
python -c "import secrets; print(secrets.token_hex(16))"
```

Use `{key_id, "key_value"}` entries in the firmware's existing `symmetric_keys`
configuration. After deployment, the server's serial log near **Open for
business** supplies separate `Authorization key(s) for use with Meinberg` and
`Authorization key(s) for use with Chrony` lines. Copy the **appropriate
client-formatted line**, not the raw `key_value`, into the client's key file;
treat the log as secret material.

- **Windows Meinberg:** in `ntp.conf`, add a `keys` directive pointing to a
  private `ntp.keys` file, `trustedkey 1`, and
  `server 192.168.1.24 iburst key 1`. Put the corresponding log-formatted
  `1 SHA256 ...` entry into `ntp.keys`. Restart NTP Daemon; check
  `ntpq -4 -c as 127.0.0.1` for `auth=ok` and a selected peer. Use the same key
  ID throughout. Native W32Time does not support these keys.
- **Linux Chrony:** in Ubuntu's `/etc/chrony/chrony.conf` (or an included
  fragment), use `keyfile /etc/chrony/chrony.keys`,
  `server 192.168.1.24 key 1 version 4 iburst`, and `authselectmode require`.
  Put the matching `1 SHA256 HEX:...` line from the Chrony-formatted serial
  output into a root-protected `chrony.keys` file. Restart `chrony.service`;
  check `chronyc ntpdata 192.168.1.24` for `Authenticated: Yes` and inspect
  `chronyc authdata`, `chronyc sources -v`, and `chronyc tracking`. Do not use
  an unsigned `xleave` association as a substitute for signed requests. Older
  builds or crypto backends may differ; verify a signed request **and** reply
  before claiming compatibility.
- **macOS built-in time service:** no symmetric-key NTPv4 support.

To rotate a key without interrupting clients, add a **new ID and secret** on the
server first, deploy it, move and verify clients, then remove the old ID in a
later deployment. Changing a secret under an existing ID immediately breaks old
clients. Revocation takes effect only when replacement firmware is running.
Multiple clients may use the same key, but this does not give them separate
identities.

### Layer 2 PTP server and LinuxPTP client

Layer 2 PTP optionally supports the **IEEE 1588-2019 AUTHENTICATION TLV**
(`0x8009`) with immediate HMAC-SHA256-128. This is different from NTP's
SHA256-based packet digest. `PTP_L2_AUTHENTICATION_ENABLED=0` gives unsigned
PTP; with it enabled, `PTP_L2_AUTHENTICATION_REQUIRED=0` accepts unsigned or
valid signed incoming packets, while `1` requires signed input. A populated
valid key ring makes **all outgoing Layer 2 messages signed even in mixed
mode**. An enabled, optional but empty ring gives unsigned-only service; an
invalid ring or failed crypto initialization disables PTP instead of silently
falling back to unsigned. UDP PTP is always unsigned and unaffected.

In the private secrets header included by the settings, configure a separate
`ptp_l2_symmetric_keys` ring with one to four unique IDs `1..4294967295`, each
with an exactly 32-character alphanumeric ASCII secret. Use a new secret, not
the NTP key. Keep `PTP_L2_AUTHENTICATION_SPP=1` and
`PTP_L2_AUTHENTICATION_ACTIVE_KEY_ID=1` unless coordinating another SPP or
active ID with clients. Deploy the server settings before requiring verification
on clients. Keys are compiled into firmware, not stored in NVS.

Generate a new secret for each key, for example with:

```bash
python -c "import secrets; print(secrets.token_hex(16))"
```

Use a LinuxPTP build with IEEE 1588-2019 authentication and a crypto backend;
older Ubuntu packages may lack support. Protect `/etc/linuxptp/sa.cfg` as
root-owned mode `600`, with this format and **your own actual 32-character
secret** substituted after `ASCII:`:

```ini
[security_association]
spp 1
allow_mutable 0
1 SHA256-128 32 ASCII:REPLACE_WITH_YOUR_32_CHAR_SECRET
```

The algorithm label is **`SHA256-128`**, not `HMAC-SHA256-128`. `ASCII:` means
the literal ASCII bytes; `HEX:` is an alternative representation of the same
bytes, not a reason to decode an ASCII key merely because it looks hexadecimal.
Add the following under `[global]` in the **active**
`/etc/linuxptp/ptp4l-chrony.conf` from either timestamping branch above:

```ini
ptp_minor_version 1
spp 1
active_key_id 1
sa_file /etc/linuxptp/sa.cfg
allow_unauth 0
```

Keep its existing `network_transport L2`, `clock_servo`, and timestamping
settings. Restart `ptp4l`. In the **hardware** branch, its local management
socket also requires authentication: arrange for `phc2sys` to keep
`-a -r -E ntpshm -M 0 -m -q` **and** pass
`--spp=1 --active_key_id=1 --sa_file=/etc/linuxptp/sa.cfg`; restart it after
`ptp4l`. Permit the SA file in its AppArmor profile if needed. Software clients
have no bridge. Check service `ExecStart`, logs, fresh Chrony samples, and, for
hardware clients, fresh bridge SHM measurements. Seeing an authentication TLV on
the wire alone does **not** prove the client verified its MAC. An
unsigned/keyless follower cannot qualify a signed-only multicast stream simply
because the server accepts unsigned incoming requests. Never relax
`allow_unauth 0` merely to conceal a key mismatch.

For rotation, add the new ID to server and client rings, deploy both sides,
switch the active outgoing ID after all peers know it, then remove the old ID.
Shared-key PTP does not encrypt traffic, distinguish key-sharing clients, or
provide comprehensive replay protection. Keep unsigned/mixed deployments on a
trusted timing segment.

## Validation and troubleshooting

- **NTP:** check a fresh, reachable source and normal leap status on each
  client. A server's stratum 1 or a client's small estimated offset is not a
  direct measurement of absolute UTC error. The ESP32's NTP holdover is bounded
  and increases dispersion; it does not make a failed reference permanently
  usable.
- **Linux PTP:** check the correct interface, Layer 2 multicast forwarding,
  domain 0 and E2E, fresh Sync/Follow_Up and Delay_Req/Delay_Resp traffic, and
  `chronyc sources -v`/`tracking`. To inspect frames use
  `sudo tcpdump -ni eno1 -e -vv 'ether proto 0x88f7'`. Announce reception alone
  is not lock. For `failed to set the clock status: Operation not permitted`,
  inspect AppArmor denials before changing its local override. Test NTP fallback
  and link recovery on the actual hardware.
- **Windows PTP:** verify `w32tm /query /status /verbose`, adapter selection
  (PTP-Operational event 513), recent synchronization, and absence of event 34.
  A packet capture filter `udp.port == 319 || udp.port == 320` can show the
  exchanges. Check third-party firewalls and GNSS/PPS qualification if the
  script reports unverified synchronization; return manually to NTP on loss.
- **All protocols:** verify on physical devices with the chosen NIC, driver,
  board, and network. Internet clock comparison sites can be a rough check but
  cannot establish microsecond or nanosecond accuracy.

## Further reading

- [PTP transport details and IPv6 scope](PTP_transports.md)
- [ESP32TimeServer status indicators](status_indicators.md)
- [Microsoft PTP validation guide and examples](https://github.com/microsoft/W32Time/tree/master/Precision%20Time%20Protocol)
- [Microsoft Windows Time tools and settings](https://learn.microsoft.com/en-us/windows-server/networking/windows-time-service/windows-time-service-tools-and-settings)
- [LinuxPTP implementation and documentation](https://github.com/richardcochran/linuxptp/)
- [LinuxPTP `ptp4l` reference](https://linuxptp.nwtime.org/documentation/ptp4l/)
- [LinuxPTP `phc2sys` reference](https://linuxptp.nwtime.org/documentation/phc2sys/)
- [LinuxPTP security association format](https://github.com/richardcochran/linuxptp/blob/master/configs/sa.cfg)
- [Chrony documentation](https://chrony-project.org/documentation.html)
- [Chrony authentication configuration](https://chrony-project.org/doc/latest/chrony.conf.html)
- [Meinberg NTP downloads](https://www.meinbergglobal.com/english/sw/ntp.htm)
- [Meinberg NTP authentication instructions](https://kb.meinbergglobal.com/kb/time_sync/ntp/configuration/ntp_authentication)
- [Apple AVB device browser (not system-clock PTP)](https://support.apple.com/guide/audio-midi-setup/browse-your-network-for-an-avb-device-amsavb001/mac)

## Appendix 1 - Third Party Firewall Rules for Windows

A third-party firewall such as Bitdefender may enforce its own rules even when
Windows Defender Firewall rules exist. The repository's `configTimeSettings.ps1`
script checks Windows Defender Firewall policy; for PTP it adds Defender rules
but **does not configure third-party products**. It may warn that another
firewall is registered. Configure only the time service you actually use,
restrict rules to the trusted wired network and intended server, and check the
product's effective inbound and outbound policy. An IP allowlist does **not**
authenticate the time source. Back up existing firewall settings before changing
them; restoring Windows time settings with the script does not remove
third-party rules.

### Windows NTP: W32Time or Meinberg

**Usually no additional NTP client rule is necessary** if the firewall permits
outbound traffic and stateful replies. If outbound traffic is blocked or the
firewall reports a denied connection, allow **outbound UDP** to the ESP32's
address (for example `192.168.1.24`) on **remote port 123**. Permit return
packets as established/related traffic; do not open inbound UDP 123 to all hosts
just to receive replies. For additional NTP peers, add narrowly scoped rules for
their addresses too. W32Time also supports IPv6 peers; scope an IPv6 rule to the
actual server address and the correct network interface.

Choose the application/service actually controlling the clock:

- **Built-in W32Time:** match the **Windows Time** service in
  `C:\Windows\System32\svchost.exe` if the product supports service-specific
  rules. This is not a rule for `SysWOW64` or `WinSxS` copies of `svchost.exe`.
  The script checks for conflicting **Windows Defender** outbound UDP 123
  policy, but does not create a third-party allow rule for NTP.
- **Meinberg NTP Daemon:** use the daemon executable shown as **Path to
  executable** in the **NTP Daemon** service properties (typically `ntpd.exe`
  under the installed NTP `bin` directory), **not** `ntpq.exe`, which only
  queries status. Its firewall behavior and local source port can differ from
  W32Time's; if a restrictive firewall blocks replies, inspect its logs and
  allow only the necessary return traffic for that daemon and trusted peer. Do
  not assume a W32Time-only allow rule applies to Meinberg.

In a third-party firewall's application rules, set **Action: Allow**,
**Protocol: UDP**, **Direction: Outbound**, **Remote address: ESP32 IP**, and
**Remote port: 123**, with a trusted LAN/network profile. Keep the local source
port unrestricted unless the firewall logs establish a stable port for the
selected daemon. Only add a narrowly scoped inbound reply rule if the product
blocks established UDP replies; a Windows machine **serving** NTP to other
clients is a different use case requiring inbound UDP 123 from those clients.
Check `w32tm /query /status /verbose` for W32Time or `ntpq -p` for Meinberg to
confirm synchronization; a successful ping alone does not test NTP.

### Windows Layer 3 UDP PTP: W32Time

Native Windows PTP uses the firmware's **UDP/IPv4** multicast service, not
Ethernet Layer 2 or NTP's UDP 123. Permit **both inbound and outbound** UDP
traffic on ports **319** (event) and **320** (general) for Windows Time, scoped
to the ESP32's unicast IPv4 address **and** PTP multicast `224.0.1.129`. The
client must send multicast Delay_Req; permitting only unicast packets from the
ESP32 is insufficient. The script's Defender rules are limited to wired Ethernet
adapters on the ESP32 subnet, the Private profile, UDP 319/320, and Windows
Time. Its inbound rule allows traffic from the ESP32; its outbound rule allows
traffic to the ESP32 **and** the multicast group. Apply equivalent restrictions
in the active third-party product; its menus and definitions of local/remote
port may vary. Do not open these ports on a WAN interface.

Use `C:\Windows\System32\svchost.exe` for Windows Time if an application path is
required. `svchost.exe` hosts other services too, so add a **Windows Time
service** constraint if supported; the trusted network, peer addresses, and
ports should still be restricted. The firmware's UDP PTP is unsigned; these
rules provide reachability, **not** authentication. Windows does not switch back
to NTP automatically if PTP becomes unreachable.

### Bitdefender example for UDP PTP

The source Windows PTP guide reports this configuration tested with Bitdefender
Firewall. In **Firewall → Rules → Add rule**, create **two** allow rules for
`C:\Windows\System32\svchost.exe`. They differ only in **Remote IP**:

1. Rule 1: `192.168.1.24` (replace with the ESP32's IPv4 address).
2. Rule 2: `224.0.1.129` (the PTP multicast address).

For **each** rule set **Apply to all applications: Off**, **Permission: Allow**,
**Protocol: 17. UDP**, **Direction: Both**, **Custom Local Address: Off**,
**Custom Remote Address: On**, and **Remote Port(s): `319, 320`**. The tested
example used **Network Type: Any Network**; if your Bitdefender version offers a
reliable trusted-LAN restriction, prefer that instead. Menu names vary by
version. Because `svchost.exe` hosts multiple services, check whether your
version also supports restricting the rule to Windows Time. Leave Windows
Defender Firewall's script-owned rules in place; the Bitdefender rules do not
replace them if Defender policy is also active.

If PTP still does not synchronize, review the firewall's blocked-traffic log,
the selected Windows adapter and its network profile, and
`w32tm /query /status /verbose`. Check PTP-Operational events 512/513 and
Time-Service event 34 as described above. `ping 224.0.1.129` is **not** a useful
PTP test: multicast devices normally do not reply. Remove these manually created
third-party rules when returning to NTP if they are no longer needed.

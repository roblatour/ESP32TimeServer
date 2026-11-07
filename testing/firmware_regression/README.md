# Host firmware regression tests

## Windows setup

Install Node.js, Python 3.9 or later, and Visual Studio with the **Desktop
development with C++** workload. ESP-IDF v6.1's Python environment is
suitable for the profile-helper tests. Include the MSVC x64/x86 build tools and
a Windows SDK. The confirmed
Visual Studio Community installation path is
`C:\Program Files\Microsoft Visual Studio\18\Community`.

Run the following in **Command Prompt**, not directly in PowerShell, to load the
x64 compiler environment and confirm that `cl.exe` is available:

```cmd
set "VSROOT=C:\Program Files\Microsoft Visual Studio\18\Community"
call "%VSROOT%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64
where cl
```

For another Visual Studio version or edition, replace the installation path. An
**x64 Native Tools Command Prompt** shortcut can be used instead, if present. A
**Developer Command Prompt for VS 18** terminal profile is also suitable after
loading the x64 environment above. If `where cl` cannot find the compiler, do
not run the tests yet. `spawnSync cl.exe ENOENT` means this environment is not
loaded or the C++ tools are missing. Repeat this setup for each new session.

Run the regression commands from this same Visual Studio x64 environment. If the
ESP-IDF environment is needed for compiler-variant checks, load it first as
shown below; then reload the Visual Studio environment if necessary and verify
`where cl` again.

## Run the regression suite

Run from `.\testing\firmware_regression`:

```cmd
node .\run_all.js C:\Temp\ESP32TimeServer-regression
```
In the example above `C:\Temp\ESP32TimeServer-regression` is where the results of the 
tests will be stored ( in a file called `results.txt` ). Here is an example [results.txt](./example_results.txt) report. 

Optional arguments are `--build-dir <idf-build-directory>` and `--overwrite`.
All relative arguments are resolved from the directory where you run the
command. 

The output directory must be outside the source tree. In the example above, the
combined log is `C:\Temp\ESP32TimeServer-regression\results.txt`. Generated
artifacts are in `C:\Temp\ESP32TimeServer-regression\host` and, when
compiler-variant checks run, `C:\Temp\ESP32TimeServer-regression\variants`. The
runner sends stdout and stderr from suite commands to the log; it displays suite
progress in the console, but not live compiler output.

By default, the runner executes the full suite of tests in
`.\testing\firmware_regression\run.js`, plus the additional `--network-only`
tests for `DEBUG=1` network modes. The full suite already includes the
authentication and PTP-focused tests, the isolated profile-helper tests, and
the native GNSS UART and startup tests, so those suites are not run again
separately.

The GNSS tests drive the production UART and native UBX parser with synthetic
timing/configuration frames, reject oversized identity-version numbers, and
check that unrelated payloads and excluded PVT fields are not retained. Board
tests cover both target pin maps, S31 PHY failure propagation, its LCD wrapper,
and optional SPI TF pins with the other accessories enabled. TF tests extract
the production setup function and cover P4 SDMMC and enabled/disabled S31 SPI,
QoS 0/1/2, both debug modes, driver/mount/FAT32/directory/probe failures,
resource cleanup, RAM fallback, and persistent backlog restoration. These host
checks do not validate physical readers, power capacity, or timing during I/O.
MQTT storage tests also cover `MQTT_USE_TF_CARD`, dynamic allocation only when
needed, runtime read/write/directory failures, bounded full-card eviction,
allocation failure and heap-reserve thresholds. Fallback allocation is attempted
once per boot and never uses PSRAM: it must leave at least 64 KB of internal
8-bit heap and a 16 KB contiguous block. If those guards fail, or heap allocation
is configured to abort on failure, MQTT queuing is disabled without disabling
publishing or time services. Failed-card backlog remains on the card for a
subsequent boot; Q/q status persists until restart, below existing timekeeping
and network faults but above MQTT backlog/broker status. A card unavailable at
startup, or intentionally disabled or unsupported TF storage, uses RAM without a
TF-failure indicator.
Native OTE tests are separate; see
`testing/ote/run_native_ote_tests.ps1` for the host harness covering wrong-chip,
incompatible-revision, and truncated-image rejection. It discovers Visual
Studio with `vswhere`, or uses `VSROOT` if explicitly set.

The MAC regression verifies that `MACAddress` configures the Ethernet interface
MAC directly, rejects malformed, multicast and all-zero addresses, and that
reporting uses the six-byte Ethernet-driver readback. A guarded buffer checks
the six-byte read boundary, while source extraction rejects use of the
expanded eFuse MAC API in this path.

MQTT checks parse generated NTP-only, PTP-only, and combined JSON, including
NTP-nested client details, a typical ten-IPv4-client report, and full client
lists of maximum-length IPv6 addresses at the configured limits. A worst-case
report (every counter at its maximum, every optional block present) must fit
the computed report buffer and the RAM queue. Queue tests cover FIFO order,
wrap markers, drop-oldest and oversized reports. PTP checks cover actual MAC
formatting, identity deduplication, period continuity, expiry/clear/rejoin,
the MAC report limit with continuous clients retained, frozen
snapshots, and exact/undersized serialization buffers. Physical delivery and
task stack high-water measurements still require hardware validation.

The UDP PTP harness exercises production framing and task code with mocked
sockets and Ethernet timestamps: multicast addresses and checksums, truncation,
separate IPv4/IPv6 event/general bindings, IPv6-only sockets and interface
membership, two-step Sync/Follow_Up, E2E responses, queue overflow, independent
elections, stale PHC epochs, holdover withdrawal, and socket cleanup. Shared
MQTT PTP objects include all enabled transports. See
[PTP transports](../../misc/PTP_transports.md) for on-device release gates.

If the output directory already contains results, the runner asks before
replacing them. The default answer is no. In a noninteractive invocation, pass
`--overwrite` to permit replacement. Declining leaves the existing log
untouched. Pressing Ctrl+C can leave a partial `results.txt`.

## Results and exit status

After all suites finish, the runner scans the saved log. Each `PASS:` line
counts as one pass; the legacy `PTP host tests passed` line also counts as one
pass. These are grouped results, not counts of individual C++ assertions. Each
`FAIL:` line counts as one failure. If a suite exits nonzero, reports an error,
or produces no pass result without already emitting a failure marker, the runner
adds one `FAIL:` line for that unsuccessful suite. A zero exit code with failure
markers is also treated as a failure.

An incomplete suite stops internally, but the runner proceeds to the next suite.
Therefore a failed run may not have executed every check. The runner exits
nonzero if any suite fails, returns `130` on interruption during test execution,
and returns `1` for a usage error or declined overwrite.

## Firmware compiler-variant checks

Pass `--build-dir` to enable
`.\testing\firmware_regression\check_build_variants.js` against an existing,
successful ESP-IDF build. For example, load ESP-IDF v6.1 and then run the
regression command with `--build-dir .\build-esp32p4-rev-3-1`:

```cmd
call C:\esp\v6.1\esp-idf\export.bat
set "VSROOT=C:\Program Files\Microsoft Visual Studio\18\Community"
call "%VSROOT%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64
where cl
node .\testing\firmware_regression\run_all.js ^
  C:\Temp\ESP32TimeServer-regression ^
  --build-dir .\build-esp32p4-rev-3-1
```

The selected build directory must match the board profile. The variant checks
are compiler syntax checks, not full builds or links of NTP-only, PTP-only, and
combined firmware. They use the shareable settings template and synthetic test
credentials rather than copying the active deployment settings into their
output. If `--build-dir` is omitted, compiler-variant checks are
reported as `SKIP`; that run is not full firmware validation. All-three-disabled configuration must be rejected. UDP-only and all-transport
builds cover every IP-enabled `PreferIPvX` mode; UDP PTP rejects mode 7.
The checks also reject Layer-2-only/no-OTE/no-MQTT
builds unless `PreferIPvX=7`, and reject mode 7 when NTP, OTE, or MQTT is
enabled. They do not establish on-device PTP interoperability or timing
accuracy; see [PTP transports](../../misc/PTP_transports.md) for hardware
release gates.

To run the syntax checker directly after loading ESP-IDF, instead of using the
runner's `--build-dir` option:

```cmd
node .\testing\firmware_regression\check_build_variants.js ^
  .\build-esp32p4-rev-3-1 C:\Temp\ESP32TimeServer-variants
```

Replace the build directory for a different board profile. This check is not a
substitute for building and linking NTP-only, PTP-only, and combined firmware.
Run it for each fresh P4 revision and S31 build after loading the matching
ESP-IDF environment; S31 compiler checks do not establish physical timing
accuracy or validate the provisional load threshold.

P4 and S31 builds in one checkout must run sequentially: the Component Manager
can remove the other target's packages from the shared `managed_components`
directory even when build directories and lock files are separate.

After each target build, check its native GNSS component graph, link map,
application and bootloader chip/revision, flash settings, and partition fit.
For example:

```cmd
node .\testing\firmware_regression\check_build_graph.js .\build_s31 s31
```

Use `pre_v3`, `v3_0`, or `v3_1` with the corresponding P4 build directory.
CI runs the same graph/image check for all four profiles. The host suite also
tests the audit itself, including paths containing retired dependency names and
injected retired components, compile inputs, link artifacts, and wrong-chip
application/bootloader images.

## Individual host runs and coverage

To run the complete host suite directly without the master log or summary:

```cmd
node .\testing\firmware_regression\run.js ^
  C:\Temp\ESP32TimeServer-host-tests
```

From `.\testing\firmware_regression`, use `node .\run.js` with the same output
directory and optional filter instead. Running the repository-relative script
path from that subdirectory duplicates the path and causes `MODULE_NOT_FOUND`.

The output directory holds generated harnesses and executables, not firmware
sources. The suite covers the 3.0.5 NTP cache, one-use interleaved cookies,
holdover, timestamps, ingress/load limits, recovery, GNSS, and restart paths in
both debug modes. PTP tests cover the restricted L2 parser, bounded follower
lifecycle, service qualification without PTP holdover, wire headers, shared
transmit handling, recovery-aware ingress, and shared timestamp conversion in
free-running and legacy PHC modes. MQTT tests parse the actual generated JSON
for all three protocol modes, including follower snapshots and buffer bounds.
PPS capture tests enforce same-core MCPWM callback registration while allowing
cross-core Ethernet activation, and cover optional-loopback failures and
recovery.

Leap-second tests cover synthetic insertion/deletion, exact UTC/TAI boundaries,
NTP repetition/skipping, announcement windows, dynamic offsets, stale/invalid
data, GPS week rollover, and historical timestamp conversion. They do not
announce a real leap or replace GNSS-simulator and on-device transition tests.
Policy tests also cover NMEA/PPS qualification for both protocols, optional or
incomplete NAV-TIMELS, source conflicts/recovery, the 20 May 2027 continuous-UTC
boundary, and operation after the preceding bulletin expires. See
[leap-second handling](../../misc/leap_second_handling.md) for the authoritative
sources and receiver-specific behavior.

Authentication tests cover all four independent NTP/PTP mixed/required policy
combinations, PTP HMAC-SHA256-128 and unchanged NTP digest reference vectors,
all outgoing PTP message types, key rotation, tampering, malformed packets,
invalid configurations, and signed/legacy transmit paths. The host crypto
adapter uses Windows BCrypt; actual PSA integration is checked by ESP-IDF
builds. Optional empty rings check one-time warnings, policy-ready but
unavailable crypto, and no PSA calls. Required empty rings, malformed records
under both policies, PSA initialization/import errors, cleanup, and cached
failures must fail closed. Populated C arrays and `std::array` rings are both
exercised.

To run only the key/policy/crypto tests:

```cmd
node .\testing\firmware_regression\run.js ^
  C:\Temp\ESP32TimeServer-host-tests --auth-only
```

To include the parser, master-selection and real PTP startup/service-loop tests:

```cmd
node .\testing\firmware_regression\run.js ^
  C:\Temp\ESP32TimeServer-host-tests --ptp-only
```

This focused suite checks unsigned v2.0 version/control/length fields for every
outgoing message in optional keyless mode, rejection of signed/tampered input
before selection or Delay_Resp generation, no PTP task publication for invalid
policies, and no unsigned resend on signing failure. Link-generation recovery,
UTC metadata and strict GNSS/no-holdover qualification remain covered. Keys and
crypto failures use isolated public fixtures, not deployment secrets.

To run only the Ethernet/static-IP tests in both debug modes:

```cmd
node .\testing\firmware_regression\run.js ^
  C:\Temp\ESP32TimeServer-host-tests --network-only
```

These tests cover static IP addressing with neither DNS server, either DNS
server alone, or both DNS servers, plus invalid DNS settings and DHCP behavior.
They also verify L2-only interface attachment without IP-stack, DHCP, static-IP,
DNS, or IPv6 setup; link reconnection; all supported network-mode readiness
rules; full MAC display at 20/22 columns; and disabled-service health-check
gating. IPv6 tests enforce netif link-up before link-local creation with DHCP,
static IPv4, and reconnection. Display tests cover startup before the Ethernet
driver exists, retry after MAC-read errors, successful MAC caching, and the
combined NTP/PTP 15-second address rotation in both debug modes. The full suite
checks release of non-PTP traffic without an IP interface.

See [PTP authentication setup](../../misc/PTP_authentication_setup.md) for
LinuxPTP configuration and pending device interoperability/accuracy tests.

GNSS-only master-selection tests cover startup listening, dataset ranking,
two-message qualification, duplicate/wrapped sequences, changing datasets,
expiry, capacity, solitary/multiple masters, and legacy suppression. The real
PTP service loop is also driven with controlled queue/time inputs for both
selection settings and unsigned/mixed/required authentication. It checks
better/inferior masters, invalid authentication, standby Delay_Resp suppression
and failover, stale-generation messages, and continued rejection of NTP holdover
as a PTP time source. Host substitutes for hardware timestamps do not establish
physical accuracy.

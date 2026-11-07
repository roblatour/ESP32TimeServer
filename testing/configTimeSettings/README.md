# configTimeSettings Tests

Run the host-side checks from the repository root in 64-bit Windows PowerShell
5.1:

```powershell
.\testing\configTimeSettings\run_tests.ps1
```

The suite covers positional IPv4 inventory/readiness/PTP command parsing,
fixed-port IPv4/IPv6 and mixed-family NTP peer formatting, selected
address and restore-name validation (arbitrary names with a case-insensitive
`.bak` extension), startup VM detection using mocked computer
identities, PTPv2 IPv4 capability assessment and IP Helper structure layout, the
directory access probe, JSON backup integrity and schema checks, backup ACLs,
bounded structured process arguments, mocked inventory ranking and Ethernet-only
eligibility (including connected Wi-Fi exclusion), and green/yellow
recommendation colors. It also covers mocked PTP readiness (green
`[PTP adapter - expected]`, yellow `[PTP adapter - alternate]` and
`[Other path]` adapters, mixed timestamp support, route warnings, report-only
ping, and the no-wired-adapter refusal), PTP firewall rules covering every PTP
adapter, registry definitions, the post-activation event 513 adapter check, and
the mocked `-set PTP` refusal, apply and rollback paths. Mocked NTP apply tests
verify numeric link-local scope propagation to ping, firewall checks and W32Time,
address normalization, duplicate rejection, and invalid/unreachable-peer refusals
before backup or configuration writes. It creates and removes
its backup fixture in a unique temporary directory.

Inventory queries supported and active PTPv2-over-UDP/IPv4 hardware timestamp
capabilities through IP Helper before recommending a NIC. Active query failures
remain Unknown with an explicit diagnostic; `ERROR_BAD_DRIVER` (2001) indicates
that the driver advertises an unsupported timestamp configuration. An advanced
driver property set to Enabled is not proof of active timestamping. Inventory
does not alter driver settings. Host tests do not exercise NIC drivers or
timestamp hardware. VM tests use simulated Windows system information and do not
verify detection on every hypervisor. The suite also does not exercise elevated
registry/firewall/service changes, live synchronization, interface-constrained
ICMP, real PTP traffic, or timing accuracy. Windows PTP cannot be bound to an
adapter; it chooses one itself, and `-set PTP` reports that choice from
PTP-Operational event 513 after the restart. Windows and firmware PTP
interoperability has been checked on one multihomed computer only. Physical
release gates are not satisfied by host tests.

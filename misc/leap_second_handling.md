# Leap-second handling and continuous UTC

Policy and source data checked on **23 October 2026**.

## Official announcements and source data

- [CGPM 2026 Resolution 5][cgpm-2026], _On the technical actions needed to
  ensure the continuity of UTC_, establishes the 20 May 2027 effective date and
  increases the permitted magnitude of UT1 minus UTC to 3,600 seconds. This is
  the current direction used by the firmware.
- [CGPM 2022 Resolution 4][cgpm-2022], _On the use and future development of
  UTC_, established the earlier objective of increasing the permitted difference
  in or before 2035. The 2026 resolution brings implementation forward; software
  should not continue treating 2035 as the effective date.
- [IERS Bulletin C][iers] is the official operational announcement service for
  leap seconds.
- The [IANA leap-second list][iana] is a machine-readable, public-domain
  publication derived from IERS announcements. The copy checked for this
  implementation has update timestamp `3992312697` and expiration timestamp
  `4023129600`, both in NTP seconds since 1900. It expires on **28 June 2027**.
  Its last offset change is **1 January 2017**, establishing **TAI minus UTC =
  37 seconds**, and it lists no later change.

The abolition decision does **not** itself specify a new zero offset. The
firmware's 37-second value comes from the published offset data, separately from
the decision to stop future leap adjustments.

These links are documentation references, not URLs contacted by the device.

## Source-selection rules

The implementation is in [leap_seconds.h](../main/custom/leap_seconds.h) and
[main.cpp](../main/main.cpp).

### Before 20 May 2027

1. Prefer fully validated UBX NAV-TIMELS information associated with a resolved
   NAV-PVT UTC label and coherent GPS time of week.
2. If a poll is unavailable or incomplete, retain compatible validated GNSS
   information for up to 300 seconds.
3. Otherwise use the embedded IANA-derived policy for its supported UTC range.
   It records the 37-second offset from 1 January 2017 and no pending change.
   Its bulletin validity is bounded by the published 28 June 2027 expiration,
   not by the 300-second GNSS-message cache limit.
4. Do not silently discard a known pending event or a changed receiver offset
   when the GNSS cache expires. If that information conflicts with the embedded
   policy, qualification is withdrawn rather than guessing.

NAV-TIMELS current offsets must have a live constellation-derived source.
Firmware-default, configured, aided-only, and unknown offset values are not
accepted as authoritative receiver data. Full receiver leap announcements
require both validity bits and a known event-information source.

Missing or invalid receiver leap metadata does not mean the UTC label is
invalid: a valid UTC label can still use the independent authoritative policy.
Conversely, an unresolved or invalid UTC label cannot be repaired by an offset
table and does not qualify service.

### From 20 May 2027

- Use the CGPM continuous-UTC policy and the published transition offset. The
  embedded data currently establishes that offset as 37 seconds.
- Do not require the future-event validity bit or a fresh NAV-TIMELS packet
  merely to keep that policy valid. This policy does not expire when the
  preceding IANA list reaches 28 June 2027.
- UBX receivers may still be polled for consistency checking. A trusted
  contradictory current offset or a qualified leap announcement effective on or
  after the continuous-UTC date withdraws qualification. The firmware does not
  silently apply a post-transition leap.
- Compatible historical transition information may be retained to convert queued
  hardware timestamps using the offset applicable to each timestamp.

This is not a claim that the offset can never change under any future
international decision. It implements the currently published policy.

## Receiver behavior

Both protocols must be enabled in settings to offer both services. Existing
`allowFallbackProcessing` and receiver/hardware settings still govern whether
NMEA fallback is permitted; this change does not override them.

All supported rows require receiver-provided UTC and PPS.

| Receiver/output                   | Offset/leap source    | Service     |
| --------------------------------- | --------------------- | ----------- |
| UBX with valid NAV-TIMELS         | Validated GNSS/policy | NTP and PTP |
| UBX without NAV-TIMELS            | Embedded policy       | NTP and PTP |
| UBX with incomplete leap metadata | Cache/policy          | NTP and PTP |
| NMEA RMC plus PPS                 | Cache/embedded policy | NTP and PTP |
| Other binary protocol only        | Not implemented       | None        |

"Eligible" does not mean unconditional availability:

- NMEA sentences must pass the existing checksum, active-fix, date, and time
  checks. Initial clock setting requires coherent PPS-aligned confirmation
  samples. Late sentences are rejected rather than associated with another PPS
  edge.
- UBX UTC must have valid date/time and be fully resolved, in addition to the
  existing fix qualification.
- A compatible source for UTC/TAI conversion must be available. A trusted
  contradiction is not treated as ordinary missing metadata.
- Recent valid GNSS and PPS, model validity, and protocol-specific readiness
  remain necessary. An offset policy is not a replacement for a working clock.
- Without PPS there is no new qualified startup service. After PPS loss, NTP
  retains only its existing learned-rate holdover, for at most the configured
  180 seconds after the last accepted PPS, with increasing dispersion. **PTP has
  no holdover** and withdraws when its strict qualification is lost.

A non-u-blox receiver can therefore use its supported NMEA output and PPS. This
change does not add parsers for arbitrary proprietary binary messages or prove
that every receiver's sentence/PPS alignment is correct on hardware.

## NTP behavior

NTP serves UTC, not TAI. Its leap indicator is:

| LI  | Meaning and firmware behavior                                   |
| --- | --------------------------------------------------------------- |
| `0` | Qualified UTC with no pending leap under the selected authority |
| `1` | Qualified positive leap pending on the current UTC day          |
| `2` | Qualified negative leap pending on the current UTC day          |
| `3` | Unqualified/unsynchronized time; replies use stratum 16         |

LI `3` is **not** a "leap information unknown but fully synchronized" flag. Most
clients will not synchronize to it. The authoritative fallback lets an NMEA/PPS
receiver legitimately use LI `0`, rather than requiring LI `3` simply because
RMC contains no leap announcement.

Pre-transition leap machinery remains available: insertion repeats `23:59:59`,
deletion skips that second, and there is no leap smear. All model, hardware
RX/TX, reference, and fallback timestamp paths use the same UTC conversion.
Basic-mode transmit-delay compensation is applied in TAI before conversion so
crossing a leap boundary cannot create a wrong-second T3. Interleaved cookies
and authentication are unchanged.

Under continuous UTC there is no leap at the policy cutover. Normal qualified
replies use LI `0`; existing faults still withdraw stratum-1 qualification.

## PTP behavior

PTP Sync/Follow_Up and Delay_Resp timestamps use continuous TAI. Announce
advertises the PTP timescale and a valid current UTC offset when service
qualifies. The current published offset is 37 seconds.

Before the policy cutover, qualified leap information controls `leap61` and
`leap59`, with the offset change applied at the associated UTC boundary. From
the continuous-UTC cutover, those flags remain clear and the published offset is
retained. The cutover does not step or reset the PHC.

An NMEA/PPS receiver can qualify this service using the independent policy; PTP
is not disabled merely because its UTC label came from NMEA. However, missing
PPS, invalid GNSS timing, clock-model faults, or an authoritative offset/policy
conflict still prevent qualified PTP service.

The prototype remains unauthenticated L2 PTP v2.0, two-step E2E, static master
with foreign-Announce suppression, not BMCA. Class `248`, unknown accuracy, and
clear time/frequency traceability flags remain conservative. Correct timescale
metadata does not establish hardware accuracy or PTP conformance.

Followers configured for the old arbitrary timescale must instead honor the
PTP-timescale and UTC-offset metadata.

## Display, reporting, storage, and maintenance

The internal system clock, PPS model, and PHC mapping use TAI. Application
calendar display and MQTT timestamps explicitly convert to UTC. Raw libc `time`
and `gettimeofday` are therefore internal TAI; future code or libraries needing
POSIX UTC must not consume them without conversion.

The PHC frequency owner and sampling architecture are unchanged. Source
selection, calibration, servo updates, and holdover add no periodic NVS writes.
Source changes and policy conflicts use timing diagnostics gated by
`DEBUG_ENABLED`.

The policy is **embedded firmware data**, not an automatic online update
service. Before the effective date, maintainers should check new IERS/BIPM
announcements and update the embedded offset/schedule if an authoritative
pre-transition change is published. An unexpired snapshot is not a promise that
no newer bulletin will supersede it. NMEA alone cannot detect such a
publication.

In particular, if an official change before 20 May 2027 alters the final offset,
update `BULLETIN_TAI_UTC` and the associated supported schedule/source data
before relying on this firmware at the cutover. Do not infer a different final
offset solely from the abolition date. Any later international policy change
likewise requires a deliberate firmware update.

## Validation and remaining release gates

Host tests exercise:

- NMEA/PPS acquisition, startup-status qualification, late/invalid RMC, and
  missing PPS without invoking UBX polling on the NMEA path.
- Missing/incomplete NAV-TIMELS, fully resolved UTC requirements, source
  precedence, contradictory offsets/events, and recovery.
- The exact continuous-UTC boundary, unchanged UTC/TAI offset, bulletin expiry,
  and continued operation after the old list's expiration.
- Synthetic positive/negative pre-transition events, UTC repetition/deletion,
  historical timestamps, wire flags/offsets, and compensated T3 boundaries.
- Existing NTP holdover, PTP withdrawal, protocol feature variants, and both PHC
  modes.

See the [testing guide](../testing/README.md) for the host and firmware compiler
commands. A clean ESP-IDF build and host/syntax tests do **not** prove physical
accuracy. Release gates still include actual NMEA and UBX receivers, PPS
association, simulated date/leap/cutover cases, packet captures, UTC
display/reporting, loss/recovery, and PTP follower interoperability. No real
leap is announced by the synthetic tests.

[cgpm-2026]: https://www.bipm.org/en/cgpm-2026/resolution-5
[cgpm-2022]: https://www.bipm.org/en/cgpm-2022/resolution-4
[iers]: https://hpiers.obspm.fr/iers/bul/bulc/bulletinc.dat
[iana]: https://data.iana.org/time-zones/data/leap-seconds.list

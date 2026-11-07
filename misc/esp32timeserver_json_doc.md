# ESP32 Time Server MQTT JSON Reporting

The ESP32 Time Server's settings file ( `main/ESP32TimeServerSettings.h` ) is
use to enable and configure MQTT reporting.

When MQTT is enabled a JSON message will be published to `<MQTT_TOPIC>/report`
every `MQTT_Reporting_Period` seconds. Other configuration settings determine
the reporting content as described in more detail below.

Additionally, near the end of this document there is a description of 
controlled restart events and an example of a MQTT message for one.

### PTP retrofit scope

The release baseline remains **3.0.5**. PTP additions below describe the
[implemented 3.1-derived retrofit](ptp_retrofit.md). Generated reports have
passed host checks across six protocol/detail combinations, including follower
expiry, frozen snapshots, and overflow bounds. Physical broker/TF delivery and
hardware accuracy remain unvalidated. Existing NTP/shared reporting and
controlled restart meanings are retained.

With the retrofit, MQTT-enabled NTP-only reports omit `this_period.ptp`;
PTP-only reports omit `this_period.ntp` and the NTP-only client fields.
Shared `current`, GNSS/PPS, memory, queue, and historical sections keep their
existing enable conditions. NTP response counters are not combined NTP/PTP
totals. GNSS/PPS indicators do not imply that the stricter PTP qualification
passes: PTP withdraws on qualification loss even during permitted NTP holdover.

## Example report

This illustrative combined report includes optional sections; it is not a
capture from validated hardware.

```json
{
    "current": {
        "time": "2026-08-30T13:49:53-0400",
        "uptime": 2187,
        "ethernet_up": true,
        "gnss_synchronized": true,
        "gnss_synchronized_indicators": {
            "locked": true,
            "timing": true,
            "gps_valid": true,
            "sync_fresh": true,
            "sanity_check_passed": true
        },
        "pps_disciplined": true,
        "pps_disciplined_indicators": {
            "pps_signals_present": true,
            "discipline_active": true,
            "pps_synchronized": true
        },
        "satellites": 21,
        "memory": {
            "malloc_cap_dma": 115088,
            "free_heap": 154668,
            "minimum_free_heap": 148520,
            "largest_free_8bit_block": 106496
        }
    },
    "queued_messages": {
        "held": 0,
        "discarded": 0
    },
    "this_period": {
        "ethernet_up_secs": 120,
        "pps_pulses": 120,
        "gnss_locked_secs": 120,
        "satellites": {
            "min": 15,
            "max": 23
        },
        "ntp": {
            "requests": {
                "valid": 4,
                "invalid": 0,
                "load_shedding": false,
                "telemetry_dropped": 0,
                "max_per_second": 1
            },
            "responses": {
                "synchronized_and_disciplined": 4,
                "gnss_unsynchronized": 0,
                "pps_undisciplined": 0
            },
            "clients": [
                {
                    "address": "192.168.1.1",
                    "requests": 2
                },
                {
                    "address": "192.168.1.10",
                    "requests": 1
                },
                {
                    "address": "192.168.1.15",
                    "requests": 1
                }
            ],
            "clients_overflown": false
        },
        "ptp": {
            "followers": {
                "current": 1,
                "min": 0,
                "max": 2
            },
            "clients": [
                {
                    "mac": "00:f1:b2:d2:ca:9a",
                    "active_entire_period": true
                },
                {
                    "mac": "00:f1:b2:d2:ca:9b",
                    "active_entire_period": false
                }
            ],
            "follower_overflow_drops": 0
        }
    },
    "historical": {
        "gnss_receiver_last": {
            "synchronized_and_disciplined": "2026-08-30T13:49:33-0400",
            "gnss_unsynchronized": "",
            "pps_undisciplined": ""
        }
    }
}
```

## Always-present sections

### `current`

The `current` object reports the following states at the time the message is
created.

- **`time`**: Local device time in ISO 8601 basic-offset format
  (`YYYY-MM-DDTHH:MM:SS±HHMM`).
- **`uptime`**: Seconds since the ESP32 Time Server booted.
- **`ethernet_up`**: If the ethernet link is currently connected.

  Note: If the Ethernet isn't currently connected this message may be queued for
  later publication depending on the MQTT QOS setting.

- **`pps_active`**: If the PPS-based time discipline is currently working as
  expected.
- **`gnss_synchronized`**: An indicator reporting if the GNSS receiver is
  synchronized.
- **`gnss_synchronized_indicators`** a series of indicators to help identify
  issues should the gnss receiver become unsynchronized.

  Note: While the `gnss_synchronized_indicators` are shown in the example above,
  in practice they will only be present when `gnss_synchronized` is `false`.

- **`pps_disciplined`**: An indicator reporting if the PPS is disciplined.
- **`pps_disciplined_indicators`** a series of indicators to help identify
  issues should the PPS become undisciplined.

  Note: While the `pps_disciplined_indicators` are shown in the example above,
  in practice they will only be present when `pps_disciplined` is `false`.

- **`satellites`**: The number of satellites currently reported by the GNSS
  receiver.

### `this_period`

The `this_period` object reports activity in the current reporting period.
Counters in this object are reset after they are included in a report.

- **`ethernet_up_secs`**: Seconds that the Ethernet link was connected during
  the reporting period.

- **`pps_pulses`**: Number of PPS pulses received during the reporting period.
- **`gnss_locked_secs`**: Seconds that GNSS time lock was active during the
  reporting period.
- **`satellites.min`**: Lowest satellite count observed during the reporting
  period.
- **`satellites.max`**: Highest satellite count observed during the reporting
  period.
- **`ntp.requests.valid`**: Number of valid NTP requests received during the
  reporting period.
- **`ntp.requests.invalid`**: Number of invalid NTP requests received during the
  reporting period.
- **`load_shedding`**: If the server is being overwhelmed with NTP requests
  (in the magnitude of several thousand per second) it may need to actively 
  ignore some of them, if this happened this value will be true, 
  otherwise it will be false. 
- **`telemetry_dropped`**: Number of valid NTP requests whose optional MQTT
  client telemetry event could not be queued, usually because the event queue
  was full during a high request-rate burst. These requests were still received,
  processed, and replied to by the NTP server, and are included in
  `ntp.requests.valid`. Their source addresses and request counts are omitted
  from `this_period.ntp.clients`. This field does not affect
  `ntp.requests.max_per_second`, which is measured independently from the NTP
  request count at PPS intervals.
- **`ntp.requests.max_per_second`**: Highest NTP request rate observed in one
  PPS-delimited second during the reporting period.
- **`ntp.responses.gnss_lock`**: Number of NTP responses sent while GNSS time
  lock was available during the reporting period.
- **`ntp.responses.gnss_unlock`**: Number of NTP responses sent without GNSS
  time lock during the reporting period.

## Conditionally included sections

The following objects are included only when their corresponding compile-time
setting is enabled.

### `this_period.ptp.followers`

Included when MQTT and either `PTP_L2_SERVER_ENABLED` or
`PTP_L3_SERVER_ENABLED` are enabled. All enabled PTP transports contribute
to this same object. Place `ptp` immediately after `ntp` in combined reports.
PTP-only reports need no `ntp` object; disabling PTP
omits the entire `ptp` object rather than supplying artificial zero counts.

- **`current`**: Distinct recently active E2E requesters observed through
  accepted `Delay_Req` messages at the report snapshot.
- **`min`**: Minimum active observed requester count during the period.
- **`max`**: Maximum active observed requester count during the period.

An identity consists of transport, domain, `clockIdentity`, and `portNumber`.
Layer 2, UDP/IPv4, and UDP/IPv6 are distinct transports; the same peer on
multiple transports contributes multiple identities. Repeated
requests refresh an identity without increasing its count. The bounded table
defaults to 32 entries for Layer 2 (`PTP_L2_FOLLOWER_CAPACITY`) and 32 per
enabled UDP family (`PTP_L3_UDP_FOLLOWER_CAPACITY`). Expiry defaults to 30
seconds, selected by `PTP_L2_FOLLOWER_TIMEOUT_SECONDS` or
`PTP_L3_UDP_FOLLOWER_TIMEOUT_SECONDS`. When full, refresh
existing identities but ignore new ones. Passive multicast listeners are not
visible; these counts do not prove master selection or clock synchronization.

**`this_period.ptp.follower_overflow_drops`** counts accepted request
observations unable to add a new identity during the period, saturating at
4,294,967,295. It counts dropped observations, not distinct omitted identities.
A nonzero value warns that the observed count may undercount requesters. Reset
this diagnostic at report generation, not on each delivery retry.

Unexpired identities persist across reporting boundaries; link loss, clock/
qualification loss, service stop, and reboot clear affected identities.
Transport-specific election or address loss does not clear other transports.
Joins, expiry,
and clears update extrema even between snapshots. Atomically snapshot the three
values, maintaining `min <= current <= max`, then seed the next period's extrema
with its starting active count. No observed identities throughout a period
means genuine `0/0/0`, not unavailable data.

Freeze these values in the generated report. Later delivery from RAM or TF
queues must not substitute live counts, reset identities, or roll the reporting
period again. Physical broker/TF delivery and retry remain validation gates.
Detailed PTP request/reply totals are not included.

### `this_period.ptp.clients`

Included whenever MQTT and PTP are enabled, independently of
`MQTT_CLIENT_REPORTING_ENABLED` (which controls NTP client details).

- **`mac`**: The Ethernet source MAC from accepted follower requests, formatted
  as six lowercase, zero-padded hexadecimal octets. This is not inferred from
  the PTP clock identity. Multiple PTP identities using the same MAC produce
  only one client record.
- **`active_entire_period`**: `true` when the MAC had at least one tracked,
  active identity continuously from the preceding report boundary through this
  snapshot. A new arrival, expiry of its last active identity, or a table clear
  makes it `false`; rejoining in the same period does not restore `true`.
  Activity uses the same accepted-request and inactivity rules as the follower
  counts; it does not establish servo lock.

The array contains at most `MQTT_PTP_CLIENT_REPORT_LIMIT` unique MACs (default
96, set in the settings file; it must cover the summed enabled Layer 2 and
per-family UDP identity capacities). Every entire-period MAC is retained
(at most 96 with all three default transports).
Remaining entries are admitted in first-observed order; once full,
additional MACs are omitted from reporting, without affecting PTP service.
Departed MACs remain listed until the report boundary, and repeated
departures/rejoins never duplicate a record. `follower_overflow_drops` concerns
the active identity table, not this reporting limit. Client records are frozen
with the rest of the report for queued delivery.

### `current.memory`

Included when `MQTT_MEMORY_REPORTING_ENABLED` is `1` (Enabled).

All values are byte counts. `malloc_cap_dma` reports free heap memory that
matches the ESP-IDF DMA allocation capability. `free_heap` is the total
currently free heap, `minimum_free_heap` is the lowest total free heap observed
since boot, and `largest_free_8bit_block` is the largest currently allocatable
8-bit-capable block.

### `queued_messages`

Included when `MQTT_QOS` is set to `1` or `2`. It describes reports that could
not be published while the MQTT broker was unavailable and are waiting to be
sent: on the TF card when one is available, otherwise in a fixed-size volatile
RAM queue (`MQTT_RAM_QUEUE_BYTES`, default 64 KB). Reports vary in size, so the
number the RAM queue holds depends on how many clients each report lists.

- **`held`**: Number of previously queued reports waiting when this report was
  created.

- **`discarded`**: Number of queued reports discarded since the preceding report
  because the queue reached its capacity; the oldest reports are dropped first
  to make room for the newest. This counter is reset after reporting.

QoS `0` does not queue reports, so this object is omitted.

Please see the ESP32 Time Server's settings file for more information on
MQTT_QOS values and the limitations associated with queueing.

### `this_period.ntp.clients` and `this_period.ntp.clients_overflown`

Included when NTP is enabled and `MQTT_CLIENT_REPORTING_ENABLED` is `1`
(Enabled). These fields have moved inside `ntp`; consumers must update their
previous `this_period.clients` and `this_period.clients_overflown` paths.

- **`clients`**: An array of unique NTP client addresses observed during the
  reporting period. Each entry contains `address`, an IPv4 or IPv6 address, and
  `requests`, the number of NTP requests from that address during the period.

- **`clients_overflown`**: This will be set to true when the internal client
  table could not hold every unique client observed during the reporting period.

  At most `MQTT_NTP_CLIENT_REPORT_LIMIT` client entries (default 256, IPv4 and
  IPv6 combined, set in the settings file) are held for reporting, with or
  without a TF card. Client information is held and reported on a first come
  first serve basis. If more unique clients are active in a reporting period,
  their NTP requests are still processed and counted, but the additional
  clients are not listed in the MQTT message; instead the overflow flag is set
  to true. This flag is reset after reporting for the next reporting period.

### `historical`

Included when `MQTT_HISTORICAL_REPORTING_ENABLED` is `1` (Enabled).

under `gnss_receiver_last`:

- **`synchronized_and_disciplined`**: Local timestamp of the when the receiver
  was both synchronized and disciplined.
- **`gnss_unsynchronized`**: Local timestamp of when the GNSS last became
  unsynchronized.
- **`pps_undisciplined`**: Local timestamp of when the PPS last became
  undisciplined.

Historical timestamps use the same format as `current - time` (above). A
timestamp is published as an empty string when no corresponding event time is
available.

### `Symmetric key authentication details`

When both NTP_AUTHENTICATION_ENABLED **and** MQTT_DISTINGUISH_AUTHENTICATED_REPORTING_ENABLED are enabled then use:
 
```json
 "ntp": {
            "requests": {
                "authenticated": {
                    "valid": 4,
                    "invalid": 0
                },
                "unauthenticated": {
                    "valid": 0,
                    "invalid": 0
                },
                "load_shedding": false,
                "telemetry_dropped": 0,
                "max_per_second": 1
            },
```
as a replacement for 
```json
 "ntp": {
            "requests": {
                "valid": 4,
                "invalid": 0,
                "load_shedding": false,
                "telemetry_dropped": 0,
                "max_per_second": 1
            },
```
as shown above.

## Controlled restart messages

Controlled restarts are triggered in specific events:
1. A manual restart was requested via uptime/restart button ("manual_restart")
2. An Over the Ethernet firmware update was requested ("over_the_ethernet_update")
3. There were more consecutive time syncs failures than the allowable threshold permits ("more_than_x_consecutive_time_sync_failures")
4. The time sync update failed the sanity check and the option to reboot for such an event was true ("time_sync_failed_sanity_check")
5. The Ethernet transport layer was stalled (became overwhelmed) while processing incoming requests - usually on a scale of over 10K, or more, requests per second over an extended time frame ("ethernet_transport_stalled")

```json
{
    "event": "controlled_restart",
    "reason": "manual_restart"
}
```

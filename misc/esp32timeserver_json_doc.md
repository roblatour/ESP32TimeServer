# ESP32 Time Server MQTT JSON Reporting

The ESP32 Time Server's settings file ( `main/ESP32TimeServerSettings.h` ) is
use to enable and configure MQTT reporting.

When MQTT is enabled a JSON message will be published to `<MQTT_TOPIC>/report`
every `MQTT_Reporting_Period` seconds. Other configuration settings determine
the reporting content as described in more detail below.

## Example report

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
            "malloc_cap_8bit": 154668,
            "malloc_cap_32bit": 154668,
            "malloc_cap_internal": 154668,
            "malloc_cap_dma": 115088,
            "malloc_cap_spiram": 0,
            "malloc_cap_default": 154668,
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
                "telemetry_dropped": 0,
                "max_per_second": 1
            },
            "responses": {
                "synchronized_and_disciplined": 4,
                "gnss_unsynchronized": 0,
                "pps_undisciplined": 0
            }
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
- **`gnss_synchronized`**: An indicator reporting if the GNSS receiver is synchronized.
- **`gnss_synchronized_indicators`** a series of indicators to help identify issues
   should the gnss receiver become unsynchronized.  
  
  Note: While the `gnss_synchronized_indicators` are shown in the example above, in practice they
will only be present when `gnss_synchronized` is `false`.
  
- **`pps_disciplined`**: An indicator reporting if the PPS is disciplined.
- **`pps_disciplined_indicators`** a series of indicators to help identify issues
should the PPS become undisciplined.

  Note: While the `pps_disciplined_indicators` are shown in the example above, in practice they
will only be present when `pps_disciplined` is `false`.


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
- **`ntp.requests.invalid`**: Number of invalid NTP requests received during
the reporting period.
- **`telemetry_dropped`**: Number of valid NTP requests whose optional MQTT
  client telemetry event could not be queued, usually because the event queue
  was full during a high request-rate burst. These requests were still received,
  processed, and replied to by the NTP server, and are included in
  `ntp.requests.valid`. Their source addresses and request counts are omitted
  from `this_period.clients`. This field does not affect
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

### `current.memory`

Included when `MQTT_MEMORY_REPORTING_ENABLED` is `1` (Enabled).

All values are byte counts. The `malloc_cap_*` fields report free heap memory
that matches the named ESP-IDF allocation capability. `free_heap` is the total
currently free heap, `minimum_free_heap` is the lowest total free heap observed
since boot, and `largest_free_8bit_block` is the largest currently allocatable
8-bit-capable block.

### `queued_messages`

Included when `MQTT_QOS` is set to `1` or `2`. It describes reports queued in
volatile RAM while the MQTT broker was unavailable.

- **`held`**: Number of previously queued reports waiting when this report was
  created.

- **`discarded`**: Number of queued reports discarded since the preceding report
  because the queue reached its capacity. This counter is reset after reporting.

QoS `0` does not queue reports, so this object is omitted.

Please see the ESP32 Time Server's settings file for more information on
MQTT_QOS values and the limitations associated with queueing.

### `this_period.clients` and `this_period.clients_overflown`

Included when `MQTT_CLIENT_REPORTING_ENABLED` is `1` (Enabled).

- **`clients`**: An array of unique NTP client addresses observed during the
  reporting period. Each entry contains `address`, an IPv4 or IPv6 address, and
  `requests`, the number of NTP requests from that address during the period.

- **`clients_overflown`**: This will be set to true when the internal client
  table could not hold every unique client observed during the reporting period.

  Without a TF card installed in the ESP32-P4-ETCH only up to 50 client entries
  that can be held for reporting at the end of the reporting period. Client
  information is held and reported on a first come first serve basis. If more
  than 50 unique clients are active in a reporting period, while their NTP
  requests will be processed, they will not be reported in the MQTT message,
  rather the overflow flag will be set to true and reported as such. This flag
  will be reset after reporting for the next reporting period.

  With a TF card installed the limit to how many client entries can be held for
  reporting is set in the Settings file (with a default of 500).

### `historical`

Included when `MQTT_HISTORICAL_REPORTING_ENABLED` is `1` (Enabled).

under `gnss_receiver_last`:
- **`synchronized_and_disciplined`**: Local timestamp of the when the receiver was both
synchronized and disciplined.
- **`gnss_unsynchronized`**: Local timestamp of when the GNSS last became unsynchronized.
- **`pps_undisciplined`**: Local timestamp of when the PPS last became undisciplined. 

Historical timestamps use the same format as `current - time` (above).  A timestamp 
is published as an empty string when no corresponding event time is available.
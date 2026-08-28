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
        "time": "2026-08-27T17:24:20-0400",
        "uptime": 15327,
        "ethernet_up": true,
        "pps_active": true,
        "gnss_locked": true,
        "satellites": 22,
        "memory": {
            "malloc_cap_8bit": 161192,
            "malloc_cap_32bit": 161192,
            "malloc_cap_internal": 161192,
            "malloc_cap_dma": 121612,
            "malloc_cap_spiram": 0,
            "malloc_cap_default": 161192,
            "free_heap": 161192,
            "minimum_free_heap": 154948,
            "largest_free_8bit_block": 114688
        }
    },
    "queued_messages": {
        "held": 0,
        "discarded": 0
    },
    "this_period": {
        "ethernet_up_secs": 900,
        "pps_pulses": 900,
        "gnss_locked_secs": 900,
        "satellites": {
            "min": 16,
            "max": 22
        },
        "ntp": {
            "requests": {
                "valid": 41,
                "invalid": 0,
                "max_per_second": 1
            },
            "responses": {
                "gnss_lock": 41,
                "gnss_unlock": 0
            }
        },
        "clients": [
            {
                "address": "192.168.1.10",
                "requests": 13
            },
            {
                "address": "192.168.1.15",
                "requests": 14
            },
            {
                "address": "192.168.4.4",
                "requests": 1
            },
            {
                "address": "192.168.7.1",
                "requests": 13
            }
        ],
        "clients_overflown": false
    },
    "historical": {
        "gnss": {
            "last_lock": "2026-08-27T13:09:20-0400",
            "last_unlock": ""
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

  Note: if the Ethernet isn't currently connected this message may be queued for
  later publication depending on the MQTT QOS setting)

- **`pps_active`**: If the PPS-based time discipline is currently working as
  expected.
- **`gnss_locked`**: If the GNSS receiver has a time lock.
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
- **`ntp.requests.max_per_second`**: Highest NTP request rate observed in one
  second.
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
  information is held and reported on a first come first serve basis.
  If more than 50 unique clients are active in a reporting period, while their 
  NTP requests will be processed, they will not be reported in the MQTT message,
  rather the overflow flag will be set to true and reported as such. 
  This flag will be reset after reporting for the next reporting period.

  With a TF card installed the limit to how many client entries can be
  held for reporting is set in the Settings file (with a default of 500).

### `historical`

Included when `MQTT_HISTORICAL_REPORTING_ENABLED` is `1` (Enabled).

- **`gnss.last_lock`**: Local timestamp of the most recent GNSS lock event.
- **`gnss.last_unlock`**: Local timestamp of the most recent GNSS unlock event.

Historical timestamps use the same format as `current.time`. A timestamp is an
empty string when no corresponding event time is available.

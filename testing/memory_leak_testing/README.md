# Memory Leak Test Instructions

## Prerequisites

- Have an MQTT broker running (such as [Eclipse Mosquitto](https://mosquitto.org/))
- Have a way to connect to it to see published results 
  (such as [MQTT Explorer](https://mqtt-explorer.com/))

## Setup

1. **Set the ESP32TimeServer up to publish via MQTT**

   In the `ESP32TimeServerSettings.h` file set:

   - `MQTT_ENABLED` to `1` (Enabled)
   - `MQTT_MEMORY_REPORTING_ENABLED` to `1` (Enabled)
   - Set the values for:
     - `MQTTServerIPAddress[]`
     - `MQTTPort`
     - `MQTTUsername`
     - `MQTTPassword`
     - `MQTTBrokerRetain` to `1`
     - `MQTTReportingPeriod` to a workable time frame for examining results, 
        such as `120` (2 minutes)

1. **Build, flash and physically set up the ESP32TimeServer**

2. **View the published results with a tool such as MQTT Explorer**

   The reported values in the memory section should be expected to change over 
   the first few publications, but then generally remain constant over time. 
   Having that said, minor amounts of ebb and flow are ok, but a general
   trending decrease in any of the reported values over time is not.


## Pro tip

With Mosquitto installed ( https://mosquitto.org/ ) here's an example CLI 
command which may be used to monitor esp32TimeServer's publications and 
write (append) them to a file:

### Windows:
```bash
"c:\program files\mosquitto\mosquitto_sub" -h MQTTServerIPAddress -u MQTTUsername -P MQTTPassword -t "ESP32TimeServer/report" -k 60 >> json_records.txt
```

### Linux
```bash
mosquitto_sub -h MQTTServerIPAddress -u MQTTUsername -P MQTTPassword -t "ESP32TimeServer report" -k 60 >> json_records.txt
```
( On most Linux distributions, mosquitto_sub is installed into /usr/bin/mosquitto_sub )

### macOS
```bash
mosquitto_sub -h MQTTServerIPAddress -u MQTTUsername -P MQTTPassword -t "ESP32TimeServer/report" -k 60 >> json_records.txt
```
( If installed via Homebrew, the path is typically /usr/local/bin/mosquitto_sub 
or /opt/homebrew/bin/mosquitto_sub on Apple Silicon. You can check with:
```bash
which mosquitto_sub
```

Where MQTTServerIPAddress, MQTTUsername, and MQTTPassword are the 
values for these fields in the `ESP32TimeServerSettings.h` file.

Here too is a python script which may be used to create a comma separated 
values (.csv) file from the output of the above (for import to program 
such as Excel):

```python
#
# Copyright Rob Latour, 2026
# License: MIT
# Website: https://github.com/roblatour/ESP32TimeServer
#

import json
import csv

INPUT_FILE = "json_records.txt"
OUTPUT_FILE = "json_records.csv"

def flatten_json(obj, prefix="", out=None):
    """Recursively flattens nested JSON objects."""
    if out is None:
        out = {}

    for key, value in obj.items():
        full_key = f"{prefix}.{key}" if prefix else key

        if isinstance(value, dict):
            flatten_json(value, full_key, out)
        elif isinstance(value, list):
            # Convert lists to JSON strings
            out[full_key] = json.dumps(value)
        else:
            out[full_key] = value

    return out

rows = []

with open(INPUT_FILE, "r") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        try:
            data = json.loads(line)
        except json.JSONDecodeError as e:
            print("Skipping invalid JSON line:", e)
            continue

        flat = flatten_json(data)
        rows.append(flat)

# Collect all possible CSV columns
all_keys = set()
for row in rows:
    all_keys.update(row.keys())
all_keys = sorted(all_keys)

# Write CSV
with open(OUTPUT_FILE, "w", newline="") as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=all_keys)
    writer.writeheader()
    writer.writerows(rows)

print("CSV written to", OUTPUT_FILE)
```



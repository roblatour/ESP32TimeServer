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

#
# Copyright Rob Latour, 2026
# License: MIT
# Website: https://github.com/roblatour/ESP32TimeServer
#

import os
import re
import statistics
import numpy as np
from scipy import stats

# check before reading
if not os.path.exists("gathered_data.txt"):
    print("The gathered_data.txt file is missing, please run the drift_test.py script first.")
    exit(1)

# Safe to read now
with open("gathered_data.txt", "r") as file:
    report_data = file.read()

# ------------------------------------------------------------
# Extract FIRST and LAST test timestamps
# ------------------------------------------------------------
# Matches timestamps like: " 6 Sep 14:53:06"
timestamp_pattern = r"\d{1,2} \w{3} \d{2}:\d{2}:\d{2}"

timestamps = re.findall(timestamp_pattern, report_data)

if len(timestamps) < 2:
    print("Could not locate at least two timestamps in gathered_data.txt")
    exit(1)

first_timestamp = timestamps[0]
last_timestamp = timestamps[-1]

# ------------------------------------------------------------
# Configuration parameters (thresholds)
# ------------------------------------------------------------
max_external_disagreement = 0.005
max_internal_difference = 0.002
max_average_master_error = 0.003
max_average_reference_error = 0.003
max_master_endpoint_drift = 0.005
max_reference_endpoint_drift = 0.005
max_internal_endpoint_drift = 0.002
max_drift_slope = 0.0001
max_outlier_count = 3

# Extract test series blocks
series_blocks = re.split(r"\*\*\*\* Test Series \d+ \*\*\*\*", report_data)[1:]
series_numbers = re.findall(r"\*\*\*\* Test Series (\d+) \*\*\*\*", report_data)

data = []

for idx, block in enumerate(series_blocks):
    series_number = int(series_numbers[idx])

    # Extract offsets
    offsets = re.findall(r"adjust time server [^\s]+ offset ([+-]?\d+\.\d+)", block)
    if len(offsets) < 4:
        raise ValueError(f"Series {series_number} does not contain enough offset entries.")

    external_a_offset = float(offsets[0])
    external_b_offset = float(offsets[1])
    master_offset = float(offsets[2])
    reference_offset = float(offsets[3])

    data.append({
        "Series Number": series_number,
        "External_A_Offset": external_a_offset,
        "External_B_Offset": external_b_offset,
        "Master_Offset": master_offset,
        "Reference_Offset": reference_offset
    })

# Calculate external consensus
for entry in data:
    entry["External_Consensus"] = (entry["External_A_Offset"] + entry["External_B_Offset"]) / 2

# Calculate primary error metrics
for entry in data:
    entry["Master_Error"] = entry["Master_Offset"] - entry["External_Consensus"]
    entry["Reference_Error"] = entry["Reference_Offset"] - entry["External_Consensus"]
    entry["Internal_Difference"] = entry["Master_Offset"] - entry["Reference_Offset"]
    entry["External_Difference"] = entry["External_A_Offset"] - entry["External_B_Offset"]

# Endpoint drift values
first = data[0]
last = data[-1]

master_endpoint_drift = last["Master_Error"] - first["Master_Error"]
reference_endpoint_drift = last["Reference_Error"] - first["Reference_Error"]
internal_endpoint_drift = last["Internal_Difference"] - first["Internal_Difference"]

# Trend slope helper
def slope_or_note(values):
    if len(values) <= 1:
        return "No trend (insufficient data)"
    if all(v == values[0] for v in values):
        return "No trend (flat data)"
    slope, _, _, _, _ = stats.linregress(range(len(values)), values)
    return slope

master_errors = [entry["Master_Error"] for entry in data]
reference_errors = [entry["Reference_Error"] for entry in data]
internal_differences = [entry["Internal_Difference"] for entry in data]

master_trend_slope = slope_or_note(master_errors)
reference_trend_slope = slope_or_note(reference_errors)
internal_trend_slope = slope_or_note(internal_differences)

# Stability statistics
def safe_mean(values):
    return statistics.mean(values) if len(values) > 1 else 0

def safe_std(values):
    return statistics.stdev(values) if len(values) > 1 else 0

master_mean = safe_mean(master_errors)
master_max = max(abs(v) for v in master_errors)
master_std = safe_std(master_errors)

reference_mean = safe_mean(reference_errors)
reference_max = max(abs(v) for v in reference_errors)
reference_std = safe_std(reference_errors)

internal_mean = safe_mean(internal_differences)
internal_max = max(abs(v) for v in internal_differences)
internal_std = safe_std(internal_differences)

external_mean = safe_mean([entry["External_Difference"] for entry in data])
external_max = max(abs(entry["External_Difference"]) for entry in data)
external_std = safe_std([entry["External_Difference"] for entry in data])

# Outlier detection
outliers = []
for entry in data:
    for key in ["Master_Error", "Reference_Error", "Internal_Difference"]:
        values = [d[key] for d in data]
        mean = safe_mean(values)
        std = safe_std(values)
        if std == 0:
            continue
        if abs(entry[key] - mean) > 3 * std:
            outliers.append({
                "Series Number": entry["Series Number"],
                "Metric": key,
                "Value": entry[key],
                "Reason Flagged": f"Outside 3σ ({mean} ± {std})"
            })

# Confidence assessment
external_confidence = "High" if external_std < max_external_disagreement else "Low"
master_stability = "Stable" if abs(master_endpoint_drift) < max_master_endpoint_drift else "Unstable"
reference_stability = "Stable" if abs(reference_endpoint_drift) < max_reference_endpoint_drift else "Unstable"
internal_sync = "Synchronized" if abs(internal_endpoint_drift) < max_internal_endpoint_drift else "Unsynchronized"

confidence_level = external_confidence
if master_stability == "Unstable" or reference_stability == "Unstable" or internal_sync == "Unsynchronized":
    confidence_level = "Medium"
if len(outliers) > max_outlier_count:
    confidence_level = "Low"

# Final PASS/FAIL
pass_fail = "PASS"
if abs(master_endpoint_drift) > max_master_endpoint_drift or abs(reference_endpoint_drift) > max_reference_endpoint_drift or abs(internal_endpoint_drift) > max_internal_endpoint_drift:
    pass_fail = "FAIL"
elif abs(master_endpoint_drift) > max_master_endpoint_drift * 0.5 or abs(reference_endpoint_drift) > max_reference_endpoint_drift * 0.5 or abs(internal_endpoint_drift) > max_internal_endpoint_drift * 0.5:
    pass_fail = "PASS WITH WARNINGS"

# ------------------------------------------------------------
# Build report (with FIRST/LAST timestamps added)
# ------------------------------------------------------------
report = f"""
Executive Summary
Overall Result: {pass_fail}
Confidence: {confidence_level}

Test Period: {first_timestamp} - {last_timestamp}

Endpoint Drift Analysis (Primary Section)
Master Server
Initial Error: {master_errors[0]}
Final Error: {master_errors[-1]}
Net Endpoint Drift: {master_endpoint_drift}
Direction: {'Ahead' if master_endpoint_drift > 0 else 'Behind'}
Assessment: {master_stability}

Reference Server
Initial Error: {reference_errors[0]}
Final Error: {reference_errors[-1]}
Net Endpoint Drift: {reference_endpoint_drift}
Direction: {'Ahead' if reference_endpoint_drift > 0 else 'Behind'}
Assessment: {reference_stability}

Internal Synchronization
Initial Difference: {internal_differences[0]}
Final Difference: {internal_differences[-1]}
Net Separation Change: {internal_endpoint_drift}
Direction: {'Master ahead' if internal_endpoint_drift > 0 else 'Reference ahead'}
Assessment: {internal_sync}

External Reference Analysis
Average Consensus Value: {external_mean}
Average Disagreement: {external_mean}
Maximum Disagreement: {external_max}
Standard Deviation: {external_std}
Confidence Assessment: {external_confidence}

Master Analysis
Average Error: {master_mean}
Maximum Error: {master_max}
Standard Deviation: {master_std}
Trend Slope: {master_trend_slope}
Endpoint Drift: {master_endpoint_drift}
Outlier Count: {len([o for o in outliers if o['Metric'] == 'Master_Error'])}

Reference Analysis
Average Error: {reference_mean}
Maximum Error: {reference_max}
Standard Deviation: {reference_std}
Trend Slope: {reference_trend_slope}
Endpoint Drift: {reference_endpoint_drift}
Outlier Count: {len([o for o in outliers if o['Metric'] == 'Reference_Error'])}

Internal Synchronization Analysis
Average Difference: {internal_mean}
Maximum Difference: {internal_max}
Standard Deviation: {internal_std}
Trend Slope: {internal_trend_slope}
Endpoint Drift: {internal_endpoint_drift}
Outlier Count: {len([o for o in outliers if o['Metric'] == 'Internal_Difference'])}

Outlier Summary
"""

for outlier in outliers:
    report += f"Series Number: {outlier['Series Number']}, Metric: {outlier['Metric']}, Value: {outlier['Value']}, Reason Flagged: {outlier['Reason Flagged']}\n"

report += """
Findings and Conclusions:
"""

if master_stability == "Stable":
    report += "The Master server exhibited no meaningful endpoint drift and remained stable relative to the external consensus reference throughout the test period.\n"
else:
    report += "The Master server accumulated measurable endpoint drift relative to the external consensus.\n"

if reference_stability == "Stable":
    report += "The Reference server exhibited no meaningful endpoint drift and remained stable relative to the external consensus reference throughout the test period.\n"
else:
    report += "The Reference server accumulated measurable endpoint drift relative to the external consensus.\n"

if internal_sync == "Synchronized":
    report += "The Master and Reference servers remained synchronized throughout the test period.\n"
else:
    report += "The Master and Reference servers showed increasing separation between the first and final test series.\n"

print(report)

# Delete final_report.txt from the current working directory if it exists
if os.path.exists("final_report.txt"):
    os.remove("final_report.txt")

with open("final_report.txt", "w", encoding="utf-8") as outfile:
    outfile.write(report)

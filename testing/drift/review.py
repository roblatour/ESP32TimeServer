#
# Copyright Rob Latour, 2026
# License: MIT
# Website: https://github.com/roblatour/ESP32TimeServer
#

import os
import re
import statistics
from datetime import datetime
from scipy import stats

if not os.path.exists("gathered_data.txt"):
    print("The gathered_data.txt file is missing, please run the drift_test.py script first.")
    exit(1)

with open("gathered_data.txt", "r", encoding="utf-8") as file:
    report_data = file.read()

timestamp_pattern = r"\d{1,2} \w{3} \d{2}:\d{2}:\d{2}"
timestamps = re.findall(timestamp_pattern, report_data)

if len(timestamps) < 2:
    print("Could not locate at least two timestamps in gathered_data.txt")
    exit(1)

first_timestamp = timestamps[0]
last_timestamp = timestamps[-1]
first_datetime = datetime.strptime(f"2001 {first_timestamp}", "%Y %d %b %H:%M:%S")
last_datetime = datetime.strptime(f"2001 {last_timestamp}", "%Y %d %b %H:%M:%S")
if last_datetime < first_datetime:
    last_datetime = last_datetime.replace(year=last_datetime.year + 1)
test_duration = last_datetime - first_datetime

server_names = re.findall(r"^ntpdate -q (\S+)$", report_data, re.MULTILINE)
if len(server_names) < 4:
    print("Could not locate all four server names in gathered_data.txt")
    exit(1)

external_pool_1, external_pool_2, master_server, reference_server = server_names[:4]
number_of_tests = len(re.findall(r"\*\*\*\* Test Series \d+ \*\*\*\*", report_data))

max_external_disagreement = 0.005
max_internal_difference = 0.002
max_master_endpoint_drift = 0.005
max_reference_endpoint_drift = 0.005
max_internal_endpoint_drift = 0.003
max_drift_slope = 0.0001

series_blocks = re.split(r"\*\*\*\* Test Series \d+ \*\*\*\*", report_data)[1:]
series_numbers = re.findall(r"\*\*\*\* Test Series (\d+) \*\*\*\*", report_data)

data = []

for idx, block in enumerate(series_blocks):
    series_number = int(series_numbers[idx])
    offsets = re.findall(r"adjust time server [^\s]+ offset ([+-]?\d+\.\d+)", block)
    if len(offsets) < 4:
        raise ValueError(f"Series {series_number} does not contain enough offset entries.")

    external_a_offset = float(offsets[0])
    external_b_offset = float(offsets[1])
    master_offset = float(offsets[2])
    reference_offset = float(offsets[3])
    external_difference = external_a_offset - external_b_offset

    data.append({
        "Series Number": series_number,
        "External_A_Offset": external_a_offset,
        "External_B_Offset": external_b_offset,
        "Master_Offset": master_offset,
        "Reference_Offset": reference_offset,
        "External_Consensus": (external_a_offset + external_b_offset) / 2,
        "External_Difference": external_difference,
        "External_Valid": abs(external_difference) <= max_external_disagreement,
    })


def slope_or_none(values):
    if len(values) < 2 or all(value == values[0] for value in values):
        return None
    slope, _, _, _, _ = stats.linregress(range(len(values)), values)
    return slope


def safe_mean(values):
    return statistics.mean(values) if values else 0


def safe_std(values):
    return statistics.stdev(values) if len(values) > 1 else 0


def format_value(value):
    return "Not available" if value is None else f"{value:.6f}"


def endpoint_assessment(server_offsets, external_offsets, threshold):
    errors = [server_offset - external_offset for server_offset, external_offset in zip(server_offsets, external_offsets)]
    endpoint_drift = errors[-1] - errors[0]
    return {
        "Initial Error": errors[0],
        "Final Error": errors[-1],
        "Endpoint Drift": endpoint_drift,
        "Trend Slope": slope_or_none(errors),
        "Assessment": "PASS" if abs(endpoint_drift) <= threshold else "FAIL",
    }


master_offsets = [entry["Master_Offset"] for entry in data]
reference_offsets = [entry["Reference_Offset"] for entry in data]
external_a_offsets = [entry["External_A_Offset"] for entry in data]
external_b_offsets = [entry["External_B_Offset"] for entry in data]
internal_differences = [master_offset - reference_offset for master_offset, reference_offset in zip(master_offsets, reference_offsets)]
external_disagreements = [abs(entry["External_Difference"]) for entry in data]
invalid_external_series = [entry["Series Number"] for entry in data if not entry["External_Valid"]]

master_vs_a = endpoint_assessment(master_offsets, external_a_offsets, max_master_endpoint_drift)
reference_vs_a = endpoint_assessment(reference_offsets, external_a_offsets, max_reference_endpoint_drift)
master_vs_b = endpoint_assessment(master_offsets, external_b_offsets, max_master_endpoint_drift)
reference_vs_b = endpoint_assessment(reference_offsets, external_b_offsets, max_reference_endpoint_drift)

source_b_comparison_valid = not invalid_external_series
if not source_b_comparison_valid:
    master_vs_b["Assessment"] = "INCONCLUSIVE"
    reference_vs_b["Assessment"] = "INCONCLUSIVE"

internal_endpoint_drift = internal_differences[-1] - internal_differences[0]
internal_max = max(abs(value) for value in internal_differences)
internal_trend_slope = slope_or_none(internal_differences)
internal_sync = "PASS" if (
    abs(internal_endpoint_drift) <= max_internal_endpoint_drift and
    internal_max <= max_internal_difference and
    (internal_trend_slope is None or abs(internal_trend_slope) <= max_drift_slope)
) else "FAIL"

if master_vs_a["Assessment"] == "FAIL" or reference_vs_a["Assessment"] == "FAIL" or internal_sync == "FAIL":
    pass_fail = "FAIL"
elif not source_b_comparison_valid:
    pass_fail = "PASS WITH WARNINGS"
else:
    pass_fail = "PASS"

external_consensus_values = [entry["External_Consensus"] for entry in data if entry["External_Valid"]]
external_consensus_mean = safe_mean(external_consensus_values)
external_disagreement_mean = safe_mean(external_disagreements)
external_disagreement_max = max(external_disagreements)
external_disagreement_std = safe_std(external_disagreements)

report = f"""
Executive Summary
Overall Result: {pass_fail}

Test Period: {first_timestamp} - {last_timestamp}

Master server: {master_server}
Reference Server: {reference_server}
External source 1: {external_pool_1}
External source 2: {external_pool_2}

Test Duration: {test_duration}
Number of tests: {number_of_tests}

Endpoint Drift Comparison: {external_pool_1}
Master Server
Initial Error: {format_value(master_vs_a['Initial Error'])}
Final Error: {format_value(master_vs_a['Final Error'])}
Net Endpoint Drift: {format_value(master_vs_a['Endpoint Drift'])}
Trend Slope: {format_value(master_vs_a['Trend Slope'])}
Assessment: {master_vs_a['Assessment']}

Reference Server
Initial Error: {format_value(reference_vs_a['Initial Error'])}
Final Error: {format_value(reference_vs_a['Final Error'])}
Net Endpoint Drift: {format_value(reference_vs_a['Endpoint Drift'])}
Trend Slope: {format_value(reference_vs_a['Trend Slope'])}
Assessment: {reference_vs_a['Assessment']}

Endpoint Drift Comparison: {external_pool_2}
Master Server
Initial Error: {format_value(master_vs_b['Initial Error'])}
Final Error: {format_value(master_vs_b['Final Error'])}
Net Endpoint Drift: {format_value(master_vs_b['Endpoint Drift'])}
Trend Slope: {format_value(master_vs_b['Trend Slope'])}
Assessment: {master_vs_b['Assessment']}

Reference Server
Initial Error: {format_value(reference_vs_b['Initial Error'])}
Final Error: {format_value(reference_vs_b['Final Error'])}
Net Endpoint Drift: {format_value(reference_vs_b['Endpoint Drift'])}
Trend Slope: {format_value(reference_vs_b['Trend Slope'])}
Assessment: {reference_vs_b['Assessment']}

Internal Synchronization
Initial Difference: {format_value(internal_differences[0])}
Final Difference: {format_value(internal_differences[-1])}
Net Separation Change: {format_value(internal_endpoint_drift)}
Maximum Separation: {format_value(internal_max)}
Trend Slope: {format_value(internal_trend_slope)}
Assessment: {internal_sync}

External Reference Consistency
Eligible two-source consensus samples: {number_of_tests - len(invalid_external_series)}
Rejected two-source consensus samples: {len(invalid_external_series)}
Average Consensus Value: {format_value(external_consensus_mean)}
Average Absolute Disagreement: {format_value(external_disagreement_mean)}
Maximum Absolute Disagreement: {format_value(external_disagreement_max)}
Disagreement Standard Deviation: {format_value(external_disagreement_std)}
Rejected Series: {invalid_external_series if invalid_external_series else 'None'}

Findings and Conclusions:
"""

if master_vs_a["Assessment"] == "PASS" and reference_vs_a["Assessment"] == "PASS":
    report += (
        f"The Master and Reference servers passed the endpoint-drift comparison relative to {external_pool_1}. "
        f"Their respective endpoint changes were {abs(master_vs_a['Endpoint Drift']):.6f} and "
        f"{abs(reference_vs_a['Endpoint Drift']):.6f} seconds, within the configured 0.005000-second thresholds.\n"
    )
else:
    report += f"At least one server failed the endpoint-drift comparison relative to {external_pool_1}.\n"

if source_b_comparison_valid:
    if master_vs_b["Assessment"] == "PASS" and reference_vs_b["Assessment"] == "PASS":
        report += f"The Master and Reference servers passed the endpoint-drift comparison relative to {external_pool_2}.\n"
    else:
        report += f"At least one server failed the endpoint-drift comparison relative to {external_pool_2}.\n"
else:
    report += (
        f"The Master and Reference endpoint-drift comparisons relative to {external_pool_2} were inconclusive because "
        f"its selected measurements disagreed with {external_pool_1} beyond the 0.005000-second external-consistency threshold "
        f"in series {invalid_external_series}.\n"
    )

if internal_sync == "PASS":
    report += "The Master-to-Reference endpoint separation remained within the configured synchronization thresholds.\n"
else:
    report += "The Master-to-Reference measurements exceeded a configured synchronization threshold.\n"

report += "\n(review.py version 1.1)\n"

print(report)

if os.path.exists("final_report.txt"):
    os.remove("final_report.txt")

with open("final_report.txt", "w", encoding="utf-8") as outfile:
    outfile.write(report)

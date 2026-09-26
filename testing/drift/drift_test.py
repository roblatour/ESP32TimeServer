#
# Copyright Rob Latour, 2026
# License: MIT
# Website: https://github.com/roblatour/ESP32TimeServer
#

import subprocess
import os
import time
from datetime import datetime, timedelta
import sys

# Global variables with default values
External_Pool_1 = "time.nrc.ca"
External_Pool_2 = "0.ca.pool.ntp.org"
ESP32TimerServer_master = "192.168.7.24"
ESP32TimeServer_reference = "192.168.1.24"
overall_report_duration_minutes = 180

# Function to check if a value is a valid integer
def is_valid_integer(value):
    try:
        int(value)
        return True
    except ValueError:
        return False

# Validate overall_report_duration_minutes
if not is_valid_integer(overall_report_duration_minutes):
    print("Error: overall_report_duration_minutes is not numeric.")
    sys.exit(1)

if overall_report_duration_minutes != int(overall_report_duration_minutes):
    print("Error: overall_report_duration_minutes is not an integer.")
    sys.exit(1)

if overall_report_duration_minutes < 30:
    print("Error: overall_report_duration_minutes is less than 30.")
    sys.exit(1)

if overall_report_duration_minutes > 2880:
    print("Error: overall_report_duration_minutes is greater than 2880 (2 days).")
    sys.exit(1)

number_of_test_series = overall_report_duration_minutes // 30 + 1
pause_between_series = 30 * 60
estimated_duration_minutes = (number_of_test_series - 1) * 30

# Define the report file name
report_file = "gathered_data.txt"

# Check if the report file exists and delete it if it does
if os.path.exists(report_file):
    os.remove(report_file)

# Create a new empty report file
with open(report_file, 'w') as file:
    pass

# Function to run the ntpdate command and append output to the report file
def run_ntpdate_and_log(server, report_file):
    try:
        # Run the ntpdate command
        result = subprocess.run(['ntpdate', '-q', server], capture_output=True, text=True, check=True)
        # Append the output to the report file
        with open(report_file, 'a') as file:
            file.write(f"ntpdate -q {server}\n")
            file.write(result.stdout)
            file.write("\n")
    except subprocess.CalledProcessError as e:
        # Handle errors in command execution
        with open(report_file, 'a') as file:
            file.write(f"ntpdate -q {server} failed with error:\n")
            file.write(e.stderr)
            file.write("\n")

# Main test series loop
start_datetime = datetime.now()
start_time = time.monotonic()
estimated_completion_time = start_datetime + timedelta(minutes=estimated_duration_minutes)
print(f"Estimated completion time: {estimated_completion_time.strftime('%Y-%m-%d %H:%M:%S')}")
for series_number in range(1, number_of_test_series + 1):
    # Append test series header to the report file
    with open(report_file, 'a') as file:
        file.write(f"**** Test Series {series_number} ****\n")

    # Run the ntpdate commands for each server
    run_ntpdate_and_log(External_Pool_1, report_file)
    run_ntpdate_and_log(External_Pool_2, report_file)
    run_ntpdate_and_log(ESP32TimerServer_master, report_file)
    run_ntpdate_and_log(ESP32TimeServer_reference, report_file)

    if series_number < number_of_test_series:
        next_test_time = start_datetime + timedelta(seconds=series_number * pause_between_series)
        print(f"Test {series_number} of {number_of_test_series} completed.  Next test will be at {next_test_time.strftime('%Y-%m-%d %H:%M:%S')}.")
        time.sleep(max(0, start_time + series_number * pause_between_series - time.monotonic()))
    else:
        print(f"Test {series_number} of {number_of_test_series} completed.")

print(f"Tests complete, results available within {os.path.abspath(report_file)}")

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
overall_report_duration_minutes = 120
number_of_test_series = 4

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

# Validate number_of_test_series
if not is_valid_integer(number_of_test_series):
    print("Error: number_of_test_series is not numeric.")
    sys.exit(1)

if number_of_test_series != int(number_of_test_series):
    print("Error: number_of_test_series is not an integer.")
    sys.exit(1)

if number_of_test_series < 2:
    print("Error: number_of_test_series is less than 2.")
    sys.exit(1)

# Calculate pause between test series
pause_between_series = (overall_report_duration_minutes * 60) / number_of_test_series

# Calculate the number of tests per hour
tests_per_hour = (number_of_test_series * 60) / overall_report_duration_minutes

# Validate that more than 2 tests would not be run an hour
# see https://www.ntppool.org/tos.html item 4 b
if tests_per_hour > 2:
    print("Error: More than 2 tests would be run an hour.")
    sys.exit(1)

# Calculate estimated completion time
estimated_completion_time = datetime.now() + timedelta(minutes=overall_report_duration_minutes)

# Define the report file name
report_file = "gathered_data.txt"

# Print estimated completion time
print(f"Estimated completion time: {estimated_completion_time.strftime('%Y-%m-%d %H:%M:%S')}")

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
for series_number in range(1, number_of_test_series + 1):
    # Append test series header to the report file
    with open(report_file, 'a') as file:
        file.write(f"**** Test Series {series_number} ****\n")

    # Run the ntpdate commands for each server
    run_ntpdate_and_log(External_Pool_1, report_file)
    run_ntpdate_and_log(External_Pool_2, report_file)
    run_ntpdate_and_log(ESP32TimerServer_master, report_file)
    run_ntpdate_and_log(ESP32TimeServer_reference, report_file)

    # Wait for the calculated pause between test series
    time.sleep(pause_between_series)

# The script will exit after completing all test series
print("All test series have been completed. Check the gathered_data.txt file for results.")

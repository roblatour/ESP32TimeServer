# ESP32TimeServer Drift Test Instructions

## Setup and usage

### Install Python

**Windows:**
```bash
winget install --id Python.Python.3.14 -e
```

**Linux:**  
Look up the command for your distribution. For example:
```bash
sudo apt install python3
```

### Install Needed Libraries

Both Windows and Linux:
```bash
pip install numpy
```
```bash
pip install scipy
```

---

## Usage Notes

1. Flash two ESP32TimerServers with the release of the ESP32TimeServer software you wish to test.
   - One ESP32TimeServer (**master**) must be used to discipline the clock on the computer on which the drift test runs.
   - The second ESP32TimeServer (**reference**) is used as a reference server.

2. Edit the `drift.py` Python test script and update the values for your system and desired test scenario:
   ```python
   External_Pool_1 = "time.nrc.ca"
   External_Pool_2 = "0.ca.pool.ntp.org"
   ESP32TimerServer_master = "192.168.7.24"
   ESP32TimeServer_reference = "192.168.1.24"
   overall_report_duration_minutes = 120
   number_of_test_series = 8
   ```

3. Stop any system-wide network time service (such as with OPNsense).

4. Ensure the computer which will be running the drift test is disciplined by the ESP32TimerServer.

5. Restart the time server service on the computer that will be running the drift test  
   *(this ensures a fresh time sync with the ESP32TimerServer_master)*.

6. Run the drift test:
   ```bash
   py drift_test.py
   ```
   - When run, it will immediately show you when the test is expected to conclude.
   - This will (re)create a file called `gathered_data.txt`.
   - When complete, the script will show you that it is complete.

7. When the drift test concludes, restart any system-wide network time service stopped in step 3 above.

8. Run the review script:
   ```bash
   py review.py
   ```
   - This will provide a review and analysis of the data found in the `gathered_data.txt` file.
   - Results will be (re)created in a file called `final_report.txt`.
   - Additionally, the script will show you those results.
 


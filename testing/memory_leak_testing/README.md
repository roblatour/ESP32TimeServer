# Memory Leak Test Instructions

## Prerequisites

- Have an MQTT broker running (such as [Eclipse Mosquitto](https://mosquitto.org/))
- Have a way to connect to it to see published results (such as [MQTT Explorer](https://mqtt-explorer.com/))

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
     - `MQTTReportingPeriod` to a workable time frame for examining results, such as `120` (2 minutes)

2. **Build, flash and physically set up the ESP32TimeServer**

3. **View the published results with a tool such as MQTT Explorer**

   The reported values in the memory section should be expected to:

   - change between the first and second publication, but then
   - remain constant over time after the second publication.

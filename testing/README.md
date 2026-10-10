# Ways to test an NTP Time Server

## Functional testing

### Internal functional health checks

ESP32TimeServer has a built in self health check logic which is disabled by default.  Enabling it allows the program to basically run tests on itself at startup - for example it will send itself an NTP request, receive and process it as it would any other external request, once done it will verify ther  results - including metrics accumulated for MQTT reporting.  In all there are 37 such tests.  

To enable a self health check set `STARTUP_HEALTH_TEST_ENABLED` to `1` near the top
of the `main.cpp` file. Once done, at startup the program will run its health tests and report the results in the console log.

This feature is disable by default as it is primary purpose is to help ensure, pre-release, code changes development do not introduce  unexpect regression issues.  Also it adds time to startup time delaying the point at which the server can come online (be open for business).

Of note: results may be misleading depending setup. For example, health checks may 'Pass' all
IPv4 tests yet 'Fail' all IPv6 tests, not because of a problem with the program but because
the network it is running on doesn't support IPv6.

### External functional testing

[ESP32TimeServerTester](https://github.com/roblatour/ESP32TimeServerTester) is an independent functional NTP server testing tool.  It tests much of the same functionality as the Health check tests do, but from an external standpoint.

### RFC 9769 compliance testing
RFC 9769 compliance testing ensures network time synchronization implements the RFC 9769 NTP interleaved modes specification for NTPv4 replies. While this testing is covered off by two testing methods above, a python testing tool is also available to specifically test RFC 9769 compliance, for more information please see: [/testing/RFC_9769_compliance/README.md](/testing/RFC_9769_compliance/README.md)


## Memory Leak Testing
A memory leak occurs when a program allocates memory but fails to release it when no longer needed. Over time, leaked memory can degrade performance and potentially cause application failures.  For more information please see: [/testing/memory_leak_testing/README.md](/testing/memory_leak_testing/README.md)

## Stress Testing
Stress testing evaluates how a program performs under extreme workloads or resource constraints beyond normal operating conditions. It helps identify stability issues, bottlenecks, and failure points. For more information please see this open source stress testing tool: [Time Server Stress Test](https://github.com/roblatour/TimeServerStressTest)

## Quality of Service

### Jitter
Jitter is the variation in network delay between NTP messages exchanged with a time server. High jitter can reduce the accuracy and stability of time synchronization. For more information please see: [/testing/jitter/README.md](/testing/jitter/README.md)

### Drift
Drift is the gradual deviation of a computer's clock from the correct reference time due to hardware clock inaccuracies. NTP compensates for drift by regularly synchronizing the system clock with a time source. For more information and a python based drift testing tool please see: [/testing/drift/README.md](/testing/drift/README.md)


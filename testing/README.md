# Overview of what's contained in the sub-folders below

## **Guidance and testing tools, or links to them, for determining / testing the following** (in the context of this proejct):

## 1. Network Time Protocol (NTP)

### 1.1 Jitter
Jitter is the variation in network delay between NTP messages exchanged with a time server. High jitter can reduce the accuracy and stability of time synchronization.

### 1.2 Drift
Drift is the gradual deviation of a computer's clock from the correct reference time due to hardware clock inaccuracies. NTP compensates for drift by regularly synchronizing the system clock with a time source.

### 1,3 RFC 9769 compliance
RFC 9769 compliance refers to ensuring network time synchronization implements the RFC 9769 NTP Interleaved Modes specification (in ESP32TimeServer's case, for NTPv3 and NTPv4 replies).  

## 2. Program Testing
 
### 2.1 Stress Testing
Stress testing evaluates how a program performs under extreme workloads or resource constraints beyond normal operating conditions. It helps identify stability issues, bottlenecks, and failure points.

### 2.2 Memory Leak Testing
A memory leak occurs when a program allocates memory but fails to release it when no longer needed. Over time, leaked memory can degrade performance and potentially cause application failures.
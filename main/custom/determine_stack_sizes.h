// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#pragma once

#include "ESP32TimeServerSettings.h"

// The following conditional compile flag is used to determine the ideal stack sizes for
// xTaskCreatePinnedToCore calls used throughout the program.
//
// Unless you are changing the code associated with options below and need to know the impacts of
// those changes have on the stack sizes then the conditional compile flags below should be disabled

#ifndef CALCULATE_STACK_SIZES_ENABLED
#define CALCULATE_STACK_SIZES_ENABLED 0 // 0 = Disabled; 1 = Enabled
#endif

// FreeRTOS task stack allocations, in bytes.

static constexpr size_t Ethernet_Transport_Recovery_Task_Stack_Size = 2573;
static constexpr size_t GNSS_Recovery_Task_Stack_Size = 12288;
static constexpr size_t GNSS_Time_Sync_Task_Stack_Size = 2798;
static constexpr size_t Hardware_NTP_Server_Task_Stack_Size = 2719;
static constexpr size_t LED_LCD_Button_Task_Stack_Size = 4096; // keep this above the measured high water mark of 3276;
static constexpr size_t Ethernet_Setup_Task_Stack_Size = 4096;
// PTP MQTT snapshots now include up to 100 MAC records; remeasure on device.
static constexpr size_t MQTT_Service_Task_Stack_Size = 3512 + (PTP_ENABLED ? 1024 : 0);
static constexpr size_t NTP_Cache_Purge_Task_Stack_Size = 2266;
static constexpr size_t NTP_Server_Task_Stack_Size = 3560;
static constexpr size_t OTE_Service_Task_Stack_Size = 3560 + (PTP_ENABLED ? 1024 : 0);
static constexpr size_t PPS_Discipline_Task_Stack_Size = 3584; // was 3135: the PTP servo runs in this task (measure again)
#if PTP_L2_SERVER_ENABLED
static constexpr size_t PTP_L2_Service_Task_Stack_Size = 4096;
#endif
#if PTP_L3_SERVER_ENABLED
static constexpr size_t PTP_L3_UDP_Service_Task_Stack_Size = 6144;
#endif

static constexpr unsigned int Default_Safety_Margin_Percent = 30; // unless otherwise specified add this percentage to the highest stack usage as a safety margin

#if CALCULATE_STACK_SIZES_ENABLED

// For the purposes of calculating stack sizes force DEBUG_ENABLED if any of the stack size calculations are enabled
// so that the task includes all debug processing is included in the stack size calculations
#undef DEBUG_ENABLED
#define DEBUG_ENABLED 1

// For the purposes of calculating stack sizes force all MQTT tasks as enabled if MQTT_ENABLED is enabled
#if MQTT_ENABLED
#undef MQTT_CLIENT_REPORTING_ENABLED
#define MQTT_CLIENT_REPORTING_ENABLED 1
#undef MQTT_MEMORY_REPORTING_ENABLED
#define MQTT_MEMORY_REPORTING_ENABLED 1
#undef MQTT_HISTORICAL_REPORTING_ENABLED
#define MQTT_HISTORICAL_REPORTING_ENABLED 1
#endif

typedef enum
{
    Ethernet_Transport_Recovery,
    GNSS_Recovery,
    GNSS_Time_Sync,
    Hardware_NTP_Server,
    LED_LCD_Button,
    MQTT_Service,
    NTP_Cache_Purge,
    NTP_Server,
    OTE_Service,
    PPS_Discipline,
#if PTP_L2_SERVER_ENABLED
    PTP_L2_Service,
#endif
#if PTP_L3_SERVER_ENABLED
    PTP_L3_UDP_Service,
#endif
} TaskToPinToCore_t;

void setup_task_stack_usage_mutex();
void report_current_task_stack_usage(TaskToPinToCore_t task_id);

#endif

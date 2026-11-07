// ESP32 Time Server
// Copyright Rob Latour, 2026
// License: MIT
// Website: https://github.com/roblatour/ESP32TimeServer
//

#include "determine_stack_sizes.h"

#if CALCULATE_STACK_SIZES_ENABLED
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "main_cpp";

static size_t Highest_Ethernet_Transport_Recovery_Task_Stack_Size = 0;
static size_t Highest_GNSS_Recovery_Task_Stack_Size = 0;
static size_t Highest_GNSS_Time_Sync_Task_Stack_Size = 0;
static size_t Highest_Hardware_NTP_Server_Task_Stack_Size = 0;
static size_t Highest_LED_LCD_Button_Task_Stack_Size = 0;
static size_t Highest_MQTT_Service_Task_Stack_Size = 0;
static size_t Highest_NTP_Cache_Purge_Task_Stack_Size = 0;
static size_t Highest_NTP_Server_Task_Stack_Size = 0;
static size_t Highest_OTE_Service_Task_Stack_Size = 0;
static size_t Highest_PPS_Discipline_Task_Stack_Size = 0;
#if PTP_L2_SERVER_ENABLED
static size_t Highest_PTP_L2_Service_Task_Stack_Size = 0;
#endif
#if PTP_L3_SERVER_ENABLED
static size_t Highest_PTP_L3_UDP_Service_Task_Stack_Size = 0;
#endif

static SemaphoreHandle_t s_task_stack_usage_mutex = nullptr;

void setup_task_stack_usage_mutex()
{
    s_task_stack_usage_mutex = xSemaphoreCreateMutex();
}

void report_current_task_stack_usage(TaskToPinToCore_t task_id)
{
    xSemaphoreTake(s_task_stack_usage_mutex, portMAX_DELAY);

    const char *task_name = nullptr;
    size_t allocated_bytes = 0;
    unsigned int safety_margin_percent = 30;
    bool highest_value_changed = false;
    bool show_details = false;

    switch (task_id)
    {
    case Ethernet_Transport_Recovery:
        task_name = "ethernet_transport_task";
        allocated_bytes = Ethernet_Transport_Recovery_Task_Stack_Size;
        break;
    case GNSS_Recovery:
        task_name = "gnss_recovery_task";
        allocated_bytes = GNSS_Recovery_Task_Stack_Size;
        break;
    case GNSS_Time_Sync:
        task_name = "gnss_time_sync_task";
        allocated_bytes = GNSS_Time_Sync_Task_Stack_Size;
        break;
    case Hardware_NTP_Server:
        task_name = "hardware_ntp_server_task";
        allocated_bytes = Hardware_NTP_Server_Task_Stack_Size;
        break;
    case LED_LCD_Button:
        task_name = "LED_LCD_Button_task";
        allocated_bytes = LED_LCD_Button_Task_Stack_Size;
        safety_margin_percent = 70;
        break;
    case MQTT_Service:
        task_name = "mqtt_service_task";
        allocated_bytes = MQTT_Service_Task_Stack_Size;
        break;
    case NTP_Cache_Purge:
        task_name = "ntp_cache_purge_task";
        allocated_bytes = NTP_Cache_Purge_Task_Stack_Size;
        break;
    case NTP_Server:
        task_name = "ntp_server_task";
        allocated_bytes = NTP_Server_Task_Stack_Size;
        safety_margin_percent = 50;
        break;
    case OTE_Service:
        task_name = "ote_service_task";
        allocated_bytes = OTE_Service_Task_Stack_Size;
        break;
    case PPS_Discipline:
        task_name = "pps_discipline_task";
        allocated_bytes = PPS_Discipline_Task_Stack_Size;
        break;
#if PTP_L2_SERVER_ENABLED
    case PTP_L2_Service:
        task_name = "ptp_service_task";
        allocated_bytes = PTP_L2_Service_Task_Stack_Size;
        break;
#endif
#if PTP_L3_SERVER_ENABLED
    case PTP_L3_UDP_Service:
        task_name = "ptp_l3_udp_service_task";
        allocated_bytes = PTP_L3_UDP_Service_Task_Stack_Size;
        break;
#endif
    default:
        task_name = "unknown_task";
        break;
    }

    // calculate the suggested stack size based on the peak usage and safety margin

    const size_t high_watermark_bytes = uxTaskGetStackHighWaterMark(nullptr);
    const size_t peak_usage_bytes = allocated_bytes > high_watermark_bytes ? allocated_bytes - high_watermark_bytes : 0;
    const size_t curent_suggestion = (peak_usage_bytes * (100U + safety_margin_percent) + 99U) / 100U;

    if (show_details)
    {
        ESP_LOGI(task_name,
                 "Stack report: Allocated=%u bytes, HighWater=%u bytes unused, PeakUsage=%u bytes, Suggested=%u bytes",
                 static_cast<unsigned int>(allocated_bytes),
                 static_cast<unsigned int>(high_watermark_bytes),
                 static_cast<unsigned int>(peak_usage_bytes),
                 static_cast<unsigned int>(curent_suggestion));
    }

    if (curent_suggestion == 0)
        ESP_LOGE(task_name, "Error: Suggested stack size is 0 bytes. Try increasing the allocted bytes of %u", static_cast<unsigned int>(allocated_bytes));

    switch (task_id)
    {
    case Ethernet_Transport_Recovery:
        if (curent_suggestion > Highest_Ethernet_Transport_Recovery_Task_Stack_Size)
        {
            Highest_Ethernet_Transport_Recovery_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case GNSS_Recovery:
        if (curent_suggestion > Highest_GNSS_Recovery_Task_Stack_Size)
        {
            Highest_GNSS_Recovery_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case GNSS_Time_Sync:
        if (curent_suggestion > Highest_GNSS_Time_Sync_Task_Stack_Size)
        {
            Highest_GNSS_Time_Sync_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case Hardware_NTP_Server:
        if (curent_suggestion > Highest_Hardware_NTP_Server_Task_Stack_Size)
        {
            Highest_Hardware_NTP_Server_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case LED_LCD_Button:
        if (curent_suggestion > Highest_LED_LCD_Button_Task_Stack_Size)
        {
            Highest_LED_LCD_Button_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case MQTT_Service:
        if (curent_suggestion > Highest_MQTT_Service_Task_Stack_Size)
        {
            Highest_MQTT_Service_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case NTP_Cache_Purge:
        if (curent_suggestion > Highest_NTP_Cache_Purge_Task_Stack_Size)
        {
            Highest_NTP_Cache_Purge_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case NTP_Server:
        if (curent_suggestion > Highest_NTP_Server_Task_Stack_Size)
        {
            Highest_NTP_Server_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
    case OTE_Service:
        if (curent_suggestion > Highest_OTE_Service_Task_Stack_Size)
        {
            Highest_OTE_Service_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }

        break;
    case PPS_Discipline:
        if (curent_suggestion > Highest_PPS_Discipline_Task_Stack_Size)
        {
            Highest_PPS_Discipline_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
#if PTP_L2_SERVER_ENABLED
    case PTP_L2_Service:
        if (curent_suggestion > Highest_PTP_L2_Service_Task_Stack_Size)
        {
            Highest_PTP_L2_Service_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
#endif
#if PTP_L3_SERVER_ENABLED
    case PTP_L3_UDP_Service:
        if (curent_suggestion > Highest_PTP_L3_UDP_Service_Task_Stack_Size)
        {
            Highest_PTP_L3_UDP_Service_Task_Stack_Size = curent_suggestion;
            highest_value_changed = true;
        }
        break;
#endif
    default:
        break;
    }

    // Report the highest suggested stack sizes for all tasks to date in such a way that they can be easily copied into the code above
    // If no new highest value has yet been reported for a task the value shown will be 0, the old value will remain as the comment
    if (highest_value_changed)
    {
        ESP_LOGI(TAG, "Suggested updated task stack sizes:");
#if PTP_L3_SERVER_ENABLED
        ESP_LOGI(TAG, "PTP UDP service stack: suggested=%u allocated=%u",
                 static_cast<unsigned>(Highest_PTP_L3_UDP_Service_Task_Stack_Size),
                 static_cast<unsigned>(PTP_L3_UDP_Service_Task_Stack_Size));
#endif
#if PTP_L2_SERVER_ENABLED
        ESP_LOGI(TAG, "PTP service stack: suggested=%u allocated=%u",
                 static_cast<unsigned>(Highest_PTP_L2_Service_Task_Stack_Size),
                 static_cast<unsigned>(PTP_L2_Service_Task_Stack_Size));
#endif
        ESP_LOGI(TAG, " ");

        if (Highest_Ethernet_Transport_Recovery_Task_Stack_Size > Ethernet_Transport_Recovery_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t Ethernet_Transport_Recovery_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_Ethernet_Transport_Recovery_Task_Stack_Size), static_cast<unsigned int>(Ethernet_Transport_Recovery_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t Ethernet_Transport_Recovery_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_Ethernet_Transport_Recovery_Task_Stack_Size), static_cast<unsigned int>(Ethernet_Transport_Recovery_Task_Stack_Size));

        if (Highest_GNSS_Recovery_Task_Stack_Size > GNSS_Recovery_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t GNSS_Recovery_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_GNSS_Recovery_Task_Stack_Size), static_cast<unsigned int>(GNSS_Recovery_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t GNSS_Recovery_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_GNSS_Recovery_Task_Stack_Size), static_cast<unsigned int>(GNSS_Recovery_Task_Stack_Size));

        if (Highest_GNSS_Time_Sync_Task_Stack_Size > GNSS_Time_Sync_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t GNSS_Time_Sync_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_GNSS_Time_Sync_Task_Stack_Size), static_cast<unsigned int>(GNSS_Time_Sync_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t GNSS_Time_Sync_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_GNSS_Time_Sync_Task_Stack_Size), static_cast<unsigned int>(GNSS_Time_Sync_Task_Stack_Size));

        if (Highest_Hardware_NTP_Server_Task_Stack_Size > Hardware_NTP_Server_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t Hardware_NTP_Server_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_Hardware_NTP_Server_Task_Stack_Size), static_cast<unsigned int>(Hardware_NTP_Server_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t Hardware_NTP_Server_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_Hardware_NTP_Server_Task_Stack_Size), static_cast<unsigned int>(Hardware_NTP_Server_Task_Stack_Size));

        if (Highest_LED_LCD_Button_Task_Stack_Size > LED_LCD_Button_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t LED_LCD_Button_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_LED_LCD_Button_Task_Stack_Size), static_cast<unsigned int>(LED_LCD_Button_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t LED_LCD_Button_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_LED_LCD_Button_Task_Stack_Size), static_cast<unsigned int>(LED_LCD_Button_Task_Stack_Size));

        if (Highest_MQTT_Service_Task_Stack_Size > MQTT_Service_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t MQTT_Service_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_MQTT_Service_Task_Stack_Size), static_cast<unsigned int>(MQTT_Service_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t MQTT_Service_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_MQTT_Service_Task_Stack_Size), static_cast<unsigned int>(MQTT_Service_Task_Stack_Size));

        if (Highest_NTP_Cache_Purge_Task_Stack_Size > NTP_Cache_Purge_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t NTP_Cache_Purge_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_NTP_Cache_Purge_Task_Stack_Size), static_cast<unsigned int>(NTP_Cache_Purge_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t NTP_Cache_Purge_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_NTP_Cache_Purge_Task_Stack_Size), static_cast<unsigned int>(NTP_Cache_Purge_Task_Stack_Size));

        if (Highest_NTP_Server_Task_Stack_Size > NTP_Server_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t NTP_Server_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_NTP_Server_Task_Stack_Size), static_cast<unsigned int>(NTP_Server_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t NTP_Server_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_NTP_Server_Task_Stack_Size), static_cast<unsigned int>(NTP_Server_Task_Stack_Size));

        if (Highest_OTE_Service_Task_Stack_Size > OTE_Service_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t OTE_Service_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_OTE_Service_Task_Stack_Size), static_cast<unsigned int>(OTE_Service_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t OTE_Service_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_OTE_Service_Task_Stack_Size), static_cast<unsigned int>(OTE_Service_Task_Stack_Size));

        if (Highest_PPS_Discipline_Task_Stack_Size > PPS_Discipline_Task_Stack_Size)
            ESP_LOGW(TAG, "static constexpr size_t PPS_Discipline_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_PPS_Discipline_Task_Stack_Size), static_cast<unsigned int>(PPS_Discipline_Task_Stack_Size));
        else
            ESP_LOGI(TAG, "static constexpr size_t PPS_Discipline_Task_Stack_Size = %u; // %u;", static_cast<unsigned int>(Highest_PPS_Discipline_Task_Stack_Size), static_cast<unsigned int>(PPS_Discipline_Task_Stack_Size));

        ESP_LOGI(TAG, " ");
    };

    xSemaphoreGive(s_task_stack_usage_mutex);
}

#endif

idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_eth esp_event esp_netif esp_hw_support esp_timer nvs_flash driver lwip hd44780 pcf8574 arduino-esp32 SparkFun_u-blox_GNSS_v3)

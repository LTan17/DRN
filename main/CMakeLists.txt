idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES MPU9250 Fusion Wifi_Station app_mqtt esp_timer driver)

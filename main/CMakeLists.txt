idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES MPU9250 QMC5883L PID Fusion Control Wifi_Station app_mqtt esp_timer driver esp_driver_gpio)

idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES MPU9250 Fusion esp_timer driver)

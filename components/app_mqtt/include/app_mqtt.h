#pragma once

#include "mqtt_client.h"
#include "esp_err.h"

typedef struct
{
    uint16_t throttle;
    int8_t pitch, roll, yaw;
} remote_t;

esp_err_t MQTT_Init();
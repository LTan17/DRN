#pragma once
#include "esp_timer.h"
#include "app_mqtt.h"
#include "math.h"
#include "common_struct.h"

typedef struct
{
    float target_X;
    float target_Y;
    float target_Z;
} Target_t;

typedef struct
{
    float kp, ki, kd;
    float integral;
    float pre_error;
    float limit_integral, limit_output;
} PID_Controller_t;

vector_t PID(remote_t *data, vector_t *gyro, vector_t *angle);
static Target_t PID_Angle(Target_t *target_angle, vector_t *gyro, vector_t *angle, float dt);
static vector_t PID_Gyro(Target_t *rate, vector_t *gyro, vector_t *angle, float dt);

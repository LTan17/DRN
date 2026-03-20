#pragma once
#include "MPU9250.h"
#include "math.h"

#define RAD_TO_DEG 57.29578f

vector_t process_imu_angles(vector_t acc, vector_t gyr, double dt);
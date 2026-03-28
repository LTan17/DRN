#pragma once
#include "math.h"
#include "common_struct.h"

#define RAD_TO_DEG 57.29578f

vector_t process_imu_angles(vector_t acc, vector_t gyr, vector_t mag, double dt);
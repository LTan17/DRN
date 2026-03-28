#include <stdio.h>
#include "Fusion.h"

vector_t process_imu_angles(vector_t acc, vector_t gyr, vector_t mag, double dt)
{
    static const float TAU = 0.1f;
    static vector_t local_angle = {0, 0, 0};
    float alpha = TAU / (TAU + dt);
    float roll_deg = atan2f(acc.y, acc.z) * RAD_TO_DEG;
    float pitch_deg = atan2f(-acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)) * RAD_TO_DEG;
    float yaw_deg = atan2f(mag.x, mag.y) * RAD_TO_DEG;
    local_angle.x = alpha * (local_angle.x + gyr.x * dt) + (1.0f - alpha) * roll_deg;
    local_angle.y = alpha * (local_angle.y + gyr.y * dt) + (1.0f - alpha) * pitch_deg;
    local_angle.z = yaw_deg;
    return local_angle;
}
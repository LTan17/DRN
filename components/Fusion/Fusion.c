#include <stdio.h>
#include "Fusion.h"

vector_t process_imu_angles(vector_t acc, vector_t gyr, double dt)
{
    static const float TAU = 0.1f;
    float alpha = TAU / (TAU + dt);
    float acc_roll_deg = atan2f(acc.y, acc.z) * RAD_TO_DEG;
    static vector_t local_angle = {0, 0, 0};
    float acc_pitch_deg = atan2f(-acc.x, sqrtf(acc.y * acc.y + acc.z * acc.z)) * RAD_TO_DEG;
    local_angle.x = alpha * (local_angle.x + gyr.x * dt) + (1.0f - alpha) * acc_roll_deg;
    local_angle.y = alpha * (local_angle.y + gyr.y * dt) + (1.0f - alpha) * acc_pitch_deg;
    return local_angle;
}
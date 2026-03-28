#include <stdio.h>
#include "PID.h"

vector_t stop = {.x = -1, .y = -1, .z = -1};

bool set_first_yaw = false;
static float target_yaw = 0;

static float PID_Update(PID_Controller_t *pid, float target, float sensor_data, float dt)
{
    float error = target - sensor_data;
    float p_out = pid->kp * error;

    pid->integral += error * dt;
    float i_out = pid->ki * pid->integral;
    i_out = fmax(fmin(i_out, pid->limit_integral), -pid->limit_integral);

    float derivative = (error - pid->pre_error) / dt;
    float d_out = pid->kd * derivative;
    pid->pre_error = error;

    float output = p_out + i_out + d_out;
    output = fmax(fmin(output, pid->limit_output), -pid->limit_output);
    return output;
}

vector_t PID(remote_t *data, vector_t *gyro, vector_t *angle)
{
    static int64_t pre_time = 0;
    static int64_t cur_time = 0;
    float dt = 0.0;
    cur_time = esp_timer_get_time();
    if (pre_time == 0)
    {
        pre_time = cur_time;
        return stop;
    }
    dt = (float)(cur_time - pre_time) / 1000000.0f;
    pre_time = cur_time;
    if (fabsf(angle->x) > 50 || fabsf(angle->y) > 50)
    {
        return stop;
    }
    static Target_t target_angle;
    target_angle.target_X = data->roll;
    target_angle.target_Y = data->pitch;
    target_angle.target_Z = data->yaw;
    Target_t rate = PID_Angle(&target_angle, gyro, angle, dt);
    return PID_Gyro(&rate, gyro, angle, dt);
}

static Target_t PID_Angle(Target_t *target_angle, vector_t *gyro, vector_t *angle, float dt)
{
    static PID_Controller_t angleX_PID = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
        .integral = 0,
        .pre_error = 0,
        .limit_integral = 50,
        .limit_output = 200};

    static PID_Controller_t angleY_PID = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
        .integral = 0,
        .pre_error = 0,
        .limit_integral = 50,
        .limit_output = 200};

    static PID_Controller_t angleZ_PID = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
        .integral = 0,
        .pre_error = 0,
        .limit_integral = 50,
        .limit_output = 200};
    Target_t rate = {0, 0, 0};
    rate.target_X = PID_Update(&angleX_PID, target_angle->target_X, angle->x, dt);
    rate.target_Y = PID_Update(&angleY_PID, target_angle->target_Y, angle->y, dt);
    float yaw_error = target_angle->target_Z - angle->z;
    if (yaw_error > 180.0f)
    {
        yaw_error -= 360.0f;
    }
    else if (yaw_error < -180.0f)
    {
        yaw_error += 360.0f;
    }
    float smart_target_yaw = angle->z + yaw_error;
    rate.target_Z = PID_Update(&angleZ_PID, smart_target_yaw, angle->z, dt);
    return rate;
}

static vector_t PID_Gyro(Target_t *rate, vector_t *gyro, vector_t *angle, float dt)
{
    static PID_Controller_t GyroX_PID = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
        .integral = 0,
        .pre_error = 0,
        .limit_integral = 150,
        .limit_output = 200};

    static PID_Controller_t GyroY_PID = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
        .integral = 0,
        .pre_error = 0,
        .limit_integral = 150,
        .limit_output = 200};

    static PID_Controller_t GyroZ_PID = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
        .integral = 0,
        .pre_error = 0,
        .limit_integral = 150,
        .limit_output = 200};
    static float gyro_filter_X = 0;
    static float gyro_filter_Y = 0;
    static float gyro_filter_Z = 0;
    float alpha = 0.1f;

    gyro_filter_X = gyro_filter_X * (1.0 - alpha) + gyro->x * alpha;
    gyro_filter_Y = gyro_filter_Y * (1.0 - alpha) + gyro->y * alpha;
    gyro_filter_Z = gyro_filter_Z * (1.0 - alpha) + gyro->z * alpha;

    float GyrX_output = PID_Update(&GyroX_PID, rate->target_X, (float)gyro_filter_X, dt);
    float GyrY_output = PID_Update(&GyroY_PID, rate->target_Y, (float)gyro_filter_Y, dt);
    float GyrZ_output = PID_Update(&GyroZ_PID, rate->target_Z, (float)gyro_filter_Z, dt);

    vector_t PID_OutPut = {
        .x = GyrX_output,
        .y = GyrY_output,
        .z = GyrZ_output};
    return PID_OutPut;
}
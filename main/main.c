#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "MPU9250.h"
#include "Fusion.h"
#include "math.h"
#include "Wifi_Station.h"
#include "app_mqtt.h"

TaskHandle_t IMU_task_handle = NULL;

vector_t public_angle;

static bool IRAM_ATTR IMU_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;

    vTaskNotifyGiveFromISR(IMU_task_handle, &high_task_awoken);
    return (high_task_awoken == pdTRUE);
}

void IMU_Timer_Setup()
{
    gptimer_handle_t gptimer_handle = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_handle));
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 2000,
        .flags.auto_reload_on_alarm = true};
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_handle, &alarm_config));
    gptimer_event_callbacks_t cb = {
        .on_alarm = IMU_timer_cb};
    gptimer_register_event_callbacks(gptimer_handle, &cb, NULL);

    ESP_ERROR_CHECK(gptimer_enable(gptimer_handle));
    ESP_ERROR_CHECK(gptimer_start(gptimer_handle));
}

void IMU_task(void *parameter)
{
    int64_t now = 0, pre_time = 0;
    float dt = 0;

    vector_t gyr, acc, mag;
    vector_t raw_angle;
    int count = 0;

    MPU9250_Init();
    IMU_Timer_Setup();
    pre_time = esp_timer_get_time();

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        now = esp_timer_get_time();
        dt = (now - pre_time) / 1000000.0f;
        pre_time = now;
        MPU9250_ReadAccel(&acc);
        MPU9250_ReadGyro(&gyr);
        MPU9250_ReadMag(&mag);
        raw_angle = process_imu_angles(acc, gyr, dt);
        // while ((count++) % 10 == 0)
        // {
        //     printf(">dt:%.5f\n>gyrX:%.3f\n>gyrY:%.3f\n>gyrZ:%.3f\n",
        //            dt, gyr.x, gyr.y, gyr.z);
        //     printf(">angleX:%.3f\n>angleY:%.3f\n>angleZ:%.3f\n", raw_angle.x, raw_angle.y, raw_angle.z);
        // }
    }
}

void app_main(void)
{
    Wifi_Station_Init();
    vTaskDelay(pdMS_TO_TICKS(10000));
    MQTT_Init();
    xTaskCreate(IMU_task, "IMU_task", 4096, NULL, 15, &IMU_task_handle);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "freertos/task.h"
#include "MPU9250.h"
#include "QMC5883L.h"
#include "Fusion.h"
#include "math.h"
#include "Wifi_Station.h"
#include "app_mqtt.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "PID.h"
#include "Control.h"

#define TIMER_RESOLUTION 1000000

#define f_2000Hz 500
#define f_1000Hz 1000
#define f_500Hz 2000

typedef bool (*timer_cb)(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);

remote_t global_remote_msg;
SemaphoreHandle_t global_remote_msg_mtx = NULL;

vector_t global_angle;
SemaphoreHandle_t global_angle_mtx = NULL;

vector_t global_gyro;
SemaphoreHandle_t global_gyro_mtx = NULL;

vector_t global_PID_output;
SemaphoreHandle_t global_PID_output_mtx = NULL;

TaskHandle_t IMU_task_handle = NULL;
TaskHandle_t PID_task_handle = NULL;
TaskHandle_t Control_task_handle = NULL;

volatile bool mag_ready = false;

static bool IRAM_ATTR IMU_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
    vTaskNotifyGiveFromISR(IMU_task_handle, &high_task_awoken);
    return (high_task_awoken == pdTRUE);
}

static bool IRAM_ATTR PID_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
    vTaskNotifyGiveFromISR(PID_task_handle, &high_task_awoken);
    return (high_task_awoken == pdTRUE);
}

static bool IRAM_ATTR control_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;
    vTaskNotifyGiveFromISR(Control_task_handle, &high_task_awoken);
    return (high_task_awoken == pdTRUE);
}

void Timer_Setup(timer_cb func, int16_t count)
{
    gptimer_handle_t gptimer_handle = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer_handle));
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = count,
        .flags.auto_reload_on_alarm = true};
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer_handle, &alarm_config));
    gptimer_event_callbacks_t cb = {
        .on_alarm = func};
    gptimer_register_event_callbacks(gptimer_handle, &cb, NULL);

    ESP_ERROR_CHECK(gptimer_enable(gptimer_handle));
    ESP_ERROR_CHECK(gptimer_start(gptimer_handle));
}

static void IRAM_ATTR qmc5883l_drdy_handler(void *arg)
{
    mag_ready = true;
}

esp_err_t MAG_ITR(void)
{
    gpio_config_t gpio_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << QMC5883L_ITR),
        .intr_type = GPIO_INTR_POSEDGE};
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&gpio_cfg));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_add(QMC5883L_ITR, qmc5883l_drdy_handler, NULL));
    gpio_dump_io_configuration(stdout, (1ULL << QMC5883L_ITR));
    return ESP_OK;
}

void IMU_task(void *parameter)
{
    int64_t now = 0, pre_time = 0;
    float dt = 0;
    vector_t gyr, acc, mag;
    vector_t raw_angle;
    int count = 0;
    Timer_Setup(IMU_timer_cb, f_1000Hz);
    pre_time = esp_timer_get_time();

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        now = esp_timer_get_time();
        dt = (now - pre_time) / 1000000.0f;
        pre_time = now;
        MPU9250_ReadAccel(&acc);
        MPU9250_ReadGyro(&gyr);

        if (mag_ready)
        {
            QMC5883L_ReadMag(&mag);
            mag_ready = false;
        }
        raw_angle = process_imu_angles(acc, gyr, mag, dt);
        if (xSemaphoreTake(global_angle_mtx, 0) == pdTRUE)
        {
            memcpy(&global_angle, &raw_angle, sizeof(vector_t));
            xSemaphoreGive(global_angle_mtx);
        }
        // if ((count++) % 50 == 0)
        // {
        //     // printf(">dt:%.5f\n>gyrX:%.3f\n>gyrY:%.3f\n>gyrZ:%.3f\n",
        //     //        dt, gyr.x, gyr.y, gyr.z);
        //     // printf(">angleX:%.3f\n>angleY:%.3f\n>angleZ:%.3f\n", raw_angle.x, raw_angle.y, raw_angle.z);
        //     // printf(">dt:%.5f\n>magX:%.5f\n>magY:%.5f\n>magZ:%.5f\n", dt, mag.x, mag.y, mag.z);
        //     printf(">IMU: %.5f\n", dt);
        // }
    }
}

void PID_task(void *parameter)
{
    int64_t now = 0, pre_time = 0;
    float dt = 0;
    int count = 0;
    remote_t local_remote_msg;
    vector_t local_angle, local_gyro;
    Timer_Setup(PID_timer_cb, f_1000Hz);
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        now = esp_timer_get_time();
        dt = (now - pre_time) / 1000000.0f;
        pre_time = now;
        if (xSemaphoreTake(global_angle_mtx, 0) == pdTRUE)
        {
            memcpy(&local_angle, &global_angle, sizeof(vector_t));
            xSemaphoreGive(global_angle_mtx);
        }
        if (xSemaphoreTake(global_remote_msg_mtx, 0) == pdTRUE)
        {
            memcpy(&local_remote_msg, &global_remote_msg, sizeof(remote_t));
            xSemaphoreGive(global_remote_msg_mtx);
        }
        if (xSemaphoreTake(global_gyro_mtx, 0) == pdTRUE)
        {
            memcpy(&local_gyro, &global_gyro, sizeof(vector_t));
            xSemaphoreGive(global_gyro_mtx);
        }
        vector_t local_PID_output = PID(&local_remote_msg, &local_gyro, &local_angle);
        if (xSemaphoreTake(global_PID_output_mtx, 0))
        {
            memcpy(&global_PID_output, &local_PID_output, sizeof(vector_t));
            xSemaphoreGive(global_PID_output_mtx);
        }
        // if ((count++) % 50 == 0)
        // {
        //     printf(">PID: %.5f\n", dt);
        // }
    }
}

void Control_task(void *parameter)
{
    remote_t local_remote_msg;
    vector_t local_PID_output;
    uint16_t base_thottle;
    ESP_ERROR_CHECK_WITHOUT_ABORT(ESC_Init());
    ESC_Start();
    Timer_Setup(control_timer_cb, f_1000Hz);
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (xSemaphoreTake(global_remote_msg_mtx, 0) == pdTRUE)
        {
            memcpy(&local_remote_msg, &global_remote_msg, sizeof(remote_t));
            xSemaphoreGive(global_remote_msg_mtx);
        }
        if (xSemaphoreTake(global_PID_output_mtx, 0))
        {
            memcpy(&local_PID_output, &global_PID_output, sizeof(vector_t));
            xSemaphoreGive(global_PID_output_mtx);
        }
        base_thottle = local_remote_msg.throttle;
        printf("remote: pitch:%d, roll:%d, yaw:%d, throttle:%d\n ", local_remote_msg.pitch, local_remote_msg.roll, local_remote_msg.yaw, local_remote_msg.throttle);
        Control_Run(base_thottle, &local_PID_output);
    }
}

void app_main(void)
{
    MAG_ITR();
    MPU9250_Init();
    vTaskDelay(pdMS_TO_TICKS(2000));
    QMC5883L_Init();
    global_remote_msg_mtx = xSemaphoreCreateMutex();
    global_gyro_mtx = xSemaphoreCreateMutex();
    global_angle_mtx = xSemaphoreCreateMutex();
    global_PID_output_mtx = xSemaphoreCreateMutex();
    vTaskDelay(pdMS_TO_TICKS(100));
    Wifi_Station_Init();
    MQTT_Init();
    xTaskCreatePinnedToCore(IMU_task, "IMU_task", 4096, NULL, 15, &IMU_task_handle, 1);
    xTaskCreatePinnedToCore(PID_task, "PID_task", 2048, NULL, 15, &PID_task_handle, 1);
    xTaskCreatePinnedToCore(Control_task, "Control_task", 4096, NULL, 10, &Control_task_handle, 0);
    return;
}
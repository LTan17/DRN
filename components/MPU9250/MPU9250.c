#include <stdio.h>
#include "esp_log.h"
#include "MPU9250.h"
#include "bus_manager.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define MPU9250_SPEED 1 * 1000 * 1000
#define MAX_SPI_TRANSFER_SIZE 16
#define MAX_SPI_RECEIVE_SIZE 64

static float GyroSensitivity = -1;
static float AcceSensitivity = -1;
static const char *TAG = "MPU9250";

static spi_device_handle_t mpu9250_dev_handle;

esp_err_t MPU9250_Init(void)
{
    bus_Init();
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = MPU9250_SPEED,
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .clock_source = SPI_CLK_SRC_DEFAULT};
    esp_err_t ret = spi_bus_add_device(GY91_SPIHOST, &dev_cfg, &mpu9250_dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "add device fail");
        return ret;
    }
    MPU9250_DefaltSetUp();
    uint8_t who = -17;
    who = WhoAmI();
    ESP_LOGI(TAG, "WHO: %u", who);
    vTaskDelay(pdMS_TO_TICKS(pdMS_TO_TICKS(3000)));
    if ((who != 0x71) && (who != 0x73))
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static uint8_t WhoAmI(void)
{
    uint8_t ans = 0;
    MPU9250_ReadRes((uint8_t)WHOAMI, &ans, 1);
    return ans;
}

static void MPU9250_DefaltSetUp(void)
{
    MPU9250_Reset();
    vTaskDelay(pdMS_TO_TICKS(100));
    MPU9250_SelectClk();
    vTaskDelay(pdMS_TO_TICKS(10));
    MPU9250_Enable();
    vTaskDelay(pdMS_TO_TICKS(10));
    MPU9250_ConfigAcce();
    vTaskDelay(pdMS_TO_TICKS(10));
    MPU9250_ConfigGyro();
    vTaskDelay(pdMS_TO_TICKS(10));
    MPU9250_SelectSPIMode();
    vTaskDelay(pdMS_TO_TICKS(10));
}

static esp_err_t MPU9250_Select(void)
{
    gpio_set_level(MPU9250_CS, 0);
    gpio_set_level(BMP280_CS, 1);
    return ESP_OK;
}

static esp_err_t MPU9250_Unselect(void)
{
    gpio_set_level(MPU9250_CS, 1);
    return ESP_OK;
}

static void MPU9250_WriteRes(uint8_t res, uint8_t *send_data, size_t size)
{
    uint8_t buffer[MAX_SPI_TRANSFER_SIZE];
    buffer[0] = res & 0x7F;
    if (size > 0)
    {
        memcpy(&buffer[1], send_data, size);
    }
    MPU9250_Transfer(buffer, NULL, size + 1);
}

static void MPU9250_ReadRes(uint8_t res, uint8_t *rev_data, size_t size)
{
    uint8_t address = res | 0x80;
    MPU9250_TransmitReceive(&address, 1, rev_data, size);
}

static void MPU9250_Reset(void)
{
    uint8_t res = (uint8_t)POWMGMT_RES;
    uint8_t data = (uint8_t)H_RESET;
    MPU9250_WriteRes(res, &data, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}
static void MPU9250_SelectClk(void)
{
    uint8_t res = (uint8_t)POWMGMT_RES;
    uint8_t data = (uint8_t)CLK_SEL;
    MPU9250_WriteRes(res, &data, 1);
}

static void MPU9250_Enable(void)
{
    uint8_t res = (uint8_t)POWMGMT2_RES;
    uint8_t data = (uint8_t)ENABLE_ALL;
    MPU9250_WriteRes(res, &data, 1);
}

static void MPU9250_SelectSPIMode(void)
{
    uint8_t res = (uint8_t)UCRL_RES;
    uint8_t data = (uint8_t)DIS_I2C;
    MPU9250_WriteRes(res, &data, 1);
}

static void MPU9250_ConfigGyro(void)
{
    // Set DLPF
    uint8_t res = (uint8_t)CONFIG_RES;
    uint8_t data = (uint8_t)_92HZ;
    MPU9250_WriteRes(res, &data, 1);

    // Set Full Scale
    res = (uint8_t)GYROCFG_RES;
    data = (uint8_t)_1000dps;
    GyroSensitivity = (float)GYRO1000_SENSITIVYTI;
    MPU9250_WriteRes(res, &data, 1);
}

static void MPU9250_ConfigAcce(void)
{
    // Set DLPF
    uint8_t res = (uint8_t)ACCCFG_RES2;
    uint8_t data = (uint8_t)_41HZ;
    MPU9250_WriteRes(res, &data, 1);

    // Set Full Scale
    res = (uint8_t)ACCCFG_RES1;
    data = (uint8_t)_8g;
    AcceSensitivity = (float)ACC8_SENSITIVITY;
    MPU9250_WriteRes(res, &data, 1);
}

static esp_err_t MPU9250_Transfer(const uint8_t *tx_buffer, uint8_t *rx_buffer, size_t length)
{
    if (tx_buffer == NULL && rx_buffer == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = bus_Lock();
    if (ret != ESP_OK)
    {
        return ret;
    }

    MPU9250_Select();

    spi_transaction_t trans = {
        .length = length * 8,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
    };

    ret = spi_device_transmit(mpu9250_dev_handle, &trans);

    MPU9250_Unselect();
    bus_Unlock();

    return ret;
}

static esp_err_t MPU9250_TransmitReceive(const uint8_t *tx_buffer, size_t tx_length, uint8_t *rx_buffer, size_t rx_length)
{
    if (tx_buffer == NULL || tx_length == 0 || rx_buffer == NULL || rx_length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }
    size_t total_len = tx_length + rx_length;
    uint8_t buf_full[total_len];
    memcpy(buf_full, tx_buffer, tx_length);
    esp_err_t ret = bus_Lock();
    if (ret != ESP_OK)
    {
        return ret;
    }

    MPU9250_Select();

    spi_transaction_t trans = {
        .length = total_len * 8,
        .tx_buffer = buf_full,
        .rx_buffer = buf_full,
    };
    ret = spi_device_transmit(mpu9250_dev_handle, &trans);
    MPU9250_Unselect();
    bus_Unlock();
    if (ret == ESP_OK)
    {

        memcpy(rx_buffer, &buf_full[tx_length], rx_length);
    }
    return ret;
}

esp_err_t MPU9250_ReadAccel(vector_t *accel_data)
{
    if (accel_data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t accel_raw[6];
    MPU9250_ReadRes(ACCEL, accel_raw, 6);

    int16_t accel_x_raw = (int16_t)((accel_raw[0] << 8) | accel_raw[1]);
    int16_t accel_y_raw = (int16_t)((accel_raw[2] << 8) | accel_raw[3]);
    int16_t accel_z_raw = (int16_t)((accel_raw[4] << 8) | accel_raw[5]);
    accel_data->x = SWAP_AXIS * accel_y_raw / AcceSensitivity;
    accel_data->y = SWAP_AXIS * accel_x_raw / AcceSensitivity;
    accel_data->z = accel_z_raw / AcceSensitivity;

    return ESP_OK;
}

esp_err_t MPU9250_ReadGyro(vector_t *gyro_data)
{
    if (gyro_data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t gyro_raw[6];
    MPU9250_ReadRes(GYRO, gyro_raw, 6);

    int16_t gyro_x_raw = (int16_t)((gyro_raw[0] << 8) | gyro_raw[1]);
    int16_t gyro_y_raw = (int16_t)((gyro_raw[2] << 8) | gyro_raw[3]);
    int16_t gyro_z_raw = (int16_t)((gyro_raw[4] << 8) | gyro_raw[5]);
    gyro_data->x = gyro_y_raw / GyroSensitivity;
    gyro_data->y = gyro_x_raw / GyroSensitivity;
    gyro_data->z = SWAP_AXIS * gyro_z_raw / GyroSensitivity;
    return ESP_OK;
}

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
    uint8_t who = -1;
    who = WhoAmI();
    ESP_LOGI(TAG, "WHO: %u", who);
    if ((who != 0x71) && (who != 0x70))
    {
        return ESP_FAIL;
    }
    return ESP_OK;
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

static void MPU9250_Reset(void)
{
    uint8_t res = (uint8_t)POWMGMT_RES;
    uint8_t data;
    MPU9250_ReadRes(res, &data, 1);
    data |= (uint8_t)H_RESET;
    MPU9250_WriteRes(res, &data, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}
static void MPU9250_SelectClk(void)
{
    uint8_t res = (uint8_t)POWMGMT_RES;
    uint8_t data;
    MPU9250_ReadRes(res, &data, 1);
    data &= 0x11111000;
    data |= (uint8_t)CLK_SEL;
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
    uint8_t data;
    MPU9250_ReadRes(res, &data, 1);
    data |= (uint8_t)DIS_I2C;
    MPU9250_WriteRes(res, &data, 1);
}

static void MPU9250_ConfigGyro(void)
{
    // Set DLPF
    uint8_t res = (uint8_t)CONFIG_RES;
    uint8_t data;
    MPU9250_ReadRes(res, &data, 1);
    data &= ~(uint8_t)~0x07;
    data |= (uint8_t)_92HZ;
    MPU9250_WriteRes(res, &data, 1);

    // Set Full Scale
    res = (uint8_t)GYROCFG_RES;
    MPU9250_ReadRes(res, &data, 1);
    data &= ~(uint8_t)0x18;
    data |= (uint8_t)_1000dps;
    GyroSensitivity = (float)GYRO1000_SENSITIVYTI;
    MPU9250_WriteRes(res, &data, 1);
}

static void MPU9250_ConfigAcce(void)
{
    // Set DLPF
    uint8_t res = (uint8_t)ACCCFG_RES2;
    uint8_t data;
    MPU9250_ReadRes(res, &data, 1);
    data &= ~(uint8_t)0x07;
    data |= (uint8_t)_41HZ;
    MPU9250_WriteRes(res, &data, 1);

    // Set Full Scale
    res = (uint8_t)ACCCFG_RES1;
    MPU9250_ReadRes(res, &data, 1);
    data &= ~(uint8_t)0x18;
    data |= (uint8_t)_8g;
    AcceSensitivity = (float)ACC8_SENSITIVITY;
    MPU9250_WriteRes(res, &data, 1);
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

static void MPU9250_MasterI2C_Enable(void)
{
    uint8_t res = (uint8_t)UCRL_RES;
    uint8_t data;
    MPU9250_ReadRes(res, &data, 1);
    data |= (uint8_t)I2C_MST_EN;
    MPU9250_WriteRes(res, &data, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void MPU9250_Set_MasterI2C_Clk(void)
{
    uint8_t res = (uint8_t)I2C_MST_CTRL;
    uint8_t data;
    MPU9250_ReadRes(res, &data, 1);
    data = ~(uint8_t)0x07;
    data = (uint8_t)_400kHz;
    MPU9250_WriteRes(res, &data, 1);
}

static void MPU9250_WriteAK9864(uint8_t subres, uint8_t write_data)
{
    uint8_t data = AK9863_ADDR;
    MPU9250_WriteRes((uint8_t)SLV4_ADDR, &data, 1);

    data = subres;
    MPU9250_WriteRes(SLV4_REG, &data, 1);

    data = write_data;
    MPU9250_WriteRes(SLV4_DO, &data, 1);

    data = (uint8_t)SLV_EN | 0x01;
    MPU9250_WriteRes(SLV4_CTRL, &data, 1);
}

static void MPU9250_ReadAK9863(uint8_t subres, uint8_t *read_buffer, size_t size)
{
    uint8_t data = AK9863_ADDR | AK9863_READ_BIT;
    MPU9250_WriteRes((uint8_t)SLV0_ADDR, &data, 1);
    // printf("res: %02X: %02x\n", SLV0_ADDR, data);

    data = subres;
    MPU9250_WriteRes(SLV0_REG, &data, 1);
    // printf("res: %02X: %02x\n", SLV0_REG, data);

    data = (uint8_t)SLV_EN | (uint8_t)size;
    MPU9250_WriteRes(SLV0_CTRL, &data, 1);
    // printf("res: %02X: %02x\n", SLV0_CTRL, data);

    MPU9250_ReadRes(EXT_DATA, read_buffer, size);
    // printf("res: %02X: %02x\n", EXT_DATA, *read_buffer);

    vTaskDelay(pdMS_TO_TICKS(10000));
}

static void MPU9250_SetUpAK9863(void)
{
    uint8_t buf = AK9863_RESET_BIT;
    MPU9250_WriteAK9864(AK9863_CTRL2, buf);
    vTaskDelay(pdMS_TO_TICKS(50));

    buf = _16BIT_MODE;
    MPU9250_WriteAK9864(AK9863_CTRL1, buf);
    vTaskDelay(pdMS_TO_TICKS(100));

    buf = CONTINUOUS_2 | _16BIT_MODE;
    MPU9250_WriteAK9864(AK9863_CTRL1, buf);
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t data;
    uint8_t who_am_i_ak8963 = 0;

    ESP_LOGI(TAG, "Bat dau doc WHO_AM_I cua AK8963...");

    // 1. Chỉ định địa chỉ I2C của AK8963 (0x0C) VÀ bật cờ ĐỌC (0x80) -> 0x8C
    // Giả sử AK9863_ADDR của bạn là 0x0C và AK9863_READ_BIT là 0x80
    data = 0x8C;
    MPU9250_WriteRes((uint8_t)SLV0_ADDR, &data, 1);

    // 2. Chỉ định thanh ghi cần đọc bên trong AK8963 là WHO_AM_I (0x00)
    // Lưu ý: SLV0_RES trong code của bạn chính là thanh ghi I2C_SLV0_REG (0x26)
    data = 0x00;
    MPU9250_WriteRes((uint8_t)SLV0_REG, &data, 1);

    // 3. Ra lệnh ĐỌC 1 byte (Bật bit EN 0x80 và set length = 1) -> 0x81
    // Giả sử SLV0_EN của bạn là 0x80
    data = 0x81;
    MPU9250_WriteRes((uint8_t)SLV0_CTRL, &data, 1);

    // 4. CHỜ MPU-9250 đi lấy dữ liệu (Rất quan trọng - Code cũ của bạn thiếu cái này)
    // Phải chờ một chút để I2C Master hoàn thành giao tiếp với AK8963
    vTaskDelay(pdMS_TO_TICKS(20));

    // 5. Đọc giá trị lấy được từ bộ đệm EXT_SENS_DATA_00
    MPU9250_ReadRes((uint8_t)EXT_DATA, &who_am_i_ak8963, 1);

    // 6. DỌN DẸP: Tắt SLV0 đi để nó không vòng lặp đọc liên tục nữa
    data = 0x00;
    MPU9250_WriteRes((uint8_t)SLV0_CTRL, &data, 1);

    // Đợi 1 nhịp nhỏ cho nó tắt hẳn
    vTaskDelay(pdMS_TO_TICKS(2));

    // 7. In kết quả ra console
    ESP_LOGI(TAG, "AK8963 WHO_AM_I: 0x%02X (Ky vong: 0x48)", who_am_i_ak8963);
    printf("===> KET QUA AK8963 WHO_AM_I: 0x%02X <===\n", who_am_i_ak8963);
    vTaskDelay(pdMS_TO_TICKS(2000));
}

esp_err_t MPU9250_ReadMag(vector_t *mag_data)
{
    float MagSensitivity = 1.0f;
    if (mag_data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t mag_raw[7];
    MPU9250_ReadRes(EXT_DATA, mag_raw, 7);

    int16_t Mag_x_raw = (int16_t)((mag_raw[1] << 8) | mag_raw[0]);
    int16_t Mag_y_raw = (int16_t)((mag_raw[3] << 8) | mag_raw[2]);
    int16_t Mag_z_raw = (int16_t)((mag_raw[5] << 8) | mag_raw[4]);
    // printf("%d, %d, %d \n", Mag_x_raw, Mag_y_raw, Mag_z_raw);
    mag_data->x = Mag_y_raw / MagSensitivity;
    mag_data->y = Mag_x_raw / MagSensitivity;
    mag_data->z = Mag_z_raw / MagSensitivity;
    return ESP_OK;
}
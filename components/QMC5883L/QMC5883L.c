#include <stdio.h>
#include "QMC5883L.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

i2c_master_bus_handle_t QMC5883L_i2c_bus = NULL;
i2c_master_dev_handle_t QMC5883L_handle = NULL;

static const char *TAG = "QMC5883L";

esp_err_t QMC5883L_Init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = QMC5883L_CLK,
        .sda_io_num = QMC5883L_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false};
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_new_master_bus(&bus_cfg, &QMC5883L_i2c_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = QMC5883L_ADDR,
        .scl_speed_hz = QMC5883L_SPEED};
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_bus_add_device(QMC5883L_i2c_bus, &dev_cfg, &QMC5883L_handle));
    esp_err_t ret = QMC5883L_Setup();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SetUp fail");
        return ret;
    }
    ESP_LOGI(TAG, "Init success!");
    uint8_t who = 0x00;
    who = Who_Am_I();
    if (who == 0xFF)
        ESP_LOGI(TAG, "WHO: 0X%02X", who);
    else
    {
        ESP_LOGW(TAG, "Incorrect ID!");
        return ESP_FAIL;
    }
    return ret;
}

static esp_err_t QMC5883L_ReadReg(uint8_t reg_addr, uint8_t *receive_buffer, size_t receive_len)
{
    return i2c_master_transmit_receive(QMC5883L_handle, &reg_addr, 1, receive_buffer, receive_len, -1);
}

static esp_err_t QMC5883L_WriteReg(uint8_t reg_addr, uint8_t transmit_data)
{
    uint8_t transmit_buffer[2] = {reg_addr, transmit_data};
    return i2c_master_transmit(QMC5883L_handle, transmit_buffer, 2, -1);
}

static esp_err_t QMC5883L_Setup(void)
{
    esp_err_t ret;
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t data = (uint8_t)SOFT_RESET;
    ret = QMC5883L_WriteReg(QMC5883L_CONTROLREG2, data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    data = (uint8_t)ROL_POINTER | INT_ENB;
    ret = QMC5883L_WriteReg(QMC5883L_CONTROLREG2, data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data = (uint8_t)continuous | _2G | _100Hz | _512;
    ret = QMC5883L_WriteReg(QMC5883L_CONTROLREG1, data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    data = 0x01;
    ret = QMC5883L_WriteReg(QMC5883L_PERIODREG, data);
    if (ret != ESP_OK)
    {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    return ret;
}

static uint8_t Who_Am_I(void)
{
    uint8_t who = 0x00;
    QMC5883L_ReadReg(0x0D, &who, 1);
    return who;
}

void QMC5883L_ReadMag(vector_t *Mag)
{
    uint8_t mag_raw[6];
    ESP_ERROR_CHECK_WITHOUT_ABORT(QMC5883L_ReadReg(QMC5883L_DATAREG, mag_raw, 6));
    int16_t Mag_x_raw = (int16_t)((mag_raw[1] << 8) | mag_raw[0]);
    int16_t Mag_y_raw = (int16_t)((mag_raw[3] << 8) | mag_raw[2]);
    int16_t Mag_z_raw = (int16_t)((mag_raw[5] << 8) | mag_raw[4]);
    // printf("%d --- %d --- %d\n", Mag_x_raw, Mag_y_raw, Mag_z_raw);
    Mag->x = Mag_x_raw / MAG_SENSITIVITY;
    Mag->y = Mag_y_raw / MAG_SENSITIVITY;
    Mag->z = Mag_z_raw / MAG_SENSITIVITY;
}

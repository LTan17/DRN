#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "common_struct.h"

#define CONFIG_RES 0x1A
typedef enum
{
    _250HZ = 0x00,
    _184HZ = 0x01,
    _92HZ = 0x02,
    _41HZ = 0x03,
    _20HZ = 0x04,
    _3600HZ = 0x07,
} DLFP_HZ_t;

#define GYROCFG_RES 0x1B

typedef enum
{
    _250dps = 0x00,
    _500dps = 0x08,
    _1000dps = 0x10,
    _2000dps = 0x18
} Gyro_FullScale_t;

#define GYRO250_SENSITIVYTI 131.072f
#define GYRO500_SENSITIVYTI 65.536f
#define GYRO1000_SENSITIVYTI 32.768f
#define GYRO2000_SENSITIVYTI 16.384f

#define ACCCFG_RES1 0x1C

typedef enum
{
    _2g = 0x00,
    _4g = 0x08,
    _8g = 0x10,
    _16g = 0x18
} Acce_FullScale_t;

#define ACC2_SENSITIVITY 16384.0f
#define ACC4_SENSITIVITY 8192.0f
#define ACC8_SENSITIVITY 4096.0f
#define ACC16_SENSITIVITY 2048.0f

#define ACCCFG_RES2 0x1D

#define ACCEL 0x3B
#define TEMP 0x41
#define GYRO 0x43

#define UCRL_RES 0x6A
#define DIS_I2C 0x10
#define I2C_MST_EN 0x20

#define POWMGMT_RES 0x6B
#define H_RESET 0x80
#define CLK_SEL 0x01

#define POWMGMT2_RES 0x6C
#define ENABLE_ALL 0x00

#define WHOAMI 0x75

#define I2C_MST_CTRL 0x24
typedef enum
{
    _400kHz = 13
} I2C_MST_CLK_t;

#define SLV0_ADDR 0x25
#define SLV0_REG 0x26
#define SLV0_DO 0x63
#define SLV0_CTRL 0x27

#define SLV4_ADDR 0x31
#define SLV4_REG 0x32
#define SLV4_DO 0x33
#define SLV4_CTRL 0x34

#define SLV_EN 0x80

#define EXT_DATA 0x49

#define SWAP_AXIS -1

#define AK9863_ADDR 0x0F
#define AK9863_READ_BIT 0x80

#define AK9863_CTRL1 0x0A
#define CONTINUOUS_2 0x06
#define _16BIT_MODE 0x10

#define AK9863_CTRL2 0x0B
#define AK9863_RESET_BIT 0x01

#define AK9863_MAG 0x03

esp_err_t MPU9250_Init(void);
esp_err_t MPU9250_ReadAccel(vector_t *accel_data);
esp_err_t MPU9250_ReadGyro(vector_t *gyro_data);

static esp_err_t MPU9250_Select(void);
static esp_err_t MPU9250_Unselect(void);

static void MPU9250_Reset(void);
static void MPU9250_SelectClk(void);
static void MPU9250_Enable(void);
static void MPU9250_SelectSPIMode(void);

static uint8_t WhoAmI(void);
static void MPU9250_DefaltSetUp();

static void MPU9250_ConfigGyro(void);
static void MPU9250_ConfigAcce(void);

static void MPU9250_ReadRes(uint8_t res, uint8_t *rev_data, size_t size);
static void MPU9250_WriteRes(uint8_t res, uint8_t *send_data, size_t size);

static esp_err_t MPU9250_Transfer(const uint8_t *tx_buffer, uint8_t *rx_buffer, size_t length);
static esp_err_t MPU9250_TransmitReceive(const uint8_t *tx_buffer, size_t tx_length, uint8_t *rx_buffer, size_t rx_length);

static void MPU9250_MasterI2C_Enable(void);
static void MPU9250_Set_MasterI2C_Clk(void);
static void MPU9250_SetUpAK9863(void);
esp_err_t MPU9250_ReadMag(vector_t *mag_data);
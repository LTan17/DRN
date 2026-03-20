#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

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
#define DIS_I2C 0x04

#define POWMGMT_RES 0x6B
#define H_RESET 0x80
#define CLK_SEL 0x01

#define POWMGMT2_RES 0x6C
#define ENABLE_ALL 0x00

#define WHOAMI 0x75

#define SWAP_AXIS -1

typedef struct
{
    uint8_t H_data;
    uint8_t L_data
} raw_data;

typedef struct
{
    float x;
    float y;
    float z;
} vector_t;

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

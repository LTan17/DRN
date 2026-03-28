#pragma once
#include "esp_err.h"
#include "common_struct.h"

#define QMC5883L_CLK 1
#define QMC5883L_SDA 2
#define QMC5883L_ITR 8

#define QMC5883L_SPEED 400000

#define QMC5883L_ADDR 0x0D
#define QMC5883L_DATAREG 0x00
#define QMC5883L_CONTROLREG1 0x09

enum
{
    standby = 0b00000000,
    continuous = 0x00000001
};

enum
{
    _10Hz = 0b00000000,
    _50Hz = 0b00000100,
    _100Hz = 0b00001000,
    _200Hz = 0b00001100,
};

enum
{
    _2G = 0b00000000,
    _8G = 0b00010000,
};

enum
{
    _512 = 0b00000000,
    _256 = 0b01000000,
    _128 = 0b10000000,
    _64 = 0b11000000
};

#define QMC5883L_CONTROLREG2 0x0A
enum
{
    SOFT_RESET = 0b10000000,
    ROL_POINTER = 0b01000000,
    INT_ENB = 0b00000000
};

#define QMC5883L_PERIODREG 0x0B

#define MAG_SENSITIVITY 12000.0f

esp_err_t QMC5883L_Init(void);
static esp_err_t QMC5883L_ReadReg(uint8_t reg_addr, uint8_t *receive_buffer, size_t receive_len);
static esp_err_t QMC5883L_WriteReg(uint8_t reg_addr, uint8_t transmit_data);
static esp_err_t QMC5883L_Setup(void);
static uint8_t Who_Am_I(void);

void QMC5883L_ReadMag(vector_t *Mag);

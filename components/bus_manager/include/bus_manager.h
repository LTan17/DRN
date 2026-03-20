#pragma once
#include "esp_err.h"
#include "driver/gpio.h"

#define GY91_SCL 9
#define GY91_MOSI 10
#define GY91_MISO 11

#define GY91_CHUNKSIZE 64
#define GY91_SPIHOST SPI2_HOST

#define MPU9250_CS GPIO_NUM_12
#define BMP280_CS GPIO_NUM_13

esp_err_t bus_Init(void);

esp_err_t bus_Lock(void);
esp_err_t bus_Unlock(void);

static esp_err_t SPI_Init(void);
static esp_err_t CS_Init(void);

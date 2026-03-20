#include <stdio.h>
#include "esp_log.h"
#include "bus_manager.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "bus init";
static SemaphoreHandle_t s_spi_bus_mutex = NULL;

esp_err_t bus_Init(void)
{
    esp_err_t ret = SPI_Init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI init fail");
        return ret;
    }
    ret = CS_Init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "CS pin init fail");
        return ret;
    }

    s_spi_bus_mutex = xSemaphoreCreateMutex();
    if (s_spi_bus_mutex == NULL)
    {
        ESP_LOGE(TAG, "failed to create SPI bus mutex");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Success!");
    return ESP_OK;
}

esp_err_t bus_Lock(void)
{
    // if (s_spi_bus_mutex == NULL)
    // {
    //     return ESP_ERR_INVALID_STATE;
    // }
    // if (xSemaphoreTake(s_spi_bus_mutex, portMAX_DELAY) != pdTRUE)
    // {
    //     return ESP_ERR_TIMEOUT;
    // }
    return ESP_OK;
}

esp_err_t bus_Unlock(void)
{
    // if (s_spi_bus_mutex == NULL)
    // {
    //     return ESP_ERR_INVALID_STATE;
    // }
    // if (xSemaphoreGive(s_spi_bus_mutex) != pdTRUE)
    // {
    //     return ESP_FAIL;
    // }
    return ESP_OK;
}

static esp_err_t SPI_Init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GY91_MOSI,
        .miso_io_num = GY91_MISO,
        .sclk_io_num = GY91_SCL,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = GY91_CHUNKSIZE};
    esp_err_t ret = spi_bus_initialize(GY91_SPIHOST, &bus_cfg, SPI_DMA_DISABLED);
    return ret;
}

static esp_err_t CS_Init(void)
{
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << MPU9250_CS) | (1ULL << BMP280_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 1,
        .intr_type = GPIO_INTR_DISABLE};
    esp_err_t ret = gpio_config(&gpio_cfg);
    if (ret != ESP_OK)
    {
        return ret;
    }

    gpio_set_level(MPU9250_CS, 1);
    gpio_set_level(BMP280_CS, 1);

    return ESP_OK;
}

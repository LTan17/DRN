#include <stdio.h>
#include "ESC.h"

ESC_control_t controller;

esp_err_t ESC_Init()
{
    esp_err_t err = ESP_OK;
    controller.propeller1 = (propeller_handle_t)malloc(sizeof(propeller_t));
    controller.propeller2 = (propeller_handle_t)malloc(sizeof(propeller_t));
    controller.propeller3 = (propeller_handle_t)malloc(sizeof(propeller_t));
    controller.propeller4 = (propeller_handle_t)malloc(sizeof(propeller_t));
    RMT_TX_Init(controller.propeller1, PROPELLER1_PIN);
    RMT_TX_Init(controller.propeller2, PROPELLER2_PIN);
    RMT_TX_Init(controller.propeller3, PROPELLER3_PIN);
    RMT_TX_Init(controller.propeller4, PROPELLER4_PIN);
    vTaskDelay(pdMS_TO_TICKS(5000));
    RMT_TX_InitEncoder();
    return err;
}

esp_err_t ESC_Start()
{
    ESP_ERROR_CHECK(RMT_TX_Transmit(controller.propeller1, 0, CURRENT));
    ESP_ERROR_CHECK(RMT_TX_Transmit(controller.propeller2, 0, CURRENT));
    ESP_ERROR_CHECK(RMT_TX_Transmit(controller.propeller3, 0, CURRENT));
    ESP_ERROR_CHECK(RMT_TX_Transmit(controller.propeller4, 0, CURRENT));
    printf("Propeller All: Starting....\n");
    vTaskDelay(pdMS_TO_TICKS(6000));
    return ESP_OK;
}

void ESC_RunPropeller1(uint16_t speed)
{
    RMT_TX_Transmit(controller.propeller1, speed, CURRENT);
}

void ESC_RunPropeller2(uint16_t speed)
{
    RMT_TX_Transmit(controller.propeller2, speed, CURRENT);
}

void ESC_RunPropeller3(uint16_t speed)
{
    RMT_TX_Transmit(controller.propeller3, speed, CURRENT);
}

void ESC_RunPropeller4(uint16_t speed)
{
    RMT_TX_Transmit(controller.propeller4, speed, CURRENT);
}

void ESC_RunAllPropeller(uint16_t speed1, uint16_t speed2, uint16_t speed3, uint16_t speed4)
{
    RMT_TX_Transmit(controller.propeller1, speed1, CURRENT);
    RMT_TX_Transmit(controller.propeller2, speed2, CURRENT);
    RMT_TX_Transmit(controller.propeller3, speed3, CURRENT);
    RMT_TX_Transmit(controller.propeller4, speed4, CURRENT);
}

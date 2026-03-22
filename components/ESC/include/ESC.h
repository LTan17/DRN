#pragma once
#include "RMT.h"
#include "esp_log.h"

#define PROPELLER1_PIN GPIO_NUM_16
#define PROPELLER2_PIN GPIO_NUM_17
#define PROPELLER3_PIN GPIO_NUM_18
#define PROPELLER4_PIN GPIO_NUM_19

#define CURRENT false

typedef struct
{
    propeller_handle_t propeller1;
    propeller_handle_t propeller2;
    propeller_handle_t propeller3;
    propeller_handle_t propeller4;
} ESC_control_t;

extern ESC_control_t controller;

esp_err_t ESC_Init();
esp_err_t ESC_Start();
void ESC_RunPropeller1(uint16_t speed);
void ESC_RunPropeller2(uint16_t speed);
void ESC_RunPropeller3(uint16_t speed);
void ESC_RunPropeller4(uint16_t speed);
void ESC_RunAllPropeller(uint16_t speed1, uint16_t speed2, uint16_t speed3, uint16_t speed4);

#pragma once
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define RESOLUTION 40000000 // 40MHz
#define BAUD_RATE 300000    // DSHOT300

#define RMT_SYMBOLS 48

typedef struct
{
    rmt_symbol_word_t bit_0;
    rmt_symbol_word_t bit_1;
    rmt_symbol_word_t bit_stop;
} dshot_symbols_t;

typedef struct
{
    rmt_encoder_handle_t dshot;
    rmt_channel_handle_t propeller_channel;
    rmt_transmit_config_t propeller_tx_cfg;
    rmt_tx_channel_config_t propeller_tx_channel_cfg;
} propeller_t;

typedef propeller_t *propeller_handle_t;

void RMT_TX_Init(propeller_handle_t propeller_handle, int PIN_NUM);
esp_err_t RMT_TX_InitEncoder();
esp_err_t RMT_TX_Transmit(propeller_handle_t propeller_handle, uint16_t new_throttle, bool IsTelemery);

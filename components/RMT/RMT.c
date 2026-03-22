#include <stdio.h>
#include "RMT.h"

dshot_symbols_t dshot_symbols;

rmt_encoder_handle_t dshot;

char *TAG = "RMT";

typedef union
{
    struct
    {
        uint16_t crc : 4;
        uint16_t tele : 1;
        uint16_t throttle : 11;
    };
    uint16_t val;
} dshot_frame_t;

dshot_frame_t RMT_TX_GetFrame(uint16_t new_throttle, bool IsTelemery)
{
    dshot_frame_t frame = {0};
    if (new_throttle > 2047)
        new_throttle = 2047;
    frame.throttle = new_throttle;
    frame.tele = 0;
    uint16_t crc = ((frame.val >> 4) ^ (frame.val >> 8) ^ (frame.val >> 12)) & 0x000F;
    frame.crc = crc;
    return frame;
}

size_t RMT_TX_Callback(const void *data, size_t data_size, size_t symbols_written, size_t symbols_free,
                       rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    if (symbols_free < 64)
        return 0;
    uint16_t frame = *((uint16_t *)data);
    for (int i = 0; i < 16; i++)
    {
        bool isHighBit = frame >> (16 - 1 - i) & 0x01;
        if (isHighBit)
        {
            symbols[i] = dshot_symbols.bit_1;
        }
        else
        {
            symbols[i] = dshot_symbols.bit_0;
        }
    }
    for (int i = 16; i < 64; i++)
    {
        symbols[i] = dshot_symbols.bit_stop;
    }
    *done = true;
    return 64;
}

esp_err_t RMT_TX_InitEncoder()
{
    esp_err_t err = ESP_OK;
    float period_ticks = (float)RESOLUTION / BAUD_RATE;
    uint16_t t1h_ticks = (uint16_t)(period_ticks * 0.7485);
    uint16_t t1l_ticks = (uint16_t)(period_ticks - t1h_ticks);
    uint16_t t0h_ticks = (uint16_t)(period_ticks * 0.37425);
    uint16_t t0l_ticks = (uint16_t)(period_ticks - t0h_ticks);

    dshot_symbols.bit_0.level0 = 1;
    dshot_symbols.bit_0.duration0 = t0h_ticks;
    dshot_symbols.bit_0.level1 = 0;
    dshot_symbols.bit_0.duration1 = t0l_ticks;

    dshot_symbols.bit_1.level0 = 1;
    dshot_symbols.bit_1.duration0 = t1h_ticks;
    dshot_symbols.bit_1.level1 = 0;
    dshot_symbols.bit_1.duration1 = t1l_ticks;

    dshot_symbols.bit_stop.level0 = 0;
    dshot_symbols.bit_stop.duration0 = RESOLUTION / 2000000;
    dshot_symbols.bit_stop.level1 = 0;
    dshot_symbols.bit_stop.duration1 = RESOLUTION / 2000000;

    const rmt_simple_encoder_config_t dshot_config = {
        .callback = RMT_TX_Callback};
    err = rmt_new_simple_encoder(&dshot_config, &dshot);
    return err;
}

void RMT_TX_Init(propeller_handle_t propeller_handle, int PIN_NUM)
{
    memset(&propeller_handle->propeller_tx_channel_cfg, 0, sizeof(rmt_tx_channel_config_t));
    propeller_handle->propeller_tx_channel_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    propeller_handle->propeller_tx_channel_cfg.gpio_num = PIN_NUM;
    propeller_handle->propeller_tx_channel_cfg.resolution_hz = RESOLUTION;
    propeller_handle->propeller_tx_channel_cfg.mem_block_symbols = 64;
    propeller_handle->propeller_tx_channel_cfg.trans_queue_depth = 10;
    propeller_handle->propeller_tx_channel_cfg.flags.invert_out = false;
    propeller_handle->propeller_tx_channel_cfg.flags.with_dma = false;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&propeller_handle->propeller_tx_channel_cfg, &propeller_handle->propeller_channel));
    ESP_ERROR_CHECK(rmt_enable(propeller_handle->propeller_channel));
    memset(&propeller_handle->propeller_tx_cfg, 0, sizeof(propeller_handle->propeller_tx_cfg));
    propeller_handle->propeller_tx_cfg.loop_count = -1;
}

esp_err_t RMT_TX_Transmit(propeller_handle_t propeller_handle, uint16_t new_throttle, bool IsTelemery)
{
    esp_err_t err;
    dshot_frame_t frame = RMT_TX_GetFrame(new_throttle, IsTelemery);
    uint16_t data = frame.val;
    ESP_ERROR_CHECK(rmt_disable(propeller_handle->propeller_channel));
    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_ERROR_CHECK(rmt_enable(propeller_handle->propeller_channel));
    err = rmt_transmit(propeller_handle->propeller_channel, dshot, (void *)&data, sizeof(data), &propeller_handle->propeller_tx_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "transmit frame failed");
        return err;
    }
    return err;
}

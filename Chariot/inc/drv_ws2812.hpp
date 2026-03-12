#pragma once

#include "stm32f4xx_hal.h"
#include <cstring>

#define WS2812_LED_NUM 120

class WS2812
{
public:
    WS2812(TIM_HandleTypeDef *htim, uint32_t channel);

    void Init();
    void SetColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
    void Update();

private:
    TIM_HandleTypeDef *m_htim;
    uint32_t m_channel;
    
    // PWM buffer: (LED_NUM * 24) + Reset (50us > 40 * 1.25us -> ~42 zeros)
    // 0 code: 29% duty (30/105)
    // 1 code: 58% duty (60/105)
    // We use uint16_t for DMA buffer
    uint16_t m_buffer[WS2812_LED_NUM * 24 + 50]; 
    DMA_HandleTypeDef m_hdma;
};

#include "drv_ws2812.hpp"
#include "tim.h" /* For htim3/htim1 if used directly or for type definitions */

// TIM1 runs at 168MHz (APB2) on STM32F407
// 800kHz -> 1.25us.
// Total period ticks = 168,000,000 / 800,000 = 210.
// 0 code: 0.3us High -> 0.3/1.25 * 210 = 50.4 -> 50
// 1 code: 0.9us High -> 0.9/1.25 * 210 = 151.2 -> 151
// Reset code: >50us Low. (Handled by padding buffer with 0s)
#define TIM_PERIOD 210
#define CCR_0      50
#define CCR_1      151

extern "C" void DMA2_Stream5_IRQHandler(void);

// Helper to access the instance primarily for IRQ handler
static DMA_HandleTypeDef* g_hdma_ws2812 = nullptr;

WS2812::WS2812(TIM_HandleTypeDef *htim, uint32_t channel)
    : m_htim(htim), m_channel(channel)
{
    memset(m_buffer, 0, sizeof(m_buffer));
}

void WS2812::Init()
{    
    if (m_htim->Instance == TIM1) {
        __HAL_RCC_DMA2_CLK_ENABLE();
        
        // Initialize DMA2 Stream 5
        m_hdma.Instance = DMA2_Stream5;
        m_hdma.Init.Channel = DMA_CHANNEL_6; // TIM1_UP
        m_hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
        m_hdma.Init.PeriphInc = DMA_PINC_DISABLE;
        m_hdma.Init.MemInc = DMA_MINC_ENABLE;
        // CCR1 is 32-bit register on TIM1? Let's treat as 16-bit or 32-bit.
        // TIM1 CCRs are 16-bit (technically 32-bit reg, top reserved).
        // DMA writes correctly if we use HALFWORD.
        m_hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        m_hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        m_hdma.Init.Mode = DMA_NORMAL;
        m_hdma.Init.Priority = DMA_PRIORITY_HIGH;
        m_hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        
        if (HAL_DMA_Init(&m_hdma) != HAL_OK) {
            // Error
        }

        // Link DMA to TIM Update handle for ease of use
        // Note: HAL_TIM_Base_Start_DMA uses hdma[TIM_DMA_ID_UPDATE]
        __HAL_LINKDMA(m_htim, hdma[TIM_DMA_ID_UPDATE], m_hdma);
        
        // Enable IRQ
        HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
        
        g_hdma_ws2812 = &m_hdma;
        
        // Reconfigure Timer Period for 800kHz @ 168MHz
        __HAL_TIM_SET_PRESCALER(m_htim, 0);
        __HAL_TIM_SET_AUTORELOAD(m_htim, TIM_PERIOD - 1);
        
        // Important: For Update-Triggered DMA to drive PWM duty cycle:
        // DMA destination should be CCR1.
        // We cannot use HAL_TIM_Base_Start_DMA because it targets ARR (AutoReload).
        // We cannot use HAL_TIM_PWM_Start_DMA because it targets CCx but uses CCx DMA request (Occupied).
        
        // Enable TIM1 PWM output on Channel 1
        HAL_TIM_PWM_Start(m_htim, TIM_CHANNEL_1);
        
    } else {
        // Fallback or other timers
    }

    memset(m_buffer, 0, sizeof(m_buffer));
}

void WS2812::SetColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_LED_NUM) return;

    // GRB format
    uint32_t data = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    
    // Fill buffer. 24 bits. MSB first.
    uint16_t* pBuf = &m_buffer[index * 24];
    for (int i = 23; i >= 0; i--) {
        if ((data >> i) & 1) {
            *pBuf = CCR_1;
        } else {
            *pBuf = CCR_0;
        }
        pBuf++;
    }
}

void WS2812::Update()
{
    // Custom Start for TIM1 Update DMA
    if (m_htim->Instance == TIM1) {
        
        // Stop DMA if valid
        __HAL_TIM_DISABLE_DMA(m_htim, TIM_DMA_UPDATE);

        // Configure DMA manually to target CCR1
        HAL_DMA_Start_IT(&m_hdma, (uint32_t)m_buffer, (uint32_t)&m_htim->Instance->CCR1, sizeof(m_buffer)/sizeof(uint16_t));
        
        // Enable TIM Update DMA Request
        __HAL_TIM_ENABLE_DMA(m_htim, TIM_DMA_UPDATE);
        
    }
}

extern "C" void DMA2_Stream5_IRQHandler(void)
{
    if (g_hdma_ws2812) {
        HAL_DMA_IRQHandler(g_hdma_ws2812);
    }
}

// extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//    // If we use Update DMA, we might get accidental interrupts if IT enabled?
//    // We are not enabling TIM_IT_UPDATE, only TIM_DMA_UPDATE.
//    // But we reuse the DMA Complete callback?
// }

void HAL_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma == g_hdma_ws2812) {
         // Stop DMA request
         // If we have access to htim... 
         // Assuming TIM1 for now.
         __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
    }
}

// We need to register the callback?
// HAL_DMA_Init does NOT register CpltCallback unless we do it.
// Or we rely on HAL_TIM_IRQHandler? No.
// We can just set it in Init.


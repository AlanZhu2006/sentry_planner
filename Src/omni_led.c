#include "omni_led.h"

#include "main.h"
#include "tim.h"

#define WS2812_BIT_COUNT 24U
#define WS2812_RESET_SLOTS 64U
#define WS2812_FRAME_SLOTS (WS2812_RESET_SLOTS + WS2812_BIT_COUNT + WS2812_RESET_SLOTS)

/* TIM1 runs at approximately 800 kHz: 24/86 and 48/86 are WS2812 0/1 pulses. */
#define WS2812_ZERO_PULSE 24U
#define WS2812_ONE_PULSE 48U

static uint16_t ws2812_frame[WS2812_FRAME_SLOTS] __attribute__((aligned(32)));
static volatile uint8_t ws2812_busy;
static uint8_t ws2812_started;

static void EncodeByte(uint8_t value, uint32_t *index)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++)
    {
        ws2812_frame[*index] = (value & (uint8_t)(0x80U >> bit)) ? WS2812_ONE_PULSE : WS2812_ZERO_PULSE;
        (*index)++;
    }
}

void OmniLedInit(void)
{
    uint32_t index;

    for (index = 0U; index < WS2812_FRAME_SLOTS; index++)
    {
        ws2812_frame[index] = 0U;
    }
    ws2812_busy = 0U;
    ws2812_started = 0U;
}

void OmniLedSetRgb(uint8_t red, uint8_t green, uint8_t blue)
{
    uint32_t index = WS2812_RESET_SLOTS;

    OmniLedWait();

    /* WS2812 data order is green, red, blue. */
    EncodeByte(green, &index);
    EncodeByte(red, &index);
    EncodeByte(blue, &index);

    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanDCache_by_Addr((uint32_t *)ws2812_frame, sizeof(ws2812_frame));
    }
    ws2812_busy = 1U;

    if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_3, (const uint32_t *)ws2812_frame,
                              WS2812_FRAME_SLOTS) != HAL_OK)
    {
        ws2812_busy = 0U;
        Error_Handler();
    }
    ws2812_started = 1U;
}

void OmniLedWait(void)
{
    while (ws2812_busy != 0U)
    {
    }

    if (ws2812_started != 0U && HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    ws2812_started = 0U;
}

void OmniLedPulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        ws2812_busy = 0U;
    }
}

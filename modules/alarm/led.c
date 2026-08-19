/**
 * @file led.c
 * @brief Status control for the on-board WS2812 RGB LED.
 */

#include "led.h"
#include "omni_led.h"

static LEDInstance led_instance;
static uint8_t output_valid;
static uint8_t output_r;
static uint8_t output_g;
static uint8_t output_b;

static uint8_t LEDScale(float value)
{
    if (value < 0.0f)
        value = 0.0f;
    if (value > 1.0f)
        value = 1.0f;
    return (uint8_t)(value * led_instance.brightness * 255.0f);
}

static void LEDSetRGB(float red, float green, float blue)
{
    uint8_t r = LEDScale(red);
    uint8_t g = LEDScale(green);
    uint8_t b = LEDScale(blue);

    if (!output_valid || r != output_r || g != output_g || b != output_b)
    {
        OmniLedSetRgb(r, g, b);
        output_r = r;
        output_g = g;
        output_b = b;
        output_valid = 1U;
    }
}

void LEDInit(void)
{
    OmniLedInit();
    led_instance.status = LED_STATUS_OFF;
    led_instance.blink_count = 0;
    led_instance.brightness = 0.3f;
    output_valid = 0U;
    LEDSetRGB(0.0f, 0.0f, 0.0f);
}

void LEDSetStatus(LED_Status_e status)
{
    led_instance.status = status;
    led_instance.blink_count = 0;
    output_valid = 0U;
}

void LEDSetBrightness(float brightness)
{
    if (brightness < 0.0f)
        brightness = 0.0f;
    if (brightness > 1.0f)
        brightness = 1.0f;
    led_instance.brightness = brightness;
    output_valid = 0U;
}

void LEDTask(void)
{
    uint8_t blink_on = (uint8_t)(((led_instance.blink_count++ / 25U) & 1U) == 0U);

    switch (led_instance.status)
    {
    case LED_STATUS_RED_BLINK:
        LEDSetRGB(blink_on, 0.0f, 0.0f);
        break;
    case LED_STATUS_BLUE_BLINK:
        LEDSetRGB(0.0f, 0.0f, blink_on);
        break;
    case LED_STATUS_GREEN_ON:
        LEDSetRGB(0.0f, 1.0f, 0.0f);
        break;
    case LED_STATUS_RED_ON:
        LEDSetRGB(1.0f, 0.0f, 0.0f);
        break;
    case LED_STATUS_BLUE_ON:
        LEDSetRGB(0.0f, 0.0f, 1.0f);
        break;
    case LED_STATUS_YELLOW_ON:
        LEDSetRGB(1.0f, 1.0f, 0.0f);
        break;
    case LED_STATUS_PURPLE_ON:
        LEDSetRGB(1.0f, 0.0f, 1.0f);
        break;
    case LED_STATUS_CYAN_ON:
        LEDSetRGB(0.0f, 1.0f, 1.0f);
        break;
    case LED_STATUS_WHITE_ON:
        LEDSetRGB(1.0f, 1.0f, 1.0f);
        break;
    case LED_STATUS_OFF:
    default:
        LEDSetRGB(0.0f, 0.0f, 0.0f);
        break;
    }
}

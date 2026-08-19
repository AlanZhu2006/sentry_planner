#include "buzzer.h"

#include "bsp_gpio.h"
#include "bsp_log.h"
#include "main.h"

#include <stdlib.h>
#include <string.h>

static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT];
static GPIOInstance *buzzer_gpio;

void BuzzerInit(void)
{
    GPIO_Init_Config_s gpio_config = {
        .GPIOx = BUZZER_GPIO_Port,
        .GPIO_Pin = BUZZER_Pin,
        .pin_state = GPIO_PIN_RESET,
        .exti_mode = GPIO_EXTI_MODE_NONE,
    };
    buzzer_gpio = GPIORegister(&gpio_config);
    if (buzzer_gpio != NULL)
        GPIOReset(buzzer_gpio);
    LOGWARNING("[buzzer] PE13 is GPIO-only in the supplied board configuration; tone output is disabled");
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (config == NULL || config->alarm_level >= BUZZER_DEVICE_CNT)
        return NULL;

    BuzzzerInstance *instance = malloc(sizeof(*instance));
    if (instance == NULL)
        return NULL;

    memset(instance, 0, sizeof(*instance));
    instance->alarm_level = config->alarm_level;
    instance->loudness = config->loudness;
    instance->octave = config->octave;
    instance->alarm_state = ALARM_OFF;
    buzzer_list[config->alarm_level] = instance;
    return instance;
}

void AlarmSetStatus(BuzzzerInstance *instance, AlarmState_e state)
{
    if (instance != NULL)
        instance->alarm_state = state;
}

void BuzzerTask(void)
{
    /* Keep the passive buzzer quiet until a board-approved timer mapping is supplied. */
    if (buzzer_gpio != NULL)
        GPIOReset(buzzer_gpio);
}

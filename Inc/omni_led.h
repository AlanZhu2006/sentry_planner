#ifndef OMNI_LED_H
#define OMNI_LED_H

#include <stdint.h>
#include "tim.h"

void OmniLedInit(void);
void OmniLedSetRgb(uint8_t red, uint8_t green, uint8_t blue);
void OmniLedWait(void);
void OmniLedPulseFinishedCallback(TIM_HandleTypeDef *htim);

#endif /* OMNI_LED_H */

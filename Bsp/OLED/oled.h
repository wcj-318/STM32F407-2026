#ifndef OLED_H
#define OLED_H

#include "stm32f4xx_hal.h"

void OLED_Init(void);
void OLED_ShowPhase2(const char *state_text, uint8_t gray8_mask, uint8_t unstable_mask,
                     int16_t error10, uint8_t line_valid, uint8_t gray5_mask,
                     int16_t left_pwm, int16_t right_pwm, int16_t correction);
void OLED_ShowTimeMs(uint32_t elapsed_ms);

#endif

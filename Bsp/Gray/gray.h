#ifndef GRAY_H
#define GRAY_H

#include "stm32f4xx_hal.h"

typedef struct
{
    uint8_t gray8_mask;
    uint8_t gray8_unstable_mask;
    uint8_t gray5_mask;
    int16_t error10;
    uint8_t line_valid;
} Gray_Sample_t;

void Gray_Init(void);
void Gray_Read(Gray_Sample_t *sample);

#endif

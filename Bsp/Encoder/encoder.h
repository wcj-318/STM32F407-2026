#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"

typedef struct
{
    int32_t left_total;
    int32_t right_total;
    int16_t left_delta;
    int16_t right_delta;
    int32_t left_counts_per_second;
    int32_t right_counts_per_second;
    uint16_t interval_ms;
} Encoder_Sample_t;

HAL_StatusTypeDef Encoder_Init(void);
void Encoder_Reset(void);
void Encoder_Update(uint32_t elapsed_ms);
const Encoder_Sample_t *Encoder_GetSample(void);

#endif

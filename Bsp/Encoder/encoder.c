#include "encoder.h"

#include "tim.h"

static uint16_t encoder_left_previous;
static uint16_t encoder_right_previous;
static Encoder_Sample_t encoder_sample;

static int16_t Encoder_Delta16(uint16_t current, uint16_t previous)
{
    uint16_t wrapped_delta = (uint16_t)(current - previous);

    if (wrapped_delta <= 0x7FFFU)
    {
        return (int16_t)wrapped_delta;
    }
    return (int16_t)((int32_t)wrapped_delta - 65536);
}

HAL_StatusTypeDef Encoder_Init(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);

    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
    {
        HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
        return HAL_ERROR;
    }

    Encoder_Reset();
    return HAL_OK;
}

void Encoder_Reset(void)
{
    encoder_left_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    encoder_right_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    encoder_sample.left_total = 0;
    encoder_sample.right_total = 0;
    encoder_sample.left_delta = 0;
    encoder_sample.right_delta = 0;
    encoder_sample.left_counts_per_second = 0;
    encoder_sample.right_counts_per_second = 0;
    encoder_sample.interval_ms = 0U;
}

void Encoder_Update(uint32_t elapsed_ms)
{
    uint16_t left_current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t right_current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t left_delta = Encoder_Delta16(left_current, encoder_left_previous);
    int16_t right_delta = Encoder_Delta16(right_current, encoder_right_previous);

    encoder_left_previous = left_current;
    encoder_right_previous = right_current;
    encoder_sample.left_delta = left_delta;
    encoder_sample.right_delta = right_delta;
    encoder_sample.left_total += left_delta;
    encoder_sample.right_total += right_delta;

    if (elapsed_ms == 0U)
    {
        encoder_sample.left_counts_per_second = 0;
        encoder_sample.right_counts_per_second = 0;
        encoder_sample.interval_ms = 0U;
        return;
    }

    encoder_sample.left_counts_per_second =
        (int32_t)(((int64_t)left_delta * 1000) / elapsed_ms);
    encoder_sample.right_counts_per_second =
        (int32_t)(((int64_t)right_delta * 1000) / elapsed_ms);
    encoder_sample.interval_ms = (elapsed_ms > 65535U) ? 65535U : (uint16_t)elapsed_ms;
}

const Encoder_Sample_t *Encoder_GetSample(void)
{
    return &encoder_sample;
}

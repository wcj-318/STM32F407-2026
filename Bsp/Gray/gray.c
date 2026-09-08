#include "gray.h"

#define GRAY8_OUT_PIN       GPIO_PIN_8
#define GRAY8_AD0_PIN       GPIO_PIN_9
#define GRAY8_AD1_PIN       GPIO_PIN_10
#define GRAY8_AD2_PIN       GPIO_PIN_11
#define GRAY8_ADDRESS_PINS  (GRAY8_AD0_PIN | GRAY8_AD1_PIN | GRAY8_AD2_PIN)
#define GRAY8_SETTLE_US      20U
#define GRAY8_VOTE_SAMPLES   7U
#define GRAY8_VOTE_DELAY_US  3U

static const int8_t gray8_weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

static void Gray_DelayUs(uint32_t microseconds)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * microseconds;

    while ((DWT->CYCCNT - start) < cycles)
    {
    }
}

static void Gray8_Select(uint8_t channel)
{
    uint32_t set_pins = 0U;

    if ((channel & 0x01U) != 0U)
    {
        set_pins |= GRAY8_AD0_PIN;
    }
    if ((channel & 0x02U) != 0U)
    {
        set_pins |= GRAY8_AD1_PIN;
    }
    if ((channel & 0x04U) != 0U)
    {
        set_pins |= GRAY8_AD2_PIN;
    }

    GPIOE->BSRR = ((uint32_t)GRAY8_ADDRESS_PINS << 16U) | set_pins;
    Gray_DelayUs(GRAY8_SETTLE_US);
}

static uint8_t Gray8_ReadMask(uint8_t *unstable_mask)
{
    uint8_t channel;
    uint8_t mask = 0U;
    uint8_t sample_index;
    uint8_t high_votes;

    *unstable_mask = 0U;

    for (channel = 0U; channel < 8U; ++channel)
    {
        Gray8_Select(channel);

        high_votes = 0U;
        for (sample_index = 0U; sample_index < GRAY8_VOTE_SAMPLES; ++sample_index)
        {
            if (HAL_GPIO_ReadPin(GPIOE, GRAY8_OUT_PIN) == GPIO_PIN_SET)
            {
                ++high_votes;
            }
            if ((sample_index + 1U) < GRAY8_VOTE_SAMPLES)
            {
                Gray_DelayUs(GRAY8_VOTE_DELAY_US);
            }
        }

        if (high_votes >= ((GRAY8_VOTE_SAMPLES / 2U) + 1U))
        {
            mask |= (uint8_t)(1U << channel);
        }
        if ((high_votes != 0U) && (high_votes != GRAY8_VOTE_SAMPLES))
        {
            *unstable_mask |= (uint8_t)(1U << channel);
        }
    }

    return mask;
}

static int16_t Gray8_CalculateError10(uint8_t mask, uint8_t *line_valid)
{
    int16_t weighted_sum = 0;
    uint8_t black_count = 0U;
    uint8_t channel;

    for (channel = 0U; channel < 8U; ++channel)
    {
        if ((mask & (uint8_t)(1U << channel)) != 0U)
        {
            weighted_sum += gray8_weight[channel];
            ++black_count;
        }
    }

    if (black_count == 0U)
    {
        *line_valid = 0U;
        return 0;
    }

    *line_valid = 1U;
    return (int16_t)((weighted_sum * 10) / (int16_t)black_count);
}

void Gray_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_GPIO_WritePin(GPIOE, GRAY8_ADDRESS_PINS, GPIO_PIN_RESET);

    gpio.Pin = GRAY8_ADDRESS_PINS;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = GRAY8_OUT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &gpio);
}

void Gray_Read(Gray_Sample_t *sample)
{
    static uint8_t previous_mask;
    static uint8_t older_mask;
    static uint8_t history_initialized;
    uint8_t raw_mask;
    uint8_t vote_unstable_mask;

    raw_mask = Gray8_ReadMask(&vote_unstable_mask);
    if (history_initialized == 0U)
    {
        previous_mask = raw_mask;
        older_mask = raw_mask;
        history_initialized = 1U;
    }

    sample->gray8_mask = (uint8_t)((raw_mask & previous_mask) |
                                   (raw_mask & older_mask) |
                                   (previous_mask & older_mask));
    sample->gray8_unstable_mask = (uint8_t)(vote_unstable_mask |
                                            (raw_mask ^ previous_mask) |
                                            (raw_mask ^ older_mask));
    older_mask = previous_mask;
    previous_mask = raw_mask;

    sample->gray5_mask = (uint8_t)((GPIOD->IDR >> 8U) & 0x1FU);
    sample->error10 = Gray8_CalculateError10(sample->gray8_mask, &sample->line_valid);
}

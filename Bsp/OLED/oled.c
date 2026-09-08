#include "oled.h"

#include "line_follow.h"

#define OLED_SCL_PIN           GPIO_PIN_8
#define OLED_SDA_PIN           GPIO_PIN_9
#define OLED_I2C_ADDRESS        0x78U
#define OLED_LINE_WIDTH         128U
#define OLED_HALF_PERIOD_US     2U

static const uint8_t font_digit[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}
};

static const uint8_t font_letter[26][5] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}
};

static void OLED_Delay(void)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000U) * OLED_HALF_PERIOD_US;

    while ((DWT->CYCCNT - start) < cycles)
    {
    }
}

static void OLED_WritePin(uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOB, pin, state);
    OLED_Delay();
}

static void OLED_Start(void)
{
    OLED_WritePin(OLED_SDA_PIN, GPIO_PIN_SET);
    OLED_WritePin(OLED_SCL_PIN, GPIO_PIN_SET);
    OLED_WritePin(OLED_SDA_PIN, GPIO_PIN_RESET);
    OLED_WritePin(OLED_SCL_PIN, GPIO_PIN_RESET);
}

static void OLED_Stop(void)
{
    OLED_WritePin(OLED_SDA_PIN, GPIO_PIN_RESET);
    OLED_WritePin(OLED_SCL_PIN, GPIO_PIN_SET);
    OLED_WritePin(OLED_SDA_PIN, GPIO_PIN_SET);
}

static void OLED_WriteByte(uint8_t value)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; ++bit)
    {
        OLED_WritePin(OLED_SDA_PIN, ((value & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        OLED_WritePin(OLED_SCL_PIN, GPIO_PIN_SET);
        OLED_WritePin(OLED_SCL_PIN, GPIO_PIN_RESET);
        value <<= 1U;
    }

    OLED_WritePin(OLED_SDA_PIN, GPIO_PIN_SET);
    OLED_WritePin(OLED_SCL_PIN, GPIO_PIN_SET);
    OLED_WritePin(OLED_SCL_PIN, GPIO_PIN_RESET);
}

static void OLED_Command(uint8_t command)
{
    OLED_Start();
    OLED_WriteByte(OLED_I2C_ADDRESS);
    OLED_WriteByte(0x00U);
    OLED_WriteByte(command);
    OLED_Stop();
}

static void OLED_WriteLine(uint8_t page, const char *text)
{
    uint8_t line[OLED_LINE_WIDTH] = {0};
    uint8_t glyph[5];
    uint8_t column = 0U;
    uint8_t index;

    while ((*text != '\0') && (column < (OLED_LINE_WIDTH - 6U)))
    {
        if ((*text >= '0') && (*text <= '9'))
        {
            for (index = 0U; index < 5U; ++index)
            {
                glyph[index] = font_digit[*text - '0'][index];
            }
        }
        else if ((*text >= 'A') && (*text <= 'Z'))
        {
            for (index = 0U; index < 5U; ++index)
            {
                glyph[index] = font_letter[*text - 'A'][index];
            }
        }
        else
        {
            glyph[0] = 0U;
            glyph[1] = (*text == '+') ? 0x08U : ((*text == '-') ? 0x08U : 0U);
            glyph[2] = (*text == '+') ? 0x1CU : ((*text == '-') ? 0x08U : ((*text == ':') ? 0x24U : ((*text == '.') ? 0x40U : 0U)));
            glyph[3] = (*text == '+') ? 0x08U : ((*text == '-') ? 0x08U : ((*text == ':') ? 0x24U : 0U));
            glyph[4] = 0U;
        }

        for (index = 0U; index < 5U; ++index)
        {
            line[column++] = glyph[index];
        }
        ++column;
        ++text;
    }

    OLED_Command((uint8_t)(0xB0U + page));
    OLED_Command(0x00U);
    OLED_Command(0x10U);
    OLED_Start();
    OLED_WriteByte(OLED_I2C_ADDRESS);
    OLED_WriteByte(0x40U);
    for (index = 0U; index < OLED_LINE_WIDTH; ++index)
    {
        OLED_WriteByte(line[index]);
    }
    OLED_Stop();
}

static void OLED_FormatBits(char *text, char prefix, uint8_t mask, uint8_t count)
{
    uint8_t index;

    text[0] = prefix;
    text[1] = ':';
    for (index = 0U; index < count; ++index)
    {
        text[index + 2U] = ((mask & (uint8_t)(1U << index)) != 0U) ? '1' : '0';
    }
    text[count + 2U] = '\0';
}

void OLED_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t page;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    HAL_GPIO_WritePin(GPIOB, OLED_SCL_PIN | OLED_SDA_PIN, GPIO_PIN_SET);
    gpio.Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_Delay(100U);
    OLED_Command(0xAEU);
    OLED_Command(0x20U);
    OLED_Command(0x02U);
    OLED_Command(0x81U);
    OLED_Command(0x7FU);
    OLED_Command(0xA1U);
    OLED_Command(0xC8U);
    OLED_Command(0xA6U);
    OLED_Command(0xA8U);
    OLED_Command(0x3FU);
    OLED_Command(0xD3U);
    OLED_Command(0x00U);
    OLED_Command(0xD5U);
    OLED_Command(0x80U);
    OLED_Command(0xD9U);
    OLED_Command(0xF1U);
    OLED_Command(0xDAU);
    OLED_Command(0x12U);
    OLED_Command(0xDBU);
    OLED_Command(0x40U);
    OLED_Command(0x8DU);
    OLED_Command(0x14U);
    OLED_Command(0xAFU);

    for (page = 0U; page < 8U; ++page)
    {
        OLED_WriteLine(page, "");
    }
}

void OLED_ShowPhase2(const char *state_text, uint8_t gray8_mask, uint8_t unstable_mask,
                     int16_t error10, uint8_t line_valid, uint8_t gray5_mask,
                     int16_t left_pwm, int16_t right_pwm, int16_t correction)
{
    char error_text[10] = "E:NOLINE";
    char gray8_text[11];
    char unstable_text[11];
    char gray5_text[8];
    char motor_text[12] = "L:000 R:000";
    char correction_text[7] = "C:+000";
    char config_text[12] = "B00 P00 D00";
    int16_t magnitude;
    int16_t left_percent = (int16_t)(left_pwm / 10);
    int16_t right_percent = (int16_t)(right_pwm / 10);
    int16_t correction_percent;
    int16_t base_percent = HLINE_BASE_PWM_PERMILLE / 10;
    int16_t kp = HLINE_LINE_KP;
    int16_t kd = HLINE_LINE_KD;

    if (line_valid != 0U)
    {
        magnitude = (error10 < 0) ? (int16_t)(-error10) : error10;
        error_text[0] = 'E';
        error_text[1] = ':';
        error_text[2] = (error10 < 0) ? '-' : '+';
        error_text[3] = (char)('0' + (magnitude / 10));
        error_text[4] = '.';
        error_text[5] = (char)('0' + (magnitude % 10));
        error_text[6] = '\0';
    }

    motor_text[2] = (char)('0' + ((left_percent / 100) % 10));
    motor_text[3] = (char)('0' + ((left_percent / 10) % 10));
    motor_text[4] = (char)('0' + (left_percent % 10));
    motor_text[8] = (char)('0' + ((right_percent / 100) % 10));
    motor_text[9] = (char)('0' + ((right_percent / 10) % 10));
    motor_text[10] = (char)('0' + (right_percent % 10));

    magnitude = (correction < 0) ? (int16_t)(-correction) : correction;
    correction_percent = (int16_t)(magnitude / 10);
    if (correction_percent > 999)
    {
        correction_percent = 999;
    }
    correction_text[2] = (correction < 0) ? '-' : '+';
    correction_text[3] = (char)('0' + ((correction_percent / 100) % 10));
    correction_text[4] = (char)('0' + ((correction_percent / 10) % 10));
    correction_text[5] = (char)('0' + (correction_percent % 10));

    if (kp > 99)
    {
        kp = 99;
    }
    if (kd > 99)
    {
        kd = 99;
    }
    config_text[1] = (char)('0' + ((base_percent / 10) % 10));
    config_text[2] = (char)('0' + (base_percent % 10));
    config_text[5] = (char)('0' + ((kp / 10) % 10));
    config_text[6] = (char)('0' + (kp % 10));
    config_text[9] = (char)('0' + ((kd / 10) % 10));
    config_text[10] = (char)('0' + (kd % 10));

    OLED_WriteLine(0U, state_text);
    OLED_FormatBits(gray8_text, 'G', gray8_mask, 8U);
    OLED_WriteLine(1U, gray8_text);
    OLED_FormatBits(unstable_text, 'U', unstable_mask, 8U);
    OLED_WriteLine(2U, unstable_text);
    OLED_WriteLine(3U, error_text);
    OLED_WriteLine(4U, motor_text);
    OLED_WriteLine(5U, correction_text);
    OLED_FormatBits(gray5_text, 'A', gray5_mask, 5U);
    OLED_WriteLine(6U, gray5_text);
    OLED_WriteLine(7U, config_text);
}

void OLED_ShowTimeMs(uint32_t elapsed_ms)
{
    char time_text[8] = "T:00.0S";
    uint32_t tenths = elapsed_ms / 100U;

    if (tenths > 999U)
    {
        tenths = 999U;
    }

    time_text[2] = (char)('0' + ((tenths / 100U) % 10U));
    time_text[3] = (char)('0' + ((tenths / 10U) % 10U));
    time_text[5] = (char)('0' + (tenths % 10U));
    OLED_WriteLine(7U, time_text);
}

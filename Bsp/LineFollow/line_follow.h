#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include "gray.h"

#define HLINE_CONTROL_PERIOD_MS          10U
#define HLINE_BASE_PWM_PERMILLE          330
#define HLINE_MAX_PWM_PERMILLE           600
#define HLINE_MIN_FORWARD_PWM_PERMILLE   130
#define HLINE_CURVE_BASE_MIN_PWM_PERMILLE 270
#define HLINE_CORRECTION_LIMIT_PERMILLE  (HLINE_CURVE_BASE_MIN_PWM_PERMILLE - HLINE_MIN_FORWARD_PWM_PERMILLE)
#define HLINE_LINE_KP                    3




#define HLINE_LINE_KD                    8
#define HLINE_CENTER_ERROR10              10
#define HLINE_CENTER_CORRECTION_PERMILLE  20
#define HLINE_CORRECTION_SLEW_PER_STEP    30
#define HLINE_LOST_GRACE_FRAMES          30U
typedef struct
{
    int16_t previous_error10;
    int16_t last_left_pwm;
    int16_t last_right_pwm;
    int16_t last_correction;
    uint8_t has_previous_error;
    uint8_t lost_frames;
} LineFollow_Controller_t;

void LineFollow_Reset(LineFollow_Controller_t *controller);
uint8_t LineFollow_Step(LineFollow_Controller_t *controller, const Gray_Sample_t *sample,
                        int16_t *left_pwm, int16_t *right_pwm, int16_t *correction);

#endif

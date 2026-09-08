#include "line_follow.h"

static int16_t LineFollow_ClampPwm(int32_t pwm)
{
    if (pwm > HLINE_MAX_PWM_PERMILLE)
    {
        return HLINE_MAX_PWM_PERMILLE;
    }
    if (pwm < 0)
    {
        return 0;
    }
    return (int16_t)pwm;
}

void LineFollow_Reset(LineFollow_Controller_t *controller)
{
    controller->previous_error10 = 0;
    controller->last_left_pwm = 0;
    controller->last_right_pwm = 0;
    controller->last_correction = 0;
    controller->has_previous_error = 0U;
    controller->lost_frames = 0U;
}

uint8_t LineFollow_Step(LineFollow_Controller_t *controller, const Gray_Sample_t *sample,
                        int16_t *left_pwm, int16_t *right_pwm, int16_t *correction)
{
    int16_t control_error10;
    int16_t error_delta10 = 0;
    int32_t base_pwm;
    int32_t correction_magnitude;
    int32_t output;

    if (sample->line_valid == 0U)
    {
        if ((controller->has_previous_error != 0U) &&
            (controller->lost_frames < HLINE_LOST_GRACE_FRAMES))
        {
            controller->lost_frames++;
            *left_pwm = controller->last_left_pwm;
            *right_pwm = controller->last_right_pwm;
            *correction = controller->last_correction;
            return 1U;
        }

        LineFollow_Reset(controller);
        *left_pwm = 0;
        *right_pwm = 0;
        *correction = 0;
        return 0U;
    }

    control_error10 = sample->error10;
    if ((control_error10 >= -HLINE_CENTER_ERROR10) &&
        (control_error10 <= HLINE_CENTER_ERROR10))
    {
        if (control_error10 < 0)
        {
            output = -HLINE_CENTER_CORRECTION_PERMILLE;
        }
        else if (control_error10 > 0)
        {
            output = HLINE_CENTER_CORRECTION_PERMILLE;
        }
        else
        {
            output = 0;
        }
    }
    else
    {
        if (controller->has_previous_error != 0U)
        {
            error_delta10 = (int16_t)(control_error10 - controller->previous_error10);
        }

        /*
         * error10 is in 0.1 sensor-weight units. Multiplying directly by the
         * document gains gives a permille correction: error +1.0 -> KP * 10.
         */
        output = (int32_t)HLINE_LINE_KP * control_error10
                 + (int32_t)HLINE_LINE_KD * error_delta10;
    }

    if (output > HLINE_CORRECTION_LIMIT_PERMILLE)
    {
        output = HLINE_CORRECTION_LIMIT_PERMILLE;
    }
    else if (output < -HLINE_CORRECTION_LIMIT_PERMILLE)
    {
        output = -HLINE_CORRECTION_LIMIT_PERMILLE;
    }

    if ((controller->has_previous_error != 0U) &&
        !((control_error10 >= -HLINE_CENTER_ERROR10) &&
          (control_error10 <= HLINE_CENTER_ERROR10) &&
          (controller->previous_error10 >= -HLINE_CENTER_ERROR10) &&
          (controller->previous_error10 <= HLINE_CENTER_ERROR10)))
    {
        if (output > ((int32_t)controller->last_correction + HLINE_CORRECTION_SLEW_PER_STEP))
        {
            output = (int32_t)controller->last_correction + HLINE_CORRECTION_SLEW_PER_STEP;
        }
        else if (output < ((int32_t)controller->last_correction - HLINE_CORRECTION_SLEW_PER_STEP))
        {
            output = (int32_t)controller->last_correction - HLINE_CORRECTION_SLEW_PER_STEP;
        }
    }

    correction_magnitude = (output < 0) ? -output : output;
    base_pwm = HLINE_BASE_PWM_PERMILLE;
    if (correction_magnitude > HLINE_CENTER_CORRECTION_PERMILLE)
    {
        base_pwm -= ((correction_magnitude - HLINE_CENTER_CORRECTION_PERMILLE) *
                     (HLINE_BASE_PWM_PERMILLE - HLINE_CURVE_BASE_MIN_PWM_PERMILLE)) /
                    (HLINE_CORRECTION_LIMIT_PERMILLE - HLINE_CENTER_CORRECTION_PERMILLE);
        if (base_pwm < HLINE_CURVE_BASE_MIN_PWM_PERMILLE)
        {
            base_pwm = HLINE_CURVE_BASE_MIN_PWM_PERMILLE;
        }
    }

    *correction = (int16_t)output;
    *left_pwm = LineFollow_ClampPwm(base_pwm + output);
    *right_pwm = LineFollow_ClampPwm(base_pwm - output);

    controller->previous_error10 = control_error10;
    controller->last_left_pwm = *left_pwm;
    controller->last_right_pwm = *right_pwm;
    controller->last_correction = *correction;
    controller->has_previous_error = 1U;
    controller->lost_frames = 0U;
    return 1U;
}

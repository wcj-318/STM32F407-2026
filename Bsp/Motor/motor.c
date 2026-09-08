#include "motor.h"

#include "main.h"
#include "tim.h"

static int16_t Motor_Clamp(int16_t pwm)
{
    if (pwm > HLINE_MOTOR_PWM_MAX_PERMILLE)
    {
        return HLINE_MOTOR_PWM_MAX_PERMILLE;
    }
    if (pwm < -HLINE_MOTOR_PWM_MAX_PERMILLE)
    {
        return -HLINE_MOTOR_PWM_MAX_PERMILLE;
    }
    return pwm;
}

static void Motor_SetDirection(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                               GPIO_TypeDef *in2_port, uint16_t in2_pin,
                               int16_t pwm, GPIO_PinState forward_in1_state)
{
    GPIO_PinState in1_state = forward_in1_state;
    GPIO_PinState in2_state = (forward_in1_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;

    if (pwm < 0)
    {
        GPIO_PinState temporary = in1_state;
        in1_state = in2_state;
        in2_state = temporary;
    }
    else if (pwm == 0)
    {
        in1_state = GPIO_PIN_RESET;
        in2_state = GPIO_PIN_RESET;
    }

    HAL_GPIO_WritePin(in1_port, in1_pin, in1_state);
    HAL_GPIO_WritePin(in2_port, in2_pin, in2_state);
}

static uint32_t Motor_PermilleToCompare(int16_t pwm)
{
    uint32_t magnitude = (pwm < 0) ? (uint32_t)(-pwm) : (uint32_t)pwm;
    uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;

    return (magnitude * period_counts) / HLINE_MOTOR_PWM_MAX_PERMILLE;
}

HAL_StatusTypeDef Motor_Init(void)
{
    Motor_Stop();
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK)
    {
        Motor_Stop();
        return HAL_ERROR;
    }
    return HAL_OK;
}

void Motor_SetSignedPermille(int16_t left_pwm, int16_t right_pwm)
{
    left_pwm = Motor_Clamp(left_pwm);
    right_pwm = Motor_Clamp(right_pwm);

    Motor_SetDirection(HLINE_MOTOR_LEFT_IN1_GPIO_Port, HLINE_MOTOR_LEFT_IN1_Pin,
                       HLINE_MOTOR_LEFT_IN2_GPIO_Port, HLINE_MOTOR_LEFT_IN2_Pin,
                       left_pwm, HLINE_MOTOR_LEFT_FORWARD_IN1_STATE);
    Motor_SetDirection(HLINE_MOTOR_RIGHT_IN1_GPIO_Port, HLINE_MOTOR_RIGHT_IN1_Pin,
                       HLINE_MOTOR_RIGHT_IN2_GPIO_Port, HLINE_MOTOR_RIGHT_IN2_Pin,
                       right_pwm, HLINE_MOTOR_RIGHT_FORWARD_IN1_STATE);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, Motor_PermilleToCompare(left_pwm));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, Motor_PermilleToCompare(right_pwm));

    if ((left_pwm == 0) && (right_pwm == 0))
    {
        HAL_GPIO_WritePin(HLINE_MOTOR_STBY_GPIO_Port, HLINE_MOTOR_STBY_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(HLINE_MOTOR_STBY_GPIO_Port, HLINE_MOTOR_STBY_Pin, GPIO_PIN_SET);
    }
}

void Motor_Brake(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    HAL_GPIO_WritePin(HLINE_MOTOR_LEFT_IN1_GPIO_Port, HLINE_MOTOR_LEFT_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HLINE_MOTOR_LEFT_IN2_GPIO_Port, HLINE_MOTOR_LEFT_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HLINE_MOTOR_RIGHT_IN1_GPIO_Port, HLINE_MOTOR_RIGHT_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HLINE_MOTOR_RIGHT_IN2_GPIO_Port, HLINE_MOTOR_RIGHT_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HLINE_MOTOR_STBY_GPIO_Port, HLINE_MOTOR_STBY_Pin, GPIO_PIN_SET);
}

void Motor_Stop(void)
{
    HAL_GPIO_WritePin(HLINE_MOTOR_STBY_GPIO_Port, HLINE_MOTOR_STBY_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0U);
    HAL_GPIO_WritePin(HLINE_MOTOR_LEFT_IN1_GPIO_Port, HLINE_MOTOR_LEFT_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HLINE_MOTOR_LEFT_IN2_GPIO_Port, HLINE_MOTOR_LEFT_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HLINE_MOTOR_RIGHT_IN1_GPIO_Port, HLINE_MOTOR_RIGHT_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HLINE_MOTOR_RIGHT_IN2_GPIO_Port, HLINE_MOTOR_RIGHT_IN2_Pin, GPIO_PIN_RESET);
}

static int16_t SpeedControl_ClampPwm(int32_t pwm)
{
    if (pwm > HLINE_SPEED_OUTPUT_MAX_PERMILLE)
    {
        return HLINE_SPEED_OUTPUT_MAX_PERMILLE;
    }
    if (pwm < 0)
    {
        return 0;
    }
    return (int16_t)pwm;
}

static int32_t SpeedControl_ClampIntegral(int32_t integral)
{
    const int32_t limit = HLINE_SPEED_CORRECTION_LIMIT_PERMILLE *
                          HLINE_SPEED_KI_DENOMINATOR;

    if (integral > limit)
    {
        return limit;
    }
    if (integral < -limit)
    {
        return -limit;
    }
    return integral;
}

static int16_t SpeedControl_Slew(int16_t previous, int16_t target)
{
    if (target > (int16_t)(previous + HLINE_SPEED_OUTPUT_SLEW_PER_STEP))
    {
        return (int16_t)(previous + HLINE_SPEED_OUTPUT_SLEW_PER_STEP);
    }
    if (target < (int16_t)(previous - HLINE_SPEED_OUTPUT_SLEW_PER_STEP))
    {
        return (int16_t)(previous - HLINE_SPEED_OUTPUT_SLEW_PER_STEP);
    }
    return target;
}

static void SpeedControl_ResetAxis(SpeedControl_Axis_t *axis)
{
    axis->integral_error = 0;
    axis->filtered_cps = 0;
    axis->output_pwm = 0;
    axis->filter_initialized = 0U;
}

static int16_t SpeedControl_AxisStep(SpeedControl_Axis_t *axis,
                                     int16_t request_pwm,
                                     int32_t reference_cps,
                                     int32_t measured_cps,
                                     int32_t *target_cps)
{
    int32_t error;
    int32_t proportional;
    int32_t candidate_integral;
    int32_t correction;
    int32_t candidate_pwm;
    int16_t limited_pwm;

    if (request_pwm <= 0)
    {
        SpeedControl_ResetAxis(axis);
        *target_cps = 0;
        return 0;
    }

    if (axis->filter_initialized == 0U)
    {
        axis->filtered_cps = measured_cps;
        axis->filter_initialized = 1U;
    }
    else
    {
        axis->filtered_cps += (measured_cps - axis->filtered_cps) /
                              HLINE_SPEED_FILTER_DIVISOR;
    }

    *target_cps = ((int32_t)request_pwm * reference_cps) /
                  HLINE_SPEED_REFERENCE_PWM_PERMILLE;
    error = *target_cps - axis->filtered_cps;
    proportional = (error * HLINE_SPEED_KP_NUMERATOR) /
                   HLINE_SPEED_KP_DENOMINATOR;
    candidate_integral = SpeedControl_ClampIntegral(axis->integral_error + error);
    correction = proportional + candidate_integral / HLINE_SPEED_KI_DENOMINATOR;

    /* Do not accumulate further while the PI correction is saturated. */
    if ((correction <= HLINE_SPEED_CORRECTION_LIMIT_PERMILLE) &&
        (correction >= -HLINE_SPEED_CORRECTION_LIMIT_PERMILLE))
    {
        axis->integral_error = candidate_integral;
    }
    else if (correction > HLINE_SPEED_CORRECTION_LIMIT_PERMILLE)
    {
        correction = HLINE_SPEED_CORRECTION_LIMIT_PERMILLE;
    }
    else
    {
        correction = -HLINE_SPEED_CORRECTION_LIMIT_PERMILLE;
    }

    candidate_pwm = (int32_t)request_pwm + correction;
    limited_pwm = SpeedControl_ClampPwm(candidate_pwm);
    axis->output_pwm = SpeedControl_Slew(axis->output_pwm, limited_pwm);
    return axis->output_pwm;
}

void SpeedControl_Reset(SpeedControl_Controller_t *controller)
{
    SpeedControl_ResetAxis(&controller->left);
    SpeedControl_ResetAxis(&controller->right);
    controller->left_target_cps = 0;
    controller->right_target_cps = 0;
    controller->left_measured_cps = 0;
    controller->right_measured_cps = 0;
    controller->left_output_pwm = 0;
    controller->right_output_pwm = 0;
}

void SpeedControl_Step(SpeedControl_Controller_t *controller,
                       int16_t left_request_pwm, int16_t right_request_pwm,
                       int32_t left_raw_cps, int32_t right_raw_cps)
{
    controller->left_measured_cps = left_raw_cps;
    controller->right_measured_cps = -right_raw_cps;

    controller->left_output_pwm = SpeedControl_AxisStep(
        &controller->left, left_request_pwm, HLINE_SPEED_LEFT_REFERENCE_CPS,
        controller->left_measured_cps, &controller->left_target_cps);
    controller->right_output_pwm = SpeedControl_AxisStep(
        &controller->right, right_request_pwm, HLINE_SPEED_RIGHT_REFERENCE_CPS,
        controller->right_measured_cps, &controller->right_target_cps);
}

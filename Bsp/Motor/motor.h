#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"

#define HLINE_MOTOR_PWM_MAX_PERMILLE          1000

/*
 * These are the initial electrical forward levels. Confirm the mechanical
 * direction with lifted wheels, then change only the affected side if needed.
 */
#define HLINE_MOTOR_LEFT_FORWARD_IN1_STATE    GPIO_PIN_SET
#define HLINE_MOTOR_RIGHT_FORWARD_IN1_STATE   GPIO_PIN_SET

#define HLINE_SPEED_MODE_LEFT_ONLY   1U
#define HLINE_SPEED_MODE_RIGHT_ONLY  2U
#define HLINE_SPEED_MODE_DUAL_FIXED  3U
#define HLINE_SPEED_MODE_LINE_TRACK  4U

/* Phase 4 final mode: gray-line outer loop with dual encoder speed PI. */
#define HLINE_SPEED_RUN_MODE                  HLINE_SPEED_MODE_LINE_TRACK
#define HLINE_SPEED_TEST_PWM_PERMILLE         300

#define HLINE_SPEED_CONTROL_PERIOD_MS          20U
#define HLINE_SPEED_LEFT_COUNTS_PER_REV_X100   146895L
#define HLINE_SPEED_RIGHT_COUNTS_PER_REV_X100  147120L
#define HLINE_SPEED_REFERENCE_PWM_PERMILLE     300
#define HLINE_SPEED_LEFT_REFERENCE_CPS         2325L
#define HLINE_SPEED_RIGHT_REFERENCE_CPS        2319L
#define HLINE_SPEED_KP_NUMERATOR               1L
#define HLINE_SPEED_KP_DENOMINATOR            10L
#define HLINE_SPEED_KI_DENOMINATOR          2000L
#define HLINE_SPEED_CORRECTION_LIMIT_PERMILLE 150L
#define HLINE_SPEED_OUTPUT_SLEW_PER_STEP       20
#define HLINE_SPEED_FILTER_DIVISOR               4L
#define HLINE_SPEED_OUTPUT_MAX_PERMILLE        600

#if ((HLINE_SPEED_RUN_MODE < HLINE_SPEED_MODE_LEFT_ONLY) || \
     (HLINE_SPEED_RUN_MODE > HLINE_SPEED_MODE_LINE_TRACK))
#error "Invalid HLINE_SPEED_RUN_MODE"
#endif

typedef struct
{
    int32_t integral_error;
    int32_t filtered_cps;
    int16_t output_pwm;
    uint8_t filter_initialized;
} SpeedControl_Axis_t;

typedef struct
{
    SpeedControl_Axis_t left;
    SpeedControl_Axis_t right;
    int32_t left_target_cps;
    int32_t right_target_cps;
    int32_t left_measured_cps;
    int32_t right_measured_cps;
    int16_t left_output_pwm;
    int16_t right_output_pwm;
} SpeedControl_Controller_t;

HAL_StatusTypeDef Motor_Init(void);
void Motor_SetSignedPermille(int16_t left_pwm, int16_t right_pwm);
void Motor_Brake(void);
void Motor_Stop(void);
void SpeedControl_Reset(SpeedControl_Controller_t *controller);
void SpeedControl_Step(SpeedControl_Controller_t *controller,
                       int16_t left_request_pwm, int16_t right_request_pwm,
                       int32_t left_raw_cps, int32_t right_raw_cps);

#endif

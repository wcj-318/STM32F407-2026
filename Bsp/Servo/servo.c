#include "servo.h"

#include "tim.h"

static uint16_t servo_angle_deg;
static int8_t servo_sweep_direction = 1;
static uint32_t servo_last_step_ms;

void Servo_SetAngle(uint16_t angle_deg)
{
    uint32_t pulse_us;

    if (angle_deg > HLINE_SERVO_ANGLE_LIMIT_DEG)
    {
        angle_deg = HLINE_SERVO_ANGLE_LIMIT_DEG;
    }

    pulse_us = HLINE_SERVO_PULSE_MIN_US +
               ((uint32_t)angle_deg *
                (HLINE_SERVO_PULSE_MAX_US - HLINE_SERVO_PULSE_MIN_US)) /
               HLINE_SERVO_ANGLE_LIMIT_DEG;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pulse_us);
    servo_angle_deg = angle_deg;
}

HAL_StatusTypeDef Servo_Init(void)
{
    Servo_SetAngle(HLINE_SERVO_SWEEP_LOW_ANGLE_DEG);
    return HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
}

void Servo_StartSweep(uint32_t now_ms)
{
    servo_sweep_direction = 1;
    servo_last_step_ms = now_ms;
    Servo_SetAngle(HLINE_SERVO_SWEEP_LOW_ANGLE_DEG);
}

void Servo_SweepStep(uint32_t now_ms)
{
    uint32_t step_count = (now_ms - servo_last_step_ms) /
                          HLINE_SERVO_SWEEP_STEP_PERIOD_MS;

    if (step_count == 0U)
    {
        return;
    }
    servo_last_step_ms += step_count * HLINE_SERVO_SWEEP_STEP_PERIOD_MS;

    while (step_count-- > 0U)
    {
        if (servo_sweep_direction > 0)
        {
            if ((servo_angle_deg + HLINE_SERVO_SWEEP_STEP_DEG) >=
                HLINE_SERVO_SWEEP_HIGH_ANGLE_DEG)
            {
                servo_angle_deg = HLINE_SERVO_SWEEP_HIGH_ANGLE_DEG;
                servo_sweep_direction = -1;
            }
            else
            {
                servo_angle_deg += HLINE_SERVO_SWEEP_STEP_DEG;
            }
        }
        else if (servo_angle_deg <= (HLINE_SERVO_SWEEP_LOW_ANGLE_DEG +
                                     HLINE_SERVO_SWEEP_STEP_DEG))
        {
            servo_angle_deg = HLINE_SERVO_SWEEP_LOW_ANGLE_DEG;
            servo_sweep_direction = 1;
        }
        else
        {
            servo_angle_deg -= HLINE_SERVO_SWEEP_STEP_DEG;
        }
    }

    Servo_SetAngle(servo_angle_deg);
}

void Servo_StopSweep(void)
{
    servo_sweep_direction = 1;
    Servo_SetAngle(HLINE_SERVO_SWEEP_LOW_ANGLE_DEG);
}

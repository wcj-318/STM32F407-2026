#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"

#define HLINE_SERVO_ANGLE_LIMIT_DEG          180U
#define HLINE_SERVO_PULSE_MIN_US            1000U
#define HLINE_SERVO_PULSE_MAX_US            2000U
#define HLINE_SERVO_SWEEP_LOW_ANGLE_DEG         0U
#define HLINE_SERVO_SWEEP_HIGH_ANGLE_DEG       45U
#define HLINE_SERVO_SWEEP_STEP_DEG              1U
#define HLINE_SERVO_SWEEP_STEP_PERIOD_MS        20U

HAL_StatusTypeDef Servo_Init(void);
void Servo_SetAngle(uint16_t angle_deg);
void Servo_StartSweep(uint32_t now_ms);
void Servo_SweepStep(uint32_t now_ms);
void Servo_StopSweep(void);

#endif

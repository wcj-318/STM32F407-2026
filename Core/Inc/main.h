/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RED_Pin GPIO_PIN_5
#define LED_RED_GPIO_Port GPIOC
#define LED_BLUE_Pin GPIO_PIN_2
#define LED_BLUE_GPIO_Port GPIOB
/* On-board KEY1 only: pressing it drives PA0 high; KEY2/NRST remains reset. */
#define START_KEY_Pin GPIO_PIN_0
#define START_KEY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

#define HLINE_MOTOR_LEFT_PWM_Pin GPIO_PIN_13
#define HLINE_MOTOR_LEFT_PWM_GPIO_Port GPIOD
#define HLINE_MOTOR_RIGHT_PWM_Pin GPIO_PIN_14
#define HLINE_MOTOR_RIGHT_PWM_GPIO_Port GPIOD
#define HLINE_MOTOR_LEFT_IN1_Pin GPIO_PIN_0
#define HLINE_MOTOR_LEFT_IN1_GPIO_Port GPIOC
#define HLINE_MOTOR_LEFT_IN2_Pin GPIO_PIN_1
#define HLINE_MOTOR_LEFT_IN2_GPIO_Port GPIOC
#define HLINE_MOTOR_RIGHT_IN1_Pin GPIO_PIN_2
#define HLINE_MOTOR_RIGHT_IN1_GPIO_Port GPIOC
#define HLINE_MOTOR_RIGHT_IN2_Pin GPIO_PIN_3
#define HLINE_MOTOR_RIGHT_IN2_GPIO_Port GPIOC
#define HLINE_MOTOR_STBY_Pin GPIO_PIN_4
#define HLINE_MOTOR_STBY_GPIO_Port GPIOC

#define HLINE_ENCODER_LEFT_A_Pin GPIO_PIN_5
#define HLINE_ENCODER_LEFT_A_GPIO_Port GPIOA
#define HLINE_ENCODER_LEFT_B_Pin GPIO_PIN_3
#define HLINE_ENCODER_LEFT_B_GPIO_Port GPIOB
#define HLINE_ENCODER_RIGHT_A_Pin GPIO_PIN_6
#define HLINE_ENCODER_RIGHT_A_GPIO_Port GPIOC
#define HLINE_ENCODER_RIGHT_B_Pin GPIO_PIN_7
#define HLINE_ENCODER_RIGHT_B_GPIO_Port GPIOC

#define HLINE_SERVO_PWM_Pin GPIO_PIN_1
#define HLINE_SERVO_PWM_GPIO_Port GPIOA

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

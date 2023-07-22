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
#include "stm32f1xx_hal.h"

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
void process_requests();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define controlMode_Pin GPIO_PIN_1
#define controlMode_GPIO_Port GPIOA
#define controlMode_EXTI_IRQn EXTI1_IRQn
#define Control_Input1_Pin GPIO_PIN_4
#define Control_Input1_GPIO_Port GPIOA
#define Control_Input1_EXTI_IRQn EXTI4_IRQn
#define Control_Input2_Pin GPIO_PIN_5
#define Control_Input2_GPIO_Port GPIOA
#define Control_Input2_EXTI_IRQn EXTI9_5_IRQn
#define levelSwitch1_Pin GPIO_PIN_6
#define levelSwitch1_GPIO_Port GPIOA
#define levelSwitch1_EXTI_IRQn EXTI9_5_IRQn
#define levelSwitch2_Pin GPIO_PIN_7
#define levelSwitch2_GPIO_Port GPIOA
#define levelSwitch2_EXTI_IRQn EXTI9_5_IRQn
#define TPIC_SER_IN_Pin GPIO_PIN_0
#define TPIC_SER_IN_GPIO_Port GPIOB
#define TPIC_SRCLK_Pin GPIO_PIN_1
#define TPIC_SRCLK_GPIO_Port GPIOB
#define TPIC_RCK_Pin GPIO_PIN_2
#define TPIC_RCK_GPIO_Port GPIOB
#define stepMotor_1_puls_Pin GPIO_PIN_12
#define stepMotor_1_puls_GPIO_Port GPIOB
#define stepMotor_2_dir_Pin GPIO_PIN_13
#define stepMotor_2_dir_GPIO_Port GPIOB
#define stepMotor_2_enable_Pin GPIO_PIN_14
#define stepMotor_2_enable_GPIO_Port GPIOB
#define stepMotor_2_puls_Pin GPIO_PIN_15
#define stepMotor_2_puls_GPIO_Port GPIOB
#define stepMotor_1_dir_Pin GPIO_PIN_11
#define stepMotor_1_dir_GPIO_Port GPIOA
#define stepMotor_1_enable_Pin GPIO_PIN_12
#define stepMotor_1_enable_GPIO_Port GPIOA
#define sysStatusLed_Pin GPIO_PIN_8
#define sysStatusLed_GPIO_Port GPIOB
#define levelSwitch_Pin GPIO_PIN_9
#define levelSwitch_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

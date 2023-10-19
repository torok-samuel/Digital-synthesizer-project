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
#include "stm32g0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define C5_Pin GPIO_PIN_9
#define C5_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_15
#define STATUS_LED_GPIO_Port GPIOC
#define R5_Pin GPIO_PIN_8
#define R5_GPIO_Port GPIOA
#define R4_Pin GPIO_PIN_6
#define R4_GPIO_Port GPIOC
#define R3_Pin GPIO_PIN_11
#define R3_GPIO_Port GPIOA
#define R2_Pin GPIO_PIN_12
#define R2_GPIO_Port GPIOA
#define R1_Pin GPIO_PIN_15
#define R1_GPIO_Port GPIOA
#define C1_Pin GPIO_PIN_3
#define C1_GPIO_Port GPIOB
#define C2_Pin GPIO_PIN_4
#define C2_GPIO_Port GPIOB
#define C3_Pin GPIO_PIN_5
#define C3_GPIO_Port GPIOB
#define LEDS_Pin GPIO_PIN_7
#define LEDS_GPIO_Port GPIOB
#define C4_Pin GPIO_PIN_8
#define C4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

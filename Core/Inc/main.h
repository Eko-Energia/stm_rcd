/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define RCD_PWM_Pin GPIO_PIN_0
#define RCD_PWM_GPIO_Port GPIOA
#define TYPE2_LED_RED_Pin GPIO_PIN_1
#define TYPE2_LED_RED_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_3
#define LED_GREEN_GPIO_Port GPIOA
#define TYPE2_LED_GREEN_Pin GPIO_PIN_4
#define TYPE2_LED_GREEN_GPIO_Port GPIOA
#define L1_SENSOR_Pin GPIO_PIN_5
#define L1_SENSOR_GPIO_Port GPIOA
#define L2_SENSOR_Pin GPIO_PIN_6
#define L2_SENSOR_GPIO_Port GPIOA
#define L3_SENSOR_Pin GPIO_PIN_7
#define L3_SENSOR_GPIO_Port GPIOA
#define PP_Pin GPIO_PIN_0
#define PP_GPIO_Port GPIOB
#define CP_Pin GPIO_PIN_8
#define CP_GPIO_Port GPIOA
#define RCD_FAULT_Pin GPIO_PIN_3
#define RCD_FAULT_GPIO_Port GPIOB
#define RCD_ERROR_Pin GPIO_PIN_4
#define RCD_ERROR_GPIO_Port GPIOB
#define RCD_TEST_Pin GPIO_PIN_6
#define RCD_TEST_GPIO_Port GPIOB
#define START_CHARGING_Pin GPIO_PIN_7
#define START_CHARGING_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

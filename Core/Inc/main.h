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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PRESSURE_Pin GPIO_PIN_7
#define PRESSURE_GPIO_Port GPIOA
#define CURRENT1_Pin GPIO_PIN_4
#define CURRENT1_GPIO_Port GPIOC
#define CURRENT2_Pin GPIO_PIN_5
#define CURRENT2_GPIO_Port GPIOC
#define CURRENT3_Pin GPIO_PIN_0
#define CURRENT3_GPIO_Port GPIOB
#define CURRENT4_Pin GPIO_PIN_1
#define CURRENT4_GPIO_Port GPIOB
#define ADDR_OUT_3_Pin GPIO_PIN_7
#define ADDR_OUT_3_GPIO_Port GPIOE
#define ADDR_IN_3_Pin GPIO_PIN_8
#define ADDR_IN_3_GPIO_Port GPIOE
#define ADDR_OUT_2_Pin GPIO_PIN_9
#define ADDR_OUT_2_GPIO_Port GPIOE
#define ADDR_IN_2_Pin GPIO_PIN_10
#define ADDR_IN_2_GPIO_Port GPIOE
#define ADDR_OUT_1_Pin GPIO_PIN_11
#define ADDR_OUT_1_GPIO_Port GPIOE
#define ADDR_IN_1__Pin GPIO_PIN_12
#define ADDR_IN_1__GPIO_Port GPIOE
#define SSR1_Pin GPIO_PIN_6
#define SSR1_GPIO_Port GPIOC
#define SSR2_Pin GPIO_PIN_7
#define SSR2_GPIO_Port GPIOC
#define SSR3_Pin GPIO_PIN_8
#define SSR3_GPIO_Port GPIOC
#define SSR4_Pin GPIO_PIN_9
#define SSR4_GPIO_Port GPIOC
#define DE_Pin GPIO_PIN_8
#define DE_GPIO_Port GPIOA
#define OUTPUT1_Pin GPIO_PIN_11
#define OUTPUT1_GPIO_Port GPIOC
#define OUTPUT2_Pin GPIO_PIN_12
#define OUTPUT2_GPIO_Port GPIOC
#define OUTPUT3_Pin GPIO_PIN_0
#define OUTPUT3_GPIO_Port GPIOD
#define OUTPUT4_Pin GPIO_PIN_1
#define OUTPUT4_GPIO_Port GPIOD
#define INPUT1_Pin GPIO_PIN_2
#define INPUT1_GPIO_Port GPIOD
#define INPUT2_Pin GPIO_PIN_3
#define INPUT2_GPIO_Port GPIOD
#define INPUT3_Pin GPIO_PIN_4
#define INPUT3_GPIO_Port GPIOD
#define INPUT4_Pin GPIO_PIN_5
#define INPUT4_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32l5xx_hal.h"


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
#define DRV_FLT_Pin GPIO_PIN_13
#define DRV_FLT_GPIO_Port GPIOC
#define DRV_OFF_Pin GPIO_PIN_14
#define DRV_OFF_GPIO_Port GPIOC
#define DRV_CS_Pin GPIO_PIN_15
#define DRV_CS_GPIO_Port GPIOC
#define DRV_IPROPI_Pin GPIO_PIN_0
#define DRV_IPROPI_GPIO_Port GPIOA
#define T_FET_Pin GPIO_PIN_1
#define T_FET_GPIO_Port GPIOA
#define V_BUS_Pin GPIO_PIN_2
#define V_BUS_GPIO_Port GPIOA
#define T_MOT_Pin GPIO_PIN_3
#define T_MOT_GPIO_Port GPIOA
#define POT_Pin GPIO_PIN_4
#define POT_GPIO_Port GPIOA
#define LEDB_Pin GPIO_PIN_0
#define LEDB_GPIO_Port GPIOB
#define LEDG_Pin GPIO_PIN_1
#define LEDG_GPIO_Port GPIOB
#define LEDR_Pin GPIO_PIN_2
#define LEDR_GPIO_Port GPIOB
#define DRV_IN1_Pin GPIO_PIN_10
#define DRV_IN1_GPIO_Port GPIOB
#define DRV_IN2_Pin GPIO_PIN_11
#define DRV_IN2_GPIO_Port GPIOB
#define ENC_CS_Pin GPIO_PIN_12
#define ENC_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

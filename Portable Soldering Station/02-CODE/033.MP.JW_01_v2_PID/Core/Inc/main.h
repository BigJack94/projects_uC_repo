/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define OUT_CONV_ACTIVE_Pin GPIO_PIN_15
#define OUT_CONV_ACTIVE_GPIO_Port GPIOC
#define CONV_U_OUT_sens_Pin GPIO_PIN_0
#define CONV_U_OUT_sens_GPIO_Port GPIOA
#define TEMP_sens_Pin GPIO_PIN_1
#define TEMP_sens_GPIO_Port GPIOA
#define BATT_U_sens_Pin GPIO_PIN_2
#define BATT_U_sens_GPIO_Port GPIOA
#define CONV_I_OUT_sens_Pin GPIO_PIN_3
#define CONV_I_OUT_sens_GPIO_Port GPIOA
#define OUT_LED_Pin GPIO_PIN_4
#define OUT_LED_GPIO_Port GPIOA
#define ENCODER_SW_Pin GPIO_PIN_5
#define ENCODER_SW_GPIO_Port GPIOA
#define ENCODER_A_Pin GPIO_PIN_6
#define ENCODER_A_GPIO_Port GPIOA
#define ENCODER_B_Pin GPIO_PIN_7
#define ENCODER_B_GPIO_Port GPIOA
#define CONV_FB_PWM_Pin GPIO_PIN_1
#define CONV_FB_PWM_GPIO_Port GPIOB
#define OUT_RELAY_Pin GPIO_PIN_3
#define OUT_RELAY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

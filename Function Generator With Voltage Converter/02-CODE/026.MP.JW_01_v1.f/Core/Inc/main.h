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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_A_Pin GPIO_PIN_0
#define SW_A_GPIO_Port GPIOA
#define SW_B_Pin GPIO_PIN_1
#define SW_B_GPIO_Port GPIOA
#define OUT_VDC_N_sens_Pin GPIO_PIN_2
#define OUT_VDC_N_sens_GPIO_Port GPIOA
#define SIGNAL_POT_sens_Pin GPIO_PIN_3
#define SIGNAL_POT_sens_GPIO_Port GPIOA
#define OFFSET_POT_sens_Pin GPIO_PIN_4
#define OFFSET_POT_sens_GPIO_Port GPIOA
#define I_OUT_BB_sens_Pin GPIO_PIN_5
#define I_OUT_BB_sens_GPIO_Port GPIOA
#define OUT_BB_sens_Pin GPIO_PIN_6
#define OUT_BB_sens_GPIO_Port GPIOA
#define OUT_DAC_sens_Pin GPIO_PIN_7
#define OUT_DAC_sens_GPIO_Port GPIOA
#define OUT_VDC_P_sens_Pin GPIO_PIN_0
#define OUT_VDC_P_sens_GPIO_Port GPIOB
#define VCC_sens_Pin GPIO_PIN_1
#define VCC_sens_GPIO_Port GPIOB
#define SW_ENCODER_Pin GPIO_PIN_11
#define SW_ENCODER_GPIO_Port GPIOB
#define LED_WORK_DAC_Pin GPIO_PIN_12
#define LED_WORK_DAC_GPIO_Port GPIOB
#define LED_STOP_BB_Pin GPIO_PIN_13
#define LED_STOP_BB_GPIO_Port GPIOB
#define LED_WORK_BB_Pin GPIO_PIN_14
#define LED_WORK_BB_GPIO_Port GPIOB
#define SW_WORK_DAC_Pin GPIO_PIN_15
#define SW_WORK_DAC_GPIO_Port GPIOB
#define SW_WORK_BB_Pin GPIO_PIN_8
#define SW_WORK_BB_GPIO_Port GPIOA
#define BB_PWM_Pin GPIO_PIN_9
#define BB_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

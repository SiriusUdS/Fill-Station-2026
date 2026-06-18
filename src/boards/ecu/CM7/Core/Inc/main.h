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
#include "stm32h7xx_hal.h"

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
#define PWM_VALVE_IPA_Pin GPIO_PIN_5
#define PWM_VALVE_IPA_GPIO_Port GPIOE
#define PWM_VALVE_NOS_Pin GPIO_PIN_6
#define PWM_VALVE_NOS_GPIO_Port GPIOE
#define SWITCH_VALVE_NOS_CLOSED_Pin GPIO_PIN_0
#define SWITCH_VALVE_NOS_CLOSED_GPIO_Port GPIOF
#define SWITCH_VALVE_NOS_OPENED_Pin GPIO_PIN_1
#define SWITCH_VALVE_NOS_OPENED_GPIO_Port GPIOF
#define SWITCH_VALVE_IPA_CLOSED_Pin GPIO_PIN_2
#define SWITCH_VALVE_IPA_CLOSED_GPIO_Port GPIOF
#define SWITCH_VALVE_IPA_OPENED_Pin GPIO_PIN_3
#define SWITCH_VALVE_IPA_OPENED_GPIO_Port GPIOF
#define BUZZER_STATE_Pin GPIO_PIN_6
#define BUZZER_STATE_GPIO_Port GPIOF
#define LED1_STATE_Pin GPIO_PIN_7
#define LED1_STATE_GPIO_Port GPIOF
#define LED2_STATE_Pin GPIO_PIN_8
#define LED2_STATE_GPIO_Port GPIOF
#define LED3_STATE_Pin GPIO_PIN_9
#define LED3_STATE_GPIO_Port GPIOF
#define ADS_DRDY_Pin GPIO_PIN_4
#define ADS_DRDY_GPIO_Port GPIOA
#define ADS_SCK_Pin GPIO_PIN_5
#define ADS_SCK_GPIO_Port GPIOA
#define ADS_MISO_Pin GPIO_PIN_6
#define ADS_MISO_GPIO_Port GPIOA
#define ADS_MOSI_Pin GPIO_PIN_7
#define ADS_MOSI_GPIO_Port GPIOA
#define ADS_CS_Pin GPIO_PIN_5
#define ADS_CS_GPIO_Port GPIOC
#define SD1_DET_Pin GPIO_PIN_7
#define SD1_DET_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

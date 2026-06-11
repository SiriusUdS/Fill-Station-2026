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
#define FILL_SWITCH_CLOSED_Pin GPIO_PIN_2
#define FILL_SWITCH_CLOSED_GPIO_Port GPIOE
#define FILL_SWITCH_OPENED_Pin GPIO_PIN_3
#define FILL_SWITCH_OPENED_GPIO_Port GPIOE
#define DUMP_SWITCH_CLOSED_Pin GPIO_PIN_4
#define DUMP_SWITCH_CLOSED_GPIO_Port GPIOE
#define HEATER_STATE_Pin GPIO_PIN_5
#define HEATER_STATE_GPIO_Port GPIOE
#define PWM_VALVE_DUMP_Pin GPIO_PIN_6
#define PWM_VALVE_DUMP_GPIO_Port GPIOE
#define LED1_STATE_Pin GPIO_PIN_1
#define LED1_STATE_GPIO_Port GPIOF
#define LED2_STATE_Pin GPIO_PIN_2
#define LED2_STATE_GPIO_Port GPIOF
#define LED3_STATE_Pin GPIO_PIN_3
#define LED3_STATE_GPIO_Port GPIOF
#define PM_CRIT_Pin GPIO_PIN_12
#define PM_CRIT_GPIO_Port GPIOF
#define PM_WARN_Pin GPIO_PIN_13
#define PM_WARN_GPIO_Port GPIOF
#define PM_SCL_Pin GPIO_PIN_14
#define PM_SCL_GPIO_Port GPIOF
#define PM_SDA_Pin GPIO_PIN_15
#define PM_SDA_GPIO_Port GPIOF
#define ADS_DRDY_Pin GPIO_PIN_7
#define ADS_DRDY_GPIO_Port GPIOE
#define TC2_CS_Pin GPIO_PIN_8
#define TC2_CS_GPIO_Port GPIOE
#define PWM_VALVE_FILL_Pin GPIO_PIN_9
#define PWM_VALVE_FILL_GPIO_Port GPIOE
#define ADS_SCK_SPI4_Pin GPIO_PIN_12
#define ADS_SCK_SPI4_GPIO_Port GPIOE
#define ADS_MISO_SPI4_Pin GPIO_PIN_13
#define ADS_MISO_SPI4_GPIO_Port GPIOE
#define ADS_MOSI_SPI4_Pin GPIO_PIN_14
#define ADS_MOSI_SPI4_GPIO_Port GPIOE
#define ADS_CS_Pin GPIO_PIN_15
#define ADS_CS_GPIO_Port GPIOE
#define EMATCH_CONT_Pin GPIO_PIN_8
#define EMATCH_CONT_GPIO_Port GPIOD
#define SOL_VALVE_CONT_Pin GPIO_PIN_9
#define SOL_VALVE_CONT_GPIO_Port GPIOD
#define BUZZER_STATE_Pin GPIO_PIN_10
#define BUZZER_STATE_GPIO_Port GPIOD
#define EMATCH_STATE_Pin GPIO_PIN_12
#define EMATCH_STATE_GPIO_Port GPIOD
#define EMATCH_DET_Pin GPIO_PIN_13
#define EMATCH_DET_GPIO_Port GPIOD
#define SOL_VALVE_STATE_Pin GPIO_PIN_14
#define SOL_VALVE_STATE_GPIO_Port GPIOD
#define SOL_VALVE_DET_Pin GPIO_PIN_15
#define SOL_VALVE_DET_GPIO_Port GPIOD
#define TC1_CS_Pin GPIO_PIN_11
#define TC1_CS_GPIO_Port GPIOG
#define TC_MISO_Pin GPIO_PIN_12
#define TC_MISO_GPIO_Port GPIOG
#define TC_SCK_Pin GPIO_PIN_13
#define TC_SCK_GPIO_Port GPIOG
#define TC_MOSI_Pin GPIO_PIN_14
#define TC_MOSI_GPIO_Port GPIOG
#define TC3_CS_Pin GPIO_PIN_6
#define TC3_CS_GPIO_Port GPIOB
#define TC4_CS_Pin GPIO_PIN_7
#define TC4_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

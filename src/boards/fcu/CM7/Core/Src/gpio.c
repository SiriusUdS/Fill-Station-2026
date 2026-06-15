/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, HEATER_STATE_Pin|ADS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED1_STATE_Pin|LED2_STATE_Pin|LED3_STATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, EMATCH_CONT_Pin|SOL_VALVE_CONT_Pin|BUZZER_STATE_Pin|EMATCH_STATE_Pin
                          |SOL_VALVE_STATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TC1_CS_GPIO_Port, TC1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FILL_SWITCH_CLOSED_Pin FILL_SWITCH_OPENED_Pin DUMP_SWITCH_CLOSED_Pin ADS_DRDY_Pin
                           TC2_CS_Pin */
  GPIO_InitStruct.Pin = FILL_SWITCH_CLOSED_Pin|FILL_SWITCH_OPENED_Pin|DUMP_SWITCH_CLOSED_Pin|ADS_DRDY_Pin
                          |TC2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : HEATER_STATE_Pin ADS_CS_Pin */
  GPIO_InitStruct.Pin = HEATER_STATE_Pin|ADS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_STATE_Pin LED2_STATE_Pin LED3_STATE_Pin */
  GPIO_InitStruct.Pin = LED1_STATE_Pin|LED2_STATE_Pin|LED3_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PM_CRIT_Pin PM_WARN_Pin */
  GPIO_InitStruct.Pin = PM_CRIT_Pin|PM_WARN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : EMATCH_CONT_Pin SOL_VALVE_CONT_Pin BUZZER_STATE_Pin EMATCH_STATE_Pin
                           SOL_VALVE_STATE_Pin */
  GPIO_InitStruct.Pin = EMATCH_CONT_Pin|SOL_VALVE_CONT_Pin|BUZZER_STATE_Pin|EMATCH_STATE_Pin
                          |SOL_VALVE_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : EMATCH_DET_Pin SOL_VALVE_DET_Pin SD_DETECT_Pin */
  GPIO_InitStruct.Pin = EMATCH_DET_Pin|SOL_VALVE_DET_Pin|SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TC1_CS_Pin */
  GPIO_InitStruct.Pin = TC1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TC1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TC3_CS_Pin TC4_CS_Pin */
  GPIO_InitStruct.Pin = TC3_CS_Pin|TC4_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

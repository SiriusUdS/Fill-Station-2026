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
#ifndef __SPI_H
#define __SPI_H

#include "main.h"

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"
#include <stdint.h>

#define flags_reset 0xFC



//Unions

union SPI_Flags {
		uint16_t byte;      //Unused
    struct {
        uint8_t PADDING1 : 2;
        uint8_t SPI1_Done : 1;
        uint8_t SPI2_Done : 1;
        uint8_t SPI3_Done : 1;
        uint8_t SPI4_Done : 1;
        uint8_t SPI5_Done : 1;
        uint8_t SPI6_Done : 1;
        uint8_t PADDING2 : 2;
        uint8_t SPI1_New_Data : 1;
        uint8_t SPI2_New_Data : 1;
        uint8_t SPI3_New_Data : 1;
        uint8_t SPI4_New_Data : 1;
        uint8_t SPI5_New_Data : 1;
        uint8_t SPI6_New_Data : 1;
        
        
    } flags;
};

struct SPI4_HANDLE {

  uint8_t *txBuff_SPI4;
  uint8_t *rxBuff_SPI4;
  uint8_t length_spi4;
  uint32_t last_timestamp;
};

struct SPI6_HANDLE {

  // __attribute__((section(".SRAM4"))) uint8_t txBuff_SPI6[LENGTH_MESSAGE_SPI6];
  // __attribute__((section(".SRAM4"))) uint8_t rxBuff_SPI6[LENGTH_MESSAGE_SPI6];

  uint8_t *txBuff_SPI6;
  uint8_t *rxBuff_SPI6;
  uint8_t length_spi6;
  uint32_t last_timestamp;

};

struct SPI4_CONFIG {
  
  GPIO_TypeDef **spi4_cs_gpio_type;
  uint16_t *spi4_cs_gpio_number;
  uint16_t spi4_cs_num;
  

};

struct SPI6_CONFIG {

  GPIO_TypeDef **spi6_cs_gpio_type;
  uint16_t *spi6_cs_gpio_number;
  uint16_t spi6_cs_num;

};

//Extern declaration
extern struct SPI4_HANDLE  spi4_handle;
extern struct SPI6_HANDLE  spi6_handle;
extern struct SPI4_CONFIG  spi4_config;
extern struct SPI6_CONFIG  spi6_config;
extern union  SPI_Flags    spi_flags;

//Fonctions

void init_SPI();
void spi_write_read(SPI_HandleTypeDef *hspi);

#endif /* __MAIN_H */

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

//Constants
#define LENGTH_MESSAGE_SPI4 0
#define LENGTH_MESSAGE_SPI6 30

#define IDCH0 0
#define IDCH1 1
#define IDCH2 2
#define IDCH3 3
#define IDCH4 4
#define IDCH5 5
#define IDCH6 6
#define IDCH7 7
//Unions

union SPI_Flags {
		uint8_t byte;      //Unused
    struct {
        uint8_t SPI1_Done : 1;
        uint8_t SPI2_Done : 1;
        uint8_t SPI3_Done : 1;
        uint8_t SPI4_Done : 1;
        uint8_t SPI4_Counter : 2;
        uint8_t SPI5_Done : 1;
        uint8_t SPI6_Done : 1;
    } flags;
};




//Fonctions

void init_SPI();
void spi_write_read(uint8_t command[6], SPI_HandleTypeDef *hspi);

#endif /* __MAIN_H */

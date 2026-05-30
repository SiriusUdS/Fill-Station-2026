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
#define LENGTH_MESSAGE_SPI1 0
#define LENGTH_MESSAGE_SPI2 0
#define LENGTH_MESSAGE_SPI3 0
#define LENGTH_MESSAGE_SPI4 30



//Unions

union SPI_Flags {
		uint8_t byte;      //Unused
    struct {
        uint8_t SPI1_Done : 1;
        uint8_t SPI2_Done : 1;
        uint8_t SPI3_Done : 1;
        uint8_t SPI4_Done : 1;
        uint8_t b4 : 1;
        uint8_t b5 : 1;
        uint8_t b6 : 1;
        uint8_t b7 : 1;
    } flags;
};

//Variables


//Fonctions


void init_SPI(SPI_HandleTypeDef *hspi);
void spi_write_read(uint8_t command[6], SPI_HandleTypeDef *hspi);

#endif /* __MAIN_H */

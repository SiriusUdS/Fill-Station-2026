#include "SPI.h"
#include "main.h"  // For CS pin definitions
#include "stdio.h"
#include "stm32h7xx_hal.h"

union	SPI_Flags SPI;

uint8_t txBuff_SPI4[LENGTH_MESSAGE_SPI4] = {0};
uint8_t rxBuff_SPI4[LENGTH_MESSAGE_SPI4] = {0};





void disable_slave(){

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);
}

void enable_slave(){

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET);

}

void init_SPI(SPI_HandleTypeDef *hspi){

SPI.byte = 0b11111111;
disable_slave();

txBuff_SPI4[0] = 0x61;  // WREG command
txBuff_SPI4[1] = 0x00;  // address + count
txBuff_SPI4[2] = 0x00;  // padding
txBuff_SPI4[3] = 0x01;  // MODE high byte
txBuff_SPI4[4] = 0x10;  // MODE low byte
txBuff_SPI4[5] = 0x00;  // padding

SPI.flags.SPI4_Done = 0;
enable_slave();
HAL_SPI_TransmitReceive_DMA(hspi, txBuff_SPI4, rxBuff_SPI4, LENGTH_MESSAGE_SPI4);

}

void spi_read(uint8_t *command, SPI_HandleTypeDef *hspi){

	switch ((uintptr_t)hspi->Instance) {

	case (uintptr_t)SPI1:

	break;

	case (uintptr_t)SPI2:

	break;

	case (uintptr_t)SPI3:

	break;

	case (uintptr_t)SPI5:

			txBuff_SPI4[0] = 0;  // WREG command
			txBuff_SPI4[1] = 0;  // address + count
			txBuff_SPI4[2] = 0;  // padding
			txBuff_SPI4[3] = 0;  // MODE high byte
			txBuff_SPI4[4] = 0;  // MODE low byte
			txBuff_SPI4[5] = 0;  // padding

		if (SPI.flags.SPI4_Done == 1)
		    {
				SPI.flags.SPI4_Done = 0;

		        enable_slave();
		        HAL_SPI_TransmitReceive_DMA(hspi, txBuff_SPI4, rxBuff_SPI4, LENGTH_MESSAGE_SPI4); //HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer, rxBuffer, dataSize);
		    }

	break;

	}

}


void spi_write(uint8_t command)
{


}



void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{


    if (hspi->Instance == SPI5)
    {
    	SPI.flags.SPI4_Done = 1;
        disable_slave();
		
    }


}

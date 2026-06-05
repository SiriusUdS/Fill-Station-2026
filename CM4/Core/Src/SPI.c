#include "spi.h"
#include "main.h"  
#include "stdio.h"
#include "stm32h7xx_hal_gpio.h"
#include "string.h"

//Variables
union	SPI_Flags SPI;

__attribute__((section(".SRAM4"))) uint8_t txBuff_SPI4[LENGTH_MESSAGE_SPI4];
__attribute__((section(".SRAM4"))) uint8_t rxBuff_SPI4[LENGTH_MESSAGE_SPI4];

__attribute__((section(".SRAM4"))) uint8_t txBuff_SPI6[LENGTH_MESSAGE_SPI6];
__attribute__((section(".SRAM4"))) uint8_t rxBuff_SPI6[LENGTH_MESSAGE_SPI6];

long unsigned int val[10];

GPIO_TypeDef *SPI4_CS_GPIO_TYPE = GPIOE;
uint16_t SPI4_CS_GPIO_NUMBER[4] = {GPIO_PIN_15,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4};

GPIO_TypeDef *SPI6_CS_GPIO_TYPE = GPIOG;
uint16_t SPI6_CS_GPIO_NUMBER = GPIO_PIN_11;


void disable_slave(GPIO_TypeDef *GPIO, uint16_t GPIO_PIN){
	HAL_GPIO_WritePin(GPIO, GPIO_PIN, GPIO_PIN_SET);
}

void enable_slave(GPIO_TypeDef *GPIO, uint16_t GPIO_PIN){

	HAL_GPIO_WritePin(GPIO, GPIO_PIN, GPIO_PIN_RESET);

}

void init_SPI(){

//Flags init
SPI.byte = 0b11110011;

//SPI4 Init
memset(txBuff_SPI6, 0, LENGTH_MESSAGE_SPI4);
memset(rxBuff_SPI6, 0, LENGTH_MESSAGE_SPI4);

//SP6 Init
memset(txBuff_SPI6, 0, LENGTH_MESSAGE_SPI6);
memset(rxBuff_SPI6, 0, LENGTH_MESSAGE_SPI6);

}

uint8_t test[6];

void spi_write_read(uint8_t command[6], SPI_HandleTypeDef *hspi){

	switch ((uintptr_t)hspi->Instance) {

	case (uintptr_t)SPI4:

        		if (SPI.flags.SPI4_Done == 1)
		    {

				SPI.flags.SPI4_Done = 0;
		        enable_slave(SPI4_CS_GPIO_TYPE,SPI4_CS_GPIO_NUMBER[SPI.flags.SPI4_Counter]);
		        HAL_SPI_TransmitReceive_DMA(hspi, txBuff_SPI4, rxBuff_SPI4, LENGTH_MESSAGE_SPI4); //HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer, rxBuffer, dataSize);
		    }

	break;

	case (uintptr_t)SPI6:

		txBuff_SPI6[0] = command[0];
		txBuff_SPI6[1] = command[1];
		txBuff_SPI6[2] = command[2];
		txBuff_SPI6[3] = command[3];
		txBuff_SPI6[4] = command[4];
		txBuff_SPI6[5] = command[5];

		if (SPI.flags.SPI6_Done == 1 && HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5) == 0) //&& HAL_GPIO_ReadPin == 1 pour data ready
		    {
				/*
				char buf[128];
				 sprintf(buf, "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
				            val[1], val[2], val[3], val[4],
				            val[5], val[6], val[7], val[8]);

				CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
				HAL_Delay(10);  // or use a timer-based approach
				*/

				SPI.flags.SPI6_Done = 0;
		        enable_slave(SPI6_CS_GPIO_TYPE,SPI6_CS_GPIO_NUMBER);
		        HAL_SPI_TransmitReceive_DMA(hspi, txBuff_SPI6, rxBuff_SPI6, LENGTH_MESSAGE_SPI6); //HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer, rxBuffer, dataSize);
		    }

	break;

	}

}




uint64_t timestamp_SPI4 = 0;
uint64_t timestamp_SPI6 = 0;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI4){

        SPI.flags.SPI4_Done = 1;
        disable_slave(SPI4_CS_GPIO_TYPE,SPI4_CS_GPIO_NUMBER[SPI.flags.SPI4_Counter]);
        SPI.flags.SPI4_Counter = (SPI.flags.SPI4_Counter + 1)%4;

		timestamp_SPI4 = HAL_GetTick();

    }

    if (hspi->Instance == SPI6)
    {
    	SPI.flags.SPI6_Done = 1;
        disable_slave(SPI6_CS_GPIO_TYPE,SPI6_CS_GPIO_NUMBER);

		//Need to add packet
		timestamp_SPI6 = HAL_GetTick();
		
        }
     


}

uint32_t last_spi_error = 0;
uint32_t spi_sr_register = 0;

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if(hspi->Instance == SPI6)
    {
        last_spi_error = HAL_SPI_GetError(hspi);
        spi_sr_register = SPI6->SR; // capture status register raw
        
        // Re-arm after error
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
        SPI.flags.SPI4_Done = 1; // allow next transaction to proceed
    }
}
#include "SPI.h"
#include "main.h"  // For CS pin definitions
#include "stdio.h"
#include "string.h"


union	SPI_Flags SPI;

//__attribute__((section(".SRAM4"))) 
uint8_t txBuff_SPI4[LENGTH_MESSAGE_SPI4] = {0};
//__attribute__((section(".SRAM4"))) 
uint8_t rxBuff_SPI4[LENGTH_MESSAGE_SPI4] = {0};
long unsigned int val[10];




void disable_slave(){
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET);
}

void enable_slave(){

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET);

}

void init_SPI(SPI_HandleTypeDef *hspi){

SPI.byte = 0b11111111;
memset(txBuff_SPI4, 0, LENGTH_MESSAGE_SPI4);
memset(rxBuff_SPI4, 0, LENGTH_MESSAGE_SPI4);

}

uint8_t test[6];

void spi_write_read(uint8_t command[6], SPI_HandleTypeDef *hspi){

	switch ((uintptr_t)hspi->Instance) {

	case (uintptr_t)SPI1:

	break;

	case (uintptr_t)SPI2:

	break;

	case (uintptr_t)SPI3:

	break;

	case (uintptr_t)SPI6:

		txBuff_SPI4[0] = command[0];
		txBuff_SPI4[1] = command[1];
		txBuff_SPI4[2] = command[2];
		txBuff_SPI4[3] = command[3];
		txBuff_SPI4[4] = command[4];
		txBuff_SPI4[5] = command[5];

		if (SPI.flags.SPI4_Done == 1)
		    {
				/*
				char buf[128];
				 sprintf(buf, "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
				            val[1], val[2], val[3], val[4],
				            val[5], val[6], val[7], val[8]);

				CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
				HAL_Delay(10);  // or use a timer-based approach
				*/

				SPI.flags.SPI4_Done = 0;
		        enable_slave();
				HAL_SPI_TransmitReceive_IT(hspi,txBuff_SPI4, rxBuff_SPI4, LENGTH_MESSAGE_SPI4);
		        //HAL_SPI_TransmitReceive_DMA(hspi, txBuff_SPI4, rxBuff_SPI4, LENGTH_MESSAGE_SPI4); //HAL_SPI_TransmitReceive_DMA(&hspi1, txBuffer, rxBuffer, dataSize);
		    }

	break;

	}

}






void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{


    if (hspi->Instance == SPI6)
    {
    	SPI.flags.SPI4_Done = 1;
        disable_slave();



            for (int i = 0; i < 10; i++) {
                val[i] = ((uint32_t)rxBuff_SPI4[i*3]     << 16) |
                         ((uint32_t)rxBuff_SPI4[i*3 + 1] << 8)  |
                          (uint32_t)rxBuff_SPI4[i*3 + 2];


        }
    }


}

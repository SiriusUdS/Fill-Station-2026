#include "spi.h"
#include "main.h"  
#include "stdio.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include "string.h"

struct SPI4_HANDLE  spi4_handle  = {0};
struct SPI6_HANDLE  spi6_handle  = {0};
struct SPI4_CONFIG  spi4_config  = {0};
struct SPI6_CONFIG  spi6_config  = {0};
union  SPI_Flags    spi_flags    = { .byte = 0xFC };


uint8_t counter_spi4 = 0;
uint8_t counter_spi6 = 0;

void disable_slave(GPIO_TypeDef *GPIO, uint16_t GPIO_PIN){
    HAL_GPIO_WritePin(GPIO, GPIO_PIN, GPIO_PIN_SET);
}

void enable_slave(GPIO_TypeDef *GPIO, uint16_t GPIO_PIN){
    HAL_GPIO_WritePin(GPIO, GPIO_PIN, GPIO_PIN_RESET);
}

void init_SPI4(){
    // SPI4 Init
    memset(spi4_handle.rxBuff_SPI4, 0, spi4_handle.length_spi4);
    memset(spi4_handle.txBuff_SPI4, 0, spi4_handle.length_spi4);
}

void init_SPI6(){
    // SPI6 Init
    memset(spi6_handle.rxBuff_SPI6, 0, spi6_handle.length_spi6);
    memset(spi6_handle.txBuff_SPI6, 0, spi6_handle.length_spi6);
}

void spi_write_read(SPI_HandleTypeDef *hspi){

    switch ((uintptr_t)hspi->Instance) {

    case (uintptr_t)SPI4:
        if (spi_flags.flags.SPI4_Done == 1)
        {
            spi_flags.flags.SPI4_Done = 0;
            //enable_slave(spi4_config.spi4_cs_gpio_type[counter_spi4], spi4_config.spi4_cs_gpio_number[counter_spi4]);
            HAL_SPI_TransmitReceive_DMA(hspi, spi4_handle.txBuff_SPI4, spi4_handle.rxBuff_SPI4, spi4_handle.length_spi4);
        }
    break;

    case (uintptr_t)SPI6:
        if (spi_flags.flags.SPI6_Done == 1)
        {
            spi_flags.flags.SPI6_Done = 0;
            enable_slave(spi6_config.spi6_cs_gpio_type[0], spi6_config.spi6_cs_gpio_number[0]);
            HAL_SPI_TransmitReceive_IT(hspi, spi6_handle.txBuff_SPI6, spi6_handle.rxBuff_SPI6, spi6_handle.length_spi6);
        }
    break;

    }
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI4)
    {
        spi_flags.flags.SPI4_Done = 1;
        //disable_slave(spi4_config.spi4_cs_gpio_type[0], spi4_config.spi4_cs_gpio_number[0]);
		counter_spi4 = (counter_spi4+1)%(spi4_config.spi4_cs_num);
        spi4_handle.last_timestamp = HAL_GetTick();
    }

    if (hspi->Instance == SPI6)
    {
        spi_flags.flags.SPI6_Done = 1;
        disable_slave(spi6_config.spi6_cs_gpio_type[0], spi6_config.spi6_cs_gpio_number[0]);
		counter_spi6 = (counter_spi6+1)%(spi6_config.spi6_cs_num);
        spi6_handle.last_timestamp = HAL_GetTick();
    }
}

uint32_t last_spi_error = 0;
uint32_t spi_sr_register = 0;

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI6)
    {
        last_spi_error = HAL_SPI_GetError(hspi);
        spi_sr_register = SPI6->SR;

        __HAL_SPI_CLEAR_OVRFLAG(hspi);
        spi_flags.flags.SPI6_Done = 1; // re-arm
    }
}
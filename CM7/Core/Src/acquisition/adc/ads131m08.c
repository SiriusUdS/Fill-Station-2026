#include "acquisition/adc/ads131m08.h"
#include "communication/spi/spi.h"
#include <stdint.h>
#include <string.h>



typedef union {
	uint16_t COMMAND;
	struct{
		uint16_t REGISTER_NUMBER : 7;
		uint16_t ADDRESS : 6;
		uint16_t CMD : 3;
	}field;
	struct{
			uint8_t LSB;
			uint8_t MSB;
	}bytes;
}RW_REG;

RW_REG write_command;



uint8_t txBuff_SPI[message_length];
uint8_t rxBuff_SPI[message_length];



void ADS131M08_write_register(uint8_t address, uint16_t value, SPI_HandleTypeDef *hspi){ //Writing directly into a singular register. Value is the all part of a register union

	write_command.field.REGISTER_NUMBER = 0;
	write_command.field.ADDRESS = address;
	write_command.field.CMD = WREG_CMD;

	uint8_t valueMSB = (value & MSB_uint16_mask) >> 8;
	uint8_t valueLSB = (value & LSB_uint16_mask);

	uint8_t msg[] = {write_command.bytes.MSB, write_command.bytes.LSB, 0, valueMSB, valueLSB, 0};
  memcpy(spi6_handle.txBuff_SPI6, msg, sizeof(msg));

  spi_write_read(hspi);

}



void ADS131M08_get_data(SPI_HandleTypeDef *hspi){ //Sending the dummy command NULL (0b00000000) to read back the adc channels value
  if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == 0){

    spi_write_read(hspi);

  }
}

void ADS131M08_spi_init(GPIO_TypeDef *csPort[], uint16_t csPin[],uint16_t cs_num){

  spi6_handle.txBuff_SPI6 = txBuff_SPI;
  spi6_handle.rxBuff_SPI6 = rxBuff_SPI;
  spi6_handle.length_spi6 = message_length;
  spi6_config.spi6_cs_gpio_type = csPort;
  spi6_config.spi6_cs_gpio_number = csPin;
  spi6_config.spi6_cs_num = cs_num;

  init_SPI6();
}

void ADS131M08_init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort[], uint16_t csPin[],uint16_t cs_num){

  ADS131M08_spi_init(csPort, csPin,cs_num);
  
  REG_CLOCK_t CLOCK_REG;
  CLOCK_REG.all = 0xFF0E;
  CLOCK_REG.bits.OSR = 0b100; //osr 2048 = 2kSPS
  ADS131M08_write_register(REG_ADDR_CLOCK, CLOCK_REG.all, hspi);

  REG_GAIN_t GAIN_REG;
  GAIN_REG.all = 0;
  GAIN_REG.bits.PGAGAIN0 = 	0b101; //101 = 32 gain
  GAIN_REG.bits.PGAGAIN1 = 	0b101;
  GAIN_REG.bits.PGAGAIN2 = 	0b101;
  GAIN_REG.bits.PGAGAIN3 = 	0b101;
  ADS131M08_write_register(REG_ADDR_GAIN, GAIN_REG.all, hspi);

  REG_GAIN2_t GAIN_REG2;
  GAIN_REG2.all = 0;
  GAIN_REG2.bits.PGAGAIN4 = 0b101;
  GAIN_REG2.bits.PGAGAIN5 = 0b101;
  GAIN_REG2.bits.PGAGAIN6 = 0b101;
  GAIN_REG2.bits.PGAGAIN7 = 0b101;
  ADS131M08_write_register(REG_ADDR_GAIN2, GAIN_REG2.all, hspi);

  memset(spi6_handle.txBuff_SPI6, 0, spi6_handle.length_spi6); //Command for get data

}




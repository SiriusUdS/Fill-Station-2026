#include "ADS131M04.h"
#include "SPI.h"



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



void ADS131M04_write_register(uint8_t address, uint16_t value, SPI_HandleTypeDef *hspi){ //Writing directly into a singular register. Value is the all part of a register union




	write_command.field.REGISTER_NUMBER = 0;
	write_command.field.ADDRESS = address;
	write_command.field.CMD = WREG_CMD;

	uint8_t valueMSB = (value & MSB_uint16_mask) >> 8;
	uint8_t valueLSB = (value & LSB_uint16_mask);

	uint8_t message[6] = {write_command.bytes.MSB, write_command.bytes.LSB,0, valueMSB, valueLSB,0}; //Zero padding since it'S on 6 byte



	spi_write_read(message,hspi);

}



void ADS131M04_get_data(SPI_HandleTypeDef *hspi){ //Sending the dummy command NULL (0b00000000) to read back the adc channels value

	uint8_t command[6] = {0}; //Zero padding since it'S on 6 byte

	spi_write_read(command,hspi);


}

void ADS131M04_init(){




}


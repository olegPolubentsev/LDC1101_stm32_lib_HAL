/*
 * LDC1101.c
 *
 *  Created on: Oct 11, 2021
 *      Author: olegp
 */
#include "LDC1101.h"

void LDC1101_Init(SPI_HandleTypeDef *hspi, defInitConfigStruct *init_struct)
{
	init_struct->spi_ldc = hspi;
	  {
		  defUserNameSpaceParametrs 	struct_parametrs;
		  struct_parametrs.time_convertion = 0x0002;
		  LDC1101_Set_Configuration(init_struct, struct_parametrs);
	  }
}

uint32_t LDC1101_Get_Value(defInitConfigStruct *init_struct)
{
	uint8_t buf[4];

	buf[0] = 0x38; //00000000xxxxxxxxxxxxxxxx
	buf[1] = 0xFF;
	buf[2] = 0xFF;
	buf[3] = 0xFF;

	buf[0] |= 1<<7; //режим чтения

	uint8_t buf_r[4];

	NSS_ON();
	HAL_SPI_TransmitReceive(init_struct->spi_ldc, buf, buf_r, 4, 1000);
	NSS_OFF();

	uint32_t value = 0;
	value |= buf_r[1]<<0;
	value |= buf_r[2]<<8;
	value |= buf_r[3]<<16;

	return value;
}
void LDC1101_Set_Configuration(defInitConfigStruct *init_struct, defUserNameSpaceParametrs config_struct)
{
	LDC1101_Set_Reg(init_struct,REG_START_CONFIG_M, Shutdown);
	HAL_Delay(1);
	LDC1101_Set_Reg(init_struct,REG_START_CONFIG_M, Sleep);


	LDC1101_Set_Reg(init_struct,REG_RP_SET, 		0b00100101);
	LDC1101_Set_Reg(init_struct,REG_TC1, 			0b01010000);
	LDC1101_Set_Reg(init_struct,REG_TC2, 			0b11011111);
	LDC1101_Set_Reg(init_struct,REG_DIG_CONFIG,		0b11000011);
	LDC1101_Set_Reg(init_struct,REG_D_CONFIG,		0b00000001);

	uint8_t LSB = config_struct.time_convertion>>8;
	uint8_t MSB = config_struct.time_convertion>>0;
	LDC1101_Set_Reg(init_struct, REG_LHR_RCOUNT_LSB, LSB);
	LDC1101_Set_Reg(init_struct, REG_LHR_RCOUNT_MSB, MSB);
}
void LDC1101_Set_Reg(defInitConfigStruct *init_struct, uint8_t read_REG_addr, uint8_t data_reg)
{
	uint8_t buf[2];
	buf[0] = read_REG_addr;
	buf[1] = data_reg;
	NSS_ON();
	HAL_SPI_Transmit(init_struct->spi_ldc, buf, 2, 1000);
	NSS_OFF();
}
void LDC1101_Eneble_Convertion(defInitConfigStruct *init_struct)
{
	uint8_t buf[2];
	buf[0] = 0x0B;
	buf[1] = 0x00;
	NSS_ON();
	HAL_SPI_Transmit(init_struct->spi_ldc, buf, 2, 1000);
	NSS_OFF();
}
void LDC1101_Disable_Convertion(defInitConfigStruct *init_struct)
{
	uint8_t buf[2];
	buf[0] = 0x0B;
	buf[1] = 0x01;
	NSS_ON();
	HAL_SPI_Transmit(init_struct->spi_ldc, buf, 2, 1000);
	NSS_OFF();
}
void NSS_ON(void)
{
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
}
void NSS_OFF(void)
{
	HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
}
EnumReadBit LDC1101_Read_Bit(defInitConfigStruct *init_struct, uint8_t read_REG_addr, uint8_t bit_num)
{
	uint8_t buf[2];

	buf[0] = read_REG_addr;
	buf[1] = 0xFF;

	buf[0] |= 1<<7; //режим чтения

	uint8_t buf_r[2];

	NSS_ON();
	HAL_SPI_TransmitReceive(init_struct->spi_ldc, buf, buf_r, 2, 1000);
	NSS_OFF();

	uint8_t value = 0;
	EnumReadBit rez;
	value |= buf_r[1]<<6;

	if (value!=0) rez = Bit_high;
	else rez = Bit_low;
	return rez;
}


uint32_t LDC1101_Get_Value_RP(defInitConfigStruct *init_struct)
{
	uint8_t buf[3];

	buf[0] = REG_RP_DATA_LSB; //00000000xxxxxxxxxxxxxxxx
	buf[1] = 0xFF;
	buf[2] = 0xFF;

	buf[0] |= 1<<7; //режим чтения

	uint8_t buf_r[3];

	NSS_ON();
	HAL_SPI_TransmitReceive(init_struct->spi_ldc, buf, buf_r, 3, 1000);
	NSS_OFF();

	uint32_t value = 0;
	value |= buf_r[1]<<8;
	value |= buf_r[2]<<0;

	return value;
}

uint32_t LDC1101_Get_Value_L(defInitConfigStruct *init_struct)
{
	uint8_t buf[3];

	buf[0] = REG_L_DATA_LSB; //00000000xxxxxxxxxxxxxxxx
	buf[1] = 0xFF;
	buf[2] = 0xFF;

	buf[0] |= 1<<7; //режим чтения

	uint8_t buf_r[3];

	NSS_ON();
	HAL_SPI_TransmitReceive(init_struct->spi_ldc, buf, buf_r, 3, 1000);
	NSS_OFF();

	uint32_t value = 0;
	value |= buf_r[1]<<8;
	value |= buf_r[2]<<0;

	return value;
}

/*
 * LDC1101.h
 *
 *  Created on: Oct 11, 2021
 *      Author: olegp
 */

#ifndef LDC1101_H_
#define LDC1101_H_

#include "main.h"

typedef enum EnumREG_START_CONFIG{
	Active,
	Sleep,
	Shutdown
}EnumREG_START_CONFIG;
typedef enum EnumReadBit{
	Bit_high,
	Bit_low
}EnumReadBit;

typedef struct
{
	SPI_HandleTypeDef *spi_ldc;
	uint8_t	REG_BP_SET;
	uint8_t	REG_TC1;
	uint8_t	REG_TC2;
	uint8_t	REG_DIG_CONFIG;
	uint8_t	REG_ALT_CONFIG;
	uint8_t	REG_RP_THRESH_H_LSB;
	uint8_t	REG_RP_THRESH_H_MSB;
	uint8_t	REG_RP_THRESH_L_LSB;
	uint8_t	REG_RP_THRESH_L_MSB;
	uint8_t	REG_INTB_MODE;
	uint8_t	REG_START_CONFIG;
	uint8_t	REG_D_CONF;
	uint8_t	REG_L_THRESH_HI_LSB;
	uint8_t	REG_L_THRESH_HI_MSB;
	uint8_t	REG_L_THRESH_LO_LSB;
	uint8_t	REG_L_THRESH_LO_MSB;
	uint8_t	REG_STATUS;
} defInitConfigStruct;

typedef struct
{
	EnumREG_START_CONFIG START_CONFIG;
	uint16_t time_convertion;

} defUserNameSpaceParametrs;


#define	REG_RP_SET				0x01
#define	REG_TC1					0x02
#define	REG_TC2					0x03
#define	REG_DIG_CONFIG			0x04
#define	REG_ALT_CONFIG			0x05
#define	REG_RP_THRESH_H_LSB		0x06
#define	REG_RP_THRESH_H_MSB		0x07
#define	REG_RP_THRESH_L_LSB		0x08
#define	REG_RP_THRESH_L_MSB		0x09
#define	REG_INTB_MODE			0x0A
#define	REG_START_CONFIG_M		0x0B
#define	REG_D_CONFIG			0x0C
#define	REG_L_THRESH_HI_LSB		0x16
#define	REG_L_THRESH_HI_MSB		0x17
#define	REG_L_THRESH_LO_LSB		0x18
#define	REG_L_THRESH_LO_MSB		0x19
#define	REG_STATUS				0x20
#define	REG_RP_DATA_LSB			0x21
#define	REG_RP_DATA_MSB			0x22
#define	REG_L_DATA_LSB			0x23
#define	REG_L_DATA_MSB			0x24
#define	REG_LHR_RCOUNT_LSB		0x30
#define	REG_LHR_RCOUNT_MSB		0x31
#define	REG_LHR_OFFSET_LSB		0x32
#define	REG_LHR_OFFSET_MSB		0x33
#define	REG_LHR_CONFIG			0x34
#define	REG_LHR_DATA_LSB		0x38
#define	REG_LHR_DATA_MID		0x39
#define	REG_LHR_DATA_MSB		0x3A
#define	REG_LHR_STATUS			0x3B
#define	REG_RID					0x3E
#define	REG_CHIP_ID				0x3F



void LDC1101_Init(SPI_HandleTypeDef *hspi, defInitConfigStruct* init_struct);
uint32_t LDC1101_Get_Value(defInitConfigStruct *init_struct);
void LDC1101_Set_Configuration(defInitConfigStruct *init_struct, defUserNameSpaceParametrs config_struct);
void LDC1101_Eneble_Convertion(defInitConfigStruct *init_struct);
void LDC1101_Disable_Convertion(defInitConfigStruct *init_struct);
void NSS_ON(void);
void NSS_OFF(void);
EnumReadBit LDC1101_Read_Bit(defInitConfigStruct *init_struct, uint8_t read_REG_addr, uint8_t bit_num);
void LDC1101_Set_Reg(defInitConfigStruct *init_struct, uint8_t read_REG_addr, uint8_t data_reg);
uint32_t LDC1101_Get_Value_RP(defInitConfigStruct *init_struct);
uint32_t LDC1101_Get_Value_L(defInitConfigStruct *init_struct);

#endif /* LDC1101_H_ */

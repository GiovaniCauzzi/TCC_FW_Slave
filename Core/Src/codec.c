/*
 * codec.c
 *
 *  Created on: Oct 9, 2023
 *      Author: giova
 */

#include "codec.h"

HAL_StatusTypeDef codec_set_reg(I2C_HandleTypeDef *i2c_instance, unsigned char MemAddress, uint8_t *pdata, unsigned int size)
{
	uint64_t delayTosco = 0;

	while(delayTosco++ < 500000)
	{

	}

	return HAL_I2C_Mem_Write(i2c_instance, dCODEC_ADDR, MemAddress, I2C_MEMADD_SIZE_8BIT, pdata, size, dCODEC_HAL_MAX_DELAY);
}

HAL_StatusTypeDef codec_set_reg_teste(I2C_HandleTypeDef *i2c_instance, unsigned char MemAddress, uint16_t u16data)
{
	uint8_t data2send[2];
	uint16_t dummy = 0;

	dummy = 0xFF00;
	data2send[0] = (uint8_t)((u16data&dummy) >> 8);
	dummy = 0x00FF;
	data2send[1] = (uint8_t)(u16data&dummy);
	return HAL_I2C_Mem_Write(i2c_instance, dCODEC_ADDR, MemAddress, I2C_MEMADD_SIZE_8BIT, &data2send[0], 2, dCODEC_HAL_MAX_DELAY);
}

/*
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
uint16_t DevAddress,
uint16_t MemAddress,
uint16_t MemAddSize,     
uint8_t *pData,
uint16_t Size,
uint32_t Timeout)*/

HAL_StatusTypeDef codec_read_reg(I2C_HandleTypeDef *i2c_instance, uint16_t MemAddress, uint8_t * pdata, uint16_t size)
{
	return HAL_I2C_Mem_Read(i2c_instance, dCODEC_ADDR, MemAddress, I2C_MEMADD_SIZE_8BIT, pdata, 2, dCODEC_HAL_MAX_DELAY);
}

/*
HAL_StatusTypeDef HAL_I2C_Mem_Read(
	I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
	uint16_t MemAddress,
	uint16_t MemAddSize,
	uint8_t *pData,
	uint16_t Size,
	uint32_t Timeout)
*/


void codec_init(I2C_HandleTypeDef *i2c_instance)
{
	uint8_t aux[2];
	uint8_t databuffer[2];
	HAL_StatusTypeDef result;

	/*
	aux[1] = ; // low byte data
	aux[0] = ; // high byte data
	result = codec_set_reg(i2c_instance, dREG_BLABLA, &aux[0], 2);
	*/

	aux[1] = 0x20;
	aux[0] = 0x00;
	result = codec_set_reg(i2c_instance, dREG_SW_RESET, &aux[0], 2);


	aux[1] = 0b00001000;//0xC8;
	aux[0] = 0b00001000;//0xC8;
	result = codec_set_reg(i2c_instance, dREG_HPOUT, &aux[0], 2);
	result = codec_read_reg(i2c_instance,dREG_HPOUT,databuffer, 2);
	// result = HAL_I2C_Mem_Write(i2c_instance, dCODEC_ADDR, dREG_SW_RESET, I2C_MEMADD_SIZE_8BIT, &aux[0], 2, dCODEC_HAL_MAX_DELAY);
	// result = HAL_I2C_Mem_Write(i2c_instance, dCODEC_ADDR, dREG_HPOUT, I2C_MEMADD_SIZE_8BIT, &aux[0], 2, dCODEC_HAL_MAX_DELAY);

	aux[1] = 0x00;       // low byte data
	aux[0] = 0b01000000;//0b01000000; // high byte data
	result = codec_set_reg(i2c_instance, dREG_HPOMIX_CTRL, &aux[0], 2);

	aux[1] = 0b00001000;//00001000;
	aux[0] = 0b00001000;//00001000;
	result = codec_set_reg(i2c_instance, dREG_LINE_OUTPUT1, &aux[0], 2);

	aux[1] = 0x00;
	aux[0] = 0x00;
	result = codec_set_reg(i2c_instance, dREG_LINE_OUTPUT2, &aux[0], 2);

	aux[1] = 0b00000000;
	aux[0] = 0b00000000;
	result = codec_set_reg(i2c_instance, dREG_MIC_INPUT_MODE_GAIN, &aux[0], 2);

	aux[1] = 0b00001000;
	aux[0] = 0b00001000;
	result = codec_set_reg(i2c_instance, dREG_LINE_INPUT_MODE, &aux[0], 2);

	aux[1] = 0b00100000;
	aux[0] = 0b00111000;
	result = codec_set_reg(i2c_instance, dREG_ADC_MIX_CONTROL, &aux[0], 2);
	result = codec_read_reg(i2c_instance,dREG_ADC_MIX_CONTROL,databuffer, 2);

	aux[1] = 0b00000000;
	aux[0] = 0b00000000;
	result = codec_set_reg(i2c_instance, dREG_RECMIXL_CTRL1, &aux[0], 2);

	aux[1] = 0b01001111; 
	aux[0] = 0b00000000;
	result = codec_set_reg(i2c_instance, dREG_RECMIXL_CTRL2, &aux[0], 2);

	aux[1] = 0b01001111; 
	aux[0] = 0b00000000;
	result = codec_set_reg(i2c_instance, dREG_RECMIXR_CTRL2, &aux[0], 2);

	aux[1] = 0x00;
	aux[0] = 0x00;
	result = codec_set_reg(i2c_instance, dREG_RECMIXR_CTRL1, &aux[0], 2);

	aux[1] = 0x00;
	aux[0] = 0x00;
	result = codec_set_reg(i2c_instance, dREG_OUTMIXL_CTRL1, &aux[0], 2);
	
	aux[1] = 0x00;
	aux[0] = 0x00;
	result = codec_set_reg(i2c_instance, dREG_OUTMIXL_CTRL2, &aux[0], 2);

	aux[1] = 0b01111000;
	aux[0] = 0b00000010;
	result = codec_set_reg(i2c_instance, dREG_OUTMIXL_CTRL3, &aux[0], 2);

	aux[1] = 0b01111000;
	aux[0] = 0b00000010;
	result = codec_set_reg(i2c_instance, dREG_OUTMIXR_CTRL3, &aux[0], 2);

	aux[1] = 0b00000000;//00000000;
	aux[0] = 0b11000000;//11000000;//00110000;
	result = codec_set_reg(i2c_instance, dREG_LOUTMIX_CTRL, &aux[0], 2);

	aux[1] = 0b00111111; // um certo ganho
	aux[0] = 0b00111111; // um certo ganho
	result = codec_set_reg(i2c_instance, dREG_ADC_DIG_VOL_CTRL, &aux[0], 2);

	aux[1] = 0x00;
	aux[0] = 0x00;
	result = codec_set_reg(i2c_instance, dREG_ADC_DIG_BOOST_CTRL, &aux[0], 2);

	aux[1] = 0b10000000; 
	aux[0] = 0b10000000; 
	result = codec_set_reg(i2c_instance, dREG_ADC2DAC_DIG_MIX_CTRL, &aux[0], 2);

	aux[1] = 0xAF;
	aux[0] = 0xAF;
	result = codec_set_reg(i2c_instance, dREG_DACL1_R1_DIG_VOL, &aux[0], 2);

	aux[1] = 0b00010010;
	aux[0] = 0b00010010;
	result = codec_set_reg(i2c_instance, dREG_DAC_DIG_MIX_CTRL, &aux[0], 2);
	

	aux[1] = 0b00010001;
	aux[0] = 0b00000000;
	result = codec_set_reg(i2c_instance, dREG_GENERAL_CTRL1, &aux[0], 2);

	aux[1] = 0b00000110;
	aux[0] = 0b10011000;
	result = codec_set_reg(i2c_instance, dREG_POWER_MANAGE_CTRL1, &aux[0], 2);

	aux[1] = 0b00000000;
	aux[0] = 0b10001000;
	result = codec_set_reg(i2c_instance, dREG_POWER_MANAGE_CTRL2, &aux[0], 2);

	aux[1] = 0b11111001;
	aux[0] = 0b11111000;
	result = codec_set_reg(i2c_instance, dREG_POWER_MANAGE_CTRL3, &aux[0], 2);

	aux[1] = 0b00110110;//00110110;
	aux[0] = 0b11001010;
	result = codec_set_reg(i2c_instance, dREG_POWER_MANAGE_CTRL4, &aux[0], 2);

	aux[1] = 0b00000000;
	aux[0] = 0b11001100;
	result = codec_set_reg(i2c_instance, dREG_POWER_MANAGE_CTRL5, &aux[0], 2);
	
	aux[1] = 0x00;
	aux[0] = 0b00111111;
	result = codec_set_reg(i2c_instance, dREG_POWER_MANAGE_CTRL6, &aux[0], 2);

	aux[1] = 0b00000000;//00000000;//00000110;
	aux[0] = 0b10000000;//10011000; // config device as slave
	result = codec_set_reg(i2c_instance, dREG_DIG_INTERFACE_CONTROL, &aux[0], 2);

	aux[1] = 0b00000101;//00001010;//00000101;
	aux[0] = 0b00000001;//00010000;
	result = codec_set_reg(i2c_instance, dREG_ADC_DAC_CLK_CTRL1, &aux[0], 2);
	result = codec_read_reg(i2c_instance,dREG_ADC_DAC_CLK_CTRL1,databuffer, 2);

	aux[1] = 0x00;
	aux[0] = 00001100;
	result = codec_set_reg(i2c_instance, dREG_ADC_DAC_CLK_CTRL2, &aux[0], 2);

	aux[1] = 0b00000000;
	aux[0] = 0b00000000;
	result = codec_set_reg(i2c_instance, dREG_GLOBAL_CLK_CTRL, &aux[0], 2);

	aux[1] = 0b00011101;
	aux[0] = 0b10000000;
	result = codec_set_reg(i2c_instance, dREG_HP_AMP_CTRL1, &aux[0], 2);

	aux[1] = 0x20;
	aux[0] = 0b00100010;
	result = codec_set_reg(i2c_instance, dREG_WIND_FILTER_CTRL2, &aux[0], 2);

	aux[1] = 0b00100001;
	aux[0] = 0b11100001;
	result = codec_set_reg(i2c_instance, dREG_DRC_AGC_CTRL1, &aux[0], 2);

	aux[1] = 0b00000000;
	aux[0] = 0b00100000;
	result = codec_set_reg(i2c_instance, dREG_DRC_AGC_CTRL2, &aux[0], 2);
}

/*
void codec_init_teste(I2C_HandleTypeDef *i2c_instance)
{
	uint8_t aux[2];
	HAL_StatusTypeDef result;

	aux[1] = 0x00;
	aux[0] = 0x00;
	result = codec_set_reg(i2c_instance, dREG_SW_RESET, &aux[0], 2);

	codec_set_reg_teste(i2c_instance, 0xFA, 0x11);        // 
    codec_set_reg_teste(i2c_instance, 0x61, 0x9806);      // i2c power
    codec_set_reg_teste(i2c_instance, 0x62, 0x8800);
    codec_set_reg_teste(i2c_instance, 0x63, 0xf8ff);
    codec_set_reg_teste(i2c_instance, 0x64, 0xfff0);
    codec_set_reg_teste(i2c_instance, 0x65, 0xffff);
    codec_set_reg_teste(i2c_instance, 0x66, 0x3fc0);

	aux[1] = 0b00000000;
	aux[0] = 0b10011000; // config device as slave
	result = codec_set_reg(i2c_instance, dREG_DIG_INTERFACE_CONTROL, &aux[0], 2);

	aux[1] = 0b00000000;//00000101;
	aux[0] = 0b00000001;//00010000;
	result = codec_set_reg(i2c_instance, dREG_ADC_DAC_CLK_CTRL1, &aux[0], 2);

	aux[1] = 0b00000000;
	aux[0] = 0b00000000;
	result = codec_set_reg(i2c_instance, dREG_GLOBAL_CLK_CTRL, &aux[0], 2);

	codec_set_reg_teste(i2c_instance, 0x6a, 0x3d);
    codec_set_reg_teste(i2c_instance, 0x6c, 0x3700);
    codec_set_reg_teste(i2c_instance, 0x0D, 0x8300);
    codec_set_reg_teste(i2c_instance, 0x0f, 0x0606);
    codec_set_reg_teste(i2c_instance, 0x1c, 0x7F7F);
    codec_set_reg_teste(i2c_instance, 0x29, 0x8080);
    codec_set_reg_teste(i2c_instance, 0x27, 0x3820);
    codec_set_reg_teste(i2c_instance, 0x74, 0x3820);
    codec_set_reg_teste(i2c_instance, 0xD3, 0x3320);
    codec_set_reg_teste(i2c_instance, 0x3c, 0x004f);
    codec_set_reg_teste(i2c_instance, 0x3e, 0x004f);
    codec_set_reg_teste(i2c_instance, 0x53, 0xc000);
    codec_set_reg_teste(i2c_instance, 0x03, 0x0606);
    codec_set_reg_teste(i2c_instance, 0x2a, 0x1212);

      //迴環 0x271
     
    codec_set_reg_teste(i2c_instance, 0x4F, 0x0278);
    codec_set_reg_teste(i2c_instance, 0x52, 0x0278);
    codec_set_reg_teste(i2c_instance, 0x02, 0x0000);
    codec_set_reg_teste(i2c_instance, 0x45, 0x2000);
    codec_set_reg_teste(i2c_instance, 0x8e, 0x0019);
    codec_set_reg_teste(i2c_instance, 0x8f, 0x2000);
    // PA
    codec_set_reg_teste(i2c_instance, 0x6a, 0x3d);
    codec_set_reg_teste(i2c_instance, 0x6c, 0x3700);
}

*/

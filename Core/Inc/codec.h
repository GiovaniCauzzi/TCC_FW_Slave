/*
 * codec.h
 *
 *  Created on: Oct 9, 2023
 *      Author: giova
 */

#ifndef INC_CODEC_H_
#define INC_CODEC_H_

#include "main.h"
#include <stdint.h>


#define dCODEC_HAL_MAX_DELAY 0xFFFFFFFFU

#define dCODEC_ADDR 0x36

#define dREG_SW_RESET               0x00
#define dREG_HPOUT                  0x02
#define dREG_LINE_OUTPUT1           0X03
#define dREG_LINE_OUTPUT2           0X05
#define dREG_MIC_INPUT_MODE_GAIN    0x0D
#define dREG_LINE_INPUT_MODE        0x0f
#define dREG_DACL1_R1_DIG_VOL       0x19
#define dREG_ADC_DIG_VOL_CTRL       0x1C
#define dREG_ADC_DIG_BOOST_CTRL     0x1e
#define dREG_ADC_MIX_CONTROL        0x27
#define dREG_ADC2DAC_DIG_MIX_CTRL   0x29
#define dREG_DAC_DIG_MIX_CTRL       0x2A
#define dREG_RECMIXL_CTRL1          0x3B
#define dREG_RECMIXL_CTRL2          0x3C
#define dREG_RECMIXR_CTRL1          0x3D
#define dREG_RECMIXR_CTRL2          0x3E
#define dREG_HPOMIX_CTRL            0x45
#define dREG_OUTMIXL_CTRL1          0x4D
#define dREG_OUTMIXL_CTRL2          0x4E
#define dREG_OUTMIXL_CTRL3          0x4F
#define dREG_OUTMIXR_CTRL3          0x52
#define dREG_LOUTMIX_CTRL           0x53
// #define dREG_
// #define dREG_

#define dREG_POWER_MANAGE_CTRL1     0x61
#define dREG_POWER_MANAGE_CTRL2     0x62
#define dREG_POWER_MANAGE_CTRL3     0x63
#define dREG_POWER_MANAGE_CTRL4     0x64
#define dREG_POWER_MANAGE_CTRL5     0x65
#define dREG_POWER_MANAGE_CTRL6     0x66
#define dREG_DIG_INTERFACE_CONTROL  0x70
#define dREG_ADC_DAC_CLK_CTRL1      0x73              // OVERSAMPLING
#define dREG_ADC_DAC_CLK_CTRL2      0x74
#define dREG_GLOBAL_CLK_CTRL        0x80
#define dREG_HP_AMP_CTRL1           0x8E
#define dREG_GENERAL_CTRL1          0xFA

#define dREG_DRC_AGC_CTRL1          0xb4
#define dREG_DRC_AGC_CTRL2          0xb5
#define dREG_WIND_FILTER_CTRL2      0xd3


void codec_init(I2C_HandleTypeDef *i2c_instance);
HAL_StatusTypeDef codec_set_reg(I2C_HandleTypeDef *i2c_instance, unsigned char reg, unsigned char *pdata, unsigned int size);
HAL_StatusTypeDef codec_read_reg(I2C_HandleTypeDef *i2c_instance, uint16_t MemAddress, uint8_t * pdata, uint16_t size);














#endif /* INC_CODEC_H_ */

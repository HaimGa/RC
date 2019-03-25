#include <stdint.h>
#include "Adafruit_ADS1015.h"
//#include "stm32f7xx_hal_i2c.h"

//extern I2C_HandleTypeDef hi2c1;

//uint16_t ADS1015_readADC_SingleEnded(int fd, uint8_t channel)
//uint16_t ADS1015_readADC_SingleEnded(uint8_t channel)
//{
//uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
//				  ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
//				  ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
//				  ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
//				  ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
//				  ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)
//
//	config |= ADS1015_REG_CONFIG_PGA_6_144V;
//// Set single-ended input channel
//	switch (channel)
//	{
//		case (0):
//		{
//			config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
//		} break;
//		case (1):
//		{
//			config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
//		} break;
//		case (2):
//		{
//			config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
//		} break;
//		case (3):
//		{
//			config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
//		} break;
//		default:
//		{
//		} break;
//	}
//        // Set 'start single-conversion' bit
//        config |= ADS1015_REG_CONFIG_OS_SINGLE;
//        HAL_I2C_Master_Transmit(&hi2c1, ADS1015_ADDRESS, &config, sizeof(config));
////        wiringPiI2CWriteReg16(fd, ADS1015_REG_POINTER_CONFIG, ntohs(config));
////        delay(ADS1015_CONVERSIONDELAY);
////        return ntohs(wiringPiI2CReadReg16(fd, ADS1015_REG_POINTER_CONVERT) >> 4);
//}

/*
 * BMP280_I2C.c
 *
 *  Created on: Mar 25, 2026
 *      Author: Lenovo
 */
#include "BMP280_I2C.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;
#define BMP280_I2C &hi2c1

#define SUPPORT_64BIT 1
//#define SUPPORT_32BIT 1

#define BMP280_ADDRESS 0xEC

extern float Temperature, Pressure;

uint8_t chip_ID;

uint8_t TrimParam[36];
int32_t tRaw,pRaw;

uint16_t dig_T1, dig_P1,dig_H1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

void TrimRead(void){
	uint8_t trimdata[24];
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, 0x88,1,trimdata,24,HAL_MAX_DELAY);

	dig_T1 = (trimdata[1]<<8)|trimdata[0];
	dig_T2 = (trimdata[3]<<8)|trimdata[2];
	dig_T3 = (trimdata[5]<<8)|trimdata[4];
	dig_P1 = (trimdata[7]<<8)|trimdata[5];
	dig_P2 = (trimdata[9]<<8)|trimdata[6];
	dig_P3 = (trimdata[11]<<8)|trimdata[10];
	dig_P4 = (trimdata[13]<<8)|trimdata[12];
	dig_P5 = (trimdata[15]<<8)|trimdata[14];
	dig_P6 = (trimdata[17]<<8)|trimdata[16];
	dig_P7 = (trimdata[19]<<8)|trimdata[18];
	dig_P8 = (trimdata[21]<<8)|trimdata[20];
	dig_P9 = (trimdata[23]<<8)|trimdata[22];
}

int BMP280_Config(uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter){
	TrimRead();

	uint8_t datatowrite = 0;
	uint8_t datacheck = 0;

	datatowrite = 0xB6;
	if(HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS, RESET_REG, 1, &datatowrite, 1, 1000)!= HAL_OK){
		return -1;
	}

	HAL_Delay(100);



	datatowrite = (t_sb << 5) | (filter << 2);

	if(HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS, CONFIG_REG, 1, &datatowrite, 1, 1000)!= HAL_OK){

		return -1;
	}
	HAL_Delay(100);
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, CONFIG_REG, 1, &datacheck, 1, 1000);

	if(datacheck != datatowrite){
		return -1;
	}




	datatowrite = (osrs_t << 5) | (osrs_p << 2) | mode;
	if(HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS, CTRL_MEAS_REG, 1, &datatowrite, 1, 1000)!= HAL_OK){

			return -1;
		}
	HAL_Delay(100);
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, CTRL_MEAS_REG, 1, &datacheck, 1, 1000);
	if(datacheck != datatowrite){
			return -1;
		}
	return 0;
}

int BMP_ReadRaw(void){
	uint8_t RawData[8];



	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, PRESS_MSB_REG, 1, RawData, 6, HAL_MAX_DELAY);


	pRaw = (RawData[0]<<12)|(RawData[1]<<4)|(RawData[2]>>4);
	tRaw = (RawData[3]<<12)|(RawData[4]<<4)|(RawData[5]>>4);

	return 0;
}


void BME280_WakeUP(void){
	uint8_t datatowrite = 0;

	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, CTRL_MEAS_REG,1,&datatowrite,1,1000);


	datatowrite = datatowrite | MODE_FORCED;
	HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS, CTRL_MEAS_REG,1,&datatowrite,1,1000);

	HAL_Delay(100);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value

typedef uint32_t BMP280_U32_t;
typedef int32_t BMP280_S32_t;
typedef int64_t BMP280_S64_t;

BMP280_S32_t t_fine;
BMP280_S32_t bmp280_compensate_T_int32()
{
BMP280_S32_t var1, var2, T;
BMP_ReadRaw();
var1 = (((double)tRaw)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
var2 = (((double)tRaw)/131072.0 - ((double)dig_T1)/8192.0) * (((double)tRaw)/131072.0 - ((double)dig_T1)/8192.0) * ((double)dig_T3);
t_fine = (BMP280_S32_t)(var1 + var2);
T = (var1 + var2) / 5120.0;
return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BMP280_U32_t bmp280_compensate_P_int64()
{
	BMP280_S64_t var1, var2, p;
	BMP_ReadRaw();
	var1 = ((BMP280_S64_t)t_fine) - 128000;
	var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
	var2 = var2 + ((var1*(BMP280_S64_t)dig_P5)<<17);
	var2 = var2 + (((BMP280_S64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3)>>8) + ((var1 * (BMP280_S64_t)dig_P2)<<12);
	var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig_P1)>>33;
	if (var1 == 0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576-pRaw;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((BMP280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);
	return (BMP280_U32_t)p;

}

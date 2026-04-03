/*
  ***************************************************************************************************************
  ***************************************************************************************************************
  ***************************************************************************************************************

  File:		  BME280_STM32.c
  Author:     ControllersTech.com
  Updated:    Dec 14, 2021

  ***************************************************************************************************************
  Copyright (C) 2017 ControllersTech.com

  This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
  of the GNU General Public License version 3 as published by the Free Software Foundation.
  This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
  or indirectly by this software, read more about this on the GNU General Public License.

  ***************************************************************************************************************
*/

#include "BME280_STM32.h"

extern I2C_HandleTypeDef hi2c1;
#define BME280_I2C &hi2c1
#define SUPPORT_64BIT 1
//#define SUPPORT_32BIT 1

#define BME280_ADDRESS 0xEC  // SDIO is grounded, the 7 bit address is 0x76 and 8 bit address = 0x76<<1 = 0xEC


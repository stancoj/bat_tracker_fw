/*
 * bmp280.c
 *
 *  Created on: 14. 11. 2020
 *      Author: Stancoj
 */

#include "bmp280.h"


bool InitBMP280(void)
{
	if(BMP280_ID_REG_VALUE == i2c_master_read(BMP280_ID_REG_ADDR, BMP280_I2C_ADDRESS, 1))
	{
		return true;
	}
	else
	{
		return false;
	}
}
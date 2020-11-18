/*
 * bmp280.c
 *
 *  Created on: 14. 11. 2020
 *      Author: Stancoj
 */

#include "bmp280.h"

static uint8_t bmp280_data_buffer[DATA_BUFFER_SIZE];


bool InitBMP280(void)
{
	/* Check if sensor has correct ID */
	if(i2c_master_read(BMP280_ID_REG_ADDR, BMP280_I2C_ADDRESS, bmp280_data_buffer, 1))
	{
		if(bmp280_data_buffer[0] != BMP280_ID_REG_VALUE)
		{
			return false;
		}
	}
	else
	{
		return false;
	}

	uint8_t reg_data = 0;

	/* Setup the sensor - configuration of CONFIG and CTRL_MEAS registers */
	/* CTRL_MEAS*/
	reg_data = BMP280_CTRL_MEAS_POWER_MODE_NORMAL | BMP280_CTRL_MEAS_OSRS_PRS2 | BMP280_CTRL_MEAS_OSRS_TEMP2;
	i2c_master_write(BMP280_CTRL_MEAS_REG_ADDR, BMP280_I2C_ADDRESS, &reg_data, 1);

	// Check if parameters were written successfully
	ReadCtrlMeas();
	if(bmp280_data_buffer[0] != reg_data)
	{
		return false;
	}
	/* CONFIG */
	reg_data = BMP280_CONFIG_3_WIRE_SPI_DISABLE | BMP280_CONFIG_IIR_FILTER_OFF | BMP280_CONFIG_STANDBY250;
	i2c_master_write(BMP280_CONFIG_REG_ADDR, BMP280_I2C_ADDRESS, &reg_data, 1);

	// Check if parameters were written successfully
	ReadConfig();
	if(bmp280_data_buffer[0] != reg_data)
	{
		return false;
	}

	return true;
}


void ReadPreasure(void)
{
	i2c_master_read(BMP280_PRESSURE_MSB_REG_ADDR, BMP280_I2C_ADDRESS, bmp280_data_buffer, 3);
}

void ReadTemperature(void)
{
	i2c_master_read(BMP280_TEMP_MSB_REG_ADDR, BMP280_I2C_ADDRESS, bmp280_data_buffer, 3);
}

void ReadConfig(void)
{
	i2c_master_read(BMP280_CONFIG_REG_ADDR, BMP280_I2C_ADDRESS, bmp280_data_buffer, 1);
}

void ReadCtrlMeas(void)
{
	i2c_master_read(BMP280_CTRL_MEAS_REG_ADDR, BMP280_I2C_ADDRESS, bmp280_data_buffer, 1);
}

/*
 * bmp280.h
 *
 *  Created on: 14. 11. 2020
 *      Author: Stancoj
 */

#ifndef BMP280_H_
#define BMP280_H_

#include "mcu.h"
#include <stdbool.h>

/* BMP sensor I2C address */
#define BMP280_I2C_ADDRESS						0xEEu
/* Revision registers */
#define BMP280_ID_REG_ADDR						0xD0u
#define BMP280_ID_REG_VALUE						0x58u
/* Data registers */
#define BMP280_TEMP_MSB_REG_ADDR				0xFAu
#define BMP280_PRESSURE_MSB_REG_ADDR			0xF7u
/* Control registers */
#define BMP280_CONFIG_REG_ADDR					0xF5u
#define BMP280_CTRL_MEAS_REG_ADDR				0xF4u
/* Status registers */
#define BMP280_STATUS_REG_ADDR					0xF3u
/* Reset registers */
#define BMP280_RESET_REG_ADDR					0xE0u
/* Calibration data */
#define BMP280_CALIBRATION_REG_ADDR				0x88u

/*------------------------------------------------------------------------------------*/

// Temperature, pressure over sampling
#define BMP280_CTRL_MEAS_OSRS_PRS1				(0x01 << 2)
#define BMP280_CTRL_MEAS_OSRS_PRS2				(0x02 << 2)
#define BMP280_CTRL_MEAS_OSRS_PRS4				(0x03 << 2)
#define BMP280_CTRL_MEAS_OSRS_PRS8				(0x04 << 2)
#define BMP280_CTRL_MEAS_OSRS_PRS16				(0x05 << 2)

#define BMP280_CTRL_MEAS_OSRS_TEMP1				(0x01 << 5)
#define BMP280_CTRL_MEAS_OSRS_TEMP2				(0x02 << 5)
#define BMP280_CTRL_MEAS_OSRS_TEMP4				(0x03 << 5)
#define BMP280_CTRL_MEAS_OSRS_TEMP8				(0x04 << 5)
#define BMP280_CTRL_MEAS_OSRS_TEMP16			(0x05 << 5)

// Sensors power mode
#define BMP280_CTRL_MEAS_POWER_MODE_SLEEP		0x00
#define BMP280_CTRL_MEAS_POWER_MODE_FORCE		0x01
#define BMP280_CTRL_MEAS_POWER_MODE_NORMAL		0x03

// Standby time between measurements [ms]
#define BMP280_CONFIG_STANDBY0_5				(0x00 << 5)
#define BMP280_CONFIG_STANDBY62_5				(0x01 << 5)
#define BMP280_CONFIG_STANDBY125				(0x02 << 5)
#define BMP280_CONFIG_STANDBY250				(0x03 << 5)
#define BMP280_CONFIG_STANDBY500				(0x04 << 5)
#define BMP280_CONFIG_STANDBY1000				(0x05 << 5)
#define BMP280_CONFIG_STANDBY2000				(0x06 << 5)
#define BMP280_CONFIG_STANDBY4000				(0x07 << 5)

// Time constant of IIR filter
#define BMP280_CONFIG_IIR_FILTER_OFF			(0x00 << 2)
#define BMP280_CONFIG_IIR_FILTER2				(0x02 << 2)
#define BMP280_CONFIG_IIR_FILTER4				(0x04 << 2)
#define BMP280_CONFIG_IIR_FILTER8				(0x08 << 2)
#define BMP280_CONFIG_IIR_FILTER16				(0x10 << 2)

// Enable 3 wire SPI
#define BMP280_CONFIG_3_WIRE_SPI_ENABLE			0x01
#define BMP280_CONFIG_3_WIRE_SPI_DISABLE		0x00


#define DATA_BUFFER_SIZE						10u

bool InitBMP280(void);
void ReadPreasure3B(void);
void ReadPreasure1B(void);

void ReadConfig(void);
void ReadCtrlMeas(void);

#endif /* BMP280_H_ */


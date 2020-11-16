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

#define BMP280_I2C_ADDRESS			0xEEu

#define BMP280_ID_REG_ADDR			0xD0u
#define BMP280_ID_REG_VALUE			0x58u


bool InitBMP280(void);

#endif /* BMP280_H_ */


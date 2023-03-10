/**
  ******************************************************************************
  * File Name          : I2C.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define I2C_REQUEST_WRITE                       0x00
#define I2C_REQUEST_READ                        0x01

void MX_I2C3_Init(void);

/* Reception handling */
void I2C3RxCompleteCallback(void);
void I2C3RxCallback(void);

/* Transfer functions */
uint8_t i2c_master_read(uint8_t register_addr, uint8_t slave_addr, uint8_t *data_buffer, uint8_t read_data_length);
uint8_t i2c_master_write(uint8_t register_addr, uint8_t slave_addr, uint8_t *data, uint8_t send_data_length);

int8_t i2c_bmp280_read(uint8_t slave_addr, uint8_t register_addr, uint8_t *data_buffer, uint16_t read_data_length);
int8_t i2c_bmp280_write(uint8_t slave_addr, uint8_t register_addr, uint8_t *data, uint16_t send_data_length);

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

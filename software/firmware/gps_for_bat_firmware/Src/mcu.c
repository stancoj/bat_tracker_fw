/*
 * mcu.c
 *
 *  Created on: 22. 11. 2018
 *      Author: Stancoj
 */

#include "mcu.h"

extern volatile uint8_t ubx_received_flag;
extern volatile uint8_t nmea_received_flag;

volatile uint8_t baud_rate_change_flag = 0;


void mcu_init(void)
{
	  flash_init();
	  MX_GPIO_Init();
	  MX_DMA_Init();
	  MX_I2C3_Init();
	  MX_USART1_UART_Init();
	  MX_USART2_UART_Init();
	  RTC_Init(0, LSI_ENABLE); // time = 0 -> no wake up timer
}


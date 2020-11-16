/*
 * mcu.c
 *
 *  Created on: Jun 1, 2020
 *      Author: Stancoj
 */

#ifndef MCU_H_
#define MCU_H_


#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

void mcu_init(void);

enum{
	LSI_ENABLE,
	LSE_ENABLE,
}OSC_TYPE_ENABLE;

#endif /* MCU_H_ */

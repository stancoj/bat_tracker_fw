/*
 * lpmode.h
 *
 *  Created on: 1. 2. 2018
 *      Author: Stancoj
 */

#ifndef LPMODE_H_
#define LPMODE_H_

#include "main.h"
#include "hw_config.h"

#define SleepOnExit_Enable		1
#define SleepOnExit_Disable		0

void enter_LP_Sleep(uint8_t sleep_on_exit);
void enter_LP_Stop(void);
void enter_LP_Standby(void);

#endif /* LPMODE_H_ */

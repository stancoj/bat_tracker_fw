/*
 * app.h
 *
 *  Created on: Jun 19, 2021
 *      Author: stancoj
 */

#ifndef APP_H_
#define APP_H_

#include "gps.h"
#include "lis2dh.h"
#include "bmp280_app.h"
#include "data_logger.h"
#include "app_ui.h"
#include "lpmode.h"
#include "mcu.h"


#define ALT_TRIGGER_COUNT	10


typedef enum
{
	UI_CONNECTED = 0,
	UI_DISCONNECTED,
}MAIN_UI_STATE;


typedef enum
{
	WAIT_FOR_FIRST_FIX = 0,
	WAIT_FOR_TIME_SETUP,
	WAIT_FOR_ALT_SETUP,
	SET_SLEEP_MODE,
	DATA_LOGER_MODE,
}MAIN_APP_STATE;


typedef enum
{
	WAIT_FOR_ALT_TRIGGER = 0,
	WAKE_UP_GPS,
	WAIT_FOR_GPS_FIX,
	LOG_MEAS_DATA,
}APP_LOGGER_STATE;

void app_main(void);
void app_main_offline(void);
void app_main_online(void);
void app_main_logger(void);

void app_set_main_ui_state(MAIN_UI_STATE state);

uint8_t get_main_app_state(void);
uint8_t get_log_app_state(void);

#endif /* APP_H_ */

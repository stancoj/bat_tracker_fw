/*
 * app_ui.h
 *
 *  Created on: Jan 10, 2021
 *      Author: stancoj
 */

#ifndef APP_UI_H_
#define APP_UI_H_

#include "main.h"
#include "mcu.h"
#include "string.h"

#include "gps.h"
#include "lis2dh.h"
#include "bmp280_app.h"
#include "flash.h"
#include "data_logger.h"

typedef enum
{
	CONNECTION_RESPONSE = 1,
	ERASE_MEMORY,
	READ_MEMORY,
	STATE_MEMORY,
	SET_TIME,
	SET_ALTITUDE,
	STATE_DEVICE,
	DISCONNECT
}APP_COMMAND_;

typedef struct
{
	uint8_t hour;
	uint8_t minute;
}app_cmd_time_;

typedef struct
{
	APP_COMMAND_ cmd_id;
	app_cmd_time_ time;
	uint16_t altitude;
	uint8_t connected;
}app_cmd_;

void receivedByteApp(uint8_t c);
void parseCmdApp(uint8_t* cmd);
void processCmdApp(void);

void appSendResponse(void);
void appSendMemoryState(void);
void appSendDeviceState(void);
void appEraseStoredData(void);
void appReadStoredData(void);
void appDisconnect(void);

void registerAppCallbackSendFn(void *callback);


#endif /* APP_UI_H_ */

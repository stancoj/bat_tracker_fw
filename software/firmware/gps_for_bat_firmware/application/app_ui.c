/*
 * app_ui.c
 *
 *  Created on: Jan 10, 2021
 *      Author: stancoj
 */

#include "app_ui.h"


#define BUFFER_LENGTH	32

extern Gps_Data_s gGpsData;
extern bmp280_sensor_data_ BMP280_data;
extern flash_state_ flash_state_data;
extern logger_state_ logger_state;

uint8_t buffer0[BUFFER_LENGTH] = {0}, buffer1[BUFFER_LENGTH] = {0};
uint8_t *buffer = buffer0;

app_cmd_ appCommand;

void (* appCallbackSendFn)(uint8_t *data, uint16_t length) = 0;


void receivedByteApp(uint8_t c)
{
	static uint8_t index = 0;
	static uint8_t store_byte = 0;

	if(c == '$' || store_byte)
	{
		if(store_byte == 0)
		{
			store_byte = 1;
			return;
		}

		buffer[index++] = c;

		if(c == '#')
		{
			parseCmdApp(buffer);
			processCmdApp();

			memset(buffer, '\0', BUFFER_LENGTH);

			if(buffer == buffer0)
			{
				buffer = buffer1;
			}
			else
			{
				buffer = buffer0;
			}

			index = 0;
			store_byte = 0;

			return;
		}

		if(index >= BUFFER_LENGTH)
		{
			index = 0;
			store_byte = 0;
		}
	}
}


void parseCmdApp(uint8_t* cmd)
{
	appCommand.cmd_id = (10 * (cmd[0] - '0')) + (cmd[1] - '0');

	if(appCommand.cmd_id == SET_TIME)
	{
		appCommand.time.hour = (10 * (cmd[2] - '0')) + (cmd[3] - '0');
		appCommand.time.minute = (10 * (cmd[4] - '0')) + (cmd[5] - '0');

		if(appCommand.time.hour > 23) appCommand.time.hour = 0;
		if(appCommand.time.minute > 59) appCommand.time.minute = 0;
	}
	else if(appCommand.cmd_id == SET_ALTITUDE)
	{
		appCommand.altitude = (1000 * (cmd[2] - '0')) + (100 * (cmd[3] - '0')) +
						   (10 * (cmd[4] - '0')) + (cmd[5] - '0');
	}
}

void processCmdApp(void)
{
	switch(appCommand.cmd_id)
	{
		case CONNECTION_RESPONSE:
			appSendResponse();
			break;

		case ERASE_MEMORY:
			break;

		case READ_MEMORY:
			break;

		case STATE_MEMORY:
			appSendMemoryState();
			break;

		case STATE_DEVICE:
			break;

		case SET_TIME:
			break;

		case SET_ALTITUDE:
			break;

		default:
			break;
	}
}


void appSendResponse(void)
{
	uint8_t data[] = "$BAT_GPS#";

	if(appCallbackSendFn)
	{
		appCallbackSendFn(data, sizeof(data));
	}
}

void appSendMemoryState(void)
{
	uint8_t data[20] = {0};
	uint8_t len;

	sprintf(data, "%lu,%d#", logger_state.free_memory, logger_state.init);

	len = strlen(data);

	for(uint8_t i = 0; i < len; i++)
	{
		  LL_USART_TransmitData8(USART1, data[i]);
		  while(!LL_USART_IsActiveFlag_TC(USART1)){};
		  LL_USART_ClearFlag_TC(USART1);
	}
}

void registerAppCallbackSendFn(void *callback)
{
	appCallbackSendFn = callback;
}



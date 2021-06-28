/*
 * app.c
 *
 *  Created on: Jun 19, 2021
 *      Author: stancoj
 */

#include "app.h"


extern Gps_Data_s gGpsData;
extern Gps_Data_Ubx_s gGpsDataUbx;
extern bmp280_sensor_data_ BMP280_data;
extern flash_state_ flash_state_data;
extern app_cmd_ appCommand;
extern logger_state_ logger_state;

extern uint64_t time;
uint64_t prev_time = 0;

MAIN_UI_STATE main_ui_state = UI_CONNECTED;
MAIN_APP_STATE main_app_state = WAIT_FOR_TIME_SETUP; //WAIT_FOR_FIRST_FIX;
APP_LOGGER_STATE log_app_state = WAIT_FOR_ALT_TRIGGER;


void app_main(void)
{
	//static MAIN_UI_STATE main_ui_state = UI_DISCONNECTED;

	switch(main_ui_state)
	{
		case UI_DISCONNECTED:
		{
			// Bat tracking
			app_main_offline();
			break;
		}
		case UI_CONNECTED:
		{
			// Application UI connected
			app_main_online();
			break;
		}
	}
}


void app_main_offline(void)
{
	//static MAIN_APP_STATE main_app_state = WAIT_FOR_TIME_SETUP; //WAIT_FOR_FIRST_FIX;

	switch(main_app_state)
	{
	  case WAIT_FOR_FIRST_FIX:
	  {
		  // Save time of the first fix
		  if((gGpsData.gpsValid == 1) && (!gGpsData.first_fix))
		  {
			  gGpsData.first_fix = time;
			  main_app_state = WAIT_FOR_TIME_SETUP;
		  }

		  break;
	  }

	  case WAIT_FOR_TIME_SETUP:
	  {
		  if(appCommand.clock.is_set && appCommand.time.is_set)
		  {
			  set_up_CLOCK(appCommand.clock.hour, appCommand.clock.minute, 0);
			  set_up_ALARM(appCommand.time.hour, appCommand.time.minute, 0);

			  main_app_state = WAIT_FOR_ALT_SETUP;
		  }

		  break;
	  }

	  case WAIT_FOR_ALT_SETUP:
	  {
		  if(appCommand.altitude > 0)
		  {
			  main_app_state = SET_SLEEP_MODE;
		  }

		  break;
	  }

	  case SET_SLEEP_MODE:
	  {
		  // Low pwr mode - GPS
		  LowPowerGPS();
		  // Inactive mode - GPS
		  SleepModeGPS();
		  // Enter sleep mode - MCU
		  enter_LP_Stop();

		  main_app_state = DATA_LOGER_MODE;

		  break;
	  }

	  case DATA_LOGER_MODE:
	  {
		  // Log data till the end of the battery life
		  app_main_logger();

		  break;
	  }
	}
}


void app_main_logger(void)
{
	//static APP_LOGGER_STATE log_app_state = WAIT_FOR_ALT_TRIGGER;
	static uint8_t baro_alt_counter = 0;

	switch(log_app_state)
	{
		case WAIT_FOR_ALT_TRIGGER:
		{
			if((rtc_getMs() - prev_time) >= 1250)
			{
				BMP280_data.lock = 1;
				calculateBMP280Altitude();
				BMP280_data.lock = 0;

				if(BMP280_data.bmp_comp_data.bmp_alt.alt_rel >= appCommand.altitude)
				{
					baro_alt_counter += 1;

					if(baro_alt_counter >= ALT_TRIGGER_COUNT)
					{
						baro_alt_counter = 0;
						log_app_state = WAKE_UP_GPS;
					}
				}
				else
				{
					baro_alt_counter = 0;
				}

				prev_time = rtc_getMs();
			}

			break;
		}

		case WAKE_UP_GPS:
		{
			if(WakeUpGPS())
			{
				log_app_state = WAIT_FOR_GPS_FIX;
			}
			else
			{
				// Give it another try
			}

			break;
		}

		case WAIT_FOR_GPS_FIX:
		{
			if(gGpsData.e2D3Dfix == e3Dfix)
			{
				log_app_state = LOG_MEAS_DATA;
			}

			break;
		}

		case LOG_MEAS_DATA:
		{
			// Make barometer altitude measurement and log data
			if((rtc_getMs() - prev_time) >= 1250)
			{
			  BMP280_data.lock = 1;
			  calculateBMP280Altitude();
			  BMP280_data.lock = 0;

			  loggerLogData();
			  prev_time = rtc_getMs();
			}

			break;
		}
	}

}


void app_main_online(void)
{
	if((rtc_getMs() - prev_time) >= 1250)
	{
		BMP280_data.lock = 1;
		calculateBMP280Altitude();
		BMP280_data.lock = 0;

		prev_time = rtc_getMs();
	}
}


void app_set_main_ui_state(MAIN_UI_STATE state)
{
	main_ui_state = state;
}

uint8_t get_main_app_state(void)
{
	return main_app_state;
}

uint8_t get_log_app_state(void)
{
	return log_app_state;
}



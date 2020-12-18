/*
 * data_logger.c
 *
 *  Created on: 27. 11. 2020
 *      Author: Stancoj
 */

#include "data_logger.h"

extern Gps_Data_s gGpsData;
extern Gps_Data_Ubx_s gGpsDataUbx;
extern bmp280_sensor_data_ BMP280_data;

logger_state_ logger_state = {0};
logger_store_data_ logger_data_to_log = {0};

uint16_t data_len = sizeof(logger_store_data_);


void loggerInit(void)
{
	logger_state.write_to_addr = flash_get_free_user_mem();
	logger_state.free_memory = loggerMemoryUsed();
	logger_state.read_from_addr = 0;
	logger_state.init = 1;
}


uint8_t loggerLogData(void)
{
	if((logger_state.free_memory < data_len) && logger_state.init && flash_is_init())
	{
		return 0;
	}

	// Update data to be stored
	logger_data_to_log.data.baro_altitude = (int32_t)(BMP280_data.bmp_comp_data.bmp_alt.alt_rel * 1000);
	logger_data_to_log.data.baro_temp = BMP280_data.bmp_comp_data.bmp_temp.temp32;
	logger_data_to_log.data.gps_longitude = gGpsData.longitude.longitudeMicroDegree;
	logger_data_to_log.data.gps_latitude = gGpsData.latitude.latitudeMicroDegree;
	logger_data_to_log.data.gps_altitude = gGpsData.altitude.altitudeUInt;

	// Write data to flash
	if(!flash_write_data(logger_state.write_to_addr, &(logger_data_to_log.ptr_start), data_len))
	{
		return 0;
	}

	// Update write to address
	logger_state.write_to_addr += data_len;
	// Update free memory left
	logger_state.free_memory = loggerMemoryUsed();

	return 1;
}


/* parsed data in CSV format */
void loggerReadOutProccesedData(void)
{
	uint32_t read_out_addr = logger_state.write_to_addr - data_len;
	logger_store_data_ retrieved_deta = {0};
	char text[75] = {0};

	while(read_out_addr >= FLASH_USER_MEM_SPACE_ADDR)
	{
		memcpy(&(retrieved_deta.ptr_start), (uint8_t*)(read_out_addr), data_len);
		sprintf(text, "%ld,%ld,%ld,%ld,%ld\n\r", retrieved_deta.data.baro_altitude, retrieved_deta.data.baro_temp, retrieved_deta.data.gps_altitude,
												 retrieved_deta.data.gps_latitude, retrieved_deta.data.gps_longitude);
		for(uint8_t i = 0; i < strlen(text); i++)
		{
			  LL_USART_TransmitData8(USART1, text[i]);
			  while(!LL_USART_IsActiveFlag_TC(USART1)){};
			  LL_USART_ClearFlag_TC(USART1);
		}

		read_out_addr -= data_len;
		memset(text, '\0', sizeof(text));
	}
}


/* Raw data. Sent as they are stored in FLASH*/
void loggerReadOutRawData(void)
{
	uint32_t i = 0;

	while((FLASH_USER_MEM_SPACE_ADDR + i) < logger_state.write_to_addr)
	{
		LL_USART_TransmitData8(USART1, flash_read_byte((FLASH_USER_MEM_SPACE_ADDR + i)));
		while(!LL_USART_IsActiveFlag_TC(USART1)){};
		LL_USART_ClearFlag_TC(USART1);

		if(i % 19 == 0)
		{
			LL_USART_TransmitData8(USART1, '\n');
			while(!LL_USART_IsActiveFlag_TC(USART1)){};
			LL_USART_ClearFlag_TC(USART1);

			LL_USART_TransmitData8(USART1, '\r');
			while(!LL_USART_IsActiveFlag_TC(USART1)){};
			LL_USART_ClearFlag_TC(USART1);
		}

		i++;
	}
}



uint32_t loggerMemoryUsed(void)
{
	return (FLASH_END_ADDR - logger_state.write_to_addr);
}



/*
 * data_logger.h
 *
 *  Created on: 27. 11. 2020
 *      Author: Stancoj
 */

#ifndef DATA_LOGGER_H_
#define DATA_LOGGER_H_

#include "flash.h"
#include <stdio.h>

#include "gps.h"
#include "bmp280_app.h"

typedef struct
{
	uint32_t write_to_addr;
	uint32_t read_from_addr;
	uint32_t free_memory;
	uint8_t init;
}logger_state_;


typedef struct
{
	int32_t gps_latitude;
	int32_t gps_longitude;
	int32_t gps_altitude;
	int32_t baro_altitude;
	int32_t baro_temp;
}logger_data_;


typedef union
{
	logger_data_ data;
	uint8_t ptr_start;
}logger_store_data_;

void loggerInit(void);
uint8_t loggerLogData(void);
void loggerReadOutProccesedData(void);
void loggerReadOutRawData(void);

uint32_t loggerMemoryUsed(void);
#endif /* DATA_LOGGER_H_ */

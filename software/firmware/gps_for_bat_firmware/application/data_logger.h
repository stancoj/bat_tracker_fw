/*
 * data_logger.h
 *
 *  Created on: 27. 11. 2020
 *      Author: Stancoj
 */

#ifndef DATA_LOGGER_H_
#define DATA_LOGGER_H_

#include "flash.h"

typedef struct
{
	uint32_t write_addr;
	uint32_t read_addr;
	uint32_t mem_used;
}logger_state_;


void loggerInit(void);
void loggerLogData(uint8_t *data, uint16_t length);
uint32_t loggerMemoryUsed(void);
#endif /* DATA_LOGGER_H_ */

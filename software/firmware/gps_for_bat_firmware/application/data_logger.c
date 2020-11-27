/*
 * data_logger.c
 *
 *  Created on: 27. 11. 2020
 *      Author: Stancoj
 */

#include "data_logger.h"

logger_state_ logger_state;


void loggerInit(void)
{
	logger_state.write_addr = *((volatile uint32_t*)(FLASH_USER_MEM_START_ADDR));
	logger_state.mem_used = loggerMemoryUsed();
	logger_state.read_addr = 0;
}


void loggerLogData(uint8_t *data, uint16_t length)
{
	if(data == 0 || length == 0) return;

	uint16_t i = 0;

	while(FLASH_END_ADDR > logger_state.write_addr && length > i)
	{
		flash_write_byte(logger_state.write_addr, *(data+i));

		i++;
		logger_state.write_addr++;
	}
}


uint32_t loggerMemoryUsed(void)
{
	return 0;
}



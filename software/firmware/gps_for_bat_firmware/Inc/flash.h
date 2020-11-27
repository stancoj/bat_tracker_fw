/*
 * flash.h
 *
 *  Created on: Nov 26, 2020
 *      Author: Stancoj
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "main.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

/* FLASH MEMORY SECTORS */
#define ADDR_FLASH_SECTOR_0     		((uint32_t)0x08000000) /* Base address of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     		((uint32_t)0x08004000) /* Base address of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     		((uint32_t)0x08008000) /* Base address of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     		((uint32_t)0x0800C000) /* Base address of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4	     		((uint32_t)0x08010000) /* Base address of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     		((uint32_t)0x08020000) /* Base address of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     		((uint32_t)0x08040000) /* Base address of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     		((uint32_t)0x08060000) /* Base address of Sector 7, 128 Kbytes */

#define FLASH_INITIAL_SECTOR			FLASH_SECTOR_5
#define FLASH_NUMBER_OF_SECTORS			1

#define FLASH_VOLTAGE_RANGE				FLASH_VOLTAGE_RANGE_3

#define FLASH_START_ADDR				ADDR_FLASH_SECTOR_5
#define FLASH_END_ADDR					(ADDR_FLASH_SECTOR_7 + 0x1FFFF)

#define FLASH_PAGE0_HEADER_BYTE_VAL		(uint8_t)0x0A
#define FLASH_USER_MEM_START_ADDR		(FLASH_START_ADDR + 4lu)
#define FLASH_USER_MEM_SPACE_ADDR		(FLASH_USER_MEM_START_ADDR + 4lu)

#define FLASH_ERROR_ERASE_SECTOR		1
#define FLASH_ERROR_PROGRAM_MEMORY		2
#define FLASH_ERROR_LOCK_MEMORY			3
#define FLASH_ERROR_UNLOCK_MEMORY		4
#define FLASH_ERROR_INIT_FAILED			5

#define FLASH_STATE_PREVIOUS_INIT		2
#define FLASH_STATE_NEW_INIT			1


typedef struct
{
	uint8_t initialized;
	uint8_t error;
	uint32_t free_memory;
}flash_state_;

void flash_init(void);
void flash_write_byte(uint32_t addr, uint8_t data);
void flash_write_word(uint32_t addr, uint32_t data);
void flash_write_data(uint32_t addr, uint8_t *data, uint16_t len);
uint8_t flash_read_byte(uint32_t addr);

uint32_t flash_get_free_mem(void);


#endif /* FLASH_H_ */

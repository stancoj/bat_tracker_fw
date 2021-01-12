/*
 * flash.c
 *
 *  Created on: Nov 26, 2020
 *      Author: Stancoj
 */


#include "flash.h"


volatile flash_state_ flash_state_data = {0};
uint8_t test_byte;

uint32_t flash_get_free_mem(void);


void flash_init(void)
{
	if(flash_read_byte(FLASH_START_ADDR) != FLASH_PAGE0_HEADER_BYTE_VAL)
	{
		/* Variable used for Erase procedure */
		static FLASH_EraseInitTypeDef EraseInitStruct;

		uint32_t SECTORError = 0;

		HAL_FLASH_Unlock();

		EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Sector        = FLASH_SECTOR_5;
		EraseInitStruct.NbSectors     = 3;

		if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
		{
		  while(1){};
		}

		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_START_ADDR, FLASH_PAGE0_HEADER_BYTE_VAL);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_MEM_START_ADDR, FLASH_USER_MEM_SPACE_ADDR);

		HAL_FLASH_Lock();

		LL_mDelay(100);

		if( (flash_read_byte(FLASH_START_ADDR) == FLASH_PAGE0_HEADER_BYTE_VAL)  &&
		    (flash_read_word(FLASH_USER_MEM_START_ADDR) == FLASH_USER_MEM_SPACE_ADDR) )
		{
			flash_state_data.initialized = FLASH_STATE_NEW_INIT;
			flash_state_data.free_memory = FLASH_USER_MEM_SPACE_ADDR;
		}
		else
		{
			flash_state_data.initialized = FLASH_ERROR_INIT_FAILED;
		}
	}
	else
	{
		if(flash_read_word(FLASH_USER_MEM_START_ADDR) == FLASH_USER_MEM_SPACE_ADDR)
		{
			flash_state_data.initialized = FLASH_STATE_PREVIOUS_INIT;
			flash_state_data.free_memory = flash_get_free_mem();
		}
		else
		{
			flash_state_data.initialized = FLASH_ERROR_INIT_FAILED;
		}
	}
}


uint8_t flash_is_init(void)
{
	if(flash_state_data.initialized == FLASH_ERROR_INIT_FAILED)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void flash_write_byte(uint32_t addr, uint8_t data)
{
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr, data))
	{
		flash_state_data.error = FLASH_ERROR_PROGRAM_MEMORY;
	}
}


void flash_write_word(uint32_t addr, uint32_t data)
{
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data))
	{
		flash_state_data.error = FLASH_ERROR_PROGRAM_MEMORY;
	}
}


uint8_t flash_write_data(uint32_t addr, uint8_t *data, uint16_t len)
{
	if((addr + len) > FLASH_END_ADDR)
	{
		return 0;
	}

	if(HAL_FLASH_Unlock())
	{
		flash_state_data.error = FLASH_ERROR_LOCK_MEMORY;
		return 0;
	}

	uint16_t i = 0;

	while(i < len)
	{
		flash_write_byte((addr + i), *(data + i));
		i++;
	}

	if(HAL_FLASH_Lock())
	{
		flash_state_data.error = FLASH_ERROR_LOCK_MEMORY;
		return 0;
	}

	return 1;
}


uint8_t flash_read_byte(uint32_t addr)
{
	return *((volatile uint8_t*)(addr));
}


uint32_t flash_read_word(uint32_t addr)
{
	return *((volatile uint32_t*)(addr));
}


void flash_lock_write(void)
{
	if(HAL_FLASH_Unlock())
	{
		flash_state_data.error = FLASH_ERROR_LOCK_MEMORY;
	}
}


void flash_unlock_write(void)
{
	if(HAL_FLASH_Unlock())
	{
		flash_state_data.error = FLASH_ERROR_UNLOCK_MEMORY;
	}
}


uint32_t flash_get_free_mem(void)
{
	uint32_t addr_to_check;

	// check the 1st page
	if(flash_read_byte(ADDR_FLASH_SECTOR_6 - 1) == 0xFF)
	{
		addr_to_check =  ADDR_FLASH_SECTOR_6 - 2;

		while(addr_to_check >= FLASH_USER_MEM_SPACE_ADDR)
		{
			if(flash_read_byte(addr_to_check) != 0xFF)
			{
				return (addr_to_check + 1);
			}

			addr_to_check--;
		}

		return FLASH_USER_MEM_SPACE_ADDR;
	}
	// check the 2nd page
	else if(flash_read_byte(ADDR_FLASH_SECTOR_7 - 1) == 0xFF)
	{
		addr_to_check =  ADDR_FLASH_SECTOR_7 - 2;

		while(addr_to_check >= ADDR_FLASH_SECTOR_6)
		{
			if(flash_read_byte(addr_to_check) != 0xFF)
			{
				return (addr_to_check + 1);
			}

			addr_to_check--;
		}

		return ADDR_FLASH_SECTOR_6;
	}
	// check the 3rd page
	else if(flash_read_byte(FLASH_END_ADDR) == 0xFF)
	{
		addr_to_check =  FLASH_END_ADDR - 1;

		while(addr_to_check >= ADDR_FLASH_SECTOR_7)
		{
			if(flash_read_byte(addr_to_check) != 0xFF)
			{
				return (addr_to_check + 1);
			}

			addr_to_check--;
		}

		return ADDR_FLASH_SECTOR_7;
	}
	else
	{
		return 0;
	}
}


uint32_t flash_get_free_user_mem(void)
{
	return flash_state_data.free_memory;
}


void flash_erase(void)
{
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError = 0;

	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FLASH_SECTOR_5;
	EraseInitStruct.NbSectors     = 3;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
	  while(1){};
	}

	HAL_FLASH_Lock();

	flash_state_data.free_memory = flash_get_free_mem();
}


/*
 * utilities.c
 *
 *  Created on: 14. 11. 2018
 *      Author: David Rau
 */

#include "utilities.h"

void copyMemory(uint8_t *source, uint8_t *dest, uint8_t length)
{
	uint8_t i;
	for(i = 0; i < length; i++)
	{
		dest[i] = source[i];
	}
}

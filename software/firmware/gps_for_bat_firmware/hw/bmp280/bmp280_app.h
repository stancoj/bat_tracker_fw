/*
 * bmp280.h
 *
 *  Created on: 14. 11. 2020
 *      Author: Stancoj
 */

#ifndef BMP280_APP_H_
#define BMP280_APP_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "mcu.h"
#include "bmp280_api.h"

#define SEAL_LEVEL_PRESSURE_Pa		1013.25f

struct preasure
{
	uint32_t pres_ref;
	double pres_asl;

	uint32_t pres32;
	double pres;
};

struct temperature
{
	int32_t temp32;
	double temp;
};

struct altitude
{
	double alt_asl;
	double alt_rel;
};

struct bmp280_comp_data
{
	struct preasure bmp_pres;
	struct temperature bmp_temp;
	struct altitude bmp_alt;
};

typedef struct
{
	struct bmp280_dev bmp;
	struct bmp280_config bmp_conf;
	struct bmp280_status bmp_status;
	struct bmp280_uncomp_data bmp_ucomp_data;
	struct bmp280_comp_data bmp_comp_data;
}bmp280_sensor_data_;

bool initBMP280_app(void);
void updateBMP280data_forced(void);
void updateBMP280data_normal(void);
void calculateRefferencePress(void);
void calculateBMP280Altitude(void);
int compare( const void* a, const void* b);


#endif /* BMP280_APP_H_ */


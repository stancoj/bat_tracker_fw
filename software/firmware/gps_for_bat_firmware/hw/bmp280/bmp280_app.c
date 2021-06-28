/*
 * bmp280.c
 *
 *  Created on: 14. 11. 2020
 *      Author: Stancoj
 */

#include "bmp280_app.h"

bmp280_sensor_data_ BMP280_data = {0};


bool initBMP280_app(void)
{
	int8_t rslt = 0;

	BMP280_data.bmp.delay_ms = LL_mDelay;
	BMP280_data.bmp.dev_id = (BMP280_I2C_ADDR_SEC << 1);
	BMP280_data.bmp.intf = BMP280_I2C_INTF;
	BMP280_data.bmp.read = i2c_bmp280_read;
	BMP280_data.bmp.write = i2c_bmp280_write;

	rslt = bmp280_init(&BMP280_data.bmp);
	if(rslt != BMP280_OK) return false;

	rslt = bmp280_get_config(&BMP280_data.bmp_conf, &BMP280_data.bmp);
	if(rslt != BMP280_OK) return false;

	// Set BMP280 configuration
	BMP280_data.bmp_conf.filter = BMP280_FILTER_OFF;
	BMP280_data.bmp_conf.os_temp = BMP280_OS_1X;
	BMP280_data.bmp_conf.os_pres = BMP280_OS_1X;
	BMP280_data.bmp_conf.odr = BMP280_ODR_1000_MS;

	// Write configuration to BMP280
    rslt = bmp280_set_config(&BMP280_data.bmp_conf, &BMP280_data.bmp);
	if(rslt != BMP280_OK) return false;
	// Set power mode - SLEEP
	rslt = bmp280_set_power_mode(BMP280_SLEEP_MODE, &BMP280_data.bmp);
	if(rslt != BMP280_OK) return false;

	// Set pressure above sea level
	BMP280_data.bmp_comp_data.bmp_pres.pres_asl = SEAL_LEVEL_PRESSURE_Pa;
	// Save ground level pressure and calculate pressure above sea level
	calculateRefferencePress();
	// Calculate zero absolute altitude
	calculateZeroAbsAltitude();

	return true;
}


void updateBMP280data_normal(void)
{
	// Get raw data
	bmp280_get_uncomp_data(&BMP280_data.bmp_ucomp_data, &BMP280_data.bmp);
	// Compensate temp raw data
	bmp280_get_comp_temp_32bit(&BMP280_data.bmp_comp_data.bmp_temp.temp32, BMP280_data.bmp_ucomp_data.uncomp_temp, &BMP280_data.bmp);
	// Compensate pres raw data
    bmp280_get_comp_pres_32bit(&BMP280_data.bmp_comp_data.bmp_pres.pres32, BMP280_data.bmp_ucomp_data.uncomp_press, &BMP280_data.bmp);
}


void updateBMP280data_forced (void)
{
	int8_t rslt = bmp280_set_power_mode(BMP280_FORCED_MODE, &BMP280_data.bmp);
	if(rslt != BMP280_OK)
	{
		BMP280_data.init_status = 1;
		return;
	}

	do
	{
		rslt = bmp280_get_status(&BMP280_data.bmp_status, &BMP280_data.bmp);
		if(rslt != BMP280_OK)
		{
			BMP280_data.init_status = 1;
			return;
		}

		BMP280_data.bmp.delay_ms(1);
	}while(BMP280_data.bmp_status.measuring == BMP280_MEAS_ONGOING || BMP280_data.bmp_status.measuring == BMP280_IM_UPDATE_ONGOING);


	// Get raw data
	bmp280_get_uncomp_data(&BMP280_data.bmp_ucomp_data, &BMP280_data.bmp);
	// Compensate temp raw data
	bmp280_get_comp_temp_32bit(&BMP280_data.bmp_comp_data.bmp_temp.temp32, BMP280_data.bmp_ucomp_data.uncomp_temp, &BMP280_data.bmp);
	BMP280_data.bmp_comp_data.bmp_temp.temp = BMP280_data.bmp_comp_data.bmp_temp.temp32 / 100.0;

	// Compensate pres raw data
    bmp280_get_comp_pres_32bit(&BMP280_data.bmp_comp_data.bmp_pres.pres32, BMP280_data.bmp_ucomp_data.uncomp_press, &BMP280_data.bmp);
}

/*
 * Takes 5 press samples, leaves the highest and lowest value and calculate mean value of 3 samples
 */
void calculateRefferencePress(void)
{
	uint8_t sample_num = 30;
	//uint32_t temporary_pres[sample_num];

	for(uint8_t i = 0; i < sample_num; i++)
	{
		//measure
		updateBMP280data_forced();
		//save
		//temporary_pres[i] = BMP280_data.bmp_comp_data.bmp_pres.pres32;
		BMP280_data.bmp_comp_data.bmp_pres.pres_ref +=  BMP280_data.bmp_comp_data.bmp_pres.pres32;
		//wait
		BMP280_data.bmp.delay_ms(500);
	}

	/*
	qsort(temporary_pres, sample_num, sizeof(uint32_t), compare);

	for(uint8_t i = 0; i < sample_num; i++)
	{
		BMP280_data.bmp_comp_data.bmp_pres.pres_ref += temporary_pres[i];
	}
	*/

	BMP280_data.bmp_comp_data.bmp_pres.pres_ref /= sample_num;
}

void calculateZeroAbsAltitude(void)
{
	BMP280_data.bmp_comp_data.bmp_alt.alt_zero_abs = calculateAltitude9thOrderPolynomial((double)BMP280_data.bmp_comp_data.bmp_pres.pres_ref);
}

void calculateBMP280Altitude(void)
{
	updateBMP280data_forced();

	//BMP280_data.bmp_comp_data.bmp_alt.alt_asl = 44330.0 * (1.0 - pow(((BMP280_data.bmp_comp_data.bmp_pres.pres / 100.0) / BMP280_data.bmp_comp_data.bmp_pres.pres_asl), 0.1903));
	//BMP280_data.bmp_comp_data.bmp_alt.alt_rel = ((pow((BMP280_data.bmp_comp_data.bmp_pres.pres_ref/(double)BMP280_data.bmp_comp_data.bmp_pres.pres32), (1/5.257)) - 1) * (BMP280_data.bmp_comp_data.bmp_temp.temp + 273.15))/0.0065;
	BMP280_data.bmp_comp_data.bmp_alt.alt_abs = calculateAltitude9thOrderPolynomial(BMP280_data.bmp_comp_data.bmp_pres.pres32);
	BMP280_data.bmp_comp_data.bmp_alt.alt_rel = BMP280_data.bmp_comp_data.bmp_alt.alt_abs - BMP280_data.bmp_comp_data.bmp_alt.alt_zero_abs;
}


double calculateAltitude9thOrderPolynomial(double p)
{
	double y;
    p = p/100;
    y = p_1 * p + p_2;
    y = y * p + p_3;
    y = y * p + p_4;
    y = y * p + p_5;
    y = y * p + p_6;
    y = y * p + p_7;
    y = y * p + p_8;
    y = y * p + p_9;
    y = y * p + p_10;
    return y;
}



int compare( const void* a, const void* b)
{
     uint32_t int_a = * ( (uint32_t*) a );
     uint32_t int_b = * ( (uint32_t*) b );

     return (int_a > int_b) - (int_a < int_b);
}



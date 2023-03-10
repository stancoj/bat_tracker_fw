/*
 * lis2dh.c
 *
 *  Created on: 14. 11. 2018
 *      Author: Stancoj
 */


#include "lis2dh.h"

#define LIS2_DH_CTRL2_FILTER_SETTINGS		(LIS2DH_CTRL2_HPM_NORMAL_REF | LIS2DH_CTRL2_HPCF0)

//extern void mode_switch(int16_t acc_z);
//extern void inclination_alert(void);
//extern void movement_alert(void);
//extern uint8_t isResponseTime(void);
//extern void device_moved();
////extern void endOfInterupt();
//extern uint8_t allow_process_acc_mode(uint8_t newAccMode);
//extern uint8_t mode;
//volatile uint8_t interrupt_processing = 0;

static uint8_t sLis2dhI2Caddress = LIS2DH_DEVICE_ADDRESS_ALTERNATIVE;

uint64_t lastMovementTick = 0;

static uint8_t sLis2dhDataBuffer[10];

uint8_t lis2dh_read_byte(uint8_t reg_addr)
{
	//uint8_t data = 0;
	return i2c_master_read(reg_addr, sLis2dhI2Caddress, sLis2dhDataBuffer, 1);
}

void lis2dh_write_byte(uint8_t reg_addr, uint8_t value)
{
	//i2c_master_write(value, reg_addr, sLis2dhI2Caddress, 0);
}

void lis2dh_readArray(uint8_t * data, uint8_t reg, uint8_t length)
{
	//i2c_master_read(data, length, reg, sLis2dhI2Caddress, 1);
}

int8_t lis2dh_get_temp()
{
	uint8_t temp[2];
	lis2dh_readArray(temp, LIS2DH_ADDRESS_TEMP_L, 2);

	return ((int8_t)temp[1]) + LIS2DH_TEMP_REFERENCE_DEG_C;
}

void lis2dh_get_acc(int16_t* x, int16_t * y, int16_t * z)
{
	uint8_t data[6];
	int16_t xx, yy, zz;

	uint8_t temp;
	//get current scale and use it for final calculation
    temp = lis2dh_read_byte(LIS2DH_ADDRESS_CTRL4);

	temp = temp >> 4;
    temp &= 0x03;			//full scale bits exctracted

//	data = i2c_master_read(6, 0x28, (0x18 << 1), 1);
	lis2dh_readArray(data, LIS2DH_ADDRESS_XL, 6);

	xx = ((uint16_t)data[1]) << 8 | data[0];
	yy = ((uint16_t)data[3]) << 8 | data[2];
	zz = ((uint16_t)data[5]) << 8 | data[4];

//	uint8_t int_source2 = lis2dh_read_byte(LIS2DH_ADDRESS_INT2_SRC);

	//= *(i2c_master_read(2, LIS2DH_ADDRESS_TEMP_L, (0x18 << 1) , 1));

    //adjusting to miliG
    *x = xx >> (4 - temp);
    *y = yy >> (4 - temp);
    *z = zz >> (4 - temp);
}


//FIFO read has value only 8-bit, is store in OUT_X-H, OUT_Y-H, OUT_Z_H,
//attention not same store as in mode without fifo, see function lis2dh_get_acc
uint8_t lis2dh_get_accFIFO(int16_t* x, int16_t* y, int16_t* z)
{
	uint8_t data[192];
	int16_t xx[32], yy[32], zz[32];
	uint8_t i;

	uint8_t temp;
	//get current scale and use it for final calculation
    temp = lis2dh_read_byte(LIS2DH_ADDRESS_CTRL4);

	temp = temp >> 4;
    temp &= 0x03;			//full scale bits exctracted

	uint8_t validCnt = lis2dh_fifo_getSampleCountInBuffer();

	lis2dh_readArray(data, LIS2DH_ADDRESS_XL, 192);

	for(i = 0; i < 32; i++)
	{
		uint8_t offset = i*6;
		xx[i] = (int16_t)((int8_t)data[1 + offset]);
		yy[i] = (int16_t)((int8_t)data[3 + offset]);
		zz[i] = (int16_t)((int8_t)data[5 + offset]);

	    x[i] = xx[i] << (4 + temp);
	    y[i] = yy[i] << (4 + temp);
	    z[i] = zz[i] << (4 + temp);
	}

	return validCnt;
}

uint8_t lis2dh_fifo_getSampleCountInBuffer()
{
	uint8_t fifoSrc = lis2dh_read_byte(LIS2DH_ADDRESS_FIFO_SRC);
	return fifoSrc & LIS2DH_FIFO_CTRL_FTH_MASK;
}

void lis2dh_prepareFIFO(uint8_t count)
{
	//interrupts did not work correctly, so not using them in this code, polling is used instead
//	uint8_t ctrl3 = lis2dh_read_byte(LIS2DH_ADDRESS_CTRL3);
//	ctrl3 |= LIS2DH_CTRL3_I1_OVERRUN;
//	lis2dh_write_byte(LIS2DH_ADDRESS_CTRL3, ctrl3);

	count &= LIS2DH_FIFO_CTRL_FTH_MASK;				//discard higher bits
	uint8_t fifoCtrl = LIS2DH_FIFO_CTRL_FM_BYPASS | LIS2DH_FIFO_CTRL_TR_INT1 | count;
	lis2dh_write_byte(LIS2DH_ADDRESS_FIFO_CTRL, fifoCtrl);

}

void lis2dh_fifoEnable()
{
	uint8_t ctrl5 = lis2dh_read_byte(LIS2DH_ADDRESS_CTRL5);
	ctrl5 |= LIS2DH_CTRL5_FIFO_EN;
	lis2dh_write_byte(LIS2DH_ADDRESS_CTRL5, ctrl5);

	uint8_t fifoCtrl = lis2dh_read_byte(LIS2DH_ADDRESS_FIFO_CTRL);
	fifoCtrl &= (~LIS2DH_FIFO_CTRL_FM_MASK);
	fifoCtrl |= LIS2DH_FIFO_CTRL_FM_STREAM;
	lis2dh_write_byte(LIS2DH_ADDRESS_FIFO_CTRL, fifoCtrl);
}

void lis2dh_fifoDisable()
{
	uint8_t ctrl5 = lis2dh_read_byte(LIS2DH_ADDRESS_CTRL5);
	ctrl5 &= ~LIS2DH_CTRL5_FIFO_EN;
	lis2dh_write_byte(LIS2DH_ADDRESS_CTRL5, ctrl5);

	uint8_t fifoCtrl = lis2dh_read_byte(LIS2DH_ADDRESS_FIFO_CTRL);
	fifoCtrl &= (~LIS2DH_FIFO_CTRL_FM_MASK);
	fifoCtrl |= LIS2DH_FIFO_CTRL_FM_BYPASS;
	lis2dh_write_byte(LIS2DH_ADDRESS_FIFO_CTRL, fifoCtrl);
}


uint8_t lis2dh_init(void)
{

	if(LIS2DH_WHO_AM_I_VALUE == i2c_master_read(LIS2DH_WHO_AM_I_ADDRES, LIS2DH_DEVICE_ADDRESS, sLis2dhDataBuffer, 1))
	{
		return 1;
	}
	else
	{
		return 0;
	}

	/*
	uint8_t status = 1;

	LIS2DH_ACC_ON;
	LL_mDelay(100);


	uint8_t val = lis2dh_read_byte(LIS2DH_WHO_AM_I_ADDRES);

	if(val == LIS2DH_WHO_AM_I_VALUE)
	{
		status = 1;
	}
	else			//if the device is not found on one address, try another one
	{
		sLis2dhI2Caddress = LIS2DH_DEVICE_ADDRESS;
		val = lis2dh_read_byte(LIS2DH_WHO_AM_I_ADDRES);
		if(val == LIS2DH_WHO_AM_I_VALUE)
		{
			status = 1;
		}
		else
		{
			status = 0;
			return status;
		}
	}

	//acc device init
	lis2dh_write_byte(LIS2DH_ADDRESS_CTRL0, LIS2DH_CTRL0_SDO_PULL_UP_OFF);

	uint8_t ctrl1 = LIS2DH_SELECTED_ODR|LIS2DH_CTRL1_AXES_ON|LIS2DH_CTRL1_LOW_POWER_ON;
	lis2dh_write_byte(LIS2DH_ADDRESS_CTRL1, ctrl1);

	uint8_t ctrl2 = LIS2_DH_CTRL2_FILTER_SETTINGS ;
	lis2dh_write_byte(LIS2DH_ADDRESS_CTRL2, ctrl2);

	uint8_t ctrl4 = LIS2DH_CTRL4_4WIRE | LIS2DH_CTRL4_SELF_TEST_NO |
		LIS2DH_CTRL4_SELF_HR_OFF | LIS2DH_CTRL4_SELF_FS_2G | LIS2DH_CTRL4_LSB_LOWEST|LIS2DH_CTRL4_BLOCK_DATA_UPDATE;
	lis2dh_write_byte(LIS2DH_ADDRESS_CTRL4, ctrl4);

	uint8_t ctrlTemp = LIS2DH_CTRL_TEMP_ON;
	lis2dh_write_byte(LIS2DH_ADDRESS_TEMP_CFG, ctrlTemp);

	lis2dh_prepareFIFO(10);

	return status;
	*/
}





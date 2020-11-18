/*
 * lis2dh.h
 *
 *  Created on: 14. 11. 2018
 *      Author: Stancoj
 */

#ifndef LIS2DH_LIS2DH_H_
#define LIS2DH_LIS2DH_H_

#include "mcu.h"

#define LIS2DH_SELECTED_FS			LIS2DH_CTRL4_SELF_FS_2G
#define LIS2DH_SELECTED_ODR			LIS2DH_CTRL1_ODR_10Hz




#define 	LIS2DH_DEVICE_ADDRESS				(0x18 << 1)
#define 	LIS2DH_DEVICE_ADDRESS_ALTERNATIVE	(0x19 << 1)				//alternative for various hardware configurations,
																		//initialization should detect which one is used
#define 	LIS2DH_ACC_ON						LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8)
#define 	LIS2DH_ACC_OFF						LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8)

#define 	LIS2DH_INT1_ACTIVE					(GPIOB->IDR & 0x0002) >> 1
#define 	LIS2DH_INT2_ACTIVE					(GPIOB->IDR & 0x0001)


#define 	LIS2DH_ADDRESS_STATUS				0x07		//
#define 	LIS2DH_ADDRESS_TEMP_L				0x0C		//
#define 	LIS2DH_ADDRESS_TEMP_H				0x0D		//
#define 	LIS2DH_ADDRESS_INT_COUNTER			0x0E
#define 	LIS2DH_ADDRESS_WHO_AM_I				0x0F

#define 	LIS2DH_ADDRESS_TEMP_CFG				0x1F

#define 	LIS2DH_ADDRESS_CTRL0				0x1E		//pull-up for address bit	//undocumented in Lis2DH datasheet, but present
#define 	LIS2DH_ADDRESS_CTRL1				0x20		//ODR, ax enable
#define 	LIS2DH_ADDRESS_CTRL2				0x21		//high pass filter
#define 	LIS2DH_ADDRESS_CTRL3				0x22		//interrupts
#define 	LIS2DH_ADDRESS_CTRL4				0x23		//endian, full scale, self test, SPI settings
#define 	LIS2DH_ADDRESS_CTRL5				0x24		//boot, fifo enable ,4D detection, latch interrupt,
#define 	LIS2DH_ADDRESS_CTRL6				0x25		//interrupts,


#define 	LIS2DH_ADDRESS_INT1_CFG				0x30		//interrupt 1 set up
#define 	LIS2DH_ADDRESS_INT1_THS				0x32		//interrupt threshold
#define		LIS2DH_ADDRESS_INT1_DUR				0x33		//interrupt condition duration
#define 	LIS2DH_ADDRESS_INT2_CFG				0x34		//interrupt 1 set up
#define 	LIS2DH_ADDRESS_INT2_THS				0x36		//interrupt threshold
#define		LIS2DH_ADDRESS_INT2_DUR				0x37		//interrupt condition duration


#define 	LIS2DH_ADDRESS_REFERENCE			0x26		//reference value for interrupt generation
#define 	LIS2DH_ADDRESS_STATUS2				0x27		//overruns, new data available

#define 	LIS2DH_ADDRESS_XL					0x28
#define 	LIS2DH_ADDRESS_XH					0x29
#define 	LIS2DH_ADDRESS_YL					0x2A
#define 	LIS2DH_ADDRESS_YH					0x2B
#define 	LIS2DH_ADDRESS_ZL					0x2C
#define 	LIS2DH_ADDRESS_ZH					0x2D

#define     LIS2DH_ADDRESS_FIFO_CTRL			0x2E
#define     LIS2DH_ADDRESS_FIFO_SRC				0x2F

#define		LIS2DH_ADDRESS_ACT_TSH				0x3E
#define		LIS2DH_ADDRESS_ACT_DUR				0x3F

#define		LIS2DH_ADDRESS_INT1_SRC				0x31
#define		LIS2DH_ADDRESS_INT2_SRC				0x35

#define		LIS2DH_READ							0x80
#define		LIS2DH_WRITE						0x00

#define		LIS2DH_AUTO_INCREMENT				0x40

#define 	LIS2DH_WHO_AM_I_VALUE				0x33
#define 	LIS2DH_WHO_AM_I_ADDRES				0x0F

//CTRL_TEMP
#define 	LIS2DH_CTRL_TEMP_OFF				0x00
#define 	LIS2DH_CTRL_TEMP_ON					0xC0

//CTRL0
#define 	LIS2DH_CTRL0_SDO_PULL_UP_ON			0x10	//default
#define 	LIS2DH_CTRL0_SDO_PULL_UP_OFF		0x90



//CTRL1
//--------------------
#define 	LIS2DH_CTRL1_ODR_POWER_DOWN			0x00
#define 	LIS2DH_CTRL1_ODR_1Hz				0x10
#define 	LIS2DH_CTRL1_ODR_10Hz				0x20
#define 	LIS2DH_CTRL1_ODR_25Hz				0x30
#define 	LIS2DH_CTRL1_ODR_50Hz				0x40
#define 	LIS2DH_CTRL1_ODR_100Hz				0x50
#define 	LIS2DH_CTRL1_ODR_200Hz				0x60
#define 	LIS2DH_CTRL1_ODR_400Hz				0x70
#define 	LIS2DH_CTRL1_ODR_1620Hz_LP			0x80
#define 	LIS2DH_CTRL1_ODR_1344_5376			0x90		//1344Hz for Hr/Normal mode;   5376Hz for Low power mode

#define		LIS2DH_CTRL1_AXES_ON				0x07
#define		LIS2DH_CTRL1_AXES_OFF				0x00

#define		LIS2DH_CTRL1_LOW_POWER_ON			0x08
#define		LIS2DH_CTRL1_LOW_POWER_OFF			0x00
//--------------------

//CTRL2
//--------------------
#define 	LIS2DH_CTRL2_HPIA1_OFF				0x00		//high pass for int1
#define 	LIS2DH_CTRL2_HPIA1_ON				0x01

#define 	LIS2DH_CTRL2_HPIA2_OFF				0x00		//high pass for int2
#define 	LIS2DH_CTRL2_HPIA2_ON				0x02

#define 	LIS2DH_CTRL2_HPCLICK_OFF			0x00		//high pass for click
#define 	LIS2DH_CTRL2_HPCLICK_ON				0x04

#define 	LIS2DH_CTRL2_FDS_OFF				0x00		//filtered data selection - for output register and fifo
#define 	LIS2DH_CTRL2_FDS_ON					0x04

#define 	LIS2DH_CTRL2_HPCF0					0x00		//high pass cutoff frequency selection
#define 	LIS2DH_CTRL2_HPCF1					0x10
#define 	LIS2DH_CTRL2_HPCF2					0x20
#define 	LIS2DH_CTRL2_HPCF3					0x30

#define 	LIS2DH_CTRL2_HPM_NORMAL_REF			0x00
#define 	LIS2DH_CTRL2_HPM_REFERENCE			0x40
#define 	LIS2DH_CTRL2_HPM_NORMAL				0x80
#define 	LIS2DH_CTRL2_HPM_AUTORESET			0xC0


//CTRL3
#define 	LIS2DH_CTRL3_I1_CLICK				0x80
#define 	LIS2DH_CTRL3_I1_IA1					0x40
#define 	LIS2DH_CTRL3_I1_IA2					0x20
#define 	LIS2DH_CTRL3_I1_ZYXDA				0x10
#define 	LIS2DH_CTRL3_I1_WTM					0x04
#define 	LIS2DH_CTRL3_I1_OVERRUN				0x02


//CTRL4
//--------------------
#define 	LIS2DH_CTRL4_3WIRE					0x01
#define 	LIS2DH_CTRL4_4WIRE					0x00

#define		LIS2DH_CTRL4_SELF_TEST_NO			0x00
#define		LIS2DH_CTRL4_SELF_TEST_0			0x02
#define		LIS2DH_CTRL4_SELF_TEST_1			0x06

#define		LIS2DH_CTRL4_SELF_HR_ON				0x08
#define		LIS2DH_CTRL4_SELF_HR_OFF			0x00

#define		LIS2DH_CTRL4_SELF_FS_2G				0x00
#define		LIS2DH_CTRL4_SELF_FS_4G				0x10
#define		LIS2DH_CTRL4_SELF_FS_8G				0x20
#define		LIS2DH_CTRL4_SELF_FS_16G			0x30

#define		LIS2DH_CTRL4_LSB_LOWEST				0x00		//endian
#define		LIS2DH_CTRL4_MSB_LOWEST				0x40		//endian

#define		LIS2DH_CTRL4_BLOCK_DATA_UPDATE		0x80
#define		LIS2DH_CTRL4_BLOCK_DATA_UPDATE_OFF	0x00
//----------------------

//CTRL5
#define 	LIS2DH_CTRL5_BOOT					0x80
#define 	LIS2DH_CTRL5_FIFO_EN				0x40
#define 	LIS2DH_CTRL5_LIR_I1					0x08
#define 	LIS2DH_CTRL5_D4D_INT1				0x04
#define 	LIS2DH_CTRL5_LIR_I2					0x02
#define 	LIS2DH_CTRL5_D4D_INT2				0x01



//CTRL6
//--------------------
#define 	LIS2DH_CTRL6_I2_CLICK				0x80
#define 	LIS2DH_CTRL6_I2_IA1					0x40
#define 	LIS2DH_CTRL6_I2_IA2					0x20
#define 	LIS2DH_CTRL6_I2_BOOT				0x10
#define 	LIS2DH_CTRL6_I2_ACT					0x08
#define 	LIS2DH_CTRL6_I2_POLARITY			0x02


//INT_CFG
//--------------------
#define 	LIS2DH_INTCFG_X_ACC_HIGH			(1 << 1)
#define 	LIS2DH_INTCFG_X_ACC_LOW				(1 << 0)

#define 	LIS2DH_INTCFG_Y_ACC_HIGH			(1 << 3)
#define 	LIS2DH_INTCFG_Y_ACC_LOW				(1 << 2)

#define 	LIS2DH_INTCFG_Z_ACC_HIGH			(1 << 5)
#define 	LIS2DH_INTCFG_Z_ACC_LOW				(1 << 4)

#define 	LIS2DH_INTCFG_AND_INT_COMBINATION	(1 << 7)
#define 	LIS2DH_INTCFG_MOVEMENT_RECOGNITION	(1 << 6)
#define 	LIS2DH_INTCFG_POSITION_RECOGNITION	(3 << 6)

//TEMPERATURE
#define 	LIS2DH_TEMP_REFERENCE_DEG_C			(25)


//INT_SRC
#define 	LIS2DH_INT_SRC_X_LOW				(0x01)
#define	 	LIS2DH_INT_SRC_X_HIGH				(0x02)
#define 	LIS2DH_INT_SRC_Y_LOW				(0x04)
#define	 	LIS2DH_INT_SRC_Y_HIGH				(0x08)
#define 	LIS2DH_INT_SRC_Z_LOW				(0x10)
#define	 	LIS2DH_INT_SRC_Z_HIGH				(0x20)
#define		LIS2DH_INT_SRC_IA					(0x40)	//interrupt occured

//FIFO_CTRL
#define 	LIS2DH_FIFO_CTRL_FM_BYPASS			(0x00)
#define 	LIS2DH_FIFO_CTRL_FM_FIFO			(0x40)
#define 	LIS2DH_FIFO_CTRL_FM_STREAM			(0x80)
#define 	LIS2DH_FIFO_CTRL_FM_STREAM_TO_FIFO  (0xC0)
#define		LIS2DH_FIFO_CTRL_FM_MASK  			(0xC0)
#define		LIS2DH_FIFO_CTRL_TR_INT1			(0x00)
#define		LIS2DH_FIFO_CTRL_TR_INT2			(0x20)
#define		LIS2DH_FIFO_CTRL_FTH_MASK			(0x1F)

//FIFO_SRC
#define 	LIS2DH_FIFO_SRC_WTM					(0x80)
#define 	LIS2DH_FIFO_SRC_OVRN_FIFO			(0x40)
#define 	LIS2DH_FIFO_SRC_EMPTY				(0x20)


uint8_t lis2dh_init(void);
void lis2dh_get_acc(int16_t* x, int16_t * y, int16_t * z);
uint8_t lis2dh_read_byte(uint8_t reg_addr);
void lis2dh_set_meas_freq(uint8_t meas_frequency);
int8_t lis2dh_get_temp();

void lis2dh_write_byte(uint8_t reg_addr, uint8_t value);

void lis2dh_fifoEnable();
void lis2dh_fifoDisable();
uint8_t lis2dh_fifo_getSampleCountInBuffer();
uint8_t lis2dh_get_accFIFO(int16_t* x, int16_t * y, int16_t * z);


#endif /* LIS2DH_LIS2DH_H_ */

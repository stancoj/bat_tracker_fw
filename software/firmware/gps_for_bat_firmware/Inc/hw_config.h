/*
 * hw_config.h
 *
 *  Created on: 21. 11. 2018
 *      Author: David Rau
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

//global settings
#undef HUMAN
#define HSI

//BUFFERS
#define MAXIMUM_GPS_BUFFER_LENGTH 128

//CALCULATION
#define SEC_TO_MS						(1000)
#define MIN_TO_MS						(60 * SEC_TO_MS)
#define HOUR_TO_MS						(60 * MIN_TO_MS)
#define DAY_TO_MS						(24 * HOUR_TO_MS)

//GPS RUN
#define ACQUISITION_DURATION_MS			(90 * SEC_TO_MS)
#define BACKUP_GPS_NO_ACTIVITY_MS		(2500)

//GPS INIT
#define GPSI_MSG_INIT_REC_TIMEOUT_MS	(3 * SEC_TO_MS)
#define GPSI_NO_ANS_REC_TIMEOUT_MS		(1 * SEC_TO_MS)
#define GPSI_DELAY_AFTER_SYNC_MS		(50)

//MESSAGES
#define TRACK_MSG_PERIOD_MS				(2 * MIN_TO_MS)
#define DEACTIVE_CONTROL_MSG_MS			(12 * HOUR_TO_MS)
#define STAND_BY_MSG_PERIOD_MS			(4 * HOUR_TO_MS)
#define DOWNLINK_PERIOD_ONE_DAY_MS		(1 * DAY_TO_MS)

//TEMPERATURE
#define TEMPERATURE_THRESHOLD_C			(50)
#define TEMP_MEASURE_TIMEOUT_MS			(15 * MIN_TO_MS)
#define TEMP_SAFETY_TIMEOUT_MS			(60 * SEC_TO_MS)

//ACC
#define ACC_MOVE_THRESHOLD_MG			(32)
#define ACC_MOVE_DURATION_MS			(400)
#define ACC_POS_THRESHOLD_MG			(714)
#define ACC_POS_DURATION_MS				(500)


//VOLTAGE
#define EXT_VOLTAGE_STEP_MV				(50)//mv
#define BAT_VOLTAGE_STEP_MV				(20)//mv
#define EXTERN_VOLT_MIN_MV				(4200)
#define EXTERN_VDD_VOLT_MIN_MV			(3400)

//MEAS
#define MEAS_GROUND_PERIOD_MS			(1000)
#define GROUND_INPUTS_AND_MEAS_MS		(100)
#define GROUND_MEAS_MS					(900)

//TIME
#define LPTIM_TICK_PERIOD_MS			(250)		//250ms for switching algorithm, 1s for voltage measurement

//SIGFOX
#define SIGFOX_DAY_LIMIT				(140)

#endif /* HW_CONFIG_H_ */

/**
  ******************************************************************************
  * File Name          : RTC.h
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __rtc_H
#define __rtc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

 extern void _Error_Handler(char *, int);

 extern void iwdg_reset_counter(void);

 #define SECONDS_PER_MIN		60
 #define SECONDS_PER_HOUR		3600
 #define HOURS_PER_DAY			24
 #define DAYS_PER_YEAR			365

 /* Defines used for the bit position in the register and perform offsets */
 #define RTC_POSITION_TR_HT            (uint32_t)20U
 #define RTC_POSITION_TR_HU            (uint32_t)16U
 #define RTC_POSITION_TR_MT            (uint32_t)12U
 #define RTC_POSITION_TR_MU            (uint32_t)8U
 #define RTC_POSITION_TR_ST            (uint32_t)4U
 #define RTC_POSITION_TR_SU            (uint32_t)0U
 #define RTC_POSITION_DR_YT            (uint32_t)20U
 #define RTC_POSITION_DR_YU            (uint32_t)16U
 #define RTC_POSITION_DR_MT            (uint32_t)12U
 #define RTC_POSITION_DR_MU            (uint32_t)8U
 #define RTC_POSITION_DR_DT            (uint32_t)4U
 #define RTC_POSITION_DR_DU            (uint32_t)0U
 #define RTC_POSITION_DR_WDU           (uint32_t)13U
 #define RTC_POSITION_ALMA_DT          (uint32_t)28U
 #define RTC_POSITION_ALMA_DU          (uint32_t)24U
 #define RTC_POSITION_ALMA_HT          (uint32_t)20U
 #define RTC_POSITION_ALMA_HU          (uint32_t)16U
 #define RTC_POSITION_ALMA_MT          (uint32_t)12U
 #define RTC_POSITION_ALMA_MU          (uint32_t)8U
 #define RTC_POSITION_ALMA_SU          (uint32_t)0U
 #define RTC_POSITION_ALMA_ST          (uint32_t)4U
 #define RTC_POSITION_ALMB_DT          (uint32_t)28U
 #define RTC_POSITION_ALMB_DU          (uint32_t)24U
 #define RTC_POSITION_ALMB_HT          (uint32_t)20U
 #define RTC_POSITION_ALMB_HU          (uint32_t)16U
 #define RTC_POSITION_ALMB_MT          (uint32_t)12U
 #define RTC_POSITION_ALMB_MU          (uint32_t)8U
 #define RTC_POSITION_ALMB_SU          (uint32_t)0U
 #define RTC_POSITION_ALMB_ST          (uint32_t)4U
 #define RTC_POSITION_PRER_PREDIV_A    (uint32_t)16U
 #define RTC_POSITION_ALMA_MASKSS      (uint32_t)24U
 #define RTC_POSITION_ALMB_MASKSS      (uint32_t)24U
 #define RTC_POSITION_TS_HU            (uint32_t)16U
 #define RTC_POSITION_TS_MNU           (uint32_t)8U
 #define RTC_POSITION_TS_WDU           (uint32_t)13U
 #define RTC_POSITION_TS_MU            (uint32_t)8U

 typedef struct{
 	uint16_t subsec;
 	uint16_t ms;
 	uint8_t sec;
 	uint8_t min;
 	uint8_t hour;
 	uint8_t day;
 	uint8_t mon;
 	uint8_t year;
 }mytime_t;

 void RTC_Init(uint16_t time, uint8_t osc_type);
 void rtc_registerCallback(void *callback);


 mytime_t rtc_getDateTime(void);
 uint32_t rtc_getSecondsOfYear(mytime_t actual_date_time);

 uint64_t rtc_getMs(void);

 uint8_t rtc_isLeapYear(uint8_t actual_year);
 uint32_t rtc_countDaysInMonths(uint8_t actual_month, uint8_t actual_year);
 uint8_t rtc_extraLeapYearDays(uint8_t actual_year);

 void rtc_ClearRSF(void);
 void set_up_WUT(uint32_t wake_up_time);
 void set_up_CLOCK(uint32_t hour, uint32_t min, uint32_t sec);
 void set_up_ALARM(uint32_t hour, uint32_t min, uint32_t sec);
 void rtc_registerCallback(void *callback);
 void Enable_LSI(void);
 void Enable_LSE(void);

#ifdef __cplusplus
}
#endif
#endif /*__ rtc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

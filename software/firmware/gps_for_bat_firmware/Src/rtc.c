/**
  ******************************************************************************
  * File Name          : RTC.c
  * Description        : This file provides code for the configuration
  *                      of the RTC instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


extern uint8_t send_regular_message;
extern uint8_t rtc_init_flag;

static void (* sCallbackRTC)(uint16_t) = 0;
mytime_t dt;


void RTC_Alarm_IRQHandler(void)
{
	if(LL_RTC_IsActiveFlag_ALRA(RTC))
	{
		rtc_ClearRSF();
		LL_RTC_ClearFlag_ALRA(RTC);
	}

	EXTI->PR |= (1 << 17);
}


/* RTC init function */
void RTC_Init(uint16_t time, uint8_t osc_type)
{
  LL_RTC_InitTypeDef RTC_InitStruct;
  LL_RTC_TimeTypeDef RTC_TimeStruct;
  LL_RTC_DateTypeDef RTC_DateStruct;

  LL_PWR_EnableBkUpAccess();

  LL_RCC_ForceBackupDomainReset();

  LL_RCC_ReleaseBackupDomainReset();

  LL_mDelay(100);

  osc_type ? Enable_LSE() : Enable_LSI();

  LL_PWR_EnableBkUpAccess();

  /*Set RTC clock source */
  osc_type ? LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE) : LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  /* Peripheral clock enable */
  LL_RCC_EnableRTC();

  /* WUT interrupt */
  //LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_20);
  //LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_20);
  //NVIC_SetPriority(RTC_WKUP_IRQn, 0);
  //NVIC_EnableIRQ(RTC_WKUP_IRQn);
  /* Alarm interrupt */
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_17);
  LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_17);
  NVIC_SetPriority(RTC_Alarm_IRQn, 0);
  NVIC_EnableIRQ(RTC_Alarm_IRQn);

  /**Initialize RTC and set the Time and Date
  */
  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 127;//31;
  RTC_InitStruct.SynchPrescaler = 255;//999;
  LL_RTC_Init(RTC, &RTC_InitStruct);

  //Todo: figure out why is power consumption increasing after the first wake up of mcu from STOP mode
  /* Enable WUT & WUTIT */
  /*
  if(time)
  {
	  LL_RTC_DisableWriteProtection(RTC);

	  LL_RTC_WAKEUP_Disable(RTC);
	  while(RTC->ISR & (1 << 10)){};
	  LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE); //LL_RTC_WAKEUPCLOCK_CKSPRE LL_RTC_WAKEUPCLOCK_DIV_16
	  LL_RTC_WAKEUP_SetAutoReload(RTC, time); //2048
	  LL_RTC_EnableIT_WUT(RTC);
	  LL_RTC_WAKEUP_Enable(RTC);
  }
  */

  //Enable read SSR, TR, DR from shadow register
  RTC->CR &= ~(1 << 5);

  LL_RTC_EnableWriteProtection(RTC);

  if(LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0) != 0x32F2)
  {

	  RTC_TimeStruct.Hours = 0;
	  RTC_TimeStruct.Minutes = 0;
	  RTC_TimeStruct.Seconds = 0;
	  LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);

	  RTC_DateStruct.Day = 0;
	  RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_MONDAY;
	  RTC_DateStruct.Month = LL_RTC_MONTH_JANUARY;
	  RTC_DateStruct.Year = 1;
	  LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_DateStruct);

	  LL_RTC_BAK_SetRegister(RTC,LL_RTC_BKP_DR0,0x32F2);
  }
    /**Initialize RTC and set the Time and Date
    */

  if(LL_RTC_BAK_GetRegister(RTC, LL_RTC_BKP_DR0) != 0x32F2)
  {
    LL_RTC_BAK_SetRegister(RTC,LL_RTC_BKP_DR0,0x32F2);
  }

  LL_PWR_DisableBkUpAccess();

}


void Enable_LSI(void)
{
  /* Enable LSI Oscillator */
  LL_RCC_LSI_Enable();

  while(LL_RCC_LSI_IsReady() != 1)
  {
  };
}

void Enable_LSE(void)
{
	/* Enable LSE Oscillator */
	LL_PWR_EnableBkUpAccess();
	LL_RCC_LSE_Enable();
	LL_PWR_DisableBkUpAccess();

	LL_mDelay(500);

	while(LL_RCC_LSE_IsReady() != 1)
	{
	};
}


void set_up_CLOCK(uint32_t hour, uint32_t min, uint32_t sec)
{
	LL_RTC_TimeTypeDef RTC_TimeStruct;

	LL_PWR_EnableBkUpAccess();
	LL_RTC_DisableWriteProtection(RTC);

	LL_mDelay(100);

	LL_RTC_EnterInitMode(RTC);

	RTC_TimeStruct.Seconds = sec;
	RTC_TimeStruct.Minutes = min;
	RTC_TimeStruct.Hours = hour;
	RTC_TimeStruct.TimeFormat = LL_RTC_TIME_FORMAT_AM_OR_24;
	LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_TimeStruct);

	LL_RTC_ExitInitMode(RTC);

	LL_RTC_EnableWriteProtection(RTC);
	LL_PWR_DisableBkUpAccess();
}


void set_up_ALARM(uint32_t hour, uint32_t min, uint32_t sec)
{
	LL_PWR_EnableBkUpAccess();
	LL_RTC_DisableWriteProtection(RTC);

	LL_mDelay(100);

	LL_RTC_ALMA_Disable(RTC);
	while(!LL_RTC_IsActiveFlag_ALRAW(RTC)){};

	LL_RTC_ClearFlag_ALRA(RTC);

	LL_RTC_ALMA_ConfigTime(	RTC,
							LL_RTC_ALMA_TIME_FORMAT_AM,
							__LL_RTC_CONVERT_BIN2BCD(hour),
							__LL_RTC_CONVERT_BIN2BCD(min),
							__LL_RTC_CONVERT_BIN2BCD(sec));

	LL_RTC_ALMA_SetMask(RTC, LL_RTC_ALMA_MASK_DATEWEEKDAY | LL_RTC_ALMA_MASK_SECONDS);
	LL_RTC_ALMA_SetSubSecondMask(RTC, 0x00);

	LL_RTC_EnableIT_ALRA(RTC);
	LL_RTC_ALMA_Enable(RTC);

	LL_RTC_EnableWriteProtection(RTC);
	LL_PWR_DisableBkUpAccess();
}


void set_up_WUT(uint32_t wake_up_time)
{
	//uint32_t wake_WUT = ceil(wake_up_time / 4) - 1;

	LL_PWR_EnableBkUpAccess();
	LL_RTC_DisableWriteProtection(RTC);

	LL_RTC_DisableIT_WUT(RTC);
	LL_RTC_WAKEUP_Disable(RTC);

	LL_mDelay(500);

	while(RTC->ISR & (1 << 10)){};

	LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE); //LL_RTC_WAKEUPCLOCK_CKSPRE LL_RTC_WAKEUPCLOCK_DIV_16

	LL_RTC_WAKEUP_SetAutoReload(RTC, (wake_up_time - 1)); //*2048

	LL_RTC_EnableIT_WUT(RTC);

	LL_RTC_WAKEUP_Enable(RTC);

	LL_RTC_EnableWriteProtection(RTC);
	LL_PWR_DisableBkUpAccess();
}


/**
 * spocitam dni za ubehnute mesiace, ak je priestupny rok, tak februar ma 29 a nie 28 dni
 */
uint32_t rtc_countDaysInMonths(uint8_t actual_month, uint8_t actual_year)
{
	static uint8_t days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	uint16_t days = 0;

	if(rtc_isLeapYear(actual_year))
	{
		days_in_month[1] = 29;
	}
	else
	{
		days_in_month[1] = 28;
	}

	for(uint8_t i = 0; i < actual_month; i++)
	{
		days = days + days_in_month[i];
	}

	return days;
}


/**
 * Zistim ci je rok priestupny
 */
uint8_t rtc_isLeapYear(uint8_t actual_year)
{
	if((actual_year % 4) == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


/**
 * Pripocitanie extra dni za vsetky ubehnute priestupne roky
 * Ak bol minuly rok priestupny a som uz v nasledujucom roku, tak ratam s n+1 dnom navyse
 */
uint8_t rtc_extraLeapYearDays(uint8_t actual_year)
{
	static uint8_t prev_year = 1, extra_days = 0;

	if(rtc_isLeapYear(prev_year) && (actual_year != prev_year))
	{
		prev_year = actual_year;
		extra_days += 1;
		return extra_days;
	}
	else
	{
		prev_year = actual_year;
		return extra_days;
	}

}


uint64_t rtc_getMs(void)
{
    //mytime_t dt;

    rtc_ClearRSF();
    while((RTC->ISR & RTC_ISR_RSF) != RTC_ISR_RSF){}; // wait until RSF is set by hardware

	uint32_t prediv_s = RTC->PRER & RTC_PRER_PREDIV_S_Msk;
	uint32_t ssr = RTC->SSR;
	uint32_t tr = RTC->TR;
	uint32_t dr = RTC->DR;

	dt.ms = (((prediv_s - ssr) * 1000)/(prediv_s + 1)); //milisec
	dt.sec = (uint8_t)((tr >> RTC_POSITION_TR_SU) & 0xF) + (uint8_t)(10 * ((tr >> RTC_POSITION_TR_ST) & 0x7));
	dt.min = (uint8_t)((tr >> RTC_POSITION_TR_MU) & 0xF) + (uint8_t)(10 * ((tr >> RTC_POSITION_TR_MT) & 0x7));
	dt.hour = (uint8_t)((tr >> RTC_POSITION_TR_HU) & 0xF) + (uint8_t)(10 * ((tr >> RTC_POSITION_TR_HT) & 0x3));
	dt.day = (uint8_t)((dr >> RTC_POSITION_DR_DU) & 0xF) + (uint8_t)(10 * ((dr >> RTC_POSITION_DR_DT) & 0x3));
	dt.mon = ((uint8_t)((dr >> RTC_POSITION_DR_MU) & 0xF) + (uint8_t)(10 * ((dr >> RTC_POSITION_DR_MT) & 0x1))) - 1;
	dt.year = (uint8_t)((dr >> RTC_POSITION_DR_YU) & 0xF) + (uint8_t)(10 * ((dr >> RTC_POSITION_DR_YT) & 0xF));

	uint64_t secTotal = 0;

	if(dt.year == 0)
	{
		secTotal = 0;
	}

    secTotal = 				(uint64_t)dt.sec;																	//sekundy
    secTotal +=				(uint64_t)(SECONDS_PER_MIN * (uint64_t)dt.min);											//minuty
    secTotal +=				(uint64_t)(SECONDS_PER_HOUR * (uint64_t)dt.hour);													//hodiny
	secTotal +=				(uint64_t)(SECONDS_PER_HOUR * HOURS_PER_DAY * (uint64_t)dt.day);									//dni
	secTotal +=				(uint64_t)(SECONDS_PER_HOUR * HOURS_PER_DAY * (uint64_t)rtc_countDaysInMonths(dt.mon, dt.year));	//mesiace
	secTotal +=				(uint64_t)(SECONDS_PER_HOUR * HOURS_PER_DAY * DAYS_PER_YEAR * (uint64_t)(dt.year - 1)); 			//roky
	secTotal +=				(uint64_t)(SECONDS_PER_HOUR * HOURS_PER_DAY * (uint64_t)rtc_extraLeapYearDays(dt.year));			//extra dni za prestupne roky

    if(dt.ms >= 1000 )    //in some cases it can happen, see RTC_SSR description -> note in reference manual
    {
        secTotal --;
        dt.sec--;
    }

    uint64_t msTotal = secTotal * 1000 + dt.ms;
    return msTotal;
}


/*
 * Reset RSF, po zresetovani sa do 2 tikov RTC aktualizuje obsah shadow registra
*/
void rtc_ClearRSF(void)
{
    LL_RTC_DisableWriteProtection(RTC);
    RTC->ISR &= ~RTC_ISR_RSF; ; // reset RSF - required after stop mode, sleep mode or reset
    LL_RTC_EnableWriteProtection(RTC);
}


void rtc_registerCallback(void *callback)
{
	sCallbackRTC = callback;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

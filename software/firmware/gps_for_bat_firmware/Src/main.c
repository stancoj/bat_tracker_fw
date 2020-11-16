/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>

#include "main.h"
#include "mcu.h"
#include "gps.h"
#include "lis2dh.h"
#include "bmp280.h"

/* Structures containing GPS data */
extern Gps_Data_s gGpsData;
extern Gps_Data_Ubx_s gGpsDataUbx;

void SystemClock_Config(void);
void sendGPStoUsart(void);

uint64_t time = 0, time_init = 0, time_first_fix = 0;

int main(void)
{
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  SystemClock_Config();

  mcu_init();

  if(InitBMP280())
  {

  }
  else
  {
	  while(1);
  }

  InitGps();
  time_init = time;

  uint64_t prev_time = 0;

  while (1)
  {
	  if((time - prev_time) > 1000)
	  {
		  sendGPStoUsart();
		  prev_time = time;
	  }

	  if((gGpsData.gpsValid == 1) && (!time_first_fix)) time_first_fix = time;

	  USART2_CheckDmaReception();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {
    
  }
  LL_PWR_EnableBkUpAccess();
  LL_RCC_ForceBackupDomainReset();
  LL_RCC_ReleaseBackupDomainReset();
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  LL_RCC_EnableRTC();
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  
  }
  LL_Init1msTick(16000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SYSTICK_EnableIT();
  LL_SetSystemCoreClock(16000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);

}


void sendGPStoUsart(void)
{
	char text[50];

	memset(text, '\0', sizeof(text));
	sprintf(text, "sysTime: %ld, initTime: %ld, fixTime: %ld", (uint32_t)(time/1000), (uint32_t)(time_init/1000), (uint32_t)(time_first_fix/1000));

	uint8_t len = strlen(text);
	for(uint8_t i = 0; i < len; i++)
	{
		  LL_USART_TransmitData8(USART1, text[i]);
		  while(!LL_USART_IsActiveFlag_TC(USART1)){};
		  LL_USART_ClearFlag_TC(USART1);
	}

	memset(text, '\0', sizeof(text));
	sprintf(text, "satTracked: %d, ", gGpsData.satTracked);
	for(uint8_t i = 0; i < len; i++)
	{
		  LL_USART_TransmitData8(USART1, text[i]);
		  while(!LL_USART_IsActiveFlag_TC(USART1)){};
		  LL_USART_ClearFlag_TC(USART1);
	}

	memset(text, '\0', sizeof(text));
	sprintf(text, "gpsValid: %d, ", gGpsData.gpsValid);
	for(uint8_t i = 0; i < strlen(text); i++)
	{
		  LL_USART_TransmitData8(USART1, text[i]);
		  while(!LL_USART_IsActiveFlag_TC(USART1)){};
		  LL_USART_ClearFlag_TC(USART1);
	}

	memset(text, '\0', sizeof(text));
	sprintf(text, "timeValid: %d, ", gGpsData.timeValid);
	for(uint8_t i = 0; i < strlen(text); i++)
	{
		  LL_USART_TransmitData8(USART1, text[i]);
		  while(!LL_USART_IsActiveFlag_TC(USART1)){};
		  LL_USART_ClearFlag_TC(USART1);
	}

	memset(text, '\0', sizeof(text));
	sprintf(text, "e2D3Dfix: %c, ", (int)gGpsData.e2D3Dfix);
	for(uint8_t i = 0; i < strlen(text); i++)
	{
		  LL_USART_TransmitData8(USART1, text[i]);
		  while(!LL_USART_IsActiveFlag_TC(USART1)){};
		  LL_USART_ClearFlag_TC(USART1);
	}

	memset(text, '\0', sizeof(text));
	sprintf(text, "gpsTime: %d:%d:%d \n \r", gGpsData.time.hour, gGpsData.time.min, gGpsData.time.sec);
	for(uint8_t i = 0; i < strlen(text); i++)
	{
		  LL_USART_TransmitData8(USART1, text[i]);
		  while(!LL_USART_IsActiveFlag_TC(USART1)){};
		  LL_USART_ClearFlag_TC(USART1);
	}
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
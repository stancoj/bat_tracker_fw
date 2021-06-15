/*
 * lpmode.c
 *
 *  Created on: 1. 2. 2018
 *      Author: Stancoj
 */

#include <lpmode.h>


void enter_LP_Sleep(uint8_t sleep_on_exit)
{
	/* Ensure that MSI is wake-up system clock */
#ifdef HSI
	LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_HSI);
#else
	LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_MSI);
#endif
	/* Enable sleep low power mode */
	LL_LPM_EnableSleep();
	/* Enable Sleep after ISR */
	if(sleep_on_exit) LL_LPM_EnableSleepOnExit();
	/* Ensure Flash memory stays on */
	FLASH->ACR &= ~FLASH_ACR_SLEEP_PD;

	/* Request Wait For Interrupt */
	__WFI();
}

void enter_LP_Stop(uint8_t sleep_on_exit)
{
	  //testovacie ucely
	  //LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_15);

	  /* Ensure that MSI is wake-up system clock */
#ifdef HSI
	  LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_HSI);
#else
	  LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_MSI);
#endif

	  /* Enable ultra low power mode */
	  LL_PWR_EnableUltraLowPower();

	  /*Enable fast wake up from ultra low power mode*/
	  LL_PWR_EnableFastWakeUp();

	  /** Set the regulator to low power before setting MODE_STOP.
	    * If the regulator remains in "main mode",
	    * it consumes more power without providing any additional feature. */
	  LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_LOW_POWER);

	  /*Enable SleepOnExit*/
	  if(sleep_on_exit) LL_LPM_EnableSleepOnExit();

	  /* Set STOP mode when CPU enters deepsleep */
	  LL_PWR_SetPowerMode(LL_PWR_MODE_STOP);

	  /* Set SLEEPDEEP bit of Cortex System Control Register */
	  LL_LPM_EnableDeepSleep();

	  /* Request Wait For Interrupt */
	  __WFI();
}

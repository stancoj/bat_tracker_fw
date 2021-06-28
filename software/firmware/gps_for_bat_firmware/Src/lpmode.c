/*
 * lpmode.c
 *
 *  Created on: 1. 2. 2018
 *      Author: Stancoj
 */

#include <lpmode.h>


void enter_LP_Sleep(uint8_t sleep_on_exit)
{
	/*
	// Ensure that MSI is wake-up system clock
#ifdef HSI
	//LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_HSI);
	LL_RCC_SetClkAfterWakeFromStop();
#else
	LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_MSI);
#endif
	// Enable sleep low power mode
	LL_LPM_EnableSleep();
	// Enable Sleep after ISR
	if(sleep_on_exit) LL_LPM_EnableSleepOnExit();
	// Ensure Flash memory stays on
	FLASH->ACR &= ~FLASH_ACR_SLEEP_PD;

	// Request Wait For Interrupt
	__WFI();
	*/
}

void enter_LP_Stop(void)
{
	/* Backup systick setup */
	uint32_t systick_reg = SysTick->CTRL;
	/* Disable the SysTick */
	SysTick->CTRL  = 0;
	/* Clear any pending EXTI */
	EXTI->PR = 0xFFFFFFFF;
	/* Clear any pending peripheral interrupts */
	for (int i = 0; i < 8; i++)
	{
	  NVIC->ICPR[i] = 0xFFFFFFFF;
	}
	/* Clear the RTC pending flags */
	RTC->ISR &= ~(RTC_ISR_ALRAF | RTC_ISR_ALRBF | RTC_ISR_WUTF | RTC_ISR_TSF | RTC_ISR_TAMP1F);

	SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP_Pos);
	PWR->CR &= ~(PWR_CR_PDDS);
	PWR->CR |= PWR_CR_LPDS;

	__WFI();

	/* Setup the systick */
	SysTick->CTRL = systick_reg;
}


void enter_LP_Standby(void)
{
	/* Disable the SysTick */
	//SysTick->CTRL  = 0;
	/* Clear any pending EXTI */
	EXTI->PR = 0xFFFFFFFF;
	/* Clear any pending peripheral interrupts */
	for (int i = 0; i < 8; i++)
	{
	  NVIC->ICPR[i] = 0xFFFFFFFF;
	}
	/* Clear the RTC pending flags */
	RTC->ISR &= ~(RTC_ISR_ALRAF | RTC_ISR_ALRBF | RTC_ISR_WUTF | RTC_ISR_TSF | RTC_ISR_TAMP1F);

	//SCB->SCR |= (1 << SCB_SCR_SLEEPDEEP_Pos);
	//PWR->CR |= PWR_CR_PDDS;
	//PWR->CR |= PWR_CR_LPDS;

	HAL_PWR_EnterSTANDBYMode();

	__WFI();
}

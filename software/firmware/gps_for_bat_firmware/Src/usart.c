/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN*/
#define USART2_BAUDRATE_BASE			9600
#define USART2_BAUDRATE_USER			9600

uint32_t baud_rate_table[7] = {9600, 19200, 38400, 57600, 115200, 230400, 460800};

// USART2 for GPS
#define DMA_USART2_BUFFER_SIZE 1024					//in order to store long NMEA data with fast sampling between sampline
uint8_t bufferUSART2dma[DMA_USART2_BUFFER_SIZE];	//and proceed them in once
uint8_t aTxBuffer[1];

// USART 1 for app
#define DMA_USART1_BUFFER_SIZE 32
uint8_t bufferUSART1dma[DMA_USART1_BUFFER_SIZE];

static void (* usart2Callback)(uint8_t) = 0;
static void (* usart2CallbackWakeUp)(void) = 0;

static void (* usart1Callback)(uint8_t) = 0;

static uint8_t gAfterTransmitUpdateBaudRate = 0;

uint8_t DMA2_USART1_Tx_flag = 0;


/* USART1 init function */
void MX_USART1_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */

  // USART1_RX Init
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);
  LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_2,
                         LL_USART_DMA_GetRegAddr(USART1),
                         (uint32_t)bufferUSART1dma,
                         LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_2));
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, DMA_USART1_BUFFER_SIZE);

  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);
  LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_2);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_2);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);

  // USART1_TX Init
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4);
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_7);

  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_7, LL_USART_DMA_GetRegAddr(USART1));
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_STREAM_7);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);

  LL_USART_EnableDMAReq_RX(USART1);
  LL_USART_EnableDMAReq_TX(USART1);
  LL_USART_EnableIT_IDLE(USART1);

  LL_USART_Enable(USART1);

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration  
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */

  // USART2_RX Init
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_4);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_5,
                         LL_USART_DMA_GetRegAddr(USART2),
                         (uint32_t)bufferUSART2dma,
                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_5));
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5, DMA_USART2_BUFFER_SIZE);

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);

  // USART2_TX Init
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_4);
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_LOW);
  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);
  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);
  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);
  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);
  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_6, LL_USART_DMA_GetRegAddr(USART2));
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_6);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);

  LL_USART_EnableDMAReq_RX(USART2);
  LL_USART_EnableDMAReq_TX(USART2);
  LL_USART_EnableIT_IDLE(USART2);

  LL_USART_Enable(USART2);
}

void USART2_PutBuffer(uint8_t *buffer, uint8_t length)
{
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);

	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)buffer);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_6, length);

	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
}

void USART2_CheckDmaReception(void)
{
	static uint16_t memTargetPrev = 0;

	if(usart2Callback == 0) return;

	uint16_t memTarget = DMA_USART2_BUFFER_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5); //DMA_GetCurrDataCounter(DMA1_Channel5);

	uint16_t i;
	for(i = memTargetPrev; i != memTarget; i=(i+1) & (DMA_USART2_BUFFER_SIZE - 1))
	{
		usart2Callback(bufferUSART2dma[i]);
	}
	memTargetPrev = memTarget;
}

void USART2_UpdateBaudRate(uint32_t baudRate)
{
	LL_USART_Disable(USART2);

	LL_USART_SetBaudRate(USART2, 16000000/*LL_RCC_GetUSARTClockSource(LL_RCC_USART1_CLKSOURCE)*/, LL_USART_GetOverSampling(USART2) ,baudRate);

	LL_USART_Enable(USART2);
}

void USART2_ChangeBaudRate(void)
{
	static uint8_t baud_rate_index = 1;

	USART2_UpdateBaudRate(baud_rate_table[baud_rate_index]);

	baud_rate_index++;
	if(baud_rate_index > 6) baud_rate_index = 0;
}

void USART2_PrepareBaudRate(void)
{
	gAfterTransmitUpdateBaudRate = 1;
}

void USART2_RegisterCallback(void *callback)
{
	usart2Callback = callback;
}

void USART2_RegisterCallbackWakeUp(void *callback)
{
	 usart2CallbackWakeUp = callback;
}

void USART1_PutBuffer(uint8_t *buffer, uint8_t length)
{
	LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);

	LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_7, (uint32_t)buffer);
	LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_7, length);

	LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_7);
	LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_7);
}

void USART1_CheckDmaReception(void)
{
	static uint16_t memTargetPrev = 0;

	if(usart1Callback == 0) return;

	uint16_t memTarget = DMA_USART1_BUFFER_SIZE - LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_2);

	uint16_t i;
	for(i = memTargetPrev; i != memTarget; i=(i+1) & (DMA_USART1_BUFFER_SIZE - 1))
	{
		usart1Callback(bufferUSART1dma[i]);
	}
	memTargetPrev = memTarget;
}

void USART1_RegisterCallback(void *callback)
{
	usart1Callback = callback;
}

/*
 * USART and USART/DMA interrupts
 */

void USART2_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(USART2))
	{
		USART2_CheckDmaReception();
		LL_USART_ClearFlag_IDLE(USART2);
	}
}

// DMA1 USART2 Tx
void DMA1_Stream6_IRQHandler(void)
{
	if(LL_DMA_IsActiveFlag_TC6(DMA1) == SET)
	{
		LL_DMA_ClearFlag_TC6(DMA1);
		while(LL_USART_IsActiveFlag_TC(USART2) == RESET);
		LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_6);
	}
}

// DMA1 USART2 Rx
void DMA1_Stream5_IRQHandler(void)
{
	if(LL_DMA_IsActiveFlag_HT5(DMA1) == SET)
	{
		USART2_CheckDmaReception();
		LL_DMA_ClearFlag_HT5(DMA1);
	}
	else if(LL_DMA_IsActiveFlag_TC5(DMA1) == SET)
	{
		USART2_CheckDmaReception();
		LL_DMA_ClearFlag_TC5(DMA1);
	}
}

// USART1 IDLE interrupt
void USART1_IRQHandler(void)
{
	if(LL_USART_IsActiveFlag_IDLE(USART1))
	{
		USART1_CheckDmaReception();
		LL_USART_ClearFlag_IDLE(USART1);
	}
}

// DMA2 USART1 Tx
void DMA2_Stream7_IRQHandler(void)
{
	if(LL_DMA_IsActiveFlag_TC7(DMA2) == SET)
	{
		LL_DMA_ClearFlag_TC7(DMA2);
		while(LL_USART_IsActiveFlag_TC(USART1) == RESET);
		LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_7);
	}
}

// DMA2 USART1 Rx
void DMA2_Stream2_IRQHandler(void)
{
	if(LL_DMA_IsActiveFlag_HT2(DMA2) == SET)
	{
		USART1_CheckDmaReception();
		LL_DMA_ClearFlag_HT2(DMA2);
	}
	else if(LL_DMA_IsActiveFlag_TC2(DMA2) == SET)
	{
		USART1_CheckDmaReception();
		LL_DMA_ClearFlag_TC2(DMA2);
	}
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

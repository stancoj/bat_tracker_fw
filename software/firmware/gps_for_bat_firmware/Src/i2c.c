/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* I2C3 init function */
void MX_I2C3_Init(void)
{
  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C3 GPIO Configuration  
  PA8   ------> I2C3_SCL
  PB4   ------> I2C3_SDA 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_9;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);

  /* I2C3 interrupt Init */
  NVIC_SetPriority(I2C3_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(I2C3_EV_IRQn);

  /** I2C Initialization 
  */
  LL_I2C_DisableOwnAddress2(I2C3);
  LL_I2C_DisableGeneralCall(I2C3);
  LL_I2C_EnableClockStretching(I2C3);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C3, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C3, 0);

  LL_I2C_Enable(I2C3);

}


uint8_t i2c_master_write(uint8_t register_addr, uint8_t slave_addr, uint8_t *data, uint8_t send_data_length)
{
	if(data == 0 || send_data_length == 0)
	{
		return 0;
	}

	LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_ACK);
	LL_I2C_GenerateStartCondition(I2C3);
	while(!LL_I2C_IsActiveFlag_SB(I2C3)){};
	LL_I2C_TransmitData8(I2C3, slave_addr);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C3)){};
	LL_I2C_ClearFlag_ADDR(I2C3);

	/* Send register address */
	LL_I2C_TransmitData8(I2C3, register_addr);
	while(!LL_I2C_IsActiveFlag_TXE(I2C3)){};

	for(uint8_t i = send_data_length; i > 0; i-- )
	{
		if(i == 1)
		{
			/* Send byte and wait for ACK */
			LL_I2C_TransmitData8(I2C3, data[i-1]);
			while(!LL_I2C_IsActiveFlag_TXE(I2C3)){};
			/* Generate Stop condition */
			LL_I2C_GenerateStopCondition(I2C3);
			LL_I2C_ClearFlag_STOP(I2C3);
		}
		else if(i > 1)
		{
			LL_I2C_TransmitData8(I2C3, data[i-1]);
			while(!LL_I2C_IsActiveFlag_TXE(I2C3)){};
		}
	}

	return 1;
}


uint8_t i2c_master_read(uint8_t register_addr, uint8_t slave_addr, uint8_t *data_rx_buffer, uint8_t read_data_length)
{
	/* Multibyte data transfer: slave => master */
	if(read_data_length > 0 && data_rx_buffer != 0)
	{
		/* Save number of bytes to be received */
		uint8_t data_to_receive = read_data_length;
		uint8_t buffer_iterator = 0;

		/**** Tx part ****/
		/* Prepare acknowledge for Master data reception */
		LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_ACK);
		/* Master Generate Start condition */
		LL_I2C_GenerateStartCondition(I2C3);
		/* Check SB flag value in ISR register */
		while(!LL_I2C_IsActiveFlag_SB(I2C3)){};
		LL_I2C_TransmitData8(I2C3, slave_addr);
		while(!LL_I2C_IsActiveFlag_ADDR(I2C3)){};
		LL_I2C_ClearFlag_ADDR(I2C3);
		/* Send register address we want to read from*/
		while(!LL_I2C_IsActiveFlag_TXE(I2C3)){};
		LL_I2C_TransmitData8(I2C3, register_addr);

		/**** Rx part ****/
		/* Enable I2C interrupts */
		LL_I2C_EnableIT_BUF(I2C3);
		/* Prepare acknowledge for Master data reception */
		if(read_data_length == 1)
		{
			LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_NACK);
		}
		else
		{
			LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_ACK);
		}
		/* Generate start condition */
		LL_I2C_GenerateStartCondition(I2C3);

		while(!LL_I2C_IsActiveFlag_SB(I2C3)){};
		// Send slave address with request to read data
		LL_I2C_TransmitData8(I2C3, (slave_addr | 0x01));
		// Wait for slave ACK
		while(!LL_I2C_IsActiveFlag_ADDR(I2C3)){};
		LL_I2C_ClearFlag_ADDR(I2C3);

		/* Wait for end of the reception */
		while(data_to_receive > 0)
		{
			if(LL_I2C_IsActiveFlag_RXNE(I2C3))
			{
				if((data_to_receive > 2) || (data_to_receive == 1))
				{
					data_rx_buffer[buffer_iterator++] = LL_I2C_ReceiveData8(I2C3);
					data_to_receive--;
				}
				else if(data_to_receive == 2)
				{
					LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_NACK);

					data_rx_buffer[buffer_iterator++] = LL_I2C_ReceiveData8(I2C3);
					data_to_receive--;
				}
			}
		};

		/* Generate Stop condition */
		LL_I2C_GenerateStopCondition(I2C3);
		LL_I2C_ClearFlag_STOP(I2C3);

		buffer_iterator = 0;

		return 1;
	}
	else
	{
		return 0;
	}


}


int8_t i2c_bmp280_write(uint8_t slave_addr, uint8_t register_addr, uint8_t *data, uint16_t send_data_length)
{
	if(data == 0 || send_data_length == 0)
	{
		return 1;
	}

	__set_BASEPRI(1 << 4);

	LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_ACK);
	LL_I2C_GenerateStartCondition(I2C3);
	while(!LL_I2C_IsActiveFlag_SB(I2C3)){};
	LL_I2C_TransmitData8(I2C3, (slave_addr));
	while(!LL_I2C_IsActiveFlag_ADDR(I2C3)){};
	LL_I2C_ClearFlag_ADDR(I2C3);

	/* Send register address */
	LL_I2C_TransmitData8(I2C3, register_addr);
	while(!LL_I2C_IsActiveFlag_TXE(I2C3)){};

	for(uint8_t i = send_data_length; i > 0; i-- )
	{
		if(i == 1)
		{
			/* Send byte and wait for ACK */
			LL_I2C_TransmitData8(I2C3, data[i-1]);
			while(!LL_I2C_IsActiveFlag_TXE(I2C3)){};
			/* Generate Stop condition */
			LL_I2C_GenerateStopCondition(I2C3);
			LL_I2C_ClearFlag_STOP(I2C3);
		}
		else if(i > 1)
		{
			LL_I2C_TransmitData8(I2C3, data[i-1]);
			while(!LL_I2C_IsActiveFlag_TXE(I2C3)){};
		}
	}

	__set_BASEPRI(0);
	return 0;
}


int8_t i2c_bmp280_read(uint8_t slave_addr, uint8_t register_addr, uint8_t *data_rx_buffer, uint16_t read_data_length)
{
	/* Multibyte data transfer: slave => master */
	if(read_data_length > 0 && data_rx_buffer != 0)
	{
		__set_BASEPRI(1 << 4);

		/* Save number of bytes to be received */
		uint8_t data_to_receive = read_data_length;
		uint8_t buffer_iterator = 0;

		/**** Tx part ****/
		/* Prepare acknowledge for Master data reception */
		LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_ACK);
		/* Master Generate Start condition */
		LL_I2C_GenerateStartCondition(I2C3);
		/* Check SB flag value in ISR register */
		while(!LL_I2C_IsActiveFlag_SB(I2C3)){};
		LL_I2C_TransmitData8(I2C3, (slave_addr));
		while(!LL_I2C_IsActiveFlag_ADDR(I2C3)){};
		LL_I2C_ClearFlag_ADDR(I2C3);
		/* Send register address we want to read from*/
		while(!LL_I2C_IsActiveFlag_TXE(I2C3)){};
		LL_I2C_TransmitData8(I2C3, register_addr);

		/**** Rx part ****/
		/* Enable I2C interrupts */
		LL_I2C_EnableIT_BUF(I2C3);
		/* Prepare acknowledge for Master data reception */
		if(read_data_length == 1)
		{
			LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_NACK);
		}
		else
		{
			LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_ACK);
		}
		/* Generate start condition */
		LL_I2C_GenerateStartCondition(I2C3);

		while(!LL_I2C_IsActiveFlag_SB(I2C3)){};
		// Send slave address with request to read data
		LL_I2C_TransmitData8(I2C3, ((slave_addr) | 0x01));
		// Wait for slave ACK
		while(!LL_I2C_IsActiveFlag_ADDR(I2C3)){};
		LL_I2C_ClearFlag_ADDR(I2C3);

		/* Wait for end of the reception */
		while(data_to_receive > 0)
		{
			if(LL_I2C_IsActiveFlag_RXNE(I2C3))
			{
				if((data_to_receive > 2) || (data_to_receive == 1))
				{
					data_rx_buffer[buffer_iterator++] = LL_I2C_ReceiveData8(I2C3);
					data_to_receive--;
				}
				else if(data_to_receive == 2)
				{
					LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_NACK);

					data_rx_buffer[buffer_iterator++] = LL_I2C_ReceiveData8(I2C3);
					data_to_receive--;
				}
			}
		};

		/* Generate Stop condition */
		LL_I2C_GenerateStopCondition(I2C3);
		LL_I2C_ClearFlag_STOP(I2C3);

		buffer_iterator = 0;

		__set_BASEPRI(0);
		return 0;
	}
	else
	{
		return 1;
	}
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

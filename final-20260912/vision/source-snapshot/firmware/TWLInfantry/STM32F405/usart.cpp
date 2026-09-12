/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  /* USER CODE END Header */
  /* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "stm32f4xx_hal_gpio.h"
#include "label.h"
#include "queue.h"

UART& UART::Init(USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart.Instance = Instance;								//设置串口通道
	huart.Init.BaudRate = BaudRate;							//设置波特率
	huart.Init.WordLength = UART_WORDLENGTH_8B;				//传输数据字长，8位
	huart.Init.StopBits = UART_STOPBITS_1;					//停止位字长，1位
	huart.Init.Parity = UART_PARITY_NONE;					//无奇偶效验位
	huart.Init.Mode = UART_MODE_TX_RX;						//收发模式
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;				//无硬件流
	huart.Init.OverSampling = UART_OVERSAMPLING_16;			//16倍过采样，去除干扰
	HAL_UART_Init(&huart);									//调用HAL库初始化函数
	return *this;
}

UART& UART::DMATxInit(void)
{
	__HAL_DMA_DISABLE(huart.hdmatx);						//清除串口DMA发送
	huart.hdmatx->Instance->PAR = reinterpret_cast<uint32_t>(&huart.Instance->DR);//写入DMA外设地址
	huart.hdmatx->Instance->M0AR = 0;						//0号存储器
	DMAClearAllFlags(huart.hdmatx);
	__HAL_DMA_ENABLE_IT(huart.hdmatx, DMA_IT_TC);			//开启串口DMA发送
	SET_BIT(huart.Instance->CR3, USART_CR3_DMAT);			//设置控制器3，使能DMA
	return *this;
}


UART& UART::DMARxInit(const uint8_t* buffer, const uint32_t size)//size = UART_MAX_LEN
{
	__HAL_DMA_DISABLE(huart.hdmarx);						//失能串口DMA接收
	huart.hdmarx->Instance->PAR = reinterpret_cast<uint32_t>(&huart.Instance->DR);
	//PAR is DMA stream x peripheral address register,DR is UART Date register address
	huart.hdmarx->Instance->NDTR = size;					//设置串口数据长度
	if (buffer == nullptr)									//设置缓冲区
		buffer = m_uartrx;
	huart.hdmarx->Instance->M0AR = reinterpret_cast<uint32_t>(buffer);//设置存储器0地址
	DMAClearAllFlags(huart.hdmarx);
	__HAL_DMA_ENABLE(huart.hdmarx);							//使能串口DMA接收	
	SET_BIT(huart.Instance->CR3, USART_CR3_DMAR);

	__HAL_UART_CLEAR_PEFLAG(&huart);						//开启串口空闲中断
	__HAL_UART_ENABLE_IT(&huart, UART_IT_IDLE);
	return *this;
}

/*
 * @brief		串口相关外设初始化
 * @param		*uartHandle		:	串口句柄
*/
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if (uartHandle->Instance == USART1)
	{
		static DMA_HandleTypeDef hdma_rx;
		static DMA_HandleTypeDef hdma_tx;
		/* USART6 clock enable */
		__HAL_RCC_USART1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/**USART6 GPIO Configuration
		PG14     ------> USART6_TX
		PG9     ------> USART6_RX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;//推挽复用功能
		GPIO_InitStruct.Pull = GPIO_PULLUP;//上拉
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USART6 DMA Init */
		__HAL_RCC_DMA2_CLK_ENABLE();
		/* USART6_RX Init */
		hdma_rx.Instance = DMA2_Stream2;
		hdma_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode = DMA_NORMAL;
		hdma_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_rx);

		__HAL_LINKDMA(uartHandle, hdmarx, hdma_rx);

		/* USART6_TX Init */
		hdma_tx.Instance = DMA2_Stream7;
		hdma_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode = DMA_NORMAL;
		hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_tx);

		__HAL_LINKDMA(uartHandle, hdmatx, hdma_tx);

		/* USART6 interrupt Init */
		HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART1_IRQn);

		/* DMA interrupt init */
		/* DMA2_Stream1_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		/* DMA2_Stream6_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	}
	if (uartHandle->Instance == USART2)
	{
		static DMA_HandleTypeDef hdma_rx;
		static DMA_HandleTypeDef hdma_tx;
		/* USART6 clock enable */
		__HAL_RCC_USART2_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/**USART6 GPIO Configuration
		PG14     ------> USART6_TX
		PG9     ------> USART6_RX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USART6 DMA Init */
		__HAL_RCC_DMA1_CLK_ENABLE();
		/* USART6_RX Init */
		hdma_rx.Instance = DMA1_Stream5;
		hdma_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode = DMA_NORMAL;
		hdma_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_rx);

		__HAL_LINKDMA(uartHandle, hdmarx, hdma_rx);

		/* USART6_TX Init */
		hdma_tx.Instance = DMA1_Stream6;
		hdma_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode = DMA_NORMAL;
		hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_tx);

		__HAL_LINKDMA(uartHandle, hdmatx, hdma_tx);

		/* USART6 interrupt Init */
		HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);//中断使能，接收到8位数据就中断一次

		/* DMA interrupt init */
		/* DMA2_Stream1_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
		/* DMA2_Stream6_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	}
	if (uartHandle->Instance == USART3)
	{
		static DMA_HandleTypeDef hdma_rx;
		static DMA_HandleTypeDef hdma_tx;
		/* USART2 clock enable */
		__HAL_RCC_USART3_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();

		/**USART6 GPIO Configuration
		PG14     ------> USART2_TX
		PG9     ------> USART2_RX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* USART2 DMA Init */
		__HAL_RCC_DMA1_CLK_ENABLE();
		/* USART2_RX Init */
		hdma_rx.Instance = DMA1_Stream1;
		hdma_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode = DMA_NORMAL;
		hdma_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_rx);

		__HAL_LINKDMA(uartHandle, hdmarx, hdma_rx);

		/* USART2_TX Init */
		hdma_tx.Instance = DMA1_Stream3;
		hdma_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode = DMA_NORMAL;
		hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_tx);

		__HAL_LINKDMA(uartHandle, hdmatx, hdma_tx);

		/* USART2 interrupt Init */
		HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);

		/* DMA interrupt init */
		/* DMA2_Stream1_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
		/* DMA2_Stream6_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	}
	if (uartHandle->Instance == UART4)
	{
		static DMA_HandleTypeDef hdma_rx;
		static DMA_HandleTypeDef hdma_tx;
		/* USART1 clock enable */
		__HAL_RCC_UART4_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/**USART1 GPIO Configuration
		PA0     ------> UART4_TX
		PA1     ------> UART4_RX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USART1 DMA Init */
		__HAL_RCC_DMA1_CLK_ENABLE();
		/* USART1_RX Init */
		hdma_rx.Instance = DMA1_Stream2;
		hdma_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode = DMA_NORMAL;
		hdma_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_rx);

		__HAL_LINKDMA(uartHandle, hdmarx, hdma_rx);
		/* USART1_TX Init */
		hdma_tx.Instance = DMA1_Stream4;
		hdma_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode = DMA_NORMAL;
		hdma_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_tx);
		__HAL_LINKDMA(uartHandle, hdmatx, hdma_tx);

		/* USART1 interrupt Init */
		HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(UART4_IRQn);

		/* DMA interrupt init */
		/* DMA2_Stream2_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
		/* DMA2_Stream7_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	}
	if (uartHandle->Instance == UART5)
	{
		static DMA_HandleTypeDef hdma_rx;
		static DMA_HandleTypeDef hdma_tx;

		__HAL_RCC_UART5_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitTypeDef GPIO_InitStruct2;
		GPIO_InitStruct2.Pin = GPIO_PIN_2;
		GPIO_InitStruct2.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct2.Pull = GPIO_PULLUP;
		GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct2.Alternate = GPIO_AF8_UART5;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct2);
		/* USART2 DMA Init */
		__HAL_RCC_DMA1_CLK_ENABLE();
		/* USART2_RX Init */
		hdma_rx.Instance = DMA1_Stream0;
		hdma_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode = DMA_NORMAL;
		hdma_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_rx);
		__HAL_LINKDMA(uartHandle, hdmarx, hdma_rx);
		/* USART2_TX Init */
		hdma_tx.Instance = DMA1_Stream7;
		hdma_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode = DMA_NORMAL;
		hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_tx);
		__HAL_LINKDMA(uartHandle, hdmatx, hdma_tx);
		/* USART2 interrupt Init */
		HAL_NVIC_SetPriority(UART5_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(UART5_IRQn);
		/* DMA interrupt init */
		/* DMA2_Stream1_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
		/* DMA2_Stream6_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	}
	if (uartHandle->Instance == USART6)
	{
		static DMA_HandleTypeDef hdma_rx;
		static DMA_HandleTypeDef hdma_tx;
		/* USART1 clock enable */
		__HAL_RCC_USART6_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();

		/**USART1 GPIO Configuration
		PC6     ------> USART6_TX
		PC7     ------> USART6_RX
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* USART6 DMA Init */
		__HAL_RCC_DMA2_CLK_ENABLE();
		/* USART6_RX Init */
		hdma_rx.Instance = DMA2_Stream1;
		hdma_rx.Init.Channel = DMA_CHANNEL_5;
		hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_rx.Init.Mode = DMA_NORMAL;
		hdma_rx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_rx);

		__HAL_LINKDMA(uartHandle, hdmarx, hdma_rx);
		/* USART6_TX Init */
		hdma_tx.Instance = DMA2_Stream6;
		hdma_tx.Init.Channel = DMA_CHANNEL_5;
		hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode = DMA_NORMAL;
		hdma_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&hdma_tx);
		__HAL_LINKDMA(uartHandle, hdmatx, hdma_tx);

		/* USART1 interrupt Init */
		HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(USART6_IRQn);

		/* DMA interrupt init */
		/* DMA2_Stream2_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
		/* DMA2_Stream7_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
	}
}

void UART::OnUARTITHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart);
		__HAL_DMA_DISABLE(huart.hdmarx);

		pd_Rx = xQueueOverwriteFromISR((QueueHandle_t)UartQueueHandler, m_uartrx, NULL);

		DMAClearAllFlags(huart.hdmarx);

		__HAL_DMA_SET_COUNTER(huart.hdmarx, UART_MAX_LEN);
		__HAL_DMA_ENABLE(huart.hdmarx);
	}
	HAL_UART_IRQHandler(&huart);
}

void UART::OnDMAITHandler(void)
{
	UART::DMAClearAllFlags(huart.hdmatx);
	__HAL_DMA_DISABLE(huart.hdmatx);
}

extern "C" void USART1_IRQHandler(void)
{
	uart1.OnUARTITHandler();
}
extern "C" void USART2_IRQHandler(void)
{
	uart2.OnUARTITHandler();
}
extern "C" void USART3_IRQHandler(void)
{
	uart3.OnUARTITHandler();
}
extern "C" void UART4_IRQHandler(void)
{
	uart4.OnUARTITHandler();
}
extern "C" void UART5_IRQHandler(void)
{
	uart5.OnUARTITHandler();
}
extern "C" void USART6_IRQHandler(void)
{
	uart6.OnUARTITHandler();
}
extern "C" void DMA1_Stream2_IRQHandler(void)
{
}
extern "C" void DMA1_Stream4_IRQHandler(void)//发送中断
{
	uart4.OnDMAITHandler();
}

void UART::DMATransmit(uint8_t* const pData, const uint32_t Size)
{
	__HAL_DMA_DISABLE(huart.hdmatx);
	huart.hdmatx->Instance->NDTR = Size;
	huart.hdmatx->Instance->M0AR = reinterpret_cast<uint32_t>(pData);
	DMAClearAllFlags(huart.hdmatx);
	__HAL_DMA_ENABLE(huart.hdmatx);
}


void UART::UARTTransmit(uint8_t* pData, uint32_t Size)
{
	HAL_UART_Transmit(&huart, static_cast<uint8_t*>(pData), Size, 0xffff);
}



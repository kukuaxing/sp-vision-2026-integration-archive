#pragma once

#include "stm32f4xx_hal.h"

#define __USART_H__
#define UART_MAX_LEN 100

typedef int BaseType_t;
typedef void* QueueHandle_t;

static const BaseType_t pdTRUE = 1;

inline BaseType_t xQueueReceive(QueueHandle_t, void*, uint32_t)
{
	return 0;
}

class UART
{
public:
	UART& Init(USART_TypeDef*, uint32_t) { return *this; }
	UART& DMARxInit(const uint8_t* = nullptr, uint32_t = UART_MAX_LEN) { return *this; }
	void UARTTransmit(uint8_t*, uint32_t) {}

	QueueHandle_t UartQueueHandler = nullptr;
	uint32_t dataDmaNum = 0U;
};

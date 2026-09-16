#pragma once
#include "stm32f4xx_hal.h"

typedef struct
{
	GPIO_TypeDef* GPIOx;
	uint32_t Pin;
}gpio;
void GPIO_CLK_ENABLE(GPIO_TypeDef* GPIOx);
void GPIO_Init(GPIO_TypeDef* GPIOx, uint32_t Mode, uint32_t Pull, uint16_t Pin);
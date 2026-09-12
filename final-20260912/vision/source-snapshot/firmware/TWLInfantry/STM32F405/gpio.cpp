#include "gpio.h"

void GPIO_CLK_ENABLE(GPIO_TypeDef* GPIOx)
{
	__IO uint32_t tmpreg = 0x00U;
	SET_BIT(RCC->AHB1ENR, 0x1U << ((((unsigned)GPIOx - AHB1PERIPH_BASE)) / 0x400U));
	/* Delay after an RCC peripheral clock enabling */
	tmpreg = READ_BIT(RCC->AHB1ENR, 0x1U << ((((unsigned)GPIOx - AHB1PERIPH_BASE)) / 0x400U));
	UNUSED(tmpreg);
}

void GPIO_Init(GPIO_TypeDef* GPIOx, uint32_t Mode, uint32_t Pull, uint16_t Pin)
{
	GPIO_CLK_ENABLE(GPIOx);
	GPIO_InitTypeDef GPIO_Initure;
	GPIO_Initure.Pin = Pin;
	GPIO_Initure.Mode = Mode;
	GPIO_Initure.Pull = Pull;
	GPIO_Initure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_Initure);
}

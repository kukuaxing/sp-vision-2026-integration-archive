#pragma once
#include "stm32f4xx_hal.h"
#include "gpio.h"

enum { BASE, PWM, IC };

class TIM
{
public:
	TIM& Init(uint32_t Mode, TIM_TypeDef* TIM, uint32_t frequency);
	void MspPostInit(gpio r) const;
	void BaseInit(void);
	TIM& PWMInit(uint32_t channel, float duty, gpio r);
	void PWMDuty(uint32_t channel, float duty) const;
	void ICInit(int32_t channel, uint32_t ICPolarity, gpio r);

	uint32_t max_duty, duty;
	bool operator==(const TIM_HandleTypeDef* htim)
	{
		return (&this->htim) == htim;
	}
	TIM_HandleTypeDef htim;
	uint32_t counter;
};

extern TIM	timer, pwm;


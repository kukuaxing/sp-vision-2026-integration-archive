#pragma once
#include "stm32f4xx_hal.h"
#include "gpio.h"

enum { BASE, PWM, IC };

class TIM
{
public:
	/* APB1 定时器时钟。系统时钟树（HSE 25 → PLL_M25/N336/P2 = 168 MHz，
	 * AHB=DIV1、APB1=DIV4 ⇒ PCLK1=42 MHz；APB 分频≠1 时定时器时钟 ×2
	 * ⇒ 84 MHz。与 TIM::Init() 里硬编码的 84000000 同源，抽成常量避免两处魔数。*/
	static constexpr uint32_t APB1_TIMER_HZ = 84000000u;

	/* ---- 1 MHz 自由计数器（TIM2，32 位）----
	 * 用途：V2 协议的 sample_time_us / mcu_receive_time_us。
	 * ⚠️ 必须【不开中断】：1 MHz 溢出即 1 M 次/秒，开中断会直接拖死系统。
	 *    所以走寄存器直配，绕开 HAL_TIM_Base_MspInit（它会开 NVIC）。
	 * ⚠️ 必须在 SystemClockConfig() 之后调用（依赖 APB1 定时器时钟已就绪）。*/
	bool UsInit(uint32_t tick_hz = 1000000u);

	/* 当前计数值（单位 µs，tick_hz=1e6 时）。
	 * 纯寄存器读，单条 LDR 原子；量化误差 ≤1 个 tick（1 µs），
	 * 相对 V2 的 2 ms 误差预算可忽略。*/
	static uint32_t NowUs();

	/* now - last，自动处理 32 位回绕（无符号减法）。
	 * 1 MHz 下回绕周期 = 2^32 µs ≈ 71.58 分钟。*/
	static uint32_t UsElapsed(uint32_t now, uint32_t last);

	/* 带符号差值：负值表示 now 早于 last（仅在人为传入 future last 时出现）。*/
	static int32_t UsDiff(uint32_t now, uint32_t last);

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
	/* 原 uint32_t counter 已删：它只在 HAL_TIM_PeriodElapsedCallback 里被
	 * 累加，而唯一的消费者（CAN 发送分频）早已改成任务私有计数器。
	 * 留着它就是留一个"看起来在用、实际没人读"的全局共享变量，
	 * 以后谁再写 timer.counter++ 会重新引入 ISR/任务竞争。*/
};

extern TIM	timer, pwm;


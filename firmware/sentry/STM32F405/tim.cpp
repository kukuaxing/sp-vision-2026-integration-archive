#include "tim.h"

TIM_HandleTypeDef* phtim[4];

/* ==========================================================================
 * 1 MHz 自由计数器（TIM2，32 位）
 *
 * 【为什么不走 HAL】
 *   HAL_TIM_Base_Init() 会回调 HAL_TIM_Base_MspInit()，而那个函数
 *   （tim.cpp 里已有）会 HAL_NVIC_EnableIRQ(TIM2_IRQn)。1 MHz 意味着
 *   每秒 1 M 次溢出中断 —— 绝对不能开。所以这里直接配寄存器：
 *   使能时钟 → 写 PSC/ARR → 清标志 → 置 CEN，全程不碰 NVIC。
 *
 * 【分频】
 *   f_counter = APB1_TIMER_HZ / (PSC+1) = 84 MHz / 84 = 1 MHz
 *   ⇒ 每个计数 = 1 µs
 *
 * 【溢出】
 *   ARR = 0xFFFFFFFF ⇒ 2^32 µs ≈ 71.58 分钟回绕一次。
 *   ⇒ 必须用无符号差值 NowUs() - last，不能比大小（见 UsElapsed）。
 *   注意：ARR 本身不产生中断，只是提供一个"到点"参考；CNT 自然回绕。
 *
 * 【精度】
 *   时钟源与 SysTick 同源（HCLK→PLL），所以 µs 时基与 xTaskGetTickCount()
 *   共享晶振漂移，二者长期一致。但 1 MHz 来自 84 MHz/84 的整数分频，
 *   不存在补不满的余数 ⇒ 无累积量化误差，只有 ≤1 µs 的读数量化。
 * ========================================================================== */
bool TIM::UsInit(uint32_t tick_hz)
{
	/* 只接受能整除 84 MHz 的频率，否则分频有截断误差 */
	if (tick_hz == 0u || APB1_TIMER_HZ % tick_hz != 0u) return false;

	/* PSC 是 16 位 ⇒ 分频比上限 65536（即 PSC = 0xFFFF）。
	 * 注意 div 恒 ≥1（tick_hz 非零且整除），不需要判下界。*/
	const uint32_t div = APB1_TIMER_HZ / tick_hz;
	if (div > 65536u) return false;

	__HAL_RCC_TIM2_CLK_ENABLE();
	(void)READ_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);   /* 写后读，等待时钟稳定 */

	TIM2->CR1  = 0;               /* 停止计数、清 CR1 全部位（关 UIE/ARPE 等） */
	TIM2->DIER = 0;               /* 🔴 强制清中断使能。
	                                若 TIM2 之前被 HAL_TIM_Base_Init 过（它会回调
	                                HAL_TIM_Base_MspInit → 开 TIM2 NVIC），残留的
	                                UIE 会让 1 MHz 计数器每秒进 100 万次 ISR 直接拖死
	                                系统。这里不碰 NVIC，只保证外设侧不再产生中断请求。*/
	TIM2->PSC = (div - 1u);       /* 84 MHz / 84 = 1 MHz */
	TIM2->ARR = 0xFFFFFFFFu;      /* 32 位满量程 */
	// PSC is buffered: load /84 now, before CEN, instead of waiting for
	// the first 32-bit overflow (~51 s at the unprescaled 84 MHz clock).
	TIM2->EGR = TIM_EGR_UG;
	TIM2->CNT = 0;                /* 从 0 起算 */
	TIM2->SR  = 0;                /* 清所有更新/捕获标志 */

	TIM2->CR1 = TIM_CR1_CEN;      /* 仅置 CEN 启动计数；UEV/DIE 保持 0 */
	return true;
}

uint32_t TIM::NowUs()
{
	return TIM2->CNT;             /* 单条 LDR，天然原子 */
}

uint32_t TIM::UsElapsed(uint32_t now, uint32_t last)
{
	return now - last;            /* 无符号减法，71.58 min 回绕自动正确 */
}

int32_t TIM::UsDiff(uint32_t now, uint32_t last)
{
	return (int32_t)(now - last); /* 负值 = now 早于 last */
}

TIM& TIM::Init(const uint32_t Mode, TIM_TypeDef* TIM, const uint32_t frequency)
{
	const float x = APB1_TIMER_HZ / frequency;
	if (x >= 5000)
	{
		htim.Init.Prescaler = x / 5000 - 1u;
		htim.Init.Period = 5000 - 1u;
	}
	else
	{
		htim.Init.Prescaler = 84u - 1u;
		htim.Init.Period = x / 84 - 1u;
	}
	htim.Instance = TIM;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	switch (Mode)
	{
	case BASE:HAL_TIM_Base_Init(&htim); break;
	case PWM:HAL_TIM_PWM_Init(&htim); break;
	case IC:HAL_TIM_IC_Init(&htim); break;
	default:;
	}
	max_duty = htim.Init.Period;
	if (TIM == TIM2)phtim[1] = &htim;
	if (TIM == TIM3)phtim[2] = &htim;
	if (TIM == TIM4)phtim[3] = &htim;
	return *this;
}
void TIM::MspPostInit(const gpio r) const
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_CLK_ENABLE(r.GPIOx);

	GPIO_InitStruct.Pin = r.Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = this->htim.Instance == TIM2 ? GPIO_AF1_TIM2 : GPIO_AF2_TIM3;//TIM\3\4都是0x02
	HAL_GPIO_Init(r.GPIOx, &GPIO_InitStruct);
}

extern "C" void TIM2_IRQHandler(void)
{
	if (phtim[1])HAL_TIM_IRQHandler(phtim[1]);
}
extern "C" void TIM3_IRQHandler(void)
{
	if (phtim[2])HAL_TIM_IRQHandler(phtim[2]);
}
extern "C" void TIM4_IRQHandler(void)
{
	if (phtim[3])HAL_TIM_IRQHandler(phtim[3]);
}

void TIM::BaseInit(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);
	HAL_TIM_Base_Start_IT(&htim);
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
	SET_BIT(RCC->APB1ENR, 0x1U << (((reinterpret_cast<unsigned>(tim_baseHandle->Instance) - APB1PERIPH_BASE)) / 0x400U));
	/* Delay after an RCC peripheral clock enabling */
	__IO uint32_t tmpreg = READ_BIT(RCC->APB1ENR, 0x1U << (((reinterpret_cast<unsigned>(tim_baseHandle->Instance) - APB1PERIPH_BASE)) / 0x400U));
	UNUSED(tmpreg);
	if (tim_baseHandle->Instance == TIM4)
	{
		HAL_NVIC_SetPriority(static_cast<IRQn_Type>((reinterpret_cast<unsigned>(tim_baseHandle->Instance) - APB1PERIPH_BASE) / 0x400U + 28), 0, 0);
	}
	else
	{
		HAL_NVIC_SetPriority(static_cast<IRQn_Type>((reinterpret_cast<unsigned>(tim_baseHandle->Instance) - APB1PERIPH_BASE) / 0x400U + 28), 0, 1);
	}
	HAL_NVIC_EnableIRQ(static_cast<IRQn_Type>((reinterpret_cast<unsigned>(tim_baseHandle->Instance) - APB1PERIPH_BASE) / 0x400U + 28));
}


extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	/* 【已清空】原来这里做 `if (timer.counter++, timer == htim){}`：
	 *   timer.counter 是【全局变量】，被所有 TIM 实例的 ISR 共享累加，
	 *   而它的唯一消费者 CanTransimtTask 的 `%3` 分频已经改成任务私有计数器
	 *   （见 taskslist.cpp），所以这个累加现在是零消费者的纯开销
	 *   —— TIM3 1 kHz 中断里白跑一次读改写。
	 *
	 * 保留空函数体是必要的：删掉它会退回 HAL 的 __weak 空实现，
	 * 那没问题；但显式写出来是为了说明"这个中断确实什么都不做"，
	 * 避免以后有人以为回调漏实现。
	 *
	 * ⚠️ 连带结论：TIM3 现在挂着一个 1 kHz 空中断（STM32F405.cpp 里
	 *    timer.Init(BASE, TIM3, 1000).BaseInit()），如果确实没有任何
	 *    消费者，这一句连同 HAL_TIM_Base_Start_IT 可以整条去掉。
	 *    （改 STM32F405.cpp 需另行授权，本次未动。）*/
	UNUSED(htim);
}

TIM& TIM::PWMInit(uint32_t channel, float duty, gpio r)
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = static_cast<unsigned>(duty * (htim.Init.Period + 1)) - 1u;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, channel);

	this->MspPostInit(r);
	HAL_TIM_PWM_Start(&htim, channel);
	return *this;
}
void TIM::PWMDuty(const uint32_t channel, const float duty) const
{
	//*reinterpret_cast<uint32_t*>(reinterpret_cast<uint8_t*>(htim.Instance->CCR1) + (channel - TIM_CHANNEL_1)) = (htim.Init.Period + 1) * duty - 1u;
	//__HAL_TIM_SET_COMPARE(&htim, channel, (htim.Init.Period + 1) * duty - 1u);
	__HAL_TIM_SET_COMPARE(&htim, channel,  duty );
}
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{
	__IO uint32_t tmpreg = 0x00U;
	SET_BIT(RCC->APB1ENR, 0x1U << (((reinterpret_cast<unsigned>(tim_pwmHandle->Instance) - APB1PERIPH_BASE)) / 0x400U));
	/* Delay after an RCC perip heral clock enabling */
	tmpreg = READ_BIT(RCC->APB1ENR, 0x1U << ((reinterpret_cast<unsigned>(tim_pwmHandle->Instance - APB1PERIPH_BASE)) / 0x400U));
	UNUSED(tmpreg);
}


void TIM::ICInit(int32_t channel, uint32_t ICPolarity, gpio r)
{
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;
	this->MspPostInit(r);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

	sConfigIC.ICPolarity = ICPolarity;//TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	HAL_TIM_IC_ConfigChannel(&htim, &sConfigIC, channel);
	HAL_TIM_IC_Start_IT(&htim, channel);
}
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{
	SET_BIT(RCC->APB1ENR, 0x1U << ((reinterpret_cast<unsigned>(tim_icHandle->Instance - APB1PERIPH_BASE)) / 0x400U));
	/* Delay after an RCC peripheral clock enabling */
	__IO uint32_t tmpreg = READ_BIT(RCC->APB1ENR, 0x1U << ((reinterpret_cast<unsigned>(tim_icHandle->Instance - APB1PERIPH_BASE)) / 0x400U));
	UNUSED(tmpreg);
	HAL_NVIC_SetPriority(static_cast<IRQn_Type>(reinterpret_cast<unsigned>(tim_icHandle->Instance - APB1PERIPH_BASE) / 0x400U + 28), 0, 0);
	HAL_NVIC_EnableIRQ(static_cast<IRQn_Type>(reinterpret_cast<unsigned>(tim_icHandle->Instance - APB1PERIPH_BASE) / 0x400U + 28));
}
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim)
{
	if (htim->Instance == TIM4)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			//HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);

			TIM_MasterConfigTypeDef sMasterConfig;
			TIM_IC_InitTypeDef sConfigIC;
			sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
			sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
			HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig);

			sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
			sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
			sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
			sConfigIC.ICFilter = 0;
			HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, htim->Channel);
			HAL_TIM_IC_Start_IT(htim, htim->Channel);
		}
	}
}


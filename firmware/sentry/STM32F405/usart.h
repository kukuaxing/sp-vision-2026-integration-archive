#pragma once

#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"
#include "FreeRTOS.h"
#include "queue.h"

#define UART_MAX_LEN 320 // A complete V2 frame can reach 142 bytes, plus concatenated commands.
	class UART
	{
	public:
		UART& Init(USART_TypeDef* Instance, uint32_t BaudRate);
		UART& DMATxInit(void);
		UART& DMARxInit(const uint8_t* buffer = nullptr, const uint32_t size = UART_MAX_LEN);

		void OnUARTITHandler();
		void OnDMAITHandler();
		/* ⚠️ 现在返回 bool：上一帧没发完时【拒绝】并计数，绝不重装在途的 DMA
		 * （原实现无脑 DISABLE→重装→ENABLE，会发出「半截 + 拼接」的帧）。
		 * 现有调用点忽略返回值也能编译，行为不变。*/
		bool DMATransmit(uint8_t* const pData, const uint32_t Size);
        bool TxIdle() const { return huart.hdmatx && huart.hdmatx->Instance->NDTR == 0u; }
		void UARTTransmit(uint8_t* pData, uint32_t Size);
		static void DMAClearAllFlags(DMA_HandleTypeDef* dmax)
		{
			const auto ele = reinterpret_cast<uint32_t>(dmax->Instance);

			(ele == reinterpret_cast<uint32_t>(DMA1_Stream0)) ? (DMA1->LIFCR = 0x0000003D) : \
				(ele == reinterpret_cast<uint32_t>(DMA1_Stream1)) ? (DMA1->LIFCR = 0x00000F40) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream2) ? (DMA1->LIFCR = 0x003D0000) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream3) ? (DMA1->LIFCR = 0x0F400000) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream4) ? (DMA1->HIFCR = 0x0000003D) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream5) ? (DMA1->HIFCR = 0x00000F40) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream6) ? (DMA1->HIFCR = 0x003D0000) : \
				(ele == reinterpret_cast<uint32_t>DMA1_Stream7) ? (DMA1->HIFCR = 0x0F400000) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream0) ? (DMA2->LIFCR = 0x0000003D) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream1) ? (DMA2->LIFCR = 0x00000F40) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream2) ? (DMA2->LIFCR = 0x003D0000) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream3) ? (DMA2->LIFCR = 0x0F400000) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream4) ? (DMA2->HIFCR = 0x0000003D) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream5) ? (DMA2->HIFCR = 0x00000F40) : \
				(ele == reinterpret_cast<uint32_t>DMA2_Stream6) ? (DMA2->HIFCR = 0x003D0000) : \
				(DMA2->HIFCR = 0x0F400000);
		}

		QueueHandle_t UartQueueHandler = xQueueCreate(1, UART_MAX_LEN);
		bool updateFlag = false;
		BaseType_t pd_Rx = false;
		uint32_t dataDmaNum{};
		uint8_t m_uartrx[UART_MAX_LEN];

		/* ==================================================================
		 * 接收元信息（IDLE 中断里更新）
		 *
		 * ⚠️ 为什么不做成队列 item：队列 item 必须是 UART_MAX_LEN，因为有 6 个
		 *    消费端（xuc / imu / RC / judgement / supercap / power）都写的是
		 *    xQueueReceive(*h, buf, ...)。改 item 长度会一次性改爆 6 个文件。
		 *    而队列是【深度 1 覆盖写】⇒「队列里那一帧」与「本成员」天然同拍，
		 *    语义严格一致。
		 *
		 * ⚠️ 这几个量是 ISR 写、任务读。成员声明为 volatile；且 32/16 位对齐访问
		 *    在 Cortex-M4 上是单条 LDR/STR（天然原子）⇒ 任务侧不会读到"半新半旧"。
		 * ================================================================== */

		/* 本帧到达时刻（µs，TIM2 @1MHz）。V2 协议的 sample_time_us 基准。
		 * ⚠️ 若 STM32F405.cpp 里还没调 TIM::UsInit()，TIM2 未使能 ⇒ 本值恒为 0，
		 *    上层可据此判断"微秒时基未就绪"。*/
		volatile uint32_t rx_time_us = 0;

		/* 本帧真实字节数 = UART_MAX_LEN - NDTR。
		 * ⚠️ DMA 缓冲【不清零】，所以 UART_MAX_LEN 只能当"上限"用，
		 *    流式解析必须靠这个长度才知道有效数据的末尾在哪。*/
		volatile uint16_t rx_len = 0;

		/* 诊断计数（ISR 里累加，watch 用） */
		volatile uint32_t rx_cnt = 0;        /* 收到的帧总数 */
		volatile uint32_t rx_drop_cnt = 0;   /* 覆盖丢帧数（队列满时又来一帧） */
		volatile uint32_t rx_overflow = 0;   /* 异常：NDTR 算出的长度 > UART_MAX_LEN */

		/* 发送诊断 */
		volatile uint32_t tx_cnt = 0;         /* 成功发起的次数 */
		volatile uint32_t tx_busy_reject = 0; /* 因上一帧在途而被拒绝的次数 */

		/* 便捷读取（任务上下文） */
        // Copy bytes and their ISR metadata in one critical section. IRQ priority 6
        // is masked by FreeRTOS configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5.
        bool ReceiveSnapshot(uint8_t* bytes, uint16_t& length, uint32_t& arrival_us);
        uint32_t GetRxTimeUs() const { return rx_time_us; }
		uint16_t GetRxLen()    const { return rx_len; }

	private:
		UART_HandleTypeDef huart;
		

	};


#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

extern UART uart1, uart2, uart3, uart4, uart5, uart6;

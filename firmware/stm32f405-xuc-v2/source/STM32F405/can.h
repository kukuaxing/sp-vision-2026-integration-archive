#pragma once
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

	/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"

class CAN
{
public:
	uint8_t can_number;
	void Init(CAN_TypeDef *instance);
	void InitFilter();
	void RefreshError(); // [后续修改] 刷新CAN错误寄存器和收发调试状态
	HAL_StatusTypeDef Transmit(const uint32_t ID, const uint8_t* const pData, const uint8_t len = 8);
	uint8_t data[12][8];//接收数据缓冲区，结合，motor.h中ontimer函数，12是防止3508和6020接收数据时存放位置冲突
	uint8_t joint_data[6][6];
	uint8_t temp_data[16];
	uint8_t jointpdata[6][8]{};
	uint8_t jointidata[6][8];
	CAN_HandleTypeDef hcan;
	BaseType_t pd_Rx = false, pd_Tx = false;
// ===== [后续修改开始] CAN调试状态：Live Watch查看错误码、寄存器和收发计数 =====
	uint32_t error = 0; // HAL_CAN_GetError返回值，整体错误码
	uint32_t esr = 0, msr = 0, tsr = 0, rf0r = 0; // CAN硬件寄存器快照，调试总线状态使用
	uint32_t last_rx_id = 0, last_tx_id = 0; // 最近一次接收/发送的标准ID
	uint32_t rx_count = 0, tx_count = 0; // CAN收发计数，用于判断通信是否持续进行
	uint8_t lec = 0, rec = 0, tec = 0; // LEC最近错误类型，REC接收错误计数，TEC发送错误计数
	HAL_StatusTypeDef init_status = HAL_OK; // CAN初始化返回状态
	HAL_StatusTypeDef filter_status = HAL_OK; // CAN过滤器配置返回状态
	HAL_StatusTypeDef last_tx_status = HAL_OK; // 最近一次CAN发送返回状态
// ===== [后续修改结束] CAN调试状态 =====

private:
	CanTxMsgTypeDef	TxMessage;//发送结构体
	CanRxMsgTypeDef RxMessage;//接收结构体
};

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

extern CAN can1, can2;


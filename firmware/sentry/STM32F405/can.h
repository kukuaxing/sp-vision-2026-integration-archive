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
	HAL_StatusTypeDef Transmit(const uint32_t ID, const uint8_t* const pData, const uint8_t len = 8);
	uint8_t data[12][8];//接收数据缓冲区，结合，motor.h中ontimer函数，12是防止3508和6020接收数据时存放位置冲突
	uint8_t joint_data[6][6];
	uint8_t temp_data[16];
	uint8_t jointpdata[6][8]{};
	uint8_t jointidata[6][8];
	CAN_HandleTypeDef hcan;
	CAN_RxHeaderTypeDef RxMessage{};
	uint8_t RxData[8]{};
	BaseType_t pd_Rx = false, pd_Tx = false;
	uint8_t DMmotor_data[2][8];//DM电机接收数据缓冲区
	volatile uint32_t DMmotor_rx_count[2]{};//每个DM反馈ID的接收计数，用于上线/掉线判定
	uint8_t DMmotor_temp_data_yaw[8];//DM电机发送数据缓冲区
	uint8_t DMmotor_temp_data_pitch[8];

private:
	CAN_TxHeaderTypeDef TxMessage{};
};

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

extern CAN can1, can2;


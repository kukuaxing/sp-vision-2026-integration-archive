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
#include "semphr.h"

class CAN
{
public:

	uint8_t can_number;
	void Init(CAN_TypeDef *instance);
	void InitFilter();
	HAL_StatusTypeDef Transmit(const uint32_t ID, const uint8_t* const pData, const uint8_t len = 8);
	uint8_t data[12][8];
	uint8_t joint_data[6][6];
	uint8_t temp_data[16];
	uint8_t jointpdata[6][8]{};
	uint8_t jointidata[6][8];
	CAN_HandleTypeDef hcan;
	BaseType_t pd_Rx = false, pd_Tx = false;
	SemaphoreHandle_t TxMutex = nullptr;

private:
	CanTxMsgTypeDef	TxMessage;
	CanRxMsgTypeDef RxMessage;
};

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

extern CAN can1, can2;


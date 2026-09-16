#include "can.h"
#include "label.h"
#include "string.h"

/*
* @brief		CAN通信初始化函数
* @param		CAN通道基地址
*/
void CAN::Init(CAN_TypeDef* instance)
{
	hcan.Instance = instance;				//can基地址赋值
	hcan.Init.Prescaler = 6;				//分频系数（不减一）
	hcan.Init.Mode = CAN_MODE_NORMAL;		//普通模式
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = ENABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	HAL_CAN_Init(&hcan);
	InitFilter();
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*
 * @brief      CAN过滤器初始化
*/
void CAN::InitFilter()
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef CAN_FilterConfigStructure{};

	//can1(0-13)和can2(14-27)分别得到一半的filter
	if (hcan.Instance == CAN1)
	{
		CAN_FilterConfigStructure.FilterBank = 0;
	}
	else if (hcan.Instance == CAN2)
	{
		CAN_FilterConfigStructure.FilterBank = 14;
	}
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;				//掩码模式
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;				//32位宽
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;							//接收所有数据
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;						//激活过滤器
	CAN_FilterConfigStructure.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan, &CAN_FilterConfigStructure);

}

/*
 * @brief       CAN外设配置
 * @param       *hcan    : CAN句柄指针
*/
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if (hcan->Instance == CAN1)
	{
		/* Peripheral clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**CAN1 GPIO Configuration
		PD0     ------> CAN1_RX
		PD1     ------> CAN1_TX*/
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(CAN1_TX_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	}
	else if (hcan->Instance == CAN2)
	{
		/* Peripheral clock enable */
		__HAL_RCC_CAN2_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**CAN1 GPIO Configuration
		PB12     ------> CAN2_RX
		PB13     ------> CAN2_TX*/
		GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(CAN2_TX_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
		HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	}
}


/*
 * @brief       CAN通信传输函数
 * @param       ID		:	CAN外设ID
 * @param		*pData	: 	传输数据数组
 * @param		len		:	传输数据长度
*/
HAL_StatusTypeDef CAN::Transmit(const uint32_t ID, const uint8_t* const pData, const uint8_t len)
{
	if (pData == nullptr || len > 8U) return HAL_ERROR;
	TxMessage.StdId = ID;
	TxMessage.ExtId = 0U;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = len;
	TxMessage.TransmitGlobalTime = DISABLE;
	uint8_t data[8]{};
	memcpy(data, pData, len);
	uint32_t mailbox = 0U;
	HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &TxMessage, data, &mailbox);

	//ready = status != HAL_ERROR;// == HAL_OK;
	return status;
}

/*
 * @brief       CAN通信接收回调函数
 * @param       *hcan		:	CAN句柄指针
*/

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	CAN* bus = (hcan == &can1.hcan) ? &can1 : (hcan == &can2.hcan) ? &can2 : nullptr;
	if (bus == nullptr ||
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &bus->RxMessage, bus->RxData) != HAL_OK)
	{
		return;
	}

	const uint32_t id = bus->RxMessage.StdId;
	if (bus == &can1 && id == 0x09U)
	{
		memcpy(can1.DMmotor_data[1], bus->RxData, sizeof(bus->RxData));
		can1.DMmotor_rx_count[1]++;
	}
	else if (bus == &can2 && id == 0x06U)
	{
		memcpy(can2.DMmotor_data[0], bus->RxData, sizeof(bus->RxData));
		can2.DMmotor_rx_count[0]++;
	}
	else if (id >= 0x201U && id < 0x201U + 12U)
	{
		memcpy(bus->data[id - 0x201U], bus->RxData, sizeof(bus->RxData));
	}
}

extern "C" void CAN1_TX_IRQHandler()
{
	HAL_CAN_IRQHandler(&can1.hcan);
}
extern "C" void CAN1_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&can1.hcan);
}
extern "C" void CAN2_TX_IRQHandler()
{
	HAL_CAN_IRQHandler(&can2.hcan);
}
extern "C" void CAN2_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&can2.hcan);
}

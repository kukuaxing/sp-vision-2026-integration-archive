#include "can.h"
#include "label.h"
#include "string.h"
#include "calibration_mode.h"
// Passive inventory: never issues discovery, enable or parameter-write commands.
struct CanObservedFrame {
    uint32_t bus, id, count, tick, dlc;
    uint8_t data[8];
};
volatile CanObservedFrame can_observed[64] = {};
volatile uint32_t can_observed_used = 0;
volatile uint32_t can_error_count[2] = {};
volatile uint32_t can_last_esr[2] = {};
volatile uint32_t can_tx_timeout[2] = {};
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* handle) {
    const uint32_t bus = handle->Instance == CAN1 ? 0 : 1;
    ++can_error_count[bus]; can_last_esr[bus] = handle->Instance->ESR;
    // Legacy HAL error handling disables RX along with error IRQs.
    __HAL_CAN_ENABLE_IT(handle, CAN_IT_FMP0);
}
volatile uint32_t can_probe_request = 0; // debugger mailbox: bus << 8 | motor ID, read register 2 only
volatile uint32_t can_probe_reply_count = 0, can_probe_reply_bus = 0;
volatile uint8_t can_probe_reply[8] = {};
void GimbalDiagnosticUpdate() {
    const uint32_t request = can_probe_request;
    if (!request) return;
    can_probe_request = 0;
    const uint32_t bus = request >> 8, id = request & 255;
    if ((bus != 1 && bus != 2) || id < 1 || id > 16) return;
    const uint8_t query[8] = {uint8_t(id), 0, 0x33, 2, 0, 0, 0, 0};
    (bus == 1 ? can1 : can2).Transmit(0x7FF, query, 8);
}
volatile uint32_t small_yaw_rx_count = 0, small_yaw_rx_tick = 0;
volatile uint32_t calibration_yaw_rx_count = 0;
volatile uint32_t calibration_yaw_rx_tick = 0;
volatile uint32_t calibration_pitch_rx_count = 0;
volatile uint32_t calibration_pitch_rx_tick = 0;
volatile uint32_t calibration_blocked_tx_count = 0;
volatile uint32_t calibration_query_tx_count = 0;
volatile uint32_t pitch_temperature_limit_reply_count = 0;
volatile uint8_t pitch_temperature_limit_reply[8] = {};

/*
* @brief		CAN通信初始化函数
* @param		CAN通道基地址
*/
void CAN::Init(CAN_TypeDef* instance)
{
	hcan.Instance = instance;				//can基地址赋值
	hcan.Init.Prescaler = 6;				//分频系数（不减一）
	hcan.Init.Mode = CAN_MODE_NORMAL;		//普通模式
	hcan.Init.SJW = CAN_SJW_1TQ;			//同步段为1个字节
	hcan.Init.BS1 = CAN_BS1_2TQ;			//相位缓冲段1为2个字节
	hcan.Init.BS2 = CAN_BS2_4TQ;			//相位缓冲段2为4个字节
	hcan.Init.TTCM = DISABLE;				//非时间触发通信模式
	hcan.Init.ABOM = ENABLE;				//允许自动离线管理
	hcan.Init.AWUM = ENABLE;				//允许自动唤醒
	hcan.Init.NART = DISABLE;				//禁止报文自动重传，即数据只传一次
	hcan.Init.RFLM = DISABLE;				//禁止报文溢出锁定
	hcan.Init.TXFP = DISABLE;				//传输优先级，EANBLE:ID优先 DISABLE:报文优先
	HAL_CAN_Init(&hcan);					//调用HAL库初始化函数
#if !SENTRY_CALIBRATION_READ_ONLY && !SENTRY_PITCH_ZERO_MODE
	HAL_CAN_Transmit_IT(&hcan);
#endif
				//开启CAN通信发送中断
	InitFilter();
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);	//开启CAN通信接收中断
}

/*
 * @brief      CAN过滤器初始化
*/
void CAN::InitFilter()
{
	//can1 &can2 use same filter config
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;

	//can1(0-13)和can2(14-27)分别得到一半的filter
	if (hcan.Instance == CAN1)
	{
		CAN_FilterConfigStructure.FilterNumber = 0;								//选择过滤器0
	}
	else if (hcan.Instance == CAN2)
	{
		CAN_FilterConfigStructure.FilterNumber = 14;							//选择过滤器0
	}
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;				//掩码模式
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;				//32位宽
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;							//接收所有数据
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;			//过滤器关联到FIFO0
	CAN_FilterConfigStructure.FilterActivation = ENABLE;						//激活过滤器
	CAN_FilterConfigStructure.BankNumber = 14; // CAN1 banks 0-13, CAN2 banks 14-27.

	HAL_CAN_ConfigFilter(&hcan, &CAN_FilterConfigStructure);

	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;
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
#if SENTRY_PITCH_ZERO_MODE
	const uint8_t read_temperature_limit[8] = {9, 0, 0x33, 2, 0, 0, 0, 0};
	const bool read_only_query = ID == 0x7FF && len == 8 && pData[0] >= 1 && pData[0] <= 16 && pData[1] == 0 && memcmp(pData + 2, read_temperature_limit + 2, 6) == 0;
	bool small_yaw_only = hcan.Instance == CAN1 && ID == 0x1FF && len == 8;
	for (int i = 2; i < 8 && small_yaw_only; ++i) small_yaw_only = pData[i] == 0;
	const bool gimbal_command = small_yaw_only || (hcan.Instance == CAN1 && (ID == 0x109 || read_only_query)) || (hcan.Instance == CAN2 && (ID == 0x206 || read_only_query));
	if (!gimbal_command || len != 8 || xTaskGetCurrentTaskHandle() != CanTxTask_Handler) {
		++calibration_blocked_tx_count; return HAL_OK;
	}
	// Hardware zero and all special commands except enable/disable are prohibited.
	bool special = true;
	for (int i = 0; i < 7; ++i) special = special && pData[i] == 0xFF;
	if (special && pData[7] != 0xFC && pData[7] != 0xFD) return HAL_ERROR;
#endif
#if SENTRY_CALIBRATION_READ_ONLY
	const uint8_t query[8] = {0x09, 0x00, 0xCC, 0, 0, 0, 0, 0};
	const uint8_t disable[8] = {255,255,255,255,255,255,255,0xFD};
	const bool permitted = len == 8 && ((ID == 0x7FF && memcmp(pData, query, 8) == 0) || (ID == 0x109 && memcmp(pData, disable, 8) == 0));
	if (hcan.Instance != CAN1 || !permitted || xTaskGetCurrentTaskHandle() != CanTxTask_Handler) {
		++calibration_blocked_tx_count;
		return HAL_OK;
	}
#endif
	hcan.pTxMsg->StdId = ID;					//设置标识符
	hcan.pTxMsg->IDE = CAN_ID_STD;				//标准帧(无拓展)
	hcan.pTxMsg->RTR = CAN_RTR_DATA;			//数据帧
	hcan.pTxMsg->DLC = len;						//设置长度
	memcpy(hcan.pTxMsg->Data, pData, len);
	HAL_StatusTypeDef status = HAL_CAN_Transmit(&hcan, 10);
	if (status != HAL_OK) ++can_tx_timeout[hcan.Instance == CAN1 ? 0 : 1];
#if SENTRY_CALIBRATION_READ_ONLY
	if (status == HAL_OK) ++calibration_query_tx_count;
#endif

	//ready = status != HAL_ERROR;// == HAL_OK;
	return status;
}

/*
 * @brief       CAN通信接收回调函数
 * @param       *hcan		:	CAN句柄指针
*/

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
    const uint32_t bus = hcan->Instance == CAN1 ? 1 : 2;
    if (hcan->pRxMsg->DLC == 8 && hcan->pRxMsg->Data[2] == 0x33 && hcan->pRxMsg->Data[3] == 2) {
        for (uint32_t i=0;i<8;++i) can_probe_reply[i] = hcan->pRxMsg->Data[i];
        can_probe_reply_bus = bus; ++can_probe_reply_count;
    }

    const uint32_t id = hcan->pRxMsg->IDE == CAN_ID_STD ? hcan->pRxMsg->StdId : (hcan->pRxMsg->ExtId | 0x80000000U);
    uint32_t slot = 0;
    while (slot < can_observed_used && (can_observed[slot].bus != bus || can_observed[slot].id != id)) ++slot;
    if (slot < 64) {
        if (slot == can_observed_used) { can_observed[slot].bus = bus; can_observed[slot].id = id; ++can_observed_used; }
        can_observed[slot].tick = HAL_GetTick();
        can_observed[slot].dlc = hcan->pRxMsg->DLC;
        for (uint32_t i=0; i<8; ++i) can_observed[slot].data[i] = hcan->pRxMsg->Data[i];
        ++can_observed[slot].count;
    }
	if (hcan == &can1.hcan)
	{
		if (hcan->pRxMsg->DLC == 8 && hcan->pRxMsg->Data[0] == 9 && hcan->pRxMsg->Data[1] == 0 && hcan->pRxMsg->Data[2] == 0x33 && hcan->pRxMsg->Data[3] == 2) {
			for (int i=0; i<8; ++i) pitch_temperature_limit_reply[i] = hcan->pRxMsg->Data[i];
			++pitch_temperature_limit_reply_count;
		}
		else if (hcan->pRxMsg->StdId == 0x09 && hcan->pRxMsg->DLC == 8)
		{
			memcpy(can1.DMmotor_data[1], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
			calibration_pitch_rx_tick = HAL_GetTick();
			++calibration_pitch_rx_count;
		}
		else if (hcan->pRxMsg->StdId >= 0x201 && hcan->pRxMsg->StdId <= 0x208)
		{
			memcpy(can1.data[hcan->pRxMsg->StdId - 0x201], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
			if (hcan->pRxMsg->StdId == 0x205 && hcan->pRxMsg->DLC == 8) { small_yaw_rx_tick = HAL_GetTick(); ++small_yaw_rx_count; }
		}
	}

	else
	{
		if (hcan->pRxMsg->StdId == 0x06 && hcan->pRxMsg->DLC == 8)
		{
			memcpy(can2.DMmotor_data[0], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
			calibration_yaw_rx_tick = HAL_GetTick();
			++calibration_yaw_rx_count;
		}
		else if (hcan->pRxMsg->StdId >= 0x201 && hcan->pRxMsg->StdId <= 0x208)
		{
			memcpy(can2.data[hcan->pRxMsg->StdId - 0x201], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
		}
	}

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
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

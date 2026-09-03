#include "can.h"
#include "label.h"
#include "string.h"

// ===== [后续修改开始] 显式声明CAN中断函数，减少C++编译器未声明警告 =====
extern "C" void CAN1_TX_IRQHandler(void);
extern "C" void CAN1_RX0_IRQHandler(void);
extern "C" void CAN2_TX_IRQHandler(void);
extern "C" void CAN2_RX0_IRQHandler(void);
// ===== [后续修改结束] CAN中断函数声明 =====

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
	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;
	init_status = HAL_CAN_Init(&hcan);		//调用HAL库初始化函数
	RefreshError(); // 刷新CAN硬件错误/状态寄存器快照
	InitFilter();
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);	//开启CAN通信接收中断
	RefreshError(); // 刷新CAN硬件错误/状态寄存器快照
}
// ===== [后续修改开始] CAN错误/状态寄存器刷新函数 =====
void CAN::RefreshError() // 将CAN硬件状态复制到类成员，方便Live Watch观察
{
	if (hcan.Instance == nullptr)
	{
		return;
	}

	error = HAL_CAN_GetError(&hcan); // 读取HAL层CAN错误码
	esr = hcan.Instance->ESR; // Error Status Register，包含LEC/TEC/REC等错误信息
	msr = hcan.Instance->MSR; // Master Status Register，记录总线/睡眠/唤醒等状态
	tsr = hcan.Instance->TSR; // Transmit Status Register，记录发送邮箱状态
	rf0r = hcan.Instance->RF0R; // FIFO0状态寄存器，判断接收FIFO情况
	lec = static_cast<uint8_t>((esr >> 4) & 0x07U); // 最近一次CAN错误类型
	tec = static_cast<uint8_t>((esr >> 16) & 0xFFU); // 发送错误计数
	rec = static_cast<uint8_t>((esr >> 24) & 0xFFU); // 接收错误计数
}

/*
 * @brief      CAN过滤器初始化
*/
// ===== [后续修改结束] CAN错误/状态寄存器刷新函数 =====
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
	CAN_FilterConfigStructure.BankNumber = 0;

	filter_status = HAL_CAN_ConfigFilter(&hcan, &CAN_FilterConfigStructure); // [后续修改] 保存过滤器配置返回值，方便排查CAN初始化问题
	RefreshError(); // 刷新CAN硬件错误/状态寄存器快照

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
	hcan.pTxMsg->StdId = ID;					//设置标识符
	hcan.pTxMsg->IDE = CAN_ID_STD;				//标准帧(无拓展)
	hcan.pTxMsg->RTR = CAN_RTR_DATA;			//数据帧
	hcan.pTxMsg->DLC = len;						//设置长度
	memcpy(hcan.pTxMsg->Data, pData, len);
// ===== [后续修改开始] 记录CAN发送状态，便于调试电机无连接问题 =====
	last_tx_id = ID; // 记录最近一次发送的CAN标准ID
	last_tx_status = HAL_CAN_Transmit(&hcan, 10); // 发送CAN帧并记录HAL返回状态
	pd_Tx = (last_tx_status == HAL_OK); // 标记最近一次发送是否成功
	if (pd_Tx)
	{
		tx_count++; // 发送成功计数递增
	}
	RefreshError(); // 刷新CAN硬件错误/状态寄存器快照

	//ready = status != HAL_ERROR;// == HAL_OK;
// ===== [后续修改结束] CAN发送状态记录 =====
	return last_tx_status;
}

/*
 * @brief       CAN通信接收回调函数
 * @param       *hcan		:	CAN句柄指针
*/
// ===== [后续修改开始] CAN接收回调：同时支持CAN1底盘3508和CAN2 yaw 6020反馈 =====
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	CAN* can = nullptr; // 根据HAL句柄反查当前是CAN1还是CAN2对象
	if (hcan == &can1.hcan)
	{
		can = &can1;
	}
	else if (hcan == &can2.hcan)
	{
		can = &can2;
	}

	if (can != nullptr)
	{
		const uint32_t std_id = hcan->pRxMsg->StdId; // 读取本次接收到的标准ID
		can->pd_Rx = true; // 标记该CAN线已收到过数据
		can->last_rx_id = std_id; // 保存最近一次接收ID，便于确认电机反馈ID
		can->rx_count++; // 接收计数递增，用于判断反馈是否持续刷新

		if (can == &can1)
		{
			if (std_id >= 1U && std_id <= 5U) // 达妙DM电机反馈ID为0x01~0x05，挂在CAN1
			{
				memcpy(can1.jointidata[std_id - 1U], hcan->pRxMsg->Data, sizeof(uint8_t) * 8); // 按ID下标保存达妙电机反馈
			}
			else
			{
				const uint32_t index = std_id - 0x201U; // DJI电机反馈ID从0x201开始，转换为data数组下标
				if (index < 12U)
				{
					memcpy(can1.data[index], hcan->pRxMsg->Data, sizeof(uint8_t) * 8); // 保存CAN1电机反馈数据
				}
			}
		}
		else
		{
			const uint32_t index = std_id - 0x201U; // DJI电机反馈ID从0x201开始，转换为data数组下标
			if (index < 12U)
			{
				memcpy(can2.data[index], hcan->pRxMsg->Data, sizeof(uint8_t) * 8); // 保存CAN2电机反馈数据，6020的0x208落在下标7
			}
		}

		can->RefreshError(); // 刷新CAN硬件错误/状态寄存器快照
	}

	//can2.pd_Rx = xQueueSendFromISR((QueueHandle_t)Can2QueueHadle, hcan->pRxMsg->Data, NULL);

/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
}

// ===== [后续修改结束] CAN接收回调 =====
// ===== [后续修改开始] CAN错误回调：错误发生时立即刷新调试变量 =====
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan) // CAN错误中断回调，用于实时更新错误码
{
	if (hcan == &can1.hcan)
	{
		can1.RefreshError(); // CAN1出错时刷新CAN1调试状态
	}
	else if (hcan == &can2.hcan)
	{
		can2.RefreshError(); // CAN2出错时刷新CAN2调试状态
	}
}

// ===== [后续修改结束] CAN错误回调 =====
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

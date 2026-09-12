#include "can.h"
#include "label.h"
#include "string.h"
#include "imu.h"          // ? 新增：为了 CanRxMsg_t / g_imu_can_queue
#include "FreeRTOS.h"
#include "queue.h"
#include "xuc_can.h" 
volatile uint32_t can2_dm_cnt[6] = { 0 };   // [1]..[5] 有效
volatile uint32_t can2_dm_last_motor_id = 0;
volatile uint32_t can2_dm_last_stdid = 0;
volatile uint8_t  can2_dm_last_d0 = 0;

static uint8_t ResolveDmMotorId(const uint8_t* data)
{
    uint8_t motor_id = data[0] & 0x0F;
    if (motor_id >= 1 && motor_id <= 5) {
        return motor_id;
    }

    return 0;
}

void CAN::Init(CAN_TypeDef* instance)
{
    hcan.Instance = instance;

    hcan.Init.Prescaler = 3;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SJW = CAN_SJW_1TQ;
    hcan.Init.BS1 = CAN_BS1_11TQ;
    hcan.Init.BS2 = CAN_BS2_2TQ;
    hcan.Init.TTCM = DISABLE;
    hcan.Init.ABOM = ENABLE;
    hcan.Init.AWUM = ENABLE;
    hcan.Init.NART = DISABLE;
    hcan.Init.RFLM = DISABLE;
    hcan.Init.TXFP = DISABLE;

    HAL_CAN_Init(&hcan);

    InitFilter();

    HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

    if (TxMutex == nullptr) {
        TxMutex = xSemaphoreCreateMutex();
    }
}

void CAN::InitFilter()
{
    CAN_FilterConfTypeDef CAN_FilterConfigStructure;

    if (hcan.Instance == CAN1) {
        CAN_FilterConfigStructure.FilterNumber = 0;
    }
    else if (hcan.Instance == CAN2) {
        CAN_FilterConfigStructure.FilterNumber = 14;
    }

    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;    // 接收所有
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;
    CAN_FilterConfigStructure.BankNumber = 14;

    HAL_CAN_ConfigFilter(&hcan, &CAN_FilterConfigStructure);

    hcan.pTxMsg = &TxMessage;
    hcan.pRxMsg = &RxMessage;
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if (hcan->Instance == CAN1)
    {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(CAN1_TX_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    }
    else if (hcan->Instance == CAN2)
    {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_CAN2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(CAN2_TX_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
        HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 6, 0);
        HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    }
}

HAL_StatusTypeDef CAN::Transmit(const uint32_t ID, const uint8_t* const pData, const uint8_t len)
{
    if (TxMutex != nullptr) {
        xSemaphoreTake(TxMutex, pdMS_TO_TICKS(20));
    }

    hcan.pTxMsg->StdId = ID;
    hcan.pTxMsg->IDE = CAN_ID_STD;
    hcan.pTxMsg->RTR = CAN_RTR_DATA;
    hcan.pTxMsg->DLC = len;
    memcpy(hcan.pTxMsg->Data, pData, len);
    HAL_StatusTypeDef ret = HAL_CAN_Transmit(&hcan, 10);

    if (TxMutex != nullptr) {
        xSemaphoreGive(TxMutex);
    }
    return ret;
}
uint32_t error = 0;
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
    uint32_t stdid = hcan->pRxMsg->StdId;
    uint8_t* d = hcan->pRxMsg->Data;

    // ========== CAN1 ==========
    if (hcan == &can1.hcan)
    {

        if (stdid >= 0x201) {
            uint32_t idx = stdid - 0x201;
            if (idx < 12) {
                memcpy(can1.data[idx], d, 8);
            }
        }

        __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
        HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
        return;
    }

    // ========== CAN2 ==========
    if (hcan == &can2.hcan)
    {
        error = HAL_CAN_GetError(&can2.hcan);
        // 保留 IMU/XUC 的 CAN 转发接口（当前改为串口方案，暂不启用）
        // if (stdid == 0x99 && hcan->pRxMsg->DLC == 8)
        // {
        //     if (g_imu_can_queue != NULL)
        //     {
        //         CanRxMsg_t m;
        //         m.id = stdid;
        //         m.dlc = hcan->pRxMsg->DLC;
        //         memcpy(m.data, d, 8);
        //
        //         BaseType_t hpw = pdFALSE;
        //         xQueueSendFromISR(g_imu_can_queue, &m, &hpw);
        //         portYIELD_FROM_ISR(hpw);
        //     }
        //
        //     __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
        //     HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
        //     return;
        // }
        // if (stdid == 0x98 && hcan->pRxMsg->DLC == 8)
        // {
        //     if (g_xuc_can_queue != NULL)
        //     {
        //         CanRxMsg_t m;
        //         m.id = stdid;
        //         m.dlc = hcan->pRxMsg->DLC;
        //         memcpy(m.data, d, 8);
        //
        //         BaseType_t hpw = pdFALSE;
        //         xQueueSendFromISR(g_xuc_can_queue, &m, &hpw);
        //         portYIELD_FROM_ISR(hpw);
        //     }
        //
        //     __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
        //     HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
        //     return;
        // }
        uint8_t motor_id = ResolveDmMotorId(d);

        can2_dm_last_stdid = stdid;
        can2_dm_last_d0 = d[0];
        can2_dm_last_motor_id = motor_id;

        if (motor_id >= 1 && motor_id <= 5) {
            can2_dm_cnt[motor_id]++;
            memcpy(can2.jointidata[motor_id - 1], d, 8);
        }

        if (stdid >= 0x201) {
            uint32_t idx = stdid - 0x201;
                if (idx < 12) {
                    memcpy(can2.data[idx], d, 8);
                }
            }
        

        __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
        HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
        return;
    }

    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
    HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}

extern "C" void CAN1_TX_IRQHandler() { HAL_CAN_IRQHandler(&can1.hcan); }
extern "C" void CAN1_RX0_IRQHandler() { HAL_CAN_IRQHandler(&can1.hcan); }
extern "C" void CAN2_TX_IRQHandler() { HAL_CAN_IRQHandler(&can2.hcan); }
extern "C" void CAN2_RX0_IRQHandler() { HAL_CAN_IRQHandler(&can2.hcan); }

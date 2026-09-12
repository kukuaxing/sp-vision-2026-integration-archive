#include "imu.h"
#include "label.h"
#include "task.h"
#include <cstring>

void IMU::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type)
{
    this->type = type;
    m_uart = huart;
    if (m_uart != nullptr) {
        m_uart->Init(Instance, BaudRate).DMARxInit(nullptr);
        queueHandler = &m_uart->UartQueueHandler;
    }
    else {
        queueHandler = NULL;
    }

    m_canQueue = NULL;
    m_lastRxTick = 0;
    ClearData();
}

void IMU::InitCAN(QueueHandle_t canQueue, uint16_t can_id, float scale, IMU_TYPE type)
{
    this->type = type;
    this->m_canQueue = canQueue;
    this->m_can_id = can_id;
    this->m_scale = (scale > 0.0f) ? scale : 100.0f;

    m_uart = nullptr;
    queueHandler = NULL;
    m_lastRxTick = 0;
    ClearData();
}

void IMU::Decode()
{
    if (type != CH010 || queueHandler == NULL || *queueHandler == NULL) {
        pd_Rx = pdFALSE;
        (void)IsDataValid();
        return;
    }

    uint8_t frame[UART_MAX_LEN]{};
    if (xQueueReceive(*queueHandler, frame, 0) != pdTRUE) {
        pd_Rx = pdFALSE;
        (void)IsDataValid();
        return;
    }

    // 串口协议按 CAN 的 8 字节数据格式解析：
    // [0..1]=yaw_q, [2..3]=pitch_q, [4..5]=roll_q
    const int16_t y_q = rd_i16_le(&frame[0]);
    const int16_t p_q = rd_i16_le(&frame[2]);
    const int16_t r_q = rd_i16_le(&frame[4]);

    angle.yaw = (float)y_q / m_scale;
    angle.pitch = (float)p_q / m_scale;
    angle.roll = (float)r_q / m_scale;
    angularvelocity = {};
    acceleration = {};
    m_lastRxTick = xTaskGetTickCount();
    m_dataValid = true;
    pd_Rx = pdTRUE;

    // 保留 IMU CAN 通讯接口（当前改为串口，暂不启用）
    // if (type != CH010 || m_canQueue == NULL) {
    //     ClearData();
    //     return;
    // }

    // CanRxMsg_t msg{};
    // CanRxMsg_t last{};
    // bool got = false;

    // while (xQueueReceive(m_canQueue, &msg, 0) == pdTRUE) {
    //     if (msg.id == m_can_id && msg.dlc == 8) {
    //         last = msg;
    //         got = true;
    //     }
    // }

    // if (got) {
    //     int16_t y_q = rd_i16_le(&last.data[0]);
    //     int16_t p_q = rd_i16_le(&last.data[2]);
    //     int16_t r_q = rd_i16_le(&last.data[4]);

    //     angle.yaw = (float)y_q / m_scale;
    //     angle.pitch = (float)p_q / m_scale;
    //     angle.roll = (float)r_q / m_scale;
    //     angularvelocity = {};
    //     acceleration = {};
    //     m_lastRxTick = xTaskGetTickCount();
    //     m_dataValid = true;
    //     pd_Rx = pdTRUE;
    //     return;
    // }

    // pd_Rx = pdFALSE;
    // (void)IsDataValid();
}

bool IMU::Check(uint8_t* pdata, uint8_t len, uint32_t com)
{
    (void)pdata; (void)len; (void)com;
    return true;
}

int16_t IMU::getword(uint8_t HighBit, uint8_t LowBits)
{
    return (int16_t)((HighBit << 8) | LowBits);
}

bool IMU::IsDataValid()
{
    if (!m_dataValid) {
        return false;
    }

    if ((TickType_t)(xTaskGetTickCount() - m_lastRxTick) > m_timeoutTicks) {
        ClearData();
        return false;
    }

    return true;
}

void IMU::ClearData()
{
    angle = {};
    angularvelocity = {};
    acceleration = {};
    m_dataValid = false;
    pd_Rx = pdFALSE;
}

float IMU::GetAngleYaw() { return IsDataValid() ? angle.yaw : 0.0f; }
float IMU::GetAnglePitch() { return IsDataValid() ? angle.pitch : 0.0f; }
float IMU::GetAngleRoll() { return IsDataValid() ? angle.roll : 0.0f; }
float IMU::getangularvelocitypitch() { return IsDataValid() ? angularvelocity.pitch : 0.0f; }

float* IMU::GetAcceleration()
{
    static float temp[3];
    if (!IsDataValid()) {
        temp[0] = 0.0f;
        temp[1] = 0.0f;
        temp[2] = 0.0f;
        return temp;
    }

    temp[0] = acceleration.x;
    temp[1] = acceleration.y;
    temp[2] = acceleration.z;
    return temp;
}

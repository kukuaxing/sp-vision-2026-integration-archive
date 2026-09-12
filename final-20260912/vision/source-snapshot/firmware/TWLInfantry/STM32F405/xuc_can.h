#pragma once

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "imu.h"   // ✅ 复用 CanRxMsg_t 定义
#include "judgement.h"

// XUC CAN 队列
extern QueueHandle_t g_xuc_can_queue;

// 上下位机串口通信协议
#pragma pack(push, 1)
struct TxPacket_TJ   // 下位机发出的数据
{
    uint8_t head[2] = { 'S', 'P' };
    uint8_t mode_TJ = 0;            // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    uint8_t robot_id = 0;     //3为红方，103为蓝方
    float bullet_speed_TJ = 0.0f;
    uint16_t bullet_count_TJ = 0;   // 子弹累计发送次数
    uint16_t crc16_TJ = 0;
};

struct RxPacket_TJ   // 下位机接收的数据
{
    uint8_t head[2] = { 'S', 'P' }; // 包头
    uint8_t control_TJ = 0;         // 是否控制云台
    uint8_t shoot_TJ = 0;           // 是否开火
    float yaw_TJ = 0.0f;            // 目标 yaw 角(rad)
    float pitch_TJ = 0.0f;          // 目标 pitch 角(rad)
    float imu_pitch_TJ = 0.0f;      // IMU 俯仰角(rad)
    float imu_yaw_TJ = 0.0f;        // IMU 航向角(rad)
    uint16_t crc16_TJ = 0;
};
#pragma pack(pop)

static_assert(
    sizeof(TxPacket_TJ) == 12,
    "TxPacket_TJ must be 12 bytes");

static_assert(
    sizeof(RxPacket_TJ) == 22,
    "RxPacket_TJ must be 22 bytes");

class XUC
{
public:
    typedef struct { float yaw, pitch; } TargetAngle;

    TxPacket_TJ Tx_TJ{};
    RxPacket_TJ Rx_TJ{};

    void Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate);
    void InitCAN(QueueHandle_t canQueue, uint16_t can_id = 0x98, float scale = 100.0f);
    void Decode();

    // 只负责打包，不发送
    void Encode();

    float GetImuYaw() { return Rx_TJ.imu_yaw_TJ; }
    float GetImuPitch() { return Rx_TJ.imu_pitch_TJ; }
    float GetTargetYaw() { return target.yaw; }
    float GetTargetPitch() { return target.pitch; }
    bool ControlRequested() const { return Rx_TJ.control_TJ != 0; }
    bool IMUDataValid();                 // 包含数值检查和通信超时检查
    uint8_t fire_auto = 0;
public:
    // 发送数组，外部可直接取用
    uint8_t tx_data[sizeof(TxPacket_TJ)] = { 0 };

private:
    uint8_t frame[UART_MAX_LEN]{};
    TargetAngle target{};
    UART* m_uart = nullptr;
    QueueHandle_t* queueHandler = NULL;
    QueueHandle_t m_canQueue = NULL;
    uint16_t m_can_id = 0x98;
    float m_scale = 100.0f;
    uint32_t m_lastRxTick = 0;
    bool m_hasValidPacket = false;
    static constexpr uint32_t RX_TIMEOUT_MS = 200;

    void StopAutoAim();

private:
    static inline int16_t rd_i16_le(const uint8_t* p)
    {
        return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    }
    float u8_to_float(uint8_t* p) {
        float s{}; uint8_t ch[4];
        ch[0] = p[3]; ch[1] = p[2]; ch[2] = p[1]; ch[3] = p[0];

        memcpy(&s, p, 4); return s;
    }
};

extern XUC xuc;
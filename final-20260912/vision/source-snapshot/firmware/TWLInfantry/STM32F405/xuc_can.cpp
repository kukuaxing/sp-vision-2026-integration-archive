#include "xuc_can.h"
#include "control.h"
#include "judgement.h"
#include "imu.h"
#include "CRC.h"
#include "RC.h"
#include "HTmotor.h"
#include <string.h>
#include <cmath>

extern int CNT_1; // 全局系统计时（ms）

void XUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
    m_uart = huart;
    if (m_uart != nullptr) {
        m_uart->Init(Instance, BaudRate).DMARxInit(nullptr);
        queueHandler = &m_uart->UartQueueHandler;
    }
    else {
        queueHandler = NULL;
    }

    m_canQueue = NULL;
    target = {};
    fire_auto = 0;
    m_lastRxTick = 0;
    m_hasValidPacket = false;
}

void XUC::InitCAN(QueueHandle_t canQueue, uint16_t can_id, float scale)
{
    m_uart = nullptr;
    queueHandler = NULL;
    m_canQueue = canQueue;
    m_can_id = can_id;
    m_scale = scale;
}

void XUC::Decode()
{
    if (queueHandler == NULL || *queueHandler == NULL) {
        return;
    }

    if (xQueueReceive(*queueHandler, frame, 0) != pdPASS) {
        // 右键释放后立即退出自瞄，不必等待通信超时。
        if (rc.pc.press_r != 1) {
            StopAutoAim();
        }

        if ((uint32_t)(CNT_1 - m_lastRxTick) > RX_TIMEOUT_MS) {
            m_hasValidPacket = false;
            StopAutoAim();
        }
        return;
    }

    const uint32_t packLen = (uint32_t)sizeof(RxPacket_TJ);
    for (uint32_t i = 0; i + packLen <= UART_MAX_LEN; ++i)
    {
        if (frame[i] != 'S' || frame[i + 1] != 'P') {
            continue;
        }

        uint8_t packet[sizeof(RxPacket_TJ)]{};
        memcpy(packet, &frame[i], sizeof(RxPacket_TJ));

        if (!VerifyCRC16CheckSum(packet, (uint32_t)sizeof(RxPacket_TJ))) {
            continue;
        }

        memcpy(&Rx_TJ, packet, sizeof(RxPacket_TJ));
        m_hasValidPacket = true;
        m_lastRxTick = (uint32_t)CNT_1;

        target.yaw = Rx_TJ.yaw_TJ;
        target.pitch = Rx_TJ.pitch_TJ;
        fire_auto =
            (Rx_TJ.control_TJ != 0 && Rx_TJ.shoot_TJ != 0) ? 1 : 0;

        const bool enable_auto =
            Rx_TJ.control_TJ != 0 &&
            rc.pc.press_r == 1 &&
            IMUDataValid();

        if (enable_auto) {
            if (can1_motor[7].mode != POS_IMU) {
                constexpr float RAD2DEG = 57.29577951308232f;
                can1_motor[7].setangle = Rx_TJ.imu_yaw_TJ * RAD2DEG;
                DMmotor[2].setPos = DMmotor[2].pos;
            }
            can1_motor[7].mode = POS_IMU;
        }
        else {
            StopAutoAim();
        }

        return;
    }

    if ((uint32_t)(CNT_1 - m_lastRxTick) > RX_TIMEOUT_MS) {
        m_hasValidPacket = false;
        StopAutoAim();
    }
}


    // 保留 XUC CAN 通讯接口（当前改为串口，暂不启用）
    // if (m_canQueue == NULL) return;

    // CanRxMsg_t msg{};
    // CanRxMsg_t last{};
    // bool got = false;

    // while (xQueueReceive(m_canQueue, &msg, 0) == pdTRUE) {
    //     if (msg.id == m_can_id && msg.dlc == 8) {
    //         last = msg;
    //         got = true;
    //     }
    // }

    // if (!got) return;

    // int16_t y_q = rd_i16_le(&last.data[0]);
    // int16_t p_q = rd_i16_le(&last.data[2]);
    // fire_auto   = last.data[4];
    // target.yaw = (float)y_q / m_scale;
    // target.pitch = (float)p_q / m_scale;
//}

void XUC::StopAutoAim()
{
    target = {};
    fire_auto = 0;
    Rx_TJ.control_TJ = 0;
    Rx_TJ.shoot_TJ = 0;

    if (can1_motor[7].mode == POS_IMU) {
        can1_motor[7].mode = POS;
        can1_motor[7].setangle = can1_motor[7].angle[now];
        ctrl.pantile.mark_yaw = can1_motor[7].setangle;

        if (std::isfinite(DMmotor[2].pos)) {
            DMmotor[2].setPos = DMmotor[2].pos;
        }
    }
}

bool XUC::IMUDataValid()
{
    if (!m_hasValidPacket) {
        return false;
    }

    if ((uint32_t)(CNT_1 - m_lastRxTick) > RX_TIMEOUT_MS) {
        return false;
    }

    const float yaw = Rx_TJ.imu_yaw_TJ;
    const float pitch = Rx_TJ.imu_pitch_TJ;

    if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
        return false;
    }

    // 两个角度同时严格为零也是合法姿态，不能作为失效判据。
    return true;
}


void XUC::Encode()
{
    if (m_uart == nullptr) {
        return;
    }

    static uint16_t bullet_count = 0;

    memset(&Tx_TJ, 0, sizeof(Tx_TJ));

    Tx_TJ.head[0] = 'S';
    Tx_TJ.head[1] = 'P';

    // 模式：0 空闲，1 自瞄，2 小符，3 大符
    // 右键是操作员的自瞄授权，向上位机回传真实请求模式。
    Tx_TJ.mode_TJ = (rc.pc.press_r == 1) ? 1 : 0;

    // 机器人ID
    Tx_TJ.robot_id = judgement.data.robot_status_t.robot_id;

    // 使用裁判系统弹速
    const float bullet_speed = judgement.data.shoot_data_t.bullet_speed;
    Tx_TJ.bullet_speed_TJ = std::isfinite(bullet_speed) ? bullet_speed : 0.0f;
    Tx_TJ.bullet_count_TJ = bullet_count++;

    // 计算数据包的总大小
    const uint32_t packet_size = (uint32_t)sizeof(TxPacket_TJ);

    // 将数据包复制到发送缓冲区并附加 CRC16（与上位机一致）
    memcpy(tx_data, &Tx_TJ, packet_size);
    AppendCRC16CheckSum(tx_data, packet_size);

    // 同步回结构体，便于调试观察 crc 字段
    memcpy(&Tx_TJ, tx_data, packet_size);

    // 发送数据
    m_uart->UARTTransmit(tx_data, packet_size);
}
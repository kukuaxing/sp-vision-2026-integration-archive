#include "HTmotor.h"
#include "delay.h"
#include "can.h"
#include "control.h"
#include "stdio.h"
#include <cstring>      // memcpy

// ===== FreeRTOS mutex for protecting can2.jointpdata =====
#include "FreeRTOS.h"
#include "semphr.h"

#define now  0
#define last 1

// 用于保护 can2.jointpdata 的互斥量（全局一个即可）
static SemaphoreHandle_t g_can2PdataMutex = nullptr;

// 确保互斥量只创建一次（在任务上下文调用）
static inline void EnsureCan2PdataMutex()
{
    if (g_can2PdataMutex == nullptr) {
        g_can2PdataMutex = xSemaphoreCreateMutex();
        // 可选：创建失败时断言
        // configASSERT(g_can2PdataMutex);
    }
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int16_t* index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t* index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

uint32_t DMMOTOR::GetControlStdId() const
{
    switch (function)
    {
    case MIT:
        return ID;
    case SPD_DM:
        return ID + 0x200;
    case POS_DM:
        return ID + 0x100;
    default:
        return ID;
    }
}

DMMOTOR& DMMOTOR::State_Decode(CAN hcan, uint8_t idata[][8])
{
    uint8_t* d = idata[ID - 1];
    err = static_cast<uint8_t>((d[0] >> 4) & 0x0F);

    uint16_t tmp16 = 0;
    uint16_t tmp12 = 0;

    // 位置：16bit 有符号量化
    tmp16 = (static_cast<uint16_t>(d[1]) << 8) | d[2];
    pos = uint_to_float(tmp16, P_MIN, P_MAX, 16);
    angle[last] = angle[now];
    angle[now] = pos;

    // 速度：12bit 有符号量化
    tmp12 = (static_cast<uint16_t>(d[3]) << 4) | (d[4] >> 4);
    curSpeed = uint_to_float(tmp12, V_MIN, V_MAX, 12);

    // 扭矩：12bit 有符号量化
    tmp12 = (static_cast<uint16_t>(d[4] & 0x0F) << 8) | d[5];
    torque = uint_to_float(tmp12, T_MIN, T_MAX, 12);
    current = torque / KT;

    return *this;
}


void DMMOTOR::DMmotor_transmit()
{
    if (!ctrl.init_DM) {
        return;
    }

    EnsureCan2PdataMutex();

    uint32_t idx = ID - 0x01;             // 0..4 对应 0x01..0x05
    if (idx >= 6) return;                 // jointpdata[6][8] 边界保护

    // 先将共享 8 字节拷贝到局部，避免发送过程中被更新任务改写
    uint8_t txbuf[8];

    if (g_can2PdataMutex) {
        xSemaphoreTake(g_can2PdataMutex, portMAX_DELAY);
        std::memcpy(txbuf, can2.jointpdata[idx], 8);
        xSemaphoreGive(g_can2PdataMutex);
    }
    else {
        std::memcpy(txbuf, can2.jointpdata[idx], 8);
    }

    can2.Transmit(GetControlStdId(), txbuf, 8);
}

void DMMOTOR::DMmotorinit()
{
    EnsureCan2PdataMutex();

    for (uint32_t i = 0; i < 3; ++i) {          // 使能前 3 个 ID 电机
        DMmotor[i].Motor_Start(can2, DMmotor[i].GetControlStdId());
        //delay.delay_ms(2);
    }
    ctrl.init_DM = 1;
}


void DMMOTOR::SetTorque(float settorque)
{
    setTorque = settorque;
}

float DMMOTOR::GetPosition()
{
    return angle[now];
}

float DMMOTOR::GetSpeed()
{
    return curSpeed;
}

float DMMOTOR::GetTorque()
{
    return torque;
}

void DMMOTOR::Motor_Start(CAN hcan, uint32_t id)
{
    CanComm_ControlCmd(hcan, CMD_MOTOR_MODE, id);
}

void DMMOTOR::Motor_Stop(CAN hcan, uint32_t id)
{
    CanComm_ControlCmd(hcan, CMD_RESET_MODE, id);
}

void DMMOTOR::CanComm_ControlCmd(CAN hcan, uint8_t cmd, uint32_t id)// 控制指令封装
{
    uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00 };

    switch (cmd)
    {
    case CMD_MOTOR_MODE:
        buf[7] = 0xFC; // 进入电机模式
        break;

    case CMD_RESET_MODE:
        buf[7] = 0xFD; // 退出电机模式
        break;

    case CMD_ZERO_POSITION:
        buf[7] = 0xFE; // 保存位置零点
        break;

    case CMD_CLEAR_MODE:
        buf[7] = 0xFB; // 清除错误
        break;

    default:
        return;
    }

    hcan.Transmit(id, buf, 8);
}

float DMMOTOR::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int DMMOTOR::float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DMMOTOR::DMmotor_Ontimer(CAN hcan, float f_kp, float f_kd, uint8_t* odata)
{
    EnsureCan2PdataMutex();

    uint32_t p = 0, v = 0, kp = 0, kd = 0, t = 0;

    // 先计算到局部数组，减少持锁时间
    uint8_t out[8] = { 0 };

    switch (function)
    {
    case MIT:
        LIMIT_MIN_MAX(setPos, P_MIN, P_MAX);
        LIMIT_MIN_MAX(setSpeed, V_MIN, V_MAX);
        LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
        LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
        LIMIT_MIN_MAX(setTorque, T_MIN, T_MAX);

        p = float_to_uint(setPos, P_MIN, P_MAX, 16);
        v = float_to_uint(setSpeed, V_MIN, V_MAX, 12);
        kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
        kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
        t = float_to_uint(setTorque, T_MIN, T_MAX, 12);

        out[0] = p >> 8;
        out[1] = p & 0xFF;
        out[2] = v >> 4;
        out[3] = ((v & 0xF) << 4) | (kp >> 8);
        out[4] = kp & 0xFF;
        out[5] = kd >> 4;
        out[6] = ((kd & 0xF) << 4) | (t >> 8);
        out[7] = t & 0xff;
        break;

    case POS_DM:
    {
        const float pos_cmd = std::isfinite(setPos) ? setPos : 0.0f;
        const float speed_cmd = std::isfinite(setSpeed) ? setSpeed : 0.0f;
        std::memcpy(&out[0], &pos_cmd, sizeof(pos_cmd));
        std::memcpy(&out[4], &speed_cmd, sizeof(speed_cmd));
        break;
    }

    case SPD_DM:
    {
        const float speed_cmd = std::isfinite(setSpeed) ? setSpeed : 0.0f;
        std::memcpy(&out[0], &speed_cmd, sizeof(speed_cmd));
        break;
    }

    default:
        return;
    }

    // 将 out 一次性写回共享 odata（受 mutex 保护）
    if (g_can2PdataMutex) {
        xSemaphoreTake(g_can2PdataMutex, portMAX_DELAY);
        std::memcpy(odata, out, 8);
        xSemaphoreGive(g_can2PdataMutex);
    }
    else {
        std::memcpy(odata, out, 8);
    }
}

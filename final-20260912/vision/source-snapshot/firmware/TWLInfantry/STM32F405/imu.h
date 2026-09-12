#pragma once
#include "stm32f4xx.h"
#include "usart.h"
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

enum IMU_TYPE { IMU601 = 0, CH010, HI226 };

// CAN RX message
typedef struct
{
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CanRxMsg_t;

// IMU CAN queue: created in main, written in can.cpp ISR, read in IMU::Decode().
extern QueueHandle_t g_imu_can_queue;

class IMU
{
public:
    float ACC_FSR = 4.f, GYRO_FSR = 2000.f;

    typedef struct { float roll, yaw, pitch; } Angle, AngularVelocity;
    typedef struct { float x{}, y{}, z{}; } Acceleration;

    void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type);
    void InitCAN(QueueHandle_t canQueue, uint16_t can_id = 0x99, float scale = 100.0f, IMU_TYPE type = CH010);

    void Decode();

    bool Check(uint8_t* pdata, uint8_t len, uint32_t com);
    int16_t getword(uint8_t HighBit, uint8_t LowBits);

    float GetAngleYaw();
    float GetAnglePitch();
    float GetAngleRoll();
    float getangularvelocitypitch();
    float* GetAcceleration();
    bool IsDataValid();
    void ClearData();

    BaseType_t pd_Rx = false;

    QueueHandle_t* queueHandler = NULL;

private:
    Angle angle{};
    AngularVelocity angularvelocity{};
    Acceleration acceleration{};
    uint16_t crc = 0, len = 0;
    IMU_TYPE type = CH010;

    uint8_t rxData[UART_MAX_LEN]{};
    UART* m_uart = nullptr;

    QueueHandle_t m_canQueue = NULL;
    uint16_t m_can_id = 0x99;
    float m_scale = 100.0f;
    TickType_t m_lastRxTick = 0;
    TickType_t m_timeoutTicks = pdMS_TO_TICKS(50);
    bool m_dataValid = false;

private:
    static inline int16_t rd_i16_le(const uint8_t* p)
    {
        return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    }
};

static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static float    R4(uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }

extern IMU imu_chassis, imu_pantile;

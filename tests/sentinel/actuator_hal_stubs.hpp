#pragma once
// Device/RTOS replacements only. Tests compile actual PID, Motor, DM motor,
// CAN feedback routing and CONTROL::SHOOTER code and their real headers.
#include <cstdint>
#include <cstring>
#include <vector>
using TaskHandle_t = void*;
using BaseType_t = int;
inline uint32_t actuator_now_ms = 1;
inline uint32_t HAL_GetTick() { return actuator_now_ms; }
#define taskENTER_CRITICAL() do {} while (0)
#define taskEXIT_CRITICAL() do {} while (0)
struct CAN_TypeDef {};
struct CanTxMsgTypeDef {};
struct CanRxMsgTypeDef {};
struct CAN_HandleTypeDef {};
using HAL_StatusTypeDef = int;
struct IMU {
    float yaw = 0;
    float GetAngleYaw() { return yaw; }
};
extern IMU imu_pantile, imu_chassis;
struct DELAY { void delay_ms(uint32_t) {} };
extern DELAY delay;

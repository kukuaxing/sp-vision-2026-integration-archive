#pragma once
#define __CAN_H__
#include <cstdint>
#include <cstdlib>
class CAN { public: uint8_t DMmotor_data[2][8]{}; uint8_t data[12][8]{}; int Transmit(uint32_t, const uint8_t*, uint8_t=8); };
extern CAN can1, can2;
uint32_t HAL_GetTick();

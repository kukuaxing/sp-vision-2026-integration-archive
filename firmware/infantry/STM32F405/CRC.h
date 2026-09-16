#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>
uint8_t GetCRC8CheckSum(uint8_t* pchMessage, uint32_t dwLength, uint8_t ucCRC8);
uint8_t VerifyCRC8CheckSum(uint8_t* pchMessage, uint32_t dwLength);
void AppendCRC8CheckSum(uint8_t* pchMessage, uint32_t dwLength);
uint16_t GetCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t VerifyCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength);
void AppendCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength);
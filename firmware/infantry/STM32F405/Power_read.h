#pragma once
#pragma once
#include "usart.h"
#include "CRC.h"
#include <string.h>

#define BUFSIZE    100
#define BUFF_SIZE  100
#define TX_SIZE    100

class POWER
{
public:
    void Receive();
    void Init(UART* huart, USART_TypeDef* uart_base, uint32_t baud);
    void Send();  // DMA·Ç×èÈû·¢ËÍ
    void Decode();
    uint8_t m_uartrx[BUFF_SIZE] = { 0 };
    uint8_t m_uarttx[TX_SIZE] = { 0 };
    UART* m_uart = nullptr;
    float power_now;
private:
    BaseType_t pd_Rx = false;
    QueueHandle_t* queueHandler = NULL;
};

extern "C" POWER power;

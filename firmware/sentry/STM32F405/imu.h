#pragma once
#include "stm32f4xx.h"
#include "usart.h"
#include <string.h>
enum IMU_TYPE { IMU601 = 0, CH010, HI226 };

class IMU
{
public:
	float ACC_FSR = 4.f, GYRO_FSR = 2000.f;
	typedef struct
	{
		float roll, yaw, pitch;
	}Angle, AngularVelocity;
	typedef struct
	{
		float x{}, y{}, z{};
	}Acceleration;

	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type);
	void Decode();
	bool Check(uint8_t* pdata, uint8_t len, uint32_t com);
	float GetAngleYaw();
	float GetAnglePitch();
	float GetAngleRoll();
	float getangularvelocitypitch();
	Acceleration GetAcceleration() const;
	void GetQ();
	int16_t getword(uint8_t HighBit, uint8_t LowBits);
	Angle angle;
	AngularVelocity angularvelocity;
	Acceleration acceleration;

	BaseType_t pd_Rx = false;
	QueueHandle_t* queueHandler = NULL;
private:

	uint16_t crc, len;
	IMU_TYPE type;

	uint8_t rxData[UART_MAX_LEN];
	UART* m_uart;

};

static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static float    R4(uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }

extern IMU imu_pantile, imu_gimbal;

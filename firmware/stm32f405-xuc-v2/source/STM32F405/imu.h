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
	bool Check(uint8_t* pdata, uint16_t len, uint32_t com);
	float GetAngleYaw();
	float GetAnglePitch();
	float GetStrictAngleYaw() const;
	float GetStrictAnglePitch() const;
	float GetAngleRoll();
	float getangularvelocitypitch();
	float GetAngularVelocityYaw(); // [后续修改] 提供yaw角速度给小陀螺锁角前馈使用
	bool HasFreshData(uint32_t now_ms, uint32_t timeout_ms = 100U) const;
	bool HasFreshStrictData(uint32_t now_ms, uint32_t timeout_ms = 100U) const;
	Acceleration GetAcceleration();
	int16_t getword(uint8_t HighBit, uint8_t LowBits);

	BaseType_t pd_Rx = false;
	QueueHandle_t* queueHandler = NULL;
	uint8_t parse_ok = 0; // [后续修改] 0未解析，1 CRC正确解析，2 CRC异常但帧类型可用
	uint16_t frame_offset = 0, frame_len = 0; // [后续修改] 记录本次有效IMU帧在DMA缓冲区中的偏移和长度
	uint32_t decode_count = 0, crc_error_count = 0, header_error_count = 0; // [后续修改] IMU调试计数器，用于判断是否持续解码
	uint32_t last_decode_ms = 0, last_strict_decode_ms = 0; // 最近任意解析/严格CRC解析时间
	uint16_t crc_calc = 0, crc_recv = 0; // [后续修改] 保存计算CRC和接收CRC，便于排查协议/字节序问题
private:
	Angle angle;
	Angle strict_angle; // 最近一次严格CRC通过的角度，供自动控制和上位机反馈
	AngularVelocity angularvelocity;
	Acceleration acceleration;
	uint16_t crc, len;
	IMU_TYPE type;

	uint8_t rxData[UART_MAX_LEN];
	UART* m_uart;

};

static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static float    R4(uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }

extern IMU imu_chassis, imu_pantile;

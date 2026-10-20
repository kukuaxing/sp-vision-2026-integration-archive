#pragma once
#include "usart.h"
#include "FreeRTOS.h"
#include <cmath>
#include <cinttypes>

#define RC_STATE(s0, s1) ( ((s0) << 8) | (s1) )
#define TIMEOUT_TICKS 50        // 50 × 5ms = 250ms
#define RC_LOST_MS 100 

/*
左拨码s[0],右拨码s[1]
上：1 中：3 下：2

右摇杆 上下 ch[1]
右摇杆 左右 ch[0]
左摇杆 上下 ch[3]

*/

class RC
{
public:
	int gear;
	bool top_mode = true;
	bool fix = false;


	struct
	{
		int16_t ch[4];
		uint8_t s[2];
	}rc, pre_rc;

	enum POSITION { UP = 1, DOWN, MID };
	struct PC
	{
		int16_t x, y, z;
		uint8_t press_l, press_r;

		uint8_t key_h, key_l;
		const float spdratio = 1.f;
	}pc;

	uint8_t* GetDMARx(void) { return m_frame; }

	bool judement_start = false;
	void Decode();
	void OnRC();
	void OnPC();
	void Update();
	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate);
	bool Shift_mode();
	void RC_CheckState();
	void RC_Control();

	/* --- 上位机（V2）分段处理，供 watch/调试 --- */
	void ApplyNav();       // 0x11 NAV_COMMAND → 底盘 speedx/speedy
	void ApplyAim();       // 0x10 AIM_COMMAND → 云台 mark_pitch / setangle
	void StopChassis();    // 底盘速度归零 + 平滑量复位

	uint16_t rc_lost_cnt = 0;    // 失联次数，上车 watch 用

private:
	QueueHandle_t* queueHandler = NULL;
	BaseType_t pd_Rx, pd_Tx;
	UART* m_uart;
	uint8_t m_frame[UART_MAX_LEN]{};
	uint32_t last_rc_tick = 0;   // 最后一次收到【有效】DBUS 帧的时刻
};

extern RC rc;

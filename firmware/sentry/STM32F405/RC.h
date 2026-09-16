#pragma once
#include "usart.h."
#include "FreeRTOS.h"
#include <cmath>
#include <cinttypes>

/*
左拨码s[0],右拨码s[1]
上：1 中：3 下：2

右摇杆 上下 ch[1]
右摇杆 左右 ch[0]
左摇杆 左右 ch[2]
左摇杆 上下 ch[3]

*/

class RC
{
public:
  struct ODOMETRY
	{
		float vx_mps = 0.0f;
		float vy_mps = 0.0f;
		float wz_rpm = 0.0f;
		float step_distance_m = 0.0f;
		float total_distance_m = 0.0f;
		TickType_t last_tick = 0;
	} odometry;

	int8_t AUTOmode{};
	enum AutoMode
	{
		Disconnected = 0, // 未连接
		Move = 1,
		Attack = 2,
		Defense = 3
	};
	AutoMode    prev_state       = Disconnected; // 上一个状态
	TickType_t  state_start_tick = 0;            // 进入当前状态的时刻（FreeRTOS tick）
	uint32_t    state_duration_ms = 0;           // 当前状态已持续时间（ms）

	// 姿态切换管理
	static constexpr uint32_t POSTURE_COOLDOWN_MS        = 5000;    // 5s 切换冷却
	static constexpr uint32_t POSTURE_DECAY_THRESHOLD_MS = 180000;  // 累计 3min 衰减阈值
	TickType_t  last_switch_tick      = 0;
	uint32_t    posture_cd_remain_ms  = 0;
	uint32_t    cumulative_move_ms    = 0;          // 移动姿态累计（ms）
	uint32_t    cumulative_attack_ms  = 0;          // 进攻姿态累计（ms）
	uint32_t    cumulative_defense_ms = 0;          // 防御姿态累计（ms）
	bool        posture_decayed       = false;      // 当前姿态核心增益是否已衰减
	int8_t      pending_state         = -1;         // 冷却期间缓存的切换请求，-1=无

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
	void OnAuto();
	void OnPC();
  void OnRMUL();
	void Update();
	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate);
	bool Shift_mode();
  void UpdateTravelEstimateFromMotor();
	void ResetTravelEstimate();
 void ResetRMUL();
	bool HasReceivedValidFrame() const { return has_received_valid_frame; }

private:
 enum RMUL_STAGE
	{
		RMUL_IDLE = 0,
		RMUL_MOVE_POS_X,
		RMUL_MOVE_POS_Y,
		RMUL_MOVE_NEG_X,
		RMUL_GYRO_AIM
	};

	RMUL_STAGE rmul_stage = RMUL_IDLE;
	float rmul_accum_m = 0.0f;
	TickType_t rmul_last_tick = 0;
	TickType_t rmul_stage_start_tick = 0;
	bool rmul_yaw_ramp_active = false;
	float rmul_yaw_start_deg = 0.0f;
	TickType_t rmul_yaw_ramp_start_tick = 0;
	float rmul_yaw_target_deg = 0.0f;
	bool rmul_active = false;

	QueueHandle_t* queueHandler = NULL;
	BaseType_t pd_Rx, pd_Tx;
	UART* m_uart;
	uint8_t m_frame[UART_MAX_LEN]{};
  bool has_received_valid_frame = false;
};

extern RC rc;

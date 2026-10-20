#pragma once
#include <vector>
#include <cmath>
#ifdef SENTINEL_ACTUATOR_HOST_TEST
#include "tests/sentinel/actuator_hal_stubs.hpp"
#else
#include "stm32f4xx.h"
#include "imu.h"
#endif
#include "motor.h"
#include "aim_position_reference.h"

class CONTROL final
{
public:
	uint8_t chassis_num{}, pantile_num{}, shooter_num{}, supply_num{};
	uint8_t init_DM = 0;
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* pantile_motor[PANTILE_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};

	enum MODE { PC = 0, RC, AUTO, RESET, ROTATION, SPINNING, FOLLOW, SEPARATE, FIRE, STOP, LOCK } mode = STOP;
	struct CHASSIS
	{


		PID chassis_reset{};
		int32_t speedx{}, speedy{}, speedz{};

		void Keep_Direction(int32_t ch_fwd, int32_t ch_left);

		void Update();
		float Ramp(float setval, float curval, uint32_t RampSlope);
	};

	struct PANTILE
	{
		enum TYPE { YAW, PITCH };
		float mark_pitch{}, mark_yaw{};
		float yaw_speed_out = 0.0f;
		const float sensitivity = 2.5f;
		bool aim = false;
		bool pitch_ready = false;
		float pitch0 = 0.0f;
		bool hold_position = false;
		bool hold_yaw_captured = false, hold_pitch_captured = false, hold_dm_yaw_captured = false;
		AimPositionReference aim_yaw_reference, aim_pitch_reference;
		uint32_t yaw_feedback_generation = 0, pitch_feedback_generation = 0, dm_yaw_feedback_generation = 0;
		void Synchronize_Feedback_Recovery();
		void Reset_Aim_References() { aim_yaw_reference.Reset(); aim_pitch_reference.Reset(); }
		void Hold_Pantile(); // Capture decoded motor positions; never use an uninitialized zero.
		void Control_Pantile(int32_t ch_dji_yaw, int32_t ch_pitch, int32_t ch_dm_yaw);
		//void Control_Pantile(int32_t ch_yaw, int32_t ch_pitch);
		void Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference);
		void Update();

		bool  yaw_lock_ready = false;// yaw锁定准备好，这是一个自动模式需要用的一个判断变量
		float yaw_lock_pos = 0.f;
		float yaw_lock_Kp = 3.0f;    // rad/s per rad，先给 3，从小往大调
	};

	struct SHOOTER
	{

		float now_bullet_speed = 0.f;

		bool auto_shoot = false;
		// Host request before local safety gating; a forced-off output must not
		// masquerade as the low edge needed to re-arm after a fault.
		bool auto_fire_request_level = false;
		bool openRub = false, supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;
		int16_t shoot_speed = 6500;
		void Update();
		void CancelPendingFeed();
		int32_t trig_raw{}; bool trig{}; bool trig_pre{}; uint16_t hold_ms{}; uint16_t rate_ms{};// 发射机构要用的：触发器：原始值、当前值、上次值、保持时间、间隔时间
		bool feed_permission_was_ready = false;
		bool feed_rearm_required = false;
		uint32_t left_feedback_generation = 0, right_feedback_generation = 0, feeder_feedback_generation = 0;
	};

	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;

	static int16_t Setrange(const int16_t original, const int16_t range);
	static float ClampAngle(float setangle, float center, float span);
	float GetDelta(float delta);
	void Init(std::vector<Motor*> motor);
	void init_dm();
private:

};

extern CONTROL ctrl;

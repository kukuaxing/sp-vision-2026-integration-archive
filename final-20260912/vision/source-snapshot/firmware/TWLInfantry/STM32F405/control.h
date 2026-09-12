#pragma once
#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"

class CONTROL final
{
public:
	uint8_t init_DM = 0;
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* pantile_motor[PANTILE_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};

	//enum MODE { PC, RC, AUTOAIM } mode;
	enum MODE { RESET, CHASSIS_GIMBAL, CHASSIS_ONLY, FIRE_CONTROL } mode;
	struct CHASSIS
	{


		PID chassis_reset{};
		int32_t speedx{}, speedy{}, speedz{};

		void Keep_Direction();

		void Update();
		float Ramp(float setval, float curval, uint32_t RampSlope);
	};

	struct PANTILE
	{


		enum TYPE { YAW, PITCH };
		float mark_pitch{}, mark_yaw{}, mark_yaw_imu{};
		PID pantile_PID[3] = { {0.04f,0.f,0.f},{0.05f,0.f,0.f}, {0.f,0.f,0.f} };
		const float sensitivity_yaw = 0.18f;
		const float sensitivity_pitch = (PI / 360.f) * 0.1f;

		bool aim = false;
		void Keep_Pantile(float angleKeep, PANTILE::TYPE type);
		void Update();
	};

	struct SHOOTER
	{

		float now_bullet_speed = 0.f;

		bool auto_shoot = false;
        bool displayOpenRub = false;
		bool openRub = false, supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;
		bool jam_block = false;      // 卡弹反转状态，供 UI 使用
		int16_t shoot_speed = 6000;
		void Update();
	};

	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;

	static int16_t Setrange(const int16_t original, const int16_t range);
	void Control_Pantile(float ch_yaw, float ch_pitch);
	float GetDelta(float delta);
	void Init(std::vector<Motor*> motor);
private:

};

extern CONTROL ctrl;

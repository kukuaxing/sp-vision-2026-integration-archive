#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include "stm32f4xx.h"
#include "motor.h"
#include "imu.h"

class CONTROL final
{
public:
	Motor* chassis_motor[CHASSIS_MOTOR_NUM]{};
	Motor* pantile_motor[PANTILE_MOTOR_NUM]{};
	Motor* shooter_motor[SHOOTER_MOTOR_NUM]{};
	Motor* supply_motor[SUPPLY_MOTOR_NUM]{};
	
	enum MODE { ROTATION, RESET, SEPARATE, FOLLOW, LOCK, TEST, AUTO, ROTATION_FREE, SEPARATE_FREE } mode; // SEPARATE_FREE保留分离底盘控制，yaw采用自由速度控制
	struct CHASSIS
	{


		PID chassis_reset{};
		int32_t speedx{}, speedy{}, speedz{};
		bool transition_from_rotation = false; // [后续修改] 记录是否刚退出小陀螺，便于平滑切换
		MODE last_mode = RESET; // 保存上一周期底盘模式，用于判断模式切换
		bool auto_turn_active = false; // MID+UP下是否正在执行360度原地旋转
		bool auto_turn_rearmed = false; // 左摇杆回中后才允许下一次触发
		bool auto_turn_fault = false; // 编码器缺失或执行超时标志
		int8_t auto_turn_dir = 1; // 自动旋转方向：+1/-1
		int32_t auto_turn_start_yaw_sum = 0; // 任务启动时yaw电机累计编码器
		int32_t auto_turn_progress_ticks = 0; // 已旋转的累计编码器量
		uint32_t auto_turn_start_ms = 0; // 任务启动时刻，用于超时保护
		
		void Keep_Direction();
		void speedCalculate(int32_t speedx_, int32_t speedy_, int32_t speedz_);//閫熷害瑙ｇ畻骞惰瀹氬嚱鏁?
		void Update();
		void Control_AutoTurn360(int32_t turn_stick, int32_t idle_forward_speed, bool link_ok);
		void Cancel_AutoTurn360();
		float Ramp(float setval, float curval, uint32_t RampSlope);
	};

	struct PANTILE
	{
		enum TYPE { YAW, PITCH };
		float mark_pitch{}, mark_yaw{};
		float target_imu_yaw{}, target_imu_pitch{}; // [后续修改] IMU角度目标，yaw锁角时使用
		float yaw_keep_error{}, yaw_keep_output{}, yaw_rotation_ff{}, yaw_deadband{}, yaw_output_limit{}, manual_yaw_step{}, manual_yaw_input{}; // [后续修改] yaw锁角/手动控制调试量，便于Live Watch观察
		bool auto_yaw_motion_active = false; // AUTO yaw启停滞回，避免静摩擦补偿在中心附近反复换向
		float pitch_keep_error{}, pitch_keep_output{}, pitch_deadband{}, pitch_output_limit{}, manual_pitch_step{}, manual_pitch_input{}; // [后续修改] pitch自稳/手动微调调试量
		float pitch_dir = -1.0f; // [后续修改] pitch方向符号（IMU角→电机位置）
		float pitch_min = -0.25f; // [后续修改] pitch软限位下限(弧度)
		float pitch_max = 0.4f;   // [后续修改] pitch软限位上限(弧度)
		float pitch_sensitivity = 0.000005f; // [后续修改] 遥控pitch灵敏度(弧度/周期)，已降低
		PID pantile_PID[3] = { {0.08f,0.f,0.f},{0.05f,0.f,0.f}, {0.f,0.f,0.f} }; // [后续修改] 云台保持PID：0为yaw，1为pitch，2预留
		const float sensitivity = 2.5f;
		bool aim = false;
		void Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference);
		void Update();
	};

	struct SHOOTER
	{

		float now_bullet_speed = 0.f;

		bool auto_shoot = false;
		bool openRub = false, supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;
		int16_t shoot_speed = 2500; // [后续修改] 摩擦轮目标转速(RPM)
		int16_t supply_speed = 2160; // [后续修改] 拨弹轮供弹转速(RPM)
			// ===== [Next] single/continuous feed state machine =====
		bool trigger_now = false, trigger_prev = false, trigger_rise = false; // left stick edge
		int8_t single_dir = 1;      // latched single-shot direction (+1 forward, -1 reverse)
		int8_t continuous_dir = 1; // live continuous direction
		bool shot_busy = false;          // single 45deg feed in progress
		bool continuous_active = false;  // right stick continuous feed in progress
		bool waiting_wheels = false;     // waiting for friction wheels to spin up
		int32_t shot_start_angle = 0;    // accumulated encoder angle at shot start
		int32_t burst_target = 0;        // rounds for this task, 0 = unlimited
		int32_t rounds_fed = 0;          // rounds fed in current task
		int32_t last_feed_angle = 0;     // last observed feed angle
		uint32_t last_angle_time = 0;    // time of last feed progress
		bool jam_recovering = false;     // jam back-off in progress
		uint32_t jam_recover_ms = 0;
		uint32_t jam_count = 0;          // total jam count for debug
		uint32_t shot_count = 0;         // total rounds fired for debug
		bool fire_fault = false;         // protection after repeated jams
		void Update();
	};

	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;
	
	static int16_t Setrange(const int16_t original, const int16_t range);
	void Control_Pantile(int32_t ch_yaw, int32_t ch_pitch);
	void Control_Chassis(int32_t ch_speedx, int32_t ch_speedy, int32_t ch_speedz);
	float GetDelta(float delta);
	void Init(std::vector<Motor*> motor);
private:

};

extern CONTROL ctrl;

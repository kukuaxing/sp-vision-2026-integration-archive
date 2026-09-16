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
	float m_current_relative_angle;
	float m_current_relative_velocity;
	bool g_start_scan;
	unsigned int radian_to_mechanical_angle(double radian_angle);
	enum MODE { ROTATION, RESET, NORMAL,SEPARATE, FOLLOW, LOCK, TEST, AUTO,SHOOT,NAVI,AUTOCONTROL,TD,STARTUP,RC_SHOOT_TEST,NAVIT} mode;

	struct CHASSIS
	{
        struct POWER_MODEL_DEBUG
		{
			float sum_torque_power{};
			float k1_sum_abs_w{};
			float k2_sum_current_sq{};
			float k3{};
			float predicted_power_raw{};
			float predicted_power_with_safety{};
			float real_power_feedback{};
		};

		// 小陀螺到正常模式的过渡控制
		bool transition_from_rotation = false;
		float power_1{}, power_2{};
      float power_model_predicted{};
      POWER_MODEL_DEBUG power_model_debug{};
		bool DM_ready = false;
		bool flag_DM_ready= true;
		uint16_t transition_timer = 0;
		bool follow_was_rotating = false;  // FOLLOW模式底盘旋转状态追踪，用于检测松杆瞬间

		PID chassis_reset{ 1500.f, 0.f, 150.f, 0.1f };  // P: rad误差→底盘转速; D: 防过冲; 符号不对时取反输出
		int32_t speedx{}, speedy{}, speedz{};
		float navi_vx = 0.f, navi_vy = 0.f, navi_vz; //储存导航速度
		// 导航速度映射：navi[i] × v_ms → setChassisspeed 的虚拟摇杆值
		// 推导：speedmax=5000, 车轮直径=151.84mm, M3508减速比=19.2
		//   navi[0/1] = (19.2*60/(PI*0.15184)) * 660/5000 ≈ 319
		//   navi[2]   = (19.2*60/(PI*0.15184)) * 0.23191 * 660/5000 ≈ 74
		float navi[3] = { 319.0f, 319.0f, 74.0f };
		float speedmax = 5000;


		void setChassisspeed(int32_t ch_speedx, int32_t ch_speedy, int32_t ch_speedz);
		void speedCalculate(int32_t speedx_, int32_t speedy_, int32_t speedz_);
		void Keep_Direction();

		void Update();
		float Ramp(float setval, float curval, uint32_t RampSlope);
	};

	struct PANTILE
	{
		float avg_rpm{};
		int fire{};
		float k_ff = 1.f;          // 小陀螺旋转时的前馈增益
		float k_ff_transition = 1.04f; // 退出小陀螺过渡期的前馈增益（底盘减速阶段，适当减小避免过补偿）
		float k_ff_follow = 1.f;  // FOLLOW模式（普通底盘旋转）时的前馈增益，与小陀螺分开调节
		float follow_ff_rpm_threshold = 100.f; // FOLLOW模式前馈死区（rpm），低于此转速时前馈为0
		enum TYPE { YAW, PITCH,L_YAW };
		float mark_pitch{}, mark_yaw{};
		PID pantile_PID[3] = { {0.4f,0.f,0.f},{0.05f,0.f,0.f}, {0.f,0.f,0.f} };
		PID keep_PID = { 5.f, 0.f, 1.5f, 0.2f };  // PD控制：去除I项避免小陀螺切换时积分饱和导致甩动；alpha=0.2对微分滤波
		PID pidAUTO[2] = { {15.f, 0.02f, 250.f},{0.5f, 0.f, 0.f} };
		PID pantile_reset{ 18.f, 0.f, 300.f };
		const float sensitivity[3] = { 0.4f , 0.2f ,0.006f };
		float angle2Keep, sensitivity_MoveYaw = 0.01f;
		bool aim = false;
		float yaw_transmission_ratio = 0.2069f; // 电机转角 / 云台实际转角，实测标定：Δmotor=0.16rad, ΔIMU=44.32° → 0.16/(44.32×π/180)≈0.2069
		const float yaw_reset_ramp_step = 2.0f; // RESET时大Yaw每帧最大位移(rad)，防止瞬变损坏齿轮
		void Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference);
		void Update();
		float pid_output{};
		float chassis_omega_rad_s{};
	};

	struct SHOOTER
	{

		float now_bullet_speed = 0.f;

		bool auto_shoot = false;
		bool openRub = false, supply_bullet = false;
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;
		int16_t shoot_speed = 6500, rc_supply_speed{};
		int16_t supply_speed = 2500;
		void Update();
	};

	struct AUTOMATION
	{
		// 状态机
		enum AIM_STATE { IDLE, SEARCH, TRACK, LOCKED } aim_state = IDLE;
		void autoMove();
		// 计数器
		int lost_count = 0;           // 丢失目标计数
		int stable_count = 0;         // 稳定锁定计数
		int search_count = 0;         // 搜索超时计数
		float YAWstep = 0.1f;
		// 上次检测位置（用于搜索起点）
		float last_yaw_pos = 0.f;
		float last_pitch_pos = 0.f;

		// 扫描参数
		float scan_speed_yaw = 0.08f, scan_speed_pitch = 0.003f;  // 加快扫描速度
		float scan_range = 0.3f;      // 搜索范围（弧度，约17度）
		bool scan_reversal_yaw = false, scan_reversal_pitch = false;
		float scan_center_yaw = 0.f;  // 扫描中心

		// 锁定阈值
		const float LOCK_THRESHOLD = 0.02f;  // 弧度，约1度内视为稳定
		const int LOCK_STABLE_COUNT = 15;    // 连续15帧稳定才算锁定
		const int LOST_TIMEOUT = 30;         // 30帧(约0.3s)后视为丢失
		const int SEARCH_TIMEOUT = 200;      // 搜索2秒后放弃

		// 状态输出
		bool target_detected = false;
		bool target_angle_aligned = false; // 有目标且实际云台角与上位机yaw/pitch误差足够小时为true
		bool is_locked = false;       // 是否稳定锁定（可以开火）

		// 函数
		void stateUpdate();
		void autoControl();
		void autoScan();
		void autoAim();               // 自瞄主函数
		void searchTarget();          // 搜索函数（小范围）

		// 兼容旧代码
		int navi_cont = 0;
		int aim_cont = 0;
		bool aim_lock = false;
		bool navi_move = false;
		bool scan_finish = false;
	};

	CHASSIS chassis;
	PANTILE pantile;
	SHOOTER shooter;
	AUTOMATION automation;

	static int16_t Setrange(const int16_t original, const int16_t range);
	void Control_Pantile(int32_t ch_yaw, int32_t ch_pitch,int32_t ch_YAW);
	float GetDelta(float delta);
	float Getdelta(float delta);
	void Init(std::vector<Motor*> motor);
	void init_dm();
private:

};

extern CONTROL ctrl;

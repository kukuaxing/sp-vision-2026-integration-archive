#include "motor.h"
#include "gpio.h"
#include "DMmotor.h"
#include "imu.h"
#include "control.h"
#include "nuc.h"
#include "slidingmodec.h"
#include "LowPassFilter.h"
#include "Algorithm.h"

AngleUnwrapper gimbal_unwrapper;  // 小云台陀螺仪
AngleUnwrapper pantile_unwrapper; // 大云台陀螺仪
LowPassFilter yaw_ref_filter(0.999f);
const float YAW_SAFE_HALF_RANGE = 70.0f;  // 小yaw限位半宽，实际限位以RESET记录中心展开
#define DEG_TO_RAD 0.017453292f  // π / 180
Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position, PID _speed2)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	memcpy(&pid[speed2], &_speed2, sizeof(PID));
	this->function = function;
}


Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed, PID _position)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	this->function = function;
}

Motor::Motor(const motor_type type, const motor_mode mode, const function_type function, const uint32_t id, PID _speed)
	: ID(id)
	, type(type)
	, mode(mode)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	this->function = function;
}

void Motor::StatusIdentifier(int32_t torque_current)
{
	if (torque_current == old_torque_current)
		disconnectCount++;
	else
		disconnectCount = 0;

	if (disconnectCount >= disconnectMax)
	{
		disconnectCount = disconnectMax;
if (old_torque_current == 0)
m_status = UNCONNECTED;
else
m_status = DISCONNECTED;
	}
	else
	m_status = FINE;

	old_torque_current = torque_current;
}
uint8_t Motor::getStatus()const
{
	return (uint8_t)m_status;
}
void Motor::Ontimer(uint8_t idata[][8], uint8_t* odata)//idate: receive;odate: trainsmit;RC
{
	uint32_t trainsmit_or_receive_ID = this->ID - ID1;

	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID += 4;
	}*/
	//----------------------------------------------------------------
	this->torque_current = getword(idata[trainsmit_or_receive_ID][4], idata[trainsmit_or_receive_ID][5]);
	this->StatusIdentifier(this->torque_current);
	this->angle[now] = getword(idata[trainsmit_or_receive_ID][0], idata[trainsmit_or_receive_ID][1]);
	this->temperature = idata[trainsmit_or_receive_ID][6];
	//Get currrent speed

	motor_status = 0;
	if (temperature > 70) {
		setspeed = 0;
	}

	if (type == EC60)
	{
		curspeed = static_cast<float>(getdeltaa(angle[now] - angle[pre])) / T / 8192.f * 60.f;
	}
	else {
		curspeed = getword(idata[trainsmit_or_receive_ID][2], idata[trainsmit_or_receive_ID][3]);
	}
	inv_curspeed = -curspeed;
	//----------------------------------------------------------------
	/*if (this->type == M6020)
	{
		trainsmit_or_receive_ID -= 4;
	}*/
	//----------------------------------------------------------------
	//20220121--hz
	if (mode == ACE)
	{

		//if (spinning)
		//{
		//	//1秒8发 36/1减速比 一圈八格
		//	current += pid[speed].Delta(setspeed - curspeed);
		//	current = setrange(current, maxcurrent);
		//}
		//else {
		//	if (need_curcircle > 0)
		//	{
		//		current += pid[speed].Delta(setspeed - curspeed);
		//		current = setrange(current, maxcurrent);
		//		uint8_t deviation = 200;
		//		if (angle[now] >= stopAngle - deviation && angle[now] <= stopAngle + deviation)
		//			need_curcircle--;
		//		if (need_curcircle <= 0)
		//		{
		//			need_curcircle = 0;
		//			stopAngle = angle[now];
		//			pd = 0;
		//			setspeed = 0;
		//			current = 0;
		//		}
		//	}
		//	else if (need_curcircle <= 0)
		//	{
		//		setspeed = 0;
		//		current += pid[speed].Delta(setspeed - curspeed);
		//		current = setrange(current, maxcurrent);
		//	}
		//}
		//if (setspeed == 0 && curspeed == 0)
		//{
		//	motor_status = 1;
		//	motor_angle_status = angle[0];
		//}
		//if (motor_status == 1 && fabs(motor_angle_status - angle[0]) < 50)
		//{
		//	current = 0;
		//}
	}
	// (假设 AngleUnwrapper 实例 和 YAW_SAFE_... 常量已定义)

	else if (mode == POS)
	{
		// 1. 获取基础数据 (保持不变)
		continuous_gimbal_yaw = gimbal_unwrapper.unwrap(imu_gimbal.angle.yaw);
		float continuous_pantile_yaw = pantile_unwrapper.unwrap(imu_pantile.angle.yaw);

		// 计算相对角度 (所有模式都需要用它来做 P 项/位置闭环)
		ctrl.m_current_relative_angle = continuous_gimbal_yaw - continuous_pantile_yaw;

		static bool last_in_reset = false;
		const bool in_reset = (ctrl.mode == CONTROL::RESET);
		const bool from_reset = (last_in_reset && !in_reset);

		// RESET模式：小Yaw改用电机角度闭环到机械角 8156，并记录当前相对角用于后续陀螺仪控制
		if (in_reset)
		{
			constexpr float RESET_SMALL_YAW_TARGET = 8156.0f;
			float mech_err = RESET_SMALL_YAW_TARGET - angle[now];
			if (mech_err > 4096.0f)  mech_err -= 8192.0f;
			if (mech_err < -4096.0f) mech_err += 8192.0f;

			current = (int32_t)ctrl.pantile.pantile_reset.Position(mech_err, (float)maxcurrent);
			current = setrange(current, maxcurrent);

			ctrl.pantile.mark_yaw = ctrl.m_current_relative_angle;
			setangle = ctrl.pantile.mark_yaw;
			yaw_limit_center_ref = ctrl.m_current_relative_angle;
			yaw_limit_center_valid = true;
		}
		else
		{
			if (from_reset)
			{
				// 退出RESET首帧：目标与当前角度对齐，避免目标突变
				setangle = ctrl.m_current_relative_angle;
				ctrl.pantile.mark_yaw = setangle;
				pid[position].ResetIntegral();
				pid[speed].ResetIntegral();
				current = 0;
			}

			const float yaw_limit_center = yaw_limit_center_valid ? yaw_limit_center_ref : 0.0f;
			const float yaw_safe_min = yaw_limit_center - YAW_SAFE_HALF_RANGE;
			const float yaw_safe_max = yaw_limit_center + YAW_SAFE_HALF_RANGE;

			// 计算两种速度供选择
			float relative_velocity = imu_gimbal.angularvelocity.yaw - imu_pantile.angularvelocity.yaw; // 原逻辑：相对速度
			float absolute_velocity = imu_gimbal.angularvelocity.yaw;                                   // 新逻辑：绝对速度

			// 默认更新相对速度到全局变量 (为了兼容其他可能用到这个变量的地方)
			ctrl.m_current_relative_velocity = relative_velocity;

			float raw_target;

			// --- 2. 状态机 (同上次) ---
			if (ctrl.mode == ctrl.AUTO || ctrl.mode == ctrl.SHOOT || ctrl.mode == ctrl.NAVI)
			{
				if (xuc.mode_TJ)
				{
					// --- 状态 1: 自瞄 ---
					ctrl.g_start_scan = false;
					absolute_target_auto = radianToDegree(xuc.yaw_TJ);
					raw_target = absolute_target_auto - continuous_pantile_yaw;

					// 【关键修复】角度归一化到 [-180, 180]
					while (raw_target > 180.0f) raw_target -= 360.0f;
					while (raw_target < -180.0f) raw_target += 360.0f;

					//// ============================================
					//// 【360度自瞄】大Yaw配合小Yaw（以RESET记录中心为基准）
					//// ============================================
					//const float SMALL_YAW_SOFT_LIMIT = 50.0f;  // 相对中心的软限位阈值
					//const float SMALL_YAW_RECENTER = 30.0f;    // 相对中心的回中目标
					//const float soft_limit_right = yaw_limit_center + SMALL_YAW_SOFT_LIMIT;
					//const float soft_limit_left = yaw_limit_center - SMALL_YAW_SOFT_LIMIT;
					//const float recenter_right = yaw_limit_center + SMALL_YAW_RECENTER;
					//const float recenter_left = yaw_limit_center - SMALL_YAW_RECENTER;

					//// 如果目标超出小Yaw软限位，让大Yaw配合
					//if (raw_target > soft_limit_right)
					//{
					//	// 目标在右边太远，大Yaw向右转
					//	float excess = raw_target - recenter_right;
					//	float adjust = fmin(excess * 0.01f, 0.1f);  // 大幅降低：最大每帧0.05度
					//	ctrl.pantile.angle2Keep += adjust;
					//}
					//else if (raw_target < soft_limit_left)
					//{
					//	// 目标在左边太远，大Yaw向左转
					//	float excess = recenter_left - raw_target;
					//	float adjust = fmin(excess * 0.01f, 0.1f);  // 大幅降低：最大每帧0.05度
					//	ctrl.pantile.angle2Keep -= adjust;
					//}

					// 对于非小陀螺模式，同步更新 DM_motorYaw.set_yaw_pos
					if (ctrl.mode != CONTROL::ROTATION && fabs(ctrl.chassis.speedz) < 100)
					{
						DM_motorYaw.set_yaw_pos = ctrl.pantile.angle2Keep;
					}

					raw_target = yaw_ref_filter.update(raw_target);
				}
				else
				{
                 // --- 无目标 ---
					if (ctrl.mode == CONTROL::NAVI)
					{
						// NAVI 模式禁用扫描，避免 Pitch 上下摆动
						ctrl.g_start_scan = false;
						raw_target = yaw_limit_center;
					}
					else
					{
						// 非 NAVI 模式保持原扫描逻辑
						ctrl.g_start_scan = true;
						ctrl.automation.autoScan();
						raw_target = yaw_limit_center;
					}
				}
			}
			else // 手动模式
			{
				// ---  STATE 4: 手动 ---
				ctrl.g_start_scan = false;
				raw_target = setangle;
			}
			// --- 3. "防卡死"限位 (同上次) ---
			float final_target;
			// (使用 m_current_relative_angle 进行判断)
			if (ctrl.m_current_relative_angle >= yaw_safe_min && ctrl.m_current_relative_angle <= yaw_safe_max)
			{
				if (raw_target > yaw_safe_max) { final_target = yaw_safe_max; }
				else if (raw_target < yaw_safe_min) { final_target = yaw_safe_min; }
				else { final_target = raw_target; }
			}
			else
			{
				if (ctrl.m_current_relative_angle > yaw_safe_max) { final_target = yaw_safe_max; }
				else { final_target = yaw_safe_min; }
			}

			// 2. 核心切换逻辑
			float pid_vel_feedback;                 // PID速度环反馈项
			float feedforward_current = 0.0f;      // 额外的前馈电流

			// 【修正】只有在小陀螺模式(ROTATION)时才用绝对速度
			// SHOOT模式下要根据底盘是否在转来判断
			if (ctrl.mode == CONTROL::ROTATION ||
				(ctrl.mode == CONTROL::SHOOT && fabs(ctrl.chassis.speedz) > 100))
			{
				// 速度环看绝对速度 (像粘在空气中一样，隔离底盘抖动)
				pid_vel_feedback = absolute_velocity;

				// 添加摩擦力前馈 (抵消大云台转动带来的物理拖拽)
				feedforward_current = 0.25f * imu_pantile.angularvelocity.yaw;
			}
			else
			{
				// 速度环看相对速度 (死死咬住大云台)
				pid_vel_feedback = relative_velocity;

				static float prev_pantile_vel = 0.0f;
				float pantile_acc = (imu_pantile.angularvelocity.yaw - prev_pantile_vel) / T;
				prev_pantile_vel = imu_pantile.angularvelocity.yaw;
				feedforward_current = (Kff_vel * imu_pantile.angularvelocity.yaw
					+ Kff_acc * pantile_acc) * 0.f;
			}

			// 3. 执行PID控制（角度环 -> 速度环）
			float angle_err = final_target - ctrl.m_current_relative_angle;
			float target_velocity = pid[position].Position(angle_err, static_cast<float>(maxspeed));
			if (target_velocity > maxspeed) target_velocity = static_cast<float>(maxspeed);
			if (target_velocity < -maxspeed) target_velocity = static_cast<float>(-maxspeed);

			float speed_err = target_velocity - pid_vel_feedback;
			float pid_current = pid[speed].Position(speed_err, static_cast<float>(maxcurrent));

			// ================= 积分补偿逻辑（消除静差） =================
			static float yaw_err_integral = 0.0f;
			if (from_reset) yaw_err_integral = 0.0f;

			// 只有在误差较小时才积分（防止大幅运动时超调）
			if (fabs(angle_err) < Err_Threshold) {
				yaw_err_integral += angle_err * Ki_comp;
			}
			else {
				yaw_err_integral = 0.0f; // 误差过大时清零积分
			}

			// 大云台高速旋转时主动清零积分：防止旋转期间积分饱和，停转后反向释放造成冲击振荡
			// 阈值30 deg/s 可根据实际小陀螺转速调整
			if (fabs(imu_pantile.angularvelocity.yaw) > 30.0f) {
				yaw_err_integral = 0.0f;
			}

			// 积分限幅
			if (yaw_err_integral > Int_Limit) yaw_err_integral = Int_Limit;
			if (yaw_err_integral < -Int_Limit) yaw_err_integral = -Int_Limit;
			// ============================================================

			// 4. 计算最终电流（PID输出 - 摩擦前馈 + 积分补偿）
			current = pid_current - feedforward_current + yaw_err_integral;
			current = setrange(current, maxcurrent);
		}

		last_in_reset = in_reset;
	}
	else if (mode == SPD)
	{
		current += pid[speed].Delta(setspeed - curspeed);
		current = setrange(current, maxcurrent);
		if (setspeed == 0 && curspeed == 0)
		{
			motor_status = 1;
			motor_angle_status = angle[0];
		}
		if (motor_status == 1&&fabs(motor_angle_status-angle[0])<50)
		{
			current = 0;
		}
	}
	recorded_the_Laps();
	GetDistanceFromMechanicalAngle();
	angle[pre] = angle[now];
	current = setrange(current, maxcurrent);
	odata[trainsmit_or_receive_ID * 2] = (current & 0xff00) >> 8;//高八位
	odata[trainsmit_or_receive_ID * 2 + 1] = current & 0x00ff;
}
void Motor::recorded_the_Laps() {
	int16_t delta = angle[now] - angle[pre];
	// 处理回绕：顺时针
	if (delta > 8192 / 2)
		delta -= 8192;
	// 处理回绕：逆时针
	else if (delta < -8192 / 2)
		delta += 8192;

	sum_angle+= delta;
//	round_count = total_count / encoder_resolution;
}

uint8_t initial_cnt=0;
void Motor::GetDistanceFromMechanicalAngle() {
	if (initial_cnt<5)
	initial_cnt++;
	distance=(6.2831853f/ 8192.0f)*sum_angle * (WHEEL_RADIUS_MM / GEAR_RATIO)-initial_x;  // 单位：mm

	if(initial_cnt<3)
	initial_x = distance;
}

void Motor::getmax(const type_t type)
{
	adjspeed = 3000;
	switch (type)
	{
	case M3508:
		maxcurrent = 16384;
		maxspeed = 9000;
		break;
	case M3510:
		maxcurrent = 13000;
		maxspeed = 9000;
		break;
	case M2310:
		maxcurrent = 13000;
		maxspeed = 9000;
		adjspeed = 1000;
		break;
	case EC60:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6623:
		maxcurrent = 5000;
		maxspeed = 300;
		break;
	case M6020:
		maxcurrent = 30000;
		maxspeed = 800;
		adjspeed = 80;
		break;
	case M2006:
		maxcurrent = 10000;
		adjspeed = 1000;
		maxspeed = 3000;
		break;
	default:;
	}
}

int16_t Motor::getdeltaa(int16_t diff)
{
	if (diff <= -4096)
		diff += 8192;
	else if (diff > 4096)
		diff -= 8192;
	return diff;
}

int16_t Motor::getword(const uint8_t high, const uint8_t low)
{
	const int16_t word = high;
	return (word << 8) + low;
}

int32_t Motor::setrange(const int32_t original, const int32_t range)
{
	return std::max(std::min(range, original), -range);
}

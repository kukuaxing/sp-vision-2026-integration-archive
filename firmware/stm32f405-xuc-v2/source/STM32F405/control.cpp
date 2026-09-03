#include "control.h"
#include "RC.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"

// ===== [后续修改开始] yaw目标机械角归一化工具 =====
static inline float normalizeMechanical(float mechanical) // 将6020机械角度归一化到0~8192
{
	while (mechanical >= 8192.0f)
	{
		mechanical -= 8192.0f; // 超过一圈则减去一圈编码器值
	}
	while (mechanical < 0.0f)
	{
		mechanical += 8192.0f; // 小于0则补回一圈编码器值
	}
	return mechanical;
}

// ===== [后续修改结束] yaw目标机械角归一化工具 =====
// ===== [后续修改开始] CONTROL初始化：按当前实车电机数量分类，并初始化IMU锁角目标 =====
void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (auto& item : chassis_motor)item = nullptr; // 先清空底盘电机指针，避免未初始化野指针
	for (auto& item : pantile_motor)item = nullptr; // 先清空云台电机指针
	for (auto& item : shooter_motor)item = nullptr; // 先清空发射电机指针
	for (auto& item : supply_motor)item = nullptr; // 先清空拨弹电机指针

	for (std::size_t i = 0; i < motor.size(); i++)
	{
		if (motor[i] == nullptr) // 跳过空指针，防止初始化阶段崩溃
		{
			continue;
		}

		switch (motor[i]->function)
		{
		case(function_type::chassis):
			if (num1 < CHASSIS_MOTOR_NUM) // 防止传入电机数量超过底盘数组容量
			{
				chassis_motor[num1++] = motor[i]; // 按function标记归类到底盘电机数组
			}
			break;
		case(function_type::pantile):
			if (num2 < PANTILE_MOTOR_NUM) // 防止云台电机数量超过当前配置
			{
				pantile_motor[num2++] = motor[i]; // 按function标记归类到云台电机数组
			}
			break;
		case(function_type::shooter):
			if (num3 < SHOOTER_MOTOR_NUM) // 防止发射电机数组越界
			{
				shooter_motor[num3++] = motor[i]; // 归类发射电机
			}
			break;
		case(function_type::supply):
			if (num4 < SUPPLY_MOTOR_NUM) // 防止拨弹电机数组越界
			{
				supply_motor[num4] = motor[i]; // 归类拨弹电机
				supply_motor[num4]->spinning = false;
				supply_motor[num4]->need_curcircle = false;
				num4++;
			}
			break;
		default:
			break;
		}
	}

	pantile.mark_yaw = para.initial_yaw; // 初始化yaw位置目标
	pantile.mark_pitch = para.initial_pitch; // 初始化pitch位置目标
	#if PANTILE_IMU_ENABLE
	pantile.target_imu_yaw = imu_pantile.GetAngleYaw(); // [后续修改] 记录当前IMU yaw作为锁角初始目标
	pantile.target_imu_pitch = imu_pantile.GetAnglePitch(); // [后续修改] 记录当前IMU pitch作为目标参考
	#else
	pantile.target_imu_yaw = 0.0f; // 未启用IMU时目标置零
	pantile.target_imu_pitch = 0.0f; // 未启用IMU时目标置零
	#endif
	if (pantile_motor[PANTILE::YAW] != nullptr)
	{
		pantile_motor[PANTILE::YAW]->setangle = para.initial_yaw; // 给yaw电机写入初始位置目标
	}
}

// ===== [后续修改结束] CONTROL初始化 =====
// ===== [后续修改开始] 云台输入缓存：右摇杆yaw先存入manual_yaw_input =====
void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch)
{
	ch_yaw *= (1.f);//方向相反修改这里正负
	pantile.manual_yaw_input = static_cast<float>(ch_yaw); // 保存右摇杆yaw原始输入，后续由Update决定锁角或自由转动
	pantile.manual_pitch_input = static_cast<float>(ch_pitch); // [后续修改] 保存右摇杆pitch原始输入，后续由Update做自稳微调
}

// ===== [后续修改结束] 云台输入缓存 =====
// ===== [后续修改开始] 底盘三轴速度限幅与方向修正 =====
void CONTROL::Control_Chassis(int32_t ch_speedx, int32_t ch_speedy, int32_t ch_speedz)
{
	const int32_t xy_limit = para.max_speed; // x/y平移速度限幅
	const int32_t z_limit = para.rota_speed > 0 ? para.rota_speed : para.max_speed; // 自转速度优先使用rota_speed限幅

	if (xy_limit > 0)
	{
		ch_speedx = std::max(std::min(ch_speedx, xy_limit), -xy_limit); // 限制前后速度，避免遥控输入过大
		ch_speedy = std::max(std::min(ch_speedy, xy_limit), -xy_limit); // 限制左右速度
	}

	if (z_limit > 0)
	{
		ch_speedz = std::max(std::min(ch_speedz, z_limit), -z_limit); // 限制自转速度
	}

	chassis.speedx = -ch_speedx; // 根据实车前后方向修正x轴符号
	chassis.speedy = ch_speedy; // 保存左右平移速度
	chassis.speedz = ch_speedz; // 保存底盘z轴自转速度
}

// ===== [后续修改结束] 底盘三轴速度限幅 =====
// ===== MID+UP左摇杆360度原地旋转状态机 =====
void CONTROL::CHASSIS::Cancel_AutoTurn360()
{
	auto_turn_active = false;
	auto_turn_rearmed = false; // 再次进入MID+UP后必须先回中，防止带杆误触发
	auto_turn_fault = false;
	auto_turn_progress_ticks = 0;
}

void CONTROL::CHASSIS::Control_AutoTurn360(int32_t turn_stick, int32_t idle_forward_speed, bool link_ok)
{
	constexpr int32_t stick_deadband = 35;
	constexpr int32_t turn_target_ticks = 8192; // 6020编码器一圈；实车可根据打滑误差微调
	constexpr int32_t stop_tolerance_ticks = 46; // 约2度允许误差
	constexpr int32_t slowdown_ticks = 1365; // 剩余约60度时开始减速
	constexpr int32_t slow_turn_speed = 450;
	constexpr uint32_t turn_timeout_ms = 7000;

	const bool stick_active = std::abs(turn_stick) >= stick_deadband;
	if (!link_ok)
	{
		auto_turn_active = false;
		auto_turn_rearmed = false;
		ctrl.Control_Chassis(0, 0, 0);
		return;
	}

	if (!stick_active)
	{
		auto_turn_rearmed = true; // 摇杆回中后才允许下一次旋转
		if (!auto_turn_active)
		{
			auto_turn_fault = false;
		}
	}

	if (!auto_turn_active && auto_turn_rearmed && stick_active)
	{
		if (ctrl.pantile_motor[PANTILE::YAW] == nullptr)
		{
			auto_turn_fault = true;
			auto_turn_rearmed = false;
			ctrl.Control_Chassis(0, 0, 0);
			return;
		}

		auto_turn_active = true;
		auto_turn_rearmed = false;
		auto_turn_fault = false;
		auto_turn_dir = (turn_stick < 0) ? 1 : -1; // 与原左摇杆小陀螺方向保持一致
		auto_turn_start_yaw_sum = ctrl.pantile_motor[PANTILE::YAW]->sum_angle;
		auto_turn_progress_ticks = 0;
		auto_turn_start_ms = HAL_GetTick();
	}

	if (auto_turn_active)
	{
		if (ctrl.pantile_motor[PANTILE::YAW] == nullptr || HAL_GetTick() - auto_turn_start_ms >= turn_timeout_ms)
		{
			auto_turn_active = false;
			auto_turn_fault = true;
			ctrl.Control_Chassis(0, 0, 0);
			return;
		}

		const int32_t yaw_ticks = ctrl.pantile_motor[PANTILE::YAW]->sum_angle - auto_turn_start_yaw_sum;
		auto_turn_progress_ticks = std::abs(yaw_ticks);
		const int32_t remaining_ticks = turn_target_ticks - auto_turn_progress_ticks;

		if (remaining_ticks <= stop_tolerance_ticks)
		{
			auto_turn_active = false;
			ctrl.Control_Chassis(0, 0, 0); // 达到360度后自动停止
			return;
		}

		const int32_t fast_turn_speed = std::max(slow_turn_speed, para.rota_speed * 2 / 3);
		int32_t turn_speed = fast_turn_speed;
		if (remaining_ticks < slowdown_ticks)
		{
			turn_speed = slow_turn_speed
				+ (fast_turn_speed - slow_turn_speed) * remaining_ticks / slowdown_ticks;
		}
		ctrl.Control_Chassis(0, 0, auto_turn_dir * turn_speed); // 任务期间禁用所有平移，保证原地旋转
		return;
	}

	if (stick_active && !auto_turn_rearmed)
	{
		ctrl.Control_Chassis(0, 0, 0); // 完成后若仍推着摇杆，保持停止且不重复触发
	}
	else
	{
		ctrl.Control_Chassis(idle_forward_speed, 0, 0); // 空闲时仅保留左摇杆上下的前后移动
	}
}
// ===== MID+UP 360度原地旋转状态机结束 =====

// ===== [后续修改开始] yaw轴IMU锁角、前馈抵消和手动微调 =====
void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference)
{
	float delta = 0; // 保存目标角与反馈角之间的误差
	if (type == YAW)
	{
		const float feedback_yaw = (ctrl.mode == CONTROL::AUTO)
			? frameOfReference.GetStrictAngleYaw()
			: frameOfReference.GetAngleYaw();
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - feedback_yaw)); // yaw误差：IMU角度差转为6020机械刻度
		if (delta <= -4096.f)
			delta += 8192.f; // 误差小于-半圈时补一圈，取最短路径
		else if (delta >= 4096.f)
			delta -= 8192.f; // 误差大于半圈时减一圈，取最短路径

		yaw_keep_error = delta; // 保存yaw锁角误差供调试观察
		if (ctrl.mode == CONTROL::ROTATION && ctrl.pantile_motor[PANTILE::YAW] != nullptr) // 小陀螺锁角模式：yaw用速度模式主动抵消底盘旋转
		{
			yaw_deadband = 2.0f; // 锁角死区，避免微小抖动导致电机频繁修正
			yaw_output_limit = 360.0f; // yaw锁角速度输出上限
			yaw_rotation_ff = -frameOfReference.GetAngularVelocityYaw() * 0.35f; // [后续修改] 使用IMU yaw角速度做反向前馈，提前抵消底盘小陀螺
			yaw_rotation_ff = std::max(std::min(yaw_rotation_ff, 260.0f), -260.0f); // 前馈限幅，防止IMU尖峰导致yaw猛转

			float yaw_pid_output = 0.0f; // yaw PID输出初始化为0
			if (fabsf(delta) >= yaw_deadband) // 误差超过死区才进行PID修正
			{
				yaw_pid_output = pantile_PID[PANTILE::YAW].Position(delta, 1000.0f) * 2.5f; // 锁角PID输出，额外放大以提高小陀螺抗扰
			}

			yaw_keep_output = yaw_pid_output + yaw_rotation_ff; // yaw最终速度 = PID纠偏 + IMU角速度前馈
			yaw_keep_output = std::max(std::min(yaw_keep_output, yaw_output_limit), -yaw_output_limit); // 最终yaw速度限幅
			ctrl.pantile_motor[PANTILE::YAW]->setspeed = static_cast<int32_t>(yaw_keep_output); // 速度模式下直接给6020目标速度
			mark_yaw = ctrl.pantile_motor[PANTILE::YAW]->angle[now]; // 同步位置目标到当前角，避免切回POS时跳变
			return;
		}
		if (ctrl.mode == CONTROL::AUTO && ctrl.pantile_motor[PANTILE::YAW] != nullptr)
		{
			// Phase 2D实车整定：AUTO直接使用6020速度环，避免把一个很小的相对位置
			// 目标反复钉在当前编码器角附近，导致云台克服一次静摩擦后再次停住。
			constexpr float yaw_stop_deadband = 10.0f;  // 约0.44度：进入后停止
			constexpr float yaw_start_deadband = 24.0f; // 约1.05度：超过后才重新启动
			yaw_deadband = auto_yaw_motion_active ? yaw_stop_deadband : yaw_start_deadband;
			yaw_output_limit = 180.0f; // 已实车验证可跟随，且低于6020的260速度上限
			yaw_rotation_ff = 0.0f;
			const float absolute_delta = fabsf(delta);
			if (auto_yaw_motion_active)
			{
				if (absolute_delta <= yaw_stop_deadband)
					auto_yaw_motion_active = false;
			}
			else if (absolute_delta >= yaw_start_deadband)
			{
				auto_yaw_motion_active = true;
			}

			float yaw_speed_command = 0.0f;
			if (auto_yaw_motion_active)
			{
				// 上位机实车日志确认：此符号使画面目标误差从16.82度收敛到3.39度。
				// 手动DR16轴的符号在RC.cpp独立处理，不能用来反转AUTO闭环。
				yaw_speed_command = delta * 1.2f;
				yaw_speed_command = std::max(
					std::min(yaw_speed_command, yaw_output_limit), -yaw_output_limit);
				const float minimum_start_speed = 120.0f; // 克服实车3.4度附近的静摩擦停滞
				if (fabsf(yaw_speed_command) < minimum_start_speed)
				{
					yaw_speed_command = (yaw_speed_command > 0.0f)
						? minimum_start_speed : -minimum_start_speed;
				}
			}

			yaw_keep_output = yaw_speed_command;
			ctrl.pantile_motor[PANTILE::YAW]->setspeed =
				static_cast<int32_t>(yaw_speed_command);
			mark_yaw = ctrl.pantile_motor[PANTILE::YAW]->angle[now];
			return;
		}

		const bool manual_yaw_active = (ctrl.mode == CONTROL::SEPARATE && fabsf(manual_yaw_input) >= 35.0f); // 分离模式下右摇杆超过死区才认为在手动调yaw
		yaw_deadband = manual_yaw_active ? 6.0f : ((ctrl.mode == CONTROL::ROTATION) ? 3.0f : 25.0f); // 不同模式使用不同yaw保持死区
		yaw_output_limit = manual_yaw_active ? 140.0f : ((ctrl.mode == CONTROL::ROTATION) ? 240.0f : 45.0f); // 不同模式限制不同yaw输出强度
		yaw_rotation_ff = 0.0f; // 默认不加前馈
		if (ctrl.mode == CONTROL::ROTATION)
		{
			yaw_rotation_ff = -static_cast<float>(ctrl.chassis.speedz) * 0.16f; // 兼容无IMU角速度时用底盘自转命令做前馈
			yaw_rotation_ff = std::max(std::min(yaw_rotation_ff, 260.0f), -260.0f); // 前馈限幅，防止IMU尖峰导致yaw猛转
		}

		float yaw_pid_output = 0.0f; // yaw PID输出初始化为0
		if (fabsf(delta) >= yaw_deadband) // 误差超过死区才进行PID修正
		{
			// AUTO目标已经由XucYawController限制为距解锁原点±25度、斜率30度/秒。
			// 这里必须把完整机械角误差交给6020自身的位置环；若再乘通用锁角
			// PID的0.08增益，1度误差只剩约1.8个编码器刻度，实车无法克服静摩擦。
			yaw_pid_output = (ctrl.mode == CONTROL::AUTO)
				? delta
				: pantile_PID[PANTILE::YAW].Position(delta, 1000.0f);
		}

		yaw_keep_output = yaw_pid_output + yaw_rotation_ff; // yaw最终速度 = PID纠偏 + IMU角速度前馈
		yaw_keep_output = std::max(std::min(yaw_keep_output, yaw_output_limit), -yaw_output_limit); // 最终yaw速度限幅
		if (ctrl.pantile_motor[PANTILE::YAW] != nullptr)
		{
			if (fabsf(yaw_keep_output) >= 1.0f)
			{
				mark_yaw = ctrl.pantile_motor[PANTILE::YAW]->angle[now] + yaw_keep_output; // 位置模式下把速度修正量折算成下一步位置目标
			}
			else
			{
				mark_yaw = ctrl.pantile_motor[PANTILE::YAW]->angle[now]; // 同步位置目标到当前角，避免切回POS时跳变
			}
		}
	}
	else if (type == PITCH)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch())); // pitch误差同样转为机械刻度
		if (delta <= -4096.f)
			delta += 8192.f; // 误差小于-半圈时补一圈，取最短路径
		else if (delta >= 4096.f)
			delta -= 8192.f; // 误差大于半圈时减一圈，取最短路径

		if (fabsf(delta) >= 10.f)
		{
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta);
		}
	}
}
// ===== [后续修改结束] yaw轴IMU锁角 =====
// ===== [后续修改开始] 底盘云台分离坐标变换 =====
void CONTROL::CHASSIS::Keep_Direction()
{
	if (ctrl.pantile_motor[PANTILE::YAW] == nullptr) // 没有yaw反馈时不能做底盘云台坐标变换
	{
		return;
	}

	const float s_x = speedx; // 保存变换前x方向速度
	const float s_y = speedy; // 保存变换前y方向速度
	const float yaw_delta = ctrl.GetDelta(mechanicalToDegree(ctrl.pantile_motor[PANTILE::YAW]->angle[now] - para.initial_yaw)); // 当前云台相对底盘初始方向的角度差
	const float theta = -yaw_delta * PI / 180.f; // 角度转弧度并取反，用于把控制量旋转到底盘坐标
	const float st = sinf(theta); // 坐标旋转使用的sin值
	const float ct = cosf(theta); // 坐标旋转使用的cos值

	speedx = static_cast<int32_t>(s_x * ct - s_y * st); // 旋转后的x速度，实现底盘云台分离
	speedy = static_cast<int32_t>(s_x * st + s_y * ct); // 旋转后的y速度，实现底盘云台分离
}

// ===== [后续修改结束] 底盘云台分离坐标变换 =====
// ===== [后续修改开始] 底盘模式更新：分离/跟随/小陀螺进入全向轮解算 =====
void CONTROL::CHASSIS::Update()
{
	transition_from_rotation = (last_mode == CONTROL::ROTATION && ctrl.mode != CONTROL::ROTATION); // 记录是否刚从锁yaw小陀螺退出
	last_mode = ctrl.mode; // 更新上一模式记录

	if (ctrl.mode == RESET)
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
	}
	else if (ctrl.mode == CONTROL::ROTATION || ctrl.mode == CONTROL::ROTATION_FREE || ctrl.mode == CONTROL::FOLLOW) // 这些模式需要根据yaw角做底盘方向解耦
	{
		Keep_Direction(); // 执行底盘云台分离坐标变换
	}

	speedCalculate(speedx, speedy, speedz); // 将三轴速度解算成4个轮子的目标速度
}

// ===== [后续修改结束] 底盘模式更新 =====
// ===== [后续修改开始] 45度安装全向轮速度解算 =====
void CONTROL::CHASSIS::speedCalculate(int32_t speedx_, int32_t speedy_, int32_t speedz_)//速度解算并设定函数
{
	if (ctrl.chassis_motor[0] == nullptr || ctrl.chassis_motor[1] == nullptr ||
		ctrl.chassis_motor[2] == nullptr || ctrl.chassis_motor[3] == nullptr)
	{
		return;
	}

	constexpr float wheel_speed_cap = 9000.0f; // 单轮目标速度限幅，保护3508输出
	auto clamp_speed = [](float target) { // 局部限幅函数，防止解算后目标速度过大
		return fminf(fmaxf(target, -wheel_speed_cap), wheel_speed_cap); // 将目标速度限制在±wheel_speed_cap内
	};
	const uint32_t ramp_slope = // 根据是否以自转为主，选择不同加速斜坡
		(std::abs(static_cast<float>(speedz_)) >
		 (std::abs(static_cast<float>(speedx_)) + std::abs(static_cast<float>(speedy_))))
		? 190U // 自转占主导时斜坡更快，提高小陀螺响应
		: 150U; // 普通平移时斜坡稍慢，减少冲击

	float target0 = clamp_speed(-speedy_ * 0.707f - speedx_ * 0.707f + speedz_); // [后续修改] 全向轮0号速度解算：平移分量+自转分量
	float target1 = clamp_speed(-speedy_ * 0.707f + speedx_ * 0.707f + speedz_); // [后续修改] 全向轮1号速度解算
	float target2 = clamp_speed(speedy_ * 0.707f + speedx_ * 0.707f + speedz_); // [后续修改] 全向轮2号速度解算
	float target3 = clamp_speed(speedy_ * 0.707f - speedx_ * 0.707f + speedz_); // [后续修改] 全向轮3号速度解算

	ctrl.chassis_motor[0]->setspeed = Ramp(target0, ctrl.chassis_motor[0]->setspeed, ramp_slope); // 0号底盘电机目标速度经斜坡缓变后写入
	ctrl.chassis_motor[1]->setspeed = Ramp(target1, ctrl.chassis_motor[1]->setspeed, ramp_slope); // 1号底盘电机目标速度经斜坡缓变后写入
	ctrl.chassis_motor[2]->setspeed = Ramp(target2, ctrl.chassis_motor[2]->setspeed, ramp_slope); // 2号底盘电机目标速度经斜坡缓变后写入
	ctrl.chassis_motor[3]->setspeed = Ramp(target3, ctrl.chassis_motor[3]->setspeed, ramp_slope); // 3号底盘电机目标速度经斜坡缓变后写入

}
// ===== [后续修改结束] 45度安装全向轮速度解算 =====
// ===== [后续修改开始] 云台模式更新：小陀螺锁yaw、自由yaw和分离模式手动yaw =====
void CONTROL::PANTILE::Update()
{
	static CONTROL::MODE last_mode = CONTROL::RESET; // 记录上一周期云台模式，检测模式切换瞬间
	static bool last_manual_yaw_active = false; // 记录上一周期是否处于手动yaw微调

	if (ctrl.mode == RESET)
	{
		if (ctrl.pantile_motor[PANTILE::YAW] != nullptr)
		{
			ctrl.pantile_motor[PANTILE::YAW]->mode = POS; // 复位模式下yaw使用位置模式
			ctrl.pantile_motor[PANTILE::YAW]->setspeed = 0; // 模式切换/复位时先清零速度命令
		}
		mark_yaw = (ctrl.pantile_motor[PANTILE::YAW] != nullptr) ? ctrl.pantile_motor[PANTILE::YAW]->angle[now] : para.initial_yaw; // 复位时目标角跟随当前yaw，避免突然回拉
		mark_pitch = para.initial_pitch; // pitch目标回到初始值
		DMmotor[0].setPos = 0.0f; // [后续修改] 使能后RESET回机械零点(pos=0)
		DMmotor[0].setSpeed = 5.0f; // [后续修改] vel_limit非零，允许运动回零点
#if PANTILE_IMU_ENABLE
		target_imu_yaw = imu_pantile.GetAngleYaw(); // 模式切换时把当前IMU yaw作为新的保持目标
		target_imu_pitch = imu_pantile.GetAnglePitch(); // 模式切换时同步pitch目标参考
#endif
	}
	else
	{
#if PANTILE_IMU_ENABLE
		if (ctrl.pantile_motor[PANTILE::YAW] != nullptr)
		{
			// 6020的普通位置环参数适合锁角，但在Phase 2A仅±1度的小目标下，
			// 稳态电流不足以克服实车静摩擦。AUTO使用更高的持续比例增益并关闭
			// 微分尖峰；离开AUTO后立即恢复原有参数，不改变手动/小陀螺手感。
			ctrl.pantile_motor[PANTILE::YAW]->pid[position].m_Kp =
				(ctrl.mode == CONTROL::AUTO) ? 4.0f : 1.2f;
			ctrl.pantile_motor[PANTILE::YAW]->pid[position].m_Td =
				(ctrl.mode == CONTROL::AUTO) ? 0.0f : 8.0f;
			ctrl.pantile_motor[PANTILE::YAW]->mode =
				(ctrl.mode == CONTROL::AUTO || ctrl.mode == CONTROL::ROTATION ||
				 ctrl.mode == CONTROL::ROTATION_FREE || ctrl.mode == CONTROL::SEPARATE_FREE)
				? SPD : POS; // AUTO和自由yaw相关模式使用速度模式，其他模式使用位置模式
		}

		if (last_mode != ctrl.mode) // 模式刚切换时刷新目标，防止旧积分和旧目标造成抢舵
		{
			auto_yaw_motion_active = false; // 新一次AUTO必须从启动阈值重新判定
			target_imu_yaw = imu_pantile.GetAngleYaw(); // 模式切换时把当前IMU yaw作为新的保持目标
			target_imu_pitch = imu_pantile.GetAnglePitch(); // 模式切换时同步pitch目标参考
			yaw_keep_error = 0.0f; // 清空yaw误差调试量
			yaw_keep_output = 0.0f; // 清空yaw输出调试量
			yaw_rotation_ff = 0.0f; // 默认不加前馈
			pantile_PID[PANTILE::YAW].m_error[0] = 0.0f; // 清除yaw PID当前误差
			pantile_PID[PANTILE::YAW].m_error[1] = 0.0f; // 清除yaw PID上一拍误差
			pantile_PID[PANTILE::YAW].m_error[2] = 0.0f; // 清除yaw PID上上拍误差
			if (ctrl.pantile_motor[PANTILE::YAW] != nullptr)
			{
				mark_yaw = ctrl.pantile_motor[PANTILE::YAW]->angle[now]; // 同步位置目标到当前角，避免切回POS时跳变
				ctrl.pantile_motor[PANTILE::YAW]->setspeed = 0; // 模式切换/复位时先清零速度命令
			}
		}

		if ((ctrl.mode == CONTROL::ROTATION_FREE || ctrl.mode == CONTROL::SEPARATE_FREE)
			&& ctrl.pantile_motor[PANTILE::YAW] != nullptr) // 自由yaw模式：右摇杆直接控制yaw速度，不做IMU锁角
		{
			const float yaw_speed_limit = static_cast<float>(ctrl.pantile_motor[PANTILE::YAW]->maxspeed); // 使用6020配置的安全速度上限
			manual_yaw_step = manual_yaw_input * yaw_speed_limit / 660.0f; // 将DR16摇杆输入映射到yaw速度
			manual_yaw_step = std::max(std::min(manual_yaw_step, yaw_speed_limit), -yaw_speed_limit); // 提高输入灵敏度时仍不超过电机速度上限
			if (fabsf(manual_yaw_input) < 35.0f) // yaw摇杆死区，防止回中后轻微漂移
			{
				manual_yaw_step = 0.0f; // 死区内强制yaw速度为0
			}
			ctrl.pantile_motor[PANTILE::YAW]->setspeed = static_cast<int32_t>(manual_yaw_step); // 自由yaw模式下直接给6020速度命令
			mark_yaw = ctrl.pantile_motor[PANTILE::YAW]->angle[now]; // 同步位置目标到当前角，避免切回POS时跳变
			last_manual_yaw_active = false; // 自由yaw模式不使用分离模式的微调状态
		}

		const bool manual_yaw_active = (ctrl.mode == CONTROL::SEPARATE && fabsf(manual_yaw_input) >= 35.0f); // 分离模式下右摇杆超过死区才认为在手动调yaw
		if (manual_yaw_active)
		{
			manual_yaw_step = manual_yaw_input * 0.0007f; // 分离模式下将右摇杆转换成IMU目标角微调量
			target_imu_yaw = ctrl.GetDelta(target_imu_yaw - manual_yaw_step); // 更新yaw锁角目标，并归一化到±180度
		}
		else if (ctrl.mode != CONTROL::ROTATION_FREE && ctrl.mode != CONTROL::SEPARATE_FREE) // 非自由yaw模式才清零手动微调量
		{
			manual_yaw_step = 0.0f; // 死区内强制yaw速度为0
			if (last_manual_yaw_active)
			{
				target_imu_yaw = imu_pantile.GetAngleYaw(); // 模式切换时把当前IMU yaw作为新的保持目标
				yaw_keep_error = 0.0f; // 清空yaw误差调试量
				yaw_keep_output = 0.0f; // 清空yaw输出调试量
				pantile_PID[PANTILE::YAW].m_error[0] = 0.0f; // 清除yaw PID当前误差
				pantile_PID[PANTILE::YAW].m_error[1] = 0.0f; // 清除yaw PID上一拍误差
				pantile_PID[PANTILE::YAW].m_error[2] = 0.0f; // 清除yaw PID上上拍误差
				if (ctrl.pantile_motor[PANTILE::YAW] != nullptr)
				{
					mark_yaw = ctrl.pantile_motor[PANTILE::YAW]->angle[now]; // 同步位置目标到当前角，避免切回POS时跳变
				}
			}
		}
		last_manual_yaw_active = manual_yaw_active; // 保存本周期手动yaw状态供下一周期判断

		if (ctrl.mode == CONTROL::SEPARATE || ctrl.mode == CONTROL::ROTATION || ctrl.mode == CONTROL::FOLLOW || ctrl.mode == CONTROL::AUTO) // 这些模式需要IMU yaw保持或跟随
		{
			Keep_Pantile(target_imu_yaw, PANTILE::YAW, imu_pantile); // 执行yaw保持闭环
		}

		// ===== [后续修改开始] pitch轴纯位置控制（相对机械零点pos=0）：遥控增量 + 软限位 =====
		const bool manual_pitch_active = (fabsf(manual_pitch_input) >= 35.0f); // 右摇杆上下超过死区才认为在调pitch
		if (manual_pitch_active)
		{
			manual_pitch_step = manual_pitch_input * pitch_sensitivity; // 摇杆→位置增量(弧度/周期)，灵敏度已降低
			DMmotor[0].setPos += pitch_dir * manual_pitch_step; // 方向符号修正
		}
		else
		{
			manual_pitch_step = 0.0f; // 死区内不动作
		}
		DMmotor[0].setPos = std::max(std::min(DMmotor[0].setPos, pitch_max), pitch_min); // 软限位[-0.25,0.4]rad
		DMmotor[0].setSpeed = 5.0f; // 位置速度模式vel_limit(速度限制,rad/s)
		// ===== [后续修改结束] pitch轴纯位置控制 =====
#endif
	}

	mark_yaw = normalizeMechanical(mark_yaw); // 统一限制yaw目标到0~8192机械角范围
	mark_pitch = std::max(std::min(mark_pitch, para.pitch_max), para.pitch_min); // pitch目标限位，保护机械结构

	if (ctrl.pantile_motor[PANTILE::YAW] != nullptr)
	{
		ctrl.pantile_motor[PANTILE::YAW]->setangle = mark_yaw; // 写入yaw位置目标，POS模式使用；SPD模式下保持同步
	}
	last_mode = ctrl.mode; // 更新上一模式记录
}

// ===== [后续修改结束] 云台模式更新 =====
// ===== [Next] shooter state machine: single 45deg feed + stick continuous feed =====
void CONTROL::SHOOTER::Update()
{
	Motor* supply = ctrl.supply_motor[0];
	const uint32_t now = HAL_GetTick();
	const bool link_ok = rc.frame_valid && (now - rc.last_frame_ms) < RC_LINK_TIMEOUT_MS;
	static int32_t local_jams = 0; // jam count of current task
	auto abs32 = [](int32_t v) { return v < 0 ? -v : v; };

	// 1) Reset or link loss: force everything off
	if (ctrl.mode == RESET || !link_ok)
	{
		openRub = false;
		supply_bullet = false;
		trigger_now = false;
		trigger_rise = false;
		shot_busy = false;
		continuous_active = false;
		waiting_wheels = false;
		jam_recovering = false;
		burst_target = 0;
		rounds_fed = 0;
		fire_fault = false;
	}

	// 2) Friction wheels spin only in shoot mode
	if (ctrl.shooter_motor[0] != nullptr)
	{
		ctrl.shooter_motor[0]->setspeed = openRub ? -shoot_speed : 0;
	}
	if (ctrl.shooter_motor[1] != nullptr)
	{
		ctrl.shooter_motor[1]->setspeed = openRub ? shoot_speed : 0;
	}

	if (supply == nullptr)
	{
		return;
	}

	const float wheel0 = fabsf(ctrl.shooter_motor[0] != nullptr ? static_cast<float>(ctrl.shooter_motor[0]->curspeed) : 0.f);
	const float wheel1 = fabsf(ctrl.shooter_motor[1] != nullptr ? static_cast<float>(ctrl.shooter_motor[1]->curspeed) : 0.f);
	const bool wheels_ready = wheel0 >= shoot_speed * WHEELS_READY_RATIO
		&& wheel1 >= shoot_speed * WHEELS_READY_RATIO;
	const bool wheels_min = wheel0 >= shoot_speed * WHEELS_MIN_RATIO
		&& wheel1 >= shoot_speed * WHEELS_MIN_RATIO;

	// 3) Leaving shoot mode aborts the current feed immediately
	if (!openRub)
	{
		supply->setspeed = 0;
		supply->spinning = false;
		shot_busy = false;
		continuous_active = false;
		waiting_wheels = false;
		jam_recovering = false;
		burst_target = 0;
		rounds_fed = 0;
		return;
	}

	// 4) Start new task: left stick single shot first, right stick continuous next
	if (!shot_busy && !continuous_active && !fire_fault)
	{
		if (trigger_rise)
		{
			shot_busy = true;
			waiting_wheels = !wheels_ready;
			burst_target = 1;
			rounds_fed = 0;
			local_jams = 0;
			shot_start_angle = supply->sum_angle;
			last_feed_angle = supply->sum_angle;
			last_angle_time = now;
			jam_recovering = false;
		}
		else if (supply_bullet)
		{
			continuous_active = true;
			waiting_wheels = !wheels_ready;
			burst_target = 0; // 0 = unlimited rounds while stick is held
			rounds_fed = 0;
			local_jams = 0;
			shot_start_angle = supply->sum_angle;
			last_feed_angle = supply->sum_angle;
			last_angle_time = now;
			jam_recovering = false;
		}
	}

	// 5) No task: stop feeder, clear fault after all inputs released
	if (!shot_busy && !continuous_active)
	{
		supply->setspeed = 0;
		supply->spinning = false;
		if (fire_fault && !trigger_now && !supply_bullet)
		{
			fire_fault = false;
		}
		return;
	}

	// 6) Continuous feed stops as soon as the right stick is released
	if (continuous_active && !supply_bullet)
	{
		supply->setspeed = 0;
		supply->spinning = false;
		continuous_active = false;
		return;
	}

	// 7) Wait for friction wheels to reach speed before feeding
	if (waiting_wheels)
	{
		supply->setspeed = 0;
		supply->spinning = false;
		if (wheels_ready)
		{
			waiting_wheels = false;
			shot_start_angle = supply->sum_angle;
			last_feed_angle = supply->sum_angle;
			last_angle_time = now;
		}
		return;
	}

	// 8) Jam recovery: reverse briefly, then continue
	if (jam_recovering)
	{
		const int8_t active_dir = shot_busy ? single_dir : continuous_dir;
		supply->setspeed = -active_dir * supply_speed / 2;
		supply->spinning = false;
		if (now - jam_recover_ms >= JAM_RECOVER_MS)
		{
			jam_recovering = false;
			last_angle_time = now;
			last_feed_angle = supply->sum_angle;
			if (++local_jams >= MAX_JAMS_PER_SHOT)
			{
				fire_fault = true;
				shot_busy = false;
				continuous_active = false;
			}
		}
		return;
	}

	// 9) Pause feeding if friction wheels drop too low
	if (!wheels_min)
	{
		supply->setspeed = 0;
		supply->spinning = false;
		return;
	}

	const int8_t active_dir = shot_busy ? single_dir : continuous_dir;
	supply->setspeed = supply_speed * active_dir;
	supply->spinning = true;

	// 10) Encoder counting: single shot stops after 36864 ticks (45deg)
	if (abs32(supply->sum_angle - last_feed_angle) >= 20)
	{
		last_feed_angle = supply->sum_angle;
		last_angle_time = now;
	}

	const int32_t fed = supply->sum_angle - shot_start_angle;
	if (burst_target > 0)
	{
		if (abs32(fed) >= burst_target * BULLET_FEED_TICKS)
		{
			supply->setspeed = 0;
			supply->spinning = false;
			shot_busy = false;
			shot_count += burst_target;
		}
	}
	else
	{
		// Continuous mode: count every full round and roll the baseline
		const int32_t fed_rounds = abs32(fed) / BULLET_FEED_TICKS;
		if (fed_rounds > 0)
		{
			shot_count += fed_rounds;
			shot_start_angle += fed_rounds * BULLET_FEED_TICKS * (fed >= 0 ? 1 : -1);
		}
	}

	// 11) Jam detection: feeding commanded but supply barely rotates
	if (fabsf(supply->curspeed) < supply_speed * JAM_SPEED_RATIO
		&& now - last_angle_time >= JAM_TIMEOUT_MS)
	{
		jam_count++;
		local_jams++;
		jam_recovering = true;
		jam_recover_ms = now;
		supply->setspeed = 0;
		supply->spinning = false;
		return;
	}
}
// ===== [Next] shooter state machine end =====

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

extern uint8_t Power_stsRx[];

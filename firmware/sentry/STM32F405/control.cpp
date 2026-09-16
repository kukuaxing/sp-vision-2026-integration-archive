#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "DMmotor.h"
#include "nuc.h"
#include "RC.h"
#include "LowPassFilter.h"
#include "supercap.h"
extern DMMOTOR DM_motorYaw;
extern DMMOTOR DM_motorPitch;
LowPassFilter yaw_rotation_speed_filter(0.8f);    // 小陀螺/RESET/SHOOT 前馈滤波器
LowPassFilter follow_rotation_speed_filter(0.8f); // FOLLOW模式独立前馈滤波器，防止与小陀螺共享状态时互相污染

namespace {
	struct FeedforwardBlendState
	{
		float cmd = 0.0f;
		bool deadband_hold = true;
	};

	FeedforwardBlendState yaw_ff_state;
	FeedforwardBlendState follow_ff_state;

	static inline float BlendFeedforwardWithHysteresis(
		float raw_ff,
		float avg_rpm,
		float enter_deadband_rpm,
		float exit_deadband_rpm,
		float blend_alpha,
		FeedforwardBlendState& state)
	{
		const float abs_rpm = std::abs(avg_rpm);

		if (state.deadband_hold)
		{
			if (abs_rpm > exit_deadband_rpm)
			{
				state.deadband_hold = false;
			}
		}
		else
		{
			if (abs_rpm < enter_deadband_rpm)
			{
				state.deadband_hold = true;
			}
		}

		float target_ff = state.deadband_hold ? 0.0f : raw_ff;
		state.cmd = blend_alpha * target_ff + (1.0f - blend_alpha) * state.cmd;
		return state.cmd;
	}
}

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			supply_motor[num4++] = motor[i];
		default:
			break;
		}
		
	}
	
	pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;
}

void CONTROL::CHASSIS::setChassisspeed(int32_t ch_speedx_, int32_t ch_speedy_, int32_t ch_speedz_)//从遥控器获取速度函数
{
 const bool bypass_power_scaling = (ctrl.mode == CONTROL::FOLLOW);
	if (bypass_power_scaling)
	{
		const float base_xy_speedmax = 5000.0f;
		const float base_spin_speedmax = 7000.0f;
		ctrl.chassis.speedmax = base_xy_speedmax;

		ctrl.chassis.speedx = ch_speedx_ / 660.0f * ctrl.chassis.speedmax;
		ctrl.chassis.speedy = -ch_speedy_ / 660.0f * ctrl.chassis.speedmax;
		ctrl.chassis.speedz = ch_speedz_ / 660.0f * base_spin_speedmax;
		return;
	}

	const float fallback_power_limit = 80.0f;
	const float nominal_power_limit = 80.0f;
	const float min_ratio = 0.45f;
	const float max_ratio = 2.0f;
	const float base_xy_speedmax = 5000.0f;
	const float base_spin_speedmax = 7000.0f;
	const float cap_full_energy = 1764.0f;
	const float cap_ready_energy = 600.0f;

	float ref_power_limit = (judgement.data.robot_status_t.chassis_power_limit > 10)
		? (float)judgement.data.robot_status_t.chassis_power_limit
		: fallback_power_limit;
	float power_ratio = ref_power_limit / nominal_power_limit;
	power_ratio = fminf(fmaxf(power_ratio, min_ratio), max_ratio);
	float cap_energy_ratio = 0.0f;
	if (supercap.connect) {
		cap_energy_ratio = (supercap.Rxsuper.cap_energy - cap_ready_energy) /
			(cap_full_energy - cap_ready_energy);
		cap_energy_ratio = fminf(fmaxf(cap_energy_ratio, 0.0f), 1.0f);
	}

	const float scaled_xy_speedmax = base_xy_speedmax * sqrtf(power_ratio);
	const float spin_energy_factor = 1.00f + 0.20f * cap_energy_ratio;
	const float scaled_spin_speedmax =
		base_spin_speedmax * sqrtf(power_ratio) * spin_energy_factor;
	ctrl.chassis.speedmax = scaled_xy_speedmax;
	if (!((rc.rc.s[0] == rc.DOWN && rc.rc.s[1] == rc.MID)))
	{
		ctrl.chassis.speedx = ch_speedx_ / 660.0f * ctrl.chassis.speedmax;
		ctrl.chassis.speedy = -ch_speedy_ / 660.0f * ctrl.chassis.speedmax;
	}
	ctrl.chassis.speedz = ch_speedz_ / 660.0f * scaled_spin_speedmax;
}

void CONTROL::CHASSIS::speedCalculate(int32_t speedx_, int32_t speedy_, int32_t speedz_)//速度解算并设定函数
{
	constexpr float wheel_speed_cap = 9000.0f;
	auto clamp_speed = [](float target) {
		return fminf(fmaxf(target, -wheel_speed_cap), wheel_speed_cap);
	};
	const uint32_t ramp_slope =
		(std::abs(static_cast<float>(speedz_)) >
		 (std::abs(static_cast<float>(speedx_)) + std::abs(static_cast<float>(speedy_))))
		? 190U
		: 150U;

	float target0 = clamp_speed(-speedy_ * 0.707f - speedx_ * 0.707f + speedz_);
	float target1 = clamp_speed(-speedy_ * 0.707f + speedx_ * 0.707f + speedz_);
	float target2 = clamp_speed(speedy_ * 0.707f + speedx_ * 0.707f + speedz_);
	float target3 = clamp_speed(speedy_ * 0.707f - speedx_ * 0.707f + speedz_);

	ctrl.chassis_motor[0]->setspeed = Ramp(target0, ctrl.chassis_motor[0]->setspeed, ramp_slope);
	ctrl.chassis_motor[1]->setspeed = Ramp(target1, ctrl.chassis_motor[1]->setspeed, ramp_slope);
	ctrl.chassis_motor[2]->setspeed = Ramp(target2, ctrl.chassis_motor[2]->setspeed, ramp_slope);
	ctrl.chassis_motor[3]->setspeed = Ramp(target3, ctrl.chassis_motor[3]->setspeed, ramp_slope);

}

void CONTROL::Control_Pantile(int32_t ch_yaw, int32_t ch_pitch,int32_t ch_YAW)
{
	ch_pitch *= (-1.f);
	ch_yaw *= (-1.f);
	ch_YAW *= (-1.f);//方向相反修改这里正负
	ctrl.pantile.mark_yaw += ctrl.pantile.sensitivity[0] * ch_yaw / 660.f;
	DM_motorYaw.set_yaw_pos += ctrl.pantile.sensitivity[1]* ctrl.pantile.sensitivity[1] * ch_YAW / 660.f;
	DM_motorPitch.set_pitch_pos += ctrl.pantile.sensitivity[2] * ch_pitch / 660.0f;

	
}

//
//void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference)
//{
//	if (type == ctrl.pantile.L_YAW) {
//		float delta = 0, adjust = sensitivity[1];
//		delta = degreeToRadian(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw()));
//		if (delta <= -3.14f)
//			delta += 2 * 3.14f;
//		else if (delta >= 3.14f)
//			delta -= 2 * 3.14f;
//		DM_motorYaw.set_speed = ctrl.pantile.keep_PID.Position(delta, 100.f);
//	}
//}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference)
{
	if (type == ctrl.pantile.L_YAW) {
		
		const float wheel_radius = 0.076f;      // m
		const float chassis_radius = 0.25f;     // m
		const float gear_ratio = 19.2f;
		

		// 1. 角度误差
		float delta = degreeToRadian(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw()));

		// 2. 归一化到 (-PI, PI)
		if (delta <= -PI)
			delta += 2.0f * PI;
		else if (delta >= PI)
			delta -= 2.0f * PI;

		// 3. PID 输出（rad/s）
		float pid_output = ctrl.pantile.keep_PID.Position(delta, 100.0f);

		// 4. 底盘平均电机转速（rpm）
		avg_rpm =
			(ctrl.chassis_motor[0]->curspeed +
				ctrl.chassis_motor[1]->curspeed +
				ctrl.chassis_motor[2]->curspeed +
				ctrl.chassis_motor[3]->curspeed) / 4.0f;

		// 5. 估算底盘自转角速度（rad/s）
		float chassis_rotation_speed =
			avg_rpm * 2.0f * PI / 60.0f / gear_ratio * wheel_radius / chassis_radius;

		// 6. 滤波（各模式使用独立滤波器，防止状态互相污染）
		float ff_rotation_speed = (ctrl.mode == CONTROL::FOLLOW)
			? follow_rotation_speed_filter.update(chassis_rotation_speed)
			: yaw_rotation_speed_filter.update(chassis_rotation_speed);

     // 7. 前馈软死区：使用滞回 + 平滑，避免减速穿越阈值时“抽一下”
		FeedforwardBlendState& ff_state = (ctrl.mode == CONTROL::FOLLOW)
			? follow_ff_state
			: yaw_ff_state;
		const float deadband_enter = (ctrl.mode == CONTROL::FOLLOW)
			? follow_ff_rpm_threshold
			: 80.0f;
		const float deadband_exit = deadband_enter + 25.0f;
		const float ff_blend_alpha = 0.20f;
		ff_rotation_speed = BlendFeedforwardWithHysteresis(
			ff_rotation_speed,
			avg_rpm,
			deadband_enter,
			deadband_exit,
			ff_blend_alpha,
			ff_state);

		// 8. 前馈叠加：过渡期 / 普通旋转 / 小陀螺 使用各自独立增益
		float active_k_ff;
		if (ctrl.chassis.transition_from_rotation)
			active_k_ff = k_ff_transition;
		else if (ctrl.mode == CONTROL::FOLLOW)
			active_k_ff = k_ff_follow;
		else
			active_k_ff = k_ff;
		float set_speed = pid_output + active_k_ff * ff_rotation_speed;


		// 9. 输出
		DM_motorYaw.set_speed = set_speed;
	}
}

void CONTROL::CHASSIS::Keep_Direction()//以云台朝向为正方向，底盘方向解算
{
	double s_x = speedx, s_y = speedy;
	// 电机与云台通过同步带/齿轮连接，存在传动比 k（电机角位移 / 云台角位移）
	// 实际云台转角 = 电机转角 / k，k 可通过对比 IMU 与电机角度变化标定
	double theat = ctrl.Getdelta(DM_motorYaw.pos - DM_motorYaw.initial_pos_yaw);
	double s_t = sin(theat);
	double c_t = cos(theat);
	speedx = (s_x * c_t - s_y * s_t);
	speedy = (s_x * s_t + s_y * c_t);

}

void CONTROL::CHASSIS::Update()
{
	if (flag_DM_ready)
	{
		ctrl.pantile.angle2Keep = imu_pantile.GetAngleYaw();
        if (DM_motorYaw.pos != 0 && DM_motorPitch.pos != 0 && rc.HasReceivedValidFrame())
		{
			DM_ready = true;
			flag_DM_ready = false;
		}
	}
	power_1 = supercap.cap_power;
	power_2 = supercap.cap_voltage * supercap.cap_current;
  if (ctrl.mode == RESET && !DM_ready)
	{
		speedx = 0;
		speedy = 0;
		speedz = 0;
		for (int i = 0; i < CHASSIS_MOTOR_NUM; ++i)
		{
			if (ctrl.chassis_motor[i] != nullptr)
			{
				ctrl.chassis_motor[i]->setspeed = 0;
			}
		}
		return;
	}

	if (ctrl.mode == RESET && DM_ready)
	{
		speedx = 0;
		speedy = 0;
		// 底盘旋转至初始相对位置：以电机编码器误差驱动底盘自转
		float reset_error = DM_motorYaw.pos - DM_motorYaw.initial_pos_yaw;
		if (reset_error > PI)  reset_error -= 2.0f * PI;
		if (reset_error < -PI) reset_error += 2.0f * PI;
		speedz = (int32_t)chassis_reset.Position(-reset_error, 3000.f);
	}
	
		Keep_Direction();
	
	ctrl.chassis.speedCalculate(speedx, speedy, speedz);
}

void CONTROL::PANTILE::Update()
{
	
	Motor* yaw_motor = ctrl.pantile_motor[PANTILE::YAW];
	if (yaw_motor != nullptr && yaw_motor->yaw_limit_center_valid)
	{
		const float yaw_safe_min = yaw_motor->yaw_limit_center_ref - 70.0f;
		const float yaw_safe_max = yaw_motor->yaw_limit_center_ref + 70.0f;
		if (mark_yaw > yaw_safe_max) mark_yaw = yaw_safe_max;
		if (mark_yaw < yaw_safe_min) mark_yaw = yaw_safe_min;
	}

	ctrl.pantile_motor[PANTILE::YAW]->setangle = mark_yaw;

	if (DM_motorPitch.set_pitch_pos > DM_motorPitch.DM_pitch_max)
	{
		DM_motorPitch.set_pitch_pos = DM_motorPitch.DM_pitch_max;
	}
	if (DM_motorPitch.set_pitch_pos < DM_motorPitch.DM_pitch_min)
	{
		DM_motorPitch.set_pitch_pos = DM_motorPitch.DM_pitch_min;
	}
	
	/*float dis[2];
	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;
	if (mark_yaw < 0.0)mark_yaw += 8192.0;

	if ((mark_yaw > 2015) && (mark_yaw < 6100))
	{
		dis[0] = mark_yaw - 2015;
		dis[1] = 6100 - mark_yaw;

		mark_yaw = (dis[0] <= dis[1]) ? 2015 : 6100 ;

	}*/

	
}

void CONTROL::SHOOTER::Update()
{
	//now_bullet_speed = judgement.data.ext_shoot_data_t.bullet_speed;
 static bool heat_lock_latched = false;
	static bool heat_lock_permanent = false;

	int16_t heat_limited_supply_speed = ctrl.shooter.supply_speed;
	const bool auto_sentry_mode =1;
	const float base_q0 = auto_sentry_mode ? 260.0f : 100.0f;
	const float cool_per_second = auto_sentry_mode ? 30.0f : 10.0f;
	const float referee_limit = static_cast<float>(judgement.data.robot_status_t.shooter_barrel_heat_limit);
	const float q0 = (referee_limit > 0.0f) ? fminf(base_q0, referee_limit) : base_q0;
	const float q1 = static_cast<float>(judgement.data.power_heat_data_t.shooter_17mm_1_barrel_heat);
	const float q2 = q0 + 100.0f;

	if (!heat_lock_permanent)
	{
		if (q1 > q2)
		{
			heat_lock_permanent = true;
			heat_lock_latched = true;
		}
		else if (heat_lock_latched)
		{
			if (q1 <= 0.0f)
			{
				heat_lock_latched = false;
			}
		}
		else if (q1 > q0)
		{
			heat_lock_latched = true;
		}
	}

	ctrl.shooter.heat_ulimit = heat_lock_latched;
	ctrl.shooter.fullheat_shoot = heat_lock_permanent;

	if (heat_lock_latched)
	{
		heat_limited_supply_speed = 0;
	}
	else
	{
		const float remain_heat = q0 - q1;
		const float reserve_heat = cool_per_second * 2.0f;
		if (remain_heat <= 0.0f)
		{
			heat_limited_supply_speed = 0;
		}
		else if (remain_heat < reserve_heat)
		{
			float ratio = remain_heat / reserve_heat;
			ratio = fmaxf(0.2f, fminf(ratio, 1.0f));
			heat_limited_supply_speed = static_cast<int16_t>(ctrl.shooter.supply_speed * ratio);
		}
	}

	if (ctrl.mode == RESET)
	{
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
	}
	if (openRub)
	{
		ctrl.shooter_motor[0]->setspeed = -shoot_speed;
		ctrl.shooter_motor[1]->setspeed = shoot_speed;
	}
	else
	{
		ctrl.shooter_motor[0]->setspeed = 0;
		ctrl.shooter_motor[1]->setspeed = 0;
		;
	}
    
	if (supply_bullet && openRub && xuc.mode_TJ)
	{
        ctrl.supply_motor[0]->setspeed = -heat_limited_supply_speed;
	}
	else if (ctrl.mode == TD)
	{
        ctrl.supply_motor[0]->setspeed = -heat_limited_supply_speed;
	}
	else 
	{
		ctrl.supply_motor[0]->setspeed = -ctrl.shooter.rc_supply_speed;
	}
}

void CONTROL::AUTOMATION::autoMove()
{
	ctrl.mode = CONTROL::NAVI;

	constexpr float M3508_RATIO = 19.2f;
	constexpr float WHEEL_DIAM_M = 0.15184f;
	constexpr float MS_TO_RPM = M3508_RATIO * 60.0f / (PI * WHEEL_DIAM_M);

	const int32_t nav_x = (int32_t)(xuc.RxNav.linear_x * MS_TO_RPM);
	const int32_t nav_y = (int32_t)(xuc.RxNav.linear_y * MS_TO_RPM);

	ctrl.chassis.navi_vx = (float)nav_x;
	ctrl.chassis.navi_vy = (float)nav_y;
	ctrl.chassis.navi_vz = xuc.RxNav.angular_z;
	DM_motorYaw.set_speed = xuc.RxNav.angular_z;

	ctrl.chassis.speedx = nav_x;
	ctrl.chassis.speedy = nav_y;
	ctrl.chassis.speedz = 0;
	ctrl.shooter.openRub = false;
	ctrl.shooter.supply_bullet = false;
}

void CONTROL::AUTOMATION::stateUpdate()
{
	// 检测是否有目标
	target_detected = (xuc.mode_TJ != 0);

	// 有目标且云台实际角与上位机目标角误差足够小时置true
	float yaw_err = xuc.yaw_TJ - degreeToRadian(imu_gimbal.angle.yaw);
	while (yaw_err > PI) yaw_err -= 2.0f * PI;
	while (yaw_err < -PI) yaw_err += 2.0f * PI;
	float pitch_err = xuc.pitch_TJ - degreeToRadian(imu_gimbal.angle.pitch);
	target_angle_aligned = target_detected
		&& (std::abs(yaw_err) < LOCK_THRESHOLD)
		&& (std::abs(pitch_err) < LOCK_THRESHOLD);
	
	// 状态机转换
	switch (aim_state)
	{
	case IDLE:
		is_locked = false;
		stable_count = 0;
		if (target_detected) {
			aim_state = TRACK;
			lost_count = 0;
		}
		break;
		
	case TRACK:
		if (target_detected) {
			lost_count = 0;
			// 记录最后检测位置（小云台位置）
			last_yaw_pos = ctrl.pantile.mark_yaw;  // 小Yaw
			last_pitch_pos = DM_motorPitch.set_pitch_pos;
			
			// 检查是否稳定锁定
			if (target_angle_aligned) {
				stable_count++;
				if (stable_count >= LOCK_STABLE_COUNT) {
					aim_state = LOCKED;
				}
			} else {
				stable_count = 0;
			}
		} else {
			lost_count++;
			stable_count = 0;
			if (lost_count > LOST_TIMEOUT) {
				aim_state = SEARCH;
				search_count = 0;
				scan_center_yaw = last_yaw_pos;  // 从丢失位置开始搜索
			}
			// 短暂丢失时保持当前位置等待重新捕获
		}
		is_locked = false;
		break;
		
	case LOCKED:
		if (target_detected) {
			lost_count = 0;
			// 继续检查是否还锁定
			if (!target_angle_aligned) {
				stable_count = 0;
				aim_state = TRACK;  // 回到跟踪状态
			}
		} else {
			lost_count++;
			if (lost_count > LOST_TIMEOUT / 2) {  // 锁定状态更快进入搜索
				aim_state = SEARCH;
				search_count = 0;
				scan_center_yaw = last_yaw_pos;
			}
		}
		is_locked = (aim_state == LOCKED);
		break;
		
	case SEARCH:
		if (target_detected) {
			aim_state = TRACK;
			lost_count = 0;
			stable_count = 0;
		} else {
			search_count++;
			if (search_count > SEARCH_TIMEOUT) {
				aim_state = IDLE;  // 搜索超时，回到空闲
			}
		}
		is_locked = false;
		break;
	}
	
	// 兼容旧变量
	aim_lock = !target_detected && (lost_count > 50);
	aim_cont = lost_count;
}

void CONTROL::AUTOMATION::autoAim()
{
	// 先更新状态
	stateUpdate();
	
	// 【关键】有目标时禁止扫描，让motor.cpp中的自瞄逻辑接管
	if (target_detected) {
		ctrl.g_start_scan = false;
	}
	
	switch (aim_state)
	{
	case IDLE:
		// 空闲状态：保持静止，等待手动控制或目标出现
		// 可以启用扫描
		ctrl.g_start_scan = true;
		break;
		
	case TRACK:
	case LOCKED:
		// 自瞄状态：禁止扫描，云台由motor.cpp中的自瞄逻辑控制
		// motor.cpp会根据xuc.mode_TJ和xuc.yaw_TJ直接计算目标角度
		// 这里不需要手动调整mark_yaw，否则会冲突！
		ctrl.g_start_scan = false;
		
		if (target_detected) {
			// 记录当前位置，用于丢失后搜索（小云台位置）
			last_yaw_pos = ctrl.pantile.mark_yaw;  // 小Yaw
			last_pitch_pos = DM_motorPitch.set_pitch_pos;
		}
		// 丢失时短暂保持（不动）
		break;
		
	case SEARCH:
		// 搜索状态：在丢失位置附近小范围搜索
		
		ctrl.g_start_scan = false;  // 不用全局扫描，用局部搜索
		searchTarget();
		break;
	}
}

void CONTROL::AUTOMATION::searchTarget()
{
	// 小范围搜索：以丢失位置为中心，左右摆动
	// *** 修正：使用 mark_yaw（小云台）而非 DM_motorYaw（大云台）***
	float current_offset = ctrl.pantile.mark_yaw - scan_center_yaw;
	
	// 边界检测
	if (current_offset >= scan_range) {
		scan_reversal_yaw = true;   // 向左
	} else if (current_offset <= -scan_range) {
		scan_reversal_yaw = false;  // 向右
	}
	
	// 移动（搜索速度慢一些）- 控制小云台
	
	
	// Pitch 小幅度上下扫
	if (DM_motorPitch.set_pitch_pos >= last_pitch_pos + 0.1f) {
		scan_reversal_pitch = true;
	} else if (DM_motorPitch.set_pitch_pos <= last_pitch_pos - 0.1f) {
		scan_reversal_pitch = false;
	}
	
	if (scan_reversal_pitch) {
		DM_motorPitch.set_pitch_pos -= scan_speed_pitch;
	} else {
		DM_motorPitch.set_pitch_pos += scan_speed_pitch;
	}
}

void CONTROL::AUTOMATION::autoControl() 
{
	// 调用新的自瞄逻辑
	autoAim();
	
	// 根据状态控制发射
	if (aim_state == LOCKED || aim_state == TRACK) {
		ctrl.shooter.openRub = true;
		// 供弹由RC档位控制，这里不自动开火
	} else if (aim_state == SEARCH) {
		ctrl.shooter.openRub = true;
		ctrl.shooter.supply_bullet = false;
	} else {
		// IDLE状态
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
	}
}
/**
 * @brief 自动扫描函数 (无参数版本)
 * 注意：此函数依赖于 m_current_relative_angle 成员变量
 */
void CONTROL::AUTOMATION::autoScan()
{
	// *** 关键修正：修复反向逻辑 ***
	// 使用 "电平触发" 逻辑，而不是 "翻转" 逻辑
	// m_current_relative_angle 必须是您在主循环中解算后的连续相对角度

	Motor* yaw_motor = ctrl.pantile_motor[PANTILE::YAW];
	const float scan_center = (yaw_motor != nullptr && yaw_motor->yaw_limit_center_valid)
		? yaw_motor->yaw_limit_center_ref
		: 0.0f;

	// 1. Yaw 轴扫描（以RESET记录中心为基准）
	if (ctrl.m_current_relative_angle >= scan_center + 45.0f)
	{
		scan_reversal_yaw = false; // 必须向负方向移动
	}
	else if (ctrl.m_current_relative_angle <= scan_center - 45.0f)
	{
		scan_reversal_yaw = true; // 必须向正方向移动
	}
	// (如果介于边界之间，scan_reversal_yaw 保持不变，继续朝当前方向移动)


	// 2.1 扫描目标限幅（同样以RESET记录中心为基准）
	const float scan_limit_min = scan_center - 65.0f;
	const float scan_limit_max = scan_center + 65.0f;
	if (ctrl.pantile.mark_yaw > scan_limit_max) ctrl.pantile.mark_yaw = scan_limit_max;
	if (ctrl.pantile.mark_yaw < scan_limit_min) ctrl.pantile.mark_yaw = scan_limit_min;

	// 3. Pitch 轴扫描 (您的原始逻辑，保持不变)
	if (DM_motorPitch.set_pitch_pos >= DM_motorPitch.DM_pitch_max)
	{
		scan_reversal_pitch = false;
	}
	if (DM_motorPitch.set_pitch_pos <= DM_motorPitch.DM_pitch_min)
	{
		scan_reversal_pitch = true;
	}
	if (scan_reversal_pitch)
	{
		DM_motorPitch.set_pitch_pos += scan_speed_pitch;
	}
	else if (!scan_reversal_pitch)
	{
		DM_motorPitch.set_pitch_pos -= scan_speed_pitch;
	}

	if (!xuc.mode_TJ)
	{
		ctrl.pantile.angle2Keep += YAWstep;
       ctrl.pantile.angle2Keep = ctrl.GetDelta(ctrl.pantile.angle2Keep);
	}

}

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
    while (delta <= -180.0f)
	{
		delta += 360.0f;
	}

	while (delta > 180.0f)
	{
		delta -= 360.0f;
	}
	return delta;
}

float CONTROL::Getdelta(float delta)//弧度制
{
  while (delta <= -PI)
	{
		delta += 2.0f * PI;
	}
	while (delta > PI)
	{
		delta -= 2.0f * PI;
	}
	return delta;
} //该函数弃用，因为当delta非常大时，单次加减2PI可能无法将其归一化到[-PI, PI]范围内。下面是改进后的版本，使用循环确保无论输入多大，都能正确归一化

//float CONTROL::Getdelta(float delta)
//{
//	while (delta < -PI)
//	{
//		delta += 2.0f * PI;
//	}
//	while (delta >= PI)
//	{
//		delta -= 2.0f * PI;
//	}
//	return delta;
//}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

extern uint8_t Power_stsRx[];

unsigned int CONTROL::radian_to_mechanical_angle(double radian_angle) {
	// 核心转换公式：先将输入范围 [-π, π] 平移到 [0, 2π]，
	// 然后计算其在总范围中的比例，最后乘以目标范围的大小 8192。
	double mechanical_value = ((radian_angle + M_PI) / (2.0 * M_PI)) * 8192.0;

	// 对计算结果进行四舍五入，并转换为无符号整数类型后返回。
	// 加 0.5 是实现四舍五入的常用技巧。
	return (unsigned int)(mechanical_value + 0.5);
}

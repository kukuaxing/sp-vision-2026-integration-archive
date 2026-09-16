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
LowPassFilter yaw_rotation_speed_filter(0.8f);    // 灏忛檧铻?RESET/SHOOT 鍓嶉婊ゆ尝鍣?
LowPassFilter follow_rotation_speed_filter(0.8f); // FOLLOW妯″紡鐙珛鍓嶉婊ゆ尝鍣紝闃叉涓庡皬闄€铻哄叡浜姸鎬佹椂浜掔浉姹℃煋

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

void CONTROL::CHASSIS::setChassisspeed(int32_t ch_speedx_, int32_t ch_speedy_, int32_t ch_speedz_)//浠庨仴鎺у櫒鑾峰彇閫熷害鍑芥暟
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

void CONTROL::CHASSIS::speedCalculate(int32_t speedx_, int32_t speedy_, int32_t speedz_)//閫熷害瑙ｇ畻骞惰瀹氬嚱鏁?
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
	ch_YAW *= (-1.f);//鏂瑰悜鐩稿弽淇敼杩欓噷姝ｈ礋
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


		// 1. 瑙掑害璇樊
		float delta = degreeToRadian(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw()));

		// 2. 褰掍竴鍖栧埌 (-PI, PI)
		if (delta <= -PI)
			delta += 2.0f * PI;
		else if (delta >= PI)
			delta -= 2.0f * PI;

		// 3. PID 杈撳嚭锛坮ad/s锛?
		float pid_output = ctrl.pantile.keep_PID.Position(delta, 100.0f);

		// 4. 搴曠洏骞冲潎鐢垫満杞€燂紙rpm锛?
		avg_rpm =
			(ctrl.chassis_motor[0]->curspeed +
				ctrl.chassis_motor[1]->curspeed +
				ctrl.chassis_motor[2]->curspeed +
				ctrl.chassis_motor[3]->curspeed) / 4.0f;

		// 5. 浼扮畻搴曠洏鑷浆瑙掗€熷害锛坮ad/s锛?
		float chassis_rotation_speed =
			avg_rpm * 2.0f * PI / 60.0f / gear_ratio * wheel_radius / chassis_radius;

		// 6. 婊ゆ尝锛堝悇妯″紡浣跨敤鐙珛婊ゆ尝鍣紝闃叉鐘舵€佷簰鐩告薄鏌擄級
		float ff_rotation_speed = (ctrl.mode == CONTROL::FOLLOW)
			? follow_rotation_speed_filter.update(chassis_rotation_speed)
			: yaw_rotation_speed_filter.update(chassis_rotation_speed);

     // 7. 鍓嶉杞鍖猴細浣跨敤婊炲洖 + 骞虫粦锛岄伩鍏嶅噺閫熺┛瓒婇槇鍊兼椂鈥滄娊涓€涓嬧€?
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

		// 8. 鍓嶉鍙犲姞锛氳繃娓℃湡 / 鏅€氭棆杞?/ 灏忛檧铻?浣跨敤鍚勮嚜鐙珛澧炵泭
		float active_k_ff;
		if (ctrl.chassis.transition_from_rotation)
			active_k_ff = k_ff_transition;
		else if (ctrl.mode == CONTROL::FOLLOW)
			active_k_ff = k_ff_follow;
		else
			active_k_ff = k_ff;
		float set_speed = pid_output + active_k_ff * ff_rotation_speed;


		// 9. 杈撳嚭
		DM_motorYaw.set_speed = set_speed;
	}
}

void CONTROL::CHASSIS::Keep_Direction()//浠ヤ簯鍙版湞鍚戜负姝ｆ柟鍚戯紝搴曠洏鏂瑰悜瑙ｇ畻
{
	double s_x = speedx, s_y = speedy;
	// 鐢垫満涓庝簯鍙伴€氳繃鍚屾甯?榻胯疆杩炴帴锛屽瓨鍦ㄤ紶鍔ㄦ瘮 k锛堢數鏈鸿浣嶇Щ / 浜戝彴瑙掍綅绉伙級
	// 瀹為檯浜戝彴杞 = 鐢垫満杞 / k锛宬 鍙€氳繃瀵规瘮 IMU 涓庣數鏈鸿搴﹀彉鍖栨爣瀹?
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
		// 搴曠洏鏃嬭浆鑷冲垵濮嬬浉瀵逛綅缃細浠ョ數鏈虹紪鐮佸櫒璇樊椹卞姩搴曠洏鑷浆
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

	const bool auto_fire_mode = ctrl.mode == CONTROL::AUTO ||
		ctrl.mode == CONTROL::SHOOT || ctrl.mode == CONTROL::AUTOCONTROL;
	const bool fire_authorized = !auto_fire_mode || xuc.FireRequested(HAL_GetTick());
	if (supply_bullet && openRub && fire_authorized)
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
	static uint32_t last_update_ms = 0U;
	static float commanded_x_mps = 0.0f;
	static float commanded_y_mps = 0.0f;
	const uint32_t now_ms = HAL_GetTick();
	if (!xuc.HasFreshNavCommand(now_ms))
	{
		commanded_x_mps = 0.0f;
		commanded_y_mps = 0.0f;
		last_update_ms = now_ms;
		ctrl.chassis.navi_vx = 0.0f;
		ctrl.chassis.navi_vy = 0.0f;
		ctrl.chassis.navi_vz = 0.0f;
		DM_motorYaw.set_speed = 0.0f;
		ctrl.chassis.speedx = 0;
		ctrl.chassis.speedy = 0;
		ctrl.chassis.speedz = 0;
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
		return;
	}

	constexpr float M3508_RATIO = 19.2f;
	constexpr float WHEEL_DIAM_M = 0.15184f;
	constexpr float MS_TO_RPM = M3508_RATIO * 60.0f / (PI * WHEEL_DIAM_M);

	const uint32_t elapsed_ms = last_update_ms == 0U ? 0U : now_ms - last_update_ms;
	last_update_ms = now_ms;
	if (xuc.RxNav.max_accel > 0.0f && elapsed_ms > 0U)
	{
		const float max_delta = xuc.RxNav.max_accel * static_cast<float>(elapsed_ms) * 0.001f;
		const float delta_x = xuc.RxNav.linear_x - commanded_x_mps;
		const float delta_y = xuc.RxNav.linear_y - commanded_y_mps;
		commanded_x_mps += fmaxf(-max_delta, fminf(delta_x, max_delta));
		commanded_y_mps += fmaxf(-max_delta, fminf(delta_y, max_delta));
	}
	else
	{
		commanded_x_mps = xuc.RxNav.linear_x;
		commanded_y_mps = xuc.RxNav.linear_y;
	}

	const int32_t nav_x = static_cast<int32_t>(commanded_x_mps * MS_TO_RPM);
	const int32_t nav_y = static_cast<int32_t>(commanded_y_mps * MS_TO_RPM);

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
	// 妫€娴嬫槸鍚︽湁鐩爣
	target_detected = xuc.ControlRequested(HAL_GetTick());

	// 鏈夌洰鏍囦笖浜戝彴瀹為檯瑙掍笌涓婁綅鏈虹洰鏍囪璇樊瓒冲灏忔椂缃畉rue
	float yaw_err = xuc.yaw_TJ - degreeToRadian(imu_gimbal.angle.yaw);
	while (yaw_err > PI) yaw_err -= 2.0f * PI;
	while (yaw_err < -PI) yaw_err += 2.0f * PI;
	float pitch_err = xuc.pitch_TJ - degreeToRadian(imu_gimbal.angle.pitch);
	target_angle_aligned = target_detected
		&& (std::abs(yaw_err) < LOCK_THRESHOLD)
		&& (std::abs(pitch_err) < LOCK_THRESHOLD);

	// 鐘舵€佹満杞崲
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
			// 璁板綍鏈€鍚庢娴嬩綅缃紙灏忎簯鍙颁綅缃級
			last_yaw_pos = ctrl.pantile.mark_yaw;  // 灏廦aw
			last_pitch_pos = DM_motorPitch.set_pitch_pos;

			// 妫€鏌ユ槸鍚︾ǔ瀹氶攣瀹?
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
				scan_center_yaw = last_yaw_pos;  // 浠庝涪澶变綅缃紑濮嬫悳绱?
			}
			// 鐭殏涓㈠け鏃朵繚鎸佸綋鍓嶄綅缃瓑寰呴噸鏂版崟鑾?
		}
		is_locked = false;
		break;

	case LOCKED:
		if (target_detected) {
			lost_count = 0;
			// 缁х画妫€鏌ユ槸鍚﹁繕閿佸畾
			if (!target_angle_aligned) {
				stable_count = 0;
				aim_state = TRACK;  // 鍥炲埌璺熻釜鐘舵€?
			}
		} else {
			lost_count++;
			if (lost_count > LOST_TIMEOUT / 2) {  // 閿佸畾鐘舵€佹洿蹇繘鍏ユ悳绱?
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
				aim_state = IDLE;  // 鎼滅储瓒呮椂锛屽洖鍒扮┖闂?
			}
		}
		is_locked = false;
		break;
	}

	// 鍏煎鏃у彉閲?
	aim_lock = !target_detected && (lost_count > 50);
	aim_cont = lost_count;
}

void CONTROL::AUTOMATION::autoAim()
{
	// 鍏堟洿鏂扮姸鎬?
	stateUpdate();

	// 銆愬叧閿€戞湁鐩爣鏃剁姝㈡壂鎻忥紝璁﹎otor.cpp涓殑鑷瀯閫昏緫鎺ョ
	if (target_detected) {
		ctrl.g_start_scan = false;
	}

	switch (aim_state)
	{
	case IDLE:
		// 绌洪棽鐘舵€侊細淇濇寔闈欐锛岀瓑寰呮墜鍔ㄦ帶鍒舵垨鐩爣鍑虹幇
		// 鍙互鍚敤鎵弿
		ctrl.g_start_scan = true;
		break;

	case TRACK:
	case LOCKED:
		// 鑷瀯鐘舵€侊細绂佹鎵弿锛屼簯鍙扮敱motor.cpp涓殑鑷瀯閫昏緫鎺у埗
		// motor.cpp浼氭牴鎹畑uc.mode_TJ鍜寈uc.yaw_TJ鐩存帴璁＄畻鐩爣瑙掑害
		// 杩欓噷涓嶉渶瑕佹墜鍔ㄨ皟鏁磎ark_yaw锛屽惁鍒欎細鍐茬獊锛?
		ctrl.g_start_scan = false;

		if (target_detected) {
			// 璁板綍褰撳墠浣嶇疆锛岀敤浜庝涪澶卞悗鎼滅储锛堝皬浜戝彴浣嶇疆锛?
			last_yaw_pos = ctrl.pantile.mark_yaw;  // 灏廦aw
			last_pitch_pos = DM_motorPitch.set_pitch_pos;
		}
		// 涓㈠け鏃剁煭鏆備繚鎸侊紙涓嶅姩锛?
		break;

	case SEARCH:
		// 鎼滅储鐘舵€侊細鍦ㄤ涪澶变綅缃檮杩戝皬鑼冨洿鎼滅储

		ctrl.g_start_scan = false;  // 涓嶇敤鍏ㄥ眬鎵弿锛岀敤灞€閮ㄦ悳绱?
		searchTarget();
		break;
	}
}

void CONTROL::AUTOMATION::searchTarget()
{
	// 灏忚寖鍥存悳绱細浠ヤ涪澶变綅缃负涓績锛屽乏鍙虫憜鍔?
	// *** 淇锛氫娇鐢?mark_yaw锛堝皬浜戝彴锛夎€岄潪 DM_motorYaw锛堝ぇ浜戝彴锛?**
	float current_offset = ctrl.pantile.mark_yaw - scan_center_yaw;

	// 杈圭晫妫€娴?
	if (current_offset >= scan_range) {
		scan_reversal_yaw = true;   // 鍚戝乏
	} else if (current_offset <= -scan_range) {
		scan_reversal_yaw = false;  // 鍚戝彸
	}

	// 绉诲姩锛堟悳绱㈤€熷害鎱竴浜涳級- 鎺у埗灏忎簯鍙?


	// Pitch 灏忓箙搴︿笂涓嬫壂
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
	// 璋冪敤鏂扮殑鑷瀯閫昏緫
	autoAim();

	// 鏍规嵁鐘舵€佹帶鍒跺彂灏?
	if (aim_state == LOCKED || aim_state == TRACK) {
		ctrl.shooter.openRub = true;
		// 渚涘脊鐢盧C妗ｄ綅鎺у埗锛岃繖閲屼笉鑷姩寮€鐏?
	} else if (aim_state == SEARCH) {
		ctrl.shooter.openRub = true;
		ctrl.shooter.supply_bullet = false;
	} else {
		// IDLE鐘舵€?
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
	}
}
/**
 * @brief 鑷姩鎵弿鍑芥暟 (鏃犲弬鏁扮増鏈?
 * 娉ㄦ剰锛氭鍑芥暟渚濊禆浜?m_current_relative_angle 鎴愬憳鍙橀噺
 */
void CONTROL::AUTOMATION::autoScan()
{
	// *** 鍏抽敭淇锛氫慨澶嶅弽鍚戦€昏緫 ***
	// 浣跨敤 "鐢靛钩瑙﹀彂" 閫昏緫锛岃€屼笉鏄?"缈昏浆" 閫昏緫
	// m_current_relative_angle 蹇呴』鏄偍鍦ㄤ富寰幆涓В绠楀悗鐨勮繛缁浉瀵硅搴?

	Motor* yaw_motor = ctrl.pantile_motor[PANTILE::YAW];
	const float scan_center = (yaw_motor != nullptr && yaw_motor->yaw_limit_center_valid)
		? yaw_motor->yaw_limit_center_ref
		: 0.0f;

	// 1. Yaw 杞存壂鎻忥紙浠ESET璁板綍涓績涓哄熀鍑嗭級
	if (ctrl.m_current_relative_angle >= scan_center + 45.0f)
	{
		scan_reversal_yaw = false; // 蹇呴』鍚戣礋鏂瑰悜绉诲姩
	}
	else if (ctrl.m_current_relative_angle <= scan_center - 45.0f)
	{
		scan_reversal_yaw = true; // 蹇呴』鍚戞鏂瑰悜绉诲姩
	}
	// (濡傛灉浠嬩簬杈圭晫涔嬮棿锛宻can_reversal_yaw 淇濇寔涓嶅彉锛岀户缁湞褰撳墠鏂瑰悜绉诲姩)


	// 2.1 鎵弿鐩爣闄愬箙锛堝悓鏍蜂互RESET璁板綍涓績涓哄熀鍑嗭級
	const float scan_limit_min = scan_center - 65.0f;
	const float scan_limit_max = scan_center + 65.0f;
	if (ctrl.pantile.mark_yaw > scan_limit_max) ctrl.pantile.mark_yaw = scan_limit_max;
	if (ctrl.pantile.mark_yaw < scan_limit_min) ctrl.pantile.mark_yaw = scan_limit_min;

	// 3. Pitch 杞存壂鎻?(鎮ㄧ殑鍘熷閫昏緫锛屼繚鎸佷笉鍙?
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

float CONTROL::Getdelta(float delta)//寮у害鍒?
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
} //璇ュ嚱鏁板純鐢紝鍥犱负褰揹elta闈炲父澶ф椂锛屽崟娆″姞鍑?PI鍙兘鏃犳硶灏嗗叾褰掍竴鍖栧埌[-PI, PI]鑼冨洿鍐呫€備笅闈㈡槸鏀硅繘鍚庣殑鐗堟湰锛屼娇鐢ㄥ惊鐜‘淇濇棤璁鸿緭鍏ュ澶э紝閮借兘姝ｇ‘褰掍竴鍖?

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
	// 鏍稿績杞崲鍏紡锛氬厛灏嗚緭鍏ヨ寖鍥?[-蟺, 蟺] 骞崇Щ鍒?[0, 2蟺]锛?
	// 鐒跺悗璁＄畻鍏跺湪鎬昏寖鍥翠腑鐨勬瘮渚嬶紝鏈€鍚庝箻浠ョ洰鏍囪寖鍥寸殑澶у皬 8192銆?
	double mechanical_value = ((radian_angle + M_PI) / (2.0 * M_PI)) * 8192.0;

	// 瀵硅绠楃粨鏋滆繘琛屽洓鑸嶄簲鍏ワ紝骞惰浆鎹负鏃犵鍙锋暣鏁扮被鍨嬪悗杩斿洖銆?
	// 鍔?0.5 鏄疄鐜板洓鑸嶄簲鍏ョ殑甯哥敤鎶€宸с€?
	return (unsigned int)(mechanical_value + 0.5);
}

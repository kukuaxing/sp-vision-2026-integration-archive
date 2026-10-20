#include "control.h"
#ifndef SENTINEL_ACTUATOR_HOST_TEST
#include "tim.h"
#include "judgement.h"
#endif
#include "HTmotor.h"
#include "motor.h"

void CONTROL::init_dm()//达妙单独初始化
{
	DMmotor[0].setPos = para.dm_initial_pitch;   // Pitch：P_S 模式，setPos 有效
	DMmotor[1].setSpeed = para.dm_initial_yaw;                  // Yaw：SPEED 模式，只有 setSpeed 有效
	init_DM = 1;                                 // 标志置位，表示已初始化
}
void CONTROL::Init(std::vector<Motor*> motor)//普通电机初始化
{
	for (int i = 0; i < motor.size(); i++)
		//新增了容量检查，因为每个类型的电机的内存是连续的，
		//所以如果不小心多了电机可能会有覆盖问题
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			if (chassis_num < CHASSIS_MOTOR_NUM) chassis_motor[chassis_num++] = motor[i];
			break;
		case(function_type::pantile):
			if (pantile_num < PANTILE_MOTOR_NUM) {
				pantile_motor[motor[i]->ID == ID5 ? PANTILE::TYPE::YAW
					: PANTILE::TYPE::PITCH] = motor[i];
				++pantile_num;
			}
			break;
		case(function_type::shooter):
			if (shooter_num < SHOOTER_MOTOR_NUM) shooter_motor[shooter_num++] = motor[i];
			break;
		case(function_type::supply):
			if (supply_num < SUPPLY_MOTOR_NUM) {
				supply_motor[supply_num] = motor[i];              // 先放进槽
				supply_motor[supply_num]->spinning = false;       // 再通过指针写成员
				supply_motor[supply_num]->need_curcircle = false;
				++supply_num;
			}
			break;//就算是最后一个，也要补上break;
		default:
			break;
		}
	}
	if (pantile_motor[PANTILE::TYPE::PITCH]) pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	if (pantile_motor[PANTILE::TYPE::YAW])   pantile_motor[PANTILE::TYPE::YAW]->setangle = para.initial_yaw;

}
float CONTROL::ClampAngle(float setangle, float center, float span)
{
	float off = (float)Motor::getdeltaa((int16_t)(setangle - center));  // 相对中点的带符号偏移
	off = std::max(std::min(off, span), -span);                        // 钳到 ±半宽
	setangle = center + off;
	if (setangle < 0.f)     setangle += 8192.f;                        // 回绕归一
	if (setangle >= 8192.f) setangle -= 8192.f;
	return setangle;
}

void CONTROL::PANTILE::Control_Pantile(int32_t ch_dji_yaw, int32_t ch_pitch, int32_t ch_dm_yaw)
{
	Reset_Aim_References();
	hold_position = false;
	// 1. 大疆Yaw/位置累加
	if (ctrl.pantile_motor[PANTILE::TYPE::YAW] && ctrl.pantile_motor[PANTILE::TYPE::YAW]->has_feedback)//feedback和他本身不为空
	{
		// 满杆 180°/s，5ms 周期
		const float yaw_rate = 8192.f * 0.5f;
		// 摇杆有推量时累加目标角度；摇杆回中(0)时目标角度自然保持不变
		ctrl.pantile_motor[PANTILE::TYPE::YAW]->setangle += -(float)ch_dji_yaw / 660.f * yaw_rate * 0.005f;
	}

	// 2. 达妙 Pitch 轴（位置模式）
	mark_pitch += -(float)ch_pitch / 660.f * (PI * 0.5f) * 0.005f;

	// 3. 达妙 Yaw 轴（纯速度模式）
	// 速度模式：摇杆推多少就给多大角速度，传 0 就立刻刹停
	const float max_dm_speed = PI; // 最大 180°/s
	yaw_speed_out = (float)ch_dm_yaw / 660.f * max_dm_speed;


}

void CONTROL::PANTILE::Hold_Pantile()
{
	Synchronize_Feedback_Recovery();
	Reset_Aim_References();
	Motor* yaw = ctrl.pantile_motor[PANTILE::TYPE::YAW];
	if (!yaw || !yaw->FeedbackFresh()) hold_yaw_captured = false;
	if (!DMmotor[0].FeedbackFresh()) hold_pitch_captured = false;
	if (!DMmotor[1].FeedbackFresh()) hold_dm_yaw_captured = false;
	if (!hold_position) {
		hold_yaw_captured = false;
		hold_pitch_captured = false;
		hold_dm_yaw_captured = false;
	}
	hold_position = true;
	yaw_speed_out = 0.f;
	Motor* y = ctrl.pantile_motor[PANTILE::TYPE::YAW];
	if (!hold_yaw_captured && y && y->FeedbackFresh() && y->has_feedback && y->angle_latched &&
	    std::isfinite(y->angle[now]) && y->angle[now] >= 0.f && y->angle[now] < 8192.f) {
		y->setangle = y->angle[now];
		hold_yaw_captured = true;
	}
	// decoded is only set after a real frame with this motor ID was decoded.
	// Fault/uninitialized feedback must not overwrite the previous goal with zero.
	// Latch each axis once per hold entry. Following every new feedback position
	// would remove restoring position error and let gravity/external loads drift.
	if (!hold_pitch_captured && DMmotor[0].FeedbackFresh() && DMmotor[0].decoded && DMmotor[0].status <= 1u && std::isfinite(DMmotor[0].pos)) {
		mark_pitch = DMmotor[0].pos;
		DMmotor[0].setPos = mark_pitch;
		if (!pitch_ready) pitch0 = mark_pitch;
		pitch_ready = true;
		hold_pitch_captured = true;
	}
	if (!hold_dm_yaw_captured && DMmotor[1].FeedbackFresh() && DMmotor[1].decoded && DMmotor[1].status <= 1u && std::isfinite(DMmotor[1].pos)) {
		yaw_lock_pos = DMmotor[1].pos;
		yaw_lock_ready = true;
		DMmotor[1].setSpeed = 0.f;
		hold_dm_yaw_captured = true;
	}
}

void CONTROL::PANTILE::Synchronize_Feedback_Recovery()
{
	Motor* y = ctrl.pantile_motor[PANTILE::TYPE::YAW];
	const uint32_t yaw_generation = y ? y->FeedbackGeneration() : 0;
	const uint32_t pitch_generation = DMmotor[0].FeedbackGeneration();
	const uint32_t dm_yaw_generation = DMmotor[1].FeedbackGeneration();
	if (yaw_generation != yaw_feedback_generation || pitch_generation != pitch_feedback_generation) {
		Reset_Aim_References();
		ctrl.shooter.CancelPendingFeed();
	}
	if (yaw_generation != yaw_feedback_generation) hold_yaw_captured = false;
	if (pitch_generation != pitch_feedback_generation) hold_pitch_captured = pitch_ready = false;
	if (dm_yaw_generation != dm_yaw_feedback_generation) hold_dm_yaw_captured = yaw_lock_ready = false;
	yaw_feedback_generation = yaw_generation;
	pitch_feedback_generation = pitch_generation;
	dm_yaw_feedback_generation = dm_yaw_generation;
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type,IMU frameOfReference)
{
	//下面是写好的函数，注意有两个变量要写到para里面，这里单独使用一个死区和pid
	//Motor* y = ctrl.pantile_motor[PANTILE::TYPE::YAW];
	//if (!y || !y->has_feedback) return;

	//// frameOfReference 传 imu_pantile ⇒ 它的 yaw 就是枪口世界朝向
	//const float err = ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw());
	//if (fabsf(err) < KEEP_DB) return;                    // 死区，防抖

	//y->setangle += KEEP_KP * err * (8192.f / 360.f);     // 度 → 编码值
}


void CONTROL::CHASSIS::Keep_Direction(int32_t ch_fwd, int32_t ch_left)
{
	const float dyaw = ctrl.GetDelta(imu_pantile.GetAngleYaw() - imu_chassis.GetAngleYaw());
	const float rad = dyaw * PI / 180.f;

	const float fwd = (float)ch_fwd * 4000.f / 660.f;   // 前后
	const float left =-(float)ch_left * 4000.f / 660.f;   // 左右

	speedx = fwd * cosf(rad) - left * sinf(rad);
	speedy = fwd * sinf(rad) + left * cosf(rad);
}


void CONTROL::CHASSIS::Update()
{
	// ① 取指令；RESET 模式直接归零，不留残留
	float vx = 0.f, vy = 0.f, wz = 0.f;
	if (ctrl.mode != CONTROL::RESET)
	{
		vx = (float)speedx;
		vy = (float)speedy;
		wz = (float)speedz;
	}

	// ② 45° X 布局逆解（数组下标 = 你的电机编号 - 1）
	const float K = 1.0f;
	float w[4];
	w[0] = -vx - vy + K * wz;   // 1 左前
	w[1] = +vx - vy + K * wz;   // 2 右前
	w[2] = +vx + vy + K * wz;   // 3 右后
	w[3] = -vx + vy + K * wz;   // 4 左后

	// ③ 下发，顺手限到各自 maxspeed
	for (int i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		if (!ctrl.chassis_motor[i]) continue;
		float lim = (float)ctrl.chassis_motor[i]->maxspeed;
		ctrl.chassis_motor[i]->setspeed = (int32_t)std::max(std::min(w[i], lim), -lim);
	}
}

void CONTROL::PANTILE::Update()
{
	Synchronize_Feedback_Recovery();
	// 1. 6020yaw
	if (ctrl.pantile_motor[PANTILE::TYPE::YAW] && !hold_position)
		ctrl.pantile_motor[PANTILE::TYPE::YAW]->setangle =
		ClampAngle(ctrl.pantile_motor[PANTILE::TYPE::YAW]->setangle, para.yaw_center, para.yaw_span);// 钳到 ±半宽

	// 2. 达妙 Pitch 保护与更新
	if (DMmotor[0].DriveReady() && std::isfinite(DMmotor[0].pos))
	{
		if (!pitch_ready) { mark_pitch = DMmotor[0].pos;pitch0 = mark_pitch; pitch_ready = true; }
		// Holding a measured pose must not command a return to a soft-limit edge.
		if (!hold_position) LIMIT_MIN_MAX(mark_pitch, para.dm_pitch_min, para.dm_pitch_max);
		DMmotor[0].setPos = mark_pitch;
		DMmotor[0].setSpeed = 1.5f;
	}
	else {
		pitch_ready = hold_pitch_captured = false;
		Reset_Aim_References();
	}

	// 3. 达妙 Yaw 下发速度指令
	// 如果上层传 0，这里就会下发 0 rad/s，电机依靠自身的阻尼和速度环稳稳刹住
	if (DMmotor[1].DriveReady() && std::isfinite(DMmotor[1].pos))
	{
		if (!yaw_lock_ready) { yaw_lock_pos = DMmotor[1].pos; yaw_lock_ready = true; }
		DMmotor[1].setSpeed = yaw_speed_out                                  // 手动微调（rad/s）
			+ yaw_lock_Kp * (yaw_lock_pos - DMmotor[1].pos); // 软锁回位
	}
	else { yaw_lock_ready = hold_dm_yaw_captured = false; DMmotor[1].setSpeed = 0.f; }

}
void CONTROL::SHOOTER::CancelPendingFeed()
{
    Motor* d = ctrl.supply_motor[0];
    const bool pending = d && d->angle_latched &&
        std::fabs(d->setangle - float(d->sum_angle)) > 1.f;
    // Losing authorization after a ready/issued shot requires a new trigger.
    // Initial wheel spin-up, before any shot was possible, is an ordinary wait.
    if (feed_permission_was_ready || pending) feed_rearm_required = true;
    feed_permission_was_ready = false;
    trig_pre = false; hold_ms = rate_ms = 0;
    if (d) {
        taskENTER_CRITICAL();
        d->CancelPositionTarget();
        taskEXIT_CRITICAL();
    }
}

void CONTROL::SHOOTER::Update()
{
    if (ctrl.shooter_motor[0]) ctrl.shooter_motor[0]->setspeed = openRub ? shoot_speed : 0;
    if (ctrl.shooter_motor[1]) ctrl.shooter_motor[1]->setspeed = openRub ? -shoot_speed : 0;

    const int16_t T_ON = 300, T_OFF = 200;
    if (!trig && trig_raw > T_ON) trig = true;
    if (trig && trig_raw < T_OFF) trig = false;
    const bool trig_eff = trig || auto_shoot;
    const bool request_level = trig || auto_fire_request_level;
    if (!request_level) feed_rearm_required = false;

    Motor* d = ctrl.supply_motor[0];
    Motor* left = ctrl.shooter_motor[0];
    Motor* right = ctrl.shooter_motor[1];
    const uint32_t left_generation = left ? left->FeedbackGeneration() : 0;
    const uint32_t right_generation = right ? right->FeedbackGeneration() : 0;
    const uint32_t feeder_generation = d ? d->FeedbackGeneration() : 0;
    if (left_generation != left_feedback_generation || right_generation != right_feedback_generation ||
        feeder_generation != feeder_feedback_generation) CancelPendingFeed();
    left_feedback_generation = left_generation;
    right_feedback_generation = right_generation;
    feeder_feedback_generation = feeder_generation;
    const float threshold = float(shoot_speed) * 0.8f;
    // Both wheels must be fresh, decoded and turning in their commanded
    // directions. An average can hide a stalled or reversed wheel.
    const bool wheels_ready = left && right &&
        left->FeedbackFresh() && right->FeedbackFresh() &&
        left->has_feedback && right->has_feedback && left->temperature <= 70 && right->temperature <= 70 &&
        left->curspeed >= threshold && right->curspeed <= -threshold;
    const bool allowed_mode = ctrl.mode == CONTROL::AUTO || ctrl.mode == CONTROL::FIRE;
    const bool allow = allowed_mode && openRub && wheels_ready && d &&
        d->FeedbackFresh() && d->has_feedback && d->angle_latched;
    if (!allow) {
        CancelPendingFeed();
        // A low trigger observed during the fault already satisfies re-arming.
        if (!request_level) feed_rearm_required = false;
        return;
    }
    if (feed_rearm_required) {
        d->CancelPositionTarget();
        return;
    }
    feed_permission_was_ready = true;

    const uint16_t TICK = 5, T_ARM = 300, T_RATE = 125;
    bool fire_pulse = false, fire_cont = false;
    if (trig_eff && !trig_pre) { fire_pulse = true; hold_ms = 0; rate_ms = T_RATE; }
    else if (trig_eff) {
        if (hold_ms < 60000) hold_ms += TICK;
        if (hold_ms >= T_ARM) fire_cont = true;
    }
    trig_pre = trig_eff;
    if (fire_pulse) d->setangle -= para.bullet_step;
    if (fire_cont) {
        if (rate_ms >= T_RATE) { rate_ms = 0; d->setangle -= para.bullet_step; }
        else rate_ms += TICK;
    } else rate_ms = 0;

    const float lead = d->setangle - float(d->sum_angle);
    if (lead > para.bullet_lead_max) d->setangle = float(d->sum_angle) + para.bullet_lead_max;
    if (lead < -para.bullet_lead_max) d->setangle = float(d->sum_angle) - para.bullet_lead_max;
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

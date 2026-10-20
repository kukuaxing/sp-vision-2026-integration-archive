#include "label.h"
#include "HTmotor.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "xuc.h"
#include "imu.h"
#include <algorithm>

void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

void RC::OnRC()
{
	const bool was_auto = ctrl.mode == CONTROL::AUTO;
	//RC失联保护，失联撤销运动
	const bool rc_lost = last_rc_tick == 0 ||
        (xTaskGetTickCount() - last_rc_tick) > pdMS_TO_TICKS(RC_LOST_MS);
	if (rc_lost) ctrl.mode = CONTROL::STOP;
	else RC_CheckState();
	// Capture before manual increments on AUTO exit; STOP latches each valid axis once.
	if (ctrl.mode == CONTROL::STOP || (was_auto && ctrl.mode != CONTROL::AUTO))
		ctrl.pantile.Hold_Pantile();
	RC_Control();

	if (rc_lost)
	{
		if (rc_lost_cnt < 60000) ++rc_lost_cnt;
		// Physical remote loss also revokes AUTO; OnPC cannot override STOP.
		ctrl.chassis.speedx = 0;
		ctrl.chassis.speedy = 0;
		ctrl.chassis.speedz = 0;
		ctrl.shooter.trig_raw = 0;
		ctrl.shooter.openRub = false;
		ctrl.pantile.yaw_speed_out = 0;    // 防达妙 yaw 吃残值漂移
	}
	else
	{
		rc_lost_cnt = 0;
	}


	if (Shift_mode())
	{
		ctrl.pantile.mark_yaw = (float)ctrl.pantile_motor[CONTROL::PANTILE::YAW]->angle[now];

	}
}

void RC::OnPC()
{
	/* 只在 AUTO 模式接管。RC_Control() 先跑并写入摇杆值，
	 * 本函数后跑 ⇒ 天然覆盖顺序正确，不需要额外插旗标。*/
	if (ctrl.mode != CONTROL::AUTO) {
        ctrl.shooter.auto_fire_request_level = false;
        xuc.RevokeAim();
        xuc.nav_accepted = xuc.nav_enable = false;
        ctrl.shooter.auto_shoot = false;
        if (ctrl.mode != CONTROL::FIRE) ctrl.shooter.CancelPendingFeed();
        return;
    }

	ApplyNav();     // 0x11 → 底盘
	ApplyAim();     // 0x10 → 云台 / 摩擦轮 / 开火
}

/* ==========================================================================
 * 底盘：视觉导航速度（m/s）→ 轮速给定（rpm）
 *
 * 口径：三个量级 —— ① 限目标 ② 限增量 ③ 下发。
 *   限幅的对象是【目标】而不是【输出】，prespeed 因此被夹在 tgt 附近，
 *   天然不会积压，也就不需要再 clamp 一次。
 *
 * 失联语义：NAV 租约到期（NavReady()==false）时，
 *   - 遥控在线 ⇒ 保留 RC_Control 写的摇杆值（台架调试仍能推杆挪车）
 *   - 遥控也失联 ⇒ 强制停（两路都没有主人时车必须停）
 * ========================================================================== */
void RC::ApplyNav()
{
	const bool rc_lost = (xTaskGetTickCount() - last_rc_tick) > pdMS_TO_TICKS(RC_LOST_MS);

	if (xuc.NavReady())
	{
		// ① 目标钳位
		const float tgt_x = std::max(std::min(xuc.nav_vx * para.K_NAV, para.NAV_MAX), -para.NAV_MAX);
		const float tgt_y = std::max(std::min(xuc.nav_vy * para.K_NAV, para.NAV_MAX), -para.NAV_MAX);

		// ② 变化率限行（每周期最多 ±DV_MAX rpm）
		float dvx = tgt_x - xuc.prespeedx;
		dvx = std::max(std::min(dvx, para.DV_MAX), -para.DV_MAX);
		float dvy = tgt_y - xuc.prespeedy;
		dvy = std::max(std::min(dvy, para.DV_MAX), -para.DV_MAX);

		xuc.prespeedx += dvx;
		xuc.prespeedy += dvy;

		// ③ 下发（int32_t 赋值即隐式取整；prespeed 不越界，无须再钳）
		ctrl.chassis.speedx = (int32_t)xuc.prespeedx;
		ctrl.chassis.speedy = (int32_t)xuc.prespeedy;
		ctrl.chassis.speedz = 0;   // nav_wz 首版未启用：转向仍由遥控/策略决定
		return;
	}

	/* --- 无有效导航许可 --- */
	xuc.prespeedx = 0.f;      // 复位平滑量，避免许可恢复瞬间从残值暴冲
	xuc.prespeedy = 0.f;
	if (rc_lost) StopChassis();
}

void RC::StopChassis()
{
	ctrl.chassis.speedx = 0;
	ctrl.chassis.speedy = 0;
	ctrl.chassis.speedz = 0;
	xuc.prespeedx = 0.f;
	xuc.prespeedy = 0.f;
}

/* ==========================================================================
 * 云台 / 发射：0x10 AIM_COMMAND
 *
 * ⚠️ 目标来自 xuc.aim_yaw_target / aim_pitch_target —— 这是【绝对目标】，
 *    由 xuc.cpp 的 HandleAim() 用「参考样本姿态 + 上位机给的误差」一次性
 *    重建（V2 §7.3）。本函数用当前IMU误差生成当前位置附近的有界参考，
 *    不把串口误差或IMU误差继续累加到旧电机目标。
 *
 * IMU给出绝对方向误差，编码器给出本周期位置基准；mark_pitch / setangle
 * 是位置参考，不能当作无界积分后的速率指令。
 * ========================================================================== */
void RC::ApplyAim()
{
	ctrl.shooter.auto_fire_request_level = xuc.aim_fire_req;
	ctrl.pantile.Synchronize_Feedback_Recovery();
#if XUC_ALLOW_CONTROL == 0
	/* 只读联调版：0x10 会被完整解析、校验、重建目标并记入 last_reject，
	 * 但本地不放行 ⇒ 不驱动任何执行器（连摩擦轮都不转）。
	 * 改这个宏之前必须先完成：
	 *   A 实验 —— sample_time_us 误差界 ≤ 2 ms（SAMPLE_TIME_VALID）
	 *   B 实验 —— 单轴 / 参考系 / gyro 符号（AXES_VERIFIED）
	 * 否则姿态与角速度不在同一可信参考系里，拿来闭环会系统性打偏。*/
	ctrl.shooter.openRub = false;
	ctrl.shooter.auto_shoot = false;
#else
	// Host sends an independently renewed wheel lease during the five-second
	// detection-loss hold. Missing/stale host data stops wheels in <=100ms.
	ctrl.shooter.openRub = xuc.FrictionReady();
	if (!xuc.AimReady()) {
		ctrl.pantile.Hold_Pantile();
        ctrl.shooter.auto_shoot = false;
        ctrl.shooter.CancelPendingFeed();
        return;
    }   // No visual/lease permission: discard queued motion and hold decoded pose.
	const float DB      = 0.3f * PI / 180.f;   // 死区，rad
	const float DEG2RAD = PI / 180.f;
	const float RAD2CNT = 8192.f / (2.f * PI);

	// Both position servos must have usable feedback before AUTO moves either
	// axis. DriveReady includes real CAN freshness, enabled status and the
	// latched-fault/startup handshake; status=0 alone never grants AUTO.
	Motor* y = ctrl.pantile_motor[CONTROL::PANTILE::YAW];
	const bool yaw_feedback_valid = y && y->FeedbackFresh() && y->has_feedback && y->angle_latched &&
		std::isfinite(y->angle[now]) && y->angle[now] >= 0.f && y->angle[now] < 8192.f;
	const bool pitch_feedback_valid = DMmotor[0].DriveReady() &&
		std::isfinite(DMmotor[0].pos);
	// An out-of-range measured pose is held, not pulled to a soft-limit edge on
	// takeover. Manual operation retains its existing limit/recovery semantics.
	const bool within_limits = yaw_feedback_valid && pitch_feedback_valid &&
		fabsf((float)Motor::getdeltaa((int16_t)(y->angle[now] - para.yaw_center))) <= para.yaw_span &&
		DMmotor[0].pos >= para.dm_pitch_min && DMmotor[0].pos <= para.dm_pitch_max;
	const float perr = xuc.aim_pitch_target - imu_pantile.GetAnglePitch() * DEG2RAD;
	const float yerr = XUC::WrapPi(xuc.aim_yaw_target - imu_pantile.GetAngleYaw() * DEG2RAD);
	if (!within_limits || !std::isfinite(perr) || !std::isfinite(yerr)) {
		ctrl.pantile.Hold_Pantile();
		ctrl.shooter.auto_shoot = false;
		ctrl.shooter.CancelPendingFeed();
		return;
	}

	// This is a proportional position correction, not an error integral. Reuse
	// of the same packet or frozen encoder can never accumulate an old goal.
	// The lead caps constrain references; they do not certify motor speed or stability.
	const float KP = 0.25f, KY = 0.25f;
	// ClampAngle quantizes to whole counts; 22 counts stays below 1 degree.
	const float YAW_LEAD_MAX = 22.f;
	const float PITCH_LEAD_MAX = 0.5f * DEG2RAD; // 0.5 degree, radians
	float yaw_reference, pitch_reference;
	const bool yaw_ready = ctrl.pantile.aim_yaw_reference.Update(
		y->angle[now], fabsf(yerr) > DB ? KY * yerr * RAD2CNT : 0.f,
		YAW_LEAD_MAX, yaw_reference);
	const bool pitch_ready = ctrl.pantile.aim_pitch_reference.Update(
		DMmotor[0].pos, fabsf(perr) > DB ? KP * perr : 0.f,
		PITCH_LEAD_MAX, pitch_reference);
	if (!yaw_ready || !pitch_ready) {
		ctrl.pantile.Hold_Pantile();
		ctrl.shooter.auto_shoot = false;
		ctrl.shooter.CancelPendingFeed();
		return;
	}
	ctrl.pantile.hold_position = false;
	y->setangle = yaw_reference;
	ctrl.pantile.mark_pitch = pitch_reference;

	/* Wheel-only heartbeats never enable angle control or feeding. */
	ctrl.shooter.auto_shoot = xuc.aim_valid && xuc.aim_friction_en && xuc.aim_fire_req;
#endif
}

void RC::Update()
{
	OnRC();
	OnPC();
}

void RC::RC_CheckState() {

	switch (RC_STATE(rc.s[0], rc.s[1]))
	{
	case RC_STATE(UP, UP):
		ctrl.mode = CONTROL::AUTO;
		break;

	case RC_STATE(UP, MID):
		ctrl.mode = CONTROL::STOP;
		break;

	case RC_STATE(UP, DOWN):
		ctrl.mode = CONTROL::SPINNING;
		break;

	case RC_STATE(MID, UP):
		ctrl.mode = CONTROL::FOLLOW;
		break;

	case RC_STATE(MID, MID):
		ctrl.mode = CONTROL::RESET;
		break;

	case RC_STATE(MID, DOWN):
		ctrl.mode = CONTROL::SEPARATE;
		break;

	case RC_STATE(DOWN, UP):
		ctrl.mode = CONTROL::FIRE;
		break;

	case RC_STATE(DOWN, MID):
		ctrl.mode = CONTROL::RESET;
		break;

	case RC_STATE(DOWN, DOWN):
		ctrl.mode = CONTROL::ROTATION;
		break;

	default:
		break;
	}
}

void RC::RC_Control() {
	if (ctrl.mode != CONTROL::RESET)
	{

		ctrl.shooter.trig_raw = 0;
		ctrl.chassis.speedz = 0;
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;

		//ctrl.chassis.Keep_Direction();

		switch (ctrl.mode)
		{
		case CONTROL::ROTATION://小陀螺
			ctrl.chassis.speedx = rc.ch[1] * 4000.f / 660.f;
			ctrl.chassis.speedy = -1 * rc.ch[0] * 4000.f / 660.f;
			ctrl.chassis.speedz = rc.ch[2];//para.rota_speed;             // 不硬编码 1000
			ctrl.pantile.Control_Pantile(0, 0, rc.ch[3]);//(0, rc.ch[3],rc.ch[2]);
			break;

		case CONTROL::FOLLOW://正方向为云台方向，跟随云台视角
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			//先要读取云台的角度，根据云台的方向计算出具体的speedx和speedy
			ctrl.chassis.Keep_Direction(rc.ch[1], rc.ch[0]); //重点是写这个按照云台的角度来计算出speedx和speedy，以云台的角度为参考系
			//其实就是重新计算speedx和speedy，把之前的覆盖
			ctrl.pantile.Control_Pantile(rc.ch[2], rc.ch[3],0);
			break;

		case CONTROL::SEPARATE://底盘与云台分离，底盘不受云台影响
			ctrl.chassis.speedx = rc.ch[1] * 4000.f / 660.f;
			ctrl.chassis.speedy = -1 * rc.ch[0] * 4000.f / 660.f;
			ctrl.pantile.Control_Pantile(rc.ch[2], rc.ch[3],0);
			break;

		case CONTROL::AUTO://自动瞄准
			ctrl.chassis.speedx = rc.ch[1] * 4000.f / 660.f;
			ctrl.chassis.speedy = -1 * rc.ch[0] * 4000.f / 660.f;
			ctrl.chassis.speedz = rc.ch[2];
			ctrl.pantile.yaw_speed_out = 0;

			//这里还要写射击，给xuc接管
			break;

		case CONTROL::FIRE://射击
			//ctrl.chassis.speedx = rc.ch[1] * 4000.f / 660.f;
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;//-1 * rc.ch[0] * 4000.f / 660.f;
			ctrl.chassis.speedz = 0;
			ctrl.shooter.trig_raw = -rc.ch[1];         // 把 ch[1] 的模拟量传入扳机
			ctrl.pantile.Control_Pantile(rc.ch[2], rc.ch[3],0);
			ctrl.shooter.openRub = true;                       // 意图交给 SHOOTER::Update()
			//这里要重新分配ch[0]和ch[1]的值，作为射击的控制，射击的时候底盘可以不动
			break;

		case CONTROL::STOP://停止
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;
			// OnRC captured the measured pose; do not restore a queued position goal.
			ctrl.shooter.trig_raw = 0;
			break;

		case CONTROL::SPINNING://超级雷霆大转盘
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = rc.ch[0] * para.max_speed / 660.f; //底盘旋转
			ctrl.pantile.Control_Pantile(rc.ch[2], rc.ch[3], rc.ch[1]);         // 小yaw，pitch，大yaw
			break;

		default:
			ctrl.chassis.speedx = 0;
			ctrl.chassis.speedy = 0;
			ctrl.chassis.speedz = 0;
			break;
		}
	}
	else {
		ctrl.chassis.speedx = 0;
		ctrl.chassis.speedy = 0;
		ctrl.chassis.speedz = 0;//这里不能写setspeed,那个只能在电机的update里写
		ctrl.shooter.trig_raw = 0;

		if (ctrl.pantile_motor[CONTROL::PANTILE::TYPE::YAW])
			ctrl.pantile_motor[CONTROL::PANTILE::TYPE::YAW]->setangle =
			ctrl.pantile_motor[CONTROL::PANTILE::TYPE::YAW]->angle[now];

		ctrl.shooter.openRub = false;
		ctrl.shooter.trig_raw = 0;                // 确保退出开火时绝不留残余信号
		ctrl.shooter.supply_bullet = false;

		ctrl.pantile.Control_Pantile(0, 0, 0);
	}
}

void RC::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // 或者报错
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	}
	if (pd_Rx != pdTRUE) return;
	if (sizeof(m_frame) < 18) return;
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

	last_rc_tick = xTaskGetTickCount();//收到有效帧

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
	if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
	if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
	if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

	pre_rc.s[0] = rc.s[0];
	pre_rc.s[1] = rc.s[1];

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);

	pc.x = m_frame[6] | (m_frame[7] << 8);
	pc.y = m_frame[8] | (m_frame[9] << 8);
	pc.z = m_frame[10] | (m_frame[11] << 8);
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];

	pc.key_h = m_frame[15];//按键的高位部分R F G Z X C
	pc.key_l = m_frame[14];//按键的低8位 W S A D SHIFT CTRL Q E

}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}

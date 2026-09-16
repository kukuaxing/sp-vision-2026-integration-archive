#include "label.h"
#include "RC.h"
#include "control.h"
#include "DMmotor.h"
#include "nuc.h"
#include "judgement.h"
extern DMMOTOR DM_motorYaw;

namespace {
  static constexpr float PI_F = 3.14159265358979323846f;
	static constexpr int32_t GYRO_SPIN_MIN = 300;
	static constexpr int32_t GYRO_SPIN_MAX = 660;
	static constexpr float GYRO_SPIN_K = 1.2f; // 变化速度参数，单位 rad/s

	struct GyroSpinState
	{
		TickType_t last_tick = 0;
		float phase = -PI_F * 0.5f; // 从下限开始
	};

	static GyroSpinState g_gyro_spin_state;

	static inline TickType_t GetTickSafe()
	{
		if (__get_IPSR() != 0U)
		{
			return xTaskGetTickCountFromISR();
		}
		return xTaskGetTickCount();
	}

	static inline float TickDiffToSeconds(TickType_t now, TickType_t last)
	{
		return (now - last) * portTICK_PERIOD_MS * 0.001f;
	}

	static inline int32_t CalcStageNavCmd(float target_m, float traveled_m, int32_t nav_cmd_min, int32_t nav_cmd_max, float nav_decel_window_m)
	{
		float remain_m = target_m - traveled_m;
		if (remain_m < 0.0f)
		{
			remain_m = 0.0f;
		}

		float ratio = remain_m / nav_decel_window_m;
		if (ratio > 1.0f) ratio = 1.0f;
		if (ratio < 0.0f) ratio = 0.0f;

		return (int32_t)(nav_cmd_min + (nav_cmd_max - nav_cmd_min) * ratio);
	}

	static inline int32_t CalcVariableGyroSpin(bool reset)
	{
		TickType_t now = GetTickSafe();
		if (reset || g_gyro_spin_state.last_tick == 0)
		{
			g_gyro_spin_state.last_tick = now;
			g_gyro_spin_state.phase = -PI_F * 0.5f;
		}

		float dt_s = TickDiffToSeconds(now, g_gyro_spin_state.last_tick);
		if (dt_s < 0.0f) dt_s = 0.0f;
		if (dt_s > 0.05f) dt_s = 0.05f;

		g_gyro_spin_state.phase += GYRO_SPIN_K * dt_s;
		if (g_gyro_spin_state.phase > PI_F)
		{
			g_gyro_spin_state.phase -= 2.0f * PI_F;
		}

		g_gyro_spin_state.last_tick = now;

		const float center = (GYRO_SPIN_MAX + GYRO_SPIN_MIN) * 0.5f;
		const float amplitude = (GYRO_SPIN_MAX - GYRO_SPIN_MIN) * 0.5f;
		float spin_f = center + amplitude * sinf(g_gyro_spin_state.phase);
		if (spin_f < (float)GYRO_SPIN_MIN) spin_f = (float)GYRO_SPIN_MIN;
		if (spin_f > (float)GYRO_SPIN_MAX) spin_f = (float)GYRO_SPIN_MAX;

		return (int32_t)spin_f;
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

	if (pd_Rx != pdTRUE) return;  // 检查是否成功接收数据
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

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
	has_received_valid_frame = true;


}

void RC::OnRC()
{


	// ================================================================
	// 通用过渡：从任意旋转档（s[0]==DOWN）切出时，底盘仍有惯性
	// 继续调用 Keep_Pantile，直到底盘真正停稳后再重置积分
	// ================================================================
	if (Shift_mode() && pre_rc.s[0] == DOWN && rc.s[0] != DOWN)
	{
		ctrl.chassis.transition_from_rotation = true;
		ctrl.chassis.transition_timer = 0;
		ctrl.pantile.angle2Keep = imu_pantile.GetAngleYaw();
		// 不在此处 ResetIntegral，保留积分补偿底盘惯性
	}

	if (ctrl.chassis.transition_from_rotation)
	{
		ctrl.pantile.Keep_Pantile(ctrl.pantile.angle2Keep, ctrl.pantile.L_YAW, imu_pantile);
		ctrl.chassis.transition_timer++;

		bool chassis_stopped = true;
		for (int i = 0; i < 4; i++) {
			if (ctrl.chassis_motor[i] != nullptr &&
				fabs(ctrl.chassis_motor[i]->curspeed) > 30) {
				chassis_stopped = false;
				break;
			}
		}
		if (chassis_stopped || ctrl.chassis.transition_timer > 500)
		{
			ctrl.chassis.transition_from_rotation = false;
			ctrl.pantile.keep_PID.ResetIntegral();   // 底盘停稳后再清零
			DM_motorYaw.set_yaw_pos = imu_pantile.GetAngleYaw();
		}
	}

	// ================================================================
	// ① RESET (UP-UP) - 安全状态
	// ================================================================
	if (rc.s[0] == UP && rc.s[1] == UP)
	{
		ctrl.mode = CONTROL::RESET;
		judgement.SendSentryPosture(1);
		if (Shift_mode())
		{
			//DM_motorYaw.MotorStart(0x206);
			//DM_motorPitch.MotorStart(0x109);
			DM_motorYaw.DM_pid.ResetIntegral();
			ctrl.pantile.keep_PID.ResetIntegral();
			ctrl.chassis.chassis_reset.ResetIntegral();
			ctrl.pantile.angle2Keep = imu_pantile.GetAngleYaw();  // 切入时锁定世界朝向
			DM_motorPitch.set_pitch_pos = DM_motorPitch.initial_pos_pitch;
		}

		while (!imu_pantile.GetAngleYaw())
		{
			ctrl.pantile.angle2Keep = imu_pantile.GetAngleYaw();
		}

		// 每帧主动稳定大Yaw，抵消底盘旋转（与ROTATION模式相同路径）
		ctrl.pantile.Keep_Pantile(ctrl.pantile.angle2Keep, ctrl.pantile.L_YAW, imu_pantile);
		ctrl.chassis.setChassisspeed(0, 0, 0);
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
	}

 // ================================================================
	// ② 仅动底盘 (UP-MID)
	// 左杆: 底盘XY，云台锁定
	// ================================================================
	else if (rc.s[0] == UP && rc.s[1] == MID)
	{
		//ctrl.mode = CONTROL::FOLLOW;
		ctrl.mode = CONTROL::NORMAL;
		if (Shift_mode())
		{
			ctrl.pantile.keep_PID.ResetIntegral();
			ctrl.pantile.angle2Keep = imu_pantile.GetAngleYaw();
			DM_motorYaw.set_yaw_pos = imu_pantile.GetAngleYaw();
		}
		ctrl.pantile.angle2Keep -= rc.ch[0] * 0.002f;
        ctrl.pantile.angle2Keep = ctrl.GetDelta(ctrl.pantile.angle2Keep);
		ctrl.pantile.Keep_Pantile(ctrl.pantile.angle2Keep, ctrl.pantile.L_YAW, imu_pantile);
		ctrl.chassis.setChassisspeed(rc.ch[3], rc.ch[2], 0);

		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
	}

 // ================================================================
	// ③ 仅动云台 (UP-DOWN)
	// 底盘锁定，仅允许云台控制
	// ================================================================
	else if (rc.s[0] == UP && rc.s[1] == DOWN)
	{
		ctrl.mode = CONTROL::NORMAL;

		if (Shift_mode())
		{
			DM_motorYaw.DM_pid.ResetIntegral();
		}

		ctrl.chassis.setChassisspeed(0, 0, 0);

		// 【修改】大Yaw用度为单位，灵敏度调整
		DM_motorYaw.set_yaw_pos -= rc.ch[0] * 0.001f;  // 度/帧
		DM_motorPitch.set_pitch_pos += rc.ch[3] * (-0.0001f);
		ctrl.pantile.mark_yaw += ctrl.pantile.sensitivity[0] * rc.ch[2] / 660.f * (-1.f);

		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
	}

 // ================================================================
	// ④ autoMove 导航测试 (MID-UP)
	// ================================================================
	else if (rc.s[0] == MID && rc.s[1] == UP)
	{
		ctrl.mode = CONTROL::RC_SHOOT_TEST;
		ctrl.chassis.setChassisspeed(0, 0, 0);
		ctrl.shooter.rc_supply_speed = abs(rc.ch[0]);
		ctrl.pantile.mark_yaw += ctrl.pantile.sensitivity[0] * rc.ch[2] / 660.f * (-1.f);
		DM_motorPitch.set_pitch_pos += rc.ch[3] * (-0.0001f);
		ctrl.shooter.openRub = true;

	}

 // ================================================================
	// ⑤ 平移自瞄 (MID-MID)
	// 底盘可平移 + 自瞄
	// ================================================================
	else if (rc.s[0] == MID && rc.s[1] == MID)
	{

			ctrl.mode = CONTROL::SHOOT;  // 启用自瞄模式

			if (Shift_mode())
			{
				DM_motorYaw.DM_pid.ResetIntegral();
				ctrl.automation.aim_state = CONTROL::AUTOMATION::IDLE;
			}

			ctrl.chassis.setChassisspeed(rc.ch[3], rc.ch[2], 0);

			// 调用自瞄状态机
			ctrl.automation.autoAim();

			// 只在无目标时，允许手动控制小云台
			if (!ctrl.automation.target_detected) {
				ctrl.pantile.mark_yaw += ctrl.pantile.sensitivity[0] * rc.ch[0] / 660.f * (-1.f);
				DM_motorPitch.set_pitch_pos += ctrl.pantile.sensitivity[2] * rc.ch[1] / 660.f * (-1.f);
			}

			ctrl.shooter.openRub = true;
			ctrl.shooter.supply_bullet = ctrl.automation.target_detected && xuc.FireRequested(HAL_GetTick());

	}

 // ================================================================
	// ⑥ AUTO 模式切换挡 (MID-DOWN)
	// Update() 中会在该挡位切到 OnAuto()
	// ================================================================
	else if (rc.s[0] == MID && rc.s[1] == DOWN)
	{
        ctrl.mode = CONTROL::NAVI;

	    ctrl.automation.autoMove();
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
	}

	// ================================================================
	// ⑦ 陀螺待机 (DOWN-UP)
	// 小陀螺 + 手动控制云台
	// ================================================================
	else if (rc.s[0] == DOWN && rc.s[1] == UP)
	{
		ctrl.mode = CONTROL::ROTATION;
		int32_t spin_input = CalcVariableGyroSpin(Shift_mode());

		if (Shift_mode())
		{
			ctrl.pantile.angle2Keep = imu_pantile.GetAngleYaw();
			ctrl.pantile.keep_PID.ResetIntegral();
		}

       // 变速小陀螺：在上下限之间连续变化，变化速度由 k 决定
		ctrl.pantile.Keep_Pantile(ctrl.pantile.angle2Keep, ctrl.pantile.L_YAW, imu_pantile);
		ctrl.chassis.setChassisspeed(rc.ch[2], -rc.ch[3], spin_input);

		// 手动控制：大Yaw通过调整angle2Keep
		ctrl.pantile.angle2Keep -= rc.ch[0] * 0.005f;
     ctrl.pantile.angle2Keep = ctrl.GetDelta(ctrl.pantile.angle2Keep);
		// 小Yaw
		//ctrl.pantile.mark_yaw += ctrl.pantile.sensitivity[0] * rc.ch[2] / 660.f * (-1.f);
		// 小Pitch
		//DM_motorPitch.set_pitch_pos += ctrl.pantile.sensitivity[2] * rc.ch[3] / 660.f * (-1.f);

		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
	}

 // ================================================================
	// ⑧ 小陀螺自瞄 (DOWN-MID)
	// ================================================================
	else if (rc.s[0] == DOWN && rc.s[1] == MID)
	{
		if (Shift_mode())
		{
			ctrl.pantile.angle2Keep = imu_pantile.GetAngleYaw();
			ctrl.pantile.keep_PID.ResetIntegral();
			ctrl.automation.aim_state = CONTROL::AUTOMATION::IDLE;
		}

		if(judgement.data.game_status_t.game_progress == 4)
		{
			ctrl.mode = CONTROL::SHOOT;  // 启用自瞄模式
			judgement.SendSentryPosture(1);

			if (xuc.RxNav.isReached)
			{
				ctrl.chassis.setChassisspeed(0, 0, 660);
				ctrl.automation.autoAim();
				ctrl.shooter.openRub = true;
				ctrl.shooter.supply_bullet = ctrl.automation.target_detected && xuc.FireRequested(HAL_GetTick());

			}
			else
			{

				ctrl.shooter.openRub = false;
				ctrl.shooter.supply_bullet = false;
				ctrl.automation.autoMove();
			}



			// 【关键】Keep_Pantile 稳定大云台，抵消底盘旋转
			ctrl.pantile.Keep_Pantile(ctrl.pantile.angle2Keep, ctrl.pantile.L_YAW, imu_pantile);

		}
		else
		{
			ctrl.mode = CONTROL::RESET;
			judgement.SendSentryPosture(1);
			if (Shift_mode())
			{
				DM_motorYaw.MotorStart(0x206);
				DM_motorPitch.MotorStart(0x109);
				DM_motorYaw.DM_pid.ResetIntegral();
				ctrl.pantile.keep_PID.ResetIntegral();
				ctrl.chassis.chassis_reset.ResetIntegral();
				ctrl.pantile.angle2Keep = imu_pantile.GetAngleYaw();  // 切入时锁定世界朝向
				DM_motorPitch.set_pitch_pos = DM_motorPitch.initial_pos_pitch;
			}

			while (!imu_pantile.GetAngleYaw())
			{
				ctrl.pantile.angle2Keep = imu_pantile.GetAngleYaw();
			}

			// 每帧主动稳定大Yaw，抵消底盘旋转（与ROTATION模式相同路径）
			ctrl.pantile.Keep_Pantile(ctrl.pantile.angle2Keep, ctrl.pantile.L_YAW, imu_pantile);
			ctrl.chassis.setChassisspeed(0, 0, 0);
			ctrl.shooter.openRub = false;
			ctrl.shooter.supply_bullet = false;
		}
	}

 // ================================================================
	// ⑨ 退弹模式 (DOWN-DOWN)
	// 开摩擦轮 + 强制供弹，底盘锁定
	// ================================================================
	else if (rc.s[0] == DOWN && rc.s[1] == DOWN)
	{
      ctrl.mode = CONTROL::TD;

		if (Shift_mode())
		{
			ctrl.pantile.angle2Keep = imu_pantile.GetAngleYaw();
			ctrl.pantile.keep_PID.ResetIntegral();
          DM_motorYaw.DM_pid.ResetIntegral();
		}

		ctrl.pantile.Keep_Pantile(ctrl.pantile.angle2Keep, ctrl.pantile.L_YAW, imu_pantile);
		ctrl.chassis.setChassisspeed(0, 0, 0);
		ctrl.shooter.openRub = true;
       ctrl.shooter.supply_bullet = true;
		//ctrl.shooter.supply_speed = 600;
	}


	// ================================================================
	// 模式切换时重置
	// ================================================================
	if (Shift_mode()) {
		ctrl.automation.scan_finish = false;
		ctrl.automation.aim_state = CONTROL::AUTOMATION::IDLE;
	}

}






void RC::Update()
{

    const uint8_t game_progress = judgement.data.game_status_t.game_progress;
	const bool match_running = (game_progress == 4);

#if ROBOT_CONTROL_MODE == ROBOT_AUTO_MODE
   if (match_running)
	{
		OnAuto();
	}
	else
	{
		OnRC();
	}
#else

		OnRC();

#endif
	//OnPC();
}


void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;

	ctrl.mode = CONTROL::RESET;
	rc.s[0] = UP;
	rc.s[1] = UP;
	pre_rc.s[0] = UP;
	pre_rc.s[1] = UP;

}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}

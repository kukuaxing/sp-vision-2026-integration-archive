#include "label.h"
#include "RC.h"
#include "control.h"

void RC::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // 鎴栬€呮姤閿?
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, m_frame, 0);
	}
	if (pd_Rx == pdTRUE)
	{
		frame_valid = true;
		last_frame_ms = HAL_GetTick();
	}

	if (sizeof(m_frame) < 18) return;
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

	pc.key_h = m_frame[15];//鎸夐敭鐨勯珮浣嶉儴鍒哛 F G Z X C 
	pc.key_l = m_frame[14];//鎸夐敭鐨勪綆8浣?W S A D SHIFT CTRL Q E

	
}

// ===== [后续修改开始] 遥控器拨杆/摇杆到车辆模式的映射 =====
void RC::OnRC()
{
	const int32_t chassis_x = (int32_t)(rc.ch[3] * para.max_speed / 660.f); // 左侧竖直摇杆映射为底盘前后速度
	const int32_t chassis_y = (int32_t)(rc.ch[2] * para.max_speed / 660.f); // 左侧横向摇杆在普通分离模式下映射为左右平移
	const int32_t chassis_z = (int32_t)(rc.ch[0] * para.rota_speed / 660.f); // 右侧横向摇杆在跟随模式下映射为底盘旋转
	const int32_t rotation_speed = (int32_t)(-rc.ch[2] * para.rota_speed / 660.f); // [后续修改] 小陀螺速度：左摇杆左拨顺时针，右拨逆时针
	const int32_t pantile_yaw = -rc.ch[0]; // 实车确认DR16 ch0与直觉操纵相反：右推应使云台右转
	const int32_t pantile_pitch = rc.ch[1]; // 右侧竖直摇杆保留为pitch输入接口
	constexpr float up_mid_gimbal_gain = 1.5f; // UP+MID云台专用灵敏度倍率
	const bool auto_turn_mode = (rc.s[0] == MID && rc.s[1] == UP);
	const bool link_ok = frame_valid && (HAL_GetTick() - last_frame_ms) < RC_LINK_TIMEOUT_MS;
	if (!auto_turn_mode)
	{
		ctrl.chassis.Cancel_AutoTurn360(); // 离开MID+UP时取消360度旋转任务
	}

	if (rc.s[0] == MID && rc.s[1] == MID)
	{
		ctrl.mode = CONTROL::RESET; // MID+MID：复位/安全模式
		ctrl.Control_Chassis(0, 0, 0);
	}
	else if (rc.s[0] == UP && rc.s[1] == MID)
	{
		ctrl.mode = CONTROL::SEPARATE_FREE; // UP+MID：保留分离底盘，右摇杆直接控制云台yaw速度
		ctrl.Control_Chassis(chassis_x, chassis_y, 0); // 分离模式下左摇杆控制底盘前后左右，不主动自转
		ctrl.Control_Pantile(
			static_cast<int32_t>(pantile_yaw * up_mid_gimbal_gain),
			static_cast<int32_t>(pantile_pitch * up_mid_gimbal_gain)); // UP+MID的yaw/pitch输入同时提高1.5倍
	}
	else if (rc.s[0] == UP && rc.s[1] == UP)
	{
		ctrl.mode = CONTROL::ROTATION; // [后续修改] UP+UP：小陀螺且yaw轴锁定地面方向
		ctrl.Control_Chassis(chassis_x, 0, rotation_speed); // 小陀螺模式下左横摇杆控制底盘自转速度
		ctrl.Control_Pantile(0, pantile_pitch); // 锁yaw模式下不接收右横摇杆yaw命令
	}
	else if (rc.s[0] == MID && rc.s[1] == UP)
	{
		ctrl.mode = CONTROL::FOLLOW; // MID+UP：左摇杆触发360度原地旋转，云台yaw保持地面方向
		ctrl.chassis.Control_AutoTurn360(rc.ch[2], chassis_x, link_ok); // 左摇杆左右选旋转方向，空闲时保留左摇杆前后
		ctrl.Control_Pantile(0, 0); // MID+UP下右摇杆的yaw/pitch功能全部禁用
	}
	else if (rc.s[0] == DOWN && rc.s[1] == DOWN)
	{
		ctrl.mode = CONTROL::LOCK; // 锁定模式，底盘停止
		ctrl.Control_Chassis(0, 0, 0);
	}
	else if (rc.s[0] == DOWN && rc.s[1] == UP)
	{
		ctrl.mode = CONTROL::SEPARATE; // DOWN+UP：底盘停止，右摇杆单独控制云台
		ctrl.Control_Chassis(0, 0, 0);
		ctrl.Control_Pantile(pantile_yaw, pantile_pitch); // 右摇杆横向控制yaw，竖向控制pitch
	}
	else if (rc.s[0] == DOWN && rc.s[1] == MID)
	{
		ctrl.mode = CONTROL::SEPARATE; // DOWN+MID：底盘停止，右摇杆单独控制云台
		ctrl.Control_Chassis(0, 0, 0);
		ctrl.Control_Pantile(pantile_yaw, pantile_pitch); // 右摇杆横向控制yaw，竖向控制pitch
	}
	else if (rc.s[0] == MID && rc.s[1] == DOWN)
	{
		ctrl.mode = CONTROL::LOCK; // 锁定模式，底盘停止
		ctrl.Control_Chassis(0, 0, 0);
	}
	else if (rc.s[0] == UP && rc.s[1] == DOWN)
	{
		ctrl.mode = CONTROL::ROTATION_FREE; // [后续修改] UP+DOWN：小陀螺且yaw轴自由速度控制
		ctrl.Control_Chassis(chassis_x, 0, rotation_speed); // 小陀螺模式下左横摇杆控制底盘自转速度
		ctrl.Control_Pantile(pantile_yaw, pantile_pitch); // 分离/自由模式下右摇杆控制云台yaw输入
	}
	else
	{
		ctrl.mode = CONTROL::LOCK; // 锁定模式，底盘停止
		ctrl.Control_Chassis(0, 0, 0);
	}

	Control_Shooter(); // 独立于底盘模式的发射控制
}
// ===== [后续修改结束] 遥控器拨杆/摇杆映射 =====
// ===== [Next] shooter input: DOWN+UP single shot, DOWN+MID continuous shot =====
void RC::Control_Shooter()
{
	// Link-loss safety: stop shooting when no fresh DR16 frame
	const bool link_ok = frame_valid && (HAL_GetTick() - last_frame_ms) < RC_LINK_TIMEOUT_MS;
	if (!link_ok)
	{
		ctrl.shooter.openRub = false;
		ctrl.shooter.supply_bullet = false;
		ctrl.shooter.trigger_now = false;
		ctrl.shooter.trigger_rise = false;
		ctrl.shooter.trigger_prev = true; // suppress accidental edge after link restore
		return;
	}

	// Shooting is enabled only in DOWN+UP and DOWN+MID; DOWN+DOWN is now disabled
	const bool left_down_right_up = (rc.s[0] == DOWN && rc.s[1] == UP);
	const bool left_down_right_mid = (rc.s[0] == DOWN && rc.s[1] == MID);
	const bool shoot_enabled = left_down_right_up || left_down_right_mid;
	ctrl.shooter.openRub = shoot_enabled;

	if (!shoot_enabled)
	{
		ctrl.shooter.supply_bullet = false;
		ctrl.shooter.trigger_now = false;
		ctrl.shooter.trigger_rise = false;
		ctrl.shooter.trigger_prev = true;
		return;
	}

	// DOWN+UP: left stick left/right triggers one shot; continuous feed is disabled
	if (left_down_right_up)
	{
		const int16_t single_stick = rc.ch[2];
		const bool single_active = (fabsf(static_cast<float>(single_stick)) >= 35.f);
		ctrl.shooter.trigger_rise = single_active && !ctrl.shooter.trigger_prev;
		ctrl.shooter.trigger_now = single_active;
		if (ctrl.shooter.trigger_rise)
		{
			ctrl.shooter.single_dir = (single_stick < 0) ? 1 : -1;
		}
		ctrl.shooter.trigger_prev = single_active;
		ctrl.shooter.supply_bullet = false;
		return;
	}

	// DOWN+MID: left stick left/right controls continuous feed; single shot is disabled
	if (left_down_right_mid)
	{
		const int16_t cont_stick = rc.ch[2];
		const bool cont_active = (fabsf(static_cast<float>(cont_stick)) >= 35.f);
		ctrl.shooter.trigger_now = false;
		ctrl.shooter.trigger_rise = false;
		ctrl.shooter.trigger_prev = true;
		ctrl.shooter.supply_bullet = cont_active;
		if (cont_active)
		{
			ctrl.shooter.continuous_dir = (cont_stick < 0) ? 1 : -1;
		}
		return;
	}
}
// ===== [Next] shooter input end =====
void RC::OnPC()
{
	;
}

void RC::Update()
{
	OnRC();
	OnPC();
}


void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

bool RC::Shift_mode()
{
	if (rc.s[0] != pre_rc.s[0] || rc.s[1] != pre_rc.s[1])
	{
		return true;
	}
	return false;
}

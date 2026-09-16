#include "nuc.h"
#include "label.h"
#include "imu.h"
#include "CRC.h"
#include "math.h"
#include "RC.h"
#include "control.h"
#include "supercap.h"
void NUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr).DMATxInit();
	m_uart = huart;
	frame = m_uart->m_uartrx;
	queue_handler = &huart->UartQueueHandler;


}

// Calibration-only, one-way relay of an already validated small-gimbal HI91 frame.
// Preserve wxyz, device timestamp and checksum. Do not synthesize a roll=0 pose.
void NUC::ForwardCalibrationPose(const uint8_t* data, uint16_t size)
{
    if (size != sizeof(calibration_pose_tx) || m_uart == NULL || m_uart->TxBusy()) return;
    memcpy(calibration_pose_tx, data, size);
    m_uart->DMATransmit(calibration_pose_tx, size);
}

void NUC::Decode()
{
	obs.cur_yaw = imu_gimbal.angle.yaw;
	obs.cur_pitch = imu_gimbal.angle.pitch;
	pd_Rx = xQueueReceive((m_uart->UartQueueHandler), m_frame, NULL);
	if (m_frame[0] == 'S' && m_frame[1] == 'P')
	{
		/*if (verifyCRC16CheckSum(m_frame, 31))
		{*/
			xuc.mode_TJ         = m_frame[2];
			xuc.yaw_TJ          = u8_to_float(m_frame + 3);
			xuc.yaw_vel_TJ      = u8_to_float(m_frame + 7);
			xuc.yaw_acc_TJ      = u8_to_float(m_frame + 11);
			xuc.pitch_TJ        = u8_to_float(m_frame + 15);
			xuc.pitch_vel_TJ    = u8_to_float(m_frame + 19);
			xuc.pitch_acc_TJ    = u8_to_float(m_frame + 23);
         xuc.state_switch_TJ = (int8_t)m_frame[27]; // 姿态状态: 3=Move 2=Attack 3=Defense
			xuc.fire_control_TJ = (int8_t)m_frame[28]; // 火控许可: 0/1
			xuc.protocol_version = m_frame[29];
			xuc.goal_id = m_frame[30];
			xuc.tactical_state = m_frame[31];
			xuc.posture = m_frame[32];
			xuc.fire_policy = m_frame[33];
			xuc.spin_mode = m_frame[34];
			xuc.supercap_mode = m_frame[35];
			xuc.rule_action_type = m_frame[36];
			xuc.ammo_exchange_target_total = (uint16_t)(m_frame[37] | ((uint16_t)m_frame[38] << 8));
			xuc.revive_cmd = m_frame[39];
			xuc.remote_ammo_req_inc = m_frame[40];
			xuc.remote_hp_req_inc = m_frame[41];
			xuc.posture_cmd_referee = m_frame[42];
			xuc.activate_energy_confirm = m_frame[43];
			xuc.claim_periodic_ammo = m_frame[44];
			xuc.crc16_TJ = (uint16_t)(m_frame[45] | ((uint16_t)m_frame[46] << 8));

			obs.target_yaw  = radianToDegree(xuc.yaw_TJ);
			
			if (xuc.mode_TJ)
			{
				obs.target_pitch     = radianToDegree(xuc.pitch_TJ);
				obs.target_pitch_pre = obs.target_pitch;
				obs.delta_pitch = obs.target_pitch - obs.cur_pitch;
				obs.delta_yaw = obs.target_yaw - obs.cur_yaw; 

			}
			else
			{
				obs.target_pitch = obs.target_pitch_pre;
			}
		//}
	}
	else if (m_frame[0] == 'N' && m_frame[1] == 'A' && m_frame[2] == 'V' && m_frame[3] == 'I')
	{
		/*if (verifyCRC16CheckSum(frame, 20))
		{ */
		xuc.RxNav.linear_x = u8_to_float(m_frame + 4);
		xuc.RxNav.linear_y = u8_to_float(m_frame + 8);
		xuc.RxNav.angular_z = u8_to_float(m_frame + 12);
		xuc.RxNav.isReached = m_frame[16];
		//}
	}
}



//同济大学（下位机和上位机）
void NUC::Encode()
{
	own_color = judgement.data.robot_status_t.robot_id <= 7 ? RED : BLUE;

	TxNuc_TJ.head[0] = 'S';
	TxNuc_TJ.head[1] = 'P';
    TxNuc_TJ.mode_TJ = 1;

	TxNuc_TJ.yaw_TJ       = imu_gimbal.angle.yaw;
	TxNuc_TJ.yaw_vel_TJ   = imu_gimbal.angularvelocity.yaw;
	TxNuc_TJ.pitch_TJ     = imu_gimbal.angle.pitch;
	TxNuc_TJ.pitch_vel_TJ = imu_gimbal.angularvelocity.pitch;
	TxNuc_TJ.roll_TJ      = imu_gimbal.angle.roll;
	TxNuc_TJ.roll_vel_TJ  = imu_gimbal.angularvelocity.roll;

	TxNuc_TJ.bullet_speed_TJ  = judgement.data.shoot_data_t.bullet_speed;
	TxNuc_TJ.bullet_count_TJ  = judgement.data.projectile_allowance_t.projectile_allowance_17mm;
	TxNuc_TJ.game_status_NAV = (judgement.data.game_status_t.game_progress == 4) ? 1 :
							   (judgement.data.game_status_t.game_progress == 5) ? 2 : 0;

    TxNuc_TJ.chassis_vx = 0.0f;
	TxNuc_TJ.chassis_vy = 0.0f;
	TxNuc_TJ.chassis_wz = 0.0f;
	if (ctrl.chassis_motor[0] && ctrl.chassis_motor[1] &&
		ctrl.chassis_motor[2] && ctrl.chassis_motor[3])
	{
     const float k = ctrl.chassis_motor[0]->const_dx;
		float m0 = (float)ctrl.chassis_motor[0]->curspeed;
		float m1 = (float)ctrl.chassis_motor[1]->curspeed;
		float m2 = (float)ctrl.chassis_motor[2]->curspeed;
		float m3 = (float)ctrl.chassis_motor[3]->curspeed;
		TxNuc_TJ.chassis_vx = ( m0 + m1 + m2 + m3) * k / 4.0f;
		TxNuc_TJ.chassis_vy = (-m0 + m1 + m2 - m3) * k / 4.0f;
     TxNuc_TJ.chassis_wz = (-m0 + m1 - m2 + m3) * k / 4.0f;
	}

	TxNuc_TJ.game_progress     = judgement.data.game_status_t.game_progress;
	TxNuc_TJ.stage_remain_time = judgement.data.game_status_t.stage_remain_time;
  TxNuc_TJ.ally_outpost_hp   = judgement.data.game_robot_HP_t.ally_outpost_HP;
	TxNuc_TJ.event_data        = judgement.data.event_data_t.event_data;

	TxNuc_TJ.robot_id                        = judgement.data.robot_status_t.robot_id;
	TxNuc_TJ.robot_level                     = judgement.data.robot_status_t.robot_level;
	TxNuc_TJ.current_HP                      = judgement.data.robot_status_t.current_HP;
	TxNuc_TJ.maximum_HP                      = judgement.data.robot_status_t.maximum_HP;
	TxNuc_TJ.shooter_barrel_cooling_value    = judgement.data.robot_status_t.shooter_barrel_cooling_value;
	TxNuc_TJ.shooter_barrel_heat_limit       = judgement.data.robot_status_t.shooter_barrel_heat_limit;
	TxNuc_TJ.chassis_power_limit             = judgement.data.robot_status_t.chassis_power_limit;
	TxNuc_TJ.power_management_gimbal_output  = judgement.data.robot_status_t.power_management_gimbal_output;
	TxNuc_TJ.power_management_chassis_output = judgement.data.robot_status_t.power_management_chassis_output;
	TxNuc_TJ.power_management_shooter_output = judgement.data.robot_status_t.power_management_shooter_output;

	TxNuc_TJ.exchanged_projectile_allowance  = judgement.data.sentry_info_t.exchanged_projectile_allowance;
	TxNuc_TJ.remote_exchange_projectile_count = judgement.data.sentry_info_t.remote_exchange_projectile_count;
	TxNuc_TJ.remote_exchange_hp_count        = judgement.data.sentry_info_t.remote_exchange_hp_count;
	TxNuc_TJ.can_confirm_revival             = judgement.data.sentry_info_t.can_confirm_revival;
	TxNuc_TJ.can_buy_instant_revival         = judgement.data.sentry_info_t.can_buy_instant_revival;
	TxNuc_TJ.instant_revival_cost            = judgement.data.sentry_info_t.instant_revival_cost;
	TxNuc_TJ.is_disengaged                   = judgement.data.sentry_info_t.is_disengaged;
	TxNuc_TJ.team_17mm_exchange_remain       = judgement.data.sentry_info_t.team_17mm_exchange_remain;
	TxNuc_TJ.sentry_posture                  = judgement.data.sentry_info_t.sentry_posture;
	TxNuc_TJ.can_activate_energy_mechanism   = judgement.data.sentry_info_t.can_activate_energy_mechanism;

	TxNuc_TJ.recovery_buff      = judgement.data.buff_t.recovery_buff;
	TxNuc_TJ.cooling_buff       = judgement.data.buff_t.cooling_buff;
	TxNuc_TJ.defence_buff       = judgement.data.buff_t.defence_buff;
	TxNuc_TJ.vulnerability_buff = judgement.data.buff_t.vulnerability_buff;
	TxNuc_TJ.attack_buff        = judgement.data.buff_t.attack_buff;
	TxNuc_TJ.remaining_energy   = judgement.data.buff_t.remaining_energy;

	TxNuc_TJ.armor_id            = judgement.data.hurt_data_t.armor_id;
	TxNuc_TJ.HP_deduction_reason = judgement.data.hurt_data_t.HP_deduction_reason;

	TxNuc_TJ.projectile_allowance_17mm     = judgement.data.projectile_allowance_t.projectile_allowance_17mm;
	TxNuc_TJ.projectile_allowance_42mm     = judgement.data.projectile_allowance_t.projectile_allowance_42mm;
	TxNuc_TJ.remaining_gold_coin           = judgement.data.projectile_allowance_t.remaining_gold_coin;
	TxNuc_TJ.projectile_allowance_fortress = judgement.data.projectile_allowance_t.projectile_allowance_fortress;

	TxNuc_TJ.supply_speed = (int16_t)judgement.data.shoot_data_t.bullet_freq;

	TxNuc_TJ.chassis_power = supercap.cap_power;
	TxNuc_TJ.remain_energy = supercap.Rxsuper.cap_energy;
	TxNuc_TJ.buffer_energy = judgement.data.power_heat_data_t.buffer_energy;

	TxNuc_TJ.rfid_status              = judgement.data.rfid_status_t.rfid_status;
	TxNuc_TJ.rfid_status_2            = judgement.data.rfid_status_t.rfid_status_2;
	TxNuc_TJ.shooter_17mm_1_barrel_heat = judgement.data.power_heat_data_t.shooter_17mm_1_barrel_heat;

	TxNuc_TJ.ally_1_robot_HP = judgement.data.game_robot_HP_t.ally_1_robot_HP;
	TxNuc_TJ.ally_2_robot_HP = judgement.data.game_robot_HP_t.ally_2_robot_HP;
	TxNuc_TJ.ally_3_robot_HP = judgement.data.game_robot_HP_t.ally_3_robot_HP;
	TxNuc_TJ.ally_4_robot_HP = judgement.data.game_robot_HP_t.ally_4_robot_HP;
	TxNuc_TJ.reserved        = judgement.data.game_robot_HP_t.reserved;
	TxNuc_TJ.ally_7_robot_HP = judgement.data.game_robot_HP_t.ally_7_robot_HP;
	TxNuc_TJ.ally_outpost_HP = judgement.data.game_robot_HP_t.ally_outpost_HP;
	TxNuc_TJ.ally_base_HP    = judgement.data.game_robot_HP_t.ally_base_HP;

	TxNuc_TJ.crc16_TJ = 0;

	m_uart->UARTTransmit(reinterpret_cast<uint8_t*>(&TxNuc_TJ), sizeof(TxNuc_TJ));
}
	
		
		
		


uint16_t NUC::getCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t ch_data;

	if (pchMessage == nullptr) return 0xFFFF;
	while (dwLength--) {
		ch_data = *pchMessage++;
		(wCRC) =
			((uint16_t)(wCRC) >> 8) ^ CRC_TAB[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
	}

	return wCRC;
}

uint32_t NUC::verifyCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_expected = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return false;

	w_expected = getCRC16CheckSum(pchMessage, dwLength - 2, CRC16_INIT);
	return (
		(w_expected & 0xff) == pchMessage[dwLength - 2] &&
		((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void NUC::appendCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_crc = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return;

	w_crc = getCRC16CheckSum(reinterpret_cast<uint8_t*>(pchMessage), dwLength - 2, CRC16_INIT);

	pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
	pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}
void NUC::send_command(uint8_t command, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3)
{
	// 1. 准备发送帧
	IMU_Send_Frame frame;
	frame.frame_header = 0xAA;
	frame.command = command;
	frame.param0 = param0;
	frame.param1 = param1;
	frame.param2 = param2;
	frame.param3 = param3;
	frame.frame_end = 0x0D;

	//// 2. 计算CRC（不包括CRC和帧尾）
	//constexpr size_t data_size = sizeof(frame) - sizeof(frame.crc16) - sizeof(frame.frame_end);
	//frame.crc16 = getCRC16CheckSum(reinterpret_cast<uint8_t*>(&frame), data_size, CRC16_INIT);

	// 3. 发送完整数据包
	const size_t packet_size = sizeof(frame);
	m_uart->UARTTransmit(reinterpret_cast<uint8_t*>(&frame), packet_size);
}
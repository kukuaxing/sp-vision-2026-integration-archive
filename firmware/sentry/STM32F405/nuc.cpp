#include "nuc.h"

#include "control.h"
#include "imu.h"
#include "judgement.h"
#include "supercap.h"

#include <math.h>
#include <string.h>

namespace
{
constexpr float PI_F = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI_F / 180.0f;
constexpr uint32_t LEGACY_TX_PERIOD_MS = 10U;
constexpr uint32_t FAST_TX_PERIOD_MS = 20U;
constexpr uint32_t STATUS_TX_PERIOD_MS = 100U;
constexpr uint8_t EXTENSION_TYPE_FAST_STATE = 1U;
constexpr uint8_t EXTENSION_TYPE_ROBOT_STATUS = 2U;

bool IsAutoAimModeSelected()
{
	return ctrl.mode == CONTROL::AUTO ||
		ctrl.mode == CONTROL::SHOOT ||
		ctrl.mode == CONTROL::AUTOCONTROL;
}

void GetChassisVelocity(float& vx, float& vy, float& wz)
{
	vx = 0.0f;
	vy = 0.0f;
	wz = 0.0f;
	if (ctrl.chassis_motor[0] == nullptr || ctrl.chassis_motor[1] == nullptr ||
		ctrl.chassis_motor[2] == nullptr || ctrl.chassis_motor[3] == nullptr)
	{
		return;
	}

	const float scale = ctrl.chassis_motor[0]->const_dx;
	const float m0 = static_cast<float>(ctrl.chassis_motor[0]->curspeed);
	const float m1 = static_cast<float>(ctrl.chassis_motor[1]->curspeed);
	const float m2 = static_cast<float>(ctrl.chassis_motor[2]->curspeed);
	const float m3 = static_cast<float>(ctrl.chassis_motor[3]->curspeed);
	vx = (m0 + m1 + m2 + m3) * scale * 0.25f;
	vy = (-m0 + m1 + m2 - m3) * scale * 0.25f;
	// This preserves the existing kinematic convention. Calibrate the lever arm
	// before treating wz as a metrology-grade angular-rate measurement.
	wz = (-m0 + m1 - m2 + m3) * scale * 0.25f;
}

float MotorRpmMagnitude(const Motor* motor)
{
	return motor == nullptr ? 0.0f : fabsf(static_cast<float>(motor->curspeed));
}
}

void NUC::Init(UART* huart, USART_TypeDef* instance, uint32_t baud_rate)
{
	uart_ = huart;
	receive_stream_size_ = 0U;
	have_aim_ = false;
	have_nav_ = false;
	nav_enabled_ = false;
	last_aim_ms_ = 0U;
	last_nav_ms_ = 0U;
	memset(&latest_aim_, 0, sizeof(latest_aim_));
	ForceSafeOutputs();

	if (uart_ != nullptr)
	{
		uart_->Init(instance, baud_rate).DMARxInit(nullptr).DMATxInit();
	}
}

void NUC::Decode()
{
	const uint32_t now_ms = HAL_GetTick();
	obs.cur_yaw = imu_gimbal.angle.yaw;
	obs.cur_pitch = imu_gimbal.angle.pitch;

	if (uart_ != nullptr)
	{
		while (xQueueReceive(uart_->UartQueueHandler, uart_chunk_, 0U) == pdTRUE)
		{
			uint32_t chunk_length = uart_->dataDmaNum;
			if (chunk_length > UART_MAX_LEN)
			{
				chunk_length = UART_MAX_LEN;
				field_errors++;
			}
			if (chunk_length > 0U)
			{
				AppendReceiveBytes(uart_chunk_, static_cast<uint16_t>(chunk_length));
				ParseReceiveStream();
			}
		}
	}

	ApplyTimeouts(now_ms);
}

void NUC::Encode()
{
	if (uart_ == nullptr)
	{
		return;
	}

	const uint32_t now_ms = HAL_GetTick();
	if (static_cast<uint32_t>(now_ms - last_legacy_tx_ms_) >= LEGACY_TX_PERIOD_MS)
	{
		last_legacy_tx_ms_ = now_ms;
		SendLegacyFeedback(now_ms);
	}
	if (static_cast<uint32_t>(now_ms - last_fast_tx_ms_) >= FAST_TX_PERIOD_MS)
	{
		last_fast_tx_ms_ = now_ms;
		SendFastState(now_ms);
	}
	if (static_cast<uint32_t>(now_ms - last_status_tx_ms_) >= STATUS_TX_PERIOD_MS)
	{
		last_status_tx_ms_ = now_ms;
		SendRobotStatus(now_ms);
	}
}

bool NUC::HasFreshAimCommand(uint32_t now_ms) const
{
	return have_aim_ && static_cast<uint32_t>(now_ms - last_aim_ms_) <= NUC_AIM_TIMEOUT_MS;
}

bool NUC::HasFreshNavCommand(uint32_t now_ms) const
{
	return have_nav_ && nav_enabled_ &&
		static_cast<uint32_t>(now_ms - last_nav_ms_) <= nav_timeout_ms_;
}

bool NUC::ControlRequested(uint32_t now_ms) const
{
	return HasFreshAimCommand(now_ms) && latest_aim_.control == 1U;
}

bool NUC::FireRequested(uint32_t now_ms) const
{
	return ControlRequested(now_ms) && latest_aim_.shoot == 1U;
}

void NUC::ForceSafeOutputs()
{
	mode_TJ = 0U;
	fire_control_TJ = 0;
	RxNav.linear_x = 0.0f;
	RxNav.linear_y = 0.0f;
	RxNav.angular_z = 0.0f;
	RxNav.max_accel = 0.0f;
	RxNav.isReached = 0;
}

uint16_t NUC::getCRC16CheckSum(const uint8_t* data, uint32_t length, uint16_t initial) const
{
	if (data == nullptr)
	{
		return 0xFFFFU;
	}

	uint16_t crc = initial;
	while (length-- > 0U)
	{
		crc = static_cast<uint16_t>(crc ^ *data++);
		for (uint8_t bit = 0U; bit < 8U; bit++)
		{
			crc = (crc & 1U) != 0U
				? static_cast<uint16_t>((crc >> 1U) ^ 0x8408U)
				: static_cast<uint16_t>(crc >> 1U);
		}
	}
	return crc;
}

uint32_t NUC::verifyCRC16CheckSum(const uint8_t* packet, uint32_t packet_size) const
{
	if (packet == nullptr || packet_size <= 2U)
	{
		return false;
	}
	const uint16_t expected = getCRC16CheckSum(packet, packet_size - 2U, 0xFFFFU);
	return packet[packet_size - 2U] == static_cast<uint8_t>(expected & 0xFFU) &&
		packet[packet_size - 1U] == static_cast<uint8_t>(expected >> 8U);
}

void NUC::appendCRC16CheckSum(uint8_t* packet, uint32_t packet_size) const
{
	if (packet == nullptr || packet_size <= 2U)
	{
		return;
	}
	const uint16_t crc = getCRC16CheckSum(packet, packet_size - 2U, 0xFFFFU);
	packet[packet_size - 2U] = static_cast<uint8_t>(crc & 0xFFU);
	packet[packet_size - 1U] = static_cast<uint8_t>(crc >> 8U);
}

void NUC::AppendReceiveBytes(const uint8_t* data, uint16_t length)
{
	if (data == nullptr || length == 0U)
	{
		return;
	}

	if (length >= RX_STREAM_CAPACITY)
	{
		data += length - RX_STREAM_CAPACITY;
		length = RX_STREAM_CAPACITY;
		receive_stream_size_ = 0U;
		dropped_bytes++;
	}
	const uint16_t required = static_cast<uint16_t>(receive_stream_size_ + length);
	if (required > RX_STREAM_CAPACITY)
	{
		const uint16_t discard = static_cast<uint16_t>(required - RX_STREAM_CAPACITY);
		ConsumeReceiveBytes(discard);
		dropped_bytes += discard;
	}
	memcpy(receive_stream_ + receive_stream_size_, data, length);
	receive_stream_size_ = static_cast<uint16_t>(receive_stream_size_ + length);
}

void NUC::ParseReceiveStream()
{
	while (receive_stream_size_ > 0U)
	{
		uint16_t packet_size = 0U;
		enum PacketKind { UNKNOWN, AIM, NAV_V3, TACTICAL_V3, NAV_LEGACY } kind = UNKNOWN;

		if (receive_stream_[0] == 'S')
		{
			if (receive_stream_size_ < 2U)
			{
				return;
			}
			if (receive_stream_[1] == 'P')
			{
				kind = AIM;
				packet_size = sizeof(AimCommandV2);
			}
			else if (receive_stream_[1] == 'N')
			{
				kind = NAV_V3;
				packet_size = sizeof(NavigationCommandV3);
			}
			else if (receive_stream_[1] == 'C')
			{
				kind = TACTICAL_V3;
				packet_size = sizeof(TacticalCommandV3);
			}
		}
		else if (receive_stream_[0] == 'N')
		{
			if (receive_stream_size_ < 4U)
			{
				return;
			}
			if (memcmp(receive_stream_, "NAVI", 4U) == 0)
			{
				kind = NAV_LEGACY;
				packet_size = sizeof(LegacyNavigationCommand);
			}
		}

		if (kind == UNKNOWN)
		{
			ConsumeReceiveBytes(1U);
			dropped_bytes++;
			continue;
		}
		if (receive_stream_size_ < packet_size)
		{
			return;
		}
		if ((kind == NAV_V3 || kind == TACTICAL_V3) && receive_stream_[4] != packet_size)
		{
			field_errors++;
			ConsumeReceiveBytes(1U);
			continue;
		}
		if (!verifyCRC16CheckSum(receive_stream_, packet_size))
		{
			crc_errors++;
			ConsumeReceiveBytes(1U);
			continue;
		}

		const uint32_t now_ms = HAL_GetTick();
		if (kind == AIM)
		{
			AimCommandV2 packet{};
			memcpy(&packet, receive_stream_, sizeof(packet));
			if (AimFieldsValid(packet)) AcceptAim(packet, now_ms); else field_errors++;
		}
		else if (kind == NAV_V3)
		{
			NavigationCommandV3 packet{};
			memcpy(&packet, receive_stream_, sizeof(packet));
			if (NavigationFieldsValid(packet)) AcceptNavigation(packet, now_ms); else field_errors++;
		}
		else if (kind == TACTICAL_V3)
		{
			TacticalCommandV3 packet{};
			memcpy(&packet, receive_stream_, sizeof(packet));
			if (packet.version == NUC_PROTOCOL_VERSION && packet.type == 1U)
				AcceptTactical(packet, now_ms);
			else
				field_errors++;
		}
		else
		{
			LegacyNavigationCommand packet{};
			memcpy(&packet, receive_stream_, sizeof(packet));
			if (LegacyNavigationFieldsValid(packet)) AcceptLegacyNavigation(packet, now_ms); else field_errors++;
		}
		ConsumeReceiveBytes(packet_size);
	}
}

void NUC::ConsumeReceiveBytes(uint16_t count)
{
	if (count >= receive_stream_size_)
	{
		receive_stream_size_ = 0U;
		return;
	}
	const uint16_t remaining = static_cast<uint16_t>(receive_stream_size_ - count);
	memmove(receive_stream_, receive_stream_ + count, remaining);
	receive_stream_size_ = remaining;
}

void NUC::ApplyTimeouts(uint32_t now_ms)
{
	if (!HasFreshAimCommand(now_ms))
	{
		have_aim_ = false;
		mode_TJ = 0U;
		fire_control_TJ = 0;
	}
	if (!HasFreshNavCommand(now_ms))
	{
		have_nav_ = false;
		nav_enabled_ = false;
		RxNav.linear_x = 0.0f;
		RxNav.linear_y = 0.0f;
		RxNav.angular_z = 0.0f;
		RxNav.max_accel = 0.0f;
		RxNav.isReached = 0;
	}
}

void NUC::AcceptAim(const AimCommandV2& packet, uint32_t now_ms)
{
	latest_aim_ = packet;
	have_aim_ = true;
	last_aim_ms_ = now_ms;
	mode_TJ = packet.control;
	fire_control_TJ = static_cast<int8_t>(packet.shoot);
	yaw_TJ = packet.yaw_rad;
	pitch_TJ = packet.pitch_rad;
	yaw_vel_TJ = yaw_acc_TJ = pitch_vel_TJ = pitch_acc_TJ = 0.0f;
	obs.target_yaw = packet.yaw_rad / DEG_TO_RAD;
	if (packet.control != 0U)
	{
		obs.target_pitch = packet.pitch_rad / DEG_TO_RAD;
		obs.target_pitch_pre = obs.target_pitch;
		obs.delta_pitch = obs.target_pitch - obs.cur_pitch;
		obs.delta_yaw = obs.target_yaw - obs.cur_yaw;
	}
	valid_aim_packets++;
}

void NUC::AcceptNavigation(const NavigationCommandV3& packet, uint32_t now_ms)
{
	nav_timeout_ms_ = packet.timeout_ms == 0U
		? static_cast<uint16_t>(NUC_NAV_DEFAULT_TIMEOUT_MS)
		: packet.timeout_ms;
	last_nav_ms_ = now_ms;
	have_nav_ = true;
	nav_enabled_ = (packet.flags & 0x01U) != 0U && (packet.flags & 0x02U) == 0U;
	RxNav.linear_x = nav_enabled_ ? packet.linear_x_mps : 0.0f;
	RxNav.linear_y = nav_enabled_ ? packet.linear_y_mps : 0.0f;
	RxNav.angular_z = nav_enabled_ ? packet.angular_z_radps : 0.0f;
	RxNav.max_accel = nav_enabled_ ? packet.max_accel_mps2 : 0.0f;
	RxNav.isReached = nav_enabled_ ? packet.goal_status : 0;
	valid_nav_packets++;
}

void NUC::AcceptLegacyNavigation(const LegacyNavigationCommand& packet, uint32_t now_ms)
{
	last_nav_ms_ = now_ms;
	nav_timeout_ms_ = static_cast<uint16_t>(NUC_NAV_DEFAULT_TIMEOUT_MS);
	have_nav_ = true;
	nav_enabled_ = true;
	RxNav.linear_x = packet.linear_x;
	RxNav.linear_y = packet.linear_y;
	RxNav.angular_z = packet.angular_z;
	RxNav.max_accel = 0.0f;
	RxNav.isReached = packet.isReached;
	valid_nav_packets++;
}

void NUC::AcceptTactical(const TacticalCommandV3& packet, uint32_t)
{
	protocol_version = packet.version;
	state_switch_TJ = static_cast<int8_t>(packet.state_switch);
	goal_id = packet.goal_id;
	tactical_state = packet.tactical_state;
	posture = packet.posture;
	fire_policy = packet.fire_policy;
	spin_mode = packet.spin_mode;
	supercap_mode = packet.supercap_mode;
	rule_action_type = packet.rule_action_type;
	ammo_exchange_target_total = packet.ammo_exchange_target_total;
	revive_cmd = packet.revive_cmd;
	remote_ammo_req_inc = packet.remote_ammo_req_inc;
	remote_hp_req_inc = packet.remote_hp_req_inc;
	posture_cmd_referee = packet.posture_cmd_referee;
	activate_energy_confirm = packet.activate_energy_confirm;
	claim_periodic_ammo = packet.claim_periodic_ammo;
	valid_tactical_packets++;
}

void NUC::SendLegacyFeedback(uint32_t now_ms)
{
	AimFeedbackV2 packet{};
	packet.head[0] = 'S';
	packet.head[1] = 'P';
	const bool auto_selected = IsAutoAimModeSelected();
	const bool imu_valid = IsFiniteFloat(imu_gimbal.angle.pitch) && IsFiniteFloat(imu_gimbal.angle.yaw);
	const bool aim_fresh = HasFreshAimCommand(now_ms);
	const bool control = ControlRequested(now_ms);
	packet.mode = auto_selected ? 1U : 0U;
	packet.gate_bits =
		(auto_selected ? (1U << 0U) : 0U) |
		(auto_selected ? (1U << 1U) : 0U) |
		(auto_selected ? (1U << 2U) : 0U) |
		(imu_valid ? (1U << 3U) : 0U) |
		(aim_fresh ? (1U << 4U) : 0U) |
		(control ? (1U << 5U) : 0U) |
		(auto_selected && control ? (1U << 6U) : 0U) |
		(auto_selected && control ? (1U << 7U) : 0U);
	packet.feeder_rpm = ctrl.supply_motor[0] == nullptr
		? -32768.0f : static_cast<float>(ctrl.supply_motor[0]->curspeed);
	packet.friction_rpm_packed = static_cast<uint16_t>(QuantizeWheelRpm(MotorRpmMagnitude(ctrl.shooter_motor[0]))) |
		static_cast<uint16_t>(static_cast<uint16_t>(QuantizeWheelRpm(MotorRpmMagnitude(ctrl.shooter_motor[1]))) << 8U);
	packet.imu_pitch_rad = imu_valid ? imu_gimbal.angle.pitch * DEG_TO_RAD : 0.0f;
	packet.imu_yaw_rad = imu_valid ? imu_gimbal.angle.yaw * DEG_TO_RAD : 0.0f;
	appendCRC16CheckSum(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
	uart_->UARTTransmit(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
}

void NUC::SendFastState(uint32_t now_ms)
{
	FastStateV3 packet{};
	packet.head[0] = 'S';
	packet.head[1] = 'V';
	packet.version = NUC_PROTOCOL_VERSION;
	packet.type = EXTENSION_TYPE_FAST_STATE;
	packet.length = sizeof(packet);
	packet.flags = (HasFreshAimCommand(now_ms) ? 1U : 0U) |
		(HasFreshNavCommand(now_ms) ? 2U : 0U) |
		(IsAutoAimModeSelected() ? 4U : 0U);
	packet.sequence = fast_sequence_++;
	packet.timestamp_ms = now_ms;
	packet.yaw_rad = imu_gimbal.angle.yaw * DEG_TO_RAD;
	packet.pitch_rad = imu_gimbal.angle.pitch * DEG_TO_RAD;
	packet.roll_rad = imu_gimbal.angle.roll * DEG_TO_RAD;
	packet.yaw_rate_radps = imu_gimbal.angularvelocity.yaw * DEG_TO_RAD;
	packet.pitch_rate_radps = imu_gimbal.angularvelocity.pitch * DEG_TO_RAD;
	packet.roll_rate_radps = imu_gimbal.angularvelocity.roll * DEG_TO_RAD;
	GetChassisVelocity(packet.chassis_vx_mps, packet.chassis_vy_mps, packet.chassis_wz_radps);
	appendCRC16CheckSum(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
	uart_->UARTTransmit(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
}

void NUC::SendRobotStatus(uint32_t now_ms)
{
	RobotStatusV3 packet{};
	packet.head[0] = 'S';
	packet.head[1] = 'V';
	packet.version = NUC_PROTOCOL_VERSION;
	packet.type = EXTENSION_TYPE_ROBOT_STATUS;
	packet.length = sizeof(packet);
	packet.flags = (HasFreshAimCommand(now_ms) ? 1U : 0U) |
		(HasFreshNavCommand(now_ms) ? 2U : 0U) |
		(FireRequested(now_ms) ? 4U : 0U);
	packet.sequence = status_sequence_++;
	packet.timestamp_ms = now_ms;
	packet.robot_id = judgement.data.robot_status_t.robot_id;
	packet.game_progress = judgement.data.game_status_t.game_progress;
	packet.power_outputs =
		(judgement.data.robot_status_t.power_management_gimbal_output ? 1U : 0U) |
		(judgement.data.robot_status_t.power_management_chassis_output ? 2U : 0U) |
		(judgement.data.robot_status_t.power_management_shooter_output ? 4U : 0U);
	packet.robot_level = judgement.data.robot_status_t.robot_level;
	packet.stage_remain_time_s = judgement.data.game_status_t.stage_remain_time;
	packet.current_hp = judgement.data.robot_status_t.current_HP;
	packet.maximum_hp = judgement.data.robot_status_t.maximum_HP;
	packet.barrel_cooling_per_s = judgement.data.robot_status_t.shooter_barrel_cooling_value;
	packet.barrel_heat_limit = judgement.data.robot_status_t.shooter_barrel_heat_limit;
	packet.barrel_heat = judgement.data.power_heat_data_t.shooter_17mm_1_barrel_heat;
	packet.ammo_17mm = judgement.data.projectile_allowance_t.projectile_allowance_17mm;
	packet.buffer_energy_j = judgement.data.power_heat_data_t.buffer_energy;
	packet.referee_bullet_speed_mps = judgement.data.shoot_data_t.bullet_speed;
	packet.feeder_rpm = ctrl.supply_motor[0] == nullptr ? 0.0f : static_cast<float>(ctrl.supply_motor[0]->curspeed);
	packet.friction_left_rpm = MotorRpmMagnitude(ctrl.shooter_motor[0]);
	packet.friction_right_rpm = MotorRpmMagnitude(ctrl.shooter_motor[1]);
	packet.chassis_power_w = supercap.cap_power;
	packet.supercap_remaining = supercap.Rxsuper.cap_energy;
	packet.event_data = judgement.data.event_data_t.event_data;
	packet.rfid_status = judgement.data.rfid_status_t.rfid_status;
	appendCRC16CheckSum(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
	uart_->UARTTransmit(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
}

bool NUC::IsFiniteFloat(float value)
{
	uint32_t bits = 0U;
	memcpy(&bits, &value, sizeof(bits));
	return (bits & 0x7F800000U) != 0x7F800000U;
}

bool NUC::AimFieldsValid(const AimCommandV2& packet)
{
	return packet.head[0] == 'S' && packet.head[1] == 'P' &&
		packet.control <= 1U && packet.shoot <= 1U &&
		IsFiniteFloat(packet.yaw_rad) && IsFiniteFloat(packet.pitch_rad) &&
		fabsf(packet.yaw_rad) <= 2.0f * PI_F && fabsf(packet.pitch_rad) <= PI_F;
}

bool NUC::NavigationFieldsValid(const NavigationCommandV3& packet)
{
	const bool timeout_valid = packet.timeout_ms == 0U ||
		(packet.timeout_ms >= 50U && packet.timeout_ms <= 1000U);
	return packet.version == NUC_PROTOCOL_VERSION && packet.type == 1U &&
		packet.length == sizeof(packet) && timeout_valid &&
		packet.goal_status >= -1 && packet.goal_status <= 1 &&
		IsFiniteFloat(packet.linear_x_mps) && fabsf(packet.linear_x_mps) <= 5.0f &&
		IsFiniteFloat(packet.linear_y_mps) && fabsf(packet.linear_y_mps) <= 5.0f &&
		IsFiniteFloat(packet.angular_z_radps) && fabsf(packet.angular_z_radps) <= 15.0f &&
		IsFiniteFloat(packet.max_accel_mps2) &&
		packet.max_accel_mps2 >= 0.0f && packet.max_accel_mps2 <= 20.0f;
}

bool NUC::LegacyNavigationFieldsValid(const LegacyNavigationCommand& packet)
{
	return memcmp(packet.head, "NAVI", 4U) == 0 &&
		packet.isReached >= -1 && packet.isReached <= 1 &&
		IsFiniteFloat(packet.linear_x) && fabsf(packet.linear_x) <= 5.0f &&
		IsFiniteFloat(packet.linear_y) && fabsf(packet.linear_y) <= 5.0f &&
		IsFiniteFloat(packet.angular_z) && fabsf(packet.angular_z) <= 15.0f;
}

uint8_t NUC::QuantizeWheelRpm(float rpm)
{
	if (!IsFiniteFloat(rpm) || rpm <= 0.0f)
	{
		return 0U;
	}
	if (rpm > 10200.0f)
	{
		rpm = 10200.0f;
	}
	return static_cast<uint8_t>((rpm + 20.0f) / 40.0f);
}

void NUC::send_command(uint8_t command, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3)
{
	if (uart_ == nullptr)
	{
		return;
	}
	IMU_Send_Frame packet{};
	packet.frame_header = 0xAAU;
	packet.command = command;
	packet.param0 = param0;
	packet.param1 = param1;
	packet.param2 = param2;
	packet.param3 = param3;
	packet.frame_end = 0x0DU;
	packet.crc16 = getCRC16CheckSum(reinterpret_cast<const uint8_t*>(&packet), 6U, 0xFFFFU);
	uart_->UARTTransmit(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
}

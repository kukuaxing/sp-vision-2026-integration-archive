#pragma once

#include "stm32f4xx_hal.h"
#include "usart.h"

#include <stdint.h>

// NUC <-> lower-board protocol. Multi-byte fields are little-endian IEEE754.
// The 14/20-byte SP packets remain wire-compatible with the current auto-aim.
static const uint8_t NUC_PROTOCOL_VERSION = 3U;
static const uint32_t NUC_AIM_TIMEOUT_MS = 200U;
static const uint32_t NUC_NAV_DEFAULT_TIMEOUT_MS = 250U;

#pragma pack(push, 1)
struct AimCommandV2                 // NUC -> lower board, 14 bytes
{
	uint8_t head[2];                // "SP"
	uint8_t control;                // 0: no target/control, 1: track target
	uint8_t shoot;                  // 0: inhibit, 1: request continuous fire
	float yaw_rad;                  // absolute yaw target
	float pitch_rad;                // absolute pitch target
	uint16_t crc16;
};

struct AimFeedbackV2                // lower board -> NUC, 20 bytes
{
	uint8_t head[2];                // "SP"
	uint8_t mode;                   // 0: manual/idle, 1: auto aim selected
	uint8_t gate_bits;              // legacy robot_id byte, now diagnostic bits
	float feeder_rpm;               // legacy bullet_speed byte range
	uint16_t friction_rpm_packed;   // each wheel: unsigned RPM / 40
	float imu_pitch_rad;
	float imu_yaw_rad;
	uint16_t crc16;
};

struct NavigationCommandV3          // NUC -> lower board, 34 bytes
{
	uint8_t head[2];                // "SN"
	uint8_t version;                // NUC_PROTOCOL_VERSION
	uint8_t type;                   // 1
	uint8_t length;                 // sizeof(NavigationCommandV3)
	uint8_t flags;                  // bit0 enable, bit1 emergency stop
	uint16_t sequence;
	uint32_t sender_timestamp_ms;
	uint16_t timeout_ms;            // 50..1000, 0 selects default
	int8_t goal_status;             // -1 failed, 0 moving, 1 reached
	uint8_t reserved;
	float linear_x_mps;
	float linear_y_mps;
	float angular_z_radps;
	float max_accel_mps2;
	uint16_t crc16;
};

struct TacticalCommandV3            // NUC -> lower board, 30 bytes
{
	uint8_t head[2];                // "SC"
	uint8_t version;
	uint8_t type;                   // 1
	uint8_t length;
	uint8_t flags;
	uint16_t sequence;
	uint32_t sender_timestamp_ms;
	uint8_t state_switch;
	uint8_t goal_id;
	uint8_t tactical_state;
	uint8_t posture;
	uint8_t fire_policy;
	uint8_t spin_mode;
	uint8_t supercap_mode;
	uint8_t rule_action_type;
	uint16_t ammo_exchange_target_total;
	uint8_t revive_cmd;
	uint8_t remote_ammo_req_inc;
	uint8_t remote_hp_req_inc;
	uint8_t posture_cmd_referee;
	uint8_t activate_energy_confirm;
	uint8_t claim_periodic_ammo;
	uint16_t crc16;
};

struct FastStateV3                  // lower board -> NUC, 50 bytes, 50 Hz
{
	uint8_t head[2];                // "SV"
	uint8_t version;
	uint8_t type;                   // 1
	uint8_t length;
	uint8_t flags;
	uint16_t sequence;
	uint32_t timestamp_ms;
	float yaw_rad;
	float pitch_rad;
	float roll_rad;
	float yaw_rate_radps;
	float pitch_rate_radps;
	float roll_rate_radps;
	float chassis_vx_mps;
	float chassis_vy_mps;
	float chassis_wz_radps;
	uint16_t crc16;
};

struct RobotStatusV3                // lower board -> NUC, 66 bytes, 10 Hz
{
	uint8_t head[2];                // "SV"
	uint8_t version;
	uint8_t type;                   // 2
	uint8_t length;
	uint8_t flags;
	uint16_t sequence;
	uint32_t timestamp_ms;
	uint8_t robot_id;
	uint8_t game_progress;
	uint8_t power_outputs;          // bit0 gimbal, bit1 chassis, bit2 shooter
	uint8_t robot_level;
	uint16_t stage_remain_time_s;
	uint16_t current_hp;
	uint16_t maximum_hp;
	uint16_t barrel_cooling_per_s;
	uint16_t barrel_heat_limit;
	uint16_t barrel_heat;
	uint16_t ammo_17mm;
	uint16_t buffer_energy_j;
	float referee_bullet_speed_mps;
	float feeder_rpm;
	float friction_left_rpm;
	float friction_right_rpm;
	float chassis_power_w;
	float supercap_remaining;
	uint32_t event_data;
	uint32_t rfid_status;
	uint16_t crc16;
};

// Accepted only with a valid CRC; retained for older navigation programs.
struct LegacyNavigationCommand
{
	uint8_t head[4];                // "NAVI"
	float linear_x;
	float linear_y;
	float angular_z;
	int8_t isReached;
	uint16_t crc16;
};

struct IMU_Send_Frame
{
	uint8_t frame_header;
	uint8_t command;
	uint8_t param0;
	uint8_t param1;
	uint8_t param2;
	uint8_t param3;
	uint16_t crc16;
	uint8_t frame_end;
};
#pragma pack(pop)

static_assert(sizeof(AimCommandV2) == 14U, "AimCommandV2 size changed");
static_assert(sizeof(AimFeedbackV2) == 20U, "AimFeedbackV2 size changed");
static_assert(sizeof(NavigationCommandV3) == 34U, "NavigationCommandV3 size changed");
static_assert(sizeof(TacticalCommandV3) == 30U, "TacticalCommandV3 size changed");
static_assert(sizeof(FastStateV3) == 50U, "FastStateV3 size changed");
static_assert(sizeof(RobotStatusV3) == 66U, "RobotStatusV3 size changed");
static_assert(sizeof(LegacyNavigationCommand) == 19U, "LegacyNavigationCommand size changed");

struct NavToNucFrame
{
	float linear_x = 0.0f;
	float linear_y = 0.0f;
	float angular_z = 0.0f;
	float max_accel = 0.0f;
	int8_t isReached = 0;
};

struct OBS
{
	float target_yaw = 0.0f;
	float cur_yaw = 0.0f;
	float target_pitch = 0.0f;
	float cur_pitch = 0.0f;
	float target_pitch_pre = 0.0f;
	float delta_yaw = 0.0f;
	float delta_pitch = 0.0f;
};

class NUC
{
public:
	void Init(UART* huart, USART_TypeDef* instance, uint32_t baud_rate);
	void Decode();
	void Encode();

	bool HasFreshAimCommand(uint32_t now_ms) const;
	bool HasFreshNavCommand(uint32_t now_ms) const;
	bool ControlRequested(uint32_t now_ms) const;
	bool FireRequested(uint32_t now_ms) const;
	void ForceSafeOutputs();

	uint16_t getCRC16CheckSum(const uint8_t* data, uint32_t length, uint16_t initial = 0xFFFFU) const;
	uint32_t verifyCRC16CheckSum(const uint8_t* packet, uint32_t packet_size) const;
	void appendCRC16CheckSum(uint8_t* packet, uint32_t packet_size) const;
	void send_command(uint8_t command, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3);

	// Existing control code consumes these compatibility fields.
	uint8_t mode_TJ = 0U;
	float yaw_TJ = 0.0f;
	float yaw_vel_TJ = 0.0f;
	float yaw_acc_TJ = 0.0f;
	float pitch_TJ = 0.0f;
	float pitch_vel_TJ = 0.0f;
	float pitch_acc_TJ = 0.0f;
	int8_t state_switch_TJ = 0;
	int8_t fire_control_TJ = 0;
	uint8_t protocol_version = NUC_PROTOCOL_VERSION;
	uint8_t goal_id = 0U;
	uint8_t tactical_state = 0U;
	uint8_t posture = 0U;
	uint8_t fire_policy = 0U;
	uint8_t spin_mode = 0U;
	uint8_t supercap_mode = 0U;
	uint8_t rule_action_type = 0U;
	uint16_t ammo_exchange_target_total = 0U;
	uint8_t revive_cmd = 0U;
	uint8_t remote_ammo_req_inc = 0U;
	uint8_t remote_hp_req_inc = 0U;
	uint8_t posture_cmd_referee = 0U;
	uint8_t activate_energy_confirm = 0U;
	uint8_t claim_periodic_ammo = 0U;
	NavToNucFrame RxNav;
	OBS obs;

	uint32_t valid_aim_packets = 0U;
	uint32_t valid_nav_packets = 0U;
	uint32_t valid_tactical_packets = 0U;
	uint32_t crc_errors = 0U;
	uint32_t field_errors = 0U;
	uint32_t dropped_bytes = 0U;

private:
	static const uint16_t RX_STREAM_CAPACITY = 256U;

	void AppendReceiveBytes(const uint8_t* data, uint16_t length);
	void ParseReceiveStream();
	void ConsumeReceiveBytes(uint16_t count);
	void ApplyTimeouts(uint32_t now_ms);
	void AcceptAim(const AimCommandV2& packet, uint32_t now_ms);
	void AcceptNavigation(const NavigationCommandV3& packet, uint32_t now_ms);
	void AcceptLegacyNavigation(const LegacyNavigationCommand& packet, uint32_t now_ms);
	void AcceptTactical(const TacticalCommandV3& packet, uint32_t now_ms);
	void SendLegacyFeedback(uint32_t now_ms);
	void SendFastState(uint32_t now_ms);
	void SendRobotStatus(uint32_t now_ms);

	static bool IsFiniteFloat(float value);
	static bool AimFieldsValid(const AimCommandV2& packet);
	static bool NavigationFieldsValid(const NavigationCommandV3& packet);
	static bool LegacyNavigationFieldsValid(const LegacyNavigationCommand& packet);
	static uint8_t QuantizeWheelRpm(float rpm);

	UART* uart_ = nullptr;
	AimCommandV2 latest_aim_{};
	bool have_aim_ = false;
	bool have_nav_ = false;
	bool nav_enabled_ = false;
	uint32_t last_aim_ms_ = 0U;
	uint32_t last_nav_ms_ = 0U;
	uint16_t nav_timeout_ms_ = static_cast<uint16_t>(NUC_NAV_DEFAULT_TIMEOUT_MS);
	uint32_t last_legacy_tx_ms_ = 0U;
	uint32_t last_fast_tx_ms_ = 0U;
	uint32_t last_status_tx_ms_ = 0U;
	uint16_t fast_sequence_ = 0U;
	uint16_t status_sequence_ = 0U;
	uint8_t uart_chunk_[UART_MAX_LEN]{};
	uint8_t receive_stream_[RX_STREAM_CAPACITY]{};
	uint16_t receive_stream_size_ = 0U;
};

extern NUC xuc;

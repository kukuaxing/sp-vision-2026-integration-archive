#pragma once

#include <stdint.h>

struct XucYawControlInput
{
	uint32_t now_ms;
	bool rc_link_valid;
	bool arm_switch_selected;
	bool sticks_centered;
	bool imu_valid;
	bool command_fresh;
	uint8_t command_control;
	float current_yaw_deg;
	float command_yaw_rad;
};

struct XucYawControlOutput
{
	bool armed;
	bool just_armed;
	float target_yaw_deg;
};

// Diagnostic-only codes reported in TxPacket_TJ::bullet_count during Phase 2D.5.
// They do not alter any arming gate or control output.
enum XucYawDisarmReason : uint16_t
{
	XUC_YAW_DISARM_NONE = 0U,
	XUC_YAW_DISARM_SWITCH_RELEASED = 1U,
	XUC_YAW_DISARM_RC_LINK = 2U,
	XUC_YAW_DISARM_STICKS_NOT_CENTERED = 3U,
	XUC_YAW_DISARM_IMU_STALE = 4U,
	XUC_YAW_DISARM_COMMAND_STALE = 5U,
	XUC_YAW_DISARM_CURRENT_YAW_INVALID = 6U,
	XUC_YAW_DISARM_COMMAND_YAW_INVALID = 7U,
	XUC_YAW_DISARM_ACTUAL_EXCURSION = 8U,
	// Code 9 is a recoverable visual hold, not a hard-gate disarm.
	XUC_YAW_DISARM_CONTROL_DROPOUT = 9U,
	XUC_YAW_DISARM_ARM_CONTROL_DISABLED = 10U
};

// Phase 2D.5 keeps the hard motion gates while making mode selection level-based:
// - the AUTO switch position arms as soon as all live inputs are valid;
// - RC, IMU and serial freshness failures still stop motion immediately and
//   recover automatically when the inputs become valid again;
// - short vision-control dropouts hold the last bounded target;
// - sustained vision-control dropouts stop at measured yaw and recover in place;
// - the total target stays within +/-25 degrees of the arm position;
// - actual IMU excursion beyond 28 degrees disarms and requires leaving AUTO;
// - valid bounded target angles pass directly to the existing gimbal angle loop.
class XucYawController
{
public:
	XucYawControlOutput Update(const XucYawControlInput& input);
	bool Armed() const { return armed_; }
	float ArmOriginYawDeg() const { return arm_origin_yaw_deg_; }
	uint16_t LastDisarmReasonCode() const
	{
		return static_cast<uint16_t>(last_disarm_reason_);
	}

	static float NormalizeDegrees(float angle_deg);
	static float ShortestDeltaDegrees(float target_deg, float current_deg);
	static float ProtocolYawToLowerImuYawRadians(float protocol_yaw_rad)
	{
		return protocol_yaw_rad;
	}

private:
	void Disarm(float current_yaw_deg, uint32_t now_ms,
		XucYawDisarmReason reason = XUC_YAW_DISARM_NONE);
	XucYawDisarmReason FirstHardInputFailure(const XucYawControlInput& input) const;
	static bool IsFiniteFloat(float value);
	static float Clamp(float value, float lower, float upper);

	bool armed_ = false;
	// Only a real-yaw excursion is latched. Clearing that physical safety fault
	// still requires leaving AUTO; ordinary input recovery needs no switch edge.
	bool hard_fault_latched_ = false;
	float arm_origin_yaw_deg_ = 0.0f;
	float target_yaw_deg_ = 0.0f;
	uint32_t last_update_ms_ = 0U;
	uint32_t last_control_true_ms_ = 0U;
	XucYawDisarmReason last_disarm_reason_ = XUC_YAW_DISARM_NONE;
};

extern XucYawController xuc_yaw_controller;

struct XucPitchControlInput
{
	uint32_t now_ms;
	bool motion_authorized;
	bool arm_switch_selected;
	uint8_t command_control;
	float current_pitch_deg;
	float current_motor_pos_rad;
	float command_pitch_rad;
};

struct XucPitchControlOutput
{
	bool active;
	bool just_armed;
	float target_pitch_deg;
	float target_motor_pos_rad;
};

enum XucPitchDisarmReason : uint16_t
{
	XUC_PITCH_DISARM_NONE = 0U,
	XUC_PITCH_DISARM_CURRENT_PITCH_INVALID = 11U,
	XUC_PITCH_DISARM_COMMAND_PITCH_INVALID = 12U,
	XUC_PITCH_DISARM_MOTOR_FEEDBACK_INVALID = 13U,
	XUC_PITCH_DISARM_ACTUAL_EXCURSION = 14U,
	XUC_PITCH_DISARM_CONTROL_DROPOUT = 15U
};

// Phase 2D.5 pitch acceptance controller.  It is subordinate to the complete
// yaw/RC/IMU/serial safety gate and maps a bounded absolute IMU pitch target to
// the existing DM position-velocity loop using anchors captured at AUTO entry.
// The moving-target profile permits +/-15 degrees at a 65 deg/s target slew,
// while an actual excursion beyond 18 degrees hard-latches motion off.
class XucPitchController
{
public:
	XucPitchControlOutput Update(const XucPitchControlInput& input);
	uint16_t LastDisarmReasonCode() const
	{
		return static_cast<uint16_t>(last_disarm_reason_);
	}

private:
	void ResetToFeedback(const XucPitchControlInput& input);
	static bool IsFiniteFloat(float value);
	static float Clamp(float value, float lower, float upper);

	bool active_ = false;
	bool hard_fault_latched_ = false;
	float arm_origin_pitch_deg_ = 0.0f;
	float arm_origin_motor_pos_rad_ = 0.0f;
	float target_pitch_deg_ = 0.0f;
	float target_motor_pos_rad_ = 0.0f;
	uint32_t last_update_ms_ = 0U;
	uint32_t last_control_true_ms_ = 0U;
	XucPitchDisarmReason last_disarm_reason_ = XUC_PITCH_DISARM_NONE;
};

extern XucPitchController xuc_pitch_controller;

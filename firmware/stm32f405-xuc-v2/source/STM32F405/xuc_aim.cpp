#include "xuc_aim.h"

#include <math.h>
#include <string.h>

namespace
{
const float RAD_TO_DEG = 57.29577951308232f;
const float MAX_TOTAL_EXCURSION_DEG = 25.0f;
const float MAX_ACTUAL_EXCURSION_DEG = 28.0f;
// The 8.4 ms camera setup keeps a tracked target through isolated misses.  A
// shorter hold limits how long the gimbal can continue toward a stale target
// after the upper controller deliberately drops control near image centre.
const uint32_t CONTROL_DROPOUT_HOLD_MS = 200U;
const float MAX_IMU_MAGNITUDE_DEG = 720.0f;
const float MAX_COMMAND_MAGNITUDE_RAD = 6.283185307179586f;
const float DEG_TO_RAD = 0.017453292519943295f;
const float MAX_PITCH_TOTAL_EXCURSION_DEG = 15.0f;
const float MAX_PITCH_ACTUAL_EXCURSION_DEG = 18.0f;
const float MAX_PITCH_MAGNITUDE_DEG = 180.0f;
// Physical acceptance on 2026-09-04 showed that decreasing DM position makes
// the strict IMU pitch decrease.  A positive absolute pitch request therefore
// needs a positive DM position delta.
const float PITCH_MOTOR_DIRECTION = 1.0f;
const float PITCH_MOTOR_MIN_RAD = -0.25f;
const float PITCH_MOTOR_MAX_RAD = 0.4f;
const float PITCH_MOTOR_FEEDBACK_MARGIN_RAD = 0.1f;
const float PITCH_TARGET_SLEW_DEG_PER_SECOND = 80.0f;
}

XucYawController xuc_yaw_controller;
XucPitchController xuc_pitch_controller;

XucYawControlOutput XucYawController::Update(const XucYawControlInput& input)
{
	if (!input.arm_switch_selected)
	{
		const XucYawDisarmReason reason =
			armed_ ? XUC_YAW_DISARM_SWITCH_RELEASED : XUC_YAW_DISARM_NONE;
		Disarm(input.current_yaw_deg, input.now_ms, reason);
		hard_fault_latched_ = false;
		return { false, false, target_yaw_deg_ };
	}

	const XucYawDisarmReason hard_input_failure = FirstHardInputFailure(input);
	const bool hard_inputs_valid = hard_input_failure == XUC_YAW_DISARM_NONE;

	if (!armed_)
	{
		if (hard_fault_latched_ || !hard_inputs_valid || input.command_control != 1U)
		{
			if (!hard_fault_latched_)
			{
				last_disarm_reason_ = hard_inputs_valid
					? XUC_YAW_DISARM_ARM_CONTROL_DISABLED
					: hard_input_failure;
			}
			Disarm(input.current_yaw_deg, input.now_ms);
			return { false, false, target_yaw_deg_ };
		}

		armed_ = true;
		arm_origin_yaw_deg_ = NormalizeDegrees(input.current_yaw_deg);
		target_yaw_deg_ = arm_origin_yaw_deg_;
		last_update_ms_ = input.now_ms;
		last_control_true_ms_ = input.now_ms;
		last_disarm_reason_ = XUC_YAW_DISARM_NONE;
		return { true, true, target_yaw_deg_ };
	}

	if (!hard_inputs_valid)
	{
		Disarm(input.current_yaw_deg, input.now_ms, hard_input_failure);
		return { false, false, target_yaw_deg_ };
	}

	// 目标限幅不能阻止接线、符号或电机方向错误造成的反向失控。
	// 独立检查真实IMU姿态；超过目标边界3度即锁存退出，必须重新拨杆解锁。
	if (fabsf(ShortestDeltaDegrees(
		input.current_yaw_deg, arm_origin_yaw_deg_)) > MAX_ACTUAL_EXCURSION_DEG)
	{
		hard_fault_latched_ = true;
		Disarm(input.current_yaw_deg, input.now_ms,
			XUC_YAW_DISARM_ACTUAL_EXCURSION);
		return { false, false, target_yaw_deg_ };
	}

	if (input.command_control != 1U)
	{
		// A detector can miss a few frames while the tracker is otherwise healthy.
		// Hold the last bounded target briefly. On sustained visual loss, stop at
		// measured yaw instead of latching AUTO off or chasing a stale target.
		// Hard RC, IMU and serial gates above still disarm immediately.
		if (static_cast<uint32_t>(input.now_ms - last_control_true_ms_) >
			CONTROL_DROPOUT_HOLD_MS)
		{
			target_yaw_deg_ = NormalizeDegrees(input.current_yaw_deg);
			last_update_ms_ = input.now_ms;
			last_disarm_reason_ = XUC_YAW_DISARM_CONTROL_DROPOUT;
			return { true, false, target_yaw_deg_ };
		}
		last_update_ms_ = input.now_ms;
		return { true, false, target_yaw_deg_ };
	}
	last_control_true_ms_ = input.now_ms;
	if (last_disarm_reason_ == XUC_YAW_DISARM_CONTROL_DROPOUT)
	{
		last_disarm_reason_ = XUC_YAW_DISARM_NONE;
	}

	last_update_ms_ = input.now_ms;

	const float command_yaw_deg = NormalizeDegrees(input.command_yaw_rad * RAD_TO_DEG);
	const float command_from_origin = Clamp(
		ShortestDeltaDegrees(command_yaw_deg, arm_origin_yaw_deg_),
		-MAX_TOTAL_EXCURSION_DEG,
		MAX_TOTAL_EXCURSION_DEG);
	// The upper computer owns target estimation and filtering.  The lower board
	// only bounds the absolute target, then hands it to the existing IMU angle
	// loop; a second trajectory generator here causes visible step-and-hold motion.
	target_yaw_deg_ = NormalizeDegrees(arm_origin_yaw_deg_ + command_from_origin);

	return { true, false, target_yaw_deg_ };
}

float XucYawController::NormalizeDegrees(float angle_deg)
{
	while (angle_deg > 180.0f)
	{
		angle_deg -= 360.0f;
	}
	while (angle_deg <= -180.0f)
	{
		angle_deg += 360.0f;
	}
	return angle_deg;
}

float XucYawController::ShortestDeltaDegrees(float target_deg, float current_deg)
{
	return NormalizeDegrees(target_deg - current_deg);
}

void XucYawController::Disarm(float current_yaw_deg, uint32_t now_ms,
	XucYawDisarmReason reason)
{
	armed_ = false;
	if (reason != XUC_YAW_DISARM_NONE)
	{
		last_disarm_reason_ = reason;
	}
	if (IsFiniteFloat(current_yaw_deg))
	{
		arm_origin_yaw_deg_ = NormalizeDegrees(current_yaw_deg);
		target_yaw_deg_ = arm_origin_yaw_deg_;
	}
	last_update_ms_ = now_ms;
}

XucYawDisarmReason XucYawController::FirstHardInputFailure(
	const XucYawControlInput& input) const
{
	if (!input.rc_link_valid)
	{
		return XUC_YAW_DISARM_RC_LINK;
	}
	if (!input.sticks_centered)
	{
		return XUC_YAW_DISARM_STICKS_NOT_CENTERED;
	}
	if (!input.imu_valid)
	{
		return XUC_YAW_DISARM_IMU_STALE;
	}
	if (!input.command_fresh)
	{
		return XUC_YAW_DISARM_COMMAND_STALE;
	}
	if (!IsFiniteFloat(input.current_yaw_deg) ||
		input.current_yaw_deg < -MAX_IMU_MAGNITUDE_DEG ||
		input.current_yaw_deg > MAX_IMU_MAGNITUDE_DEG)
	{
		return XUC_YAW_DISARM_CURRENT_YAW_INVALID;
	}
	if (!IsFiniteFloat(input.command_yaw_rad) ||
		input.command_yaw_rad < -MAX_COMMAND_MAGNITUDE_RAD ||
		input.command_yaw_rad > MAX_COMMAND_MAGNITUDE_RAD)
	{
		return XUC_YAW_DISARM_COMMAND_YAW_INVALID;
	}
	return XUC_YAW_DISARM_NONE;
}

bool XucYawController::IsFiniteFloat(float value)
{
	uint32_t bits = 0U;
	memcpy(&bits, &value, sizeof(bits));
	return (bits & 0x7F800000U) != 0x7F800000U;
}

float XucYawController::Clamp(float value, float lower, float upper)
{
	if (value < lower)
	{
		return lower;
	}
	if (value > upper)
	{
		return upper;
	}
	return value;
}

XucPitchControlOutput XucPitchController::Update(const XucPitchControlInput& input)
{
	if (!input.arm_switch_selected)
	{
		hard_fault_latched_ = false;
		last_disarm_reason_ = XUC_PITCH_DISARM_NONE;
		ResetToFeedback(input);
		return { false, false, target_pitch_deg_, target_motor_pos_rad_ };
	}

	if (!input.motion_authorized || hard_fault_latched_)
	{
		ResetToFeedback(input);
		return { false, false, target_pitch_deg_, target_motor_pos_rad_ };
	}

	if (!IsFiniteFloat(input.current_pitch_deg) ||
		input.current_pitch_deg < -MAX_PITCH_MAGNITUDE_DEG ||
		input.current_pitch_deg > MAX_PITCH_MAGNITUDE_DEG)
	{
		last_disarm_reason_ = XUC_PITCH_DISARM_CURRENT_PITCH_INVALID;
		ResetToFeedback(input);
		return { false, false, target_pitch_deg_, target_motor_pos_rad_ };
	}
	if (!IsFiniteFloat(input.command_pitch_rad) ||
		input.command_pitch_rad < -MAX_COMMAND_MAGNITUDE_RAD ||
		input.command_pitch_rad > MAX_COMMAND_MAGNITUDE_RAD)
	{
		last_disarm_reason_ = XUC_PITCH_DISARM_COMMAND_PITCH_INVALID;
		ResetToFeedback(input);
		return { false, false, target_pitch_deg_, target_motor_pos_rad_ };
	}
	if (!IsFiniteFloat(input.current_motor_pos_rad) ||
		input.current_motor_pos_rad < PITCH_MOTOR_MIN_RAD - PITCH_MOTOR_FEEDBACK_MARGIN_RAD ||
		input.current_motor_pos_rad > PITCH_MOTOR_MAX_RAD + PITCH_MOTOR_FEEDBACK_MARGIN_RAD)
	{
		last_disarm_reason_ = XUC_PITCH_DISARM_MOTOR_FEEDBACK_INVALID;
		active_ = false;
		return { false, false, target_pitch_deg_, target_motor_pos_rad_ };
	}

	if (!active_)
	{
		active_ = true;
		arm_origin_pitch_deg_ = input.current_pitch_deg;
		arm_origin_motor_pos_rad_ = input.current_motor_pos_rad;
		target_pitch_deg_ = arm_origin_pitch_deg_;
		target_motor_pos_rad_ = Clamp(
			arm_origin_motor_pos_rad_, PITCH_MOTOR_MIN_RAD, PITCH_MOTOR_MAX_RAD);
		last_update_ms_ = input.now_ms;
		last_control_true_ms_ = input.now_ms;
		last_disarm_reason_ = XUC_PITCH_DISARM_NONE;
		return { true, true, target_pitch_deg_, target_motor_pos_rad_ };
	}

	if (fabsf(input.current_pitch_deg - arm_origin_pitch_deg_) >
		MAX_PITCH_ACTUAL_EXCURSION_DEG)
	{
		hard_fault_latched_ = true;
		last_disarm_reason_ = XUC_PITCH_DISARM_ACTUAL_EXCURSION;
		ResetToFeedback(input);
		return { false, false, target_pitch_deg_, target_motor_pos_rad_ };
	}

	if (input.command_control != 1U)
	{
		if (static_cast<uint32_t>(input.now_ms - last_control_true_ms_) >
			CONTROL_DROPOUT_HOLD_MS)
		{
			target_pitch_deg_ = input.current_pitch_deg;
			target_motor_pos_rad_ = Clamp(
				input.current_motor_pos_rad, PITCH_MOTOR_MIN_RAD, PITCH_MOTOR_MAX_RAD);
			last_disarm_reason_ = XUC_PITCH_DISARM_CONTROL_DROPOUT;
		}
		last_update_ms_ = input.now_ms;
		return { true, false, target_pitch_deg_, target_motor_pos_rad_ };
	}
	last_control_true_ms_ = input.now_ms;
	if (last_disarm_reason_ == XUC_PITCH_DISARM_CONTROL_DROPOUT)
	{
		last_disarm_reason_ = XUC_PITCH_DISARM_NONE;
	}

	const float requested_pitch_deg = input.command_pitch_rad * RAD_TO_DEG;
	const float bounded_delta_deg = Clamp(
		requested_pitch_deg - arm_origin_pitch_deg_,
		-MAX_PITCH_TOTAL_EXCURSION_DEG,
		MAX_PITCH_TOTAL_EXCURSION_DEG);
	const float desired_pitch_deg = arm_origin_pitch_deg_ + bounded_delta_deg;
	const float desired_motor_pos_rad = Clamp(
		arm_origin_motor_pos_rad_ +
			PITCH_MOTOR_DIRECTION * bounded_delta_deg * DEG_TO_RAD,
		PITCH_MOTOR_MIN_RAD,
		PITCH_MOTOR_MAX_RAD);

	uint32_t dt_ms = static_cast<uint32_t>(input.now_ms - last_update_ms_);
	if (dt_ms > 50U)
	{
		dt_ms = 50U;
	}
	const float max_step_rad =
		PITCH_TARGET_SLEW_DEG_PER_SECOND * DEG_TO_RAD *
		static_cast<float>(dt_ms) / 1000.0f;
	target_motor_pos_rad_ += Clamp(
		desired_motor_pos_rad - target_motor_pos_rad_, -max_step_rad, max_step_rad);
	target_motor_pos_rad_ = Clamp(
		target_motor_pos_rad_, PITCH_MOTOR_MIN_RAD, PITCH_MOTOR_MAX_RAD);
	target_pitch_deg_ = desired_pitch_deg;
	last_update_ms_ = input.now_ms;

	return { true, false, target_pitch_deg_, target_motor_pos_rad_ };
}

void XucPitchController::ResetToFeedback(const XucPitchControlInput& input)
{
	active_ = false;
	if (IsFiniteFloat(input.current_pitch_deg))
	{
		target_pitch_deg_ = input.current_pitch_deg;
	}
	if (IsFiniteFloat(input.current_motor_pos_rad) &&
		input.current_motor_pos_rad >= PITCH_MOTOR_MIN_RAD - PITCH_MOTOR_FEEDBACK_MARGIN_RAD &&
		input.current_motor_pos_rad <= PITCH_MOTOR_MAX_RAD + PITCH_MOTOR_FEEDBACK_MARGIN_RAD)
	{
		target_motor_pos_rad_ = Clamp(
			input.current_motor_pos_rad, PITCH_MOTOR_MIN_RAD, PITCH_MOTOR_MAX_RAD);
	}
	last_update_ms_ = input.now_ms;
}

bool XucPitchController::IsFiniteFloat(float value)
{
	uint32_t bits = 0U;
	memcpy(&bits, &value, sizeof(bits));
	return (bits & 0x7F800000U) != 0x7F800000U;
}

float XucPitchController::Clamp(float value, float lower, float upper)
{
	if (value < lower)
	{
		return lower;
	}
	if (value > upper)
	{
		return upper;
	}
	return value;
}

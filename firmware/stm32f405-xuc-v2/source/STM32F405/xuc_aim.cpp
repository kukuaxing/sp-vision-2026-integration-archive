#include "xuc_aim.h"

#include <math.h>
#include <string.h>

namespace
{
const float RAD_TO_DEG = 57.29577951308232f;
const float MAX_TOTAL_EXCURSION_DEG = 25.0f;
const float MAX_ACTUAL_EXCURSION_DEG = 28.0f;
const float MAX_SLEW_RATE_DEG_PER_SECOND = 30.0f;
const uint32_t MAX_ACCOUNTED_STEP_MS = 20U;
const uint32_t CONTROL_DROPOUT_HOLD_MS = 500U;
const float MAX_IMU_MAGNITUDE_DEG = 720.0f;
const float MAX_COMMAND_MAGNITUDE_RAD = 6.283185307179586f;
}

XucYawController xuc_yaw_controller;

XucYawControlOutput XucYawController::Update(const XucYawControlInput& input)
{
	const bool rising_arm_edge = input.arm_switch_selected && !previous_arm_switch_;
	previous_arm_switch_ = input.arm_switch_selected;

	if (!input.arm_switch_selected)
	{
		Disarm(input.current_yaw_deg, input.now_ms, false);
		rearm_ready_ = true;
		return { false, false, target_yaw_deg_ };
	}

	const bool hard_inputs_valid =
		input.rc_link_valid &&
		input.sticks_centered &&
		input.imu_valid &&
		input.command_fresh &&
		IsFiniteFloat(input.current_yaw_deg) &&
		IsFiniteFloat(input.command_yaw_rad) &&
		input.current_yaw_deg >= -MAX_IMU_MAGNITUDE_DEG &&
		input.current_yaw_deg <= MAX_IMU_MAGNITUDE_DEG &&
		input.command_yaw_rad >= -MAX_COMMAND_MAGNITUDE_RAD &&
		input.command_yaw_rad <= MAX_COMMAND_MAGNITUDE_RAD;

	if (!armed_)
	{
		if (!(rising_arm_edge && rearm_ready_ && hard_inputs_valid &&
			input.command_control == 1U))
		{
			Disarm(input.current_yaw_deg, input.now_ms, false);
			return { false, false, target_yaw_deg_ };
		}

		armed_ = true;
		rearm_ready_ = false;
		arm_origin_yaw_deg_ = NormalizeDegrees(input.current_yaw_deg);
		target_yaw_deg_ = arm_origin_yaw_deg_;
		last_update_ms_ = input.now_ms;
		last_control_true_ms_ = input.now_ms;
		return { true, true, target_yaw_deg_ };
	}

	if (!hard_inputs_valid)
	{
		Disarm(input.current_yaw_deg, input.now_ms, true);
		return { false, false, target_yaw_deg_ };
	}

	// 目标限幅不能阻止接线、符号或电机方向错误造成的反向失控。
	// 独立检查真实IMU姿态；超过目标边界3度即锁存退出，必须重新拨杆解锁。
	if (fabsf(ShortestDeltaDegrees(
		input.current_yaw_deg, arm_origin_yaw_deg_)) > MAX_ACTUAL_EXCURSION_DEG)
	{
		Disarm(input.current_yaw_deg, input.now_ms, true);
		return { false, false, target_yaw_deg_ };
	}

	if (input.command_control != 1U)
	{
		// A detector can miss a few frames while the tracker is otherwise healthy.
		// Hold the last target briefly instead of forcing a manual switch re-arm.
		// A sustained loss is still treated as a latched safety failure.
		if (static_cast<uint32_t>(input.now_ms - last_control_true_ms_) >
			CONTROL_DROPOUT_HOLD_MS)
		{
			Disarm(input.current_yaw_deg, input.now_ms, true);
			return { false, false, target_yaw_deg_ };
		}
		last_update_ms_ = input.now_ms;
		return { true, false, target_yaw_deg_ };
	}
	last_control_true_ms_ = input.now_ms;

	uint32_t elapsed_ms = static_cast<uint32_t>(input.now_ms - last_update_ms_);
	if (elapsed_ms > MAX_ACCOUNTED_STEP_MS)
	{
		elapsed_ms = MAX_ACCOUNTED_STEP_MS;
	}
	last_update_ms_ = input.now_ms;

	const float command_yaw_deg = NormalizeDegrees(input.command_yaw_rad * RAD_TO_DEG);
	const float command_from_origin = Clamp(
		ShortestDeltaDegrees(command_yaw_deg, arm_origin_yaw_deg_),
		-MAX_TOTAL_EXCURSION_DEG,
		MAX_TOTAL_EXCURSION_DEG);
	const float bounded_target = NormalizeDegrees(arm_origin_yaw_deg_ + command_from_origin);
	const float maximum_step =
		MAX_SLEW_RATE_DEG_PER_SECOND * static_cast<float>(elapsed_ms) / 1000.0f;
	const float requested_step = ShortestDeltaDegrees(bounded_target, target_yaw_deg_);
	target_yaw_deg_ = NormalizeDegrees(
		target_yaw_deg_ + Clamp(requested_step, -maximum_step, maximum_step));

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

void XucYawController::Disarm(float current_yaw_deg, uint32_t now_ms, bool require_new_edge)
{
	armed_ = false;
	if (require_new_edge)
	{
		rearm_ready_ = false;
	}
	if (IsFiniteFloat(current_yaw_deg))
	{
		arm_origin_yaw_deg_ = NormalizeDegrees(current_yaw_deg);
		target_yaw_deg_ = arm_origin_yaw_deg_;
	}
	last_update_ms_ = now_ms;
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

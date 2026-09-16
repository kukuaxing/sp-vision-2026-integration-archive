#include "xuc_aim.h"

#include <cmath>
#include <cstdio>

namespace
{
constexpr float DEG_TO_RAD = 0.017453292519943295f;

bool Near(float lhs, float rhs, float tolerance = 0.0001f)
{
	return std::fabs(lhs - rhs) <= tolerance;
}

XucYawControlInput SafeInput(uint32_t now_ms, bool arm_switch, float current_deg, float command_deg)
{
	XucYawControlInput input{};
	input.now_ms = now_ms;
	input.rc_link_valid = true;
	input.arm_switch_selected = arm_switch;
	input.sticks_centered = true;
	input.imu_valid = true;
	input.command_fresh = true;
	input.command_control = 1U;
	input.current_yaw_deg = current_deg;
	input.command_yaw_rad = command_deg * DEG_TO_RAD;
	return input;
}
}

int main()
{
	// xuc_yaw_sign already controls the command convention independently from
	// xuc_imu_yaw_sign. Bench logs prove the protocol and CH010 control yaw agree.
	if (!Near(XucYawController::ProtocolYawToLowerImuYawRadians(1.25f), 1.25f))
	{
		return 16;
	}

	XucYawController controller;

	// AUTO is level-selected: booting in AUTO arms as soon as all live gates pass.
	auto output = controller.Update(SafeInput(0U, true, 10.0f, 20.0f));
	if (!output.armed || !output.just_armed || !Near(output.target_yaw_deg, 10.0f))
	{
		return 1;
	}

	// A selected mode with vision control temporarily false stays stopped and
	// arms in place when control becomes available, without a switch cycle.
	XucYawController delayed_control_controller;
	auto control_disabled = SafeInput(0U, true, 5.0f, 8.0f);
	control_disabled.command_control = 0U;
	output = delayed_control_controller.Update(control_disabled);
	if (output.armed || delayed_control_controller.LastDisarmReasonCode() !=
		XUC_YAW_DISARM_ARM_CONTROL_DISABLED)
	{
		return 2;
	}
	output = delayed_control_controller.Update(SafeInput(5U, true, 5.0f, 8.0f));
	if (!output.armed || !output.just_armed || !Near(output.target_yaw_deg, 5.0f))
	{
		return 9;
	}

	// A valid absolute target passes through immediately; motion shaping belongs
	// to the electrical gimbal position loop, not this safety gate.
	output = controller.Update(SafeInput(5U, true, 10.0f, 20.0f));
	if (!output.armed || !Near(output.target_yaw_deg, 20.0f, 0.0002f))
	{
		return 3;
	}

	// No amount of time may move the target more than 25 degrees from arm origin.
	for (uint32_t now_ms = 10U; now_ms <= 2010U; now_ms += 5U)
	{
		output = controller.Update(SafeInput(now_ms, true, 10.0f, 80.0f));
	}
	if (!Near(output.target_yaw_deg, 35.0f, 0.0002f) ||
		std::fabs(XucYawController::ShortestDeltaDegrees(
			output.target_yaw_deg, controller.ArmOriginYawDeg())) > 25.0001f)
	{
		return 4;
	}

	// A wrong sign or motor direction must not drive the physical yaw past the
	// bounded target. Actual motion beyond 28 degrees latches AUTO off.
	output = controller.Update(SafeInput(2015U, true, 38.1f, 38.1f));
	if (output.armed ||
		controller.LastDisarmReasonCode() != XUC_YAW_DISARM_ACTUAL_EXCURSION)
	{
		return 13;
	}
	output = controller.Update(SafeInput(2020U, true, 38.1f, 38.1f));
	if (output.armed)
	{
		return 14;
	}
	controller.Update(SafeInput(2025U, false, 38.1f, 38.1f));
	output = controller.Update(SafeInput(2030U, true, 38.1f, 38.1f));
	if (!output.armed || controller.LastDisarmReasonCode() != XUC_YAW_DISARM_NONE)
	{
		return 15;
	}

	// A brief control=false caused by detector dropout holds the target and recovers.
	auto dropout = SafeInput(2035U, true, 34.0f, 34.0f);
	dropout.command_control = 0U;
	const float held_target = output.target_yaw_deg;
	output = controller.Update(dropout);
	if (!output.armed || !Near(output.target_yaw_deg, held_target))
	{
		return 5;
	}
	output = controller.Update(SafeInput(2190U, true, 34.0f, 34.0f));
	if (!output.armed || output.just_armed)
	{
		return 6;
	}

	// A stale serial/IMU/RC gate stops immediately, then recovers in AUTO without
	// requiring the operator to cycle the switch.
	auto stale = SafeInput(2195U, true, 34.0f, 34.0f);
	stale.command_fresh = false;
	output = controller.Update(stale);
	if (output.armed || !Near(output.target_yaw_deg, 34.0f) ||
		controller.LastDisarmReasonCode() != XUC_YAW_DISARM_COMMAND_STALE)
	{
		return 7;
	}
	output = controller.Update(SafeInput(2200U, true, 34.0f, 34.0f));
	if (!output.armed || !output.just_armed)
	{
		return 8;
	}

	// A sustained control=false beyond 200 ms stops at measured yaw, remains
	// armed, and resumes without requiring another switch edge.
	auto long_dropout = SafeInput(2405U, true, 34.0f, 34.0f);
	long_dropout.command_control = 0U;
	output = controller.Update(long_dropout);
	if (!output.armed || !Near(output.target_yaw_deg, 34.0f) ||
		controller.LastDisarmReasonCode() != XUC_YAW_DISARM_CONTROL_DROPOUT)
	{
		return 10;
	}
	output = controller.Update(SafeInput(2410U, true, 34.0f, 36.0f));
	if (!output.armed || output.just_armed ||
		controller.LastDisarmReasonCode() != XUC_YAW_DISARM_NONE ||
		!Near(output.target_yaw_deg, 36.0f, 0.0002f))
	{
		return 18;
	}

	// The +/-180 degree wrap takes the short path and remains bounded.
	XucYawController wrap_controller;
	wrap_controller.Update(SafeInput(5U, true, 179.0f, -179.0f));
	for (uint32_t now_ms = 10U; now_ms <= 1010U; now_ms += 5U)
	{
		output = wrap_controller.Update(SafeInput(now_ms, true, 179.0f, -179.0f));
	}
	if (!Near(output.target_yaw_deg, -179.0f, 0.0002f))
	{
		return 11;
	}

	// Implausible but finite IMU/command values are rejected before normalization.
	XucYawController range_controller;
	output = range_controller.Update(SafeInput(5U, true, 1000000.0f, 0.0f));
	if (output.armed ||
		range_controller.LastDisarmReasonCode() != XUC_YAW_DISARM_CURRENT_YAW_INVALID)
	{
		return 12;
	}

	XucPitchController pitch_controller;
	XucPitchControlInput pitch_input{};
	pitch_input.now_ms = 0U;
	pitch_input.motion_authorized = true;
	pitch_input.arm_switch_selected = true;
	pitch_input.command_control = 1U;
	pitch_input.current_pitch_deg = 6.0f;
	pitch_input.current_motor_pos_rad = 0.0f;
	pitch_input.command_pitch_rad = 8.0f * DEG_TO_RAD;
	auto pitch_output = pitch_controller.Update(pitch_input);
	if (!pitch_output.active || !pitch_output.just_armed ||
		!Near(pitch_output.target_pitch_deg, 6.0f) ||
		!Near(pitch_output.target_motor_pos_rad, 0.0f))
	{
		return 19;
	}

	// The first moving cycle is limited to 80 deg/s and uses the direction
	// confirmed by the physical IMU response: positive pitch needs positive DM
	// position.
	pitch_input.now_ms = 5U;
	pitch_output = pitch_controller.Update(pitch_input);
	if (!pitch_output.active || pitch_output.just_armed ||
		!Near(pitch_output.target_pitch_deg, 8.0f) ||
		!Near(pitch_output.target_motor_pos_rad, 0.006981317f, 0.00001f))
	{
		return 20;
	}

	for (uint32_t now_ms = 10U; now_ms <= 410U; now_ms += 5U)
	{
		pitch_input.now_ms = now_ms;
		pitch_output = pitch_controller.Update(pitch_input);
	}
	if (!Near(pitch_output.target_motor_pos_rad, 2.0f * DEG_TO_RAD, 0.0001f))
	{
		return 21;
	}

	// Commands beyond the low-speed ballistic moving-target envelope stay within +/-25 degrees.
	pitch_input.now_ms = 415U;
	pitch_input.command_pitch_rad = 40.0f * DEG_TO_RAD;
	pitch_output = pitch_controller.Update(pitch_input);
	if (!pitch_output.active || !Near(pitch_output.target_pitch_deg, 31.0f))
	{
		return 22;
	}

	// A real pitch movement beyond 28 degrees latches until AUTO is released.
	pitch_input.now_ms = 420U;
	pitch_input.current_pitch_deg = 34.1f;
	pitch_input.current_motor_pos_rad = 0.3f;
	pitch_output = pitch_controller.Update(pitch_input);
	if (pitch_output.active ||
		pitch_controller.LastDisarmReasonCode() != XUC_PITCH_DISARM_ACTUAL_EXCURSION)
	{
		return 23;
	}
	pitch_input.now_ms = 425U;
	pitch_input.current_pitch_deg = 6.0f;
	pitch_input.current_motor_pos_rad = 0.0f;
	pitch_output = pitch_controller.Update(pitch_input);
	if (pitch_output.active)
	{
		return 24;
	}
	pitch_input.now_ms = 430U;
	pitch_input.arm_switch_selected = false;
	pitch_controller.Update(pitch_input);
	pitch_input.now_ms = 435U;
	pitch_input.arm_switch_selected = true;
	pitch_input.command_pitch_rad = 6.0f * DEG_TO_RAD;
	pitch_output = pitch_controller.Update(pitch_input);
	if (!pitch_output.active || !pitch_output.just_armed ||
		pitch_controller.LastDisarmReasonCode() != XUC_PITCH_DISARM_NONE)
	{
		return 25;
	}

	// Uninitialised/stale DM feedback cannot arm pitch motion.
	XucPitchController invalid_feedback_controller;
	pitch_input.now_ms = 5U;
	pitch_input.current_motor_pos_rad = -12.566f;
	pitch_output = invalid_feedback_controller.Update(pitch_input);
	if (pitch_output.active ||
		invalid_feedback_controller.LastDisarmReasonCode() !=
			XUC_PITCH_DISARM_MOTOR_FEEDBACK_INVALID)
	{
		return 26;
	}

	// Outside AUTO, both friction wheels and feeder must remain off.
	XucAutoFireGate fire_gate;
	XucFireControlInput fire_input{};
	fire_input.now_ms = 100U;
	fire_input.rc_link_valid = true;
	fire_input.command_fresh = true;
	fire_input.tracking_armed = true;
	fire_input.pitch_active = true;
	fire_input.wheels_ready = false;
	fire_input.shooter_fault = false;
	fire_input.command_control = 1U;
	fire_input.command_shoot = 1U;
	fire_input.completed_round_count = 0U;
	auto fire_output = fire_gate.Update(fire_input);
	if (fire_output.open_friction || fire_output.feed_continuous)
	{
		return 27;
	}

	// AUTO immediately starts and continuously holds the friction wheels.
	fire_input.auto_selected = true;
	fire_input.command_shoot = 0U;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || fire_output.feed_continuous)
	{
		return 28;
	}

	// Detection/control loss stops only the feeder, never the AUTO wheels.
	fire_input.command_control = 0U;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || fire_output.feed_continuous)
	{
		return 29;
	}
	fire_input.command_control = 1U;
	fire_input.tracking_armed = false;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || fire_output.feed_continuous)
	{
		return 30;
	}
	fire_input.tracking_armed = true;
	fire_input.pitch_active = false;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || fire_output.feed_continuous)
	{
		return 31;
	}

	// A qualified decision cannot feed while the wheels are below threshold.
	fire_input.pitch_active = true;
	fire_input.command_shoot = 1U;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || fire_output.feed_continuous ||
		fire_gate.DebugCode() != 170)
	{
		return 32;
	}

	// The threshold must remain continuously true for 200 ms.
	fire_input.wheels_ready = true;
	fire_input.now_ms = 200U;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || fire_output.feed_continuous)
	{
		return 33;
	}
	fire_input.now_ms = 399U;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || fire_output.feed_continuous)
	{
		return 34;
	}
	fire_input.now_ms = 400U;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || !fire_output.feed_continuous)
	{
		return 35;
	}

	// Once accepted, a brief upper qualification dropout cannot truncate the
	// physical 45-degree feeder step.
	fire_input.command_shoot = 0U;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || !fire_output.feed_continuous ||
		fire_gate.DebugCode() != 210)
	{
		return 36;
	}

	// The completed-round counter releases the latch.
	fire_input.completed_round_count = 1U;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || fire_output.feed_continuous)
	{
		return 37;
	}

	// A shooter fault also clears a newly latched request.
	fire_input.command_shoot = 1U;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.feed_continuous)
	{
		return 38;
	}
	fire_input.shooter_fault = true;
	fire_output = fire_gate.Update(fire_input);
	if (!fire_output.open_friction || fire_output.feed_continuous)
	{
		return 39;
	}

	// Basic hard gates stop the entire shooter immediately.
	fire_input.shooter_fault = false;
	fire_input.command_fresh = false;
	fire_output = fire_gate.Update(fire_input);
	if (fire_output.open_friction || fire_output.feed_continuous)
	{
		return 40;
	}
	fire_input.command_fresh = true;
	fire_input.rc_link_valid = false;
	fire_output = fire_gate.Update(fire_input);
	if (fire_output.open_friction || fire_output.feed_continuous)
	{
		return 41;
	}

	// A real AUTO exit is also an immediate full stop.
	fire_input.rc_link_valid = true;
	fire_input.command_shoot = 1U;
	fire_input.auto_selected = false;
	fire_output = fire_gate.Update(fire_input);
	if (fire_output.open_friction || fire_output.feed_continuous)
	{
		return 42;
	}

	std::puts("XUC_PHASE2_YAW_GATE_TEST=PASS");
	return 0;
}

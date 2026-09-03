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

	// Booting with the arm switch already selected must not arm.
	auto output = controller.Update(SafeInput(0U, true, 10.0f, 20.0f));
	if (output.armed)
	{
		return 1;
	}

	// A deliberate safe-position -> arm-position edge is required.
	controller.Update(SafeInput(5U, false, 10.0f, 20.0f));
	output = controller.Update(SafeInput(10U, true, 10.0f, 20.0f));
	if (!output.armed || !output.just_armed || !Near(output.target_yaw_deg, 10.0f))
	{
		return 2;
	}

	// At a 5 ms control period, a far-away command advances only 0.15 degree.
	output = controller.Update(SafeInput(15U, true, 10.0f, 20.0f));
	if (!output.armed || !Near(output.target_yaw_deg, 10.15f, 0.0002f))
	{
		return 3;
	}

	// No amount of time may move the target more than 25 degrees from arm origin.
	for (uint32_t now_ms = 20U; now_ms <= 2020U; now_ms += 5U)
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
	output = controller.Update(SafeInput(2025U, true, 38.1f, 38.1f));
	if (output.armed)
	{
		return 13;
	}
	output = controller.Update(SafeInput(2030U, true, 38.1f, 38.1f));
	if (output.armed)
	{
		return 14;
	}
	controller.Update(SafeInput(2035U, false, 38.1f, 38.1f));
	output = controller.Update(SafeInput(2040U, true, 38.1f, 38.1f));
	if (!output.armed)
	{
		return 15;
	}

	// A brief control=false caused by detector dropout holds the target and recovers.
	auto dropout = SafeInput(2045U, true, 34.0f, 34.0f);
	dropout.command_control = 0U;
	const float held_target = output.target_yaw_deg;
	output = controller.Update(dropout);
	if (!output.armed || !Near(output.target_yaw_deg, held_target))
	{
		return 5;
	}
	output = controller.Update(SafeInput(2200U, true, 34.0f, 34.0f));
	if (!output.armed || output.just_armed)
	{
		return 6;
	}

	// A stale serial/IMU/RC gate still disarms immediately and requires a new edge.
	auto stale = SafeInput(2205U, true, 34.0f, 34.0f);
	stale.command_fresh = false;
	output = controller.Update(stale);
	if (output.armed || !Near(output.target_yaw_deg, 34.0f))
	{
		return 7;
	}
	output = controller.Update(SafeInput(2210U, true, 34.0f, 34.0f));
	if (output.armed)
	{
		return 8;
	}
	controller.Update(SafeInput(2215U, false, 34.0f, 34.0f));
	output = controller.Update(SafeInput(2220U, true, 34.0f, 34.0f));
	if (!output.armed)
	{
		return 9;
	}

	// A sustained control=false also disarms and latches until a new edge.
	auto long_dropout = SafeInput(2725U, true, 34.0f, 34.0f);
	long_dropout.command_control = 0U;
	output = controller.Update(long_dropout);
	if (output.armed)
	{
		return 10;
	}

	// The +/-180 degree wrap takes the short path and remains bounded.
	XucYawController wrap_controller;
	wrap_controller.Update(SafeInput(0U, false, 179.0f, -179.0f));
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
	range_controller.Update(SafeInput(0U, false, 0.0f, 0.0f));
	output = range_controller.Update(SafeInput(5U, true, 1000000.0f, 0.0f));
	if (output.armed)
	{
		return 12;
	}

	// shoot/pitch are intentionally absent from this controller's inputs.
	std::puts("XUC_PHASE2_YAW_GATE_TEST=PASS");
	return 0;
}

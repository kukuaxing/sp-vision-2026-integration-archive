#include "../STM32F405/auto_yaw_drive.h"

#include <cmath>
#include <cstdio>

namespace
{
bool Near(float actual, float expected, float tolerance = 0.001f)
{
	return std::fabs(actual - expected) <= tolerance;
}
}

int main()
{
	AutoYawDrive drive;
	drive.Reset(0.0f, 0U);

	auto output = drive.Update(0.0f, 10.0f, 0.0f, 5U);
	if (!output.motion_active || output.speed_rpm <= 0.0f ||
		output.speed_rpm > 0.301f || output.current_feedforward <= 0 ||
		output.current_feedforward > 61)
		return 1;

	float previous_speed = output.speed_rpm;
	int32_t previous_current = output.current_feedforward;
	for (uint32_t now_ms = 10U; now_ms <= 200U; now_ms += 5U)
	{
		output = drive.Update(0.0f, 10.0f, 0.0f, now_ms);
		if (output.speed_rpm - previous_speed > 0.301f ||
			output.current_feedforward - previous_current > 61)
			return 2;
		previous_speed = output.speed_rpm;
		previous_current = output.current_feedforward;
	}
	if (!Near(output.speed_rpm, 12.0f, 0.01f) ||
		output.current_feedforward != 2400)
		return 3;

	// A single low-speed feedback sample must not collapse the friction torque.
	// Instrumented hardware testing showed that the old 2600 kinetic value
	// stalled the 6020 indefinitely after the first 1 rpm sample.
	for (uint32_t now_ms = 205U; now_ms <= 400U; now_ms += 5U)
		output = drive.Update(0.0f, 10.0f, 2.0f, now_ms);
	if (output.current_feedforward != 4500)
		return 9;

	// Once motion is clearly established, use lower kinetic-friction torque.
	// A later stall must automatically restore the breakaway current.
	AutoYawDrive friction_transition_drive;
	friction_transition_drive.Reset(0.0f, 0U);
	for (uint32_t now_ms = 5U; now_ms <= 400U; now_ms += 5U)
		output = friction_transition_drive.Update(0.0f, 10.0f, 0.0f, now_ms);
	for (uint32_t now_ms = 405U; now_ms <= 500U; now_ms += 5U)
		output = friction_transition_drive.Update(0.0f, 10.0f, 6.0f, now_ms);
	if (output.current_feedforward != 3200)
		return 10;
	for (uint32_t now_ms = 505U; now_ms <= 650U; now_ms += 5U)
		output = friction_transition_drive.Update(0.0f, 10.0f, 0.0f, now_ms);
	if (output.current_feedforward != 4500)
		return 11;

	// A 2 deg/s moving target remains active even with near-zero position error.
	for (uint32_t now_ms = 405U; now_ms <= 2405U; now_ms += 5U)
	{
		const float target_deg = 2.0f * static_cast<float>(now_ms - 400U) / 1000.0f;
		output = drive.Update(target_deg, 0.05f, 0.5f, now_ms);
	}
	if (!output.motion_active || output.target_rate_rpm < 0.20f ||
		output.speed_rpm <= 0.0f || output.current_feedforward <= 0)
		return 4;

	// A stationary centered target ramps both outputs completely to zero.
	for (uint32_t now_ms = 2410U; now_ms <= 2700U; now_ms += 5U)
		output = drive.Update(4.01f, 0.05f, 0.0f, now_ms);
	if (output.motion_active || !Near(output.speed_rpm, 0.0f) ||
		output.current_feedforward != 0)
		return 5;

	// A reversal crosses zero with bounded speed/current slew and no hard step.
	drive.Reset(5.0f, 3000U);
	for (uint32_t now_ms = 3005U; now_ms <= 3150U; now_ms += 5U)
		output = drive.Update(5.0f, 8.0f, 0.0f, now_ms);
	previous_speed = output.speed_rpm;
	previous_current = output.current_feedforward;
	for (uint32_t now_ms = 3155U; now_ms <= 3400U; now_ms += 5U)
	{
		output = drive.Update(4.99f, -8.0f, 0.0f, now_ms);
		if (std::fabs(output.speed_rpm - previous_speed) > 0.901f ||
			std::abs(output.current_feedforward - previous_current) > 601)
			return 6;
		previous_speed = output.speed_rpm;
		previous_current = output.current_feedforward;
	}
	if (output.speed_rpm >= 0.0f || output.current_feedforward >= 0)
		return 7;

	if (!Near(AutoYawDrive::ShortestDeltaDegrees(-179.0f, 179.0f), 2.0f) ||
		!Near(AutoYawDrive::ShortestDeltaDegrees(179.0f, -179.0f), -2.0f))
		return 8;

	std::puts("AUTO_YAW_DRIVE_TEST=PASS");
	return 0;
}

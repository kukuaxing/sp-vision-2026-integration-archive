#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

struct AutoYawDriveOutput
{
	float speed_rpm = 0.0f;
	int32_t current_feedforward = 0;
	float target_rate_rpm = 0.0f;
	bool motion_active = false;
};

// Converts the bounded IMU yaw target into a continuous 6020 speed command.
// A ramped Coulomb-friction feedforward supplies holding torque at low speed,
// avoiding the old 0 <-> 120 rpm start/stop cycle during slow target motion.
class AutoYawDrive
{
public:
	void Reset(float target_yaw_deg, uint32_t now_ms)
	{
		initialized_ = true;
		last_target_yaw_deg_ = target_yaw_deg;
		last_update_ms_ = now_ms;
		filtered_target_rate_rpm_ = 0.0f;
		speed_command_rpm_ = 0.0f;
		current_feedforward_ = 0.0f;
		kinetic_motion_latched_ = false;
	}

	AutoYawDriveOutput Update(
		float target_yaw_deg, float error_deg, float measured_speed_rpm, uint32_t now_ms)
	{
		if (!initialized_)
		{
			Reset(target_yaw_deg, now_ms);
			return {};
		}

		uint32_t elapsed_ms = now_ms - last_update_ms_;
		if (elapsed_ms == 0U)
			elapsed_ms = 1U;
		if (elapsed_ms > max_accounted_step_ms)
			elapsed_ms = max_accounted_step_ms;

		const float target_step_deg = ShortestDeltaDegrees(
			target_yaw_deg, last_target_yaw_deg_);
		float raw_target_rate_rpm = target_step_deg *
			(1000.0f / static_cast<float>(elapsed_ms)) / 6.0f;
		raw_target_rate_rpm = Clamp(raw_target_rate_rpm,
			-max_target_rate_rpm, max_target_rate_rpm);
		filtered_target_rate_rpm_ += target_rate_filter_alpha *
			(raw_target_rate_rpm - filtered_target_rate_rpm_);

		last_target_yaw_deg_ = target_yaw_deg;
		last_update_ms_ = now_ms;

		const bool target_moving =
			std::fabs(filtered_target_rate_rpm_) >= moving_rate_threshold_rpm;
		const bool correction_needed = std::fabs(error_deg) >= stationary_deadband_deg;
		const bool motion_active = target_moving || correction_needed;
		float desired_speed_rpm = 0.0f;
		if (motion_active)
		{
			desired_speed_rpm = filtered_target_rate_rpm_ + error_gain_rpm_per_deg * error_deg;
			desired_speed_rpm = Clamp(desired_speed_rpm, -speed_limit_rpm, speed_limit_rpm);
			if (std::fabs(desired_speed_rpm) < minimum_nonzero_speed_rpm)
			{
				const float direction = (std::fabs(error_deg) >= stationary_deadband_deg)
					? error_deg : filtered_target_rate_rpm_;
				desired_speed_rpm = (direction >= 0.0f)
					? minimum_nonzero_speed_rpm : -minimum_nonzero_speed_rpm;
			}
		}

		// The 6020 needs the full breakaway current while stationary, but keeping
		// that current after the rotor is moving overwhelms the low-speed loop and
		// produces a several-degree limit cycle.  Enter the lower kinetic-friction
		// state only after clearly established motion; do not drop it on the first
		// 1 rpm feedback sample (that behaviour stalled on the real mechanism).
		const bool moving_in_command_direction =
			measured_speed_rpm * desired_speed_rpm > 0.0f;
		if (!motion_active || !moving_in_command_direction ||
			std::fabs(measured_speed_rpm) <= kinetic_exit_speed_rpm)
		{
			kinetic_motion_latched_ = false;
		}
		else if (std::fabs(measured_speed_rpm) >= kinetic_enter_speed_rpm)
		{
			kinetic_motion_latched_ = true;
		}

		const float dt_seconds = static_cast<float>(elapsed_ms) / 1000.0f;
		const bool speed_reversing_or_reducing =
			speed_command_rpm_ * desired_speed_rpm < 0.0f ||
			std::fabs(desired_speed_rpm) < std::fabs(speed_command_rpm_);
		const float speed_slew = speed_reversing_or_reducing
			? speed_fall_slew_rpm_per_second : speed_rise_slew_rpm_per_second;
		speed_command_rpm_ = SlewTowards(speed_command_rpm_, desired_speed_rpm,
			speed_slew * dt_seconds);

		float desired_current_feedforward = 0.0f;
		if (motion_active)
		{
			const float direction = (std::fabs(speed_command_rpm_) >= 0.05f)
				? speed_command_rpm_ : desired_speed_rpm;
			const float friction_current = kinetic_motion_latched_
				? kinetic_friction_current : breakaway_friction_current;
			desired_current_feedforward = (direction >= 0.0f)
				? friction_current : -friction_current;
		}
		const bool reversing_or_reducing =
			(current_feedforward_ * desired_current_feedforward < 0.0f) ||
			(std::fabs(desired_current_feedforward) < std::fabs(current_feedforward_));
		const float current_slew = reversing_or_reducing
			? current_fall_slew_per_second : current_rise_slew_per_second;
		current_feedforward_ = SlewTowards(current_feedforward_,
			desired_current_feedforward, current_slew * dt_seconds);

		if (!motion_active && std::fabs(speed_command_rpm_) < 0.01f &&
			std::fabs(current_feedforward_) < 0.5f)
		{
			speed_command_rpm_ = 0.0f;
			current_feedforward_ = 0.0f;
		}

		AutoYawDriveOutput output{};
		output.speed_rpm = speed_command_rpm_;
		output.current_feedforward = static_cast<int32_t>(current_feedforward_);
		output.target_rate_rpm = filtered_target_rate_rpm_;
		output.motion_active = motion_active;
		return output;
	}

	static float ShortestDeltaDegrees(float target_deg, float current_deg)
	{
		float delta = target_deg - current_deg;
		while (delta > 180.0f)
			delta -= 360.0f;
		while (delta <= -180.0f)
			delta += 360.0f;
		return delta;
	}

	static constexpr float stationary_deadband_deg = 0.80f;
	static constexpr float speed_limit_rpm = 60.0f;
	// Instrumented Phase 2D.5.2 testing showed that 2600 feedforward plus
	// roughly 250 from the speed loop (about 2850 total) could not break static
	// friction. Use 4500 only for breakaway, then reduce it after measured speed
	// proves that the rotor is moving; stopping automatically restores breakaway.
	static constexpr float friction_current = 4500.0f;

private:
	static float Clamp(float value, float minimum, float maximum)
	{
		return std::max(std::min(value, maximum), minimum);
	}

	static float SlewTowards(float current, float target, float maximum_step)
	{
		return current + Clamp(target - current, -maximum_step, maximum_step);
	}

	static constexpr uint32_t max_accounted_step_ms = 20U;
	static constexpr float max_target_rate_rpm = 2.5f; // 15 deg/s lower target slew.
	static constexpr float target_rate_filter_alpha = 0.18f;
	static constexpr float moving_rate_threshold_rpm = 0.12f; // 0.72 deg/s.
	static constexpr float error_gain_rpm_per_deg = 1.5f;
	static constexpr float minimum_nonzero_speed_rpm = 1.0f;
	static constexpr float kinetic_enter_speed_rpm = 4.0f;
	static constexpr float kinetic_exit_speed_rpm = 0.4f;
	static constexpr float kinetic_friction_current = 3200.0f;
	static constexpr float breakaway_friction_current = friction_current;
	static constexpr float speed_rise_slew_rpm_per_second = 60.0f;
	static constexpr float speed_fall_slew_rpm_per_second = 180.0f;
	static constexpr float current_rise_slew_per_second = 12000.0f;
	static constexpr float current_fall_slew_per_second = 120000.0f;

	bool initialized_ = false;
	float last_target_yaw_deg_ = 0.0f;
	uint32_t last_update_ms_ = 0U;
	float filtered_target_rate_rpm_ = 0.0f;
	float speed_command_rpm_ = 0.0f;
	float current_feedforward_ = 0.0f;
	bool kinetic_motion_latched_ = false;
};

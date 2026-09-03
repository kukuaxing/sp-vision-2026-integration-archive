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

// Phase 2B keeps the Phase 2A safety gates while allowing practical yaw tracking:
// - an explicit DR16 switch edge is required to arm;
// - RC, IMU and serial freshness failures still disarm immediately;
// - short vision-control dropouts hold the current yaw target;
// - the total target stays within +/-25 degrees of the arm position;
// - actual IMU excursion beyond 28 degrees disarms and requires a new edge;
// - the target slews at no more than 30 degrees per second.
class XucYawController
{
public:
	XucYawControlOutput Update(const XucYawControlInput& input);
	bool Armed() const { return armed_; }
	float ArmOriginYawDeg() const { return arm_origin_yaw_deg_; }

	static float NormalizeDegrees(float angle_deg);
	static float ShortestDeltaDegrees(float target_deg, float current_deg);
	static float ProtocolYawToLowerImuYawRadians(float protocol_yaw_rad)
	{
		return protocol_yaw_rad;
	}

private:
	void Disarm(float current_yaw_deg, uint32_t now_ms, bool require_new_edge);
	static bool IsFiniteFloat(float value);
	static float Clamp(float value, float lower, float upper);

	bool armed_ = false;
	bool previous_arm_switch_ = false;
	bool rearm_ready_ = false;
	float arm_origin_yaw_deg_ = 0.0f;
	float target_yaw_deg_ = 0.0f;
	uint32_t last_update_ms_ = 0U;
	uint32_t last_control_true_ms_ = 0U;
};

extern XucYawController xuc_yaw_controller;

#pragma once
#include <cstdint>

constexpr uint16_t PITCH_ZERO_RAW = 32685;
constexpr float PITCH_ZERO_RAD = PITCH_ZERO_RAW * (25.13274f / 65535.0f) - 12.56637f;
constexpr float PITCH_RETURN_SPEED = 0.10f;
// Temporary return corridor, not calibrated mechanical travel limits.
constexpr float PITCH_RETURN_WINDOW = 0.20f;
void PitchZeroUpdate();
extern volatile uint32_t pitch_zero_state;
extern volatile uint32_t pitch_zero_fault;
extern volatile float pitch_relative_rad;
extern volatile float pitch_command_rad;

// Validated DBUS frames; only UP/DOWN enables calibration joystick control.
void GimbalRemoteFrame(const uint8_t* frame);
extern volatile uint32_t gimbal_remote_active;
extern volatile uint32_t pitch_home_ready;
extern volatile uint32_t yaw_manual_state;
extern volatile uint32_t yaw_manual_fault;

extern volatile uint32_t small_yaw_state, small_yaw_fault;
extern volatile float small_yaw_relative_rad, small_yaw_target_rad;
extern volatile int32_t small_yaw_output;

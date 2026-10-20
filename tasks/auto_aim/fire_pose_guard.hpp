#pragma once

#include <cmath>
#include <limits>

#include "io/sentinel/pose_history.hpp"

namespace auto_aim {

// Prediction and actuator following must use the same calibrated yaw basis.
// Apply the fixed boresight/impact correction once, before the optional event
// trim. The candidate gate and the final post-wait gate share this reference.
inline double calibrated_fire_yaw(double predicted_yaw, double fixed_correction, double trim)
{
  if (!std::isfinite(predicted_yaw) || !std::isfinite(fixed_correction) || !std::isfinite(trim))
    return NAN;
  return std::remainder(predicted_yaw + fixed_correction - trim, 2.0 * io::sentinel::pi);
}

enum class FirePoseReject {
  none, invalid_pose, reference_changed, invalid_numeric, yaw_error, pitch_error
};

struct FirePoseGuardResult {
  bool allowed = false;
  FirePoseReject reason = FirePoseReject::invalid_pose;
  double yaw_error_rad = std::numeric_limits<double>::quiet_NaN();
  double pitch_error_rad = std::numeric_limits<double>::quiet_NaN();
};

// Revalidate a previously qualified shot after its deliberate deadline wait.
// All angles are radians. Both yaw values must share a basis (Sentinel uses
// sensor Euler yaw after inverse geometry). The pitch residual must include the calibrated
// image_angular_delta from the observation pose to the newest pose.
//
// This guard does not infer image motion from a raw pitch delta: the real
// camera lever arm and installation rotation couple all three attitude axes.
// Relative reprojection remains an estimate for the last observed world point,
// not a fresh image observation or a prediction of unobserved target motion.
// Main must additionally recheck current AUTO/control permission before TX.
inline FirePoseGuardResult fire_pose_guard(
  const io::sentinel::PoseEstimate & before,
  const io::sentinel::PoseEstimate & after,
  double current_world_yaw_rad, double command_world_yaw_rad,
  double yaw_tolerance_rad, double latest_pitch_residual_rad, double pitch_tolerance_rad)
{
  FirePoseGuardResult out;
  if (!before.valid || !after.valid) return out;
  if (before.boot_id != after.boot_id || before.epoch != after.epoch ||
      before.generation != after.generation) {
    out.reason = FirePoseReject::reference_changed;
    return out;
  }
  if (!std::isfinite(current_world_yaw_rad) || !std::isfinite(command_world_yaw_rad) ||
      !std::isfinite(yaw_tolerance_rad) || yaw_tolerance_rad < 0 ||
      !std::isfinite(latest_pitch_residual_rad) ||
      !std::isfinite(pitch_tolerance_rad) || pitch_tolerance_rad < 0) {
    out.reason = FirePoseReject::invalid_numeric;
    return out;
  }
  out.yaw_error_rad = std::remainder(
    command_world_yaw_rad - current_world_yaw_rad, 2.0 * io::sentinel::pi);
  out.pitch_error_rad = latest_pitch_residual_rad;
  if (!std::isfinite(out.yaw_error_rad)) {
    out.reason=FirePoseReject::invalid_numeric;
    return out;
  }
  if (std::abs(out.yaw_error_rad) > yaw_tolerance_rad) {
    out.reason = FirePoseReject::yaw_error;
    return out;
  }
  if (std::abs(out.pitch_error_rad) > pitch_tolerance_rad) {
    out.reason = FirePoseReject::pitch_error;
    return out;
  }
  out.allowed = true;
  out.reason = FirePoseReject::none;
  return out;
}

} // namespace auto_aim

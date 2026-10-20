#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <Eigen/Geometry>

namespace auto_aim {

// Sentinel sends sensor-body -> gravity-aligned NAV attitude. Installation
// rotates G into the sensor body; it must not rotate NAV a second time.
class SentinelGeometry {
public:
  SentinelGeometry(const Eigen::Matrix3d & sensor_from_gimbal,
                   const Eigen::Matrix3d & gimbal_from_camera)
  : sensor_from_gimbal_(sensor_from_gimbal),
    camera_forward_sensor_((sensor_from_gimbal * gimbal_from_camera.col(2)).normalized()) {}

  static Eigen::Quaterniond attitude(const std::array<double, 3> & rpy) {
    return Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
  }

  Eigen::Matrix3d world_rotation(const std::array<double, 3> & sensor_rpy) const {
    return attitude(sensor_rpy).toRotationMatrix() * sensor_from_gimbal_;
  }

  double camera_world_yaw(const std::array<double, 3> & sensor_rpy) const {
    const Eigen::Vector3d forward = attitude(sensor_rpy) * camera_forward_sensor_;
    if (!forward.allFinite() || forward.head<2>().norm() < 1e-6)
      return std::numeric_limits<double>::quiet_NaN();
    return std::atan2(forward.y(), forward.x());
  }

  // At unchanged sensor pitch/roll, changing sensor yaw rotates the optical
  // ray about NAV Z by exactly the same angle. Keep command filters/limiters
  // in the same sensor Euler basis as the MCU, rather than mixing world yaw.
  double sensor_yaw_for_world_bearing(
    double bearing, const std::array<double, 3> & sensor_rpy) const {
    return wrap(sensor_rpy[2] + wrap(bearing - camera_world_yaw(sensor_rpy)));
  }

  double world_bearing_for_sensor_yaw(
    double sensor_yaw, const std::array<double, 3> & sensor_rpy) const {
    return wrap(camera_world_yaw(sensor_rpy) + wrap(sensor_yaw - sensor_rpy[2]));
  }

  // Stationary fire uses the same point-to-sensor mapping and fixed correction
  // as following. Image-centre residuals omit boresight and principal-point
  // offsets and can veto an already settled, calibrated following command.
  double calibrated_yaw_for_world_point(
    const Eigen::Vector3d & point, const std::array<double, 3> & sensor_rpy,
    double fixed_correction) const {
    if (!point.allFinite() || point.head<2>().norm() < 1e-6 ||
        !std::isfinite(fixed_correction)) return std::numeric_limits<double>::quiet_NaN();
    return wrap(sensor_yaw_for_world_bearing(
      std::atan2(point.y(), point.x()), sensor_rpy) + fixed_correction);
  }

  // Native Aimer supplies a world optical-ray yaw and pitch (positive pitch
  // points down). Invert Rz(yaw) Ry(pitch) Rx(current_roll) * camera_forward_S.
  // Direct pixel-pitch control already produces sensor pitch and bypasses this
  // conversion after replacing the native command; it is never converted twice.
  std::optional<Eigen::Vector2d> sensor_angles_for_world_ray(
    double yaw, double pitch, const std::array<double, 3> & sensor_rpy) const {
    if (!std::isfinite(yaw) || !std::isfinite(pitch) ||
        !std::isfinite(sensor_rpy[0]) || !std::isfinite(sensor_rpy[1]) ||
        !std::isfinite(sensor_rpy[2]) || std::abs(pitch) >= pi / 2)
      return std::nullopt;
    const Eigen::Vector3d v =
      Eigen::AngleAxisd(sensor_rpy[0], Eigen::Vector3d::UnitX()) * camera_forward_sensor_;
    const double radius = std::hypot(v.x(), v.z());
    const double target_z = -std::sin(pitch);
    if (!v.allFinite() || radius < 1e-6 || std::abs(target_z) > radius)
      return std::nullopt;
    const double alpha = std::atan2(v.z(), v.x());
    const double beta = std::asin(target_z / radius);
    double best_distance = std::numeric_limits<double>::infinity();
    std::optional<Eigen::Vector2d> best;
    for (const double solution : {beta, pi - beta}) {
      const double sensor_pitch = wrap(alpha - solution);
      // Sensor ZYX Euler representation must remain on its ordinary pitch
      // branch. An unreachable ray is rejected, not folded through a gimbal flip.
      if (std::abs(sensor_pitch) >= pi / 2) continue;
      const double x = std::cos(sensor_pitch) * v.x() + std::sin(sensor_pitch) * v.z();
      const double sensor_yaw = wrap(yaw - std::atan2(v.y(), x));
      const double distance = std::abs(sensor_pitch - sensor_rpy[1]);
      if (std::isfinite(sensor_yaw) && distance < best_distance) {
        best_distance = distance;
        best = Eigen::Vector2d(sensor_yaw, sensor_pitch);
      }
    }
    return best;
  }

private:
  static constexpr double pi = 3.14159265358979323846;
  static double wrap(double angle) { return std::remainder(angle, 2 * pi); }
  Eigen::Matrix3d sensor_from_gimbal_;
  Eigen::Vector3d camera_forward_sensor_;
};

} // namespace auto_aim

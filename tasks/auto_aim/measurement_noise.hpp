#ifndef AUTO_AIM__MEASUREMENT_NOISE_HPP
#define AUTO_AIM__MEASUREMENT_NOISE_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace auto_aim
{

// Distance-aware polar covariance derived from angular and radial sensor
// uncertainty.  This is the polar equivalent of SHtech's anisotropic Cartesian
// covariance and avoids treating a five-metre PnP range like a close target.
inline std::array<double, 4> armor_measurement_variances(
  double range_m, double incidence_rad, double confidence,
  double height_px = std::numeric_limits<double>::infinity(),
  double width_px = std::numeric_limits<double>::infinity(),
  double reprojection_error_px = 0.0, double pose_quality = 1.0)
{
  constexpr double DEG = M_PI / 180.0;
  range_m = std::clamp(std::abs(range_m), 0.2, 15.0);
  confidence = std::clamp(confidence, 0.25, 1.0);
  const double covariance_scale = 1.0 / confidence;

  const double yaw_sigma = 0.75 * DEG;
  const double pitch_sigma = 0.75 * DEG;
  double range_sigma = 0.03 + 0.025 * range_m;
  double armor_yaw_sigma =
    (8.0 + 18.0 * std::abs(std::sin(incidence_rad))) * DEG;
  if (std::isfinite(height_px) && std::isfinite(width_px) &&
      height_px > 0.0 && width_px > 0.0) {
    // A small reprojection residual does not make the plate normal observable.
    // Propagate a pixel-error floor through range ~ 1 / image height and the
    // near-frontal width ~ cos(yaw) ambiguity. The latter is second-order:
    // sigma_yaw ~ sqrt(2*sqrt(2)*sigma_pixel / image_width).
    const double residual = std::isfinite(reprojection_error_px)
      ? std::clamp(reprojection_error_px, 0.0, 4.0) : 0.0;
    const double quality = std::isfinite(pose_quality)
      ? std::clamp(pose_quality, 0.25, 1.0) : 0.25;
    const double pixel_sigma = std::max(0.75, residual) / std::sqrt(quality);
    range_sigma = std::max(range_sigma, range_m * pixel_sigma / std::max(2.0, height_px));
    armor_yaw_sigma = std::max(armor_yaw_sigma, std::min(
      M_PI / 2.0, std::sqrt(2.0 * std::sqrt(2.0) * pixel_sigma / std::max(3.0, width_px))));
  }
  return {
    covariance_scale * yaw_sigma * yaw_sigma,
    covariance_scale * pitch_sigma * pitch_sigma,
    covariance_scale * range_sigma * range_sigma,
    covariance_scale * armor_yaw_sigma * armor_yaw_sigma};
}

}  // namespace auto_aim

#endif  // AUTO_AIM__MEASUREMENT_NOISE_HPP

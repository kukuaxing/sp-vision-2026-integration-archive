#ifndef AUTO_AIM__MEASUREMENT_NOISE_HPP
#define AUTO_AIM__MEASUREMENT_NOISE_HPP

#include <algorithm>
#include <array>
#include <cmath>

namespace auto_aim
{

// Distance-aware polar covariance derived from angular and radial sensor
// uncertainty.  This is the polar equivalent of SHtech's anisotropic Cartesian
// covariance and avoids treating a five-metre PnP range like a close target.
inline std::array<double, 4> armor_measurement_variances(
  double range_m, double incidence_rad, double confidence)
{
  constexpr double DEG = M_PI / 180.0;
  range_m = std::clamp(std::abs(range_m), 0.2, 15.0);
  confidence = std::clamp(confidence, 0.25, 1.0);
  const double covariance_scale = 1.0 / confidence;

  const double yaw_sigma = 0.75 * DEG;
  const double pitch_sigma = 0.75 * DEG;
  const double range_sigma = 0.03 + 0.025 * range_m;
  const double armor_yaw_sigma =
    (8.0 + 18.0 * std::abs(std::sin(incidence_rad))) * DEG;
  return {
    covariance_scale * yaw_sigma * yaw_sigma,
    covariance_scale * pitch_sigma * pitch_sigma,
    covariance_scale * range_sigma * range_sigma,
    covariance_scale * armor_yaw_sigma * armor_yaw_sigma};
}

}  // namespace auto_aim

#endif  // AUTO_AIM__MEASUREMENT_NOISE_HPP

#ifndef AUTO_AIM__FINAL_YAW_LIMITER_HPP
#define AUTO_AIM__FINAL_YAW_LIMITER_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

namespace auto_aim
{
// Conditions the complete absolute yaw command, including the first frame of
// every control session. This does not establish actuator/IMU axis conventions.
class FinalYawLimiter
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit FinalYawLimiter(double max_rate_rad_s) : max_rate_rad_s_(max_rate_rad_s) {}

  std::optional<double> update(
    double requested_yaw, double measured_yaw, bool control_active, bool lower_armed,
    TimePoint now)
  {
    // Keep the high-water mark even if a caller supplies an older timestamp.
    const double dt = have_time_
      ? std::clamp(std::chrono::duration<double>(now - last_update_).count(), 0.0, 0.05)
      : 0.0;
    if (!have_time_ || now > last_update_) last_update_ = now;
    have_time_ = true;

    if (!control_active || !std::isfinite(requested_yaw) ||
        !std::isfinite(measured_yaw) || !std::isfinite(max_rate_rad_s_) ||
        max_rate_rad_s_ <= 0.0) {
      active_ = armed_ = false;
      return std::nullopt;
    }

    // Send a real measured-angle target to arm the lower board. Waiting for its
    // armed bit before conditioning would let the first full target bypass the
    // limiter. Re-anchor on a revoked/reacquired lease as well as an AUTO entry.
    if (!active_ || !lower_armed || !armed_) {
      filtered_yaw_ = wrap(measured_yaw);
      active_ = true;
      armed_ = lower_armed;
      return filtered_yaw_;
    }

    const double step = wrap(requested_yaw - filtered_yaw_);
    const double max_step = max_rate_rad_s_ * dt;
    filtered_yaw_ = wrap(filtered_yaw_ + std::clamp(step, -max_step, max_step));
    return filtered_yaw_;
  }

private:
  static double wrap(double angle)
  {
    constexpr double two_pi = 6.28318530717958647692;
    return std::remainder(angle, two_pi);
  }

  double max_rate_rad_s_;
  bool have_time_ = false;
  bool active_ = false;
  bool armed_ = false;
  double filtered_yaw_ = 0.0;
  TimePoint last_update_{};
};
}  // namespace auto_aim

#endif  // AUTO_AIM__FINAL_YAW_LIMITER_HPP

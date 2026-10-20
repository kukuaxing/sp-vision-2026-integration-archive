#ifndef AUTO_AIM__FINAL_PITCH_LIMITER_HPP
#define AUTO_AIM__FINAL_PITCH_LIMITER_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

namespace auto_aim
{
// Pitch is a linear command angle: never take a wrapped path through +/-pi.
// Start every AUTO/reacquired lease at its measured command-basis angle.
class FinalPitchLimiter
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit FinalPitchLimiter(double max_rate_rad_s) : max_rate_rad_s_(max_rate_rad_s) {}

  std::optional<double> update(
    double requested_pitch, double measured_pitch, bool control_active, bool lower_armed,
    TimePoint now)
  {
    const double dt = have_time_
      ? std::clamp(std::chrono::duration<double>(now - last_update_).count(), 0.0, 0.05)
      : 0.0;
    if (!have_time_ || now > last_update_) last_update_ = now;
    have_time_ = true;

    if (!control_active || !std::isfinite(requested_pitch) ||
        !std::isfinite(measured_pitch) || !std::isfinite(max_rate_rad_s_) ||
        max_rate_rad_s_ <= 0.0) {
      active_ = armed_ = false;
      return std::nullopt;
    }
    if (!active_ || !lower_armed || !armed_) {
      filtered_pitch_ = measured_pitch;
      active_ = true;
      armed_ = lower_armed;
      return filtered_pitch_;
    }

    const double max_step = max_rate_rad_s_ * dt;
    filtered_pitch_ += std::clamp(
      requested_pitch - filtered_pitch_, -max_step, max_step);
    return filtered_pitch_;
  }

private:
  double max_rate_rad_s_;
  bool have_time_ = false;
  bool active_ = false;
  bool armed_ = false;
  double filtered_pitch_ = 0.0;
  TimePoint last_update_{};
};
}  // namespace auto_aim

#endif  // AUTO_AIM__FINAL_PITCH_LIMITER_HPP

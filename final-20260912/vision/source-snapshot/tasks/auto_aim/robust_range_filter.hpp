#ifndef AUTO_AIM__ROBUST_RANGE_FILTER_HPP
#define AUTO_AIM__ROBUST_RANGE_FILTER_HPP

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_aim
{

// Smooths the range used by both ballistic elevation and gyro time-of-flight.
// The rigid-body model is the primary observation; raw single-frame PnP depth
// is retained only as a small, bounded correction because oblique armor poses
// can make monocular depth jump by metres without any physical target motion.
class RobustRangeFilter
{
public:
  struct Config
  {
    double time_constant_s = 0.08;
    double max_rate_mps = 6.0;
    double raw_blend = 0.05;
    double max_raw_model_residual_m = 0.40;
    double reset_gap_s = 0.50;
    double hard_reset_innovation_m = 1.50;
    double min_range_m = 0.10;
    double max_range_m = 30.0;
  };

  struct Result
  {
    bool valid = false;
    bool used_model = false;
    bool raw_limited = false;
    double range_m = std::numeric_limits<double>::quiet_NaN();
    double candidate_m = std::numeric_limits<double>::quiet_NaN();
  };

  RobustRangeFilter() = default;
  explicit RobustRangeFilter(const Config & config) : config_(config) {}

  void reset()
  {
    initialized_ = false;
    value_m_ = std::numeric_limits<double>::quiet_NaN();
    last_time_s_ = 0.0;
  }

  Result update(double time_s, double model_range_m, double raw_range_m)
  {
    Result out;
    if (!std::isfinite(time_s)) return out;

    const bool model_valid = range_valid(model_range_m);
    const bool raw_valid = range_valid(raw_range_m);
    if (!model_valid && !raw_valid) return out;

    double candidate_m = model_valid ? model_range_m : raw_range_m;
    out.used_model = model_valid;
    if (model_valid && raw_valid) {
      const double residual_m = raw_range_m - model_range_m;
      const double limited_residual_m = std::clamp(
        residual_m,
        -std::max(0.0, config_.max_raw_model_residual_m),
        std::max(0.0, config_.max_raw_model_residual_m));
      out.raw_limited = std::abs(limited_residual_m - residual_m) > 1e-9;
      candidate_m += std::clamp(config_.raw_blend, 0.0, 1.0) * limited_residual_m;
    }
    candidate_m = std::clamp(candidate_m, config_.min_range_m, config_.max_range_m);
    out.candidate_m = candidate_m;

    const double dt_s = initialized_ ? time_s - last_time_s_ : 0.0;
    if (!initialized_ || !(dt_s > 0.0) || dt_s > config_.reset_gap_s ||
        std::abs(candidate_m - value_m_) > config_.hard_reset_innovation_m) {
      value_m_ = candidate_m;
      initialized_ = true;
    } else {
      const double alpha = config_.time_constant_s > 0.0
        ? 1.0 - std::exp(-dt_s / config_.time_constant_s)
        : 1.0;
      double step_m = alpha * (candidate_m - value_m_);
      if (config_.max_rate_mps > 0.0) {
        const double max_step_m = config_.max_rate_mps * dt_s;
        step_m = std::clamp(step_m, -max_step_m, max_step_m);
      }
      value_m_ += step_m;
    }
    last_time_s_ = time_s;
    out.valid = true;
    out.range_m = value_m_;
    return out;
  }

  bool initialized() const { return initialized_; }
  double value_m() const { return value_m_; }

private:
  bool range_valid(double range_m) const
  {
    return std::isfinite(range_m) && range_m >= config_.min_range_m &&
      range_m <= config_.max_range_m;
  }

  Config config_{};
  bool initialized_ = false;
  double value_m_ = std::numeric_limits<double>::quiet_NaN();
  double last_time_s_ = 0.0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ROBUST_RANGE_FILTER_HPP

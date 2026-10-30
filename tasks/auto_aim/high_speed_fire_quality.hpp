// Adapted from Reimuaa/Fujian_AutoAim @ bb0240bbdbdce95e35fbd7eaa442f167e302ce31.
// MIT license and attribution: docs/third_party_fujian_autoaim.md.
#ifndef AUTO_AIM__HIGH_SPEED_FIRE_QUALITY_HPP
#define AUTO_AIM__HIGH_SPEED_FIRE_QUALITY_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>

namespace auto_aim
{
// Active high-speed qualification requires mature evidence for the current
// face/phase. Missing or reset history cannot authorize a new request.
// "eligible" also accepts a briefly retained historical verdict. Production
// checks current forecast eligibility and observation freshness separately;
// a retained verdict alone must never authorize a request.
inline bool past_forecast_quality_allows(
  bool active, bool ready, bool eligible, double bound_rad, double limit_rad)
{
  return !active || (ready && eligible && std::isfinite(bound_rad) && bound_rad >= 0.0 &&
    std::isfinite(limit_rad) && limit_rad > 0.0 && bound_rad <= limit_rad);
}

// Current visual/servo qualification only. Does not schedule or retain shots.
class HighSpeedPitchGate
{
public:
  struct Config
  {
    bool enabled = false;
    double residual_limit_rad = 0.45 * degree;
    double tracking_limit_rad = 0.8 * degree;
    double variation_limit_rad = 0.15 * degree;
    int64_t stable_ns = 30000000;
  };
  struct Input
  {
    int64_t frame_ns = 0;
    uint64_t epoch = 0;
    bool high_speed = false, context_valid = false;
    double residual_rad = NAN, tracking_error_rad = NAN;
  };
  struct Output
  {
    bool active = false, residual_ok = false, tracking_ok = false;
    bool stable = false, open = true;
    double variation_rad = NAN;
    int64_t stable_span_ns = 0;
  };

  HighSpeedPitchGate() = default;
  explicit HighSpeedPitchGate(Config config) : config_(config)
  {
    if (!std::isfinite(config.residual_limit_rad) || config.residual_limit_rad <= 0.0 ||
        !std::isfinite(config.tracking_limit_rad) || config.tracking_limit_rad <= 0.0 ||
        !std::isfinite(config.variation_limit_rad) || config.variation_limit_rad <= 0.0 ||
        config.stable_ns < 0 || config.stable_ns > 200000000)
      throw std::invalid_argument("Invalid high-speed pitch qualification");
  }

  Output update(const Input & in)
  {
    Output out;
    if (!config_.enabled || !in.high_speed) { reset(); return out; }
    out.active = true;
    out.open = false;
    if (!in.context_valid || !in.epoch || in.frame_ns <= 0 ||
        !std::isfinite(in.residual_rad) || !std::isfinite(in.tracking_error_rad)) {
      reset(); return out;
    }
    if (epoch_ != in.epoch || (last_ns_ &&
        (in.frame_ns <= last_ns_ || in.frame_ns - last_ns_ > 50000000))) reset();
    epoch_ = in.epoch;
    last_ns_ = in.frame_ns;
    out.residual_ok = std::abs(in.residual_rad) <= config_.residual_limit_rad;
    out.tracking_ok = std::abs(in.tracking_error_rad) <= config_.tracking_limit_rad;
    if (!out.residual_ok || !out.tracking_ok) { samples_.clear(); return out; }
    samples_.push_back({in.frame_ns, in.residual_rad, in.tracking_error_rad});
    // Keep one point at/before the window boundary; discarding every older
    // point would prevent a non-integral camera period ever completing dwell.
    const auto boundary = in.frame_ns - config_.stable_ns;
    while (samples_.size() > 1 && samples_[1].time <= boundary) samples_.pop_front();
    while (samples_.size() > 64) samples_.pop_front();
    double low = INFINITY, high = -INFINITY, tracking_low = INFINITY, tracking_high = -INFINITY;
    for (const auto & s : samples_) {
      low = std::min(low, s.residual); high = std::max(high, s.residual);
      tracking_low = std::min(tracking_low, s.tracking);
      tracking_high = std::max(tracking_high, s.tracking);
    }
    out.variation_rad = std::max(high - low, tracking_high - tracking_low);
    out.stable_span_ns = in.frame_ns - samples_.front().time;
    out.stable = out.stable_span_ns >= config_.stable_ns &&
      out.variation_rad <= config_.variation_limit_rad;
    out.open = out.stable;
    return out;
  }

private:
  static constexpr double degree = 3.14159265358979323846 / 180.0;
  struct Sample { int64_t time; double residual, tracking; };
  Config config_;
  uint64_t epoch_ = 0;
  int64_t last_ns_ = 0;
  std::deque<Sample> samples_;
  void reset() { samples_.clear(); epoch_ = 0; last_ns_ = 0; }
};
}  // namespace auto_aim
#endif

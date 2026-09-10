#ifndef AUTO_AIM__GYRO_PHASE_SCHEDULER_HPP
#define AUTO_AIM__GYRO_PHASE_SCHEDULER_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace auto_aim
{

// Robust event clock for a four-armor small gyro.
//
// The caller supplies the EKF body yaw relative to the vehicle-center bearing,
// folded modulo pi/2 into [-pi/4, pi/4].  Folding makes all four armor ids share
// one continuous face-center event and prevents an armor association change
// from moving the clock by a whole face.  Observed crossings update a bounded
// phase-locked loop (PLL): robust windows estimate period, phase and their live
// uncertainty while bounded alpha-beta corrections prevent outlier jumps.
class GyroPhaseScheduler
{
public:
  struct Config
  {
    double crossing_capture_rad = 20.0 * M_PI / 180.0;
    double min_face_period_s = 0.12;
    double max_face_period_s = 1.20;
    double period_ema_alpha = 0.20;
    double max_period_relative_error = 0.22;
    double max_period_step_ratio = 0.025;
    double omega_period_weight = 0.15;
    double period_residual_alpha = 0.08;
    double phase_correction_alpha = 0.15;
    double max_phase_correction_s = 0.003;
    double max_phase_residual_s = 0.090;
    double phase_uncertainty_floor_s = 0.002;
    double max_stale_periods = 2.5;
    int period_window_size = 7;
    int phase_residual_window_size = 5;
    int min_lock_updates = 4;
    int relock_after_rejections = 3;
  };

  struct Observation
  {
    bool crossing = false;
    bool period_updated = false;
    bool rejected = false;
    bool reacquired = false;
    bool locked = false;
    int periods_elapsed = 0;
    int lock_updates = 0;
    int consecutive_rejections = 0;
    double crossing_time_s = 0.0;
    double period_sample_s = 0.0;
    double face_period_s = 0.0;
    double phase_residual_s = 0.0;
    double applied_phase_correction_s = 0.0;
    double phase_uncertainty_s = std::numeric_limits<double>::infinity();
  };

  struct Solution
  {
    bool valid = false;
    bool interval_safe = false;
    std::int64_t hit_cycle = -1;
    double command_due_time_s = 0.0;
    double time_until_command_s = std::numeric_limits<double>::infinity();
    double time_until_hit_center_s = std::numeric_limits<double>::infinity();
    double face_period_s = 0.0;
    double omega_rad_s = 0.0;
    double half_hit_window_rad = 0.0;
    double worst_interval_phase_rad = std::numeric_limits<double>::infinity();
    double phase_uncertainty_rad = std::numeric_limits<double>::infinity();
  };

  GyroPhaseScheduler() = default;
  explicit GyroPhaseScheduler(const Config & config) : config_(config) {}

  void reset()
  {
    have_sample_ = false;
    have_crossing_ = false;
    period_estimate_valid_ = false;
    locked_ = false;
    previous_time_s_ = 0.0;
    previous_phase_error_rad_ = 0.0;
    last_observed_crossing_time_s_ = 0.0;
    phase_anchor_time_s_ = 0.0;
    face_period_s_ = 0.0;
    crossing_cycle_ = 0;
    lock_updates_ = 0;
    consecutive_rejections_ = 0;
    period_samples_.clear();
    phase_residuals_.clear();
    phase_uncertainty_s_ = std::numeric_limits<double>::infinity();
  }

  Observation observe(
    double sample_time_s, double folded_phase_error_rad, double ekf_omega_rad_s)
  {
    Observation out;
    if (!std::isfinite(sample_time_s) || !std::isfinite(folded_phase_error_rad)) {
      have_sample_ = false;
      return out;
    }

    folded_phase_error_rad = std::remainder(folded_phase_error_rad, M_PI / 2.0);
    if (!have_sample_) {
      have_sample_ = true;
      previous_time_s_ = sample_time_s;
      previous_phase_error_rad_ = folded_phase_error_rad;
      return out;
    }

    const double sample_dt_s = sample_time_s - previous_time_s_;
    const double previous_error = previous_phase_error_rad_;
    previous_time_s_ = sample_time_s;
    previous_phase_error_rad_ = folded_phase_error_rad;

    if (!(sample_dt_s > 0.0 && sample_dt_s <= 0.10)) return out;
    const bool rising_crossing =
      previous_error <= 0.0 && folded_phase_error_rad >= 0.0;
    const bool falling_crossing =
      previous_error >= 0.0 && folded_phase_error_rad <= 0.0;
    const bool sign_crossing = std::isfinite(ekf_omega_rad_s) &&
        std::abs(ekf_omega_rad_s) > 0.05
      ? (ekf_omega_rad_s > 0.0 ? rising_crossing : falling_crossing)
      : (rising_crossing || falling_crossing);
    if (!sign_crossing ||
        std::max(std::abs(previous_error), std::abs(folded_phase_error_rad)) >
          config_.crossing_capture_rad) {
      return out;
    }

    const double denominator = std::abs(previous_error) + std::abs(folded_phase_error_rad);
    const double fraction = denominator > 1e-9
      ? std::clamp(std::abs(previous_error) / denominator, 0.0, 1.0)
      : 0.5;
    const double crossing_time_s = sample_time_s - sample_dt_s + fraction * sample_dt_s;
    out.crossing = true;
    out.crossing_time_s = crossing_time_s;

    if (!have_crossing_) {
      have_crossing_ = true;
      last_observed_crossing_time_s_ = crossing_time_s;
      phase_anchor_time_s_ = crossing_time_s;
      crossing_cycle_ = 0;
      return out;
    }

    const double elapsed_s = crossing_time_s - last_observed_crossing_time_s_;
    double expected_period_s = face_period_s_;
    const double omega_period_s = std::isfinite(ekf_omega_rad_s) &&
        std::abs(ekf_omega_rad_s) > 0.05
      ? (M_PI / 2.0) / std::abs(ekf_omega_rad_s)
      : 0.0;
    if (!(expected_period_s > 0.0)) expected_period_s = omega_period_s;
    if (!(expected_period_s > 0.0)) expected_period_s = elapsed_s;

    int periods_elapsed = static_cast<int>(std::llround(elapsed_s / expected_period_s));
    periods_elapsed = std::clamp(periods_elapsed, 1, 4);
    const double period_sample_s = elapsed_s / static_cast<double>(periods_elapsed);
    const double relative_error = std::abs(period_sample_s - expected_period_s) /
      std::max(expected_period_s, 1e-6);

    out.periods_elapsed = periods_elapsed;
    out.period_sample_s = period_sample_s;
    if (period_sample_s < config_.min_face_period_s ||
        period_sample_s > config_.max_face_period_s ||
        relative_error > config_.max_period_relative_error) {
      out.rejected = true;
      out.consecutive_rejections = consecutive_rejections_;
      return out;
    }

    if (!period_estimate_valid_) {
      // Do not seed the PLL from a single noisy image interval.  Field data
      // contains paired long/short crossings; the already-converged EKF yaw
      // rate provides a substantially better initial period, after which the
      // robust image-event median remains authoritative.
      face_period_s_ = omega_period_s >= config_.min_face_period_s &&
          omega_period_s <= config_.max_face_period_s
        ? 0.98 * omega_period_s + 0.02 * period_sample_s
        : period_sample_s;
      phase_anchor_time_s_ = crossing_time_s;
      crossing_cycle_ += periods_elapsed;
      last_observed_crossing_time_s_ = crossing_time_s;
      period_estimate_valid_ = true;
      lock_updates_ = 1;
      consecutive_rejections_ = 0;
      push_period_sample(period_sample_s);
      phase_residuals_.clear();
      phase_uncertainty_s_ = std::max(0.0, config_.phase_uncertainty_floor_s);
      fill_observation(out, 0.0, 0.0);
      return out;
    }

    const double predicted_crossing_time_s = phase_anchor_time_s_ +
      static_cast<double>(periods_elapsed) * face_period_s_;
    const double phase_residual_s = crossing_time_s - predicted_crossing_time_s;
    if (std::abs(phase_residual_s) > config_.max_phase_residual_s) {
      // The crossing interval is physically plausible, so this is a phase-lock
      // loss rather than a spurious folded-angle wrap.  Advance the predicted
      // clock and remember this observed crossing.  The old implementation
      // left both timestamps frozen; all later samples then grew without bound
      // and the scheduler could never recover without leaving gyro mode.
      push_period_sample(period_sample_s);
      phase_anchor_time_s_ = predicted_crossing_time_s;
      crossing_cycle_ += periods_elapsed;
      last_observed_crossing_time_s_ = crossing_time_s;
      locked_ = false;
      lock_updates_ = 0;
      ++consecutive_rejections_;
      phase_uncertainty_s_ = std::abs(phase_residual_s);

      out.rejected = true;
      out.phase_residual_s = phase_residual_s;
      out.consecutive_rejections = consecutive_rejections_;

      if (consecutive_rejections_ >= std::max(1, config_.relock_after_rejections)) {
        // Re-anchor only after several mutually consistent physical crossings.
        // The image-event median is the long-term frequency reference; EKF
        // omega is retained as a light prior instead of dominating it.
        double reacquired_period_s = median_period_sample();
        if (omega_period_s >= config_.min_face_period_s &&
            omega_period_s <= config_.max_face_period_s &&
            std::abs(omega_period_s - reacquired_period_s) /
                std::max(reacquired_period_s, 1e-6) <=
              config_.max_period_relative_error) {
          const double omega_weight = std::clamp(config_.omega_period_weight, 0.0, 1.0);
          reacquired_period_s =
            (1.0 - omega_weight) * reacquired_period_s +
            omega_weight * omega_period_s;
        }
        face_period_s_ = std::clamp(
          reacquired_period_s, config_.min_face_period_s, config_.max_face_period_s);
        phase_anchor_time_s_ = crossing_time_s;
        lock_updates_ = 1;
        consecutive_rejections_ = 0;
        phase_residuals_.clear();
        phase_uncertainty_s_ = std::max(0.0, config_.phase_uncertainty_floor_s);
        out.reacquired = true;
        fill_observation(out, phase_residual_s, crossing_time_s - predicted_crossing_time_s);
      }
      return out;
    }

    push_period_sample(period_sample_s);
    push_phase_residual(phase_residual_s);
    const double robust_phase_residual_s = median_value(phase_residuals_);
    double target_period_s = median_period_sample();
    if (omega_period_s >= config_.min_face_period_s &&
        omega_period_s <= config_.max_face_period_s &&
        std::abs(omega_period_s - target_period_s) /
            std::max(target_period_s, 1e-6) <= config_.max_period_relative_error) {
      // Body-yaw rate is continuous across armor handovers, but field replay
      // showed a persistent EKF-rate bias (about 320 ms versus 327 ms per
      // face).  Keep omega as a light prior and let the robust event median
      // remove that bias before it integrates into a phase-lock loss.
      const double omega_weight = std::clamp(config_.omega_period_weight, 0.0, 1.0);
      target_period_s =
        (1.0 - omega_weight) * target_period_s + omega_weight * omega_period_s;
    }
    target_period_s += config_.period_residual_alpha *
      robust_phase_residual_s / static_cast<double>(std::max(1, periods_elapsed));
    const double max_period_step_s =
      std::max(1e-6, config_.max_period_step_ratio * face_period_s_);
    target_period_s = std::clamp(
      target_period_s, face_period_s_ - max_period_step_s,
      face_period_s_ + max_period_step_s);
    face_period_s_ = (1.0 - config_.period_ema_alpha) * face_period_s_ +
      config_.period_ema_alpha * target_period_s;

    const double requested_phase_correction_s =
      config_.phase_correction_alpha * robust_phase_residual_s;
    const double applied_phase_correction_s = std::clamp(
      requested_phase_correction_s, -config_.max_phase_correction_s,
      config_.max_phase_correction_s);
    phase_anchor_time_s_ = predicted_crossing_time_s + applied_phase_correction_s;
    const double robust_spread_s = median_absolute_deviation(
      phase_residuals_, robust_phase_residual_s);
    const double robust_standard_error_s = phase_residuals_.empty()
      ? 0.0
      : 1.4826 * robust_spread_s /
        std::sqrt(static_cast<double>(phase_residuals_.size()));
    // The persistent residual mean is part of the field-calibrated fire-delay
    // zero and must not be counted a second time as random uncertainty.  Use
    // the robust residual spread for the physical hit-width gate; abrupt phase
    // changes are still handled by the rejection/relock path above.
    phase_uncertainty_s_ = std::max(
      std::max(0.0, config_.phase_uncertainty_floor_s),
      robust_standard_error_s);
    crossing_cycle_ += periods_elapsed;
    last_observed_crossing_time_s_ = crossing_time_s;
    ++lock_updates_;
    consecutive_rejections_ = 0;
    locked_ = lock_updates_ >= std::max(1, config_.min_lock_updates);
    fill_observation(out, phase_residual_s, applied_phase_correction_s);
    return out;
  }

  Solution solve(
    double now_s,
    double nominal_fire_delay_s,
    double min_fire_delay_s,
    double max_fire_delay_s,
    double flight_time_s,
    double flight_time_uncertainty_s,
    double armor_radius_m,
    double armor_width_m,
    double armor_edge_margin_m,
    double model_phase_uncertainty_rad) const
  {
    Solution out;
    if (!locked_ || !std::isfinite(now_s) ||
        !std::isfinite(nominal_fire_delay_s) || !std::isfinite(flight_time_s) ||
        !(face_period_s_ > 0.0) || !std::isfinite(armor_radius_m) ||
        armor_radius_m <= 0.0 || !std::isfinite(armor_width_m) ||
        now_s - last_observed_crossing_time_s_ > config_.max_stale_periods * face_period_s_) {
      return out;
    }

    const double nominal_total_s = nominal_fire_delay_s + flight_time_s;
    std::int64_t steps = static_cast<std::int64_t>(std::ceil(
      (now_s + nominal_total_s - phase_anchor_time_s_) / face_period_s_ - 1e-9));
    steps = std::max<std::int64_t>(1, steps);
    const double hit_center_time_s = phase_anchor_time_s_ +
      static_cast<double>(steps) * face_period_s_;

    out.valid = true;
    out.face_period_s = face_period_s_;
    out.omega_rad_s = (M_PI / 2.0) / face_period_s_;
    out.hit_cycle = crossing_cycle_ + steps;
    out.command_due_time_s = hit_center_time_s - nominal_total_s;
    out.time_until_command_s = out.command_due_time_s - now_s;
    out.time_until_hit_center_s = hit_center_time_s - now_s;

    const double usable_half_width_m = std::max(
      0.0, armor_width_m * 0.5 - std::max(0.0, armor_edge_margin_m));
    out.half_hit_window_rad = std::atan2(
      usable_half_width_m, std::max(armor_radius_m, 1e-6));

    const double requested_min_fire_delay_s = min_fire_delay_s;
    min_fire_delay_s = std::min(requested_min_fire_delay_s, max_fire_delay_s);
    max_fire_delay_s = std::max(requested_min_fire_delay_s, max_fire_delay_s);
    const double earliest_total_s = min_fire_delay_s +
      std::max(0.0, flight_time_s - std::max(0.0, flight_time_uncertainty_s));
    const double latest_total_s = max_fire_delay_s + flight_time_s +
      std::max(0.0, flight_time_uncertainty_s);
    const double early_phase = out.omega_rad_s * (earliest_total_s - nominal_total_s);
    const double late_phase = out.omega_rad_s * (latest_total_s - nominal_total_s);
    out.worst_interval_phase_rad =
      std::max(std::abs(early_phase), std::abs(late_phase)) +
      std::max(0.0, model_phase_uncertainty_rad) +
      out.omega_rad_s * std::max(0.0, phase_uncertainty_s_);
    out.phase_uncertainty_rad =
      out.omega_rad_s * std::max(0.0, phase_uncertainty_s_);
    out.interval_safe = out.worst_interval_phase_rad <= out.half_hit_window_rad;
    return out;
  }

  bool period_valid() const { return period_estimate_valid_; }
  bool locked() const { return locked_; }
  int lock_updates() const { return lock_updates_; }
  double face_period_s() const { return face_period_s_; }
  double phase_anchor_time_s() const { return phase_anchor_time_s_; }
  double phase_uncertainty_s() const { return phase_uncertainty_s_; }
  std::int64_t crossing_cycle() const { return crossing_cycle_; }

private:
  void push_period_sample(double sample_s)
  {
    period_samples_.push_back(sample_s);
    const auto max_size = static_cast<std::size_t>(
      std::max(3, config_.period_window_size));
    while (period_samples_.size() > max_size) period_samples_.pop_front();
  }

  double median_period_sample() const
  {
    return median_value(period_samples_);
  }

  template<typename Container>
  static double median_value(const Container & values)
  {
    if (values.empty()) return 0.0;
    std::vector<double> sorted(values.begin(), values.end());
    std::sort(sorted.begin(), sorted.end());
    const std::size_t middle = sorted.size() / 2;
    if (sorted.size() % 2 == 0) {
      return 0.5 * (sorted[middle - 1] + sorted[middle]);
    }
    return sorted[middle];
  }

  void push_phase_residual(double residual_s)
  {
    phase_residuals_.push_back(residual_s);
    const auto max_size = static_cast<std::size_t>(
      std::max(3, config_.phase_residual_window_size));
    while (phase_residuals_.size() > max_size) phase_residuals_.pop_front();
  }

  template<typename Container>
  static double median_absolute_deviation(const Container & values, double median)
  {
    if (values.empty()) return 0.0;
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (const double value : values) deviations.push_back(std::abs(value - median));
    return median_value(deviations);
  }

  void fill_observation(
    Observation & out, double phase_residual_s, double applied_phase_correction_s) const
  {
    out.period_updated = true;
    out.face_period_s = face_period_s_;
    out.phase_residual_s = phase_residual_s;
    out.applied_phase_correction_s = applied_phase_correction_s;
    out.phase_uncertainty_s = phase_uncertainty_s_;
    out.lock_updates = lock_updates_;
    out.consecutive_rejections = consecutive_rejections_;
    out.locked = locked_;
  }

  Config config_{};
  bool have_sample_ = false;
  bool have_crossing_ = false;
  bool period_estimate_valid_ = false;
  bool locked_ = false;
  double previous_time_s_ = 0.0;
  double previous_phase_error_rad_ = 0.0;
  double last_observed_crossing_time_s_ = 0.0;
  double phase_anchor_time_s_ = 0.0;
  double face_period_s_ = 0.0;
  std::int64_t crossing_cycle_ = 0;
  int lock_updates_ = 0;
  int consecutive_rejections_ = 0;
  std::deque<double> period_samples_;
  std::deque<double> phase_residuals_;
  double phase_uncertainty_s_ = std::numeric_limits<double>::infinity();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__GYRO_PHASE_SCHEDULER_HPP

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
    double crossing_rearm_rad = 12.0 * M_PI / 180.0;
    double min_crossing_separation_ratio = 0.55;
    double min_face_period_s = 0.12;
    double max_face_period_s = 1.20;
    double period_ema_alpha = 0.20;
    double max_period_relative_error = 0.22;
    double max_period_step_ratio = 0.025;
    double omega_period_weight = 0.15;
    double omega_alias_guard_relative_error = 0.35;
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
    bool stale_reseeded = false;
    bool locked = false;
    bool omega_alias_guarded = false;
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
    // Explicit fixed-centre bench: geometry and the locked signed rate share
    // one reference for both the send time and the expected send angle.
    bool geometry_aligned = false;
    double reference_phase_rad = std::numeric_limits<double>::quiet_NaN();
    double reference_time_s = std::numeric_limits<double>::quiet_NaN();
    double signed_omega_rad_s = std::numeric_limits<double>::quiet_NaN();
    double total_lead_s = std::numeric_limits<double>::quiet_NaN();
    double command_phase_rad = std::numeric_limits<double>::quiet_NaN();

    double phase_at(double time_s) const
    {
      return geometry_aligned && std::isfinite(time_s)
        ? std::remainder(reference_phase_rad + signed_omega_rad_s *
            (time_s - reference_time_s), M_PI / 2.0)
        : std::numeric_limits<double>::quiet_NaN();
    }

    double command_phase_error_at(double time_s) const
    {
      return std::remainder(phase_at(time_s) - command_phase_rad, M_PI / 2.0);
    }

    bool command_phase_in_window_at(double time_s, double tolerance_rad, bool after_only) const
    {
      const double error = command_phase_error_at(time_s);
      if (!std::isfinite(error) || !std::isfinite(tolerance_rad) || tolerance_rad <= 0.0 ||
          !std::isfinite(signed_omega_rad_s) || std::abs(signed_omega_rad_s) < 0.05)
        return false;
      const double progress = std::copysign(1.0, signed_omega_rad_s) * error;
      return std::abs(error) <= tolerance_rad && (!after_only || progress >= 0.0);
    }
  };

  // Preserve the PLL lock/uncertainty gates. Align its next event with the
  // observed body phase; a separate EKF rate must not pick a different angle.
  static Solution align_to_geometry(
    Solution out, double now_s, double sample_time_s,
    double body_phase_relative_to_center_rad, double direction_rate_rad_s)
  {
    if (!out.valid || !std::isfinite(now_s) || !std::isfinite(sample_time_s) ||
        sample_time_s > now_s || !std::isfinite(body_phase_relative_to_center_rad) ||
        !std::isfinite(direction_rate_rad_s) || std::abs(direction_rate_rad_s) < 0.05 ||
        !std::isfinite(out.omega_rad_s) || out.omega_rad_s <= 0.0 ||
        !std::isfinite(out.face_period_s) || out.face_period_s <= 0.0 ||
        !std::isfinite(out.command_due_time_s) ||
        !std::isfinite(out.time_until_hit_center_s)) {
      out.valid = false;
      return out;
    }
    const double lead_s = out.time_until_hit_center_s - out.time_until_command_s;
    if (!std::isfinite(lead_s) || lead_s <= 0.0) {
      out.valid = false;
      return out;
    }
    const double omega = std::copysign(out.omega_rad_s, direction_rate_rad_s);
    const double old_hit_time_s = now_s + out.time_until_hit_center_s;
    const double phase_at_old_hit = std::remainder(
      body_phase_relative_to_center_rad + omega * (old_hit_time_s - sample_time_s),
      M_PI / 2.0);
    double due_s = out.command_due_time_s - phase_at_old_hit / omega;
    const auto shift = static_cast<std::int64_t>(std::ceil(
      (now_s - due_s) / out.face_period_s - 1e-9));
    due_s += static_cast<double>(shift) * out.face_period_s;
    if (out.hit_cycle + shift < 0 || due_s < now_s - 1e-7) {
      out.valid = false;
      return out;
    }
    out.hit_cycle += shift;
    out.command_due_time_s = due_s;
    out.time_until_command_s = std::max(0.0, due_s - now_s);
    out.time_until_hit_center_s = out.time_until_command_s + lead_s;
    out.geometry_aligned = true;
    out.reference_phase_rad = body_phase_relative_to_center_rad;
    out.reference_time_s = sample_time_s;
    out.signed_omega_rad_s = omega;
    out.total_lead_s = lead_s;
    out.command_phase_rad = std::remainder(-omega * lead_s, M_PI / 2.0);
    return out;
  }

  GyroPhaseScheduler() = default;
  explicit GyroPhaseScheduler(const Config & config) : config_(config) {}

  void reset()
  {
    have_sample_ = false;
    have_crossing_ = false;
    crossing_armed_ = true;
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

    // A noisy folded phase can bounce around zero for several frames. Accept
    // only one crossing until the phase has moved well outside a Schmitt band.
    // This rejects the 17--38 ms duplicate "periods" seen in the field log
    // without treating them as a real PLL phase loss.
    if (!crossing_armed_) {
      if (std::abs(folded_phase_error_rad) >=
          std::max(0.0, config_.crossing_rearm_rad)) {
        crossing_armed_ = true;
      }
      // Even a sparse replay sample is allowed to re-arm the Schmitt trigger,
      // but a discontinuous dt is never used to interpolate a crossing.
      if (!(sample_dt_s > 0.0 && sample_dt_s <= 0.10)) return out;
      if (!crossing_armed_) return out;
    }
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
    crossing_armed_ = false;
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

    const double min_crossing_separation_s = std::clamp(
      config_.min_crossing_separation_ratio, 0.0, 0.95) * expected_period_s;
    if (elapsed_s < min_crossing_separation_s) {
      // Keep the last physical crossing and lock state untouched. Marking a
      // duplicate as a phase residual rejection used to start an unnecessary
      // four-crossing relock sequence and caused multi-second fire gaps.
      out.crossing = false;
      return out;
    }

    // The solve freshness limit must also end this acquisition epoch. Keeping
    // an ever-growing rejected gap with the four-period cap below otherwise
    // rejects all later observations. A new seed is never a fire permission:
    // require the complete fresh-lock sequence and preserve cycle monotonicity.
    if (expected_period_s >= config_.min_face_period_s &&
        expected_period_s <= config_.max_face_period_s &&
        elapsed_s > config_.max_stale_periods * expected_period_s) {
      const double seed_period_s =
        omega_period_s >= config_.min_face_period_s &&
          omega_period_s <= config_.max_face_period_s
        ? omega_period_s : expected_period_s;
      face_period_s_ = seed_period_s;
      phase_anchor_time_s_ = crossing_time_s;
      last_observed_crossing_time_s_ = crossing_time_s;
      ++crossing_cycle_;
      period_estimate_valid_ = true;
      locked_ = false;
      lock_updates_ = 0;
      consecutive_rejections_ = 0;
      period_samples_.clear();
      period_samples_.push_back(seed_period_s);
      phase_residuals_.clear();
      phase_uncertainty_s_ = std::max(0.0, config_.phase_uncertainty_floor_s);
      out.period_sample_s = seed_period_s;
      out.reacquired = true;
      out.stale_reseeded = true;
      fill_observation(out, 0.0, 0.0);
      return out;
    }

    // A stale PLL period can turn one slow physical interval into three fast
    // intervals (the 2026-09-11 field failure).  When the independently
    // estimated EKF rate strongly disagrees with the PLL, use it only to
    // disambiguate the integer number of elapsed face periods.  The robust
    // crossing median remains the frequency estimator after that choice.
    double period_count_reference_s = expected_period_s;
    if (omega_period_s >= config_.min_face_period_s &&
        omega_period_s <= config_.max_face_period_s &&
        face_period_s_ > 0.0 &&
        std::abs(omega_period_s - face_period_s_) /
            std::max(omega_period_s, 1e-6) >
          std::max(0.0, config_.omega_alias_guard_relative_error)) {
      period_count_reference_s = omega_period_s;
      out.omega_alias_guarded = true;
    }

    int periods_elapsed = static_cast<int>(
      std::llround(elapsed_s / period_count_reference_s));
    periods_elapsed = std::clamp(periods_elapsed, 1, 4);
    const double period_sample_s = elapsed_s / static_cast<double>(periods_elapsed);
    const double relative_error =
      std::abs(period_sample_s - period_count_reference_s) /
      std::max(period_count_reference_s, 1e-6);

    out.periods_elapsed = periods_elapsed;
    out.period_sample_s = period_sample_s;
    if (out.omega_alias_guarded) {
      // The old frequency hypothesis is no longer credible. Re-seed from the
      // independent rate immediately and require a fresh lock sequence; mixing
      // old fast samples with new slow samples is what created the 3x alias.
      face_period_s_ = omega_period_s;
      phase_anchor_time_s_ = crossing_time_s;
      crossing_cycle_ += periods_elapsed;
      last_observed_crossing_time_s_ = crossing_time_s;
      period_estimate_valid_ = true;
      locked_ = false;
      lock_updates_ = 1;
      consecutive_rejections_ = 0;
      period_samples_.clear();
      period_samples_.push_back(omega_period_s);
      phase_residuals_.clear();
      phase_uncertainty_s_ = std::max(0.0, config_.phase_uncertainty_floor_s);
      out.reacquired = true;
      fill_observation(out, 0.0, 0.0);
      return out;
    }
    if (period_sample_s < config_.min_face_period_s ||
        period_sample_s > config_.max_face_period_s ||
        relative_error > config_.max_period_relative_error) {
      // This is not a near-zero duplicate (those were suppressed above), so
      // temporarily veto firing until a consistent physical crossing arrives.
      // Preserve the mature anchor and lock count so one bad PnP phase sample
      // costs at most one event instead of forcing a full reacquisition.
      ++consecutive_rejections_;
      phase_uncertainty_s_ = std::abs(
        elapsed_s - static_cast<double>(periods_elapsed) * expected_period_s);
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
      // One bad monocular phase observation must not destroy a mature clock.
      // Keep the last accepted anchor so the next physical crossing can span
      // two periods and restore normal operation immediately. Several
      // consecutive, mutually consistent residuals still trigger a safe
      // reacquisition for a genuine phase discontinuity.
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
        crossing_cycle_ += periods_elapsed;
        last_observed_crossing_time_s_ = crossing_time_s;
        locked_ = false;
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
    double model_phase_uncertainty_rad,
    double fixed_planning_omega_rad_s = 0.0) const
  {
    Solution out;
    if (!locked_ || consecutive_rejections_ > 0 || !std::isfinite(now_s) ||
        !std::isfinite(nominal_fire_delay_s) || !std::isfinite(flight_time_s) ||
        !(face_period_s_ > 0.0) || !std::isfinite(armor_radius_m) ||
        armor_radius_m <= 0.0 || !std::isfinite(armor_width_m) ||
        !std::isfinite(fixed_planning_omega_rad_s) || fixed_planning_omega_rad_s < 0.0 ||
        now_s - last_observed_crossing_time_s_ > config_.max_stale_periods * face_period_s_) {
      return out;
    }

    // An explicit bench setpoint changes planning only. The observed PLL
    // period, lock, rejection and staleness checks remain independent.
    const double planning_period_s = fixed_planning_omega_rad_s > 0.0
      ? (M_PI / 2.0) / fixed_planning_omega_rad_s : face_period_s_;
    const double nominal_total_s = nominal_fire_delay_s + flight_time_s;
    std::int64_t steps = static_cast<std::int64_t>(std::ceil(
      (now_s + nominal_total_s - phase_anchor_time_s_) / planning_period_s - 1e-9));
    steps = std::max<std::int64_t>(1, steps);
    const double hit_center_time_s = phase_anchor_time_s_ +
      static_cast<double>(steps) * planning_period_s;

    out.valid = true;
    out.face_period_s = planning_period_s;
    out.omega_rad_s = (M_PI / 2.0) / planning_period_s;
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
  bool crossing_armed_ = true;
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

// Fixed-centre periodic bench policy. The configured speed seeds the clock;
// mature, robust visual crossings slowly calibrate its period and phase. This
// is not a second per-frame TX-window veto. Missed slots remain discarded and
// only a successful write consumes a slot. AUTO/pose/target gates stay external.
class FixedCadenceTrigger
{
public:
  void reset()
  {
    anchored_ = false;
    last_sample_s_ = NAN;
    visual_phase_error_rad_ = NAN;
    period_s_ = 0.0;
    calibration_updates_ = 0;
    large_residual_updates_ = 0;
    last_phase_adjustment_s_ = 0.0;
    have_phase_sample_ = false;
    crossing_armed_ = true;
    crossings_.clear();
    ++cycle_;
  }

  GyroPhaseScheduler::Solution observe(
    double now_s, double sample_s, double phase_rad, double signed_omega,
    double total_lead_s, double tolerance_rad, double uncertainty_rad,
    double max_age_s, double offset_s, int every_n_faces,
    double max_wait_s, double late_tolerance_s, double max_observation_gap_s = 0.25)
  {
    GyroPhaseScheduler::Solution out;
    if (!std::isfinite(now_s) || !std::isfinite(sample_s) ||
        !std::isfinite(phase_rad) || !std::isfinite(signed_omega) ||
        !std::isfinite(total_lead_s) || total_lead_s <= 0.0 ||
        !std::isfinite(tolerance_rad) || tolerance_rad <= 0.0 || tolerance_rad >= M_PI/4.0 ||
        !std::isfinite(uncertainty_rad) || uncertainty_rad < 0.0 ||
        !std::isfinite(max_age_s) || max_age_s <= 0.0 ||
        !std::isfinite(offset_s) || !std::isfinite(max_wait_s) || max_wait_s < 0.0 ||
        !std::isfinite(late_tolerance_s) || late_tolerance_s < 0.0 ||
        !std::isfinite(max_observation_gap_s) || max_observation_gap_s <= 0.0 ||
        sample_s > now_s || now_s - sample_s > max_age_s ||
        std::abs(signed_omega) < .05 || every_n_faces < 1) {
      reset(); return out;
    }
    if (anchored_ && (signed_omega * nominal_omega_ <= 0.0 ||
        std::abs(signed_omega - nominal_omega_) > 1e-6 ||
        every_n_faces != every_n_faces_ || std::abs(offset_s - configured_offset_s_) > 1e-9 ||
        (std::isfinite(last_sample_s_) &&
          (sample_s < last_sample_s_ || sample_s - last_sample_s_ > max_observation_gap_s)))) reset();
    last_sample_s_ = sample_s;
    const double nominal_face_period_s = (M_PI/2.0) / std::abs(signed_omega);
    const double folded = std::remainder(phase_rad, M_PI/2.0);
    if (!anchored_) {
      // Capture-time phase, rather than processing-time phase, defines t0.
      // Only a physically near-centre observation may start the clock.
      if (std::abs(folded) > tolerance_rad) return out;
      nominal_omega_ = omega_ = signed_omega;
      every_n_faces_ = every_n_faces;
      configured_offset_s_ = offset_s;
      period_s_ = every_n_faces * nominal_face_period_s;
      offset_in_face_s_ = offset_modulo(offset_s, nominal_face_period_s);
      command_phase_ = std::remainder(omega_ * offset_in_face_s_, M_PI/2.0);
      anchor_s_ = sample_s - folded / omega_;
      next_due_s_ = anchor_s_ + offset_in_face_s_;
      anchored_ = true;
      ++cycle_;
    }
    observe_crossing(sample_s, folded);
    const double face_period_s = period_s_ / every_n_faces_;
    // No overdue burst, including after a long processing stall.
    if (next_due_s_ < now_s - late_tolerance_s) {
      const auto skip = static_cast<std::int64_t>(std::ceil(
        (now_s - late_tolerance_s - next_due_s_) / period_s_));
      next_due_s_ += skip * period_s_;
      cycle_ += skip;
    }
    out.geometry_aligned = true;
    // Mature crossings calibrate the clock; individual noisy image phases do
    // not veto a slot. Report the remaining error against that calibrated clock.
    visual_phase_error_rad_ = std::remainder(
      phase_rad - omega_ * (sample_s-anchor_s_), M_PI/2.0);
    out.reference_phase_rad = 0.0;
    out.reference_time_s = anchor_s_;
    out.signed_omega_rad_s = omega_;
    out.omega_rad_s = std::abs(omega_);
    out.face_period_s = face_period_s;
    out.total_lead_s = total_lead_s; // Diagnostic prediction only; offset owns TX timing.
    out.command_phase_rad = command_phase_;
    out.command_due_time_s = next_due_s_;
    out.time_until_command_s = std::max(0.0, next_due_s_ - now_s);
    out.time_until_hit_center_s = out.time_until_command_s + total_lead_s;
    out.hit_cycle = cycle_;
    out.half_hit_window_rad = tolerance_rad;
    out.phase_uncertainty_rad = uncertainty_rad;
    out.worst_interval_phase_rad = NAN;
    out.valid = next_due_s_ - now_s <= max_wait_s &&
      now_s - next_due_s_ <= late_tolerance_s;
    out.interval_safe = out.valid;
    return out;
  }

  void mark_fired() { if (anchored_) { next_due_s_ += period_s_; ++cycle_; } }
  double visual_phase_error_rad() const { return anchored_ ? visual_phase_error_rad_ : NAN; }
  double command_interval_s() const { return anchored_ ? period_s_ : 0.0; }
  std::uint64_t calibration_updates() const { return calibration_updates_; }
  std::uint64_t large_residual_updates() const { return large_residual_updates_; }
  double last_phase_adjustment_s() const { return last_phase_adjustment_s_; }

private:
  struct CrossingSample
  {
    std::int64_t index;
    double time_s;
  };

  static double median(std::vector<double> values)
  {
    std::sort(values.begin(), values.end());
    const auto middle = values.size()/2;
    return values.size()%2 ? values[middle] : .5*(values[middle-1]+values[middle]);
  }

  void observe_crossing(double sample_s, double folded)
  {
    const double previous_s = previous_phase_time_s_;
    const double previous_phase = previous_phase_rad_;
    previous_phase_time_s_ = sample_s;
    previous_phase_rad_ = folded;
    if (!have_phase_sample_) { have_phase_sample_ = true; return; }
    const double dt = sample_s - previous_s;
    if (!crossing_armed_ && std::abs(folded) >= 12.0*M_PI/180.0)
      crossing_armed_ = true;
    if (!crossing_armed_ || dt <= 0.0 || dt > .10) return;
    const bool crossing = nominal_omega_ > 0.0
      ? previous_phase <= 0.0 && folded >= 0.0
      : previous_phase >= 0.0 && folded <= 0.0;
    if (!crossing || std::max(std::abs(previous_phase),std::abs(folded)) > 20.0*M_PI/180.0)
      return;
    const double denominator = std::abs(previous_phase)+std::abs(folded);
    if (denominator <= 1e-12) return;
    crossing_armed_ = false;
    const double cross_s = previous_s + dt*std::abs(previous_phase)/denominator;
    const double face_s = period_s_/every_n_faces_;
    if (!crossings_.empty()) {
      const double gap_s = cross_s - crossings_.back().time_s;
      if (gap_s < .55*face_s) return; // noisy duplicate, not another armor face
      if (gap_s > 2.5*face_s) crossings_.clear();
    }
    const auto index = crossings_.empty() ? std::int64_t{0}
      : crossings_.back().index + std::max<std::int64_t>(1,
          std::llround((cross_s-crossings_.back().time_s)/face_s));
    crossings_.push_back({index,cross_s});
    while (crossings_.size()>24) crossings_.pop_front();
    if (crossings_.size()<8) return;

    // Pairwise slopes spanning at least four passages suppress the paired
    // long/short intervals in field data. The configured speed only seeds the
    // count; it is not blended back as an independent frequency measurement.
    std::vector<double> slopes;
    for (std::size_t i=0;i<crossings_.size();++i)
      for (std::size_t j=i+1;j<crossings_.size();++j) {
        const auto count=crossings_[j].index-crossings_[i].index;
        if (count>=4) slopes.push_back(
          (crossings_[j].time_s-crossings_[i].time_s)/static_cast<double>(count));
      }
    if (slopes.empty()) return;
    const double fitted_period_s = median(slopes);
    std::vector<double> anchors;
    for (const auto& point:crossings_)
      anchors.push_back(point.time_s+(index-point.index)*fitted_period_s);
    calibrate(fitted_period_s,median(anchors));
  }

  static double offset_modulo(double offset, double face_period)
  {
    double value = std::fmod(offset, face_period);
    return value < 0.0 ? value + face_period : value;
  }

  void calibrate(double observed_face_period_s, double observed_anchor_s)
  {
    const double old_face_period_s = period_s_ / every_n_faces_;
    const double nominal_face_period_s = (M_PI/2.0) / std::abs(nominal_omega_);
    if (!std::isfinite(observed_face_period_s) || !std::isfinite(observed_anchor_s) ||
        observed_face_period_s < 0.9 * nominal_face_period_s ||
        observed_face_period_s > 1.1 * nominal_face_period_s) return;

    // Pivot at the recent crossing, rather than scaling the entire elapsed time
    // from t0. Otherwise a small period update makes a large late-session jump.
    const double crossing_index = std::round(
      (observed_anchor_s - anchor_s_) / old_face_period_s);
    const double predicted_anchor_s = anchor_s_ + crossing_index * old_face_period_s;
    const double residual_s = observed_anchor_s - predicted_anchor_s;
    // The crossing sequence has already earned a robust multi-passage fit.
    // A large phase residual must not discard its period update too: with a
    // correct period but a >quarter-face phase error that policy never
    // recovers, and an imperfect period keeps firing on a drifting clock.
    // Associate the nearest face, retain the nominal period bounds above, and
    // slew phase by at most 3 ms rather than snapping to the new observation.
    if (std::abs(residual_s) > 0.25 * old_face_period_s) ++large_residual_updates_;
    const double faces_to_slot = std::round(
      (next_due_s_ - offset_in_face_s_ - predicted_anchor_s) / old_face_period_s);

    const double new_face_period_s = std::clamp(
      observed_face_period_s, old_face_period_s * 0.995, old_face_period_s * 1.005);
    last_phase_adjustment_s_ = std::clamp(0.25 * residual_s, -0.003, 0.003);
    anchor_s_ = predicted_anchor_s + last_phase_adjustment_s_;
    period_s_ = every_n_faces_ * new_face_period_s;
    omega_ = std::copysign((M_PI/2.0) / new_face_period_s, nominal_omega_);
    offset_in_face_s_ = offset_modulo(configured_offset_s_, new_face_period_s);
    command_phase_ = std::remainder(omega_ * offset_in_face_s_, M_PI/2.0);
    next_due_s_ = anchor_s_ + faces_to_slot * new_face_period_s + offset_in_face_s_;
    ++calibration_updates_;
  }

  bool anchored_ = false;
  double nominal_omega_ = 0.0, omega_ = 0.0, period_s_ = 0.0, next_due_s_ = 0.0;
  int every_n_faces_ = 1;
  double configured_offset_s_ = 0.0, offset_in_face_s_ = 0.0;
  double command_phase_ = 0.0, last_sample_s_ = NAN;
  double anchor_s_ = 0.0, visual_phase_error_rad_ = NAN;
  double last_phase_adjustment_s_ = 0.0;
  std::uint64_t calibration_updates_ = 0;
  std::uint64_t large_residual_updates_ = 0;
  bool have_phase_sample_ = false, crossing_armed_ = true;
  double previous_phase_time_s_ = 0.0, previous_phase_rad_ = 0.0;
  std::deque<CrossingSample> crossings_;
  std::int64_t cycle_ = 0;
};

class PhaseWindowTrigger
{
public:
  void reset()
  {
    in_window_ = false;
    consumed_ = false;
    direction_ = 0;
    ++cycle_;
  }

  GyroPhaseScheduler::Solution observe(
    double now_s, double sample_time_s, double relative_phase_rad,
    double signed_omega_rad_s, double total_lead_s, double tolerance_rad,
    double phase_uncertainty_rad, double max_image_age_s, bool after_only = false)
  {
    GyroPhaseScheduler::Solution out;
    if (!std::isfinite(now_s) || !std::isfinite(sample_time_s) ||
        sample_time_s > now_s || now_s - sample_time_s > max_image_age_s ||
        !std::isfinite(max_image_age_s) || max_image_age_s <= 0.0 ||
        !std::isfinite(relative_phase_rad) || !std::isfinite(signed_omega_rad_s) ||
        std::abs(signed_omega_rad_s) < 0.05 ||
        !std::isfinite(total_lead_s) || total_lead_s <= 0.0 ||
        !std::isfinite(tolerance_rad) || tolerance_rad <= 0.0 || tolerance_rad >= M_PI/4.0 ||
        !std::isfinite(phase_uncertainty_rad) || phase_uncertainty_rad < 0.0) {
      reset();
      return out;
    }
    const int direction = signed_omega_rad_s > 0.0 ? 1 : -1;
    if (direction_ != 0 && direction != direction_) reset();
    direction_ = direction;
    out.geometry_aligned = true;
    out.reference_phase_rad = relative_phase_rad;
    out.reference_time_s = sample_time_s;
    out.signed_omega_rad_s = signed_omega_rad_s;
    out.omega_rad_s = std::abs(signed_omega_rad_s);
    out.face_period_s = (M_PI/2.0) / out.omega_rad_s;
    out.total_lead_s = total_lead_s;
    out.command_phase_rad = std::remainder(-signed_omega_rad_s * total_lead_s, M_PI/2.0);
    out.command_due_time_s = now_s;
    out.time_until_command_s = 0.0;
    out.time_until_hit_center_s = total_lead_s;
    out.half_hit_window_rad = tolerance_rad;
    out.phase_uncertainty_rad = phase_uncertainty_rad;
    // The mechanical min/max interval is not a veto in this explicit policy.
    out.worst_interval_phase_rad = std::numeric_limits<double>::quiet_NaN();
    const double error = std::abs(out.command_phase_error_at(now_s));
    if (after_only) {
      const double progress = direction * out.command_phase_error_at(now_s);
      // An early image may prepare a request, but the caller must wait until
      // this due instant and recheck the one-sided window before transmitting.
      out.time_until_command_s = std::max(0.0, -progress / out.omega_rad_s);
      out.command_due_time_s = now_s + out.time_until_command_s;
      out.time_until_hit_center_s = out.time_until_command_s + total_lead_s;
    }
    if (error > std::min(M_PI/4.0, 2.0 * tolerance_rad)) {
      in_window_ = false;
      consumed_ = false;
    }
    if (error <= tolerance_rad && !in_window_) {
      in_window_ = true;
      ++cycle_;
    }
    out.hit_cycle = cycle_;
    out.valid = error <= tolerance_rad && !consumed_;
    out.interval_safe = out.valid;
    return out;
  }

  void mark_fired() { if (in_window_) consumed_ = true; }

private:
  std::int64_t cycle_ = 0;
  int direction_ = 0;
  bool in_window_ = false;
  bool consumed_ = false;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__GYRO_PHASE_SCHEDULER_HPP

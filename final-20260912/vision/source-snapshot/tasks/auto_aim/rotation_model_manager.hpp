#ifndef AUTO_AIM__ROTATION_MODEL_MANAGER_HPP
#define AUTO_AIM__ROTATION_MODEL_MANAGER_HPP

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_aim
{

// Trust-aware selector for the three prediction regimes used by the planner.
// It deliberately does not estimate pose: its job is to decide which existing
// estimator may drive the gimbal/fire policy, and when a phase clock is stale.
class RotationModelManager
{
public:
  enum class Mode { armor = 0, vehicle = 1, high_speed_phase = 2 };

  struct Config
  {
    double vehicle_enter_rad_s = 0.8;
    double vehicle_exit_rad_s = 0.5;
    double high_speed_enter_rad_s = 3.2;
    double high_speed_exit_rad_s = 2.5;
    double mode_dwell_s = 0.15;
    double tracker_dropout_hold_s = 0.0;
    double rate_filter_tau_s = 0.0;
    double max_rate_disagreement_rad_s = 1.2;
    double speed_step_reset_rad_s = 1.2;
    int rate_agreement_updates = 2;
    int rate_disagreement_updates = 3;
  };

  struct Result
  {
    Mode mode = Mode::armor;
    bool mode_changed = false;
    bool phase_reset = false;
    bool tracker_trusted = false;
    bool rate_consistent = false;
    bool phase_fire_trusted = false;
    double rate_disagreement_rad_s = std::numeric_limits<double>::infinity();
  };

  RotationModelManager() = default;
  explicit RotationModelManager(const Config & config) : config_(config) {}

  void reset()
  {
    mode_ = Mode::armor;
    candidate_mode_ = Mode::armor;
    candidate_since_s_ = 0.0;
    have_candidate_ = false;
    have_last_rate_ = false;
    last_abs_ekf_rate_ = 0.0;
    have_filtered_rate_ = false;
    filtered_abs_rate_ = 0.0;
    last_update_s_ = 0.0;
    agreement_updates_ = 0;
    disagreement_updates_ = 0;
    have_last_trusted_update_ = false;
    last_trusted_update_s_ = 0.0;
  }

  Result update(
    double now_s, bool tracker_converged, double ekf_omega_rad_s,
    bool phase_rate_valid = false,
    double phase_omega_rad_s = std::numeric_limits<double>::quiet_NaN())
  {
    Result out;
    out.tracker_trusted = tracker_converged && std::isfinite(now_s) &&
      std::isfinite(ekf_omega_rad_s);
    if (!out.tracker_trusted) {
      // A detector association hand-over can make convergence false for only a
      // few frames.  Keep the selected motion regime through that gap (fire is
      // still inhibited by tracker_trusted=false) instead of immediately
      // collapsing high-speed -> armor and rebuilding the model on every face.
      if (have_last_trusted_update_ && std::isfinite(now_s) &&
          now_s - last_trusted_update_s_ <=
            std::max(0.0, config_.tracker_dropout_hold_s)) {
        out.mode = mode_;
        return out;
      }
      out.phase_reset = mode_ == Mode::high_speed_phase;
      reset();
      out.mode = mode_;
      return out;
    }
    have_last_trusted_update_ = true;
    last_trusted_update_s_ = now_s;

    const double raw_abs_rate = std::abs(ekf_omega_rad_s);
    if (!have_filtered_rate_) {
      filtered_abs_rate_ = raw_abs_rate;
      have_filtered_rate_ = true;
    } else {
      const double dt = std::clamp(now_s - last_update_s_, 0.0, 0.25);
      const double tau = std::max(0.0, config_.rate_filter_tau_s);
      const double alpha = tau > 0.0 ? 1.0 - std::exp(-dt / tau) : 1.0;
      filtered_abs_rate_ += alpha * (raw_abs_rate - filtered_abs_rate_);
    }
    last_update_s_ = now_s;
    const double abs_rate = filtered_abs_rate_;
    const Mode desired = desired_mode(abs_rate);
    if (!have_candidate_ || desired != candidate_mode_) {
      candidate_mode_ = desired;
      candidate_since_s_ = now_s;
      have_candidate_ = true;
    }

    if (desired != mode_ &&
        now_s - candidate_since_s_ >= std::max(0.0, config_.mode_dwell_s)) {
      const Mode old_mode = mode_;
      mode_ = desired;
      out.mode_changed = true;
      out.phase_reset = old_mode == Mode::high_speed_phase ||
        mode_ == Mode::high_speed_phase;
      agreement_updates_ = 0;
      disagreement_updates_ = 0;
    }

    if (mode_ == Mode::high_speed_phase && !phase_rate_valid && have_last_rate_ &&
        std::abs(abs_rate - last_abs_ekf_rate_) >=
          std::max(0.0, config_.speed_step_reset_rad_s)) {
      // With a mature physical crossing clock, one-frame ESKF speed steps are
      // usually armor hand-over artifacts. Preserve the clock and let its own
      // robust residual gate decide whether the motion actually changed.
      out.phase_reset = true;
      agreement_updates_ = 0;
      disagreement_updates_ = 0;
    }
    have_last_rate_ = true;
    last_abs_ekf_rate_ = abs_rate;

    if (mode_ == Mode::high_speed_phase && phase_rate_valid &&
        std::isfinite(phase_omega_rad_s)) {
      out.rate_disagreement_rad_s =
        std::abs(abs_rate - std::abs(phase_omega_rad_s));
      out.rate_consistent = out.rate_disagreement_rad_s <=
        std::max(0.0, config_.max_rate_disagreement_rad_s);
      if (out.rate_consistent) {
        ++agreement_updates_;
        disagreement_updates_ = 0;
      } else {
        agreement_updates_ = 0;
        ++disagreement_updates_;
        if (disagreement_updates_ >=
            std::max(1, config_.rate_disagreement_updates)) {
          out.phase_reset = true;
          disagreement_updates_ = 0;
        }
      }
    } else {
      agreement_updates_ = 0;
      disagreement_updates_ = 0;
    }

    out.mode = mode_;
    out.phase_fire_trusted = mode_ == Mode::high_speed_phase &&
      phase_rate_valid && out.rate_consistent &&
      agreement_updates_ >= std::max(1, config_.rate_agreement_updates) &&
      !out.phase_reset;
    return out;
  }

  Mode mode() const { return mode_; }

  static const char * name(Mode mode)
  {
    switch (mode) {
      case Mode::armor: return "armor";
      case Mode::vehicle: return "vehicle";
      case Mode::high_speed_phase: return "high_speed_phase";
    }
    return "unknown";
  }

private:
  Mode desired_mode(double abs_rate) const
  {
    if (mode_ == Mode::high_speed_phase) {
      if (abs_rate >= config_.high_speed_exit_rad_s) {
        return Mode::high_speed_phase;
      }
    } else if (abs_rate >= config_.high_speed_enter_rad_s) {
      return Mode::high_speed_phase;
    }
    if (mode_ == Mode::vehicle || mode_ == Mode::high_speed_phase) {
      return abs_rate >= std::max(0.0, config_.vehicle_exit_rad_s)
        ? Mode::vehicle
        : Mode::armor;
    }
    return abs_rate >= config_.vehicle_enter_rad_s ? Mode::vehicle : Mode::armor;
  }

  Config config_{};
  Mode mode_ = Mode::armor;
  Mode candidate_mode_ = Mode::armor;
  double candidate_since_s_ = 0.0;
  bool have_candidate_ = false;
  bool have_last_rate_ = false;
  double last_abs_ekf_rate_ = 0.0;
  bool have_filtered_rate_ = false;
  double filtered_abs_rate_ = 0.0;
  double last_update_s_ = 0.0;
  int agreement_updates_ = 0;
  int disagreement_updates_ = 0;
  bool have_last_trusted_update_ = false;
  double last_trusted_update_s_ = 0.0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ROTATION_MODEL_MANAGER_HPP

#ifndef AUTO_AIM__SHOT_DELAY_ADAPTER_HPP
#define AUTO_AIM__SHOT_DELAY_ADAPTER_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

namespace auto_aim
{

// Adapts only the observable command-to-feeder part of the calibrated total
// firing delay.  The unobservable feeder-to-muzzle remainder stays fixed, so a
// noisy motor sample can never rewrite the complete ballistic calibration.
class ShotDelayAdapter
{
public:
  struct Config
  {
    bool enabled = false;
    double nominal_delay_s = 0.085;
    double min_delay_s = 0.065;
    double max_delay_s = 0.105;
    // <= 0 learns a session reference during warm-up without moving nominal.
    double reference_feedback_s = 0.0;
    double sample_min_s = 0.005;
    double sample_max_s = 0.100;
    std::size_t window_size = 9;
    std::size_t min_samples = 5;
    double ewma_alpha = 0.35;
    double adaptation_gain = 1.0;
    double max_step_s = 0.0015;
    double outlier_sigma = 3.5;
    double outlier_floor_s = 0.004;
    double min_prediction_half_width_s = 0.008;
  };

  struct Result
  {
    bool accepted = false;
    bool ready = false;
    bool rejected_outlier = false;
    bool regime_change = false;
    std::size_t sample_count = 0;
    double sample_s = std::numeric_limits<double>::quiet_NaN();
    double robust_feedback_s = std::numeric_limits<double>::quiet_NaN();
    double reference_feedback_s = std::numeric_limits<double>::quiet_NaN();
    double delay_s = std::numeric_limits<double>::quiet_NaN();
    double delay_min_s = std::numeric_limits<double>::quiet_NaN();
    double delay_max_s = std::numeric_limits<double>::quiet_NaN();
  };

  ShotDelayAdapter() = default;
  explicit ShotDelayAdapter(const Config & config) : config_(config)
  {
    normalize_config();
    reset();
  }

  void reset()
  {
    samples_.clear();
    delay_s_ = config_.nominal_delay_s;
    filtered_feedback_s_ = std::numeric_limits<double>::quiet_NaN();
    reference_feedback_s_ = config_.reference_feedback_s;
    ready_ = false;
    consecutive_outliers_ = 0;
    outlier_candidate_s_ = std::numeric_limits<double>::quiet_NaN();
    prediction_half_width_s_ = std::max(0.0, config_.min_prediction_half_width_s);
  }

  Result update(double sample_s)
  {
    Result out = snapshot();
    bool regime_change = false;
    out.sample_s = sample_s;
    if (!config_.enabled || !std::isfinite(sample_s) ||
        sample_s < config_.sample_min_s || sample_s > config_.sample_max_s) {
      return out;
    }

    if (samples_.size() >= config_.min_samples) {
      const double center_s = median(samples_);
      const double sigma_s = 1.4826 * median_absolute_deviation(samples_, center_s);
      const double threshold_s = std::max(
        config_.outlier_floor_s, config_.outlier_sigma * sigma_s);
      if (std::abs(sample_s - center_s) > threshold_s) {
        if (std::isfinite(outlier_candidate_s_) &&
            std::abs(sample_s - outlier_candidate_s_) <= config_.outlier_floor_s) {
          ++consecutive_outliers_;
          outlier_candidate_s_ +=
            (sample_s - outlier_candidate_s_) /
            static_cast<double>(consecutive_outliers_);
        } else {
          consecutive_outliers_ = 1;
          outlier_candidate_s_ = sample_s;
        }
        if (consecutive_outliers_ < 3) {
          out.rejected_outlier = true;
          return out;
        }

        // Three mutually consistent outliers are a new mechanical regime, not
        // a single bad packet. Rebuild the robust window while preserving the
        // original calibrated/session reference, so the correction is learned
        // gradually after another full warm-up.
        samples_.clear();
        filtered_feedback_s_ = std::numeric_limits<double>::quiet_NaN();
        ready_ = false;
        consecutive_outliers_ = 0;
        outlier_candidate_s_ = std::numeric_limits<double>::quiet_NaN();
        regime_change = true;
      }
    }

    if (!regime_change) {
      consecutive_outliers_ = 0;
      outlier_candidate_s_ = std::numeric_limits<double>::quiet_NaN();
    }

    samples_.push_back(sample_s);
    while (samples_.size() > config_.window_size) samples_.pop_front();
    out.accepted = true;

    if (samples_.size() >= config_.min_samples) {
      const double robust_feedback_s = median(samples_);
      if (!std::isfinite(filtered_feedback_s_)) {
        filtered_feedback_s_ = robust_feedback_s;
      } else {
        filtered_feedback_s_ += config_.ewma_alpha *
          (robust_feedback_s - filtered_feedback_s_);
      }
      if (!(reference_feedback_s_ > 0.0)) {
        // The old camera-loop feedback was quantized by one frame, so learn a
        // fresh reference after receive-thread timestamping is active.
        reference_feedback_s_ = filtered_feedback_s_;
      }

      const double target_delay_s = std::clamp(
        config_.nominal_delay_s + config_.adaptation_gain *
          (filtered_feedback_s_ - reference_feedback_s_),
        config_.min_delay_s,
        config_.max_delay_s);
      const double step_s = std::clamp(
        target_delay_s - delay_s_, -config_.max_step_s, config_.max_step_s);
      delay_s_ = std::clamp(
        delay_s_ + step_s, config_.min_delay_s, config_.max_delay_s);

      const double mad_s = median_absolute_deviation(samples_, robust_feedback_s);
      // 2.5 robust sigmas is a conservative small-sample approximation of a
      // 95% prediction interval.  Never make the interval narrower than the
      // configured allowance for serial and feeder-to-muzzle uncertainty.
      prediction_half_width_s_ = std::max(
        config_.min_prediction_half_width_s, 2.5 * 1.4826 * mad_s);
      prediction_half_width_s_ = std::min(
        prediction_half_width_s_,
        0.5 * (config_.max_delay_s - config_.min_delay_s));
      ready_ = true;
    }

    out = snapshot();
    out.accepted = true;
    out.regime_change = regime_change;
    out.sample_s = sample_s;
    return out;
  }

  bool ready() const { return ready_; }
  double delay_s() const { return delay_s_; }
  double delay_adjustment_s() const { return delay_s_ - config_.nominal_delay_s; }
  double delay_min_s() const
  {
    return ready_
      ? std::max(config_.min_delay_s, delay_s_ - prediction_half_width_s_)
      : config_.min_delay_s;
  }
  double delay_max_s() const
  {
    return ready_
      ? std::min(config_.max_delay_s, delay_s_ + prediction_half_width_s_)
      : config_.max_delay_s;
  }

private:
  void normalize_config()
  {
    config_.nominal_delay_s = std::max(0.0, config_.nominal_delay_s);
    config_.min_delay_s = std::max(0.0, config_.min_delay_s);
    config_.max_delay_s = std::max(config_.min_delay_s, config_.max_delay_s);
    config_.nominal_delay_s = std::clamp(
      config_.nominal_delay_s, config_.min_delay_s, config_.max_delay_s);
    config_.window_size = std::max<std::size_t>(3, config_.window_size);
    config_.min_samples = std::clamp<std::size_t>(
      config_.min_samples, 3, config_.window_size);
    config_.ewma_alpha = std::clamp(config_.ewma_alpha, 0.01, 1.0);
    config_.adaptation_gain = std::clamp(config_.adaptation_gain, 0.0, 1.0);
    config_.max_step_s = std::max(0.0, config_.max_step_s);
    config_.outlier_sigma = std::max(1.0, config_.outlier_sigma);
    config_.outlier_floor_s = std::max(0.0, config_.outlier_floor_s);
    config_.min_prediction_half_width_s =
      std::max(0.0, config_.min_prediction_half_width_s);
  }

  static double median(const std::deque<double> & values)
  {
    std::vector<double> sorted(values.begin(), values.end());
    std::sort(sorted.begin(), sorted.end());
    const std::size_t mid = sorted.size() / 2;
    return sorted.size() % 2 == 0
      ? 0.5 * (sorted[mid - 1] + sorted[mid])
      : sorted[mid];
  }

  static double median_absolute_deviation(
    const std::deque<double> & values, double center)
  {
    std::deque<double> deviations;
    for (const double value : values) deviations.push_back(std::abs(value - center));
    return median(deviations);
  }

  Result snapshot() const
  {
    Result out;
    out.ready = ready_;
    out.sample_count = samples_.size();
    out.robust_feedback_s = filtered_feedback_s_;
    out.reference_feedback_s = reference_feedback_s_;
    out.delay_s = delay_s_;
    out.delay_min_s = delay_min_s();
    out.delay_max_s = delay_max_s();
    return out;
  }

  Config config_{};
  std::deque<double> samples_;
  double delay_s_ = 0.085;
  double filtered_feedback_s_ = std::numeric_limits<double>::quiet_NaN();
  double reference_feedback_s_ = 0.0;
  double prediction_half_width_s_ = 0.020;
  bool ready_ = false;
  int consecutive_outliers_ = 0;
  double outlier_candidate_s_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SHOT_DELAY_ADAPTER_HPP

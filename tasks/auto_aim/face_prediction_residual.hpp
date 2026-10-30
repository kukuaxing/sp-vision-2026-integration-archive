// Adapted from Reimuaa/Fujian_AutoAim @ bb0240bbdbdce95e35fbd7eaa442f167e302ce31.
// MIT license and attribution: docs/third_party_fujian_autoaim.md.
#ifndef AUTO_AIM__FACE_PREDICTION_RESIDUAL_HPP
#define AUTO_AIM__FACE_PREDICTION_RESIDUAL_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace auto_aim
{
// Compare RAW forecasts with later, accepted observations of the SAME face.
// This estimates visual consistency, not physical impact or actuation delay.
// No corrected forecast is ever fed back into the estimator.
class FacePredictionResidual
{
public:
  struct Config
  {
    bool enabled = false;
    double max_correction_rad = 0.3 * pi / 180.0;
    bool phase_conditioned_enabled = false;
  };
  struct Input
  {
    int64_t frame_ns = 0;
    uint64_t epoch = 0;
    bool context_valid = false, high_speed_mode = false;
    bool rebuilt = false, observations_valid = false;
    double omega_rad_s = NAN, center_range_m = NAN;
    // Empty entries mean no unique, accepted, solved observation of that face.
    std::array<std::optional<std::array<double, 2>>, 4> observations;
    bool forecast_valid = false;
    int face = -1;
    int64_t prediction_ns = 0;
    double raw_yaw_rad = NAN, delay_s = NAN, speed_mps = NAN;
    // Raw forecast face angle relative to the forecast centre bearing.
    double forecast_alignment_rad = NAN;
  };
  struct Output
  {
    bool eligible = false, ready = false;
    double correction_rad = 0.0, bias_rad = NAN, scatter_rad = NAN;
    int cycles = 0;
    uint64_t generation = 0, matched = 0, unmatched = 0;
    bool phase_conditioned = false;
    int phase_cycles = 0;
    double phase_bias_rad = NAN, phase_scatter_rad = NAN;
    uint64_t model_revision = 0;
    // Past issued (already corrected) forecasts, scored only after their full
    // future reference bracket and an independent passage have completed.
    // History applicability survives a brief visibility gap; current-frame
    // correction eligibility and the observation/fire interlocks stay separate.
    bool quality_eligible = false;
    // A recent historical query can be retained through a missing forecast,
    // but must never authorize one: quality_eligible remains false while held.
    bool quality_retained = false;
    bool quality_ready = false;
    int quality_cycles = 0;
    double past_error_bound_rad = NAN;
  };

  // A moving query along ONE learned curve is not a model replacement. Reset
  // consecutive-frame evidence only when learning/fallback changes the model.
  static bool same_model(const Output & a, const Output & b)
  {
    // Preserve the existing flat/zero-correction path, including low speed.
    // Its inactive monitor clears a generation every frame by design.
    if (!a.phase_conditioned && !b.phase_conditioned)
      return std::abs(a.correction_rad - b.correction_rad) <= 1e-9;
    return a.generation == b.generation && a.model_revision == b.model_revision &&
      a.ready == b.ready && a.phase_conditioned == b.phase_conditioned;
  }

  FacePredictionResidual() = default;
  explicit FacePredictionResidual(Config config) : config_(config)
  {
    if (!std::isfinite(config.max_correction_rad) || config.max_correction_rad <= 0.0 ||
        config.max_correction_rad > 0.3 * pi / 180.0)
      throw std::invalid_argument("Invalid high-speed face residual limits");
  }

  Output update(const Input & in)
  {
    if (!config_.enabled) return {};
    const bool timed = in.frame_ns > 0 && (!last_frame_ns_ || in.frame_ns > last_frame_ns_);
    const bool context = timed && in.context_valid && in.high_speed_mode && in.epoch != 0 &&
      std::isfinite(in.omega_rad_s) && std::abs(in.omega_rad_s) > 0.1 &&
      std::isfinite(in.center_range_m) && in.center_range_m > 0.0;
    if (!context) {
      // A short qualification outage is not a new physical target. Keep the
      // learned history, but never make an invalid current frame eligible.
      const bool same_context = timed && in.high_speed_mode && !in.rebuilt &&
        in.epoch != 0 && in.epoch == epoch_ &&
        std::isfinite(in.omega_rad_s) && sign_ * in.omega_rad_s > 0.0 &&
        std::isfinite(in.center_range_m) && in.center_range_m > 0.0 &&
        last_context_ns_ > 0 && in.frame_ns - last_context_ns_ < frame_gap_ns &&
        (!last_frame_ns_ || in.frame_ns - last_frame_ns_ < frame_gap_ns);
      if (!same_context) clear();
      last_frame_ns_ = in.frame_ns;
      return status();
    }
    const int sign = in.omega_rad_s > 0.0 ? 1 : -1;
    if (in.rebuilt || epoch_ != in.epoch || sign_ != sign ||
        (last_frame_ns_ && in.frame_ns - last_frame_ns_ >= frame_gap_ns) ||
        (last_context_ns_ && in.frame_ns - last_context_ns_ >= frame_gap_ns)) clear();
    last_frame_ns_ = in.frame_ns;
    last_context_ns_ = in.frame_ns;
    epoch_ = in.epoch;
    sign_ = sign;

    const double horizon_s = (in.prediction_ns - in.frame_ns) / 1e9;
    const bool forecast = in.forecast_valid && in.face >= 0 && in.face < 4 &&
      std::isfinite(in.raw_yaw_rad) && std::isfinite(in.delay_s) &&
      std::isfinite(in.speed_mps) && in.speed_mps > 0.0 &&
      horizon_s > 0.0 && horizon_s <= 2.0;
    // Anchor each learning context. Comparing only adjacent frames would let
    // old corrections survive gradual changes in range/rate or timing.
    if (forecast && anchor_ &&
        (std::abs(in.center_range_m - anchor_->range) > 0.5 ||
         std::abs(in.omega_rad_s - anchor_->omega) > 1.5 ||
         std::abs(in.delay_s - anchor_->delay) > 0.001 ||
         std::abs(in.speed_mps - anchor_->speed) > 1.0 ||
         std::abs(horizon_s - anchor_->horizon) > 0.12)) clear_history();
    if (last_observation_ns_ && in.frame_ns - last_observation_ns_ >= observation_loss_ns)
      clear_history();
    if (forecast && !anchor_)
      anchor_ = Anchor{in.center_range_m, in.omega_rad_s, in.delay_s, in.speed_mps, horizon_s};

    if (in.observations_valid) {
      for (int face = 0; face < 4; ++face) {
        if (!in.observations[face]) continue;
        const auto & xy = *in.observations[face];
        if (!std::isfinite(xy[0]) || !std::isfinite(xy[1]) || std::hypot(xy[0], xy[1]) <= 0)
          continue;
        auto & history = faces_[face].observations;
        history.push_back({in.frame_ns, xy});
        while (history.size() > 160 ||
               (!history.empty() && in.frame_ns - history.front().time > 2100000000LL))
          history.pop_front();
        last_observation_ns_ = in.frame_ns;
      }
    }
    // A full +/-25ms reference window must have elapsed. Missing reference
    // brackets are discarded, never extrapolated or replaced by another face.
    while (!pending_.empty() && pending_.front().time + bracket_ns <= in.frame_ns) {
      const auto p = pending_.front();
      pending_.pop_front();
      const auto reference = interpolate(faces_[p.face].observations, p.time);
      if (!reference) { ++unmatched_; continue; }
      const double error = std::remainder(p.yaw - *reference, 2.0 * pi);
      auto & quality_face = faces_[p.face];
      if (std::isfinite(error)) {
        if (!quality_face.quality_group.empty() &&
            p.time + bracket_ns - quality_face.quality_last > cycle_gap_ns)
          close_quality(quality_face);
        if (quality_face.quality_group.empty()) quality_face.quality_first = p.time + bracket_ns;
        if (quality_face.quality_group.size() < 128)
          quality_face.quality_group.push_back({p.alignment,
            std::remainder(error + p.issued_correction, 2.0 * pi)});
        quality_face.quality_last = p.time + bracket_ns;
      }
      if (!std::isfinite(error) || std::abs(error) > 3.0 * pi / 180.0) {
        ++unmatched_; continue;
      }
      ++matched_;
      auto & f = faces_[p.face];
      const int64_t available = p.time + bracket_ns;
      if (!f.group.empty() && available - f.last_matured > cycle_gap_ns) close(f);
      // Bounded storage even for a stuck face identity; adjacent frames still
      // contribute only ONE completed visibility interval.
      if (f.group.empty()) f.first_matured = available;
      if (f.group.size() < 128) f.group.push_back({p.alignment, error});
      f.last_matured = available;
    }
    for (auto & f : faces_) {
      if (!f.quality_group.empty() && in.frame_ns - f.quality_last > cycle_gap_ns)
        close_quality(f);
      while (!f.quality_cycles.empty() && in.frame_ns - f.quality_cycles.front().time > age_ns)
        f.quality_cycles.pop_front();
      if (!f.group.empty() && in.frame_ns - f.last_matured > cycle_gap_ns) close(f);
      while (!f.cycles.empty() && in.frame_ns - f.cycles.front().time > age_ns) {
        f.cycles.pop_front();
        ++f.revision;
      }
    }

    Output out = status();
    out.eligible = forecast && in.observations_valid && last_observation_ns_ == in.frame_ns;
    out.quality_eligible = forecast && last_observation_ns_ > 0 &&
      in.frame_ns - last_observation_ns_ < observation_loss_ns;
    if (out.quality_eligible) {
      last_quality_query_ = QualityQuery{in.frame_ns, in.face, in.forecast_alignment_rad};
    } else if (!forecast && last_quality_query_ && last_observation_ns_ > 0 && anchor_ &&
        in.frame_ns - last_quality_query_->time < frame_gap_ns &&
        in.frame_ns - last_observation_ns_ < observation_loss_ns &&
        std::abs(in.center_range_m - anchor_->range) <= 0.5 &&
        std::abs(in.omega_rad_s - anchor_->omega) <= 1.5) {
      // Do not renew the query time. Retain only the last queried face/phase;
      // recompute its quality so expiry and newly matured bad evidence still
      // close the gate immediately. This is not a minimum-open timer.
      out.quality_retained = true;
    }
    if (out.quality_eligible || out.quality_retained) {
      const auto & query = *last_quality_query_;
      const auto & face = faces_[query.face];
      std::vector<double> passage_bounds;
      double coverage_low = INFINITY, coverage_high = -INFINITY;
      for (const auto & c : face.quality_cycles) {
        std::vector<double> errors;
        double low = INFINITY, high = -INFINITY;
        for (const auto & s : c.samples) {
          if (!std::isfinite(s.alignment) || !std::isfinite(query.alignment)) continue;
          const double delta = std::remainder(s.alignment - query.alignment, 2.0 * pi);
          if (std::abs(delta) > 20.0 * pi / 180.0) continue;
          low = std::min(low, delta); high = std::max(high, delta);
          errors.push_back(std::abs(s.error));
        }
        if (errors.size() < 3) continue;
        coverage_low = std::min(coverage_low, low);
        coverage_high = std::max(coverage_high, high);
        std::sort(errors.begin(), errors.end());
        passage_bounds.push_back(errors[static_cast<std::size_t>(std::ceil(0.9 * errors.size())) - 1]);
      }
      out.quality_cycles = static_cast<int>(passage_bounds.size());
      if (passage_bounds.size() >= 3 && coverage_low <= 0.0 && coverage_high >= 0.0) {
        // Each passage contributes one tail error, regardless of frame count.
        // Nearby independent passages jointly bracket the query. Requiring
        // each sampled passage to bracket it made tiny phase changes discard
        // mature evidence. Keep the +/-20deg neighbourhood and require pooled
        // coverage on both sides, so quality is not extrapolated beyond it.
        // This is empirical visual consistency, not a hit probability or a
        // confidence interval for an independently measured projectile.
        std::sort(passage_bounds.begin(), passage_bounds.end());
        out.past_error_bound_rad = passage_bounds[
          static_cast<std::size_t>(std::ceil(0.9 * passage_bounds.size())) - 1];
        out.quality_ready = std::isfinite(out.past_error_bound_rad);
      }
    }
    if (out.eligible) {
      const auto & face = faces_[in.face];
      const auto & cycles = face.cycles;
      out.model_revision = face.revision;
      out.cycles = static_cast<int>(cycles.size());
      if (cycles.size() >= 3) {
        std::vector<double> values;
        for (const auto & c : cycles) values.push_back(c.value);
        out.bias_rad = median(values);
        for (auto & v : values) v = std::abs(v - out.bias_rad);
        out.scatter_rad = median(values);
        // A large cycle-to-cycle spread is not a stable face bias.
        out.ready = out.scatter_rad <= 0.15 * pi / 180.0;
        if (out.ready)
          out.correction_rad = -std::clamp(out.bias_rad, -config_.max_correction_rad,
                                         config_.max_correction_rad);
      }
      if (config_.phase_conditioned_enabled && std::isfinite(in.forecast_alignment_rad)) {
        std::vector<double> local;
        for (const auto & cycle : cycles) {
          const auto value = local_bias(cycle, in.forecast_alignment_rad);
          if (value) local.push_back(*value);
        }
        out.phase_cycles = static_cast<int>(local.size());
        if (local.size() >= 3) {
          out.phase_bias_rad = median(local);
          std::vector<double> deviations;
          for (double v : local) deviations.push_back(std::abs(v - out.phase_bias_rad));
          out.phase_scatter_rad = median(deviations);
          const double baseline = out.correction_rad;
          const auto correction = [&](double bias) {
            // The extra model can extend the old correction by at most 0.3deg;
            // it does not enlarge the fire tolerance or alter the delay.
            return std::clamp(-bias,
              std::max(-0.6 * pi / 180.0, baseline - config_.max_correction_rad),
              std::min(0.6 * pi / 180.0, baseline + config_.max_correction_rad));
          };
          // Each completed turn is held out in turn. A phase-specific model
          // must improve the existing correction on independent past passages.
          std::vector<double> improvements;
          int improved = 0;
          for (std::size_t i = 0; i < local.size(); ++i) {
            auto others = local;
            others.erase(others.begin() + i);
            const double gain = std::abs(local[i] + baseline) -
              std::abs(local[i] + correction(median(others)));
            improvements.push_back(gain);
            if (gain > 0.0) ++improved;
          }
          if (out.phase_scatter_rad <= 0.15 * pi / 180.0 &&
              improved * 2 > static_cast<int>(local.size()) &&
              median(improvements) >= 0.02 * pi / 180.0) {
            out.ready = true;
            out.phase_conditioned = true;
            out.correction_rad = correction(out.phase_bias_rad);
          }
        }
      }
    }
    if (forecast && in.observations_valid && last_observation_ns_ == in.frame_ns) {
      if (pending_.size() >= 256) { pending_.pop_front(); ++unmatched_; }
      const Pending p{in.prediction_ns, in.face, in.raw_yaw_rad, in.forecast_alignment_rad,
        out.correction_rad};
      pending_.insert(std::upper_bound(pending_.begin(), pending_.end(), p.time,
        [](int64_t time, const Pending & v) { return time < v.time; }), p);
    }
    return out;
  }

private:
  static constexpr double pi = 3.14159265358979323846;
  static constexpr int64_t bracket_ns = 25000000, cycle_gap_ns = 250000000;
  static constexpr int64_t age_ns = 8000000000LL, frame_gap_ns = 150000000;
  // Keep high-speed calibration through short visibility gaps. A stalled
  // processing stream still resets at 150ms; only observation loss gets 200ms.
  static constexpr int64_t observation_loss_ns = 200000000;
  struct Observation { int64_t time; std::array<double, 2> xy; };
  struct Sample { double alignment, error; };
  struct Cycle { int64_t time; double value; std::vector<Sample> samples; };
  struct Face
  {
    std::deque<Observation> observations;
    std::deque<Cycle> cycles;
    std::vector<Sample> group;
    int64_t first_matured = 0, last_matured = 0;
    uint64_t revision = 0;
    std::vector<Sample> quality_group;
    std::deque<Cycle> quality_cycles;
    int64_t quality_first = 0, quality_last = 0;
  };
  struct Pending { int64_t time; int face; double yaw, alignment, issued_correction; };
  struct Anchor { double range, omega, delay, speed, horizon; };
  struct QualityQuery { int64_t time; int face; double alignment; };
  Config config_;
  std::array<Face, 4> faces_;
  std::deque<Pending> pending_;
  std::optional<Anchor> anchor_;
  std::optional<QualityQuery> last_quality_query_;
  uint64_t epoch_ = 0, generation_ = 0, matched_ = 0, unmatched_ = 0;
  int sign_ = 0;
  int64_t last_frame_ns_ = 0, last_observation_ns_ = 0, last_context_ns_ = 0;

  static double median(std::vector<double> values)
  {
    std::sort(values.begin(), values.end());
    const auto n = values.size();
    return n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
  }
  static std::optional<double> local_bias(const Cycle & cycle, double alignment)
  {
    // Nearby forecast phases only; never borrow a different physical face or
    // extrapolate a passage beyond its recorded angular coverage.
    constexpr double width = 20.0 * pi / 180.0;
    double low = INFINITY, high = -INFINITY;
    std::vector<double> values;
    for (const auto & s : cycle.samples) {
      if (!std::isfinite(s.alignment)) continue;
      const double delta = std::remainder(s.alignment - alignment, 2.0 * pi);
      if (std::abs(delta) <= width) {
        low = std::min(low, delta);
        high = std::max(high, delta);
        values.push_back(s.error);
      }
    }
    if (values.size() < 3 || low > 0.0 || high < 0.0) return {};
    return median(values);
  }
  static std::optional<double> interpolate(const std::deque<Observation> & history, int64_t t)
  {
    const auto right = std::lower_bound(history.begin(), history.end(), t,
      [](const Observation & o, int64_t time) { return o.time < time; });
    if (right == history.end()) return {};
    if (right->time == t) return std::atan2(right->xy[1], right->xy[0]);
    if (right == history.begin()) return {};
    const auto & left = *std::prev(right);
    if (t - left.time > bracket_ns || right->time - t > bracket_ns) return {};
    const double w = static_cast<double>(t - left.time) / (right->time - left.time);
    const double x = left.xy[0] + w * (right->xy[0] - left.xy[0]);
    const double y = left.xy[1] + w * (right->xy[1] - left.xy[1]);
    if (std::hypot(x, y) <= 0) return {};
    return std::atan2(y, x);
  }
  void close(Face & f)
  {
    const int64_t center = f.first_matured + (f.last_matured - f.first_matured) / 2;
    // A detection gap can split one passage into two intervals. Do not count
    // it as another independent turn: require >=3/4 of the anchored revolution.
    const double min_separation_ns = anchor_
      ? 0.75 * 2.0 * pi / std::abs(anchor_->omega) * 1e9 : 0.0;
    if (f.cycles.empty() || center - f.cycles.back().time >= min_separation_ns) {
      std::vector<double> errors;
      for (const auto & sample : f.group) errors.push_back(sample.error);
      f.cycles.push_back({center, median(errors), std::move(f.group)});
      ++f.revision;
    }
    if (f.cycles.size() > 32) f.cycles.pop_front();
    f.group.clear();
  }
  void close_quality(Face & f)
  {
    const int64_t center = f.quality_first + (f.quality_last - f.quality_first) / 2;
    const double separation = anchor_ ? 0.75 * 2.0 * pi / std::abs(anchor_->omega) * 1e9 : 0.0;
    if (f.quality_cycles.empty() || center - f.quality_cycles.back().time >= separation)
      f.quality_cycles.push_back({center, 0.0, std::move(f.quality_group)});
    while (f.quality_cycles.size() > 32) f.quality_cycles.pop_front();
    f.quality_group.clear();
  }
  Output status() const
  {
    Output out;
    out.generation = generation_;
    out.matched = matched_;
    out.unmatched = unmatched_;
    return out;
  }
  void clear_history()
  {
    faces_ = {};
    pending_.clear();
    anchor_.reset();
    last_quality_query_.reset();
    last_observation_ns_ = 0;
    ++generation_;
  }
  void clear()
  {
    clear_history(); epoch_ = 0; sign_ = 0; last_frame_ns_ = 0; last_context_ns_ = 0;
  }
};
}  // namespace auto_aim
#endif

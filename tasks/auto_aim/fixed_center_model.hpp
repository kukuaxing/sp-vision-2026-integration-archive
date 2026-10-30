// Adapted from Reimuaa/Fujian_AutoAim @ bb0240bbdbdce95e35fbd7eaa442f167e302ce31.
// MIT license and attribution: docs/third_party_fujian_autoaim.md.
#ifndef AUTO_AIM__FIXED_CENTER_MODEL_HPP
#define AUTO_AIM__FIXED_CENTER_MODEL_HPP

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

namespace auto_aim
{
// High-speed estimator. All accepted faces share ONE centre, two radii
// and a jointly fitted phase/rate. No velocity state, no first-frame centre
// lock, and no mutation of the tracker used for motion classification.
class FixedCenterModel
{
public:
  struct Observation { int face = -1; Eigen::Vector3d xyz; };
  struct Input
  {
    int64_t frame_ns = 0;
    uint64_t epoch = 0;
    bool active = false, context_valid = false, rebuilt = false;
    // Caller-certified missing capture pose only; never a usable observation.
    bool capture_pose_gap = false;
    Eigen::VectorXd seed;
    std::vector<Observation> observations;
  };
  struct Output
  {
    bool active = false, ready = false, fitted = false;
    uint64_t generation = 0;
    int observations = 0, heldout_observations = 0;
    double heldout_yaw_error_rad = NAN, fit_cost = NAN;
    Eigen::VectorXd state;
  };

  Output update(const Input & in)
  {
    Output out;
    if (!in.active) { reset(); return out; }
    out.active = true;
    if (!in.epoch || in.frame_ns <= 0 || in.seed.size() != 11 ||
        !in.seed.allFinite() || std::abs(in.seed[7]) < 1.5 || std::abs(in.seed[7]) > 8.0) {
      reset(); return out;
    }
    const int sign = in.seed[7] > 0 ? 1 : -1;
    if (!in.context_valid) {
      const bool retain = in.capture_pose_gap && !in.rebuilt && epoch_ == in.epoch &&
        sign_ == sign && last_context_ > 0 && last_frame_ > 0 &&
        in.frame_ns > last_frame_ && in.frame_ns-last_frame_ < 150000000 &&
        in.frame_ns-last_context_ < 150000000 && last_observation_ > 0 &&
        in.frame_ns-last_observation_ < 200000000;
      if (retain) {
        last_frame_ = in.frame_ns;
        out.observations = static_cast<int>(samples_.size());
        out.generation = generation_;
        // Do not consume supplied observations, refit, renew context time or
        // publish the previous fitted state as ready during the missing pose.
        return out;
      }
      reset(); return out;
    }
    if (in.rebuilt || epoch_ != in.epoch || sign_ != sign ||
        (last_frame_ && (in.frame_ns <= last_frame_ || in.frame_ns - last_frame_ >= 150000000)) ||
        (last_context_ && in.frame_ns - last_context_ >= 150000000) ||
        (last_observation_ && in.frame_ns - last_observation_ >= 200000000)) reset();
    epoch_ = in.epoch; sign_ = sign; last_frame_ = last_context_ = in.frame_ns;
    std::array<int, 4> counts{};
    for (const auto & o : in.observations) if (o.face >= 0 && o.face < 4) ++counts[o.face];
    bool fresh = false;
    for (const auto & o : in.observations) {
      if (o.face < 0 || o.face >= 4 || counts[o.face] != 1 || !o.xyz.allFinite() ||
          o.xyz.head<2>().norm() < 0.1) continue;
      fresh = true;
      // Bound CPU and memory independently of camera FPS. Faces at the same
      // timestamp remain one frame, and neither is duplicated as another pass.
      if (!last_stored_ || in.frame_ns - last_stored_ >= 25000000 ||
          (!samples_.empty() && samples_.back().time == in.frame_ns))
        samples_.push_back({in.frame_ns, o.face, o.xyz});
    }
    if (fresh) {
      last_observation_ = in.frame_ns;
      if (!samples_.empty() && samples_.back().time == in.frame_ns) last_stored_ = in.frame_ns;
    }
    while (!samples_.empty() && (in.frame_ns - samples_.front().time > 3200000000LL ||
                                samples_.size() > 160)) samples_.pop_front();
    out.observations = static_cast<int>(samples_.size());
    out.generation = generation_;
    if (!fresh) return out;
    if ((!last_fit_ || in.frame_ns - last_fit_ >= 80000000) && samples_.size() >= 48 &&
        in.frame_ns - samples_.front().time >= 2700000000LL &&
        std::abs(in.seed[7]) * (in.frame_ns - samples_.front().time) / 1e9 >= 3.0 * pi) {
      last_fit_ = in.frame_ns;
      ready_ = false;
      Parameters seed;
      seed << in.seed[0], in.seed[2], in.seed[6], in.seed[7], in.seed[8], in.seed[8] + in.seed[9];
      std::vector<Sample> train, holdout, all(samples_.begin(), samples_.end());
      for (const auto & s : samples_)
        (s.time <= in.frame_ns - 600000000 ? train : holdout).push_back(s);
      Parameters trained = seed;
      if (holdout.size() >= 8 && fit(train, in.frame_ns, sign, trained)) {
        std::vector<double> errors;
        for (const auto & s : holdout) {
          const auto point = position(trained, s, in.frame_ns);
          errors.push_back(std::abs(std::remainder(std::atan2(point.y(), point.x()) -
            std::atan2(s.xyz.y(), s.xyz.x()), 2.0 * pi)));
        }
        std::sort(errors.begin(), errors.end());
        heldout_error_ = errors[static_cast<std::size_t>(std::ceil(0.9 * errors.size())) - 1];
        heldout_count_ = static_cast<int>(errors.size());
        // Retain the chronological held-out score. A full refit is used only
        // after this test, never to rewrite its earlier validation residual.
        Parameters full = trained;
        if (heldout_error_ <= 0.6 * pi / 180.0 && fit(all, in.frame_ns, sign, full)) {
          parameters_ = full;
          cost_ = cost(all, in.frame_ns, full);
          for (int parity = 0; parity < 2; ++parity) {
            std::vector<double> heights;
            for (const auto & s : all) if (s.face % 2 == parity) heights.push_back(s.xyz.z());
            std::sort(heights.begin(), heights.end());
            height_[parity] = heights[heights.size() / 2];
          }
          ready_ = true;
          out.fitted = true;
        }
      }
    }
    out.heldout_yaw_error_rad = heldout_error_;
    out.heldout_observations = heldout_count_;
    out.fit_cost = cost_;
    out.ready = ready_ && last_fit_ && in.frame_ns - last_fit_ < 150000000;
    if (out.ready) {
      out.state = in.seed;
      out.state[0] = parameters_[0]; out.state[2] = parameters_[1];
      out.state[1] = out.state[3] = out.state[5] = 0.0;
      out.state[4] = height_[0]; out.state[10] = height_[1] - height_[0];
      out.state[6] = std::remainder(parameters_[2] + parameters_[3] *
        (in.frame_ns - last_fit_) / 1e9, 2.0 * pi);
      out.state[7] = parameters_[3]; out.state[8] = parameters_[4];
      out.state[9] = parameters_[5] - parameters_[4];
    }
    return out;
  }

private:
  static constexpr double pi = 3.14159265358979323846;
  using Parameters = Eigen::Matrix<double, 6, 1>;
  struct Sample { int64_t time; int face; Eigen::Vector3d xyz; };
  std::deque<Sample> samples_;
  Parameters parameters_ = Parameters::Zero();
  std::array<double, 2> height_{};
  uint64_t epoch_ = 0;
  uint64_t generation_ = 0;
  int sign_ = 0, heldout_count_ = 0;
  int64_t last_frame_ = 0, last_context_ = 0, last_observation_ = 0, last_stored_ = 0, last_fit_ = 0;
  bool ready_ = false;
  double heldout_error_ = NAN, cost_ = NAN;

  static Eigen::Vector2d position(const Parameters & p, const Sample & s, int64_t now)
  {
    const double a = p[2] + p[3] * (s.time - now) / 1e9 + s.face * pi / 2.0;
    return {p[0] - p[4 + s.face % 2] * std::cos(a),
            p[1] - p[4 + s.face % 2] * std::sin(a)};
  }
  static Eigen::Matrix2d whitening(const Sample & s)
  {
    const double bearing = std::atan2(s.xyz.y(), s.xyz.x());
    const double radial_sigma = 0.03 + 0.025 * s.xyz.head<2>().norm();
    Eigen::Matrix2d w;
    // PnP depth is much less reliable than transverse bearing. Both rows come
    // from the same position; do not also fuse its orientation as independent truth.
    w << std::cos(bearing) / radial_sigma, std::sin(bearing) / radial_sigma,
        -std::sin(bearing) / 0.015, std::cos(bearing) / 0.015;
    return w;
  }
  static double cost(const std::vector<Sample> & samples, int64_t now, const Parameters & p)
  {
    double sum = 0.0;
    for (const auto & s : samples) {
      const double n = (whitening(s) * (s.xyz.head<2>() - position(p, s, now))).norm();
      sum += n <= 2.5 ? 0.5 * n * n : 2.5 * (n - 1.25);
    }
    return sum / std::max<std::size_t>(1, samples.size());
  }
  static bool valid(const Parameters & p, int sign)
  {
    return p.allFinite() && std::hypot(p[0], p[1]) > 0.1 &&
      p[3] * sign >= 1.5 && p[3] * sign <= 8.0 &&
      p[4] > 0.05 && p[4] < 0.5 && p[5] > 0.05 && p[5] < 0.5;
  }
  static bool fit(const std::vector<Sample> & samples, int64_t now, int sign, Parameters & p)
  {
    std::array<int, 4> count{};
    for (const auto & s : samples) ++count[s.face];
    if (samples.size() < 32 || *std::min_element(count.begin(), count.end()) < 3 || !valid(p, sign))
      return false;
    for (int iteration = 0; iteration < 10; ++iteration) {
      Eigen::Matrix<double, 6, 6> normal = Eigen::Matrix<double, 6, 6>::Zero();
      Parameters rhs = Parameters::Zero();
      for (const auto & s : samples) {
        const double dt = (s.time - now) / 1e9, a = p[2] + p[3] * dt + s.face * pi / 2.0;
        const double r = p[4 + s.face % 2], sa = std::sin(a), ca = std::cos(a);
        Eigen::Matrix<double, 2, 6> j = Eigen::Matrix<double, 2, 6>::Zero();
        j(0, 0) = j(1, 1) = 1.0;
        j(0, 2) = r * sa; j(1, 2) = -r * ca;
        j.col(3) = j.col(2) * dt;
        j(0, 4 + s.face % 2) = -ca; j(1, 4 + s.face % 2) = -sa;
        const auto w = whitening(s);
        const Eigen::Vector2d residual = w * (s.xyz.head<2>() - position(p, s, now));
        const double weight = std::min(1.0, 2.5 / std::max(1e-12, residual.norm()));
        const Eigen::Matrix<double, 2, 6> h = w * j;
        normal.noalias() += weight * h.transpose() * h;
        rhs.noalias() += weight * h.transpose() * residual;
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> spectrum(normal);
      if (spectrum.info() != Eigen::Success || spectrum.eigenvalues()[0] <= 1e-6 ||
          spectrum.eigenvalues()[5] / spectrum.eigenvalues()[0] > 1e8) return false;
      normal.diagonal().array() += 1e-3;
      Parameters step = normal.ldlt().solve(rhs);
      if (!step.allFinite()) return false;
      const std::array<double, 6> bounds{0.05, 0.05, 0.15, 0.2, 0.02, 0.02};
      for (int i = 0; i < 6; ++i) step[i] = std::clamp(step[i], -bounds[i], bounds[i]);
      const double before = cost(samples, now, p);
      bool accepted = false;
      for (double scale : {1.0, 0.5, 0.25, 0.125}) {
        const Parameters candidate = p + scale * step;
        if (valid(candidate, sign) && cost(samples, now, candidate) <= before + 1e-12) {
          p = candidate; accepted = true; break;
        }
      }
      if (!accepted || step.norm() < 1e-5) break;
    }
    return valid(p, sign);
  }
  void reset()
  {
    ++generation_;
    samples_.clear(); epoch_ = 0; sign_ = 0;
    last_frame_ = last_context_ = last_observation_ = last_stored_ = last_fit_ = 0;
    ready_ = false; heldout_error_ = cost_ = NAN; heldout_count_ = 0;
  }
};

// Source ownership survives a short blind interval, but availability never does.
// This only chooses a model; it does not retain or schedule a fire request.
class FixedCenterOwnership
{
public:
  struct Input
  {
    int64_t frame_ns = 0;
    uint64_t epoch = 0, generation = 0;
    bool active = false, context_valid = false, rebuilt = false, ready = false;
    bool capture_pose_gap = false;
  };
  struct Output { bool fixed = false, available = false, changed = false; };
  Output update(const Input & in)
  {
    const bool previous = fixed_;
    if (in.active && !in.context_valid && in.capture_pose_gap && !in.rebuilt &&
        in.epoch != 0 && epoch_ == in.epoch && generation_ == in.generation &&
        last_frame_ > 0 && in.frame_ns > last_frame_ &&
        in.frame_ns-last_frame_ < 150000000 && last_ready_ > 0 &&
        in.frame_ns-last_ready_ < 150000000) {
      // Keep ownership/dedup context only; neither timestamp is renewed and
      // availability is always false while capture pose is missing.
      return {fixed_,false,false};
    }
    const bool valid = in.active && in.context_valid && in.epoch && in.frame_ns > 0;
    const bool rebuilt = in.rebuilt || epoch_ != in.epoch || generation_ != in.generation ||
      (last_frame_ && (in.frame_ns <= last_frame_ || in.frame_ns - last_frame_ >= 150000000));
    if (!valid || rebuilt) reset();
    if (valid) {
      epoch_ = in.epoch; generation_ = in.generation; last_frame_ = in.frame_ns;
      if (last_ready_ && in.frame_ns - last_ready_ >= 150000000) {
        fixed_ = false; ready_since_ = last_ready_ = 0; ready_frames_ = 0;
      }
      if (in.ready) {
        if (!ready_since_) ready_since_ = in.frame_ns;
        last_ready_ = in.frame_ns;
        ready_frames_ = std::min(ready_frames_ + 1, 3);
        if (ready_frames_ >= 3 && in.frame_ns - ready_since_ >= 200000000) fixed_ = true;
      }
    }
    return {fixed_, fixed_ && valid && in.ready,
      previous != fixed_ || (previous && rebuilt)};
  }
private:
  bool fixed_ = false;
  int ready_frames_ = 0;
  uint64_t epoch_ = 0, generation_ = 0;
  int64_t last_frame_ = 0, ready_since_ = 0, last_ready_ = 0;
  void reset()
  {
    fixed_ = false; ready_frames_ = 0; epoch_ = generation_ = 0;
    last_frame_ = ready_since_ = last_ready_ = 0;
  }
};
}  // namespace auto_aim
#endif

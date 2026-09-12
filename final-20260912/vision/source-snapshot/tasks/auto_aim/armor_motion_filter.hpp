#ifndef AUTO_AIM__ARMOR_MOTION_FILTER_HPP
#define AUTO_AIM__ARMOR_MOTION_FILTER_HPP

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace auto_aim
{

// Constant-velocity filter for a physical armor face.  This complements the
// rigid vehicle model at low rotation rates, where directly estimating the
// visible armor translation is better conditioned than center/radius fitting.
class ArmorMotionFilter
{
public:
  struct Config
  {
    double acceleration_variance = 25.0;
    double measurement_sigma_base_m = 0.010;
    double measurement_sigma_per_m = 0.015;
    double nis_reject_threshold = 16.27;  // chi-square(3), approximately 99.9%
    int min_trusted_updates = 4;
    double max_position_std_m = 0.35;
    double max_prediction_age_s = 0.75;
    double max_speed_mps = 5.0;
    int reset_after_consecutive_rejections = 3;
  };

  ArmorMotionFilter() = default;
  explicit ArmorMotionFilter(const Config & config) : config_(config) {}

  void reset(const Eigen::Vector3d & position)
  {
    x_.setZero();
    x_.head<3>() = position;
    P_.setZero();
    P_.diagonal() << 0.04, 0.04, 0.04, 9.0, 9.0, 9.0;
    initialized_ = true;
    accepted_updates_ = 1;
    rejected_updates_ = 0;
    consecutive_rejections_ = 0;
    age_since_update_s_ = 0.0;
    last_nis_ = 0.0;
  }

  void predict(double dt)
  {
    if (!initialized_ || !std::isfinite(dt) || dt <= 0.0) return;
    age_since_update_s_ += dt;

    // Ballistic prediction horizons are commonly 0.35--0.50 s. Clamping the
    // whole call to 0.15 s silently discarded most of the low-speed lead.
    // Integrate the complete duration in bounded covariance steps instead.
    double remaining = std::min(dt, 2.0);
    while (remaining > 1e-9) {
      const double step = std::min(remaining, 0.15);
      Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
      F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * step;
      const double dt2 = step * step;
      const double dt3 = dt2 * step;
      const double dt4 = dt2 * dt2;
      Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
      Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() *
        (0.25 * dt4 * config_.acceleration_variance);
      Q.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() *
        (0.5 * dt3 * config_.acceleration_variance);
      Q.block<3, 3>(3, 0) = Q.block<3, 3>(0, 3);
      Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() *
        (dt2 * config_.acceleration_variance);
      x_ = F * x_;
      P_ = F * P_ * F.transpose() + Q;
      P_ = 0.5 * (P_ + P_.transpose());
      remaining -= step;
    }
  }

  bool update(const Eigen::Vector3d & position)
  {
    if (!position.allFinite()) return false;
    if (!initialized_) {
      reset(position);
      return true;
    }
    const double range = position.norm();
    const double sigma = config_.measurement_sigma_base_m +
      config_.measurement_sigma_per_m * std::max(0.0, range);
    const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * sigma * sigma;
    const Eigen::Matrix<double, 3, 6> H =
      (Eigen::Matrix<double, 3, 6>() <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0).finished();
    const Eigen::Vector3d innovation = position - H * x_;
    const Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    const auto ldlt = S.ldlt();
    last_nis_ = innovation.dot(ldlt.solve(innovation));
    if (!std::isfinite(last_nis_) || last_nis_ > config_.nis_reject_threshold) {
      ++rejected_updates_;
      ++consecutive_rejections_;
      // A physical face may disappear for a long time and later be assigned a
      // genuinely new track.  Do not let a stale CV state reject it forever.
      // Reinitialize only after several consecutive contradictions; the filter
      // must then earn min_trusted_updates again before driving prediction.
      if (consecutive_rejections_ >=
          std::max(1, config_.reset_after_consecutive_rejections)) {
        const int total_rejections = rejected_updates_;
        reset(position);
        rejected_updates_ = total_rejections;
      }
      return false;
    }
    const Eigen::Matrix<double, 6, 3> K =
      P_ * H.transpose() * ldlt.solve(Eigen::Matrix3d::Identity());
    x_ += K * innovation;
    const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 6> IKH = I - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
    ++accepted_updates_;
    consecutive_rejections_ = 0;
    age_since_update_s_ = 0.0;
    return true;
  }

  bool initialized() const { return initialized_; }
  bool trusted() const
  {
    return initialized_ && accepted_updates_ >= config_.min_trusted_updates &&
      consecutive_rejections_ == 0 &&
      age_since_update_s_ <= std::max(0.0, config_.max_prediction_age_s) &&
      x_.tail<3>().norm() <= std::max(0.0, config_.max_speed_mps) &&
      P_.block<3, 3>(0, 0).diagonal().maxCoeff() <
        config_.max_position_std_m * config_.max_position_std_m;
  }
  Eigen::Vector3d position() const { return x_.head<3>(); }
  Eigen::Vector3d velocity() const { return x_.tail<3>(); }
  double last_nis() const { return last_nis_; }
  int accepted_updates() const { return accepted_updates_; }
  int rejected_updates() const { return rejected_updates_; }
  int consecutive_rejections() const { return consecutive_rejections_; }
  double age_since_update_s() const { return age_since_update_s_; }

private:
  Config config_;
  Eigen::Matrix<double, 6, 1> x_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> P_ = Eigen::Matrix<double, 6, 6>::Identity();
  bool initialized_ = false;
  int accepted_updates_ = 0;
  int rejected_updates_ = 0;
  int consecutive_rejections_ = 0;
  double age_since_update_s_ = 0.0;
  double last_nis_ = 0.0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_MOTION_FILTER_HPP

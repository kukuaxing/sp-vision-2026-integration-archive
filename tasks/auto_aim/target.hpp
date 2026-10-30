#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "armor_motion_filter.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "yaw_eskf.hpp"

namespace auto_aim
{

class Target
{
public:
  struct IteratedUpdateConfig
  {
    bool enabled = true;
    int max_iterations = 4;
    double stop_threshold = 1e-4;
    double huber_threshold = 2.5;
  };

  struct CenterProcessConfig {
    bool adaptive_enabled = false;
    double acceleration_variance = 100.0;
    double rotation_acceleration_variance = 10.0;
    double rotation_rate_rad_s = 0.8;
  };

  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig,
    const YawESKF::Config & yaw_eskf_config = YawESKF::Config{});
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig, const YawESKF::Config & yaw_eskf_config,
    const IteratedUpdateConfig & iterated_update_config);
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig, const YawESKF::Config & yaw_eskf_config,
    const IteratedUpdateConfig & iterated_update_config, const CenterProcessConfig & center_config);
  double center_acceleration_variance() const { return center_acceleration_variance_; }
  std::vector<Eigen::Vector4d> whole_ekf_armor_xyza_list() const {
    return association_armor_xyza_list();
  }
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);
  void update(const Armor & armor, int armor_id);
  void update_batch(
    const std::vector<std::pair<Armor, int>> & observations, int primary_armor_id);

  Eigen::VectorXd ekf_x() const;
  // Sample the current kinematic mean at a relative time (negative allowed).
  // This is not a filter prediction/update: covariance, time and trust stay
  // unchanged. Used by the planner's centered finite-difference trajectory.
  Eigen::VectorXd ekf_x_at(double relative_time_s) const;
  const tools::ExtendedKalmanFilter & ekf() const;
  struct PriorInnovation {
    bool valid = false;
    Eigen::Vector4d residual = Eigen::Vector4d::Constant(NAN);
    Eigen::Vector4d variance = Eigen::Vector4d::Constant(NAN);
    double nis = NAN;
  };
  PriorInnovation prior_innovation(const Armor & measurement, int face) const;
  // Prediction-only copy; never feeds a fitted state back into association/KF.
  std::optional<Target> with_fixed_center_prediction(const Eigen::VectorXd & state) const;
  // Fixed-center prediction copies need only the phase at a future instant.
  // No filter/covariance copy, allocation or mutation; other models return null.
  std::optional<Eigen::Vector4d> fixed_center_armor_at(
    int face, std::chrono::steady_clock::time_point time) const;
  bool yaw_eskf_trusted() const;
  // Confidence of the coherent vehicle state, independent of a yaw splice.
  bool whole_rotation_trusted() const;
  // Stable diagnostic bits; zero is exactly the existing trust predicate.
  enum WholeRotationRejection : std::uint32_t {
    wrong_layout = 1u << 0, no_face_diversity = 1u << 1,
    immature = 1u << 2, invalid_dimensions = 1u << 3,
    nonfinite_state = 1u << 4, divergent = 1u << 5,
    excessive_rate = 1u << 6, phase_variance = 1u << 7,
    rate_variance = 1u << 8, center_x_variance = 1u << 9,
    center_z_variance = 1u << 10, radius_variance = 1u << 11,
    second_radius_variance = 1u << 12
  };
  std::uint32_t whole_rotation_rejection_mask() const;
  bool federated_phase_consistent() const;
  double yaw_eskf_angle() const;
  double yaw_eskf_rate() const;
  double yaw_eskf_acceleration() const;
  double yaw_eskf_nis() const;
  int yaw_eskf_accepted_updates() const;
  int yaw_eskf_rejected_updates() const;
  bool armor_motion_trusted(int id) const;
  Eigen::Vector3d armor_motion_velocity(int id) const;
  double armor_motion_nis(int id) const;
  std::vector<Eigen::Vector4d> armor_xyza_list(bool prefer_direct_motion = false) const;
  // Data association must use the same phase/geometry as the vehicle filter's
  // measurement function, not a separately estimated yaw pasted into its mean.
  std::vector<Eigen::Vector4d> association_armor_xyza_list() const;
  std::vector<Eigen::Vector4d> armor_xyza_list_at(
    double relative_time_s, bool prefer_direct_motion = false) const;
  int armor_num() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;

  tools::ExtendedKalmanFilter ekf_;
  bool fixed_center_prediction_ = false;
  YawESKF yaw_eskf_;
  std::array<ArmorMotionFilter, 4> armor_motion_filters_;
  IteratedUpdateConfig iterated_update_config_;
  CenterProcessConfig center_process_config_;
  double center_acceleration_variance_ = 100.0;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  void update_yaw_eskf(const Armor & armor, int id);

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP

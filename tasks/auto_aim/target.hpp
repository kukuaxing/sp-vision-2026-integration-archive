#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <array>
#include <chrono>
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
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);
  void update(const Armor & armor, int armor_id);
  void update_batch(
    const std::vector<std::pair<Armor, int>> & observations, int primary_armor_id);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  bool yaw_eskf_trusted() const;
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
  YawESKF yaw_eskf_;
  std::array<ArmorMotionFilter, 4> armor_motion_filters_;
  IteratedUpdateConfig iterated_update_config_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  void update_yaw_eskf(const Armor & armor, int id);

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP

#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <cstdint>
#include <limits>
#include <list>

#include "io/shoot_mode.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

enum class AimGeometry { existing, whole_ekf };

// Optional observer of the existing iterations. Never used for control.
struct AimPredictionTrace
{
  bool valid = false, converged = false;
  bool whole_ekf_geometry = false;
  bool high_speed_delay = false;
  int face_id = -1, iterations = 0;
  int64_t base_prediction_ns = 0, prediction_ns = 0;
  double delay_s = std::numeric_limits<double>::quiet_NaN();
  double delay_motion_rate_rad_s = std::numeric_limits<double>::quiet_NaN();
  double speed_mps = std::numeric_limits<double>::quiet_NaN();
  double flight_time_used_s = std::numeric_limits<double>::quiet_NaN();
  double flight_time_s = std::numeric_limits<double>::quiet_NaN();
  bool final_point_selection = false;
  int high_speed_rotation_sign = 0;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    const std::list<Target> & targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true, AimPredictionTrace * trace = nullptr,
    AimGeometry geometry = AimGeometry::existing, bool rigid_high_speed = false);

  io::Command aim(
    const std::list<Target> & targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true, AimPredictionTrace * trace = nullptr,
    AimGeometry geometry = AimGeometry::existing);

  // Single selected target; null means no target. No temporary owning list.
  io::Command aim_target(
    const Target * target, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true, AimPredictionTrace * trace = nullptr,
    AimGeometry geometry = AimGeometry::existing, bool rigid_high_speed = false);

  // Applies the same bounded runtime correction to both speed branches.  Their
  // calibrated base values remain unchanged and are restored with zero.
  void set_delay_adjustment(double adjustment_s);

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double high_speed_delay_time_cw_, high_speed_delay_time_ccw_;
  double low_speed_delay_time_;
  double delay_adjustment_time_ = 0.0;
  double decision_speed_;

  AimPoint choose_aim_point(
    const Target & target, bool whole_ekf_geometry, int * face_id = nullptr);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP

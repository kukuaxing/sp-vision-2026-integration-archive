#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include "tools/trajectory.hpp"

namespace io::rmv {
inline constexpr double debug_radians_per_degree=3.141592653589793/180;
inline constexpr double debug_max_displacement_degrees=30;
inline double debug_limit_radians(double degrees) {
  // Validate in the configuration's unit before converting. Multiplication
  // order can make 3*(pi/180) one ULP larger than (3*pi)/180.
  if(!std::isfinite(degrees) || degrees<0 || degrees>debug_max_displacement_degrees)
    throw std::invalid_argument("Debug displacement limit must be <=30 degrees");
  return degrees*debug_radians_per_degree;
}
struct JointGoal {
  double yaw, pitch;
  double yaw_rate=0, pitch_rate=0, yaw_accel=0, pitch_accel=0;
  bool trajectory=false;
};
inline double bounded_trajectory_step(double position,double velocity,double acceleration,
  double dt,double maximum,double& rate){
  rate=std::clamp(velocity+acceleration*dt,-maximum,maximum);
  double t=dt;
  if(acceleration>0 && velocity+acceleration*dt>maximum)t=std::max(0.,(maximum-velocity)/acceleration);
  if(acceleration<0 && velocity+acceleration*dt<-maximum)t=std::max(0.,(-maximum-velocity)/acceleration);
  return position+velocity*t+.5*acceleration*t*t+rate*(dt-t);
}
inline JointGoal advance_trajectory(JointGoal goal,double dt){
  if(!goal.trajectory || dt<=0)return goal;
  goal.yaw=bounded_trajectory_step(goal.yaw,goal.yaw_rate,goal.yaw_accel,dt,2,goal.yaw_rate);
  goal.pitch=bounded_trajectory_step(goal.pitch,goal.pitch_rate,goal.pitch_accel,dt,1.5,goal.pitch_rate);
  goal.yaw=std::remainder(goal.yaw,2*M_PI);
  return goal;
}

// An empirical command pitch trim is represented as a bore elevation bias.
// Negative trim raises the modelled bore relative to G, so inverse aiming
// lowers the joint command. Use this same bore for aiming and shot validation.
inline Eigen::Vector3d bore_for_pitch_trim(const Eigen::Vector3d& nominal,double trim_deg){
  if(!nominal.allFinite() || std::abs(nominal.norm()-1)>1e-3 || nominal.x()<=0 ||
     !std::isfinite(trim_deg) || std::abs(trim_deg)>5)
    throw std::invalid_argument("Invalid RMV empirical bore/pitch trim");
  return Eigen::AngleAxisd(trim_deg*debug_radians_per_degree,Eigen::Vector3d::UnitY())*nominal;
}

// G retains the reported joint zeros; b is the actual gun direction in G.
inline std::optional<JointGoal> inverse_boresight(
  const Eigen::Quaterniond& qNG, JointGoal current, const Eigen::Vector3d& ray,
  const Eigen::Vector3d& b, double yaw_sign=-1, double pitch_sign=1)
{
  if (!qNG.coeffs().allFinite() || std::abs(qNG.norm()-1)>1e-3 ||
      !ray.allFinite() || ray.norm()<1e-9 || !b.allFinite() ||
      std::abs(b.norm()-1)>1e-3 || b.x()<=0 ||
      !std::isfinite(current.yaw) || !std::isfinite(current.pitch) ||
      std::abs(yaw_sign)!=1 || std::abs(pitch_sign)!=1) return {};
  const Eigen::Quaterniond qNY = qNG * Eigen::Quaterniond(
    Eigen::AngleAxisd(pitch_sign*current.pitch, Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d v = qNY.conjugate()*ray.normalized();
  const double radius=std::hypot(b.x(),b.z());
  if (std::abs(v.z())>radius+1e-10) return {};
  const double x=std::sqrt(std::max(0.0,radius*radius-v.z()*v.z()));
  if (std::hypot(x,b.y())<=1e-4) return {};
  const double delta=std::atan2(v.y(),v.x())-std::atan2(b.y(),x);
  return JointGoal{current.yaw+std::atan2(std::sin(delta),std::cos(delta))/yaw_sign,
    (std::atan2(v.z(),x)-std::atan2(b.z(),b.x()))/pitch_sign};
}

inline Eigen::Quaterniond goal_pose(
  const Eigen::Quaterniond& qNG, JointGoal current, JointGoal goal,double yaw_sign=-1)
{
  return qNG * Eigen::Quaterniond(Eigen::AngleAxisd(current.pitch,Eigen::Vector3d::UnitY())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(yaw_sign*(goal.yaw-current.yaw),Eigen::Vector3d::UnitZ())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(-goal.pitch,Eigen::Vector3d::UnitY()));
}

// Line-of-sight commissioning only: include the rotating muzzle lever arm,
// without ballistic elevation, shot scheduling or an assumed bullet speed.
inline std::optional<JointGoal> point_from_muzzle(
  const Eigen::Quaterniond& qNG, JointGoal current, const Eigen::Vector3d& point,
  const Eigen::Vector3d& muzzle, const Eigen::Vector3d& bore,double yaw_sign=-1)
{
  if (!point.allFinite() || !muzzle.allFinite()) return {};
  JointGoal goal=current;
  for (int i=0;i<32;++i) {
    const Eigen::Vector3d ray=point-goal_pose(qNG,current,goal,yaw_sign)*muzzle;
    auto next=inverse_boresight(qNG,current,ray,bore,yaw_sign);
    if (!next) return {};
    const double error=std::hypot(next->yaw-goal.yaw,next->pitch-goal.pitch);
    goal=*next;
    if (error<1e-7) return goal;
  }
  return {}; // Do not execute a non-convergent target.
}

struct BallisticGoal {
  JointGoal joints;
  double flight_time;
  Eigen::Vector3d muzzle_world;
};

// Motor RPM is not wheel RPM when a gearbox is present. This is a surface
// speed estimate multiplied by an explicit, unmeasured transfer factor.
inline double estimate_projectile_speed(double diameter_m,double motor_rpm,double ratio,double efficiency){
  if(!std::isfinite(diameter_m) || !std::isfinite(motor_rpm) || !std::isfinite(ratio) || !std::isfinite(efficiency) ||
     diameter_m<=0 || diameter_m>.5 || ratio<=0 || motor_rpm<=0 || efficiency<=0 || efficiency>1)
    throw std::invalid_argument("Wheel diameter, motor/wheel ratio and transfer efficiency are required");
  return 3.141592653589793*diameter_m*motor_rpm/(60*ratio)*efficiency;
}

// Low-arc gravity solution in the mounted IMU navigation frame (+Z up).
// Recompute the muzzle lever arm as the requested P/Y orientation changes.
// No legacy camera offsets, pitch trim or assumed G+X barrel direction.
inline std::optional<BallisticGoal> ballistic_from_muzzle(
  const Eigen::Quaterniond& qNG, JointGoal current, const Eigen::Vector3d& point,
  const Eigen::Vector3d& muzzle, const Eigen::Vector3d& bore, double speed,
  double yaw_sign=1, double range_correction=0)
{
  if(!point.allFinite() || !muzzle.allFinite() || !std::isfinite(speed) || speed<=0) return {};
  JointGoal goal=current;
  for(int i=0;i<32;++i){
    const Eigen::Vector3d origin=goal_pose(qNG,current,goal,yaw_sign)*muzzle;
    const Eigen::Vector3d delta=point-origin;
    const double range=std::hypot(delta.x(),delta.y());
    const tools::Trajectory trajectory(speed,range+range_correction,delta.z());
    if(trajectory.unsolvable) return {};
    const Eigen::Vector3d ray(delta.x()/range*std::cos(trajectory.pitch),
      delta.y()/range*std::cos(trajectory.pitch),std::sin(trajectory.pitch));
    const auto next=inverse_boresight(qNG,current,ray,bore,yaw_sign);
    if(!next)return {};
    const double error=std::hypot(std::remainder(next->yaw-goal.yaw,2*M_PI),next->pitch-goal.pitch);
    goal=*next;
    if(error<1e-9) return BallisticGoal{goal,trajectory.fly_time,origin};
  }
  return {};
}
} // namespace io::rmv

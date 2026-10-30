#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace fixed_sim {
// Remaining horizontal time margin from ESTIMATED geometry/current encoder.
// Width and angular reserve are declared controller assumptions, not plant truth.
inline std::optional<double> horizontal_time_headroom(double phase,double omega,
    double r0,double r1,double range,double tracking_error,double width,double angular_reserve) {
  for(double x:{phase,omega,r0,r1,range,tracking_error,width,angular_reserve})
    if(!std::isfinite(x))return std::nullopt;
  if(std::abs(omega)<.1||r0<=.05||r0>=.5||r1<=.05||r1>=.5||range<=.1||range>30||
     width<=0||width>.3||angular_reserve<0||angular_reserve>.02||std::abs(tracking_error)>.1)
    return std::nullopt;
  const double usable=width/2-range*(std::abs(std::tan(tracking_error))+std::tan(angular_reserve));
  if(usable<=0)return std::nullopt;
  const double capacity=std::atan(usable/std::max(r0,r1));
  return (capacity-std::abs(std::remainder(phase,2*3.14159265358979323846)))/std::abs(omega);
}
// Experimental observation-only opportunity filter. Zero preserves the old
// policy; a nonzero bound can only REMOVE permissions, never create one.
inline bool centered_opportunity(double phase,double limit_deg) {
  if(!std::isfinite(limit_deg)||limit_deg<0||limit_deg>20)
    throw std::invalid_argument("center phase limit must be in [0,20] degrees");
  if(limit_deg==0)return true;
  return std::isfinite(phase)&&std::abs(std::remainder(phase,2*3.14159265358979323846))<=
    limit_deg*3.14159265358979323846/180;
}
// Select from fitted geometry only. A near-tie keeps both parities available;
// invalid geometry never authorizes a shot. World radii/identities are excluded.
inline bool preferred_radius_face(int face,double radius0,double radius1) {
  if(face<0||face>=4||!std::isfinite(radius0)||!std::isfinite(radius1)||
     radius0<=.05||radius0>=.5||radius1<=.05||radius1>=.5)return false;
  if(std::abs(radius0-radius1)<=.003)return true;
  return face%2==(radius0<radius1?0:1);
}
// Choose using the observed model rate, never the world's true rotation sign.
inline double directional_time_offset(double observed_omega,double fallback,double cw,double ccw) {
  if(!std::isfinite(observed_omega)||std::abs(observed_omega)<.1)return fallback;
  return observed_omega<0?cw:ccw;
}
// Observation/model-only, bounded host deadline. Callers must retain all gates
// and commit only before another observation can invalidate this decision.
inline std::optional<double> center_crossing_wait(double phase,double omega,double horizon) {
  if(!std::isfinite(phase)||!std::isfinite(omega)||!std::isfinite(horizon)||
     std::abs(omega)<.1||horizon<0||horizon>.05)return std::nullopt;
  const double wait=-std::remainder(phase,2*3.14159265358979323846)/omega;
  if(wait<0||wait>horizon)return std::nullopt;
  return wait;
}
}

#include "tasks/auto_aim/fire_pose_guard.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main()
{
  using auto_aim::fire_pose_guard;
  using auto_aim::FirePoseReject;
  using io::sentinel::pi;
  io::sentinel::PoseEstimate before, after;
  before.valid=true; before.boot_id=7; before.epoch=2; before.generation=3;
  after=before;
  const auto check=[&](double yaw_now=0.0,double yaw_command=0.0,
                       double pitch_now=0.0) {
    return fire_pose_guard(before,after,yaw_now,yaw_command,0.125,pitch_now,0.25);
  };
  assert(check().allowed);
  assert(check(0,0,0.25).allowed); // Existing boundary, no extra tuned threshold.
  assert(check(0,0,-0.25).allowed);
  assert(check(0,0,0.251).reason==FirePoseReject::pitch_error);
  assert(check(0,0,-0.251).reason==FirePoseReject::pitch_error);
  assert(check(0,0.125).allowed);
  assert(check(0,0.126).reason==FirePoseReject::yaw_error);
  const auto wrap=check(pi-0.02,-pi+0.02);
  assert(wrap.allowed && std::abs(wrap.yaw_error_rad-0.04)<1e-12);

  after.valid=false;assert(check().reason==FirePoseReject::invalid_pose);after=before;
  before.valid=false;assert(check().reason==FirePoseReject::invalid_pose);before=after;
  after.boot_id++;assert(check().reason==FirePoseReject::reference_changed);after=before;
  after.epoch++;assert(check().reason==FirePoseReject::reference_changed);after=before;
  after.generation++;assert(check().reason==FirePoseReject::reference_changed);after=before;

  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  assert(check(nan).reason==FirePoseReject::invalid_numeric);
  assert(check(0,inf).reason==FirePoseReject::invalid_numeric);
  assert(check(0,0,nan).reason==FirePoseReject::invalid_numeric);
  assert(check(0,0,inf).reason==FirePoseReject::invalid_numeric);
  assert(fire_pose_guard(before,after,0,0,-0.1,0,0.1).reason==FirePoseReject::invalid_numeric);
  assert(fire_pose_guard(before,after,0,0,0.1,0,-0.1).reason==FirePoseReject::invalid_numeric);
  assert(fire_pose_guard(before,after,0,0,inf,0,0.1).reason==FirePoseReject::invalid_numeric);
  assert(fire_pose_guard(before,after,0,0,0.1,0,nan).reason==FirePoseReject::invalid_numeric);
  assert(check(-std::numeric_limits<double>::max(),std::numeric_limits<double>::max()).reason==FirePoseReject::invalid_numeric);
  // A calibrated following reference must pass both the candidate and final
  // pose checks. The old uncorrected prediction would incorrectly reject it.
  const double native_yaw=.4, correction=-3.42*pi/180., trim=.2*pi/180.;
  const double reference=auto_aim::calibrated_fire_yaw(native_yaw,correction,trim);
  assert(std::abs(reference-(native_yaw+correction-trim))<1e-12);
  assert(check(reference,reference).allowed);
  assert(fire_pose_guard(before,after,reference,reference,1e-6,0,.25).allowed);
  assert(fire_pose_guard(before,after,reference,native_yaw,1e-6,0,.25).reason==FirePoseReject::yaw_error);
  assert(std::abs(auto_aim::calibrated_fire_yaw(pi-.01,.03,0)-(-pi+.02))<1e-12);
  assert(auto_aim::calibrated_fire_yaw(.5,0,0)==.5);
  assert(std::isnan(auto_aim::calibrated_fire_yaw(nan,0,0)));
  assert(std::isnan(auto_aim::calibrated_fire_yaw(0,inf,0)));
  assert(std::isnan(auto_aim::calibrated_fire_yaw(0,0,nan)));
  std::cout << "PASS post-wait fire pose: reference identity, yaw wrap, reprojected pitch residual, existing bounds, invalid inputs\n";
}

#include "io/rmv/geometry.hpp"
#include "io/rmv/joint_pose.hpp"
#include <cassert>
#include <iostream>
using namespace io::rmv;
using Q=Eigen::Quaterniond;
Q rot(double a,const Eigen::Vector3d& v){return Q(Eigen::AngleAxisd(a,v));}
int main(){
  {
    PoseHistory poses;JointPoseHistory joints;
    assert(poses.add({10000,1,1,{1,0,0,0}},30000));
    assert(poses.add({20000,2,1,{1,0,0,0}},30000));
    JointSample old;old.time=15000;old.moving={.2,.1};joints.add(old);
    JointSample next=old;next.time=25000;next.moving={.3,.2};joints.add(next);
    auto pair=joints.newest_bracketed(poses,30000);
    assert(pair && pair->joints.time==15000 && pair->joints.moving.yaw==.2);
    assert(poses.add({30000,3,1,{1,0,0,0}},30000));
    pair=joints.newest_bracketed(poses,30000);assert(pair && pair->joints.time==25000);
    assert(!joints.newest_bracketed(poses,130001)); // Stale pose remains blocked.
    joints.clear();assert(!joints.newest_bracketed(poses,30000)); // Invalid feedback clears fallback.
    next.time=90000;joints.add(next);assert(!joints.newest_bracketed(poses,30000));
    joints.clear();next.time=25000;next.maximum_feedback_age=99000;joints.add(next);
    assert(!joints.newest_bracketed(poses,30000)); // Include motor feedback age.
  }
  assert(debug_limit_radians(3.0)==3.0*debug_radians_per_degree);
  assert(debug_limit_radians(30.0)==30.0*debug_radians_per_degree);
  assert(debug_limit_radians(1.0)==debug_radians_per_degree);
  assert(debug_limit_radians(.00001)>0);
  assert(debug_limit_radians(0)==0);
  for(double invalid:{-1.,30.0001,std::nextafter(30.,INFINITY),double(NAN),double(INFINITY)}){
    bool rejected=false;
    try{debug_limit_radians(invalid);}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  const Eigen::Vector3d bore(.9631717485778611,.006317492260651429,-.26881270809432134);
  const Eigen::Vector3d muzzle(.12635586983173663,.0007757848490019296,-.030554742953153517);
  // A repeatable high impact calls for a bore bias distinct from bullet speed.
  // Exercise below-axis targets and tilted installations with an independent
  // forward projectile integration, not just an inverse/forward angle mirror.
  unsigned trim_cases=0;
  for(double trim:{-0.95,0.,0.95})for(double roll:{-.08,0.,.08})
    for(double height:{-.3,-.07,0.,.3}){
      const Eigen::Vector3d actual_bore=bore_for_pitch_trim(bore,trim);
      const Q base=rot(.4,Eigen::Vector3d::UnitZ())*rot(roll,Eigen::Vector3d::UnitX());
      const JointGoal current{-.2,.25};
      const Q pose=base*rot(current.yaw,Eigen::Vector3d::UnitZ())*rot(-current.pitch,Eigen::Vector3d::UnitY());
      const Eigen::Vector3d point=base*Eigen::Vector3d(3.,0.,height);
      const auto nominal=ballistic_from_muzzle(pose,current,point,muzzle,bore,26.6);
      const auto adjusted=ballistic_from_muzzle(pose,current,point,muzzle,actual_bore,26.6);
      assert(nominal && adjusted);
      if(trim<0)assert(adjusted->joints.pitch<nominal->joints.pitch);
      if(trim>0)assert(adjusted->joints.pitch>nominal->joints.pitch);
      const Q final=goal_pose(pose,current,adjusted->joints,1);
      const Eigen::Vector3d hit=final*muzzle+26.6*adjusted->flight_time*(final*actual_bore)
        -Eigen::Vector3d(0,0,.5*9.7833*adjusted->flight_time*adjusted->flight_time);
      assert((hit-point).norm()<1e-7);
      ++trim_cases;
    }
  for(double bad:{double(NAN),double(INFINITY),-5.01,5.01}){
    bool rejected=false;
    try{bore_for_pitch_trim(bore,bad);}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  unsigned cases=0;
  for(double sign:{-1.,1.})for(double fixed_small:{-.3,0.,.4})
  for(double tilt:{-.2,0.,.2})for(double yaw:{-.6,0.,.4})for(double pitch:{-.35,0.,.25}){
    Q base=rot(tilt,Eigen::Vector3d::UnitX())*rot(.17,Eigen::Vector3d::UnitY())*rot(-fixed_small,Eigen::Vector3d::UnitZ());
    JointGoal current{yaw,pitch};
    Q pose=base*rot(sign*yaw,Eigen::Vector3d::UnitZ())*rot(-pitch,Eigen::Vector3d::UnitY());
    auto hold=inverse_boresight(pose,current,pose*bore,bore,sign);
    assert(hold && std::hypot(hold->yaw-yaw,hold->pitch-pitch)<1e-9);
    for(double dy:{-30*debug_radians_per_degree,-.03,0.,.03,30*debug_radians_per_degree})
      for(double dp:{-30*debug_radians_per_degree,-.03,0.,.03,30*debug_radians_per_degree})
      for(double range:{.5,2.,8.}){
      JointGoal wanted{yaw+dy,pitch+dp};
      Q future=base*rot(sign*wanted.yaw,Eigen::Vector3d::UnitZ())*rot(-wanted.pitch,Eigen::Vector3d::UnitY());
      assert(goal_pose(pose,current,wanted,sign).angularDistance(future)<1e-9);
      Eigen::Vector3d point=future*(muzzle+range*bore);
      auto got=point_from_muzzle(pose,current,point,muzzle,bore,sign);
      assert(got && std::hypot(got->yaw-wanted.yaw,got->pitch-wanted.pitch)<2e-7);
      assert((goal_pose(pose,current,*got,sign)*bore).cross(point-goal_pose(pose,current,*got,sign)*muzzle).norm()<1e-6);
      ++cases;
    }
  }
  assert(!inverse_boresight(Q::Identity(),{0,0},Eigen::Vector3d::Zero(),bore));
  assert(!inverse_boresight(Q::Identity(),{0,0},Eigen::Vector3d::UnitZ(),bore));
  assert(!inverse_boresight(Q::Identity(),{0,0},bore,2*bore));
  assert(!point_from_muzzle(Q::Identity(),{0,0},{NAN,0,0},muzzle,bore));
  // Independent forward projectile integration checks signs, barrel mounting,
  // moving muzzle origin and gravity, including a tilted base and yaw wrap.
  unsigned ballistic_cases=0;
  for(double roll:{-.08,0.,.08})for(double base_yaw:{-3.12,.4,3.12})
    for(double range:{1.,3.,6.})for(double height:{-.3,0.,.5}){
      const Q base=rot(base_yaw,Eigen::Vector3d::UnitZ())*rot(roll,Eigen::Vector3d::UnitX());
      const JointGoal current{3.10,.22};
      const Q pose=base*rot(current.yaw,Eigen::Vector3d::UnitZ())*rot(-current.pitch,Eigen::Vector3d::UnitY());
      const Eigen::Vector3d point=base*Eigen::Vector3d(range,range*.08,height);
      const auto result=ballistic_from_muzzle(pose,current,point,muzzle,bore,10.);
      assert(result && result->flight_time>0);
      const Q final=goal_pose(pose,current,result->joints,1);
      const Eigen::Vector3d origin=final*muzzle;
      const Eigen::Vector3d hit=origin+10.*result->flight_time*(final*bore)
        -Eigen::Vector3d(0,0,.5*9.7833*result->flight_time*result->flight_time);
      assert((hit-point).norm()<1e-7);
      ++ballistic_cases;
    }
  assert(!ballistic_from_muzzle(Q::Identity(),{0,0},{3,0,0},muzzle,bore,0));
  assert(!ballistic_from_muzzle(Q::Identity(),{0,0},{30,0,10},muzzle,bore,5));
  assert(std::abs(estimate_projectile_speed(.1,2500,1,.8)-10.471975511965978)<1e-10);
  assert(std::abs(estimate_projectile_speed(.1,2500,19,.8)-10.471975511965978/19)<1e-10);
  bool unknown_ratio_rejected=false;
  try{estimate_projectile_speed(.1,2500,0,.8);}catch(const std::invalid_argument&){unknown_ratio_rejected=true;}
  assert(unknown_ratio_rejected);
  std::cout<<"ballistic forward integration: "<<ballistic_cases<<" cases\n";
  std::cout<<"empirical bore trim, including below-axis targets: "<<trim_cases<<" cases\n";
  std::cout<<"rmv_geometry_test: PASS "<<cases<<" muzzle/boresight cases\n";
}

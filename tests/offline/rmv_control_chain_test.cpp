// Actual estimator, installed geometry, centered MPC and wire trajectory.
// Synthetic observations only: no camera, serial port, motor or firing.
#include "tasks/auto_aim/rmv_rotating.hpp"
#include "tools/math_tools.hpp"
#include <cassert>
#include <iostream>

using namespace auto_aim;
Armor observation(double t,double rate,double range,int& id){
  const double bearing=std::atan2(.2*t,range+.25);
  double best=INFINITY;id=0;
  for(int k=0;k<4;++k){double a=std::abs(std::remainder(rate*t+k*M_PI/2-bearing,2*M_PI));
    if(a<best){best=a;id=k;}}
  const double phase=rate*t+id*M_PI/2;
  Armor a(2,.99f,{600,400,80,35},{{600,400},{680,400},{680,435},{600,435}},YOLOVersion::YOLO11);
  a.xyz_in_world={range+.25-.25*std::cos(phase),.2*t-.25*std::sin(phase),-.067};
  a.ypr_in_world={tools::limit_rad(phase),0,0};a.ypd_in_world=tools::xyz2ypd(a.xyz_in_world);
  return a;
}
int main(){
  auto cfg=YAML::LoadFile("configs/standard.yaml");
  Planner planner("configs/standard.yaml");
  const auto camera=cfg["gun_forward_in_gimbal"].as<std::vector<double>>();
  const auto muzzle_v=cfg["t_muzzle2gimbal"].as<std::vector<double>>();
  Eigen::Vector3d muzzle(muzzle_v.data()),nominal(camera.data());
  const auto bore=io::rmv::bore_for_pitch_trim(nominal,cfg["rmv_ballistic_pitch_trim_deg"].as<double>());
  unsigned cases=0;
  for(double range:{3.,5.})for(double rate:{-1.8,-.15,.15,1.8}){
    int id=0;auto a=observation(0,rate,range,id);
    Target target(a,std::chrono::steady_clock::time_point{},.25,4,Eigen::VectorXd::Ones(11));
    RmvRotatingAim aim(cfg,26.6);io::rmv::JointGoal current{0,.25};
    RmvRotatingAim::Solution s;Eigen::Quaterniond pose;
    for(int step=1;step<=700;++step){
      double t=step*.01;a=observation(t,rate,range,id);target.predict(.01);target.update(a,id);
      pose=Eigen::Quaterniond(Eigen::AngleAxisd(current.yaw,Eigen::Vector3d::UnitZ()))*
        Eigen::Quaterniond(Eigen::AngleAxisd(-current.pitch,Eigen::Vector3d::UnitY()));
      s=aim.update(&target,true,10+t,10+t+.01,pose,current,muzzle,bore,&a.xyz_in_world);
      if(step<700 && s.goal)current=*s.goal;
    }
    assert(s.converged && s.yaw_trusted && s.goal && s.face_id>=0);
    if(std::abs(rate)<.2){
      assert(s.model.mode==RotationModelManager::Mode::armor && s.direct_motion);
      assert(s.face_id==target.last_id);
    }
    const auto before=target.ekf_x();const auto covariance=target.ekf().P;
    const auto ref=aim.trajectory(target,s,.01,pose,current,muzzle,bore);
    assert(ref && ref->allFinite());
    assert(std::abs(std::remainder((*ref)(0,HALF_HORIZON)-s.goal->yaw,2*M_PI))<1e-5);
    assert(std::abs((*ref)(2,HALF_HORIZON)-s.goal->pitch)<1e-5);
    const auto p=planner.plan_joint_trajectory(*ref,current.yaw);assert(p.control && !p.fire);
    assert(std::abs(std::remainder(p.target_yaw-s.goal->yaw,2*M_PI))<1e-5);
    assert(std::abs(p.target_pitch-s.goal->pitch)<1e-5);
    assert(std::abs(p.yaw_vel)<.2);
    if(std::abs(rate)>1)assert(std::abs(p.yaw_vel)>.01);
    assert(std::abs(std::remainder(p.yaw-s.goal->yaw,2*M_PI))<.01);
    assert(std::abs(p.pitch-s.goal->pitch)<.01);
    assert(target.ekf_x().isApprox(before,1e-14) && target.ekf().P.isApprox(covariance,1e-14));
    std::cout<<"range="<<range<<" omega="<<s.omega<<" yaw_rate="<<p.yaw_vel<<"\n";++cases;
  }
  // Fixed physical position with independent orientation noise.
  {
    int id=0;auto a=observation(0,0,3,id);
    Target target(a,std::chrono::steady_clock::time_point{},.25,4,Eigen::VectorXd::Ones(11));
    RmvRotatingAim aim(cfg,26.6);io::rmv::JointGoal current{0,.25};
    const Eigen::Quaterniond pose(Eigen::AngleAxisd(-current.pitch,Eigen::Vector3d::UnitY()));
    RmvRotatingAim::Solution s;
    for(int step=1;step<=500;++step){
      const double t=.01*step;
      a.xyz_in_world={3,.001*std::sin(2*M_PI*8*t),-.067};
      a.ypr_in_world={.025*std::sin(2*M_PI*t),0,0};
      a.ypd_in_world=tools::xyz2ypd(a.xyz_in_world);
      target.predict(.01);target.update(a,0);
      s=aim.update(&target,true,10+t,10+t+.02,pose,current,muzzle,bore,&a.xyz_in_world);
    }
    assert(s.goal && s.converged && s.yaw_trusted && s.direct_motion && s.face_id==0);
    const auto x=target.ekf_x();const auto covariance=target.ekf().P;
    const auto direct_now=target.armor_xyza_list_at(0,true)[0].head<3>().eval();
    const auto velocity=target.armor_motion_velocity(0);
    for(double dt:{-.5,.0,.2,.5}){
      const auto sampled=target.armor_xyza_list_at(dt,true);
      assert(sampled[0].head<3>().isApprox(direct_now+velocity*dt,1e-12));
    }
    const auto ref=aim.trajectory(target,s,.02,pose,current,muzzle,bore);
    assert(ref && std::abs(std::remainder((*ref)(0,HALF_HORIZON)-s.goal->yaw,2*M_PI))<1e-5);
    assert(std::abs((*ref)(1,HALF_HORIZON))<.01);
    assert(target.ekf_x().isApprox(x,1e-14) && target.ekf().P.isApprox(covariance,1e-14));
    std::cout<<"PASS noisy stationary face: immutable CV sampling and same impact/trajectory source\n";
  }
  // Constant-rate yaw crossing the encoder discontinuity must not produce a
  // 2*pi jump or accidentally take the 500 ms future MPC output.
  Trajectory wrap;
  for(int i=0;i<HORIZON;++i)wrap.col(i)<<tools::limit_rad(3.13+.4*DT*(i-HALF_HORIZON)),.4,.2,0;
  const auto p=planner.plan_joint_trajectory(wrap,3.13);
  assert(p.control && std::abs(std::remainder(p.yaw-3.13,2*M_PI))<.002 && std::abs(p.yaw_vel-.4)<.005);
  wrap(1,0)=NAN;assert(!planner.plan_joint_trajectory(wrap,3.13).control);
  // MCU/host must agree on bounded polynomial integration, without changing
  // native velocity caps or treating the P speed cap as signed feedforward.
  io::rmv::JointGoal g{0,0,1.9,0,100,0,true};g=io::rmv::advance_trajectory(g,.03);
  assert(std::abs(g.yaw-.05995)<1e-12 && g.yaw_rate==2);
  std::cout<<"PASS "<<cases<<" moving 4-armor cases, exact impact center, centered MPC, yaw wrap\n";
}

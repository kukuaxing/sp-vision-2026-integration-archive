#include "tasks/auto_aim/rmv_rotating.hpp"
#include "tools/math_tools.hpp"
#include <cassert>
#include <iostream>

using namespace auto_aim;
Armor observed(double time,double rate,int& face){
  double best=INFINITY;face=0;
  for(int id=0;id<4;++id){const double a=std::abs(std::remainder(rate*time+id*M_PI/2,2*M_PI));if(a<best){best=a;face=id;}}
  const double angle=std::remainder(rate*time+face*M_PI/2,2*M_PI);
  Armor a(2,.99f,{600,400,80,35},{{600,400},{680,400},{680,435},{600,435}},YOLOVersion::YOLO11);
  a.xyz_in_world={3.25-.25*std::cos(angle),-.25*std::sin(angle),0};
  a.ypr_in_world={angle,0,0};a.ypd_in_world=tools::xyz2ypd(a.xyz_in_world);
  return a;
}
int main(){
  const auto cfg=YAML::LoadFile("configs/standard.yaml");
  const Eigen::Vector3d muzzle(.1263558698,.0007757848,-.0305547430),nominal_bore(.9631717486,.0063174923,-.2688127081);
  const auto bore=io::rmv::bore_for_pitch_trim(nominal_bore,
    cfg["rmv_ballistic_pitch_trim_deg"]?cfg["rmv_ballistic_pitch_trim_deg"].as<double>():0);
  unsigned high_locked=0,qualified=0;
  for(double rate:{-4.,0.,1.8,4.}){
    int id=0;auto a=observed(0,rate,id);
    Target target(a,std::chrono::steady_clock::time_point{},.25,4,Eigen::VectorXd::Ones(11));
    RmvRotatingAim aim(cfg,10.);
    io::rmv::JointGoal current{0,.28};
    RmvRotatingAim::Solution s;
    unsigned local_qualified=0;
    for(int step=1;step<=1200;++step){
      const double t=step*.01;
      a=observed(t,rate,id);target.predict(.01);target.update(a,id);
      const auto pose=Eigen::Quaterniond(Eigen::AngleAxisd(current.yaw,Eigen::Vector3d::UnitZ()))*
        Eigen::Quaterniond(Eigen::AngleAxisd(-current.pitch,Eigen::Vector3d::UnitY()));
      const auto before=target.ekf_x();
      s=aim.update(&target,true,10+t,10+t+.01,pose,current,muzzle,bore);
      assert((target.ekf_x()-before).norm()==0); // scheduling samples never advance the live estimator
      if(s.goal){assert(std::isfinite(s.goal->yaw)&&std::isfinite(s.goal->pitch));current=*s.goal;}
      if(s.qualified)++local_qualified;
    }
    assert(s.converged&&s.yaw_trusted);
    if(std::abs(rate)>3.2){assert(aim.phase_locked()&&s.event.valid);++high_locked;assert(local_qualified>0);}
    if(rate==0){
      assert(s.model.mode==RotationModelManager::Mode::armor && s.qualified && local_qualified>0);
      const auto aligned=aim.update(&target,true,22,22.01,
        Eigen::Quaterniond(Eigen::AngleAxisd(current.yaw,Eigen::Vector3d::UnitZ()))*
        Eigen::Quaterniond(Eigen::AngleAxisd(-current.pitch,Eigen::Vector3d::UnitY())),current,muzzle,bore);
      assert(aligned.qualified);
      const io::rmv::JointGoal off_current{current.yaw,current.pitch+.05};
      const auto off_bore=aim.update(&target,true,22,22.01,
        Eigen::Quaterniond(Eigen::AngleAxisd(off_current.yaw,Eigen::Vector3d::UnitZ()))*
        Eigen::Quaterniond(Eigen::AngleAxisd(-off_current.pitch,Eigen::Vector3d::UnitY())),
        off_current,muzzle,bore);
      assert(!off_bore.qualified && off_bore.reason=="gun_not_on_future_armor");
    }
    if(rate!=0)assert(std::abs(s.aim_point.y())<.04); // center bearing through four physical faces
    qualified+=local_qualified;
    const auto pose=Eigen::Quaterniond(Eigen::AngleAxisd(current.yaw,Eigen::Vector3d::UnitZ()))*
      Eigen::Quaterniond(Eigen::AngleAxisd(-current.pitch,Eigen::Vector3d::UnitY()));
    if(rate!=0){const auto hold=aim.update(nullptr,false,22.02,22.03,pose,current,muzzle,bore);assert(hold.goal&&!hold.qualified);}
    assert(!aim.update(nullptr,false,23,23.01,pose,current,muzzle,bore).goal);
    RmvRotatingAim unknown(cfg,0);
    const auto no_speed=unknown.update(&target,true,22,22.01,pose,current,muzzle,bore);
    assert(no_speed.goal&&!no_speed.qualified&&no_speed.reason=="projectile_speed_unset");
    assert(!unknown.update(&target,true,22,22.081,pose,current,muzzle,bore).goal);
    std::cout<<"rate="<<rate<<" omega="<<s.omega<<" qualified="<<local_qualified<<" reason="<<s.reason<<"\n";
  }
  assert(high_locked==2&&qualified>0);
  // Independent constant 25-degree plate observations: the old 15-degree
  // face window rejects a valid stationary armor; the field 30-degree window
  // allows it only after the model converges and the bore is aligned.
  {
    const double angle=25*io::rmv::debug_radians_per_degree;
    int id=0;auto a=observed(0,0,id);
    a.xyz_in_world={3.25-.25*std::cos(angle),-.25*std::sin(angle),0};
    a.ypr_in_world={angle,0,0};a.ypd_in_world=tools::xyz2ypd(a.xyz_in_world);
    Target target(a,std::chrono::steady_clock::time_point{},.25,4,Eigen::VectorXd::Ones(11));
    auto narrow_cfg=YAML::Clone(cfg);narrow_cfg["rmv_fire_face_max_alignment_deg"]=15.;
    auto field_cfg=YAML::Clone(cfg);field_cfg["rmv_fire_face_max_alignment_deg"]=30.;
    RmvRotatingAim narrow(narrow_cfg,26.6),field(field_cfg,26.6);
    io::rmv::JointGoal current{0,.28};
    RmvRotatingAim::Solution old_s,new_s;
    for(int step=1;step<=300;++step){
      target.predict(.01);target.update(a,0);
      const auto pose=Eigen::Quaterniond(Eigen::AngleAxisd(current.yaw,Eigen::Vector3d::UnitZ()))*
        Eigen::Quaterniond(Eigen::AngleAxisd(-current.pitch,Eigen::Vector3d::UnitY()));
      old_s=narrow.update(&target,true,30+step*.01,30+step*.01+.01,pose,current,muzzle,bore);
      new_s=field.update(&target,true,30+step*.01,30+step*.01+.01,pose,current,muzzle,bore);
      if(new_s.goal)current=*new_s.goal;
    }
    assert(!old_s.qualified && old_s.reason=="future_face_ambiguous");
    assert(new_s.qualified && new_s.face_alignment>24*io::rmv::debug_radians_per_degree &&
      new_s.face_alignment<27*io::rmv::debug_radians_per_degree);
    for(double bad:{0.,90.,double(NAN)}){
      auto bad_cfg=YAML::Clone(cfg);bad_cfg["rmv_fire_face_max_alignment_deg"]=bad;
      bool rejected=false;try{RmvRotatingAim invalid(bad_cfg,26.6);}catch(const std::runtime_error&){rejected=true;}
      assert(rejected);
    }
  }
  std::cout<<"PASS: real Target/ESKF -> mode/PLL/face -> measured-bore ballistic joints; no devices\n";
}

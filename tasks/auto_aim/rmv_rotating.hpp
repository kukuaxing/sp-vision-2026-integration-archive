#pragma once
#include "target.hpp"
#include "gyro_phase_scheduler.hpp"
#include "gyro_face_selector.hpp"
#include "rotation_model_manager.hpp"
#include "robust_range_filter.hpp"
#include "planner/planner.hpp"
#include "io/rmv/geometry.hpp"
#include <yaml-cpp/yaml.h>

namespace auto_aim {
// The same rigid-body estimator, motion-regime manager, crossing PLL and
// per-cycle face commitment as standard's maintained SP/XUC path. The adapter
// replaces its old installation/empirical corrections with this vehicle's G
// pose, muzzle and barrel geometry. It never talks to serial or fires a motor.
class RmvRotatingAim {
public:
  struct Solution {
    std::optional<io::rmv::JointGoal> goal;
    Eigen::Vector3d aim_point=Eigen::Vector3d::Zero(),impact_face=Eigen::Vector3d::Zero();
    RotationModelManager::Result model;
    GyroPhaseScheduler::Solution event;
    bool converged=false,yaw_trusted=false,face_valid=false,qualified=false;
    bool direct_motion=false;
    int face_id=-1;
    double omega=0,flight_time=0,face_alignment=0,yaw_error=0,pitch_error=0;
    double filtered_range=0,range_correction=0;
    std::string reason="no_fresh_target";
  };
private:
  RotationModelManager manager_;
  GyroPhaseScheduler clock_;
  GyroFaceSelector faces_;
  RobustRangeFilter range_;
  double range_image_s_=-1,range_correction_=0;
  double speed_,delay_,delay_min_,delay_max_,flight_uncertainty_,width_,height_,margin_,phase_uncertainty_,grace_;
  double face_alignment_max_;
  bool require_face_consistency_;
  int target_id_=-1;
  std::optional<Eigen::Vector3d> last_center_point_;
  double last_center_s_=0;
  static double number(const YAML::Node& c,const char* key,double fallback){return c[key]?c[key].as<double>():fallback;}
  static double face_limit(const YAML::Node& c){
    const double degrees=number(c,"rmv_fire_face_max_alignment_deg",number(c,"xuc_event_face_max_alignment_deg",15));
    if(!std::isfinite(degrees) || degrees<=0 || degrees>=90)
      throw std::runtime_error("Invalid RMV armor face alignment limit");
    return degrees*io::rmv::debug_radians_per_degree;
  }
  static RotationModelManager::Config motion_config(const YAML::Node& c){
    RotationModelManager::Config m;
    m.vehicle_enter_rad_s=number(c,"xuc_gyro_omega_threshold_rad_s",.8);
    m.vehicle_exit_rad_s=number(c,"xuc_vehicle_exit_rad_s",.5);
    m.high_speed_enter_rad_s=number(c,"xuc_high_speed_enter_rad_s",3.2);
    m.high_speed_exit_rad_s=number(c,"xuc_high_speed_exit_rad_s",2.5);
    m.mode_dwell_s=number(c,"xuc_rotation_mode_dwell_ms",600)*.001;
    m.tracker_dropout_hold_s=number(c,"xuc_rotation_tracker_dropout_hold_ms",250)*.001;
    m.rate_filter_tau_s=number(c,"xuc_rotation_rate_filter_tau_ms",100)*.001;
    m.max_rate_disagreement_rad_s=number(c,"xuc_rotation_rate_disagreement_rad_s",2.2);
    m.speed_step_reset_rad_s=number(c,"xuc_rotation_speed_step_reset_rad_s",2.5);
    m.rate_agreement_updates=static_cast<int>(number(c,"xuc_rotation_rate_agreement_updates",3));
    m.rate_disagreement_updates=static_cast<int>(number(c,"xuc_rotation_rate_disagreement_updates",18));
    return m;
  }
  static RobustRangeFilter::Config range_config(const YAML::Node& c){
    RobustRangeFilter::Config r;
    r.time_constant_s=number(c,"xuc_range_filter_tau_ms",80)*.001;
    r.max_rate_mps=number(c,"xuc_range_filter_max_rate_mps",6);
    r.raw_blend=number(c,"xuc_range_filter_raw_blend",.05);
    r.max_raw_model_residual_m=number(c,"xuc_range_filter_max_raw_residual_m",.4);
    return r;
  }
  std::optional<io::rmv::BallisticGoal> ballistic(const Eigen::Quaterniond& pose,
    io::rmv::JointGoal current,const Eigen::Vector3d& point,const Eigen::Vector3d& muzzle,
    const Eigen::Vector3d& bore)const{
    return io::rmv::ballistic_from_muzzle(pose,current,point,muzzle,bore,speed_,1,range_correction_);
  }
  static GyroPhaseScheduler::Config phase_config(const YAML::Node& c){
    GyroPhaseScheduler::Config p;
    constexpr double d=io::rmv::debug_radians_per_degree;
    p.crossing_capture_rad=number(c,"xuc_event_crossing_capture_deg",20)*d;
    p.crossing_rearm_rad=number(c,"xuc_event_crossing_rearm_deg",12)*d;
    p.min_crossing_separation_ratio=number(c,"xuc_event_min_crossing_separation_ratio",.55);
    p.min_face_period_s=number(c,"xuc_event_period_min_ms",120)*.001;
    p.max_face_period_s=number(c,"xuc_event_period_max_ms",1200)*.001;
    p.period_ema_alpha=number(c,"xuc_event_period_ema_alpha",.2);
    p.max_period_relative_error=number(c,"xuc_event_max_period_relative_error",.22);
    p.max_period_step_ratio=number(c,"xuc_event_max_period_step_ratio",.025);
    p.omega_period_weight=number(c,"xuc_event_omega_period_weight",.15);
    p.period_residual_alpha=number(c,"xuc_event_period_residual_alpha",.08);
    p.phase_correction_alpha=number(c,"xuc_event_phase_correction_alpha",.15);
    p.max_phase_correction_s=number(c,"xuc_event_max_phase_correction_ms",3)*.001;
    p.max_phase_residual_s=number(c,"xuc_event_max_phase_residual_ms",90)*.001;
    p.period_window_size=static_cast<int>(number(c,"xuc_event_period_window_size",7));
    p.phase_residual_window_size=static_cast<int>(number(c,"xuc_event_phase_residual_window_size",5));
    p.phase_uncertainty_floor_s=number(c,"xuc_event_phase_uncertainty_floor_ms",2)*.001;
    p.min_lock_updates=static_cast<int>(number(c,"xuc_event_min_lock_updates",3));
    p.relock_after_rejections=static_cast<int>(number(c,"xuc_event_relock_rejections",3));
    return p;
  }
public:
  RmvRotatingAim(const YAML::Node& c,double speed)
  :manager_(motion_config(c)),clock_(phase_config(c)),
    faces_(face_limit(c)),range_(range_config(c)),
    speed_(speed),delay_(number(c,"rmv_fire_delay_s",.085)),
    delay_min_(number(c,"rmv_fire_delay_min_s",.065)),delay_max_(number(c,"rmv_fire_delay_max_s",.105)),
    flight_uncertainty_(number(c,"rmv_flight_time_uncertainty_s",.05)),
    width_(number(c,"xuc_event_armor_width_m",.135)),height_(number(c,"rmv_armor_height_m",.055)),
    margin_(number(c,"xuc_event_armor_edge_margin_m",0)),
    phase_uncertainty_(number(c,"xuc_event_model_phase_uncertainty_deg",.7)*io::rmv::debug_radians_per_degree),
    grace_(number(c,"xuc_event_phase_grace_deg",4)*io::rmv::debug_radians_per_degree),
    face_alignment_max_(face_limit(c)),
    require_face_consistency_(c["xuc_event_require_face_consistency"]?c["xuc_event_require_face_consistency"].as<bool>():true){
    for(double v:{speed_,delay_,delay_min_,delay_max_,flight_uncertainty_,width_,height_,margin_,phase_uncertainty_,grace_})
      if(!std::isfinite(v))throw std::runtime_error("Non-finite RMV rotating-target configuration");
    if(speed_<0 || (speed_>0 && speed_<1) || speed_>35 || delay_min_<0 || delay_min_>delay_ || delay_>delay_max_ ||
       delay_max_>.5 || flight_uncertainty_<0 || width_<=0 || height_<=0 || margin_<0 || margin_>=width_*.5)
      throw std::runtime_error("Invalid RMV rotating-target speed/timing/armor dimensions");
  }
  void reset(){manager_.reset();clock_.reset();faces_.reset();range_.reset();range_image_s_=-1;
    range_correction_=0;target_id_=-1;last_center_point_.reset();}
  Solution update(Target* target,bool observed,double image_s,double now_s,
    const Eigen::Quaterniond& pose,io::rmv::JointGoal current,
    const Eigen::Vector3d& muzzle,const Eigen::Vector3d& bore,
    const Eigen::Vector3d* raw_point=nullptr){
    Solution s;
    if(!std::isfinite(image_s) || !std::isfinite(now_s) || now_s<image_s || now_s-image_s>.08){reset();return s;}
    if(!target || !observed){
      s.model=manager_.update(image_s,false,0);
      if(s.model.phase_reset){clock_.reset();faces_.reset();}
      if(last_center_point_ && now_s>=last_center_s_ && now_s-last_center_s_<=.3){
        s.aim_point=*last_center_point_;s.reason="brief_visual_gap_center_hold";
        if(speed_>0){const auto g=ballistic(pose,current,s.aim_point,muzzle,bore);if(g)s.goal=g->joints;}
        else s.goal=io::rmv::point_from_muzzle(pose,current,s.aim_point,muzzle,bore,1);
      }else reset();
      return s;
    }
    if(target_id_!=static_cast<int>(target->name)){reset();target_id_=static_cast<int>(target->name);}
    const auto x=target->ekf_x();
    s.converged=target->convergened() && !target->diverged();
    s.yaw_trusted=target->yaw_eskf_trusted();
    if(x.size()<11 || !x.allFinite() || target->armor_num()!=4){s.reason="requires_four_armor_model";return s;}
    s.omega=x[7];
    s.model=manager_.update(image_s,s.converged && s.yaw_trusted,s.omega,
      clock_.period_valid(),clock_.period_valid()?(M_PI/2)/clock_.face_period_s():NAN);
    if(s.model.phase_reset){clock_.reset();faces_.reset();}
    if(!s.converged || !s.yaw_trusted){s.reason="tracker_or_yaw_warming_up";return s;}
    const double age=now_s-image_s;
    const double bearing=std::atan2(x[2],x[0]);
    const bool rotating=s.model.mode!=RotationModelManager::Mode::armor;
    if(s.model.mode==RotationModelManager::Mode::high_speed_phase)
      clock_.observe(image_s,std::remainder(x[6]-bearing,M_PI/2),s.omega);
    else {clock_.reset();faces_.reset();}
    const auto state_now=target->ekf_x_at(age);
    Eigen::Vector3d center(state_now[0],state_now[2],state_now[4]);
    const double radius=std::max(x[8],x[8]+x[9]);
    if(radius<.05 || radius>.5 || center.norm()<.5 || center.norm()>20){s.reason="invalid_model_geometry";return s;}
    // Center-follow targets the front radius on the center bearing. The gun
    // remains on that bearing through plate handovers; no extra speed ramp.
    const double center_bearing=std::atan2(center.y(),center.x());
    Eigen::Vector3d point=center-Eigen::Vector3d(radius*std::cos(center_bearing),radius*std::sin(center_bearing),0);
    // As in standard: a rotating model supplies the stable front-center
    // range; armor mode uses its observed physical face. Raw PnP is bounded.
    if(image_s!=range_image_s_){
      const auto visible=target->armor_xyza_list_at(age,!rotating);
      const Eigen::Vector3d origin=pose*muzzle;
      const Eigen::Vector3d model_point=!rotating && target->last_id>=0 && target->last_id<int(visible.size())?
        Eigen::Vector3d(visible[target->last_id].head<3>()):point;
      const double model_range=(model_point-origin).head<2>().norm();
      const double raw_range=raw_point?(*raw_point-origin).head<2>().norm():NAN;
      const auto r=range_.update(image_s,model_range,raw_range);
      if(r.valid)range_correction_=r.range_m-model_range;
      range_image_s_=image_s;
    }
    s.filtered_range=range_.initialized()?range_.value_m():0;s.range_correction=range_correction_;
    if(speed_==0){
      const auto visible=target->armor_xyza_list_at(age);
      if(target->last_id>=0 && target->last_id<static_cast<int>(visible.size()))point.z()=visible[target->last_id].z();
      s.aim_point=point;s.goal=io::rmv::point_from_muzzle(pose,current,point,muzzle,bore,1);
      if(rotating && s.goal){last_center_point_=point;last_center_s_=now_s;}
      s.reason="projectile_speed_unset";return s;
    }
    double horizon=age+delay_;
    for(int i=0;i<4;++i){
      const auto trj=ballistic(pose,current,point,muzzle,bore);
      if(!trj){s.reason="ballistics_unsolvable";return s;}
      s.flight_time=trj->flight_time;
      if(s.model.mode==RotationModelManager::Mode::high_speed_phase){
        s.event=clock_.solve(now_s,delay_,delay_min_,delay_max_,s.flight_time,flight_uncertainty_,
          radius,width_,margin_,phase_uncertainty_);
        if(s.event.valid)horizon=age+s.event.time_until_hit_center_s;
        else horizon=age+delay_+s.flight_time;
      }else horizon=age+delay_+s.flight_time;
      const auto future=target->armor_xyza_list_at(horizon,!rotating);
      const auto future_x=target->ekf_x_at(horizon);
      const double future_bearing=std::atan2(future_x[2],future_x[0]);
      double best=INFINITY;int id=-1;
      if(s.event.valid){
        std::vector<GyroFaceSelector::Candidate> candidates;
        for(unsigned k=0;k<future.size();++k)candidates.push_back({static_cast<int>(k),future[k][3]});
        const auto selection=faces_.select(candidates,future_bearing,s.event.hit_cycle);
        if(selection.valid){id=selection.face_id;best=selection.alignment_rad;}
      }else if(!rotating && target->last_id>=0 && target->last_id<int(future.size()) &&
        target->armor_motion_trusted(target->last_id) &&
        std::abs(std::remainder(future[target->last_id][3]-future_bearing,2*M_PI))<=75*M_PI/180){
        // Match Aimer's low-speed policy: keep the observed physical face.
        // The stricter fire-facing gate below remains unchanged.
        id=target->last_id;best=std::abs(std::remainder(future[id][3]-future_bearing,2*M_PI));
      }else for(unsigned k=0;k<future.size();++k){
        const double a=std::abs(std::remainder(future[k][3]-future_bearing,2*M_PI));
        if(a<best){best=a;id=static_cast<int>(k);}
      }
      s.face_id=id;s.face_alignment=std::isfinite(best)?best:0;
      s.direct_motion=!rotating && target->armor_motion_trusted(id);
      s.face_valid=id>=0 && best<=face_alignment_max_;
      if(id>=0)s.impact_face=future[id].head<3>();
      if(rotating){
        const double r=id>=0?std::hypot(future[id][0]-future_x[0],future[id][1]-future_x[2]):radius;
        point={future_x[0]-r*std::cos(future_bearing),future_x[2]-r*std::sin(future_bearing),
          id>=0?future[id][2]:future_x[4]};
      }else if(id>=0)point=future[id].head<3>();
    }
    const auto control=ballistic(pose,current,point,muzzle,bore);
    if(!control){s.reason="ballistics_unsolvable";return s;}
    s.goal=control->joints;s.aim_point=point;s.flight_time=control->flight_time;
    if(rotating){last_center_point_=point;last_center_s_=now_s;}
    // Armor mode also covers a stationary target. The same future-face,
    // alignment, freshness and timing tests apply before any shot qualifies.
    if(!s.face_valid){s.reason="future_face_ambiguous";return s;}
    const auto hit=ballistic(pose,current,s.impact_face,muzzle,bore);
    if(!hit){s.reason="future_face_ballistics_unsolvable";return s;}
    s.yaw_error=std::remainder(hit->joints.yaw-current.yaw,2*M_PI);
    s.pitch_error=hit->joints.pitch-current.pitch;
    const double distance=(s.impact_face-hit->muzzle_world).norm();
    if(s.goal->pitch<-.72 || s.goal->pitch>.45){s.reason="pitch_outside_mechanical_range";return s;}
    // These are geometric hit-width tests, not command speed/displacement limits.
    if(std::abs(s.yaw_error)>std::atan2(width_*.5-margin_,distance) ||
       std::abs(s.pitch_error)>std::atan2(height_*.5,distance)){s.reason="gun_not_on_future_armor";return s;}
    if(s.model.mode==RotationModelManager::Mode::high_speed_phase){
      if(!s.event.valid || !s.model.phase_fire_trusted){s.reason="phase_clock_not_locked_or_consistent";return s;}
      const double window=s.event.half_hit_window_rad+grace_;
      if(s.event.worst_interval_phase_rad>window || (require_face_consistency_ &&
         std::hypot(s.event.worst_interval_phase_rad,s.face_alignment)>window)){
        s.reason="timing_uncertainty_exceeds_armor_window";return s;
      }
    }else if(std::abs(s.omega)*(std::max(delay_-delay_min_,delay_max_-delay_)+flight_uncertainty_)+phase_uncertainty_>
             std::atan2(width_*.5-margin_,radius)+grace_){
      s.reason="timing_uncertainty_exceeds_armor_window";return s;
    }
    s.qualified=true;s.reason="qualified_dry_run";return s;
  }
  bool phase_locked()const{return clock_.locked();}
  double face_period_s()const{return clock_.period_valid()?clock_.face_period_s():0;}
  // Sample one immutable estimator and one committed model/face through the
  // original centered MPC window. Sampling cannot advance filters or the PLL.
  std::optional<Trajectory> trajectory(const Target& target,const Solution& s,double age,
    const Eigen::Quaterniond& pose,io::rmv::JointGoal current,
    const Eigen::Vector3d& muzzle,const Eigen::Vector3d& bore)const{
    if(!s.goal || !s.converged || !s.yaw_trusted || s.face_id<0 || speed_<=0)return {};
    const bool center_follow=s.model.mode!=RotationModelManager::Mode::armor;
    const auto sample=[&](double offset)->std::optional<io::rmv::JointGoal>{
      const double event_offset=s.event.valid?s.event.time_until_hit_center_s-delay_-s.flight_time:0;
      double horizon=age+offset+delay_+s.flight_time+event_offset;
      std::optional<io::rmv::BallisticGoal> goal;
      for(int i=0;i<4;++i){
        const auto state=target.ekf_x_at(horizon);
        const auto faces=target.armor_xyza_list_at(horizon,s.direct_motion);
        if(s.face_id>=int(faces.size()))return {};
        Eigen::Vector3d point=faces[s.face_id].head<3>();
        if(center_follow){
          const double bearing=std::atan2(state[2],state[0]);
          const double radius=std::hypot(point.x()-state[0],point.y()-state[2]);
          point.x()=state[0]-radius*std::cos(bearing);point.y()=state[2]-radius*std::sin(bearing);
        }
        goal=ballistic(pose,current,point,muzzle,bore);if(!goal)return {};
        horizon=age+offset+delay_+goal->flight_time+event_offset;
      }
      return goal->joints;
    };
    auto last=sample(-DT*(HALF_HORIZON+1));auto now=sample(-DT*HALF_HORIZON);
    if(!last || !now)return {};
    Trajectory ref;
    for(int i=0;i<HORIZON;++i){
      const auto next=sample(DT*(i-HALF_HORIZON+1));if(!next)return {};
      ref.col(i)<<now->yaw,std::remainder(next->yaw-last->yaw,2*M_PI)/(2*DT),
        now->pitch,(next->pitch-last->pitch)/(2*DT);
      last=now;now=next;
    }
    return ref;
  }
};
} // namespace auto_aim

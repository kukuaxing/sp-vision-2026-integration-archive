#pragma once
#include "io/rmv/serial.hpp"
#include "io/rmv/session.hpp"
#include "io/rmv/geometry.hpp"
#include "io/rmv/joint_pose.hpp"
#include "io/rmv/shot_batch.hpp"
#include "io/rmv/held_shot.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <random>
#include <thread>

namespace io::rmv {
inline std::uint64_t debug_now_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline bool debug_firmware_supported(std::uint32_t version) {
  return version==2026100602 || version==2026100603 || version==2026100604 || version==2026100605 || version==2026100701 || version==2026100702 || version==2026100703 || version==2026100704 || version==2026100705 || version==2026100706 || version==2026100707 || version==2026100708;
}
struct DebugSnapshot {
  bool active=false, ready=false;
  std::optional<Quaternion> image_q, current_q;
  JointGoal joints{0,0};
  double held_small_yaw=0;
  std::optional<JointGoal> anchor, sent_goal;
  std::uint32_t firmware=0, geometry=0, received=0, applied=0;
  std::uint16_t constraints=0;
  std::uint8_t mode=0, owner=1;
  std::string ready_reason="inactive_session";
  std::optional<std::uint64_t> status_age_us, joint_age_us;
  std::uint64_t joint_sample_us=0, newest_joint_sample_us=0;
  std::array<double,3> joint_velocity{}, joint_target{};
  std::array<std::uint32_t,3> joint_feedback_age_us{};
  std::array<double,3> body_gyro{};
  bool gyro_valid=false;
  double state_extrapolation_s=0;
  std::array<bool,3> joint_target_valid{};
  std::uint32_t faults=0;
  bool mcu_permission=false;
  bool fire_permission=false,shoot_state_fresh=false;
  std::array<double,3> shooter_rpm{};
  std::uint32_t shoot_valid=0,last_shot_id=0,event_request_id=0;
  std::uint8_t shot_event=255;
  std::uint16_t shot_reason=0;
  std::uint64_t shot_event_mcu_us=0;
  ShotBatch batch;
  bool held_position_settled=false;
  std::string error;
};
struct WheelMeasurementRecord {
  enum Kind {wheel,shot,command} kind=wheel;
  std::uint64_t host_us=0,mcu_us=0,flush_done_host_us=0;
  std::uint32_t seq=0,request_id=0,valid=0,aim_seq=0;
  std::array<double,3> rpm{};
  std::uint8_t event=255,evidence=0;
  std::uint16_t reason=0;
  double clock_uncertainty_us=-1;
  bool has_feeder_trace=false;
  std::int32_t feeder_position=0,feeder_target=0,feeder_remaining=0,command_current=0,feedback_current=0;
  double command_rpm=0;
  std::array<std::uint32_t,3> feedback_age_us{};
  std::uint8_t feeder_state=0,motor_mode=0;
  std::uint16_t feeder_fault=0;
  std::uint32_t feeder_flags=0,feeder_completed=0;
};

// Serial/session ownership stays on the worker under one mutex; inference may
// take longer than the heartbeat period. Failed links require an explicit restart.
class DebugLink {
  SerialTransport port_;
  Session session_;
  Decoder raw_;
  std::mutex mutex_;
  std::thread worker_;
  std::atomic<bool> quit_{false};
  bool follow_,fire_=false,trajectory_required_=false,mode_attempted_=false;
  std::uint32_t shot_id_=0;
  ShotBatch batch_;
  HeldShotStability held_stability_;
  bool wheel_measurement_=false,wheel_profile_requested_=false,wheel_profile_ready_=false,finishing_=false;
  bool wheel_restore_requested_=false;
  std::optional<bool> wheel_restore_result_;
  std::deque<WheelMeasurementRecord> wheel_records_;
  void record_wheel(WheelMeasurementRecord record) {
    if(wheel_records_.size()>=4096)throw std::runtime_error("Wheel measurement log queue overflow; no further shots");
    wheel_records_.push_back(record);
  }
  std::optional<std::uint32_t> shoot_seq_,event_seq_,wheel_log_seq_;
  DebugSnapshot shooting_;
  std::uint64_t shoot_host_us_=0;
  std::optional<std::uint32_t> epoch_, joint_seq_;
  std::optional<JointGoal> anchor_;
  std::optional<JointGoal> sent_goal_;
  JointGoal joints_{0,0};
  double small_yaw_=0;
  std::uint64_t joint_us_=0;
  JointPoseHistory joint_history_;
  std::string error_;
  std::uint32_t expected_geometry_;
  double limit_;
  void flush(std::uint64_t now) {
    for (const auto& bytes:session_.take_outgoing(now)) port_.send(bytes);
  }
  void receive_joints(const Frame& f) {
    const auto& p=f.payload;
    if (!session_.active() || f.session!=session_.session_id() || f.flags ||
        f.type!=0x103 || p.size()<16 || p.size()!=16+std::size_t(p[0])*32 ||
        (joint_seq_ && !newer(f.seq,*joint_seq_))) return;
    const auto epoch=static_cast<std::uint32_t>(integer(p,12,4));
    if (epoch_ && epoch!=*epoch_) throw std::runtime_error("IMU reference changed; restart debug");
    epoch_=epoch; joint_seq_=f.seq;
    bool found[3]={false,false,false}; JointGoal next{};JointSample sample;
    for (unsigned i=0;i<p[0];++i) {
      const unsigned k=16+32*i, id=p[k];
      if(id>2)continue;
      const unsigned slot=id==2?0:(id==1?1:2);
      if (found[slot] || (p[k+1]&27)!=27 || integer(p,k+2,2) ||
          integer(p,k+28,4)>100000 || !std::isfinite(real(p,k+4)) || !std::isfinite(real(p,k+8))) {
        joint_us_=0;joint_history_.clear();held_stability_.clear();return;
      }
      found[slot]=true;
      if (id==2) next.yaw=real(p,k+4); else if(id==1) next.pitch=real(p,k+4);
      else small_yaw_=real(p,k+4);
      sample.angle[id]=real(p,k+4);sample.velocity[id]=real(p,k+8);
      sample.feedback_age_us[id]=static_cast<std::uint32_t>(integer(p,k+28,4));
      sample.target_valid[id]=(p[k+1]&32) && std::isfinite(real(p,k+12));
      sample.target[id]=sample.target_valid[id]?real(p,k+12):0;
      sample.maximum_feedback_age=std::max(sample.maximum_feedback_age,static_cast<std::uint32_t>(integer(p,k+28,4)));
    }
    if (found[0] && found[1] && found[2]) {
      joints_=next;joint_us_=integer(p,4,8);sample.time=joint_us_;sample.moving=next;joint_history_.add(sample);
      if(batch_.limit)held_stability_.observe(joint_us_,next.yaw,next.pitch);
    }else{joint_us_=0;joint_history_.clear();held_stability_.clear();}
  }
  DebugSnapshot snapshot_locked(std::uint64_t image_us, std::uint64_t now) {
    DebugSnapshot s; s.active=session_.active(); s.firmware=session_.firmware_version();
    s.geometry=session_.geometry_revision(); s.error=error_; s.joints=joints_;
    s.anchor=anchor_;s.sent_goal=sent_goal_;
    s.held_small_yaw=small_yaw_;
    s.shooter_rpm=shooting_.shooter_rpm;s.shoot_valid=shooting_.shoot_valid;s.last_shot_id=shooting_.last_shot_id;
    s.event_request_id=shooting_.event_request_id;s.shot_event=shooting_.shot_event;
    s.shot_reason=shooting_.shot_reason;s.shot_event_mcu_us=shooting_.shot_event_mcu_us;
    s.batch=batch_;
    s.shoot_state_fresh=shoot_host_us_ && now>=shoot_host_us_ && now-shoot_host_us_<=100000;
    const auto& t=session_.telemetry();
    s.ready_reason=session_.control_ready_reason(now);
    s.status_age_us=t.status_age_us(now);
    if (!s.active) return s;
    s.image_q=t.camera_pose_at(image_us,now);
    auto mcu=t.clock.to_mcu(now,now);
    s.held_position_settled=mcu && held_stability_.settled(*mcu);
    s.newest_joint_sample_us=joint_us_;
    if(mcu)if(auto pair=joint_history_.newest_bracketed(t.poses,*mcu)) {
      s.joint_sample_us=pair->joints.time;s.joint_age_us=*mcu-pair->joints.time;
      s.current_q=pair->pose;s.joints=pair->joints.moving;s.held_small_yaw=pair->joints.angle[0];
      s.joint_velocity=pair->joints.velocity;s.joint_target=pair->joints.target;s.joint_target_valid=pair->joints.target_valid;
      s.joint_feedback_age_us=pair->joints.feedback_age_us;
      if(auto g=t.poses.gyro_at(pair->joints.time,*mcu)){s.body_gyro=*g;s.gyro_valid=true;}
    }
    if (t.status) {
      s.mode=t.status->mode; s.owner=t.status->owner;
      s.received=t.status->received_seq; s.applied=t.status->applied_seq;
      s.constraints=t.status->constraints;
      s.faults=t.status->faults;s.mcu_permission=t.status->control_permitted;
      s.fire_permission=t.status->fire_permitted;
    }
    s.ready=session_.control_ready(now) && s.current_q.has_value() &&
      s.geometry==expected_geometry_ && debug_firmware_supported(s.firmware) &&
      (!wheel_measurement_ || wheel_profile_ready_);
    if(s.ready_reason=="ready"){
      if(!s.current_q)s.ready_reason="waiting_joint_pose_alignment";
      else if(s.geometry!=expected_geometry_)s.ready_reason="geometry_mismatch";
      else if(!debug_firmware_supported(s.firmware))s.ready_reason="firmware_mismatch";
      else if(wheel_measurement_&&!wheel_profile_ready_)s.ready_reason="waiting_wheel_measurement_subscription";
    }
    return s;
  }
  void run() {
    try {
      std::random_device r; auto nonce=(std::uint64_t(r())<<32)^r();
      { std::lock_guard<std::mutex> l(mutex_);
        if (!session_.begin(nonce?nonce:1,debug_now_us(),1))
          throw std::runtime_error("Cannot start RMV debug session");
        flush(debug_now_us()); }
      while (!quit_) {
        const auto bytes=port_.receive();
        if(bytes.empty())std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::lock_guard<std::mutex> l(mutex_);
        const auto now=debug_now_us(); session_.ingest(bytes,now);
        for (const auto& f:raw_.feed(bytes,now)){
          receive_joints(f);
          if(!session_.active() || f.session!=session_.session_id() || f.flags)continue;
          const bool trace=f.type==0x109 && f.payload.size()==76 && (session_.firmware_version()==2026100704 || session_.firmware_version()==2026100705 || session_.firmware_version()==2026100706 || session_.firmware_version()==2026100707 || session_.firmware_version()==2026100708);
          if((trace || (f.type==0x104 && f.payload.size()==52)) && (!shoot_seq_ || newer(f.seq,*shoot_seq_))){
            const auto& p=f.payload;
            const auto stamp=integer(p,0,8);
            const auto valid=static_cast<std::uint32_t>(integer(p,trace?12:8,4));
            bool well_formed=!(valid&~127u);
            for(unsigned k=0;k<3;++k)if((valid&(1u<<k))&&!std::isfinite(real(p,16+4*k)))well_formed=false;
            if(!well_formed){shoot_host_us_=0;continue;}
            // Record well-formed MCU samples even when host clock sync is
            // temporarily uncertain. Control still requires the strict clock gate.
            if(wheel_measurement_ && (!wheel_log_seq_ || newer(f.seq,*wheel_log_seq_))){
              WheelMeasurementRecord r;r.kind=WheelMeasurementRecord::wheel;
              r.host_us=now;r.mcu_us=stamp;r.seq=f.seq;
              r.request_id=static_cast<std::uint32_t>(integer(p,trace?8:12,4));r.valid=valid;
              for(unsigned k=0;k<3;++k)r.rpm[k]=real(p,16+4*k);
              if(trace){
                r.has_feeder_trace=true;r.feeder_position=std::int32_t(integer(p,28,4));
                r.feeder_target=std::int32_t(integer(p,32,4));r.feeder_remaining=std::int32_t(integer(p,36,4));
                r.command_rpm=real(p,40);r.command_current=std::int32_t(integer(p,44,4));
                r.feedback_current=std::int32_t(integer(p,48,4));
                for(unsigned k=0;k<3;++k)r.feedback_age_us[k]=integer(p,52+4*k,4);
                r.feeder_state=p[64];r.motor_mode=p[65];r.feeder_fault=integer(p,66,2);
                r.feeder_flags=integer(p,68,4);r.feeder_completed=integer(p,72,4);
              }
              wheel_log_seq_=f.seq;record_wheel(r);
            }
            const auto mcu=session_.telemetry().clock.to_mcu(now,now);
            if(!mcu || stamp>*mcu+5000 || (*mcu>stamp && *mcu-stamp>100000))continue;
            shoot_seq_=f.seq;shoot_host_us_=now;
            shooting_.shoot_valid=valid;
            shooting_.last_shot_id=static_cast<std::uint32_t>(integer(p,trace?8:12,4));
            for(unsigned k=0;k<3;++k)shooting_.shooter_rpm[k]=real(p,16+4*k);
          }else if(f.type==0x105 && f.payload.size()==20 && (!event_seq_ || newer(f.seq,*event_seq_))){
            const auto& p=f.payload;if(p[4]>6 || p[5]>2)continue;
            event_seq_=f.seq;shooting_.event_request_id=static_cast<std::uint32_t>(integer(p,0,4));
            shooting_.shot_event=p[4];shooting_.shot_reason=static_cast<std::uint16_t>(integer(p,6,2));
            shooting_.shot_event_mcu_us=integer(p,8,8);
            batch_.event(shooting_.event_request_id,shooting_.shot_event,shooting_.shot_reason,now);
            if(wheel_measurement_) {
              WheelMeasurementRecord r;r.kind=WheelMeasurementRecord::shot;
              r.host_us=now;r.mcu_us=shooting_.shot_event_mcu_us;r.seq=f.seq;
              r.request_id=shooting_.event_request_id;r.event=p[4];r.evidence=p[5];
              r.reason=shooting_.shot_reason;r.aim_seq=static_cast<std::uint32_t>(integer(p,16,4));record_wheel(r);
            }
          }
        }
        if (raw_.crc_errors || raw_.format_errors || raw_.timeouts)
          throw std::runtime_error("RMV stream error; restart debug");
        if (session_.state()==SessionState::disconnected)
          throw std::runtime_error("RMV disconnected; restart debug");
        if (follow_ && session_.active()) {
          if(batch_.limit && session_.firmware_version()!=2026100703 && session_.firmware_version()!=2026100704 && session_.firmware_version()!=2026100705 && session_.firmware_version()!=2026100706 && session_.firmware_version()!=2026100707 && session_.firmware_version()!=2026100708)
            throw std::runtime_error("Shot test requires firmware 2026100703/04");
          if(wheel_measurement_ && session_.firmware_version()!=2026100704 && session_.firmware_version()!=2026100705 && session_.firmware_version()!=2026100706 && session_.firmware_version()!=2026100707 && session_.firmware_version()!=2026100708)
            throw std::runtime_error("Detailed wheel measurement requires firmware 2026100704 through 2026100708");
          if (!debug_firmware_supported(session_.firmware_version()) || session_.geometry_revision()!=expected_geometry_ ||
              (session_.capabilities()&8201)!=8201 || (session_.capabilities()&4))
            throw std::runtime_error("Follow requires big-Y firmware 2026100602/03/04/05 / matching geometry");
          if(trajectory_required_ && ((session_.firmware_version()!=2026100701 && session_.firmware_version()!=2026100702 && session_.firmware_version()!=2026100703 && session_.firmware_version()!=2026100704 && session_.firmware_version()!=2026100705 && session_.firmware_version()!=2026100706 && session_.firmware_version()!=2026100707 && session_.firmware_version()!=2026100708) || !(session_.capabilities()&(1u<<15))))
            throw std::runtime_error("Trajectory control requires firmware 2026100701/02/03 / joint trajectory capability");
          if(fire_ && ((session_.firmware_version()!=2026100605 && session_.firmware_version()!=2026100701 && session_.firmware_version()!=2026100702 && session_.firmware_version()!=2026100703 && session_.firmware_version()!=2026100704 && session_.firmware_version()!=2026100705 && session_.firmware_version()!=2026100706 && session_.firmware_version()!=2026100707 && session_.firmware_version()!=2026100708) || !(session_.capabilities()&128)))
            throw std::runtime_error("Fire requires joint-VISION firmware 2026100605 with shooting capability");
          const auto& status=session_.telemetry().status;
          if(wheel_measurement_ && !finishing_) {
            const auto& profile_result=session_.last_result();
            if(wheel_profile_requested_ && profile_result && profile_result->type==4) {
              if(profile_result->completion!=Completion::accepted)
                throw std::runtime_error("Wheel measurement subscription rejected or timed out");
              wheel_profile_ready_=true;
            }
            if(!wheel_profile_requested_ && !session_.pending())
              wheel_profile_requested_=session_.request_wheel_measurement_profile(true,now);
          }
          if (status && status->owner==1) {mode_attempted_=false;anchor_.reset();}
          // owner=0 in IDLE means the RC has selected AUTOAIM; no request is
          // attempted when the operator is in local hold/manual control.
          if (status && status->mode==0 && status->owner==0 && !status->faults &&
              !finishing_ && (!wheel_measurement_ || wheel_profile_ready_) && !mode_attempted_ && !session_.pending()) {
            if (session_.request_mode(fire_?2:3,6,now)) mode_attempted_=true;
          }
          if (session_.request_timed_out())
            throw std::runtime_error("RMV request timed out; restart debug");
          const auto& result=session_.last_result();
          if (result && result->type==0x200 && result->completion==Completion::rejected)
            throw std::runtime_error("RMV rejected joint debug mode");
        }
        if(wheel_restore_requested_ && !wheel_restore_result_) {
          const auto& r=session_.last_result();
          if(r && r->type==4 && !session_.pending())
            wheel_restore_result_=r->completion==Completion::accepted;
        }
        batch_.tick(now);
        session_.tick(now); flush(debug_now_us());
      }
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> l(mutex_); error_=e.what();
      try { if(follow_ && session_.active()){session_.stop(debug_now_us());flush(debug_now_us());} } catch(...) {}
      session_.disconnect();
    }
  }
public:
  DebugLink(const std::string& path, std::uint32_t baud, bool follow,
            std::uint32_t geometry, double limit_degrees,bool fire=false,bool trajectory_required=false,unsigned shot_limit=0,bool wheel_measurement=false)
    :follow_(follow),fire_(fire),trajectory_required_(trajectory_required),batch_(shot_limit),wheel_measurement_(wheel_measurement),expected_geometry_(geometry),limit_(debug_limit_radians(limit_degrees)) {
    if(fire_&&!follow_)throw std::invalid_argument("RMV fire requires explicit follow");
    if(shot_limit && !fire_)throw std::invalid_argument("Shot batch requires fire mode");
    if(wheel_measurement && !shot_limit)throw std::invalid_argument("Wheel measurement requires a bounded stationary shot test");
    port_.open(path,baud);
    try {worker_=std::thread([this]{run();});} catch(...){port_.close();throw;}
  }
  ~DebugLink() {
    if(wheel_measurement_ && !finishing_)try{finish_wheel_measurement();}catch(...){}
    quit_=true; if(worker_.joinable())worker_.join();
    try {std::lock_guard<std::mutex> l(mutex_);
      if(follow_ && session_.active()){session_.stop(debug_now_us());flush(debug_now_us());}
    }catch(...){}
    try{port_.close();}catch(...){}
  }
  DebugSnapshot snapshot(std::uint64_t image_us) {
    std::lock_guard<std::mutex> l(mutex_);return snapshot_locked(image_us,debug_now_us());
  }
  std::deque<WheelMeasurementRecord> take_wheel_measurement() {
    std::lock_guard<std::mutex> l(mutex_);
    std::deque<WheelMeasurementRecord> result;result.swap(wheel_records_);return result;
  }
  bool finish_wheel_measurement() {
    if(!wheel_measurement_)return true;
    {std::lock_guard<std::mutex> l(mutex_);finishing_=true;
      if(!session_.active())return false;
      session_.stop(debug_now_us());flush(debug_now_us());}
    // Stop first, then restore the default telemetry, while the worker still
    // receives ACKs. A disconnected peer restores defaults on its next HELLO.
    for(unsigned i=0;i<100;++i){
      {std::lock_guard<std::mutex> l(mutex_);
        if(!session_.active())return false;
        if(!session_.pending() && session_.request_wheel_measurement_profile(false,debug_now_us())){
          wheel_restore_requested_=true;wheel_restore_result_.reset();
          flush(debug_now_us());break;
        }
        if(i==99)return false;}
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    for(unsigned i=0;i<100;++i){
      {std::lock_guard<std::mutex> l(mutex_);
        if(!session_.active())return false;
        if(wheel_restore_result_)return *wheel_restore_result_;}
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }
  DebugSnapshot control_snapshot(std::uint64_t image_us){
    auto s=snapshot(image_us);
    if(!s.current_q || !s.joint_age_us)return s;
    if(*s.joint_age_us>50000){s.ready=false;s.current_q.reset();s.ready_reason="joint_state_too_old_for_extrapolation";return s;}
    const double dt=*s.joint_age_us*1e-6;s.state_extrapolation_s=dt;
    const JointGoal paired{s.joints.yaw+s.joint_velocity[2]*s.joint_feedback_age_us[2]*1e-6,
      s.joints.pitch+s.joint_velocity[1]*s.joint_feedback_age_us[1]*1e-6};
    const JointGoal next{paired.yaw+s.joint_velocity[2]*dt,paired.pitch+s.joint_velocity[1]*dt};
    Eigen::Quaterniond q((*s.current_q)[0],(*s.current_q)[1],(*s.current_q)[2],(*s.current_q)[3]);
    if(s.gyro_valid){
      const Eigen::Vector3d g(s.body_gyro[0],s.body_gyro[1],s.body_gyro[2]);
      if(g.norm()>1e-9)q=q*Eigen::Quaterniond(Eigen::AngleAxisd(g.norm()*dt,g.normalized()));
    }else q=goal_pose(q,paired,next,1);
    q.normalize();s.current_q=Quaternion{q.w(),q.x(),q.y(),q.z()};s.joints=next;
    return s;
  }
  bool submit(std::optional<JointGoal> goal) {
    if(!follow_) return false;
    std::lock_guard<std::mutex> l(mutex_); const auto now=debug_now_us();
    // A replacement AIM before execution would cancel the related single shot.
    if(batch_.active_id && batch_.phase!=1)return false;
    auto s=snapshot_locked(now,now); if(!s.ready) return false;
    if(!anchor_)anchor_=s.joints;
    if(goal) {
      if(limit_>0){
        const double y=anchor_->yaw+std::clamp(std::remainder(goal->yaw-anchor_->yaw,6.283185307179586),-limit_,limit_);
        const double p=std::clamp(goal->pitch,anchor_->pitch-limit_,anchor_->pitch+limit_);
        if(y!=goal->yaw){goal->yaw_rate=goal->yaw_accel=0;}if(p!=goal->pitch){goal->pitch_rate=goal->pitch_accel=0;}
        goal->yaw=y;goal->pitch=p;
      }
      // Current native envelope, preserving its original zeros and signs.
      goal->yaw=std::remainder(goal->yaw,6.283185307179586);
      const double p=std::clamp(goal->pitch,-.72,.45);
      if(p!=goal->pitch)goal->pitch_rate=goal->pitch_accel=0;goal->pitch=p;
    }
    const bool sent=session_.aim(goal?goal->yaw:0,goal?goal->pitch:0,now,2,60000,goal.has_value(),2,
      goal&&goal->trajectory,goal?goal->yaw_rate:0,goal?goal->pitch_rate:0,goal?goal->yaw_accel:0,goal?goal->pitch_accel:0);
    // Do not leave a goal queued across a blocking serial read or inference.
    if(sent){flush(debug_now_us());sent_goal_=goal;}
    return sent;
  }
  std::optional<std::uint32_t> fire_once(JointGoal goal,std::uint64_t image_us){
    if(!fire_)return {};
    std::lock_guard<std::mutex> l(mutex_);const auto now=debug_now_us();
    return fire_locked(goal,image_us,now,true);
  }
  // Explicit stationary shot test: no camera/target prediction. Still requires
  // fresh pose/joints, double-UP control mode, settled wheels and MCU shooting gates.
  std::optional<std::uint32_t> fire_held_once(JointGoal goal){
    if(!fire_)return {};
    std::lock_guard<std::mutex> l(mutex_);const auto now=debug_now_us();
    const auto s=snapshot_locked(now,now);
    if(!batch_.can_send(now) || (s.firmware!=2026100703 && s.firmware!=2026100704 && s.firmware!=2026100705 && s.firmware!=2026100706 && s.firmware!=2026100707 && s.firmware!=2026100708) || !s.joint_age_us ||
       *s.joint_age_us>50000 || !s.held_position_settled ||
       std::abs(std::remainder(goal.yaw-s.joints.yaw,2*M_PI))>.03 ||
       std::abs(goal.pitch-s.joints.pitch)>.02 ||
       std::abs(std::abs(s.shooter_rpm[0])-6500)>300 ||
       std::abs(std::abs(s.shooter_rpm[1])-6500)>300)return {};
    return fire_locked(goal,now,now,false);
  }
private:
  std::optional<std::uint32_t> fire_locked(JointGoal goal,std::uint64_t image_us,std::uint64_t now,bool image_required){
    const auto s=snapshot_locked(image_us,now);
    if(!s.ready || !s.fire_permission || !s.shoot_state_fresh || (s.shoot_valid&7)!=7 ||
       (image_required && (!s.image_q || now<image_us || now-image_us>80000)) || session_.pending() ||
       (batch_.limit && !batch_.can_send(now)) ||
       !std::isfinite(goal.yaw) || !std::isfinite(goal.pitch) || goal.pitch<-.72 || goal.pitch>.45 ||
       limit_>0 || shot_id_==0xffffffffu)return {};
    goal.yaw=std::remainder(goal.yaw,2*M_PI);
    if(!session_.aim(goal.yaw,goal.pitch,now,2,60000,true,2,goal.trajectory,
      goal.yaw_rate,goal.pitch_rate,goal.yaw_accel,goal.pitch_accel))return {};
    const auto id=++shot_id_;
    const bool sent=session_.shoot(id,now,30000);
    if(sent && batch_.limit)batch_.sent(id,now);
    flush(debug_now_us());sent_goal_=goal;
    if(sent && wheel_measurement_) {
      WheelMeasurementRecord r;r.kind=WheelMeasurementRecord::command;r.request_id=id;
      r.host_us=now;r.flush_done_host_us=debug_now_us();
      r.mcu_us=session_.telemetry().clock.to_mcu(now,now).value_or(0);
      r.clock_uncertainty_us=session_.telemetry().clock.uncertainty_us(now).value_or(-1);
      record_wheel(r);
    }
    return sent?std::optional<std::uint32_t>(id):std::nullopt;
  }
};
} // namespace io::rmv

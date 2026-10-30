#pragma once
// Opt-in file-only clock/IMU fault fixture. No device access. Truth attitudes
// generate synthetic sensor packets; only PoseHistory estimates reach Solver.
#include "io/sentinel/pose_history.hpp"
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <deque>
#include <optional>
#include <vector>

namespace fixed_sim {
class TimingPipeline {
 public:
  using Json=nlohmann::json;
  explicit TimingPipeline(const Json& settings):enabled(!settings.is_null()),settings_(settings) {
    if(!enabled)return;
    if(!settings.is_object())throw std::invalid_argument("timing_pipeline must be an object");
    for(const auto& f:settings.value("faults",Json::array())) {
      const double start=f.at("start_s"),end=f.at("end_s");
      if(!(start>=0&&end>start&&end<=300))throw std::invalid_argument("invalid timing fault interval");
      for(const char* key:{"image_delay_s","sensor_latency_s","imu_transport_delay_s","post_wait_s"}) {
        const double value=f.value(key,0.);
        if(!std::isfinite(value)||value<0||value>.5)throw std::invalid_argument("invalid timing fault delay");
      }
      const double bias=f.value("image_stamp_bias_s",0.);
      if(!std::isfinite(bias)||std::abs(bias)>.5)throw std::invalid_argument("invalid image clock bias");
      faults_.push_back(f);reset_done_.push_back(false);
    }
  }
  const bool enabled;
  double value(const char* key,double absolute)const {
    double result=0;for(const auto& f:faults_)if(active(f,absolute))result+=f.value(key,0.);return result;
  }
  bool flag(const char* key,double absolute)const {
    for(const auto& f:faults_)if(active(f,absolute)&&f.value(key,false))return true;return false;
  }
  uint64_t generation()const{return generation_;}
  // Hardware-side event. The host learns it only when a new packet arrives.
  void change_reference(){++source_generation_;++boot_;}
  void deliver(double now) {
    std::stable_sort(pending_.begin(),pending_.end(),[](const Packet&a,const Packet&b){return a.arrival<b.arrival;});
    while(!pending_.empty()&&pending_.front().arrival<=now+1e-10) {
      const auto p=pending_.front();pending_.erase(pending_.begin());
      // A frame delayed from an old physical session cannot revive that session.
      if(p.generation<generation_){++discarded_old_packets;continue;}
      if(p.generation>generation_) {
        generation_=p.generation;history_.reset();reset_pending_=true;
      }
      history_.push(p.imu,p.boot,p.generation,ns(p.arrival));++delivered_packets;
    }
  }
  void sample(double time,const Eigen::Quaterniond& true_pose) {
    for(std::size_t i=0;i<faults_.size();++i)if(!reset_done_[i]&&time-1>=faults_[i].at("start_s").get<double>()-1e-10&&
      faults_[i].value("new_generation",false)) {
      reset_done_[i]=true;change_reference();
    }
    attitudes_.push_back({time,true_pose});
    while(attitudes_.size()>400)attitudes_.pop_front();
    if(flag("drop_imu",time))return;
    const double sensor_time=time-value("sensor_latency_s",time);
    auto attitude=attitude_at(sensor_time);
    if(!attitude)return; // Do not invent a prehistory for a delayed sensor.
    const auto r=attitude->toRotationMatrix();
    io::sentinel::Imu imu;imu.id=1;imu.received=true;imu.epoch=1;imu.seq=++sequence_;
    imu.flags=flag("invalid_pose",time)?0:io::sentinel::orient_valid;
    imu.roll=std::atan2(r(2,1),r(2,2));imu.pitch=std::asin(std::clamp(-r(2,0),-1.,1.));
    imu.yaw=std::atan2(r(1,0),r(0,0));
    // Device stamp describes packet sampling/processing time. Sensor internal
    // latency is intentionally not included, matching the unknown in real code.
    imu.time_us=static_cast<uint32_t>(std::llround(time*1e6));
    if(flag("freeze_imu_clock",time))imu.time_us=last_source_us_;
    else last_source_us_=imu.time_us;
    pending_.push_back({time+value("imu_transport_delay_s",time),imu,boot_,source_generation_});
  }
  bool consume_reset(){const bool r=reset_pending_;reset_pending_=false;return r;}
  auto capture(double query,double now) {deliver(now);return history_.at(ns(query),ns(now));}
  auto latest(double now){deliver(now);return history_.latest(ns(now));}
  static Eigen::Quaterniond rotation(const io::sentinel::PoseEstimate& p) {
    return Eigen::AngleAxisd(p.rpy_rad[2],Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(p.rpy_rad[1],Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(p.rpy_rad[0],Eigen::Vector3d::UnitX());
  }
  static int64_t ns(double t){return static_cast<int64_t>(std::llround(t*1e9));}
  int delivered_packets=0,discarded_old_packets=0;
 private:
  struct Packet{double arrival;io::sentinel::Imu imu;uint32_t boot;uint64_t generation;};
  static bool active(const Json& f,double time){return time-1>=f.at("start_s").get<double>()-1e-10&&time-1<f.at("end_s").get<double>()-1e-10;}
  std::optional<Eigen::Quaterniond> attitude_at(double time)const {
    if(attitudes_.empty()||time<attitudes_.front().first-1e-9)return {};
    for(std::size_t i=1;i<attitudes_.size();++i)if(time<=attitudes_[i].first+1e-9) {
      const auto& a=attitudes_[i-1];const auto& b=attitudes_[i];
      return a.second.slerp(std::clamp((time-a.first)/(b.first-a.first),0.,1.),b.second);
    }
    return attitudes_.back().second;
  }
  Json settings_;std::vector<Json> faults_;std::vector<bool> reset_done_;
  io::sentinel::PoseHistory history_;std::vector<Packet> pending_;
  std::deque<std::pair<double,Eigen::Quaterniond>> attitudes_;
  uint64_t generation_=1,source_generation_=1;uint32_t boot_=1,last_source_us_=0;uint16_t sequence_=0;bool reset_pending_=false;
};
}

#pragma once
#include "io/rmv/core.hpp"
#include "io/rmv/geometry.hpp"

namespace io::rmv {
struct JointSample {
  std::uint64_t time=0;
  std::uint32_t maximum_feedback_age=0;
  JointGoal moving{0,0};
  std::array<double,3> angle{}, velocity{}, target{};
  std::array<bool,3> target_valid{};
  std::array<std::uint32_t,3> feedback_age_us{};
};
struct JointPosePair { JointSample joints; Quaternion pose; };

// Joint packets and IMU samples have different phases. Retain recent packets
// so the newest one cannot discard an older, fully bracketed fresh pair.
// No extrapolation, timestamp tolerance expansion or target-lifetime change.
class JointPoseHistory {
  std::deque<JointSample> samples_;
public:
  void clear(){samples_.clear();}
  void add(const JointSample& sample){
    if(!samples_.empty() && sample.time<=samples_.back().time)return;
    samples_.push_back(sample);
    while(samples_.size()>32)samples_.pop_front();
  }
  std::optional<JointPosePair> newest_bracketed(const PoseHistory& poses,std::uint64_t now) const {
    if(samples_.empty() || (samples_.back().time>now && samples_.back().time-now>5000))return {};
    for(auto it=samples_.rbegin();it!=samples_.rend();++it){
      if(now<it->time)continue;
      const auto age=now-it->time;
      if(age+it->maximum_feedback_age>100000)continue;
      if(auto q=poses.at(it->time,now))return JointPosePair{*it,*q};
    }
    return {};
  }
};
} // namespace io::rmv

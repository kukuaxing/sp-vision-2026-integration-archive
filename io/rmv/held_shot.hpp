#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>

namespace io::rmv {
// Motor velocity is quantised and can report ~0.033 rad/s at rest. For a held
// angle test, qualify actual position span instead of a continuous speed gate.
class HeldShotStability {
  struct Sample {std::uint64_t time;double yaw,pitch;};
  std::deque<Sample> samples_;
public:
  void clear(){samples_.clear();}
  void observe(std::uint64_t time,double yaw,double pitch) {
    if(!std::isfinite(yaw) || !std::isfinite(pitch)){clear();return;}
    if(!samples_.empty()) {
      if(time<=samples_.back().time)return;
      if(time-samples_.back().time>50000)clear();
      else yaw=samples_.back().yaw+std::remainder(yaw-samples_.back().yaw,6.283185307179586);
    }
    samples_.push_back({time,yaw,pitch});
    // Keep the sample preceding the 200 ms boundary to prove a full window.
    while(samples_.size()>2 && time-samples_[1].time>=200000)samples_.pop_front();
    while(samples_.size()>64)samples_.pop_front();
  }
  bool settled(std::uint64_t now) const {
    if(samples_.size()<2 || now<samples_.back().time ||
       now-samples_.back().time>50000 ||
       samples_.back().time-samples_.front().time<200000)return false;
    double ymin=samples_.front().yaw,ymax=ymin;
    double pmin=samples_.front().pitch,pmax=pmin;
    for(const auto& s:samples_){
      ymin=std::min(ymin,s.yaw);ymax=std::max(ymax,s.yaw);
      pmin=std::min(pmin,s.pitch);pmax=std::max(pmax,s.pitch);
    }
    return ymax-ymin<=.005 && pmax-pmin<=.005; // ~0.286 degrees peak to peak.
  }
};
}

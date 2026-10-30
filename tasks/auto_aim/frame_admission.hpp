#pragma once
#include <cstdint>
#include <deque>
#include "io/sentinel/pose_history.hpp"

namespace auto_aim {
// Exact recent metadata replay detection, not a clock repair or fire grant.
// Unknown/new frame IDs with bad timestamps still take the ordinary fault path.
class FrameAdmission {
 public:
  struct Input {
    uint64_t frame_id=0,generation=0;
    uint32_t boot_id=0;uint16_t epoch=0;
    int64_t capture_ns=0,now_ns=0;
    bool reference_valid=false;
  };
  bool reject_replay(const Input& in) {
    constexpr auto age=io::sentinel::PoseHistory::max_age_ns;
    if(!in.reference_valid||!in.frame_id||!in.generation||!in.boot_id||
       in.capture_ns<=0||in.now_ns<in.capture_ns||in.now_ns-in.capture_ns>age) {
      reset();return false;
    }
    if(in.generation!=generation_||in.boot_id!=boot_||in.epoch!=epoch_||
       (last_now_&&in.now_ns<last_now_))reset();
    generation_=in.generation;boot_=in.boot_id;epoch_=in.epoch;last_now_=in.now_ns;
    while(!seen_.empty()&&in.now_ns-seen_.front().capture>age)seen_.pop_front();
    for(const auto& frame:seen_)if(frame.id==in.frame_id&&frame.capture==in.capture_ns)
      return true; // Never renew its lifetime or move the accepted watermark.
    // A mismatched pair may be a clock/camera restart. Do not preserve its
    // history by guessing that either its timestamp or frame counter is right.
    if(!seen_.empty()&&(in.frame_id<=seen_.back().id||in.capture_ns<=seen_.back().capture))seen_.clear();
    seen_.push_back({in.frame_id,in.capture_ns});
    while(seen_.size()>64)seen_.pop_front();
    return false;
  }
  void reset(){seen_.clear();generation_=0;boot_=0;epoch_=0;last_now_=0;}
 private:
  struct Frame{uint64_t id;int64_t capture;};
  std::deque<Frame> seen_;
  uint64_t generation_=0;uint32_t boot_=0;uint16_t epoch_=0;int64_t last_now_=0;
};
}

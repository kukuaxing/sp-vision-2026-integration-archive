#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>

namespace io::rmv {
// A bounded batch counts requests and encoder-confirmed cycles, never projectiles.
// A missing/rejected action ends the batch instead of retrying a possibly fired shot.
struct ShotBatch {
  unsigned limit=0, issued=0, completed=0;
  std::uint32_t active_id=0;
  std::uint8_t phase=255;
  std::uint64_t sent_us=0, completed_us=0;
  std::string error;
  explicit ShotBatch(unsigned count=0):limit(count) {
    if(count>20)throw std::invalid_argument("Shot test allows 1 to 20 requests");
  }
  bool can_send(std::uint64_t now) const {
    return limit && issued<limit && !active_id && error.empty() &&
      (!completed_us || (now>=completed_us && now-completed_us>=150000));
  }
  void sent(std::uint32_t id,std::uint64_t now) {
    if(!id || !can_send(now))throw std::logic_error("Shot batch request out of order");
    ++issued;active_id=id;phase=255;sent_us=now;
  }
  void event(std::uint32_t id,std::uint8_t event,std::uint16_t reason,std::uint64_t now) {
    if(!active_id || id!=active_id)return;
    if(event==0){if(phase==255)phase=0;return;}
    if(event==1){phase=1;return;}
    if(event==2 && !reason) {
      ++completed;active_id=0;phase=2;completed_us=now;return;
    }
    if(event>=3 || reason) {
      error="Shot "+std::to_string(id)+" stopped: event="+std::to_string(event)+
        ", reason="+std::to_string(reason);
    }
  }
  void tick(std::uint64_t now) {
    if(active_id && now>=sent_us && now-sent_us>=8000000 && error.empty())
      error="Shot completion timeout; no retry";
  }
  bool done() const {return limit && completed==limit && !active_id && error.empty();}
};
}

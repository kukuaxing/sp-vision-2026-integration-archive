#include "io/rmv/shot_batch.hpp"
#include <cassert>
int main() {
  using io::rmv::ShotBatch;
  ShotBatch b(20);std::uint64_t now=1000000;
  for(unsigned id=1;id<=20;++id) {
    assert(b.can_send(now));b.sent(id,now);
    assert(!b.can_send(now+1000));
    b.event(id+1,2,0,now+2000);assert(b.completed==id-1);
    b.event(id,0,0,now+3000);b.event(id,1,0,now+4000);
    b.event(id,0,0,now+5000);assert(b.phase==1);
    b.event(id,2,0,now+6000);b.event(id,2,0,now+7000);
    assert(b.completed==id && !b.active_id);
    assert(!b.can_send(now+10000));now+=200000;
  }
  assert(b.done() && !b.can_send(now+1000000));
  ShotBatch rejected(20);rejected.sent(1,1);rejected.event(1,4,7,2);
  assert(!rejected.can_send(1000000) && !rejected.error.empty());
  ShotBatch cancelled(20);cancelled.sent(1,1);cancelled.event(1,6,8,2);
  assert(!cancelled.can_send(1000000) && cancelled.completed==0);
  ShotBatch lost(20);lost.sent(1,1);lost.tick(8000001);
  assert(!lost.can_send(9000000) && !lost.error.empty());
  ShotBatch missed_start(1);missed_start.sent(1,1);missed_start.event(1,2,0,10);
  assert(missed_start.done()); // completion still proves the cycle if start telemetry was lost
  ShotBatch unused;assert(!unused.can_send(1) && !unused.done());
  try{ShotBatch invalid(21);assert(false);}catch(const std::invalid_argument&){}
}

#include "tasks/auto_aim/frame_admission.hpp"
#include <cassert>
#include <iostream>
int main() {
  using auto_aim::FrameAdmission;
  constexpr int64_t ms=1000000,t=10000000000LL;
  FrameAdmission a;
  FrameAdmission::Input p{10,1,7,2,t,t+10*ms,true};
  assert(!a.reject_replay(p));p.now_ns+=20*ms;assert(a.reject_replay(p));
  auto next=p;next.frame_id=11;next.capture_ns=t+20*ms;next.now_ns=t+30*ms;
  assert(!a.reject_replay(next));p.now_ns=t+40*ms;assert(a.reject_replay(p));
  // Same time with a NEW ID is not a proven replay; it must reach fault handling.
  next.frame_id=12;assert(!a.reject_replay(next));
  // Same ID with a different timestamp is not a proven replay either.
  next.capture_ns+=ms;assert(!a.reject_replay(next));
  // A real reference transition cannot be hidden by a matching image pair.
  next.generation++;assert(!a.reject_replay(next));assert(a.reject_replay(next));
  next.boot_id++;assert(!a.reject_replay(next));next.epoch++;assert(!a.reject_replay(next));
  // Timeouts never refresh just because the replay keeps arriving.
  a.reset();p={10,1,7,2,t,t+10*ms,true};assert(!a.reject_replay(p));
  for(int i=20;i<=100;i+=10){p.now_ns=t+i*ms;assert(a.reject_replay(p));}
  p.now_ns=t+101*ms;assert(!a.reject_replay(p));
  p.now_ns=t-1;assert(!a.reject_replay(p));
  p.now_ns=t+10*ms;p.frame_id=0;assert(!a.reject_replay(p));assert(!a.reject_replay(p));
  p.frame_id=10;p.reference_valid=false;assert(!a.reject_replay(p));
  p.reference_valid=true;assert(!a.reject_replay(p));
  // An unseen out-of-order pair is not reconstructed or silently discarded.
  next=p;next.frame_id=9;next.capture_ns=t-ms;assert(!a.reject_replay(next));
  std::cout<<"PASS exact-pair replay, bounded lifetime, unknown metadata and reference/clock isolation\n";
}

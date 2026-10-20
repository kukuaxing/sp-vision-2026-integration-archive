#include "io/sentinel/pose_history.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace io::sentinel;

Imu sample(uint32_t us, float yaw = 0, uint16_t epoch = 1, uint8_t flags = 3) {
  Imu s;
  s.time_us=us; s.yaw=yaw; s.epoch=epoch; s.flags=flags; s.id=1; s.received=true;
  s.gx=1; s.gy=2; s.gz=3;
  return s;
}

int main() {
  constexpr int64_t ms=1000000, t=10000000000;
  PoseHistory h;
  assert(!h.at(t,t).valid);
  auto a=sample(1000000,179*deg_to_rad); a.seq=10;
  auto b=sample(1020000,-179*deg_to_rad); b.seq=11;
  auto c=sample(1040000,-177*deg_to_rad); c.seq=12;
  h.push(a,1,1,t); h.push(b,1,1,t+20*ms); h.push(c,1,1,t+40*ms);
  const auto middle=h.at(t+10*ms,t+40*ms);
  assert(middle.valid && middle.interpolated && middle.source_seq==11);
  assert(std::abs(std::abs(middle.rpy_rad[2])-pi)<1e-5);
  assert(!middle.synchronized && !middle.sample_certified && !middle.axes_verified);
  assert(middle.time_uncertainty_ns>=30*ms && middle.source_time_us==1010000);
  assert(std::abs(middle.gyro_rad_s[1]-2)<1e-6);
  assert(!h.at(t+41*ms,t+41*ms).valid); // No future bracket/extrapolation.
  assert(h.at(t+41*ms,t+41*ms).status==PoseStatus::bracket_unavailable);
  assert(!h.at(t+10*ms,t+150*ms).valid && !h.latest(t+150*ms).valid);

  h.push(sample(1060000,0,2),1,2,t+60*ms);
  const auto epoch=h.at(t+50*ms,t+60*ms);
  assert(!epoch.valid && epoch.status==PoseStatus::warming_up && epoch.epoch==2);
  h.push(sample(1080000,0,2),1,2,t+80*ms);
  h.push(sample(1100000,0,2),1,2,t+100*ms);
  assert(h.at(t+90*ms,t+100*ms).valid);
  h.push(sample(1120000,0,2),2,3,t+120*ms);
  assert(!h.at(t+90*ms,t+120*ms).valid); // Never cross a reboot/generation.
  h.push(sample(1140000,0,2,0),2,3,t+140*ms);
  assert(!h.latest(t+140*ms).valid); // Invalid orientation revokes cached poses.

  h.reset();
  for(int i=0;i<3;++i) h.push(sample(uint32_t(0xffffff00u+i*20000u)),7,4,t+i*20*ms);
  assert(h.at(t+30*ms,t+40*ms).valid); // 32-bit MCU time wrap.
  h.push(sample(0xffffff00u+40000u),7,4,t+50*ms);
  assert(h.latest(t+50*ms).source_time_us==uint32_t(0xffffff00u+40000u)); // Frozen source cannot renew.
  assert(!h.latest(t+150*ms).valid);

  h.reset();
  h.push(sample(1000000),1,1,t); h.push(sample(1020000),1,1,t+20*ms);
  h.push(sample(1080000),1,1,t+80*ms);
  assert(h.at(t+50*ms,t+80*ms).status==PoseStatus::gap);
  h.push(sample(1100000),1,1,t+160*ms);
  assert(h.at(t+90*ms,t+160*ms).status==PoseStatus::clock_unstable);

  // Same OS read may contain several frames: equal RX timestamps are allowed.
  h.reset();
  h.push(sample(1000000),1,1,t); h.push(sample(1020000),1,1,t+20*ms);
  h.push(sample(1040000),1,1,t+20*ms);
  assert(h.latest(t+20*ms).source_time_us==1040000);
  assert(h.at(t+10*ms,t+20*ms).valid);

  std::cout << "PASS capture-pose clock estimate, quaternion wrap, no extrapolation, stale/gap/jitter, boot/epoch isolation\n";
}

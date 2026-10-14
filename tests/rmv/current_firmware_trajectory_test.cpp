// Compile with the maintained firmware include directory, never a copied core.
// Real host Session -> real MCU Device. All I/O remains in memory.
#include "io/rmv/session.hpp"
#include "rmv_core.h"
#include <cassert>
#include <iostream>
struct Backend:rmv::Backend{
  io::rmv::Bytes replies;
  bool send(const uint8_t* p,size_t n)override{replies.insert(replies.end(),p,p+n);return true;}
  bool save(const float*,size_t)override{assert(false);return false;}
};
int main(){
  Backend backend;rmv::Device mcu(backend);io::rmv::Session host;
  constexpr uint64_t offset=5000000;uint64_t now=1000000;
  unsigned wheel_samples=0,trace_samples=0;io::rmv::Decoder observer;
  mcu.init(123,2026100708,11419|(1u<<15));mcu.input.geometry=1;
  mcu.input.shoot_valid=7;mcu.input.wheel_rpm[0]=6450;mcu.input.wheel_rpm[1]=-6451;
  mcu.input.feeder_position=-12345;mcu.input.feeder_target=-54475;
  mcu.input.feeder_command_current=-900;mcu.input.feeder_feedback_current=-800;
  mcu.input.permit=true;mcu.input.enabled=mcu.input.homed=7;
  for(auto& j:mcu.input.joints){j.bits=31;j.age=0;}
  auto exchange=[&](){
    auto& p=mcu.input.poses[0];p.flags=51;p.time=now+offset;p.counter++;
    mcu.tick(now+offset);
    for(const auto& wire:host.take_outgoing(now))mcu.feed(wire.data(),wire.size(),now+offset);
    mcu.input.shoot_sample_us=now+offset;
    mcu.telemetry(now+offset);
    for(const auto& frame:observer.feed(backend.replies,now)){
      if(frame.type==0x104)++wheel_samples;
      if(frame.type==0x109){++trace_samples;
        assert(frame.payload.size()==76 && io::rmv::integer(frame.payload,0,8)==now+offset);
        assert(io::rmv::real(frame.payload,20)==-6451 &&
          std::int32_t(io::rmv::integer(frame.payload,28,4))==-12345 &&
          std::int32_t(io::rmv::integer(frame.payload,44,4))==-900);
      }
    }
    now+=200;
    host.ingest(backend.replies,now);backend.replies.clear();
  };
  assert(host.begin(42,now,1));exchange();assert(host.active());
  for(int i=0;i<8;++i){now+=10000;assert(host.request_time_sync(now));exchange();}
  now+=10000;exchange();assert(host.telemetry().epoch && host.telemetry().status);
  assert(host.request_wheel_measurement_profile(true,now));exchange();
  assert(host.last_result()->completion==io::rmv::Completion::accepted);
  wheel_samples=trace_samples=0;
  for(unsigned i=0;i<20;++i){now+=4800+(i%2?80:-80);exchange();}
  assert(trace_samples==20 && wheel_samples==0); // Fixed phase survives +/-80us task jitter.
  assert(host.request_wheel_measurement_profile(false,now));exchange();
  assert(host.last_result()->completion==io::rmv::Completion::accepted);
  wheel_samples=trace_samples=0;
  for(unsigned i=0;i<20;++i){now+=5000;exchange();}
  assert(wheel_samples==2 && trace_samples==0); // Default20Hz, combined trace disabled.
  assert(host.request_mode(3,6,now));exchange();now+=50000;exchange();
  assert(host.control_ready(now));
  assert(host.aim(.1,.2,now,2,60000,true,2,true,.4,-.3,2,-1));exchange();
  assert(mcu.target.valid && mcu.target.trajectory && mcu.target.yaw_rate==.4f &&
    mcu.target.pitch_rate==-.3f && mcu.target.yaw_accel==2 && mcu.target.pitch_accel==-1);
  float y,p,yr,pr;
  mcu.target.sample(mcu.target.produced+10000,y,p,yr,pr);
  assert(std::abs(y-.1041f)<1e-6 && std::abs(p-.19695f)<1e-6);
  assert(std::abs(yr-.42f)<1e-6 && std::abs(pr+.31f)<1e-6);
  now+=61000;mcu.tick(now+offset);assert(!mcu.target.valid);
  std::cout<<"PASS actual host/firmware: capability, clock, mode, trajectory wire, native sampling, expiry\n";
}

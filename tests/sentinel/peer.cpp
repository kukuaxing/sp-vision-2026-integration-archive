// Private PTY only. Exercises production sender/receiver; no robot devices.
#include "io/xuc.hpp"
#include <cstdio>
#include <cstring>
#include <thread>
#include <cassert>
int main(int argc,char**argv){
  if(argc<2 || argc>3)return 2;io::XucSender link(argv[1]);if(!link.enabled()||!link.is_sentinel())return 3;
  if(argc==3 && std::strcmp(argv[2],"cadence")==0){
    const auto ready_until=io::XucSender::Clock::now()+std::chrono::seconds(3);
    while(!link.control_ready() && io::XucSender::Clock::now()<ready_until)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    assert(link.control_ready());
    const auto start=io::XucSender::Clock::now();
    const auto minimum_ns=link.fire_min_interval_ms()*1000000;
    const int min_fires=minimum_ns==500000000 ? 3 : 2;
    const int max_fires=minimum_ns==500000000 ? 5 : 3;
    int fires=0,limited=0,clear_updates=0,stops=0;int64_t last_fire=0;
    while(true){
      const double t=std::chrono::duration<double>(io::XucSender::Clock::now()-start).count();
      if(t>2.25)break;
      io::Command c{};c.control=true;c.armor_detected=true;
      // A held level, followed by noisy rising edges and precise event deadlines.
      c.shoot=t<1.1 || static_cast<int>(t*100)%2==0;
      c.yaw=179*io::sentinel::deg_to_rad;c.pitch=12.5*io::sentinel::deg_to_rad;
      if(t>.6 && t<.65){link.stop();++stops;}
      else{
        const auto r=link.send(c,io::XucSender::Clock::now()+std::chrono::milliseconds(20));
        if(r.fire_sent){
          assert(!r.fire_rate_limited && (!last_fire || r.tx_time_ns-last_fire>=minimum_ns));
          last_fire=r.tx_time_ns;++fires;
        }
        if(r.fire_rate_limited){
          assert(!r.fire_sent);++limited;
          if(r.control_sent && r.friction_sent)++clear_updates;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    link.stop();
    assert(fires>=min_fires && fires<=max_fires && limited>0 && clear_updates>0 && stops>0);
    std::printf("SUMMARY cadence=1 fires=%d limited=%d control_during_cooldown=%d stop_preserves_cooldown=1 interval_ms=%lld\n",
      fires,limited,clear_updates,static_cast<long long>(link.fire_min_interval_ms()));
    return 0;
  }
  if(argc==3 && std::strcmp(argv[2],"sendresult")==0){
    const auto until=io::XucSender::Clock::now()+std::chrono::seconds(3);
    io::XucSender::PoseSnapshot capture;
    while(io::XucSender::Clock::now()<until){
      capture=link.pose_at_capture(io::XucSender::Clock::now()-std::chrono::milliseconds(30));
      if(capture.valid && capture.control_ready)break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto current=link.pose_now();
    assert(capture.valid && current.valid && current.mode==1 && current.lower_yaw_armed);
    assert(capture.generation==current.generation && !capture.synchronized && !capture.sample_certified);
    assert(std::abs(current.rpy_rad[2]/io::sentinel::deg_to_rad+179)<.001);
    io::Command c{};c.control=true;c.armor_detected=true;
    c.yaw=179*io::sentinel::deg_to_rad;c.pitch=12.5*io::sentinel::deg_to_rad;
    const auto wrong_generation=link.send(c,io::XucSender::Clock::time_point{},current.generation+1);
    assert(current.generation!=0 && wrong_generation.status==io::XucSender::SendStatus::rejected &&
      !wrong_generation.control_sent && !wrong_generation.fire_sent && wrong_generation.tx_time_ns==0);
    const auto normal=link.send(c,io::XucSender::Clock::time_point{},current.generation);
    const auto throttled=link.send(c);
    c.shoot=true;
    const auto fire=link.send(c,io::XucSender::Clock::now()+std::chrono::milliseconds(20));
    const auto expired=link.send(c,io::XucSender::Clock::now()-std::chrono::milliseconds(1));
    const auto stopped=link.stop();
    assert(normal.status==io::XucSender::SendStatus::sent && normal.control_sent && !normal.fire_sent);
    assert(throttled.status==io::XucSender::SendStatus::throttled && !throttled.fire_sent);
    assert(fire.status==io::XucSender::SendStatus::sent && fire.fire_sent && fire.tx_time_ns>0);
    assert(expired.status==io::XucSender::SendStatus::sent && expired.fire_expired && !expired.fire_sent);
    assert(stopped.status==io::XucSender::SendStatus::sent && !stopped.control_sent && !stopped.fire_sent);
    assert(link.last_shoot_tx_ns()==fire.tx_time_ns);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    std::puts("SUMMARY sendresult=1 pose_history=1 fire_bypasses_throttle=1 expired_fire=0 stop=1");
    return 0;
  }
  const bool wheel_profile=argc==3 && std::strcmp(argv[2],"friction")==0;
  const bool startup_profile=argc==3 && std::strcmp(argv[2],"startup")==0;
  const auto start=std::chrono::steady_clock::now();bool pose=false,invalid=false,reset=false,unverified=false;
  while(true){
    const double t=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();if(t>(wheel_profile?6.9:3.7))break;
    pose|=link.rx_valid();invalid|=link.feedback_link_valid()&&!link.rx_valid();reset|=link.consume_tracker_reset();
    unverified|=link.rx_valid() && (link.sentinel_feedback().gimbal.flags&12)!=12 && !link.control_ready();
    io::Command c{};c.control=!(t>2.45 && t<2.65);c.shoot=t>.45 && t<.65;
    c.armor_detected=wheel_profile ? ((t>.35 && t<.55)||(t>5.95 && t<6.1)||(t>6.65 && t<6.75)) : true;
    if(wheel_profile){c.control=false;c.shoot=true;} // must never reach the wire as target/fire
    c.yaw=179*io::sentinel::deg_to_rad;c.pitch=12.5*io::sentinel::deg_to_rad;
    if((!wheel_profile && !c.control)||(wheel_profile && t>6.2 && t<6.4))link.stop();else link.send(c);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const auto a=link.imu_angles_rad(),g=link.gimbal_gyro_rad_s();
  const bool units=std::abs(a[0]/io::sentinel::deg_to_rad-22)<.001 &&
    std::abs(a[1]/io::sentinel::deg_to_rad-11)<.001 && std::abs(a[2]/io::sentinel::deg_to_rad+179)<.001 &&
    std::abs(g[0]-1)<.001 && std::abs(g[1]-2)<.001 && std::abs(g[2]+3)<.001;
  std::printf("SUMMARY pose=%d invalid=%d reset=%d unverified=%d units=%d crc=%llu\n",pose,invalid,reset,unverified,units,(unsigned long long)link.crc_errors());
  // A startup-cache stream has no fresh link while its first pose expires;
  // its Python peer separately asserts recovery references and TX reseeding.
  return pose && (startup_profile || invalid) && reset && (argc==3 || unverified) && units && link.crc_errors()?0:4;
}

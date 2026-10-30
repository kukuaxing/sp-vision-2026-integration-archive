// Offline PTY fixture only. Never pass a physical serial device.
#include "io/mission_bridge.hpp"
#include <iostream>
int main(int argc,char** argv){
  if(argc!=2)return 2;
  io::XucSender xuc(argv[1]);io::MissionBridge bridge(xuc);
  const auto end=io::XucSender::Clock::now()+std::chrono::seconds(17);
  while(io::XucSender::Clock::now()<end){
    const auto now=io::XucSender::Clock::now();
    bridge.frame(true,std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
    io::Command c{};c.control=true;c.armor_detected=true;c.shoot=true;
    c.yaw=0;c.pitch=0;xuc.send(c);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

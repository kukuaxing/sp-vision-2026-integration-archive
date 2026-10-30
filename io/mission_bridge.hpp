#pragma once
// Local atomic-file IPC. Only XucSender owns the physical serial port.
#include "io/xuc.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <unistd.h>

namespace io {
class MissionBridge {
  using Json=nlohmann::json;
  XucSender& xuc_;
  std::filesystem::path dir_;
  std::string instance_;
  std::atomic<bool> quit_{false};
  std::atomic<int64_t> frame_ns_{0},armor_ns_{0};
  std::atomic<bool> task_active_{false};
  std::thread worker_;
  static int64_t ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(
    XucSender::Clock::now().time_since_epoch()).count();}
  void loop(){
    int64_t last_status=0;bool nav_active=false;std::string error;
    while(!quit_.load()){
      const auto now=ns();bool active=false,task=false;
      float vx=0,vy=0,wz=0;uint32_t limit=0,generation=0;int64_t until=0;
      const auto feedback=xuc_.sentinel_feedback();
      try{
        std::ifstream f(dir_/"command.json");Json j;f>>j;
        if(j.at("schema")!=2||j.at("count_source")!="host_tx")
          throw std::runtime_error("Mission count schema mismatch");
        const auto stamp=j.at("monotonic_ns").get<int64_t>();
        if(j.at("vision_instance")==instance_&&j.at("mcu_boot")==feedback.boot_id&&
           now>=stamp&&now-stamp<100000000){
          active=j.at("nav_enabled").get<bool>();task=j.at("task_enabled").get<bool>();
          vx=j.at("vx").get<float>();vy=j.at("vy").get<float>();wz=j.at("wz").get<float>();
          limit=j.at("count_limit").get<uint32_t>();generation=j.at("count_generation").get<uint32_t>();
          if(!std::isfinite(vx)||!std::isfinite(vy)||!std::isfinite(wz)||
             std::abs(vx)+std::abs(vy)+std::abs(wz)>.0f+3.5f||
             std::abs(wz)>.5f|| (task&&(!active||vx||vy||wz)))
            throw std::runtime_error("Invalid local mission command");
          until=stamp+100000000;
        }
        error.clear();
      }catch(const std::exception& e){error=e.what();active=task=false;}
      // A camera stall never leaves the last task lease alive.
      task=task&&frame_ns_.load()>0&&now-frame_ns_.load()<300000000;
      task_active_.store(task);
      xuc_.mission_task_until(task?until:0,limit,generation);
      if(!task)xuc_.stop();
      if(active){
        if(!xuc_.send_navigation(vx,vy,wz,true))error="NAV send unavailable";
        nav_active=true;
      }else if(nav_active){xuc_.send_navigation(0,0,0,false);nav_active=false;}
      if(now-last_status>=50000000){
        try{
          Json j={{"schema",2},{"count_source","host_tx"},
            {"vision_instance",instance_},{"monotonic_ns",now},
            {"frame_ns",frame_ns_.load()},{"armor_last_ns",armor_ns_.load()},
            {"link_fresh",xuc_.mission_link_fresh()},{"control_ready",xuc_.control_ready()},
            {"pose_fresh",xuc_.rx_valid()},
            {"pose_generation",xuc_.pose_now().generation},
            {"mcu_boot",feedback.boot_id},{"mcu_state",feedback.link.state},
            {"mcu_reject",feedback.link.last_reject},
            {"count_generation",0},{"fire_requests_sent",xuc_.fire_requests_sent()},
            {"error",error}};
          const auto temp=dir_/"vision.json.tmp";
          {std::ofstream out(temp);out<<j.dump();if(!out)throw std::runtime_error("status write failed");}
          std::filesystem::rename(temp,dir_/"vision.json");
        }catch(const std::exception&){} // A stale/missing status revokes at the mission manager.
        last_status=now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    xuc_.mission_task_until(0,0,0);xuc_.stop();
    xuc_.send_navigation(0,0,0,false);
  }
public:
  bool fixed_center_allowed() const { return dir_.empty() || task_active_.load(); }
  explicit MissionBridge(XucSender& xuc):xuc_(xuc){
    const char* path=std::getenv("RM27_MISSION_DIR");
    if(!path||!*path)return;
    if(!xuc_.is_sentinel())throw std::runtime_error("Mission requires Sentinel");
    dir_=path;if(!std::filesystem::is_directory(dir_))throw std::runtime_error("Mission IPC directory missing");
    instance_=std::to_string(getpid())+"-"+std::to_string(ns());
    xuc_.configure_mission(true);worker_=std::thread([this]{loop();});
  }
  ~MissionBridge(){quit_.store(true);if(worker_.joinable())worker_.join();}
  void frame(bool armor,int64_t capture_ns){
    const auto now=ns();
    if(capture_ns<=0||now<capture_ns||now-capture_ns>300000000)return;
    frame_ns_.store(capture_ns);if(armor)armor_ns_.store(capture_ns);
  }
};
}

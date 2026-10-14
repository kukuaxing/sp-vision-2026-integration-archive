// Test harness accepts ONLY Linux pseudoterminals. It cannot open a robot port.
#include "io/rmv/debug_link.hpp"
#include <iostream>
#include <cassert>
int main(int argc,char** argv){
  if(argc!=5 || std::string(argv[1])!="--pty-offline" ||
     std::string(argv[2]).rfind("/dev/pts/",0)!=0)return 2;
  try{
    const bool negative=std::string(argv[3])=="negative";
    const bool fire=std::string(argv[3]).rfind("fire",0)==0;
    const bool trajectory=std::string(argv[3]).rfind("trajectory",0)==0;
    const bool batch=std::string(argv[3]).rfind("fire_batch",0)==0;
    const bool rpm=std::string(argv[3]).rfind("fire_batch_rpm",0)==0;
    io::rmv::DebugLink link(argv[2],460800,std::string(argv[3])!="readonly",1,std::stod(argv[4]),fire,trajectory,batch?20:0,rpm);
    unsigned wheel_records=0,trace_records=0,command_records=0,shot_records=0;
    auto start=io::rmv::debug_now_us();unsigned sent=0,shots=0;bool ready=false,used_prior_pair=false;
    std::optional<io::rmv::JointGoal> held;
    while(io::rmv::debug_now_us()-start<(batch?5500000:2300000)){
      const auto now=io::rmv::debug_now_us();auto s=link.snapshot(now-20000);
      ready|=s.ready;
      for(const auto& r:link.take_wheel_measurement()){
        if(r.kind==io::rmv::WheelMeasurementRecord::wheel){++wheel_records;
          if(r.has_feeder_trace){++trace_records;assert(r.feeder_position==-1234 &&
            r.command_current==-900 && r.feedback_current==-890 && r.feedback_age_us[0]==1000);}}
        else if(r.kind==io::rmv::WheelMeasurementRecord::command)++command_records;
        else ++shot_records;
      }
      used_prior_pair|=s.ready && s.joint_sample_us<s.newest_joint_sample_us;
      if(!s.error.empty()){std::cout<<"ERROR "<<s.error<<std::endl;return 1;}
      if(batch && !s.batch.error.empty()){std::cout<<"BATCH_ERROR "<<s.batch.error<<std::endl;return 0;}
      if(batch && s.batch.done()){
        std::cout<<"BATCH_DONE completed="<<s.batch.completed<<" shots="<<shots<<std::endl;
        if(rpm){assert(command_records==20 && shot_records==60 && wheel_records>600 && trace_records>600);
          assert(link.finish_wheel_measurement());std::cout<<"RPM_RESTORED samples="<<wheel_records<<std::endl;}
        return 0;
      }
      if(batch){
        if(!held && s.ready){held=s.joints;held->trajectory=false;}
        if(held && link.submit(*held))++sent;
      }else if(trajectory){
        auto c=link.control_snapshot(now-20000);assert(!c.ready || c.state_extrapolation_s<=.05);
        if(link.submit(io::rmv::JointGoal{.2,.1,.4,-.2,.1,-.1,true}))++sent;
      }else if(link.submit(io::rmv::JointGoal{fire?0.:negative?-1.:1.,fire?0.:negative?-1.:1.}))++sent;
      if(batch){if(held && link.fire_held_once(*held))++shots;}
      else if(fire&&link.fire_once({0,0},now-20000))++shots;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout<<"DONE ready="<<ready<<" submitted="<<sent<<" used_prior_pair="<<used_prior_pair<<" shots="<<shots<<std::endl;
  }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}

// Re-evaluate real historical primary observations, preserving recorded frame
// times/face labels. Not an end-to-end video replay or an impact measurement.
#include "tasks/auto_aim/fixed_center_model.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <string>
int main(int argc,char**argv) {
  if(argc<2)return 2;
  nlohmann::json reports=nlohmann::json::array();
  for(int arg=1;arg<argc;++arg) {
    std::ifstream file(argv[arg]);if(!file)return 2;
    auto_aim::FixedCenterModel model;
    uint64_t epoch=1,last_id=0;int64_t previous_ns=0;
    int frames=0,usable=0,ready=0,fits=0,malformed=0;
    std::string line;
    while(std::getline(file,line)) {
      try {
        const auto j=nlohmann::json::parse(line);++frames;
        const auto & m=j.at("model");const auto & o=j.at("observation");
        if(m.is_null()||o.is_null())continue;
        const auto & tr=j.at("tracker");
        const auto state=m.at("whole_state").get<std::vector<double>>();
        const auto xyz=o.at("xyz_world_m").get<std::vector<double>>();
        if(state.size()!=11||xyz.size()!=3)continue;
        const auto id=tr.at("id").get<uint64_t>();
        const auto time=static_cast<int64_t>(j.at("time_s").get<double>()*1e9);
        const bool rebuilt=id!=last_id||time<=previous_ns||tr.at("rebuilt").get<bool>();
        if(rebuilt)++epoch;
        last_id=id;previous_ns=time;
        auto_aim::FixedCenterModel::Input in;
        in.active=true;in.epoch=epoch;in.frame_ns=time;in.rebuilt=rebuilt;
        in.seed=Eigen::Map<const Eigen::VectorXd>(state.data(),11);
        in.context_valid=tr.at("identity").get<bool>()&&
          tr.at("state")!="lost"&&o.at("pose_valid").get<bool>()&&
          !(m.value("whole_trust_reject_mask",0u)&1u);
        if(in.context_valid && tr.at("updated").get<bool>() && tr.at("state")=="tracking") {
          ++usable;
          in.observations.push_back({tr.at("primary_face").get<int>(),Eigen::Vector3d(xyz.data())});
        }
        const auto out=model.update(in);ready+=out.ready;fits+=out.fitted;
      }catch(const std::exception&){++malformed;}
    }
    reports.push_back({{"source",argv[arg]},{"frames",frames},{"usable",usable},
      {"fixed_ready_frames",ready},{"successful_fits",fits},{"unreadable_records",malformed},
      {"scope","recorded primary observations and old labels only; no camera/UART/actuation"}});
  }
  std::cout<<reports.dump(2)<<'\n';
}

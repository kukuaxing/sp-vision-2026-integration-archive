#include "gyro_phase_scheduler.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

// Read-only model replay. Uses a fixed 25 ms processing age, not a recreation
// of serial/MCU timing. It measures clock/model consistency, never hit rate.
int main(int argc,char** argv) {
  using nlohmann::json;
  if(argc<2) return 2;
  json reports=json::array();
  for(int arg=1;arg<argc;++arg) {
    std::ifstream in(argv[arg]); if(!in) return 2;
    auto_aim::FixedCadenceTrigger clock;
    std::vector<std::pair<double,double>> errors;
    std::string line; std::size_t rows=0,eligible=0,slots=0;
    std::uint64_t previous_track=0,updates_max=0;
    double first=0,last=0;
    while(std::getline(in,line)) {
      const auto r=json::parse(line); ++rows;
      const auto& tr=r.at("transport");const auto& m=r.at("model");
      const auto& g=r.at("fire_gates"); const auto& pose=r.at("pose");
      if(!tr.is_object()||!m.is_object()||!g.is_object()||!pose.is_object())continue;
      const auto track=r.at("tracker").value("id",std::uint64_t{0});
      const bool active=g.is_object()&&g.value("auto",false)&&pose.value("context_matches",false);
      if(!active||m.value("phase_reset",false)||(previous_track&&track!=previous_track))clock.reset();
      previous_track=track;
      if(!active||!m.value("fresh_target_observed",false)||
          !g.value("high_speed_prediction",false)||!tr.value("phase_window_geometry_trusted",false))continue;
      const auto& x=m.at("whole_state");if(!x.is_array()||x.size()!=11)continue;
      const double sample=r.at("time_s"),phase=x[6].get<double>()-std::atan2(x[2].get<double>(),x[0].get<double>());
      const double seed=std::copysign(tr.value("gyro_event_configured_omega_rad_s",4.56),x[7].get<double>());
      const double offset=tr.value("periodic_phase_offset_ms",460.)*.001;
      const int stride=tr.value("periodic_every_n_faces",2);
      const auto s=clock.observe(sample+.025,sample,phase,seed,.7765,
        4.*M_PI/180.,.02,.1,offset,stride,.05,.002,.25);
      ++eligible;updates_max=std::max(updates_max,clock.calibration_updates());
      if(!s.geometry_aligned)continue;
      if(errors.empty())first=sample;last=sample;
      errors.emplace_back(sample,std::abs(clock.visual_phase_error_rad())*180./M_PI);
      if(s.valid){++slots;clock.mark_fired();}
    }
    std::vector<double> all,tail;
    for(const auto& v:errors){all.push_back(v.second);if(v.first>=last-10.)tail.push_back(v.second);}
    const auto quant=[](std::vector<double> v,double q){if(v.empty())return 0.;std::sort(v.begin(),v.end());return v[std::min(v.size()-1,std::size_t(q*(v.size()-1)))];};
    reports.push_back({{"file",argv[arg]},{"rows",rows},{"eligible",eligible},{"span_s",last-first},
      {"slots",slots},{"updates_max",updates_max},{"median_abs_phase_deg",quant(all,.5)},
      {"p90_abs_phase_deg",quant(all,.9)},{"last10s_median_abs_phase_deg",quant(tail,.5)},
      {"last10s_p90_abs_phase_deg",quant(tail,.9)},
      {"limitation","25ms processing-age model replay; not measured muzzle timing or hit rate"}});
  }
  std::cout<<reports.dump(2)<<'\n';
}

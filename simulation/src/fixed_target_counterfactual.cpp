// Evaluator-only frozen-shot interventions. No estimation, gate decisions,
// permissions or new schedules are generated; never a deployable controller.
#include "fixed_target_world.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
using Json=nlohmann::json;

static fixed_sim::World world_for(const Json& e,const Json& r) {
  const auto& t=e.at("scenario").at("target");fixed_sim::World w;
  const auto c=t.at("center_m").get<std::array<double,3>>();w.center={c[0],c[1],c[2]};
  w.radii=t.at("radii_m").get<std::array<double,4>>();w.omega=r.at("omega_rad_s");
  w.initial_phase=r.at("initial_phase_rad");w.epoch=0;w.height_delta=t.at("height_delta_m");
  w.width=t.at("armor_width_m");w.height=t.at("armor_hit_height_m");
  w.tilt=t.at("armor_pitch_deg").get<double>()*fixed_sim::pi/180;
  w.gravity=e.at("scenario").at("plant").at("gravity_mps2");return w;
}
static Json delay_grid(const Json& data) {
  if(!data.value("complete",false))throw std::invalid_argument("incomplete source data");
  Json rows=Json::array();int checked=0;
  for(const auto& e:data.at("experiments")) {
    const auto& p=e.at("scenario").at("plant");
    const double mean=p.at("muzzle_delay_s"),sigma=p.at("muzzle_delay_jitter_s");
    if(!(mean>0&&sigma>0))throw std::invalid_argument("delay grid requires positive original mean/sigma");
    for(double shift_ms:{-20.,-10.,0.,10.,20.})for(double sigma_ms:{0.,10.,25.,40.}) {
      int hits=0,total=0,lost=0,gained=0;double worst=1.;Json runs=Json::array();std::map<int,int> directions;
      for(const auto& r:e.at("runs")) {
        const auto w=world_for(e,r);int count=0;
        for(const auto& s:r.at("shots")) {
          const double observed=s.at("muzzle_s").get<double>()-s.at("issue_s").get<double>();
          // An observed zero could have been clipped by the original plant;
          // then its latent standardized draw cannot be recovered uniquely.
          if(observed<=0)throw std::invalid_argument("clipped original delay cannot be standardized");
          const double z=(observed-mean)/sigma;
          fixed_sim::Projectile shot;
          shot.muzzle_time=s.at("issue_s").get<double>()+std::max(0.,mean+shift_ms*.001+z*sigma_ms*.001);
          const double yaw=s.at("world_yaw"),pitch=s.at("world_elevation"),speed=s.at("physical_speed_mps");
          shot.velocity=speed*Eigen::Vector3d(std::cos(pitch)*std::cos(yaw),std::cos(pitch)*std::sin(yaw),std::sin(pitch));
          const auto hit=fixed_sim::collide(w,shot,p.at("collision_step_s"),p.at("flight_timeout_s"));
          const bool valid=hit&&hit->normal_speed>=p.value("min_normal_speed_mps",0.);
          const bool actual=s.at("valid_hit");
          if(shift_ms==0&&std::abs(sigma_ms*.001-sigma)<1e-12) {
            if(bool(hit)!=s.at("geometric_hit").get<bool>()||valid!=actual)
              throw std::runtime_error("delay-grid zero intervention disagrees with recorded score");
            ++checked;
          }
          ++total;count+=valid;lost+=actual&&!valid;gained+=!actual&&valid;
        }
        hits+=count;directions[r.at("direction").get<int>()]+=count;
        worst=std::min(worst,static_cast<double>(count)/r.at("ammo_budget").get<int>());
        runs.push_back({{"seed",r.at("seed")},{"direction",r.at("direction")},
          {"initial_phase_rad",r.at("initial_phase_rad")},{"hits",count}});
      }
      rows.push_back({{"experiment",e.at("name")},{"physical_mean_shift_ms",shift_ms},
        {"physical_sigma_ms",sigma_ms},{"hits",hits},{"shots",total},{"worst_fraction",worst},
        {"negative_hits",directions[-1]},{"positive_hits",directions[1]},{"lost",lost},{"gained",gained},{"runs",runs}});
    }
  }
  return {{"schema",1},{"scope","evaluator-only physical delay mean/sigma grid; frozen schedules/rays; not a controller or achieved improvement"},
    {"physical_device_opened",false},{"truth_available_only_to_evaluator",true},
    {"zero_intervention_shots_verified",checked},{"rows",rows}};
}
int main(int argc,char** argv) {
 try {
  if(argc!=3&&!(argc==4&&std::string(argv[3])=="--delay-grid"))
    throw std::invalid_argument("usage: fixed_target_counterfactual RESULT.json OUTPUT.json [--delay-grid]");
  Json data;std::ifstream(argv[1])>>data;
  if(argc==4) {
    const auto result=delay_grid(data);std::ofstream out(argv[2]);out<<result.dump(2)<<'\n';
    if(!out)throw std::runtime_error("cannot save delay grid");
    std::cout<<"PASS delay-grid original replay "<<result.at("zero_intervention_shots_verified")
      <<" shots; "<<result.at("rows").size()<<" diagnostic cells\n";return 0;
  }
  Json rows=Json::array();int checked=0;
  for(const auto& e:data.at("experiments")) {
    bool has_servo=true;
    for(const auto& r:e.at("runs"))for(const auto& s:r.at("shots"))
      has_servo=has_servo&&s.contains("muzzle_target_yaw_rad");
    std::vector<std::string> treatments{"actual"};
    if(has_servo)treatments.insert(treatments.end(),{"ideal_current_target","ideal_yaw_only","ideal_pitch_only","hold_issue_target"});
    for(const auto& treatment:treatments)for(double shift_ms:{-20.,-10.,-5.,0.,5.,10.,20.}) {
      int hits=0,total=0,lost=0,gained=0;double worst=1.;Json per_run=Json::array();
      std::map<int,int> directions;
      for(const auto& r:e.at("runs")) {
        const auto w=world_for(e,r);const auto& p=e.at("scenario").at("plant");int count=0;
        for(const auto& s:r.at("shots")) {
          double yaw=s.at("world_yaw"),elevation=s.at("world_elevation");
          // Retain exact realized barrel bias/spread by replacing only the
          // recorded servo contribution, at the SAME frozen launch event.
          if(treatment=="ideal_current_target"||treatment=="ideal_yaw_only")
            yaw+=s.at("muzzle_target_yaw_rad").get<double>()-s.at("muzzle_servo_yaw_rad").get<double>();
          if(treatment=="ideal_current_target"||treatment=="ideal_pitch_only")
            elevation+=s.at("muzzle_target_elevation_rad").get<double>()-s.at("muzzle_servo_elevation_rad").get<double>();
          if(treatment=="hold_issue_target") {
            yaw+=s.at("issue_desired_yaw_rad").get<double>()-s.at("muzzle_servo_yaw_rad").get<double>();
            elevation+=s.at("issue_desired_elevation_rad").get<double>()-s.at("muzzle_servo_elevation_rad").get<double>();
          }
          fixed_sim::Projectile shot;shot.muzzle_time=s.at("muzzle_s").get<double>()+shift_ms*.001;
          const double speed=s.at("physical_speed_mps");
          shot.velocity=speed*Eigen::Vector3d(std::cos(elevation)*std::cos(yaw),std::cos(elevation)*std::sin(yaw),std::sin(elevation));
          const auto hit=fixed_sim::collide(w,shot,p.at("collision_step_s"),p.at("flight_timeout_s"));
          const bool valid=hit&&hit->normal_speed>=p.value("min_normal_speed_mps",0.);
          const bool actual=s.at("valid_hit");
          if(treatment=="actual"&&shift_ms==0.) {
            if(bool(hit)!=s.at("geometric_hit").get<bool>()||valid!=actual)
              throw std::runtime_error("zero-intervention replay disagrees with recorded score");
            ++checked;
          }
          ++total;count+=valid;lost+=actual&&!valid;gained+=!actual&&valid;
        }
        hits+=count;directions[r.at("direction").get<int>()]+=count;
        worst=std::min(worst,static_cast<double>(count)/r.at("ammo_budget").get<int>());
        per_run.push_back({{"seed",r.at("seed")},{"direction",r.at("direction")},
          {"initial_phase_rad",r.at("initial_phase_rad")},{"hits",count}});
      }
      rows.push_back({{"experiment",e.at("name")},{"treatment",treatment},{"launch_shift_ms",shift_ms},
        {"hits",hits},{"shots",total},{"worst_fraction",worst},{"lost",lost},{"gained",gained},
        {"negative_hits",directions[-1]},{"positive_hits",directions[1]},{"runs",per_run}});
    }
  }
  Json result={{"schema",1},{"scope","post-hoc frozen-shot diagnostic interventions; not a new controller or deployable hit rate"},
    {"physical_device_opened",false},{"truth_available_only_to_evaluator",true},
    {"zero_intervention_shots_verified",checked},{"rows",rows}};
  std::ofstream out(argv[2]);out<<result.dump(2)<<'\n';
  std::cout<<"PASS frozen-shot replay "<<checked<<" shots; "<<rows.size()<<" diagnostic cells\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}

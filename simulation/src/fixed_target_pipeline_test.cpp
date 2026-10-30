#include "fixed_target_pipeline.hpp"
#include <cassert>
#include <iostream>
using fixed_sim::TimingPipeline;
using Json=nlohmann::json;
static Eigen::Quaterniond motion(double t){return Eigen::Quaterniond(Eigen::AngleAxisd(.3*(t-1),Eigen::Vector3d::UnitZ()));}
static void fill(TimingPipeline& p,double end) {
  for(int i=0;i<=int(std::llround((end-1)/.005));++i){const double t=1+i*.005;p.sample(t,motion(t));p.deliver(t);}
}
int main() {
  TimingPipeline off(Json{});assert(!off.enabled);
  TimingPipeline normal(Json::object());fill(normal,1.2);
  auto a=normal.capture(1.19,1.2);assert(a.valid);
  assert(std::abs(a.rpy_rad[2]-.3*.19)<1e-6);
  assert(!normal.capture(1.21,1.2).valid);
  TimingPipeline gap(Json{{"faults",Json::array({{{"start_s",.05},{"end_s",.3},{"drop_imu",true}}})}});
  fill(gap,1.2);assert(!gap.latest(1.2).valid);
  TimingPipeline freeze(Json{{"faults",Json::array({{{"start_s",.05},{"end_s",.3},{"freeze_imu_clock",true}}})}});
  fill(freeze,1.2);assert(!freeze.latest(1.2).valid);
  TimingPipeline restart(Json{{"faults",Json::array({{{"start_s",.08},{"end_s",.1},{"new_generation",true}}})}});
  fill(restart,1.1);assert(restart.generation()==2&&restart.consume_reset()&&!restart.consume_reset());
  assert(!restart.capture(1.06,1.1).valid);assert(restart.capture(1.09,1.1).valid);
  TimingPipeline hidden(Json{{"faults",Json::array({{{"start_s",0.},{"end_s",1.},{"sensor_latency_s",.03}}})}});
  fill(hidden,1.2);a=hidden.capture(1.19,1.2);assert(a.valid&&!a.sensor_latency_known);
  assert(std::abs(a.rpy_rad[2]-.3*(.19-.03))<1e-6);
  TimingPipeline transport(Json{{"faults",Json::array({{{"start_s",0.},{"end_s",1.},{"imu_transport_delay_s",.025}}})}});
  fill(transport,1.2);a=transport.capture(1.19,1.2);assert(a.valid&&!a.synchronized);
  assert(std::abs(a.rpy_rad[2]-.3*(.19-.025))<1e-6);
  TimingPipeline causal(Json{{"faults",Json::array({{{"start_s",0.},{"end_s",1.},{"imu_transport_delay_s",.05}}})}});
  fill(causal,1.1);causal.change_reference();assert(causal.generation()==1&&!causal.consume_reset());
  for(int i=21;i<=32;++i) {
    const double t=1+i*.005;causal.sample(t,motion(t));causal.deliver(t);
    if(t<1.155-1e-9)assert(causal.generation()==1);
  }
  assert(causal.generation()==2&&causal.consume_reset());
  // Invalid orientation revokes the old cache; late old-session frames cannot revive it.
  TimingPipeline invalid(Json{{"faults",Json::array({{{"start_s",.05},{"end_s",.3},{"invalid_pose",true}}})}});
  fill(invalid,1.2);assert(!invalid.latest(1.2).valid);
  bool rejected=false;try{TimingPipeline bad(Json{{"faults",Json::array({{{"start_s",.1},{"end_s",.2},{"image_delay_s",-.01}}})}});}
  catch(const std::invalid_argument&){rejected=true;}assert(rejected);
  std::cout<<"PASS opt-in clock fixture: interpolation, stale/frozen source, reset isolation, hidden latency and invalid inputs\n";
}

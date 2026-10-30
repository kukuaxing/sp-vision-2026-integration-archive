#include "tasks/auto_aim/high_speed_vision.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tools/extended_kalman_filter.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace auto_aim;
using Clock = std::chrono::steady_clock;
static Clock::time_point at(double t) {
  return Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(t)));
}
int main() {
  // A linear observation must have the same posterior for 1..8 iterations.
  for (int iterations : {1,2,4,8}) {
    Eigen::VectorXd x(1),z(1); x<<0;z<<4;
    Eigen::MatrixXd P(1,1),R(1,1);P<<1;R<<3;
    tools::ExtendedKalmanFilter filter(x,P);
    const auto minus=[](const Eigen::VectorXd&a,const Eigen::VectorXd&b)->Eigen::VectorXd{return a-b;};
    filter.update_iterated(z,R,[](const Eigen::VectorXd&a)->Eigen::VectorXd{return a;},
      [](const Eigen::VectorXd&)->Eigen::MatrixXd{return Eigen::MatrixXd::Identity(1,1);},minus,minus,iterations,1e-9,100);
    assert(std::abs(filter.x[0]-1)<1e-10);
    assert(std::abs(filter.P(0,0)-0.75)<1e-10);
  }
  for (double omega : {-4.56,4.56}) {
    FixedCenterModel fit;
    FixedCenterModel interrupted_fit;
    FixedCenterModel::Input last_input;
    FixedCenterModel::Output interrupted_output;
    bool interrupted_ready = false;
    FixedCenterOwnership owner;
    FixedCenterModel::Output output;
    FixedCenterOwnership::Output source;
    for (int k=0;k<300;++k) {
      const double time=10+k*0.02, phase=0.2+omega*(time-10);
      FixedCenterModel::Input input;
      input.active=input.context_valid=true;input.epoch=4;
      input.frame_ns=static_cast<int64_t>(time*1e9);
      input.seed=Eigen::VectorXd::Zero(11);
      input.seed[0]=3.02;input.seed[2]=0.095;input.seed[6]=std::remainder(phase,2*M_PI);
      input.seed[7]=omega;input.seed[8]=0.20;input.seed[9]=0.02;
      for(int face=0;face<4;++face) {
        const double angle=phase+face*M_PI/2, r=face%2?0.23:0.20;
        // Same physical faces are visible on successive complete passages.
        if(std::abs(std::remainder(angle-std::atan2(.1,3.),2*M_PI))<0.85)
          input.observations.push_back({face,{3-r*std::cos(angle),.1-r*std::sin(angle),face%2?.12:.1}});
      }
      output=fit.update(input);
      last_input=input;
      auto interrupted=input;
      if(k%23==22) {
        interrupted.context_valid=false;interrupted.capture_pose_gap=true;
        // A malformed observation on a missing-pose frame must be ignored.
        interrupted.observations={{0,{9,9,9}}};
      }
      interrupted_output=interrupted_fit.update(interrupted);
      if(k%23==22)assert(!interrupted_output.ready && !interrupted_output.fitted && interrupted_output.state.size()==0);
      else interrupted_ready|=interrupted_output.ready;
      source=owner.update({input.frame_ns,input.epoch,output.generation,true,true,false,output.ready});
    }
    assert(output.ready && source.available);
    assert(interrupted_ready && interrupted_output.ready);
    assert(std::abs(output.state[0]-3)<0.003 && std::abs(output.state[2]-.1)<0.003);
    assert(std::abs(output.state[7]-omega)<0.01);
    assert(output.state[1]==0 && output.state[3]==0);
    // Keep only history over a certified capture-pose gap, never an output
    // forecast. Repeated gaps do not renew the last valid context deadline.
    const auto generation=output.generation;
    const auto samples=output.observations;
    for(int ms:{20,60,140}) {
      auto gap=last_input;gap.frame_ns+=int64_t(ms)*1000000;
      gap.context_valid=false;gap.capture_pose_gap=true;
      const auto held=fit.update(gap);
      assert(!held.ready && !held.fitted && held.state.size()==0);
      assert(held.observations==samples && held.generation==generation);
    }
    auto expired=last_input;expired.frame_ns+=150000000;
    expired.context_valid=false;expired.capture_pose_gap=true;
    assert(fit.update(expired).observations==0);
    // Each other invalidation still clears learning immediately.
    for(int fault=0;fault<7;++fault) {
      FixedCenterModel probe;
      auto good=last_input;good.frame_ns+=1000000000;
      const auto before=probe.update(good);assert(before.observations>0);
      auto bad=good;bad.frame_ns+=20000000;bad.context_valid=false;bad.capture_pose_gap=true;
      if(fault==0)bad.capture_pose_gap=false;
      if(fault==1)++bad.epoch;
      if(fault==2)bad.rebuilt=true;
      if(fault==3)bad.seed[7]=-bad.seed[7];
      if(fault==4)bad.frame_ns=good.frame_ns;
      if(fault==5)bad.seed[0]=NAN;
      if(fault==6)bad.active=false;
      const auto after=probe.update(bad);
      assert(!after.ready && after.observations==0);
    }
    source=owner.update({16000000000LL,4,output.generation,true,false,false,false,true});
    assert(source.fixed && !source.available && !source.changed);
    source=owner.update({16020000000LL,4,output.generation,true,true,false,true});
    assert(source.fixed && source.available && !source.changed);
    source=owner.update({16040000000LL,4,output.generation,true,true,false,false});
    assert(!source.available); // A remembered owner cannot authorize stale data.
    source=owner.update({16170000000LL,4,output.generation,true,false,false,false,true});
    assert(!source.fixed && !source.available); // 150ms since last ready, no renewal.
    FixedCenterModel::Input bad;
    bad.active=true;bad.context_valid=false;
    assert(!fit.update(bad).ready);
  }
  assert(!past_forecast_quality_allows(true,false,true,0.0,.1));
  assert(!past_forecast_quality_allows(true,true,false,0.0,.1));
  assert(!past_forecast_quality_allows(true,true,true,NAN,.1));
  assert(past_forecast_quality_allows(true,true,true,.01,.1));
  const auto yaml=YAML::Load("xuc_high_speed_strategy: teammate_prediction");
  HighSpeedVision integration(yaml);
  // Exercise the real caller adapter, including its NaN range during one
  // missing frame. The retained metadata must not authorize that frame.
  for(double sign:{-1.,1.}) {
    HighSpeedVision adapter(yaml);
    FacePredictionResidual::Input good;
    good.frame_ns=10000000000LL;good.epoch=71;
    good.context_valid=good.high_speed_mode=good.observations_valid=true;
    good.omega_rad_s=sign*4.56;good.center_range_m=3;
    const auto initial=adapter.update_residual(good,true);
    auto gap=good;gap.frame_ns+=20000000;
    gap.context_valid=gap.observations_valid=gap.forecast_valid=false;
    gap.omega_rad_s=gap.center_range_m=NAN;
    const auto held=adapter.update_residual(gap,true);
    assert(held.generation==initial.generation);
    assert(!held.eligible && !held.quality_eligible && !adapter.quality_allows(held));
    assert(gap.context_valid==false && gap.observations_valid==false && gap.forecast_valid==false);
    assert(gap.center_range_m==3 && gap.omega_rad_s==sign*4.56);
    // Repeated missing frames must not renew the remembered valid time.
    for(int n=2;n<=7;++n){gap.frame_ns=good.frame_ns+n*20000000;gap.center_range_m=gap.omega_rad_s=NAN;
      const auto still_held=adapter.update_residual(gap,true);assert(still_held.generation==initial.generation);}
    gap.frame_ns=good.frame_ns+150000000;gap.center_range_m=gap.omega_rad_s=NAN;
    const auto expired=adapter.update_residual(gap,true);
    assert(expired.generation>initial.generation && !adapter.quality_allows(expired));
    good.frame_ns+=200000000;adapter.update_residual(good,true);
    gap=good;gap.frame_ns+=20000000;gap.context_valid=gap.observations_valid=false;
    gap.center_range_m=NAN;gap.omega_rad_s=-sign*4.56;
    const auto reversed_context=adapter.update_residual(gap,true);
    assert(!adapter.quality_allows(reversed_context));
    // Target replacement, invalid sensor/navigation context and rebuild each
    // discard metadata instead of creating a stale permission.
    for(int fault=0;fault<3;++fault){
      good.frame_ns+=200000000;adapter.update_residual(good,true);
      gap=good;gap.frame_ns+=20000000;gap.context_valid=gap.observations_valid=false;
      gap.center_range_m=gap.omega_rad_s=NAN;
      if(fault==0)++gap.epoch;if(fault==1)gap.rebuilt=true;
      const auto invalid=adapter.update_residual(gap,fault!=2);
      assert(!std::isfinite(gap.center_range_m) && !adapter.quality_allows(invalid));
    }
  }
  for (double sign : {-1.,1.}) {
    HighSpeedVision guard(yaml);
    guard.observe_phase(10000000000LL,0.,true);
    const auto pass=guard.passage(0,sign*4.56,10010000000LL,0);
    assert(guard.passage_available(0,pass));
    // A failed serial write does not call commit; the opportunity survives.
    assert(guard.passage_available(0,pass));
    guard.commit_passage(0,pass);
    assert(!guard.passage_available(0,pass));
    for(int k=1;k<=80;++k)guard.observe_phase(10000000000LL+k*20000000,
      std::remainder(sign*4.56*k*.02,2*M_PI),true);
    const auto next=guard.passage(0,sign*4.56,11610000000LL,0);
    assert(next!=pass && guard.passage_available(0,next));
  }
  FacePredictionResidual::Output missing;
  assert(!integration.quality_allows(missing));
  FacePredictionResidual residual({true,.3*M_PI/180.,true});
  FacePredictionResidual::Input input;
  input.context_valid=input.high_speed_mode=input.observations_valid=input.forecast_valid=true;
  input.epoch=1;input.omega_rad_s=4.56;input.center_range_m=3;
  input.delay_s=.33;input.speed_mps=22;input.face=0;input.forecast_alignment_rad=0;
  for(int k=0;k<10;++k) {
    input.frame_ns=10000000000LL+k*20000000;
    input.prediction_ns=input.frame_ns+200000000;
    input.raw_yaw_rad=.02;input.observations[0]=std::array<double,2>{3.,0.};
    const auto status=residual.update(input);
    assert(!status.ready && !status.quality_ready); // future has not happened
    assert(status.correction_rad==0);
  }
  input.frame_ns+=20000000;input.omega_rad_s=-4.56;
  const auto reversed=residual.update(input);
  assert(!reversed.ready && reversed.correction_rad==0);
  // Repeated *independent* passages eventually mature a constant visual bias;
  // predictions are checked only against later observations of the same face.
  for(double omega:{-4.56,4.56}) {
    FacePredictionResidual monitor({true,.3*M_PI/180.,true});
    HighSpeedVision intermittent(yaml);
    bool matured=false,corrected=false,intermittent_matured=false;
    for(int k=0;k<750;++k) {
      const double time=20+k*.02, horizon=.30;
      FacePredictionResidual::Input in;
      in.frame_ns=static_cast<int64_t>(time*1e9);in.prediction_ns=in.frame_ns+300000000;
      in.epoch=2;in.context_valid=in.high_speed_mode=in.observations_valid=true;
      in.omega_rad_s=omega;in.center_range_m=3;in.delay_s=.33;in.speed_mps=22;
      for(int face=0;face<4;++face) {
        const double phase=omega*(time-20)+face*M_PI/2;
        if(std::abs(std::remainder(phase,2*M_PI))<1.05)
          in.observations[face]=std::array<double,2>{3-.2*std::cos(phase),-.2*std::sin(phase)};
        const double future=phase+omega*horizon;
        if(std::abs(std::remainder(future,2*M_PI))<.75) {
          in.forecast_valid=true;in.face=face;
          in.raw_yaw_rad=std::atan2(-.2*std::sin(future),3-.2*std::cos(future))+.001;
          in.forecast_alignment_rad=std::remainder(future,2*M_PI);
        }
      }
      const auto status=monitor.update(in);
      auto intermittent_input=in;
      const bool dropped=k%23==22;
      if(dropped) {
        intermittent_input.context_valid=intermittent_input.observations_valid=intermittent_input.forecast_valid=false;
        intermittent_input.omega_rad_s=intermittent_input.center_range_m=NAN;
        intermittent_input.observations={};
      }
      const auto intermittent_status=intermittent.update_residual(intermittent_input,true);
      if(dropped)assert(!intermittent_status.eligible && !intermittent.quality_allows(intermittent_status));
      else intermittent_matured|=intermittent_status.quality_ready;
      matured|=status.quality_ready;
      corrected|=std::abs(status.correction_rad)>.0003;
      assert(std::abs(status.correction_rad)<=.6*M_PI/180.+1e-12);
    }
    assert(matured && corrected && intermittent_matured);
  }
  // Production Aimer: fixed-face ballistic iteration, both signs, explicit
  // direction delays; no camera/UART and no impact-timing claim.
  const auto config=std::filesystem::temp_directory_path()/"rm27_teammate_aim_fixture.yaml";
  {std::ofstream out(config);out<<"yaw_offset: 0\npitch_offset: 0\ncomming_angle: 60\nleaving_angle: 20\n"
    "high_speed_delay_time: .33\nlow_speed_delay_time: .33\ndecision_speed: .8\n"
    "high_speed_delay_time_cw: .30\nhigh_speed_delay_time_ccw: .36\n";}
  Aimer aimer(config.string());
  std::filesystem::remove(config);
  for(double omega:{-4.56,4.56}) {
    bool solved=false;
    for(int k=0;k<80;++k) {
      Target target(3.,omega,.2,.02);
      auto state=target.ekf().x;state[6]=k*.02;state[4]=.1;
      const auto fixed=target.with_fixed_center_prediction(state);assert(fixed);
      // Compare the allocation-free fixed-center sampler against the original
      // full prediction, including wrap boundaries and both physical heights.
      assert(!target.fixed_center_armor_at(0,at(.4)));
      const auto prior_mean=fixed->ekf().x;
      const auto prior_covariance=fixed->ekf().P;
      for (double dt : {0., .33, .47, 1.4}) {
        auto full=*fixed;full.predict(at(dt));
        const auto faces=full.whole_ekf_armor_xyza_list();
        for(int face=0;face<4;++face) {
          const auto sampled=fixed->fixed_center_armor_at(face,at(dt));assert(sampled);
          assert((*sampled-faces[face]).norm()<1e-14);
        }
      }
      assert((fixed->ekf().x-prior_mean).norm()==0);
      assert((fixed->ekf().P-prior_covariance).norm()==0);
      const auto original=target.ekf().x;
      AimPredictionTrace trace;
      const auto command=aimer.aim({*fixed},at(0),22,false,&trace,AimGeometry::whole_ekf,true);
      assert((target.ekf().x-original).norm()==0);
      if(!command.control)continue;
      solved=true;assert(trace.valid && trace.converged && trace.final_point_selection);
      assert(std::abs(trace.delay_s-(omega>0?.36:.30))<1e-12);
      assert(std::abs(trace.flight_time_used_s-trace.flight_time_s)<1e-5);
      const double dt=trace.prediction_ns*1e-9;
      const double phase=std::remainder(state[6]+omega*dt+trace.face_id*M_PI/2,2*M_PI);
      assert(std::abs(std::remainder(aimer.debug_aim_point.xyza[3]-phase,2*M_PI))<1e-8);
    }
    assert(solved);
  }
  std::cout<<"teammate_vision_test: PASS both directions, ownership/loss, causal residuals, rigid-face iterations, linear KF\n";
}

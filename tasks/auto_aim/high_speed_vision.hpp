#pragma once
#include <yaml-cpp/yaml.h>
#include "fixed_center_model.hpp"
#include "face_prediction_residual.hpp"
#include "high_speed_fire_quality.hpp"

namespace auto_aim {
// Vision-only adaptation boundary. No UART, IMU axes, mission counts or actuator
// state is interpreted here. The caller supplies already synchronized NAV poses.
class HighSpeedVision {
public:
  explicit HighSpeedVision(const YAML::Node & yaml)
  : enabled(yaml["xuc_high_speed_strategy"] &&
      yaml["xuc_high_speed_strategy"].as<std::string>() == "teammate_prediction"),
    quality_required(yaml["xuc_high_speed_prediction_quality_enabled"]
      ? yaml["xuc_high_speed_prediction_quality_enabled"].as<bool>() : true),
    quality_limit_rad(degrees(yaml, "xuc_high_speed_prediction_quality_max_error_deg", 0.45)),
    residual(FacePredictionResidual::Config{true, 0.3 * M_PI / 180.0, true}),
    pitch(HighSpeedPitchGate::Config{true,
      degrees(yaml, "xuc_fire_pitch_tolerance_deg", 0.4),
      degrees(yaml, "xuc_fire_pitch_tolerance_deg", 0.4),
      degrees(yaml, "xuc_high_speed_pitch_variation_deg", 0.15), pitch_stability_ns(yaml)})
  {
    if (yaml["xuc_high_speed_strategy"]) {
      const auto strategy = yaml["xuc_high_speed_strategy"].as<std::string>();
      if (strategy != "teammate_prediction" && strategy != "periodic")
        throw std::invalid_argument("Unknown xuc_high_speed_strategy");
    }
  }
  const bool enabled, quality_required;
  const double quality_limit_rad;
  FixedCenterModel model;
  FixedCenterOwnership ownership;
  FacePredictionResidual residual;
  HighSpeedPitchGate pitch;
  FacePredictionResidual::Output previous_residual;
  std::uint64_t epoch = 1;
  bool fixed_owned = false;
  FacePredictionResidual::Output update_residual(FacePredictionResidual::Input & in, bool same_context) {
    // Preserve *context metadata*, not a forecast or firing permission, over
    // a short qualification gap. The caller used to supply NaN range here,
    // defeating FacePredictionResidual's existing same-context retention.
    const bool timed=in.frame_ns>0;
    const bool discontinuity=in.rebuilt || !same_context || !in.high_speed_mode || !in.epoch ||
      context_epoch_!=in.epoch || !timed ||
      (context_time_ && (in.frame_ns<=context_time_ || in.frame_ns-context_time_>=150000000));
    if(discontinuity){context_time_=0;context_epoch_=0;}
    if(in.context_valid && same_context && in.high_speed_mode && timed && in.epoch &&
       std::isfinite(in.omega_rad_s) && std::abs(in.omega_rad_s)>.1 &&
       std::isfinite(in.center_range_m) && in.center_range_m>0) {
      context_time_=in.frame_ns;context_epoch_=in.epoch;
      context_omega_=in.omega_rad_s;context_range_=in.center_range_m;
    } else if(context_time_ && !discontinuity &&
       (!std::isfinite(in.omega_rad_s) ||
        (in.omega_rad_s*context_omega_>0 && std::abs(in.omega_rad_s-context_omega_)<=1.5))) {
      in.omega_rad_s=context_omega_;in.center_range_m=context_range_;
      // Do not refresh context_time_ or change context_valid/forecast_valid/
      // observations_valid. A run of missing frames still expires at 150ms.
    } else {context_time_=0;context_epoch_=0;}
    return residual.update(in);
  }
  void reset_passages() {
    pass_time_=0; fired_pass_.fill(std::numeric_limits<int64_t>::min());
  }
  void observe_phase(int64_t time, double phase, bool valid) {
    if (!valid || !std::isfinite(phase)) return;
    if (pass_time_ && (time <= pass_time_ || time-pass_time_ >= 150000000)) reset_passages();
    unwrapped_phase_ = pass_time_ ? unwrapped_phase_ + std::remainder(phase-last_phase_,2*M_PI) : phase;
    last_phase_=phase;pass_time_=time;
  }
  int64_t passage(int face,double omega,int64_t impact,double bearing) const {
    if (!pass_time_ || face<0 || face>=4 || !std::isfinite(omega) || !std::isfinite(bearing))
      return std::numeric_limits<int64_t>::min();
    const double phase=unwrapped_phase_+omega*(impact-pass_time_)/1e9+face*M_PI/2-bearing;
    return static_cast<int64_t>(std::floor((phase+M_PI)/(2*M_PI)));
  }
  bool passage_available(int face,int64_t pass) const {
    return face>=0 && face<4 && pass!=std::numeric_limits<int64_t>::min() && fired_pass_[face]!=pass;
  }
  void commit_passage(int face,int64_t pass) {
    if (passage_available(face,pass)) fired_pass_[face]=pass;
  }
  bool quality_allows(const FacePredictionResidual::Output & value) const {
    return past_forecast_quality_allows(quality_required, value.quality_ready,
      value.quality_eligible && value.eligible, value.past_error_bound_rad, quality_limit_rad);
  }
private:
  static int64_t pitch_stability_ns(const YAML::Node & yaml) {
    const double ms = yaml["xuc_high_speed_pitch_stable_ms"]
      ? yaml["xuc_high_speed_pitch_stable_ms"].as<double>() : 30.0;
    if (!std::isfinite(ms) || ms < 5.0 || ms > 200.0)
      throw std::invalid_argument("xuc_high_speed_pitch_stable_ms must be finite in [5,200] ms");
    return static_cast<int64_t>(ms * 1e6);
  }
  int64_t context_time_=0;
  uint64_t context_epoch_=0;
  double context_omega_=NAN,context_range_=NAN;
  int64_t pass_time_ = 0;
  double unwrapped_phase_ = 0, last_phase_ = 0;
  std::array<int64_t,4> fired_pass_{{INT64_MIN,INT64_MIN,INT64_MIN,INT64_MIN}};
  static double degrees(const YAML::Node & yaml, const char * key, double fallback) {
    const double value = yaml[key] ? yaml[key].as<double>() : fallback;
    if (!std::isfinite(value) || value <= 0 || value > 10)
      throw std::invalid_argument(std::string(key) + " must be finite in (0,10] degrees");
    return value * M_PI / 180.0;
  }
};
}

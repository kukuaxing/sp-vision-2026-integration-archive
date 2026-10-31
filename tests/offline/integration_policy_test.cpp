// Targeted integration checks; no devices, model inference, or shot requests.
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detection_search.hpp"
#include "tasks/auto_aim/rotation_fire_window.hpp"
#include "src/standard_settings.hpp"
#include <cassert>
#include <iostream>
#include <yaml-cpp/yaml.h>

int main() {
  using namespace auto_aim;
  // A one-second native-prediction cooldown skips visual opportunities. It
  // must not be rejected by the inactive legacy ~344 ms periodic cadence.
  auto settings_yaml = YAML::LoadFile("configs/standard.yaml");
  settings_yaml["xuc_periodic_fire_enabled"] = true;
  settings_yaml["xuc_event_phase_window_only"] = true;
  settings_yaml["xuc_event_fixed_range_m"] = 3.0;
  settings_yaml["xuc_event_fixed_omega_rad_s"] = 4.56;
  settings_yaml["xuc_periodic_every_n_faces"] = 1;
  settings_yaml["sentinel_fire_min_interval_ms"] = 1000;
  settings_yaml["xuc_high_speed_strategy"] = "teammate_prediction";
  settings_yaml.remove("xuc_high_speed_wait_for_best_frame");
  assert(StandardSettings(settings_yaml, true).xuc_high_speed_wait_for_best_frame);
  settings_yaml["xuc_high_speed_wait_for_best_frame"] = false;
  assert(!StandardSettings(settings_yaml, true).xuc_high_speed_wait_for_best_frame);
  const auto settings_allowed = [&]() {
    try { const StandardSettings settings(settings_yaml, true); return true; }
    catch (const std::invalid_argument &) { return false; }
  };
  assert(settings_allowed());
  settings_yaml["xuc_high_speed_strategy"] = "periodic";
  assert(!settings_allowed()); // active legacy schedule still needs its margin
  settings_yaml["xuc_periodic_every_n_faces"] = 3;
  assert(settings_allowed());
  settings_yaml["xuc_high_speed_strategy"] = "teammate_prediction";
  settings_yaml["sentinel_fire_min_interval_ms"] = 5000;
  assert(settings_allowed());
  settings_yaml["sentinel_fire_min_interval_ms"] = 199;
  assert(!settings_allowed());
  settings_yaml["sentinel_fire_min_interval_ms"] = 5001;
  assert(!settings_allowed());
  settings_yaml["sentinel_fire_min_interval_ms"] = 1000;
  settings_yaml["xuc_periodic_phase_offset_ms"] = 2001;
  assert(!settings_allowed()); // other existing validity guards remain active

  DetectionSearch search;
  const cv::Rect roi(420,50,600,600);
  std::vector<cv::Point2f> plate{{650,250},{720,250},{720,280},{650,280}};
  assert(search.full_frame());
  search.observe("detecting", 0, false, &plate, roi); assert(search.full_frame());
  search.observe("tracking", 0, false, &plate, roi); assert(!search.full_frame());
  search.observe("temp_lost", 1, false, nullptr, roi); assert(!search.full_frame());
  search.observe("temp_lost", 2, false, nullptr, roi); assert(!search.full_frame());
  search.observe("temp_lost", 3, false, nullptr, roi); assert(search.full_frame());
  search.observe("tracking", 0, false, &plate, roi); assert(!search.full_frame());
  plate[0].x = 425;
  search.observe("tracking", 0, false, &plate, roi); assert(search.full_frame());
  plate[0].x = 650;
  search.observe("tracking", 0, true, &plate, roi); assert(search.full_frame());
  assert(clipped_detection_roi({1000,900,-1,-1},{1440,1080}) == cv::Rect(1000,900,440,180));
  assert(clipped_detection_roi({2000,0,500,500},{1440,1080}) == cv::Rect(0,0,1440,1080));

  RotationFireWindow window;
  const auto start = std::chrono::steady_clock::time_point{};
  auto observe = [&](int frame, double error, double phase, double omega = 1.0,
                     std::uint64_t id = 1, std::uint64_t generation = 1,
                     bool valid = true, bool rebuilt = false, int face = 0) {
    window.update(valid, rebuilt, id, generation, 4, 0, 4, face, false, phase, error,
      start + std::chrono::milliseconds(frame * 20), 2, .3, .04, frame,
      phase, omega, 1.0, .35);
  };
  observe(1,.03,.20); assert(!window.two_frames_ready);
  assert(!window.allows_fire(false)); // never authorize a lone observation
  observe(2,.02,.23); assert(window.two_frames_ready && !window.current_frame_is_better);
  assert(!window.allows_fire() && window.allows_fire(false)); // early qualified frame
  observe(3,.025,.26); assert(window.current_frame_is_better);
  assert(window.allows_fire() && window.allows_fire(false));
  // Still improving, but next observed step leaves Aimer's unchanged selection edge.
  observe(4,.02,.32); assert(window.exiting_selection_next_step);
  observe(5,.02,.34,1.0,2); assert(!window.two_frames_ready); // different track
  assert(!window.allows_fire(false)); // no retained fast-path permission
  observe(6,.02,.36,1.0,2,2); assert(!window.two_frames_ready); // MCU epoch
  observe(7,.02,.38,1.0,2,2,false); assert(!window.two_frames_ready);
  observe(8,.02,.40,1.0,2,2); assert(!window.two_frames_ready);
  observe(10,.02,.42,1.0,2,2); assert(!window.two_frames_ready); // skipped observation
  assert(!window.allows_fire(false));
  observe(11,.02,.44,1.0,2,2,true,true); assert(!window.two_frames_ready);
  observe(12,.02,.46,1.0,2,2,true,false,1); assert(!window.two_frames_ready);
  window = {};
  observe(1,.03,-.20,-1); observe(2,.02,-.32,-1);
  assert(window.exiting_selection_next_step); // negative rotation symmetry

  Aimer aimer("configs/standard.yaml");
  Target target(3., 1.5, .2, .04);
  AimPredictionTrace trace;
  const auto command = aimer.aim({target}, start, 19.36, false, &trace, AimGeometry::whole_ekf);
  assert(command.control && trace.valid && trace.converged && trace.whole_ekf_geometry);
  assert(trace.face_id >= 0 && trace.iterations > 0);
  const auto config=YAML::LoadFile("configs/standard.yaml");
  assert(trace.high_speed_delay && std::abs(trace.delay_motion_rate_rad_s-1.5)<1e-9);
  assert(std::abs(trace.delay_s-config["high_speed_delay_time"].as<double>())<1e-9);
  assert(std::abs(trace.flight_time_used_s - trace.flight_time_s) < .001);
  auto predicted = target;
  predicted.predict(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(trace.prediction_ns)));
  const auto faces = predicted.whole_ekf_armor_xyza_list();
  assert((faces.at(trace.face_id) - aimer.debug_aim_point.xyza).norm() < 1e-8);
  assert(predicted.center_acceleration_variance() == 100.); // untrusted spin never stiffens center
  // A qualified high-speed whole model must retain its own phase/face geometry
  // through all flight-time iterations, in either rotation direction.
  for (double rate : {-4.33, 4.33}) {
    Target fast(3., rate, .26, .04);
    const auto fast_command = aimer.aim(
      {fast}, start, 19.36, false, &trace, AimGeometry::whole_ekf);
    assert(fast_command.control && trace.valid && trace.converged && trace.whole_ekf_geometry);
    assert(trace.high_speed_delay && std::abs(trace.delay_motion_rate_rad_s-rate)<1e-9);
    auto future_fast = fast;
    future_fast.predict(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(trace.prediction_ns)));
    const auto fast_faces = future_fast.whole_ekf_armor_xyza_list();
    assert((fast_faces.at(trace.face_id)-aimer.debug_aim_point.xyza).norm()<1e-8);
  }
  aimer.aim({}, start, 19.36, false, &trace, AimGeometry::whole_ekf);
  assert(!trace.valid && !aimer.debug_aim_point.valid);
  Target stationary(3., 0., .2, .04);
  aimer.aim({stationary}, start, 19.36, false, &trace, AimGeometry::whole_ekf);
  assert(trace.valid && !trace.whole_ekf_geometry);
  assert(!trace.high_speed_delay && std::abs(trace.delay_motion_rate_rad_s)<1e-9);
  assert(std::abs(trace.delay_s-config["low_speed_delay_time"].as<double>())<1e-9);
  std::cout << "ROI recovery, window continuity/edges, exact prediction geometry passed\n";
}

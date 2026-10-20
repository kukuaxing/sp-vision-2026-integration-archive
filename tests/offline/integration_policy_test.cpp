// Targeted integration checks; no devices, model inference, or shot requests.
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/detection_search.hpp"
#include "tasks/auto_aim/rotation_fire_window.hpp"
#include <cassert>
#include <iostream>
#include <yaml-cpp/yaml.h>

int main() {
  using namespace auto_aim;
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
  observe(2,.02,.23); assert(window.two_frames_ready && !window.current_frame_is_better);
  observe(3,.025,.26); assert(window.current_frame_is_better);
  // Still improving, but next observed step leaves Aimer's unchanged selection edge.
  observe(4,.02,.32); assert(window.exiting_selection_next_step);
  observe(5,.02,.34,1.0,2); assert(!window.two_frames_ready); // different track
  observe(6,.02,.36,1.0,2,2); assert(!window.two_frames_ready); // MCU epoch
  observe(7,.02,.38,1.0,2,2,false); assert(!window.two_frames_ready);
  observe(8,.02,.40,1.0,2,2); assert(!window.two_frames_ready);
  observe(10,.02,.42,1.0,2,2); assert(!window.two_frames_ready); // skipped observation
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
  aimer.aim({}, start, 19.36, false, &trace, AimGeometry::whole_ekf);
  assert(!trace.valid && !aimer.debug_aim_point.valid);
  Target stationary(3., 0., .2, .04);
  aimer.aim({stationary}, start, 19.36, false, &trace, AimGeometry::whole_ekf);
  assert(trace.valid && !trace.whole_ekf_geometry);
  assert(!trace.high_speed_delay && std::abs(trace.delay_motion_rate_rad_s)<1e-9);
  assert(std::abs(trace.delay_s-config["low_speed_delay_time"].as<double>())<1e-9);
  std::cout << "ROI recovery, window continuity/edges, exact prediction geometry passed\n";
}

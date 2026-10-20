#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace auto_aim {
// Observation history only. Successful writes remain the only event commitment.
struct RotationFireWindow
{
  std::uint64_t track_id = 0, generation = 0;
  int frames = 0;
  int target_name = -1, target_type = -1, armor_count = 0, face_id = -1;
  bool event_face = false;
  double phase_rad = NAN, error_rad = NAN, next_error_rad = NAN;
  double selection_phase_rad = NAN, selection_omega_rad_s = NAN;
  uint64_t frame_sequence = 0;
  std::chrono::steady_clock::time_point timestamp{};
  bool two_frames_ready = false, current_frame_is_better = false;
  bool exiting_selection_next_step = false;

  void update(
    bool valid, bool rebuilt, std::uint64_t track, std::uint64_t boot_generation,
    int name, int type, int count, int face,
    bool uses_event_face, double phase, double error,
    std::chrono::steady_clock::time_point time, int required_frames,
    double max_phase_step, double tolerance, uint64_t sequence,
    double selection_phase, double omega, double coming, double leaving)
  {
    two_frames_ready = false;
    current_frame_is_better = false;
    exiting_selection_next_step = false;
    next_error_rad = NAN;
    if (!valid || face < 0 || !std::isfinite(phase) || !std::isfinite(error)) {
      frames = 0;
      return;
    }
    const bool continuous = frames > 0 && !rebuilt && time > timestamp &&
      time - timestamp <= std::chrono::milliseconds(100) &&
      track == track_id && boot_generation == generation && sequence == frame_sequence + 1 &&
      name == target_name && type == target_type && count == armor_count &&
      face == face_id && uses_event_face == event_face &&
      std::abs(std::remainder(phase - phase_rad, 2.0 * M_PI)) <= max_phase_step;
    const int needed = std::max(2, required_frames);
    frames = continuous ? std::min(frames + 1, needed) : 1;
    if (continuous) {
      // One observed frame step forward; no future sample or fitted model.
      next_error_rad = std::remainder(
        error + std::remainder(error - error_rad, 2.0 * M_PI), 2.0 * M_PI);
      current_frame_is_better = std::abs(error) <= std::abs(next_error_rad) ||
        std::abs(next_error_rad) > tolerance;
      if (!uses_event_face && count == 4 && sequence == frame_sequence + 1 &&
          std::isfinite(selection_phase) && std::isfinite(selection_phase_rad) &&
          std::isfinite(omega) && std::isfinite(selection_omega_rad_s) &&
          omega * selection_omega_rad_s > 0.0) {
        // Match Aimer's unchanged selection edges. Use only the current and
        // preceding observed step; the current frame must still pass every gate.
        const double step = std::remainder(
          selection_phase - selection_phase_rad, 2.0 * M_PI);
        exiting_selection_next_step = omega > 0.0
          ? selection_phase >= -coming && selection_phase < leaving &&
              step > 0.0 && selection_phase + step >= leaving
          : selection_phase > -leaving && selection_phase <= coming &&
              step < 0.0 && selection_phase + step <= -leaving;
      }
    }
    two_frames_ready = frames >= needed;
    track_id = track; generation = boot_generation;
    target_name = name;
    target_type = type;
    armor_count = count;
    face_id = face;
    event_face = uses_event_face;
    phase_rad = phase;
    error_rad = error;
    selection_phase_rad = selection_phase;
    selection_omega_rad_s = omega;
    frame_sequence = sequence;
    timestamp = time;
  }
};

}  // namespace auto_aim

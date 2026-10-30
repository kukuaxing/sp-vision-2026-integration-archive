#ifndef AUTO_AIM__GYRO_FACE_SELECTOR_HPP
#define AUTO_AIM__GYRO_FACE_SELECTOR_HPP

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace auto_aim
{

// Commits one physical armor id to a predicted face-center event.  Repeated
// frames solving the same hit cycle keep that id even if two faces momentarily
// have nearly equal scores; a handover is allowed only when the event cycle
// advances or the committed face is no longer geometrically plausible.
class GyroFaceSelector
{
public:
  struct Candidate
  {
    int face_id = -1;
    double normal_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  };

  struct Selection
  {
    bool valid = false;
    bool changed = false;
    int face_id = -1;
    std::int64_t hit_cycle = -1;
    double alignment_rad = std::numeric_limits<double>::infinity();
  };

  // Visualization attribution only. Preserve the complete body phase here:
  // folding modulo pi/2 identifies an angle, but loses which of four faces
  // is expected to occupy that angle at the planned impact time.
  static Selection face_at_hit_time(
    double body_phase_rad, double center_yaw_rad, double signed_omega_rad_s,
    double capture_to_hit_s)
  {
    Selection out;
    if (!std::isfinite(body_phase_rad) || !std::isfinite(center_yaw_rad) ||
        !std::isfinite(signed_omega_rad_s) || !std::isfinite(capture_to_hit_s) ||
        capture_to_hit_s < 0.0) return out;
    const double impact_phase = body_phase_rad + signed_omega_rad_s * capture_to_hit_s;
    if (!std::isfinite(impact_phase)) return out;
    for (int face = 0; face < 4; ++face) {
      const double alignment = std::abs(std::remainder(
        impact_phase + face * M_PI / 2.0 - center_yaw_rad, 2.0 * M_PI));
      if (alignment < out.alignment_rad) {
        out.valid = true;
        out.face_id = face;
        out.alignment_rad = alignment;
      }
    }
    return out;
  }

  explicit GyroFaceSelector(double max_alignment_rad)
  : max_alignment_rad_(std::max(0.0, max_alignment_rad))
  {
  }

  void reset()
  {
    committed_cycle_ = -1;
    committed_face_id_ = -1;
  }

  Selection select(
    const std::vector<Candidate> & candidates, double center_yaw_rad,
    std::int64_t hit_cycle)
  {
    Selection out;
    out.hit_cycle = hit_cycle;
    if (!std::isfinite(center_yaw_rad) || hit_cycle < 0 || candidates.empty()) {
      return out;
    }

    const Candidate * best = nullptr;
    double best_alignment = std::numeric_limits<double>::infinity();
    const Candidate * committed = nullptr;
    double committed_alignment = std::numeric_limits<double>::infinity();
    for (const auto & candidate : candidates) {
      if (candidate.face_id < 0 || !std::isfinite(candidate.normal_yaw_rad)) continue;
      const double alignment = std::abs(std::remainder(
        candidate.normal_yaw_rad - center_yaw_rad, 2.0 * M_PI));
      if (alignment < best_alignment) {
        best = &candidate;
        best_alignment = alignment;
      }
      if (candidate.face_id == committed_face_id_) {
        committed = &candidate;
        committed_alignment = alignment;
      }
    }

    const bool same_cycle_commit_valid = hit_cycle == committed_cycle_ &&
      committed != nullptr && committed_alignment <= max_alignment_rad_;
    if (same_cycle_commit_valid) {
      out.valid = true;
      out.face_id = committed_face_id_;
      out.alignment_rad = committed_alignment;
      return out;
    }
    if (best == nullptr || best_alignment > max_alignment_rad_) return out;

    const int previous_face_id = committed_face_id_;
    const std::int64_t previous_cycle = committed_cycle_;
    committed_cycle_ = hit_cycle;
    committed_face_id_ = best->face_id;
    out.valid = true;
    out.changed = previous_face_id >= 0 &&
      (previous_face_id != committed_face_id_ || previous_cycle != committed_cycle_);
    out.face_id = committed_face_id_;
    out.alignment_rad = best_alignment;
    return out;
  }

private:
  double max_alignment_rad_;
  std::int64_t committed_cycle_ = -1;
  int committed_face_id_ = -1;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__GYRO_FACE_SELECTOR_HPP

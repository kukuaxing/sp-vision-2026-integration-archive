#ifndef AUTO_AIM__DIRECT_MEASUREMENT_HPP
#define AUTO_AIM__DIRECT_MEASUREMENT_HPP

#include <cmath>
#include <limits>
#include <list>
#include <optional>

#include "armor.hpp"

namespace auto_aim
{
// A short measurement gap may retain existing authority, but cannot create it.
// Every genuine target loss requires a fresh converged measurement to re-arm.
class DirectControlGate
{
public:
  void reset() { confirmed_ = false; }
  bool confirmed() const { return confirmed_; }

  bool update(
    bool target_present, bool measurement_valid, bool tracker_converged, bool within_hold)
  {
    if (!target_present || (!measurement_valid && !within_hold)) reset();
    else if (measurement_valid && tracker_converged) confirmed_ = true;
    return confirmed_ && (measurement_valid || within_hold);
  }

private:
  bool confirmed_ = false;
};

// Detector candidates can remain unsolved when the tracker follows another
// class. Never read their coordinates or silently switch the control target.
inline const Armor * select_direct_measurement(
  const std::list<Armor> & armors, int associated_detection_index,
  bool prefer_association, std::optional<double> previous_yaw,
  ArmorName tracked_name = ArmorName::not_armor,
  ArmorType tracked_type = ArmorType::small)
{
  const auto usable = [tracked_name, tracked_type](const Armor & armor) {
    return armor.pose_valid && armor.xyz_in_gimbal.allFinite() &&
      armor.xyz_in_world.allFinite() &&
      (tracked_name == ArmorName::not_armor ||
       (armor.name == tracked_name && armor.type == tracked_type));
  };

  const Armor * first_valid = nullptr;
  const Armor * nearest = nullptr;
  double nearest_distance = std::numeric_limits<double>::infinity();
  const bool have_previous_yaw = previous_yaw && std::isfinite(*previous_yaw);
  int detection_index = 0;
  for (const auto & armor : armors) {
    const int current_index = detection_index++;
    if (!usable(armor)) continue;
    if (prefer_association && current_index == associated_detection_index) return &armor;
    if (first_valid == nullptr) first_valid = &armor;
    if (have_previous_yaw) {
      const double bearing = std::atan2(armor.xyz_in_world.y(), armor.xyz_in_world.x());
      const double distance = std::abs(std::remainder(bearing - *previous_yaw, 2.0 * M_PI));
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = &armor;
      }
    }
  }
  return nearest != nullptr ? nearest : first_valid;
}
}  // namespace auto_aim

#endif  // AUTO_AIM__DIRECT_MEASUREMENT_HPP

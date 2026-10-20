#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <cstdint>
#include <list>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tasks/omniperception/detection.hpp"

// ROS2 前向声明（避免强制依赖）
#ifdef AMENT_CMAKE_FOUND
namespace rclcpp {
class Node;
class Time;
}
#endif

namespace auto_aim
{
class Tracker
{
public:
  struct ArmorAssociation
  {
    int detection_index = -1;
    int face_id = -1;
    double cost = 0.0;
  };

  struct FrameDiagnostics {
    bool rebuilt = false, batch_update = false;
    int invalid_pose = 0, identity_rejected = 0, position_rejected = 0;
    int cost_rejected = 0, candidates = 0, size_correction_rejected = 0;
    double best_cost = std::numeric_limits<double>::infinity();
    double center_error = 0, corner_error = 0, angle_error = 0, range_error = 0;
  };
  const FrameDiagnostics & diagnostics() const { return diagnostics_; }
  int temp_lost_count() const { return temp_lost_count_; }
  const Armor * primary_observation() const {
    return measurement_updated() && primary_observation_ ? &*primary_observation_ : nullptr;
  }

  Tracker(const std::string & config_path, Solver & solver);

  std::string state() const;
  void reset();
  bool set_enemy_color(Color color);
  const std::vector<ArmorAssociation> & last_associations() const;
  int primary_detection_index() const;
  int primary_face_id() const;
  // ID belongs to a geometrically continuous target, not a detector class.
  std::uint64_t track_id() const;
  bool measurement_updated() const;
  bool identity_confirmed() const;

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

#ifdef AMENT_CMAKE_FOUND
  // 设置 ROS2 节点用于发布 marker（可选）
  void set_ros2_node(std::shared_ptr<rclcpp::Node> node);

  // 发布当前跟踪目标的 marker 到 rviz2
  // timestamp: 可选的时间戳，如果不提供则使用ros_node_->now()
  void publish_markers(
    const std::list<Target> & targets,
    const rclcpp::Time & timestamp,
    const Eigen::Vector4d & aim_xyza = Eigen::Vector4d::Zero(),
    bool has_aim_point = false);
#endif

private:
  struct AssociationCandidate
  {
    Armor * armor = nullptr;
    Armor measurement;
    ArmorName observed_name = ArmorName::not_armor;
    ArmorType observed_type = ArmorType::small;
    int detection_index = -1;
    int face_id = -1;
    double cost = std::numeric_limits<double>::infinity();
    double alignment_rad = std::numeric_limits<double>::infinity();
  };
  std::vector<AssociationCandidate> collect_association_candidates(
    std::list<Armor> & armors, const std::vector<Eigen::Vector4d> & predicted_faces,
    int & detection_index);
  std::vector<AssociationCandidate> select_associations(
    const std::vector<AssociationCandidate> & candidates, int detection_index,
    std::size_t face_count) const;
  bool has_rigid_pair(const std::vector<AssociationCandidate> & selected) const;

  Solver & solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  double association_max_cost_;
  int batch_max_observations_;
  double batch_min_omega_rad_s_;
  int target_switch_min_frames_ = 3;
  ArmorName switch_candidate_name_ = ArmorName::not_armor;
  int switch_candidate_frames_ = 0;
  YawESKF::Config yaw_eskf_config_;
  Target::IteratedUpdateConfig iterated_update_config_;
  Target::CenterProcessConfig center_process_config_;
  FrameDiagnostics diagnostics_;
  std::optional<Armor> primary_observation_;
  std::vector<ArmorAssociation> last_associations_;
  int primary_detection_index_;
  int primary_face_id_;
  std::uint64_t next_track_id_ = 1;
  std::uint64_t track_id_ = 0;
  bool measurement_updated_ = false;
  bool identity_confirmed_ = false;
  int identity_confirm_frames_ = 5;
  double identity_min_confidence_ = 0.1;
  ArmorName identity_candidate_name_ = ArmorName::not_armor;
  ArmorType identity_candidate_type_ = ArmorType::small;
  int identity_candidate_frames_ = 0;

#ifdef AMENT_CMAKE_FOUND
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<void> marker_pub_;  // 使用 void* 类型擦除，避免暴露 ROS2 具体类型
#endif

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
  void initialize_target(const Armor & armor, std::chrono::steady_clock::time_point t);
  bool update_identity(
    Armor & measurement, ArmorName observed_name, ArmorType observed_type,
    std::chrono::steady_clock::time_point t);
  Armor * select_target(std::list<Armor> & armors);
  bool should_switch_target(const std::list<Armor> & armors);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP

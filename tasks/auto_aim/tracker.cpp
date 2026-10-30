#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// ROS2 headers (仅在 ROS2 可用时编译)
#ifdef AMENT_CMAKE_FOUND
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#endif

namespace auto_aim
{
void Tracker::reset()
{
  diagnostics_ = {};
  primary_observation_.reset();
  state_ = pre_state_ = "lost";
  detect_count_ = temp_lost_count_ = switch_candidate_frames_ = 0;
  switch_candidate_name_ = ArmorName::not_armor;
  primary_detection_index_ = primary_face_id_ = -1;
  last_associations_.clear();
  measurement_updated_ = identity_confirmed_ = false;
  identity_candidate_name_ = ArmorName::not_armor;
  identity_candidate_frames_ = 0;
  rate_mismatch_since_.reset();
  rate_mismatch_samples_ = 0;
  last_timestamp_ = std::chrono::steady_clock::now();
}

bool Tracker::set_enemy_color(Color color)
{
  if (enemy_color_ == color) return false;
  enemy_color_ = color;
  reset();
  return true;
}

Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth},
  association_max_cost_(2.0),
  batch_max_observations_(4),
  batch_min_omega_rad_s_(3.0),
  primary_detection_index_(-1),
  primary_face_id_(-1)
{
  auto yaml = YAML::LoadFile(config_path);
  if (yaml["center_process_adaptive_enabled"])
    center_process_config_.adaptive_enabled = yaml["center_process_adaptive_enabled"].as<bool>();
  if (yaml["center_process_acceleration_variance"])
    center_process_config_.acceleration_variance = std::clamp(
      yaml["center_process_acceleration_variance"].as<double>(), 1.0, 1000.0);
  if (yaml["center_process_rotation_acceleration_variance"])
    center_process_config_.rotation_acceleration_variance = std::clamp(
      yaml["center_process_rotation_acceleration_variance"].as<double>(), 1.0,
      center_process_config_.acceleration_variance);
  if (yaml["center_process_rotation_rate_rad_s"])
    center_process_config_.rotation_rate_rad_s = std::clamp(
      yaml["center_process_rotation_rate_rad_s"].as<double>(), 0.1, 10.0);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = std::max(1, yaml["min_detect_count"].as<int>());
  identity_confirm_frames_ = std::max(3, min_detect_count_);
  if (yaml["min_confidence"])
    identity_min_confidence_ = std::clamp(yaml["min_confidence"].as<double>(), 0.1, 1.0);
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
  if (yaml["tracker_association_max_cost"]) {
    association_max_cost_ = std::clamp(
      yaml["tracker_association_max_cost"].as<double>(), 0.05, 20.0);
  }
  if (yaml["tracker_batch_max_observations"]) {
    batch_max_observations_ = std::clamp(
      yaml["tracker_batch_max_observations"].as<int>(), 1, 4);
  }
  if (yaml["tracker_batch_min_omega_rad_s"]) {
    batch_min_omega_rad_s_ = std::clamp(
      yaml["tracker_batch_min_omega_rad_s"].as<double>(), 0.0, 20.0);
  }
  if (yaml["tracker_target_switch_min_frames"]) {
    target_switch_min_frames_ = std::clamp(
      yaml["tracker_target_switch_min_frames"].as<int>(), 1, 30);
  }
  if (yaml["yaw_eskf_angular_jerk_variance"]) {
    yaw_eskf_config_.angular_jerk_variance = std::clamp(
      yaml["yaw_eskf_angular_jerk_variance"].as<double>(), 0.1, 5000.0);
  }
  if (yaml["yaw_eskf_measurement_floor_deg"]) {
    const double sigma_rad = std::clamp(
      yaml["yaw_eskf_measurement_floor_deg"].as<double>(), 0.1, 20.0) *
      M_PI / 180.0;
    yaw_eskf_config_.measurement_variance_floor = sigma_rad * sigma_rad;
  }
  if (yaml["yaw_eskf_nis_reject_threshold"]) {
    yaw_eskf_config_.nis_reject_threshold = std::clamp(
      yaml["yaw_eskf_nis_reject_threshold"].as<double>(), 1.0, 100.0);
  }
  if (yaml["yaw_eskf_min_trusted_updates"]) {
    yaw_eskf_config_.min_trusted_updates = std::clamp(
      yaml["yaw_eskf_min_trusted_updates"].as<int>(), 2, 100);
  }
  if (yaml["yaw_eskf_trusted_rate_std_rad_s"]) {
    const double stddev = std::clamp(
      yaml["yaw_eskf_trusted_rate_std_rad_s"].as<double>(), 0.1, 10.0);
    yaw_eskf_config_.trusted_rate_variance = stddev * stddev;
  }
  if (yaml["yaw_eskf_max_trusted_abs_rate_rad_s"]) {
    yaw_eskf_config_.max_trusted_abs_rate_rad_s = std::clamp(
      yaml["yaw_eskf_max_trusted_abs_rate_rad_s"].as<double>(), 0.5, 30.0);
  }
  if (yaml["yaw_eskf_max_trusted_abs_acceleration_rad_s2"]) {
    yaw_eskf_config_.max_trusted_abs_acceleration_rad_s2 = std::clamp(
      yaml["yaw_eskf_max_trusted_abs_acceleration_rad_s2"].as<double>(),
      1.0, 500.0);
  }
  if (yaml["whole_state_iekf_enabled"])
    iterated_update_config_.enabled = yaml["whole_state_iekf_enabled"].as<bool>();
  if (yaml["whole_state_iekf_max_iterations"])
    iterated_update_config_.max_iterations = std::clamp(
      yaml["whole_state_iekf_max_iterations"].as<int>(), 1, 10);
  if (yaml["whole_state_iekf_stop_threshold"])
    iterated_update_config_.stop_threshold = std::clamp(
      yaml["whole_state_iekf_stop_threshold"].as<double>(), 1e-8, 0.1);
  if (yaml["whole_state_iekf_huber_threshold"])
    iterated_update_config_.huber_threshold = std::clamp(
      yaml["whole_state_iekf_huber_threshold"].as<double>(), 0.5, 10.0);
  // Only the explicit fixed-speed Sentinel bench provides a speed reference.
  // A rejected double-rate branch must reacquire instead of remaining alive
  // indefinitely on alternating faces. Ordinary match tracking is unchanged.
  if (yaml["board_protocol"] && yaml["board_protocol"].as<std::string>() == "SENTINEL" &&
      yaml["xuc_event_phase_window_only"] && yaml["xuc_event_phase_window_only"].as<bool>() &&
      yaml["xuc_event_fixed_range_m"] && yaml["xuc_event_fixed_range_m"].as<double>() > 0.0 &&
      yaml["xuc_event_fixed_omega_rad_s"])
    bench_reference_rate_ = std::clamp(
      yaml["xuc_event_fixed_omega_rad_s"].as<double>(), 0.0, 8.0);
  if (yaml["xuc_rotation_rate_disagreement_rad_s"])
    bench_rate_tolerance_ = std::clamp(
      yaml["xuc_rotation_rate_disagreement_rad_s"].as<double>(), 0.1, 10.0);
}

bool Tracker::recover_bench_rate_alias(std::chrono::steady_clock::time_point t, bool updated)
{
  const auto & x = target_.ekf().x;
  const bool excessive = bench_reference_rate_ > 0.0 && state_ != "lost" &&
    target_.armor_num() == 4 && x.size() == 11 && std::isfinite(x[7]) &&
    !(target_.whole_rotation_rejection_mask() & Target::immature) &&
    std::abs(x[7]) > bench_reference_rate_ + bench_rate_tolerance_;
  if (!excessive) {
    rate_mismatch_since_.reset();
    rate_mismatch_samples_ = 0;
    return false;
  }
  if (!updated) {
    if (rate_mismatch_since_ && tools::delta_time(t, rate_mismatch_last_sample_) > 0.1) {
      rate_mismatch_since_.reset();
      rate_mismatch_samples_ = 0;
    }
    return false;
  }
  if (!rate_mismatch_since_ || tools::delta_time(t, rate_mismatch_last_sample_) > 0.1) {
    rate_mismatch_since_ = t;
    rate_mismatch_samples_ = 0;
  }
  rate_mismatch_last_sample_ = t;
  ++rate_mismatch_samples_;
  const double duration = tools::delta_time(t, *rate_mismatch_since_);
  if (duration < 0.5 || rate_mismatch_samples_ < 10) return false;
  tools::logger()->warn(
    "[Tracker][RATE_ALIAS] track={} omega={:+.3f} reference={:.3f} "
    "duration_ms={:.0f} samples={}; discard model and reacquire",
    track_id_, x[7], bench_reference_rate_, duration * 1000.0, rate_mismatch_samples_);
  reset();
  diagnostics_.rate_alias_reset = true;
  return true;
}

std::string Tracker::state() const { return state_; }

const std::vector<Tracker::ArmorAssociation> & Tracker::last_associations() const
{
  return last_associations_;
}

int Tracker::primary_detection_index() const { return primary_detection_index_; }

int Tracker::primary_face_id() const { return primary_face_id_; }

std::uint64_t Tracker::track_id() const { return state_ == "lost" ? 0 : track_id_; }

bool Tracker::measurement_updated() const { return state_ != "lost" && measurement_updated_; }

bool Tracker::identity_confirmed() const { return state_ != "lost" && identity_confirmed_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  measurement_updated_ = false;
  primary_observation_.reset();
  diagnostics_ = {};
  auto dt = tools::delta_time(t, last_timestamp_);
  if (state_ != "lost" && dt <= 0.0) {
    // Replayed/out-of-order images must not rewind filters or claim a fresh
    // association. Leave the established state available only as prediction.
    primary_detection_index_ = primary_face_id_ = -1;
    last_associations_.clear();
    for (auto & armor : armors) armor.invalidate_pose();
    return {target_};
  }
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  if (use_enemy_color)
    armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });
  for (auto & armor : armors) armor.invalidate_pose();

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  } else if (state_ == "tracking" && should_switch_target(armors)) {
    found = set_target(armors, t);
    tools::logger()->info(
      "[Tracker] stable target switch to {} after {} frames",
      ARMOR_NAMES[target_.name], target_switch_min_frames_);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  if (state_ != "lost" && recover_bench_rate_alias(t, found)) return {};

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {};
  }
  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    // Print detailed EKF diagnostics to help debug convergence issues
    try {
      auto & ekf = target_.ekf();
      double recent_rate = ekf.data.at("recent_nis_failures");
      double nis = ekf.data.at("nis");
      double nees = ekf.data.at("nees");
      double ry = ekf.data.at("residual_yaw");
      double rp = ekf.data.at("residual_pitch");
      double rd = ekf.data.at("residual_distance");
      double ra = ekf.data.at("residual_angle");

      tools::logger()->debug(
        "[Target] Bad Converge Found! recent_rate={:.3f} last_nis={:.3f} nees={:.3f} resid=(yaw={:.3f},pitch={:.3f},dist={:.3f},ang={:.3f})",
        recent_rate, nis, nees, ry, rp, rd, ra);
    } catch (const std::exception & e) {
      tools::logger()->debug("[Target] Bad Converge Found! (failed to read EKF data: {})", e.what());
    }

    state_ = "lost";
    return {};
  }

  if (state_ == "lost") {
    measurement_updated_ = false;
    primary_observation_.reset();
    return {};
  }

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  measurement_updated_ = false;
  primary_observation_.reset();
  diagnostics_ = {};
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }
  if (use_enemy_color)
    armors.remove_if([&](const Armor & armor) { return armor.color != enemy_color_; });
  for (auto & armor : armors) armor.invalidate_pose();

  auto dt = tools::delta_time(t, last_timestamp_);
  if (state_ != "lost" && dt <= 0.0) {
    primary_detection_index_ = primary_face_id_ = -1;
    last_associations_.clear();
    return {switch_target, {target_}};
  }
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  if (state_ != "lost" && recover_bench_rate_alias(t, found)) return {switch_target, {}};

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  last_associations_.clear();
  primary_detection_index_ = -1;
  primary_face_id_ = -1;
  if (armors.empty()) return false;

  Armor * selected = select_target(armors);
  if (selected == nullptr) return false;
  auto & armor = *selected;
  int selected_detection_index = 0;
  for (auto it = armors.begin(); it != armors.end() && &*it != selected; ++it)
    ++selected_detection_index;
  switch_candidate_name_ = ArmorName::not_armor;
  switch_candidate_frames_ = 0;

  initialize_target(armor, t);
  track_id_ = next_track_id_++;
  identity_confirmed_ = false;
  identity_candidate_name_ = armor.name;
  identity_candidate_type_ = armor.type;
  identity_candidate_frames_ = armor.confidence >= identity_min_confidence_ ? 1 : 0;
  measurement_updated_ = true;
  primary_observation_ = armor;

  last_associations_.push_back({selected_detection_index, 0, 0.0});
  primary_detection_index_ = selected_detection_index;
  primary_face_id_ = 0;

  return true;
}

void Tracker::initialize_target(const Armor & armor, std::chrono::steady_clock::time_point t)
{
  rate_mismatch_since_.reset();
  rate_mismatch_samples_ = 0;
  diagnostics_.rebuilt = true;
  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 0.0064, 0, 0}};
    target_ = Target(
      armor, t, 0.2, 2, P0_dig, yaw_eskf_config_, iterated_update_config_, center_process_config_);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(
      armor, t, 0.2765, 3, P0_dig, yaw_eskf_config_, iterated_update_config_, center_process_config_);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(
      armor, t, 0.3205, 3, P0_dig, yaw_eskf_config_, iterated_update_config_, center_process_config_);
  }

  else {
    // A single stationary face cannot separate center from radius. A one-
    // metre standard deviation let pixel/PnP yaw noise explain itself as an
    // impossible vehicle radius and repeatedly reset the tracker. Use a broad
    // physical prior (8 cm radius/axis difference, 10 cm height), keeping both
    // radii free to converge once motion/multiple faces make them observable.
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 0.0064, 0.0064, 0.01}};
    target_ = Target(
      armor, t, 0.2, 4, P0_dig, yaw_eskf_config_, iterated_update_config_, center_process_config_);
  }

}

bool Tracker::update_identity(
  Armor & measurement, ArmorName observed_name, ArmorType observed_type,
  std::chrono::steady_clock::time_point t)
{
  if (measurement.confidence < identity_min_confidence_) {
    identity_candidate_frames_ = 0;
    return false;
  }
  std::optional<Armor> corrected_size;
  if (observed_type != target_.armor_type) {
    // A physical plate cannot change width as it rotates. Consecutive class
    // logits alone do not justify discarding its established kinematics.
    // Compare BOTH size hypotheses on these same current corners, and count a
    // correction vote only when the proposed size explains their geometry better.
    Armor trial = measurement;
    trial.name = observed_name;
    trial.type = observed_type;
    solver_.solve(trial);
    const bool geometry_improved = trial.pose_valid &&
      std::isfinite(trial.pose_quality) && std::isfinite(measurement.pose_quality) &&
      trial.pose_quality >= 0.2 && trial.pose_quality > 1.25 * measurement.pose_quality &&
      std::isfinite(trial.model_reprojection_error_px) &&
      std::isfinite(measurement.model_reprojection_error_px) &&
      trial.model_reprojection_error_px < 0.8 * measurement.model_reprojection_error_px;
    if (!geometry_improved) {
      ++diagnostics_.size_correction_rejected;
      identity_candidate_frames_ = 0;
      return false;
    }
    corrected_size = std::move(trial);
  }
  if (observed_name != identity_candidate_name_ || observed_type != identity_candidate_type_) {
    identity_candidate_name_ = observed_name;
    identity_candidate_type_ = observed_type;
    identity_candidate_frames_ = 1;
  } else {
    identity_candidate_frames_ = std::min(identity_confirm_frames_, identity_candidate_frames_ + 1);
  }
  if (identity_candidate_frames_ < identity_confirm_frames_) return false;
  if (observed_name == target_.name && observed_type == target_.armor_type) {
    identity_confirmed_ = true;
    return false;
  }

  // A consistent class change at this *same geometric track* may correct the
  // first-frame class/size. Re-solve the proposed model before committing it;
  // never feed an old-size position into a new-size vehicle state.
  Armor corrected = corrected_size ? *corrected_size : measurement;
  if (!corrected_size) {
    corrected.name = observed_name;
    corrected.type = observed_type;
    solver_.solve(corrected);
  }
  if (!corrected.pose_valid) return false;
  measurement = std::move(corrected);
  initialize_target(measurement, t);
  identity_confirmed_ = true;
  state_ = "detecting";
  detect_count_ = 0;
  return true;
}

Armor * Tracker::select_target(std::list<Armor> & armors)
{
  Armor * best = nullptr;
  double best_score = std::numeric_limits<double>::infinity();
  for (auto & armor : armors) {
    solver_.solve(armor);
    if (!armor.pose_valid) { ++diagnostics_.invalid_pose; continue; }
    const double center_score = std::hypot(
      armor.center_norm.x - 0.5, armor.center_norm.y - 0.5);
    const double range_score = armor.xyz_in_world.norm() / 10.0;
    // Priority remains dominant.  Distance and image-center position break
    // ties, matching SHtech's fixed-target selection without target chatter.
    const double score = 10.0 * static_cast<int>(armor.priority) +
      0.65 * center_score + 0.35 * range_score -
      0.20 * std::clamp(armor.confidence, 0.0, 1.0);
    if (score < best_score) {
      best_score = score;
      best = &armor;
    }
  }
  return best;
}

bool Tracker::should_switch_target(const std::list<Armor> & armors)
{
  const auto candidate = std::find_if(
    armors.begin(), armors.end(), [this](const Armor & armor) {
      return armor.priority < target_.priority && armor.name != target_.name;
    });
  if (candidate == armors.end()) {
    switch_candidate_name_ = ArmorName::not_armor;
    switch_candidate_frames_ = 0;
    return false;
  }
  if (candidate->name != switch_candidate_name_) {
    switch_candidate_name_ = candidate->name;
    switch_candidate_frames_ = 1;
  } else {
    ++switch_candidate_frames_;
  }
  return switch_candidate_frames_ >= target_switch_min_frames_;
}

std::vector<Tracker::AssociationCandidate> Tracker::collect_association_candidates(
  std::list<Armor> & armors, const std::vector<Eigen::Vector4d> & predicted_faces,
  int & detection_index)
{
  std::vector<AssociationCandidate> candidates;
  detection_index = 0;
  for (auto & armor : armors) {
    const int current_detection_index = detection_index++;
    if (armor.name == ArmorName::not_armor) continue;
    const bool identity_matches =
      armor.name == target_.name && armor.type == target_.armor_type;
    Armor measurement = armor;
    // Class logits are observations, not a persistent object ID. A matching
    // geometric track retains its confirmed physical size through a transient
    // digit/size misclassification. The stricter gate below prevents nearby
    // unrelated detections from being relabelled into this vehicle.
    measurement.name = target_.name;
    measurement.type = target_.armor_type;
    solver_.solve(measurement);
    if (!measurement.pose_valid) { ++diagnostics_.invalid_pose; continue; }
    const double observed_range_m = std::max(0.05, measurement.ypd_in_world[2]);
    const double detection_scale_px = std::max(
      30.0, std::sqrt(std::max(1.0, static_cast<double>(armor.box.area()))));
    cv::Point2f observed_center = armor.center;
    if (!armor.points.empty()) {
      observed_center = cv::Point2f(0.0F, 0.0F);
      for (const auto & point : armor.points) observed_center += point;
      observed_center *= 1.0F / static_cast<float>(armor.points.size());
    }

    for (std::size_t id = 0; id < predicted_faces.size(); ++id) {
      const auto & face = predicted_faces[id];
      const auto projected = solver_.reproject_armor(
        face.head(3), face[3], target_.armor_type, target_.name);
      if (projected.empty()) continue;

      cv::Point2f projected_center(0.0F, 0.0F);
      for (const auto & point : projected) projected_center += point;
      projected_center *= 1.0F / static_cast<float>(projected.size());

      const double center_error =
        cv::norm(observed_center - projected_center) / detection_scale_px;
      double corner_squared_error = 0.0;
      if (projected.size() != measurement.points.size()) continue;
      for (std::size_t corner = 0; corner < projected.size(); ++corner) {
        const double error = cv::norm(measurement.points[corner] - projected[corner]);
        corner_squared_error += error * error;
      }
      const double corner_error =
        std::sqrt(corner_squared_error / projected.size()) / detection_scale_px;
      // Require actual image continuity before tolerating different labels.
      // A class mismatch must not use the broad normal association envelope.
      if (!identity_matches && (center_error > 0.55 || corner_error > 0.65)) {
        ++diagnostics_.identity_rejected; continue;
      }
      const double angle_error = std::abs(tools::limit_rad(
        measurement.ypr_in_world[0] - face[3])) / (20.0 * CV_PI / 180.0);
      const double predicted_range_m = std::max(
        0.05, tools::xyz2ypd(face.head(3))[2]);
      const double range_error =
        std::abs(std::log(observed_range_m / predicted_range_m)) / 0.25;
      const double position_error_m = (measurement.xyz_in_world - face.head(3)).norm();
      if (position_error_m > std::max(0.18, 0.12 * predicted_range_m)) {
        ++diagnostics_.position_rejected; continue;
      }
      const double confidence_bonus = 0.10 * std::clamp(armor.confidence, 0.0, 1.0);
      const double cost = center_error * center_error +
        0.20 * angle_error * angle_error +
        0.20 * range_error * range_error - confidence_bonus;
      if (std::isfinite(cost) && cost < diagnostics_.best_cost) {
        diagnostics_.best_cost = cost;
        if (association_diagnostics_enabled_)
          diagnostics_.association_innovation = target_.prior_innovation(measurement, static_cast<int>(id));
        diagnostics_.center_error = center_error; diagnostics_.corner_error = corner_error;
        diagnostics_.angle_error = angle_error; diagnostics_.range_error = range_error;
      }
      if (!std::isfinite(cost) || cost > association_max_cost_) ++diagnostics_.cost_rejected;
      if (std::isfinite(cost) && cost <= association_max_cost_) {
        ++diagnostics_.candidates;
        const auto state = target_.ekf_x();
        const double center_yaw = std::atan2(state[2], state[0]);
        candidates.push_back({
          &armor, measurement, armor.name, armor.type,
          current_detection_index, static_cast<int>(id), cost,
          std::abs(tools::limit_rad(face[3] - center_yaw))});
      }
    }
  }

  return candidates;
}

std::vector<Tracker::AssociationCandidate> Tracker::select_associations(
  const std::vector<AssociationCandidate> & candidates, int detection_index,
  std::size_t face_count) const
{
  std::vector<bool> detection_used(static_cast<std::size_t>(detection_index), false);
  std::vector<AssociationCandidate> current_assignment;
  std::vector<AssociationCandidate> selected;
  double best_assignment_cost = std::numeric_limits<double>::infinity();
  std::function<void(int, double)> assign_face = [&](int face_id, double total_cost) {
    if (face_id >= static_cast<int>(face_count) ||
        static_cast<int>(current_assignment.size()) >= batch_max_observations_) {
      if (current_assignment.size() > selected.size() ||
          (current_assignment.size() == selected.size() &&
           total_cost < best_assignment_cost)) {
        selected = current_assignment;
        best_assignment_cost = total_cost;
      }
      return;
    }

    // A face can be out of view; skipping it is a valid branch.
    assign_face(face_id + 1, total_cost);
    for (const auto & candidate : candidates) {
      if (candidate.face_id != face_id || candidate.detection_index < 0) continue;
      const auto detection_slot = static_cast<std::size_t>(candidate.detection_index);
      if (detection_used[detection_slot]) continue;
      detection_used[detection_slot] = true;
      current_assignment.push_back(candidate);
      assign_face(face_id + 1, total_cost + candidate.cost);
      current_assignment.pop_back();
      detection_used[detection_slot] = false;
    }
  };
  assign_face(0, 0.0);

  return selected;
}

bool Tracker::has_rigid_pair(const std::vector<AssociationCandidate> & selected) const
{
  bool rigid_pair_observed = false;
  for (std::size_t first = 0; first < selected.size(); ++first) {
    for (std::size_t second = first + 1; second < selected.size(); ++second) {
      const auto & a = selected[first];
      const auto & b = selected[second];
      if (a.observed_name != target_.name || b.observed_name != target_.name ||
          a.observed_type != target_.armor_type || b.observed_type != target_.armor_type ||
          a.measurement.color != b.measurement.color ||
          a.measurement.confidence < identity_min_confidence_ ||
          b.measurement.confidence < identity_min_confidence_) continue;
      const double observed_difference = tools::limit_rad(
        a.measurement.ypr_in_world[0] - b.measurement.ypr_in_world[0]);
      const double face_difference = (a.face_id - b.face_id) *
        2.0 * CV_PI / target_.armor_num();
      // A simultaneously observed rigid pair supplies radius/center evidence
      // before angular-rate convergence. Waiting for that convergence first
      // creates an observability deadlock at the first face handover. Distinct
      // normals must agree with their face offsets within the existing 20 deg
      // association scale; two same-facing independent objects do not qualify.
      if (std::abs(tools::limit_rad(observed_difference - face_difference)) <=
          20.0 * CV_PI / 180.0) rigid_pair_observed = true;
    }
  }
  return rigid_pair_observed;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  last_associations_.clear();
  primary_detection_index_ = -1;
  primary_face_id_ = -1;

  // Associate in the image plane against every rigid-body face, then update
  // the EKF once with the best observation.  The old loop updated the same EKF
  // sequentially with every same-class detection in one frame.  During a face
  // handover that could pull body yaw toward two different armor ids at the
  // same timestamp and create a long/short phase-crossing pair.
  const auto predicted_faces = target_.association_armor_xyza_list();
  int detection_index = 0;
  const auto candidates = collect_association_candidates(armors, predicted_faces, detection_index);
  auto selected = select_associations(candidates, detection_index, predicted_faces.size());

  if (selected.empty()) {
    identity_candidate_frames_ = 0;
    return false;
  }
  auto primary_it = std::min_element(
    selected.begin(), selected.end(), [](const AssociationCandidate & a, const AssociationCandidate & b) {
      if (std::abs(a.alignment_rad - b.alignment_rad) > 1e-6) {
        return a.alignment_rad < b.alignment_rad;
      }
      return a.cost < b.cost;
    });
  primary_detection_index_ = primary_it->detection_index;
  primary_face_id_ = primary_it->face_id;

  if (update_identity(
        primary_it->measurement, primary_it->observed_name, primary_it->observed_type, t)) {
    // Identity was corrected at the same image object. New model coordinates
    // start from face 0; the externally visible geometric ID is unchanged.
    *primary_it->armor = primary_it->measurement;
    primary_face_id_ = 0;
    last_associations_.push_back({primary_detection_index_, 0, primary_it->cost});
    measurement_updated_ = true;
    primary_observation_ = primary_it->measurement;
    return true;
  }

  // A rate spike alone cannot enable joint updates. Require either a trusted
  // rotating model or simultaneous, geometrically compatible rigid faces.
  const bool rigid_pair_observed = has_rigid_pair(selected);
  const bool use_batch = rigid_pair_observed || (target_.yaw_eskf_trusted() &&
    std::abs(target_.yaw_eskf_rate()) >= batch_min_omega_rad_s_);
  std::vector<std::pair<Armor, int>> batch;
  batch.reserve(use_batch ? selected.size() : 1);
  for (const auto & association : selected) {
    last_associations_.push_back({
      association.detection_index, association.face_id, association.cost});
    if (use_batch || association.detection_index == primary_detection_index_) {
      *association.armor = association.measurement;
      batch.emplace_back(association.measurement, association.face_id);
    }
  }

  static int association_log_counter = 0;
  if (++association_log_counter % 50 == 0) {
    tools::logger()->info(
      "[ASSOC] primary_face={} matches={} cost={:.3f} detections={} confidence={:.2f}",
      primary_face_id_, batch.size(), primary_it->cost, armors.size(),
      primary_it->armor->confidence);
  }
  diagnostics_.batch_update = use_batch;
  if (use_batch) {
    target_.update_batch(batch, primary_face_id_);
  } else {
    target_.update(primary_it->measurement, primary_face_id_);
  }
  measurement_updated_ = true;
  primary_observation_ = primary_it->measurement;
  return true;
}

#ifdef AMENT_CMAKE_FOUND
void Tracker::set_ros2_node(std::shared_ptr<rclcpp::Node> node)
{
  ros_node_ = node;
  if (ros_node_) {
    // 创建 publisher 并存储为 void*（类型擦除）
    auto pub = ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "armor_markers", 10);
    marker_pub_ = std::static_pointer_cast<void>(pub);
    tools::logger()->info("[Tracker] ROS2 marker publisher initialized on topic: armor_markers");
  }
}

void Tracker::publish_markers(
  const std::list<Target> & targets,
  const rclcpp::Time & timestamp,
  const Eigen::Vector4d & aim_xyza,
  bool has_aim_point)
{
  if (!ros_node_ || !marker_pub_) {
    return;  // ROS2 未初始化，直接返回
  }

  if (targets.empty()) {
    return;  // 没有目标，不发布
  }

  // 将 void* 转换回正确的类型
  auto pub = std::static_pointer_cast<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>(
    marker_pub_);

  visualization_msgs::msg::MarkerArray marker_array;

  // 遍历所有目标
  for (const auto & target : targets) {
    // 获取所有装甲板位姿
    std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();

    // 为每个装甲板创建一个 marker
    int marker_id = 0;
    for (const Eigen::Vector4d & xyza : armor_xyza_list) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";  // 根据你的坐标系修改
      marker.header.stamp = timestamp;  // 🔑 使用硬同步时间戳
      marker.ns = "armor_plates";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;  // 改为球体
      marker.action = visualization_msgs::msg::Marker::ADD;

      // 设置位置 (x, y, z 在世界坐标系中)
      marker.pose.position.x = xyza[0];
      marker.pose.position.y = xyza[1];
      marker.pose.position.z = xyza[2];

      // 设置姿态 (从 yaw 角度转换为四元数)
      double yaw = xyza[3];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = std::sin(yaw / 2.0);
      marker.pose.orientation.w = std::cos(yaw / 2.0);

      // 设置尺寸 - 球体（直径）
      double diameter;
      if (target.armor_type == ArmorType::small) {
        diameter = 0.08;  // 小装甲板：80mm 直径
      } else {
        diameter = 0.12;  // 大装甲板：120mm 直径
      }
      marker.scale.x = diameter;
      marker.scale.y = diameter;
      marker.scale.z = diameter;

      // 设置颜色 (绿色，半透明)
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.7;

      marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 = 永不过期 (用于诊断)

      marker_array.markers.push_back(marker);
    }

    // 如果有瞄准点，也创建一个 marker（红色球体）
    if (has_aim_point) {
      visualization_msgs::msg::Marker aim_marker;
      aim_marker.header.frame_id = "world";
      aim_marker.header.stamp = timestamp;  // 🔑 使用硬同步时间戳
      aim_marker.ns = "aim_point";
      aim_marker.id = 1000;
      aim_marker.type = visualization_msgs::msg::Marker::SPHERE;  // 球体
      aim_marker.action = visualization_msgs::msg::Marker::ADD;

      aim_marker.pose.position.x = aim_xyza[0];
      aim_marker.pose.position.y = aim_xyza[1];
      aim_marker.pose.position.z = aim_xyza[2];

      double yaw = aim_xyza[3];
      aim_marker.pose.orientation.x = 0.0;
      aim_marker.pose.orientation.y = 0.0;
      aim_marker.pose.orientation.z = std::sin(yaw / 2.0);
      aim_marker.pose.orientation.w = std::cos(yaw / 2.0);

      // 设置尺寸 - 立着的长方体
      if (target.armor_type == ArmorType::small) {
        aim_marker.scale.x = 0.08;  // 宽度（水平方向）
        aim_marker.scale.y = 0.08;  // 厚度（很薄）
        aim_marker.scale.z = 0.08;  // 高度（竖直方向）
      } else {
        aim_marker.scale.x = 0.08;  // 宽度（水平方向）
        aim_marker.scale.y = 0.08;  // 厚度（很薄）
        aim_marker.scale.z = 0.08;  // 高度（竖直方向）
      }

      // 红色
      aim_marker.color.r = 1.0;
      aim_marker.color.g = 0.0;
      aim_marker.color.b = 0.0;
      aim_marker.color.a = 0.9;

      aim_marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 = 永不过期 (用于诊断)

      marker_array.markers.push_back(aim_marker);
    }

    // 🆕 添加敌车中心的蓝色球体
    visualization_msgs::msg::Marker center_marker;
    center_marker.header.frame_id = "world";
    center_marker.header.stamp = timestamp;  // 🔑 使用硬同步时间戳
    center_marker.ns = "vehicle_center";
    center_marker.id = 2000;
    center_marker.type = visualization_msgs::msg::Marker::SPHERE;
    center_marker.action = visualization_msgs::msg::Marker::ADD;

    // 从 EKF 状态获取敌车中心位置
    Eigen::VectorXd ekf_x = target.ekf_x();
    center_marker.pose.position.x = ekf_x[0];  // 中心 X
    center_marker.pose.position.y = ekf_x[2];  // 中心 Y
    center_marker.pose.position.z = ekf_x[4];  // 中心 Z

    // 姿态（可选，球体不受姿态影响）
    center_marker.pose.orientation.x = 0.0;
    center_marker.pose.orientation.y = 0.0;
    center_marker.pose.orientation.z = 0.0;
    center_marker.pose.orientation.w = 1.0;

    double center_diameter = 0.08;  // 80mm 直径
    center_marker.scale.x = center_diameter;
    center_marker.scale.y = center_diameter;
    center_marker.scale.z = center_diameter;

    // 蓝色
    center_marker.color.r = 0.0;
    center_marker.color.g = 0.0;
    center_marker.color.b = 1.0;
    center_marker.color.a = 0.8;

    center_marker.lifetime = rclcpp::Duration::from_seconds(0.0);  // 0 = 永不过期 (用于诊断)

    marker_array.markers.push_back(center_marker);
  }

  // 发布 marker array
  pub->publish(marker_array);
}
#endif

}  // namespace auto_aim

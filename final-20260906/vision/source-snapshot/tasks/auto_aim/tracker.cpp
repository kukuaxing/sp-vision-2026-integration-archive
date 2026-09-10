#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
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
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
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
}

std::string Tracker::state() const { return state_; }

const std::vector<Tracker::ArmorAssociation> & Tracker::last_associations() const
{
  return last_associations_;
}

int Tracker::primary_detection_index() const { return primary_detection_index_; }

int Tracker::primary_face_id() const { return primary_face_id_; }

std::list<Target> Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // 过滤掉非我方装甲板
  armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

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
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

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

  if (state_ == "lost") return {};

  std::list<Target> targets = {target_};
  return targets;
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
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

  auto & armor = armors.front();
  solver_.solve(armor);

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  last_associations_.push_back({0, 0, 0.0});
  primary_detection_index_ = 0;
  primary_face_id_ = 0;

  return true;
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
  const auto predicted_faces = target_.armor_xyza_list();
  struct Candidate
  {
    Armor * armor = nullptr;
    int detection_index = -1;
    int face_id = -1;
    double cost = std::numeric_limits<double>::infinity();
    double alignment_rad = std::numeric_limits<double>::infinity();
  };
  std::vector<Candidate> candidates;
  int detection_index = 0;
  for (auto & armor : armors) {
    const int current_detection_index = detection_index++;
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;

    solver_.solve(armor);
    const double observed_range_m = std::max(0.05, armor.ypd_in_world[2]);
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
      const double angle_error = std::abs(tools::limit_rad(
        armor.ypr_in_world[0] - face[3])) / (20.0 * CV_PI / 180.0);
      const double predicted_range_m = std::max(
        0.05, tools::xyz2ypd(face.head(3))[2]);
      const double range_error =
        std::abs(std::log(observed_range_m / predicted_range_m)) / 0.25;
      const double confidence_bonus = 0.10 * std::clamp(armor.confidence, 0.0, 1.0);
      const double cost = center_error * center_error +
        0.20 * angle_error * angle_error +
        0.20 * range_error * range_error - confidence_bonus;
      if (std::isfinite(cost) && cost <= association_max_cost_) {
        const auto state = target_.ekf_x();
        const double center_yaw = std::atan2(state[2], state[0]);
        candidates.push_back({
          &armor, current_detection_index, static_cast<int>(id), cost,
          std::abs(tools::limit_rad(face[3] - center_yaw))});
      }
    }
  }

  // Solve the complete one-to-one assignment.  A greedy lowest-pair-first
  // choice can consume the only plausible detection for one face and force a
  // bad handover on another.  There are at most four faces, so an exhaustive
  // minimum-cost assignment is both deterministic and inexpensive.
  std::vector<bool> detection_used(static_cast<std::size_t>(detection_index), false);
  std::vector<Candidate> current_assignment;
  std::vector<Candidate> selected;
  double best_assignment_cost = std::numeric_limits<double>::infinity();
  std::function<void(int, double)> assign_face = [&](int face_id, double total_cost) {
    if (face_id >= static_cast<int>(predicted_faces.size()) ||
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

  if (selected.empty()) return false;
  const auto primary_it = std::min_element(
    selected.begin(), selected.end(), [](const Candidate & a, const Candidate & b) {
      if (std::abs(a.alignment_rad - b.alignment_rad) > 1e-6) {
        return a.alignment_rad < b.alignment_rad;
      }
      return a.cost < b.cost;
    });
  primary_detection_index_ = primary_it->detection_index;
  primary_face_id_ = primary_it->face_id;

  // Preserve the proven single-observation path on fixed and slow targets.
  // Joint observations are enabled only after the target is clearly in gyro
  // motion, where two simultaneously visible faces materially improve the
  // rigid-body centre, radius and phase estimate.
  const bool use_batch = target_.ekf_x().size() > 7 &&
    std::abs(target_.ekf_x()[7]) >= batch_min_omega_rad_s_;
  std::vector<std::pair<Armor, int>> batch;
  batch.reserve(use_batch ? selected.size() : 1);
  for (const auto & association : selected) {
    last_associations_.push_back({
      association.detection_index, association.face_id, association.cost});
    if (use_batch || association.detection_index == primary_detection_index_) {
      batch.emplace_back(*association.armor, association.face_id);
    }
  }

  static int association_log_counter = 0;
  if (++association_log_counter % 50 == 0) {
    tools::logger()->info(
      "[ASSOC] primary_face={} matches={} cost={:.3f} detections={} confidence={:.2f}",
      primary_face_id_, batch.size(), primary_it->cost, armors.size(),
      primary_it->armor->confidence);
  }
  if (use_batch) {
    target_.update_batch(batch, primary_face_id_);
  } else {
    target_.update(*primary_it->armor, primary_face_id_);
  }
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

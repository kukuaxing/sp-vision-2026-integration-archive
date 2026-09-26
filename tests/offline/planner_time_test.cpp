// True Target + Planner regression. Synthetic 3D observations, no device I/O.
#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
auto_aim::Armor observation(double yaw) {
  const std::vector<cv::Point2f> corners{{10, 10}, {10, 30}, {50, 30}, {50, 10}};
  auto_aim::Armor armor(2, .99f, cv::Rect(10, 10, 40, 20), corners, auto_aim::YOLOVersion::YOLO11);
  armor.name = auto_aim::three;
  armor.type = auto_aim::small;
  armor.priority = auto_aim::first;
  armor.confidence = 1.0;
  armor.xyz_in_world = Eigen::Vector3d(4.0 - 0.25 * std::cos(yaw),
                                     0.3 - 0.25 * std::sin(yaw), 0.2);
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  armor.ypr_in_world = Eigen::Vector3d(tools::limit_rad(yaw), 0.0, 0.0);
  return armor;
}
Eigen::Vector4d nearest(const auto_aim::Target & target) {
  const auto faces = target.armor_xyza_list();
  return *std::min_element(faces.begin(), faces.end(), [](const auto & a, const auto & b) {
    return a.template head<2>().norm() < b.template head<2>().norm();
  });
}
}  // namespace

int main() {
  using namespace auto_aim;
  const auto epoch = std::chrono::steady_clock::time_point{} + std::chrono::seconds(10);
  // Keep yaw selected across the whole MPC window. This intentionally broad
  // covariance trust limit isolates time consistency, not production tuning.
  // With the default limit, falling back to the vehicle EKF can accidentally
  // hide the center error while the earlier reference samples remain shifted.
  YawESKF::Config yaw_config;
  yaw_config.trusted_rate_variance = 100.0;
  Target target(observation(-0.3), epoch, 0.25, 4, Eigen::VectorXd::Ones(11), yaw_config);
  for (int i = 1; i <= 300; ++i) {
    target.predict(epoch + std::chrono::milliseconds(i * 10));
    target.update(observation(-0.3 + 0.01 * i), 0);
  }
  assert(target.yaw_eskf_trusted());
  const Eigen::VectorXd before = target.ekf_x();
  const Eigen::MatrixXd covariance_before = target.ekf().P;
  const auto current_faces = target.armor_xyza_list();
  const auto direct_faces = target.armor_xyza_list(true);
  for (double relative_time : {-0.51, -0.01, 0.0, 0.01, 0.49}) {
    const auto sampled = target.ekf_x_at(relative_time);
    const double expected_yaw = tools::limit_rad(before[6] + before[7] * relative_time +
      0.5 * target.yaw_eskf_acceleration() * relative_time * relative_time);
    assert(std::abs(tools::limit_rad(sampled[6] - expected_yaw)) < 1e-12);
    assert(std::abs(sampled[0] - before[0] - before[1] * relative_time) < 1e-12);
    const auto faces = target.armor_xyza_list_at(relative_time);
    assert(faces.size() == current_faces.size());
    if (relative_time == 0.0) {
      for (std::size_t i = 0; i < faces.size(); ++i)
        assert((faces[i] - current_faces[i]).norm() < 1e-12);
    }
  }
  assert(target.ekf_x().isApprox(before, 1e-14));
  assert(target.ekf().P.isApprox(covariance_before, 1e-14));
  const auto direct_after = target.armor_xyza_list(true);
  for (std::size_t i = 0; i < direct_after.size(); ++i)
    assert((direct_after[i] - direct_faces[i]).norm() < 1e-12);
  for (double invalid_time : {std::numeric_limits<double>::quiet_NaN(),
       std::numeric_limits<double>::infinity(), std::numeric_limits<double>::max()}) {
    bool rejected = false;
    try { target.ekf_x_at(invalid_time); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
  }
  // The mean sampler must also work before yaw has earned trust. Then the
  // ordinary constant-yaw-rate vehicle model is used in both directions.
  const Target untrusted(4.0, 1.5, 0.25, 0.0);
  assert(!untrusted.yaw_eskf_trusted());
  assert(std::abs(untrusted.ekf_x_at(-0.4)[6] + 0.6) < 1e-12);
  assert(std::abs(untrusted.ekf_x_at(0.4)[6] - 0.6) < 1e-12);
  assert(untrusted.ekf_x_at(1e100).allFinite());

  std::filesystem::create_directories("build/offline");
  const std::string path = "build/offline/planner_fixture.yaml";
  {
    std::ofstream file(path);
    file << "yaw_offset: 0\npitch_offset: 0\nfire_thresh: 0.0035\n"
         << "decision_speed: 4\nhigh_speed_delay_time: 0.085\nlow_speed_delay_time: 0.085\n"
         << "actuation_delay_ms: 40\nmax_yaw_acc: 100\nQ_yaw: [9000000,0]\nR_yaw: [0.5]\n"
         << "max_pitch_acc: 100\nQ_pitch: [9000000,0]\nR_pitch: [1]\n";
  }
  const auto current_face = nearest(target);
  const tools::Trajectory flight(22.0, current_face.head<2>().norm(), current_face[2]);
  assert(!flight.unsolvable);
  Target impact = target;
  impact.predict(flight.fly_time + 0.04);
  assert(impact.yaw_eskf_trusted());
  Target end_of_old_window = impact;
  for (int i = 0; i < HORIZON + 1; ++i) end_of_old_window.predict(DT);
  assert(end_of_old_window.yaw_eskf_trusted());
  const auto impact_face = nearest(impact);
  const double expected_yaw = std::atan2(impact_face[1], impact_face[0]);
  const tools::Trajectory impact_flight(22.0, impact_face.head<2>().norm(), impact_face[2]);
  Planner planner(path);
  const auto plan = planner.plan(target, 22.0);
  assert(plan.control);
  const double center_error = tools::limit_rad(plan.target_yaw - expected_yaw);
  std::cout << "planner center yaw error=" << center_error << " rad" << std::endl;
  assert(std::abs(center_error) < 2e-6);
  assert(std::abs(plan.target_pitch + impact_flight.pitch) < 2e-6);
  // The optional-target entry point adds processing delay, and delegates the
  // one actuation delay to plan(Target). Use an injected clock, not timing
  // tolerance dependent on workstation speed or other workloads.
  const auto now = epoch + std::chrono::seconds(3);
  Target manually_delayed = target;
  manually_delayed.predict(now + std::chrono::milliseconds(85));
  const auto manual_plan = planner.plan(manually_delayed, 22.0);
  const auto wrapped_plan = planner.plan(std::optional<Target>{target}, 22.0, now);
  assert(manual_plan.control && wrapped_plan.control);
  assert(std::abs(tools::limit_rad(wrapped_plan.target_yaw - manual_plan.target_yaw)) < 2e-6);
  assert(std::abs(wrapped_plan.target_pitch - manual_plan.target_pitch) < 2e-6);
  assert(!planner.plan(std::optional<Target>{}, 22.0, now).control);
  assert(target.ekf_x().isApprox(before, 1e-14));
  assert(target.ekf().P.isApprox(covariance_before, 1e-14));
  std::cout << "planner_time_test: PASS; trusted yaw window centered at impact; no device I/O\n";
}

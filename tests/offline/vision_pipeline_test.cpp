// Real Solver -> Target -> Aimer, synthetic corners, no model/device/actuation.
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/direct_measurement.hpp"
#include "tools/trajectory.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>

int main() {
  using namespace auto_aim;
  std::filesystem::create_directories("build/offline");
  const std::string path = "build/offline/geometry_fixture.yaml";
  {
    std::ofstream file(path);
    file << "R_gimbal2imubody: [0,-1,0,1,0,0,0,0,1]\n"
         << "R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0]\n"
         << "t_camera2gimbal: [0.02,0.01,0.03]\n"
         << "camera_matrix: [1000,0,640,0,1000,512,0,0,1]\n"
         << "distort_coeffs: [0,0,0,0,0]\n"
         << "yaw_offset: 0\npitch_offset: 0\ncomming_angle: 70\nleaving_angle: 30\n"
         << "high_speed_delay_time: 0.085\nlow_speed_delay_time: 0.085\ndecision_speed: 4\n";
  }
  Solver solver(path);
  const Eigen::Quaterniond q(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(.08, Eigen::Vector3d::UnitY()));
  solver.set_R_gimbal2world(q);
  const auto legacy = solver.R_gimbal2world();
  assert((legacy - q.toRotationMatrix()).norm() > .05); // distinguishes the conventions
  assert(solver.set_R_gimbal2world_direct(q));
  assert((solver.R_gimbal2world() - q.toRotationMatrix()).norm() < 1e-12);

  const auto fixture_points = solver.reproject_armor(Eigen::Vector3d(3., .1, .25), .1, small, three);
  Armor unsolved(3, .99f, cv::boundingRect(fixture_points), fixture_points, YOLOVersion::YOLO11);
  assert(!unsolved.pose_valid && !unsolved.xyz_in_gimbal.allFinite() && !unsolved.xyz_in_world.allFinite());
  assert(!unsolved.ypr_in_gimbal.allFinite() && !unsolved.ypr_in_world.allFinite() &&
         !unsolved.ypd_in_world.allFinite() && std::isnan(unsolved.yaw_raw));
  auto valid = unsolved;
  solver.solve(valid);
  assert(valid.pose_valid);
  // An earlier success must never survive a failed/malformed solve.
  auto failed = valid;
  failed.points.clear();
  solver.solve(failed);
  assert(!failed.pose_valid && !failed.xyz_in_world.allFinite() && !failed.xyz_in_gimbal.allFinite());
  failed = valid;
  failed.points[0].x = std::numeric_limits<float>::quiet_NaN();
  solver.solve(failed);
  assert(!failed.pose_valid && !failed.xyz_in_world.allFinite());
  failed = valid;
  failed.points.assign(4, cv::Point2f(640, 512)); // degenerate but correctly sized PnP input
  solver.solve(failed);
  assert(!failed.pose_valid && !failed.xyz_in_world.allFinite());
  solver.set_R_gimbal2world(Eigen::Quaterniond(std::numeric_limits<double>::quiet_NaN(), 0, 0, 0));
  failed = valid;
  solver.solve(failed);
  assert(!failed.pose_valid && !failed.xyz_in_world.allFinite());
  assert(solver.set_R_gimbal2world_direct(q));

  // The balance path skips yaw optimization but must still commit a valid pose.
  const Eigen::Vector3d balance_truth(3., .1, .25);
  const auto balance_points = solver.reproject_armor(balance_truth, .1, big, three);
  Armor balance(3, .99f, cv::boundingRect(balance_points), balance_points, YOLOVersion::YOLO11);
  balance.type = big;
  solver.solve(balance);
  assert(balance.pose_valid && (balance.xyz_in_world - balance_truth).norm() < .005 && std::isfinite(balance.yaw_raw));

  // Regression for fallback reading an unsolved detection's residual memory.
  // A false validity flag rejects even finite, plausible old coordinates.
  unsolved.xyz_in_gimbal = valid.xyz_in_gimbal;
  unsolved.xyz_in_world = valid.xyz_in_world;
  std::list<Armor> candidates{unsolved, valid};
  const auto first_valid = std::next(candidates.begin());
  assert(select_direct_measurement(candidates, -1, false, std::nullopt) == &*first_valid);
  assert(select_direct_measurement(candidates, 0, true, std::nullopt) == &*first_valid);
  candidates.front().pose_valid = true;
  candidates.front().xyz_in_world[0] = std::numeric_limits<double>::quiet_NaN();
  assert(select_direct_measurement(candidates, 0, true, std::nullopt) == &*first_valid);
  candidates.pop_back();
  assert(select_direct_measurement(candidates, -1, false, std::nullopt) == nullptr);
  auto wrong_name = valid;wrong_name.name = four;
  auto wrong_type = valid;wrong_type.type = big;
  candidates = {wrong_name, wrong_type, valid};
  assert(select_direct_measurement(candidates, 0, true, std::nullopt, three, small) == &candidates.back());
  auto other = valid;
  other.xyz_in_world = Eigen::Vector3d(3., 1., .25);
  candidates = {valid, other};
  assert(select_direct_measurement(candidates, 1, true, 0.0, three, small) == &candidates.back());
  assert(select_direct_measurement(candidates, 1, false, 0.0, three, small) == &candidates.front());
  assert(select_direct_measurement(candidates, -1, false, std::atan2(1., 3.), three, small) == &candidates.back());

  DirectControlGate control_gate;
  assert(!control_gate.confirmed());
  assert(!control_gate.update(true, true, false, false)); // provisional first detection
  assert(!control_gate.update(true, false, false, true)); // gap cannot bypass convergence
  assert(!control_gate.confirmed());
  assert(control_gate.update(true, true, true, false));
  assert(control_gate.confirmed());
  assert(control_gate.update(true, false, false, true)); // established target, short hold
  assert(!control_gate.update(false, false, false, true)); // target loss revokes even in hold
  assert(!control_gate.confirmed());
  assert(!control_gate.update(true, true, false, true)); // reacquisition must converge anew
  assert(!control_gate.update(true, false, true, true)); // prediction alone cannot arm
  assert(control_gate.update(true, true, true, false));
  assert(!control_gate.update(true, false, false, false)); // long gap revokes
  assert(!control_gate.confirmed());
  assert(!control_gate.update(true, true, false, false));
  assert(control_gate.update(true, true, true, false));
  control_gate.reset();
  assert(!control_gate.confirmed() && !control_gate.update(true, false, false, true));
  assert(!control_gate.update(false, true, true, true)); // no target cannot arm
  assert(!solver.set_R_gimbal2world_direct(Eigen::Quaterniond(0,0,0,0)));
  assert((solver.R_gimbal2world() - q.toRotationMatrix()).norm() < 1e-12);

  double worst_position_error = 0;
  for (double distance : {2., 4., 7.}) {
    for (double offset_y : {-.2, 0., .2}) {
      const Eigen::Vector3d truth(distance, offset_y, .25);
      const auto points = solver.reproject_armor(truth, .1, small, three);
      Armor armor(2, .99f, cv::boundingRect(points), points, YOLOVersion::YOLO11);
      solver.solve(armor);
      const auto error = (armor.xyz_in_world - truth).norm();
      worst_position_error = std::max(worst_position_error, error);
      assert(armor.pose_valid && armor.xyz_in_world.allFinite() && error < .005);
      assert((armor.xyz_in_world - solver.R_gimbal2world() * armor.xyz_in_gimbal).norm() < 1e-10);
      const auto base = std::chrono::steady_clock::time_point{} + std::chrono::seconds(10);
      Target target(armor, base, .25, 4, Eigen::VectorXd::Ones(11));
      for (int i = 1; i <= 40; ++i) {
        target.predict(base + std::chrono::milliseconds(10*i));
        target.update(armor, 0);
      }
      assert(target.ekf_x().allFinite());
      Aimer aimer(path);
      const auto command = aimer.aim({target}, base + std::chrono::milliseconds(400), 20., false);
      assert(command.control && !command.shoot);
      assert(std::isfinite(command.yaw) && std::isfinite(command.pitch));
      assert(std::abs(command.yaw - std::atan2(offset_y, distance)) < .01);
      assert(command.pitch < 0); // legacy Aimer convention: upward pitch is negative
      assert(!aimer.aim({}, base, 20., false).control);
    }
  }
  // Malformed calibration must fail before Eigen reads beyond a short vector.
  {
    std::ofstream file(path);
    file << "R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]\n"
         << "R_camera2gimbal: [1,0]\nt_camera2gimbal: [0,0,0]\n";
  }
  bool rejected = false;
  try { Solver invalid(path); } catch (const std::runtime_error&) { rejected = true; }
  assert(rejected);
  std::cout << "vision_pipeline_test: PASS; unsolved/failed/nonfinite/balance poses, direct selection; 9 synthetic views; max position error="
            << worst_position_error << " m; no detector/model/hardware tested\n";
}

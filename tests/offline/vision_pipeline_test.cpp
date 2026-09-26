// Real Solver -> Target -> Aimer, synthetic corners, no model/device/actuation.
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tools/trajectory.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>

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
      assert(armor.xyz_in_world.allFinite() && error < .005);
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
  std::cout << "vision_pipeline_test: PASS; 9 synthetic views; max position error="
            << worst_position_error << " m; no detector/model/hardware tested\n";
}

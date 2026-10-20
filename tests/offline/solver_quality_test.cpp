#include "tasks/auto_aim/solver.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

int main()
{
  using namespace auto_aim;
  const auto path = std::filesystem::temp_directory_path() /
    ("sp_solver_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  {
    std::ofstream file(path);
    file << "R_gimbal2imubody: [1,0,0,0,1,0,0,0,1]\n"
         << "R_camera2gimbal: [0,0,1,-1,0,0,0,-1,0]\n"
         << "t_camera2gimbal: [0,0,0]\n"
         << "camera_matrix: [1000,0,640,0,1000,512,0,0,1]\n"
         << "distort_coeffs: [0,0,0,0,0]\n";
  }
  Solver solver(path.string());
  std::filesystem::remove(path);
  const Eigen::Vector3d truth(3.0, 0.0, 0.0);
  const auto points = solver.reproject_armor(truth, 0.0, small, four);
  Armor valid(11, .95F, cv::boundingRect(points), points, YOLOVersion::YOLO26);
  solver.solve(valid);
  assert(valid.pose_valid && (valid.xyz_in_world - truth).norm() < .001);
  assert(valid.reprojection_error_px < .01 && valid.pose_quality > .99);

  // Numerically finite but physically degenerate shapes must remain invalid.
  for (const auto & bad : std::vector<std::vector<cv::Point2f>>{
         {{700,500},{800,500},{800,501},{700,501}},
         {{700,500},{800,540},{800,500},{700,540}},
         {{700,500},{700,500},{700,500},{700,500}},
         {{700,500},{720,500},{800,540},{700,540}}}) {
    Armor armor = valid;
    armor.points = bad;
    solver.solve(armor);
    assert(!armor.pose_valid && !armor.xyz_in_world.allFinite());
    assert(armor.pose_quality == 0.0 && !std::isfinite(armor.reprojection_error_px));
  }

  // A convex quad can still disagree with any rigid rectangle projection.
  Armor distorted = valid;
  distorted.points[1] += cv::Point2f(20.0F, -10.0F);
  solver.solve(distorted);
  assert(!distorted.pose_valid && distorted.pose_quality == 0.0);
  std::cout << "solver_quality_test: PASS; valid residual/quality, thin/crossed/collapsed/warped quadrilaterals\n";
}

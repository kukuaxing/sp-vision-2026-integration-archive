#include "tasks/auto_aim/sentinel_geometry.hpp"
#include "tasks/auto_aim/fire_pose_guard.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/math_tools.hpp"

#include <yaml-cpp/yaml.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

namespace {
constexpr double pi=3.14159265358979323846;
Eigen::Matrix3d rotation(double yaw, double pitch, double roll) {
  return (Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
}
double difference(double a,double b) { return std::abs(std::remainder(a-b,2*pi)); }
}

int main() {
  auto_aim::Solver solver("configs/standard.yaml"), legacy("configs/standard.yaml");
  const Eigen::Matrix3d m=solver.R_gimbal2imubody();
  const Eigen::Vector3d optical_sensor=m*solver.R_camera2gimbal().col(2);
  const auto_aim::SentinelGeometry geometry(m,solver.R_camera2gimbal());
  int ray_cases=0, plate_cases=0;
  double worst_legacy_model_error=0;
  for(double yaw:{-3.10,-1.0,0.0,1.0,3.10})
    for(double pitch:{-.5,0.,.5})
      for(double roll:{-.25,0.,.25}) {
        const std::array<double,3> rpy{roll,pitch,yaw};
        const Eigen::Matrix3d nav_sensor=rotation(yaw,pitch,roll);
        const Eigen::Vector3d forward=nav_sensor*optical_sensor;
        const double current_heading=std::atan2(forward.y(),forward.x());
        const double current_pitch=-std::asin(forward.z());
        const auto held=geometry.sensor_angles_for_world_ray(current_heading,current_pitch,rpy);
        assert(held && difference((*held)[0],yaw)<1e-10 && difference((*held)[1],pitch)<1e-10);
        for(double desired_yaw:{-3.12,-.7,.4,3.12})
          for(double desired_pitch:{-.6,-.1,.2,.6}) {
            const auto command=geometry.sensor_angles_for_world_ray(desired_yaw,desired_pitch,rpy);
            assert(command);
            const Eigen::Vector3d actual=rotation((*command)[0],(*command)[1],roll)*optical_sensor;
            const Eigen::Vector3d desired(std::cos(desired_pitch)*std::cos(desired_yaw),
              std::cos(desired_pitch)*std::sin(desired_yaw),-std::sin(desired_pitch));
            assert((actual-desired).norm()<1e-10);
            ++ray_cases;
          }
        for(double desired_yaw:{-3.12,-.7,.4,3.12}) {
          const auto command=geometry.sensor_yaw_for_world_bearing(desired_yaw,rpy);
          const Eigen::Vector3d actual=rotation(command,pitch,roll)*optical_sensor;
          assert(difference(std::atan2(actual.y(),actual.x()),desired_yaw)<1e-10);
          assert(difference(geometry.world_bearing_for_sensor_yaw(command,rpy),desired_yaw)<1e-10);
        }
      }

  // Generate actual image corners from an independent NAV camera transform,
  // with a physical plate whose top leans back by 15 degrees. The old tilted
  // world can fit raw PnP yet disagrees with that gravity-fixed model.
  const auto yaml=YAML::LoadFile("configs/standard.yaml");
  const auto k=yaml["camera_matrix"].as<std::vector<double>>();
  const auto d=yaml["distort_coeffs"].as<std::vector<double>>();
  cv::Mat K(3,3,CV_64F,const_cast<double*>(k.data()));
  cv::Mat D(1,5,CV_64F,const_cast<double*>(d.data()));
  // Real calibration: a settled stationary target can be far from image
  // centre. Candidate and post-wait checks must compare calibrated sensor yaw.
  {
    const double correction=(yaml["xuc_direct_yaw_boresight_correction_deg"].as<double>()+
      yaml["xuc_direct_yaw_impact_correction_deg"].as<double>())*pi/180.;
    const std::array<double,3> rpy{.01,-.02,.5};
    const auto nav=rotation(rpy[2],rpy[1],rpy[0]);
    const Eigen::Vector3d forward=nav*optical_sensor;
    const double bearing=geometry.camera_world_yaw(rpy)-correction;
    const Eigen::Vector3d point(3*std::cos(bearing),3*std::sin(bearing),
      3*forward.z()/forward.head<2>().norm());
    const double ref=geometry.calibrated_yaw_for_world_point(point,rpy,correction);
    assert(difference(ref,rpy[2])<1e-10);
    io::sentinel::PoseEstimate before,after;
    before.valid=true;before.boot_id=1;before.epoch=1;before.generation=1;after=before;
    const double tolerance=yaml["xuc_fire_yaw_tolerance_deg"].as<double>()*pi/180.;
    assert(auto_aim::fire_pose_guard(before,after,rpy[2],ref,tolerance,0,.1).allowed);
    auto moved=rpy;moved[2]+=.03;
    const double final_ref=geometry.calibrated_yaw_for_world_point(point,moved,correction);
    assert(difference(final_ref,ref)<1e-10);
    assert(auto_aim::fire_pose_guard(before,after,moved[2],final_ref,tolerance,0,.1).reason==
      auto_aim::FirePoseReject::yaw_error);
    const Eigen::Vector3d camera=solver.R_camera2gimbal().transpose()*
      (geometry.world_rotation(rpy).transpose()*point-solver.t_camera2gimbal());
    std::vector<cv::Point2d> pixel;
    cv::projectPoints(std::vector<cv::Point3d>{{camera.x(),camera.y(),camera.z()}},
      cv::Vec3d{},cv::Vec3d{},K,D,pixel);
    const double old_error=std::atan2(pixel[0].x-720.,k[0])-
      yaml["xuc_direct_yaw_impact_correction_deg"].as<double>()*pi/180.;
    assert(std::abs(old_error)>tolerance); // regression: old gate rejects settled aim
    assert(std::isnan(geometry.calibrated_yaw_for_world_point(Eigen::Vector3d::Zero(),rpy,correction)));
  }
  const std::vector<cv::Point3f> points{{0,.0675f,.028f},{0,-.0675f,.028f},
    {0,-.0675f,-.028f},{0,.0675f,-.028f}};
  const Eigen::Matrix3d plate_local=rotation(0,15*pi/180,0);
  assert((plate_local*Eigen::Vector3d(0,0,.028)).x()>0);
  for(double yaw:{-1.5,-.9,.4,1.5})
    for(double pitch:{-.15,.15}) {
      const Eigen::Matrix3d nav_sensor=rotation(yaw,pitch,.02);
      const Eigen::Matrix3d nav_gimbal=nav_sensor*m;
      const Eigen::Matrix3d nav_camera=nav_gimbal*solver.R_camera2gimbal();
      const Eigen::Vector3d center=nav_gimbal*(solver.R_camera2gimbal()*
        Eigen::Vector3d(0,0,3)+solver.t_camera2gimbal());
      const double camera_yaw=std::atan2(nav_camera(1,2),nav_camera(0,2));
      const Eigen::Matrix3d camera_plate=nav_camera.transpose()*rotation(camera_yaw+.25,15*pi/180,0);
      cv::Mat rm;cv::eigen2cv(camera_plate,rm);cv::Vec3d rv;cv::Rodrigues(rm,rv);
      std::vector<cv::Point2f> observed;
      cv::projectPoints(points,rv,cv::Vec3d(0,0,3),K,D,observed);
      auto_aim::Armor armor(11,.9f,cv::boundingRect(observed),observed,auto_aim::YOLOVersion::YOLO26);
      armor.type=auto_aim::ArmorType::small;
      armor.name=auto_aim::ArmorName::three;armor.confidence=.9;armor.pose_observation_valid=true;
      assert(solver.set_R_gimbal2world_absolute_imu(Eigen::Quaterniond(nav_sensor)));
      assert(solver.R_gimbal2world().isApprox(nav_gimbal,1e-12));
      assert(geometry.world_rotation({.02,pitch,yaw}).isApprox(nav_gimbal,1e-12));
      solver.solve(armor);
      assert(armor.pose_valid);
      assert((armor.xyz_in_world-center).norm()<.002);
      assert(armor.model_reprojection_error_px<1.0);
      auto old_armor=armor;
      legacy.set_R_gimbal2world(Eigen::Quaterniond(nav_sensor));legacy.solve(old_armor);
      assert(old_armor.pose_valid);
      worst_legacy_model_error=std::max(worst_legacy_model_error,old_armor.model_reprojection_error_px);
      ++plate_cases;
    }
  assert(worst_legacy_model_error>3.0);
  const auto held_rotation=solver.R_gimbal2world();
  assert(!solver.set_R_gimbal2world_absolute_imu(Eigen::Quaterniond(0,0,0,0)));
  assert(solver.R_gimbal2world().isApprox(held_rotation,0));
  assert(!geometry.sensor_angles_for_world_ray(NAN,0,{0,0,0}));
  assert(!geometry.sensor_angles_for_world_ray(0,pi/2,{0,0,0}));
  std::cout<<"PASS Sentinel NAV geometry: "<<ray_cases<<" optical-ray inverse cases, "
    <<plate_cases<<" tilted-plate image cases; legacy worst model error="<<worst_legacy_model_error<<" px\n";
}

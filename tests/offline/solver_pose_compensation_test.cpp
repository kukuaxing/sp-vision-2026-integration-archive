#include "tasks/auto_aim/solver.hpp"

#include <yaml-cpp/yaml.h>

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
Eigen::Matrix3d rotation(double yaw,double pitch,double roll) {
  return (Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// Independent Brown-Conrady projection (not another call to projectPoints).
Eigen::Vector2d image_angle(const Eigen::Vector3d & world,const Eigen::Matrix3d & pose,
  const Eigen::Matrix3d & camera_rotation,const Eigen::Vector3d & camera_translation,
  const std::vector<double> & k,const std::vector<double> & d,const cv::Size & image_size) {
  const Eigen::Vector3d p=camera_rotation.transpose()*(pose.transpose()*world-camera_translation);
  assert(p.z()>0);
  const double x=p.x()/p.z(),y=p.y()/p.z(),r2=x*x+y*y;
  const double radial=1+d[0]*r2+d[1]*r2*r2+d[4]*r2*r2*r2;
  const double xd=x*radial+2*d[2]*x*y+d[3]*(r2+2*x*x);
  const double yd=y*radial+d[2]*(r2+2*y*y)+2*d[3]*x*y;
  return {std::atan2(k[0]*xd+k[2]-.5*image_size.width,k[0]),
    std::atan2(k[4]*yd+k[5]-.5*image_size.height,k[4])};
}
}

int main() {
  auto yaml=YAML::LoadFile("configs/standard.yaml");
  auto_aim::Solver solver("configs/standard.yaml");
  const auto k=yaml["camera_matrix"].as<std::vector<double>>();
  const auto d=yaml["distort_coeffs"].as<std::vector<double>>();
  const cv::Size image_size(yaml["camera_width"].as<int>(),yaml["camera_height"].as<int>());
  const Eigen::Matrix3d camera_rotation=solver.R_camera2gimbal();
  const Eigen::Vector3d camera_translation=solver.t_camera2gimbal();
  const Eigen::Matrix3d mounting=solver.R_gimbal2imubody();
  solver.set_R_gimbal2world(Eigen::Quaterniond(rotation(.5,.2,-.1)));
  const Eigen::Matrix3d held_solver_pose=solver.R_gimbal2world();
  int cases=0;
  bool yaw_to_pitch_coupling_observed=false;
  for(double range:{.5,1.,3.,7.,12.})
    for(double yaw:{-2.8,.3,2.9})
      for(double pitch:{-.45,0.,.45})
        for(const Eigen::Vector2d ray:{Eigen::Vector2d(-.15,-.12),Eigen::Vector2d(.13,.09)})
          for(const Eigen::Vector3d change:{Eigen::Vector3d(.10,0,0),Eigen::Vector3d(0,.20,0),
              Eigen::Vector3d(-.12,-.18,.07),Eigen::Vector3d::Zero().eval()}) {
            const Eigen::Matrix3d from=mounting.transpose()*rotation(yaw,pitch,.1)*mounting;
            // Exact-zero identity requires bit-identical input poses. Separate
            // floating-point rotation evaluations can round differently under
            // optimized GCC, even when the requested angle change is zero.
            Eigen::Matrix3d to=from;
            if(!change.isZero(0))
              to=mounting.transpose()*
                rotation(yaw+change[0],pitch+change[1],.1+change[2])*mounting;
            const Eigen::Vector3d world=from*(camera_rotation*
              Eigen::Vector3d(ray.x()*range,ray.y()*range,range)+camera_translation);
            const auto delta=solver.image_angular_delta(world,from,to,image_size);
            assert(delta);
            const Eigen::Vector2d expected=image_angle(world,to,camera_rotation,camera_translation,k,d,image_size)-
              image_angle(world,from,camera_rotation,camera_translation,k,d,image_size);
            assert((*delta-expected).norm()<1e-10);
            if(change.isZero(0))assert(delta->isZero(0));
            if(change[0]!=0 && change[1]==0 && change[2]==0 && std::abs((*delta)[1])>1e-4)
              yaw_to_pitch_coupling_observed=true;
            // A relative correction retimes even a biased original measurement
            // without replacing that measurement by a fitted PnP centre.
            const Eigen::Vector2d original=image_angle(world,from,camera_rotation,camera_translation,k,d,image_size)+
              Eigen::Vector2d(.003,-.004);
            const Eigen::Vector2d corrected=original+*delta;
            const Eigen::Vector2d truth=image_angle(world,to,camera_rotation,camera_translation,k,d,image_size)+
              Eigen::Vector2d(.003,-.004);
            assert((corrected-truth).norm()<1e-10);
            ++cases;
          }
  assert(yaw_to_pitch_coupling_observed);
  assert(solver.R_gimbal2world().isApprox(held_solver_pose,0));
  const Eigen::Matrix3d identity=Eigen::Matrix3d::Identity();
  const Eigen::Vector3d world=camera_rotation*Eigen::Vector3d(0,0,3)+camera_translation;
  const Eigen::Vector3d behind=camera_rotation*Eigen::Vector3d(0,0,-3)+camera_translation;
  assert(!solver.image_angular_delta(behind,identity,identity,image_size));
  assert(!solver.image_angular_delta(world,identity,rotation(CV_PI,0,0),image_size));
  assert(!solver.image_angular_delta(world,identity,identity,cv::Size(0,image_size.height)));
  Eigen::Vector3d bad_world=world;bad_world.x()=std::numeric_limits<double>::quiet_NaN();
  assert(!solver.image_angular_delta(bad_world,identity,identity,image_size));
  Eigen::Matrix3d bad_pose=identity;bad_pose(0,0)=std::numeric_limits<double>::infinity();
  assert(!solver.image_angular_delta(world,bad_pose,identity,image_size));
  assert(!solver.image_angular_delta(world,identity,bad_pose,image_size));
  std::cout << "PASS pose reprojection: "<<cases<<" real-calibration multi-range/large-pitch/combined-axis cases; "
    "independent distortion model, zero-motion identity, residual preservation, behind-camera/nonfinite rejection\n";
}

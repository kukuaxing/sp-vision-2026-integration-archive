#pragma once
#include "fixed_target_world.hpp"
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

namespace fixed_sim {
struct Camera {
  cv::Mat K,dist,R,t;
  Eigen::Matrix3d camera_rotation;
  Eigen::Vector3d translation,origin;
  Camera(const YAML::Node& config,const Eigen::Matrix3d& sensor_pose) {
    const auto k=config["camera_matrix"].as<std::vector<double>>();
    const auto d=config["distort_coeffs"].as<std::vector<double>>();
    const auto r=config["R_camera2gimbal"].as<std::vector<double>>();
    const auto tr=config["t_camera2gimbal"].as<std::vector<double>>();
    K=cv::Mat(3,3,CV_64F,const_cast<double*>(k.data())).clone();
    dist=cv::Mat(1,5,CV_64F,const_cast<double*>(d.data())).clone();
    camera_rotation=Eigen::Matrix<double,3,3,Eigen::RowMajor>(r.data());
    translation=Eigen::Vector3d(tr.data());pose(sensor_pose);
  }
  void pose(const Eigen::Matrix3d& sensor_pose) {
    const Eigen::Matrix3d world_rotation=sensor_pose*camera_rotation;
    origin=sensor_pose*translation;
    cv::Mat camera_to_world(3,3,CV_64F);
    for(int row=0;row<3;++row)for(int col=0;col<3;++col)
      camera_to_world.at<double>(row,col)=world_rotation(row,col);
    R=camera_to_world.t();t=-R*cv::Mat(3,1,CV_64F,origin.data());
  }
  std::vector<cv::Point2f> project(const std::array<Eigen::Vector3d,4>& points) const {
    std::vector<cv::Point3d> world;for(const auto& p:points)world.emplace_back(p.x(),p.y(),p.z());
    cv::Mat rv;cv::Rodrigues(R,rv);std::vector<cv::Point2d> pixels;
    cv::projectPoints(world,rv,t,K,dist,pixels);
    std::vector<cv::Point2f> result;for(const auto& p:pixels)result.emplace_back(p);return result;
  }
  bool in_front(const std::array<Eigen::Vector3d,4>& points) const {
    for(const auto& p:points)if(R.at<double>(2,0)*p.x()+R.at<double>(2,1)*p.y()+
      R.at<double>(2,2)*p.z()+t.at<double>(2)<=1e-6)return false;
    return true;
  }
};
}

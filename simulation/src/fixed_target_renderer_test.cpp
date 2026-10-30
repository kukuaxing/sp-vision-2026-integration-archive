#include "fixed_target_renderer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include <cassert>
#include <iostream>
int main() {
  const auto yaml=YAML::LoadFile("configs/standard.yaml");
  auto_aim::Solver solver("configs/standard.yaml");fixed_sim::World world;
  const auto pose=Eigen::Quaterniond::FromTwoVectors(solver.R_camera2gimbal().col(2),world.center.normalized());
  solver.set_R_gimbal2world_direct(pose);fixed_sim::Camera camera(yaml,pose.toRotationMatrix());
  const auto independently_projected=camera.project(world.corners(0,0,.056));
  const auto production_projection=solver.reproject_armor(world.plate(0,0).center,0,auto_aim::small,auto_aim::four);
  for(int i=0;i<4;++i)assert(cv::norm(independently_projected[i]-production_projection[i])<.001);
  // Independent world projection -> production PnP must preserve metric scale
  // across range, obliquity and camera/gimbal pose, with real nonzero extrinsics.
  int roundtrips=0;
  for(double range:{2.,3.,4.5})for(double yaw_deg:{-8.,0.,8.})for(double pitch_deg:{-5.,5.}) {
    fixed_sim::World fixture;fixture.center={range,.07,.03};
    const Eigen::Quaterniond attitude=Eigen::AngleAxisd(yaw_deg*fixed_sim::pi/180,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(pitch_deg*fixed_sim::pi/180,Eigen::Vector3d::UnitY())*pose;
    solver.set_R_gimbal2world_direct(attitude);fixed_sim::Camera sensor(yaml,attitude.toRotationMatrix());
    for(double phase_deg:{-50.,-25.,0.,25.,50.}) {
      fixture.initial_phase=phase_deg*fixed_sim::pi/180;
      const auto pixels=sensor.project(fixture.corners(0,0,.056));
      auto_aim::Armor a(11,.99f,cv::boundingRect(pixels),pixels,auto_aim::YOLOVersion::YOLO26);
      a.pose_observation_valid=true;solver.solve(a);
      assert(a.pose_valid && (a.xyz_in_world-fixture.plate(0,0).center).norm()<2e-5);
      ++roundtrips;
    }
  }
  assert(roundtrips==90);solver.set_R_gimbal2world_direct(pose);
  fixed_sim::Renderer renderer(nlohmann::json::object(),{1440,1080},27);
  const auto image=renderer.render(world,camera,0);assert(image.size()==cv::Size(1440,1080));
  fixed_sim::Renderer explicit_original({{"lamp_style","original"}},{1440,1080},27);
  assert(cv::norm(image,explicit_original.render(world,camera,0),cv::NORM_INF)==0);
  for(const std::string style:{"uniform_round","uniform_flat"}) {
    fixed_sim::Renderer appearance({{"lamp_style",style}},{1440,1080},27);
    const auto changed=appearance.render(world,camera,0);
    assert(cv::norm(image,changed,cv::NORM_L1)>100);
    const auto label=appearance.truth(world,camera,0,55*fixed_sim::pi/180);
    assert(label.size()==1);
    for(int i=0;i<4;++i)assert(cv::norm(label.front().corners[i]-independently_projected[i])<.001);
    // The new fixtures must not change extent while changing photometry/caps.
    for(const auto ends:{std::pair<int,int>{0,3},std::pair<int,int>{1,2}}) {
      const auto start=independently_projected[ends.first],end=independently_projected[ends.second];
      const double length=cv::norm(end-start);const auto along=(end-start)*(1./length);
      double low=1e9,high=-1e9;
      for(int row=0;row<changed.rows;++row)for(int col=0;col<changed.cols;++col) {
        const auto color=changed.at<cv::Vec3b>(row,col);
        if(color[0]<=100 || static_cast<int>(color[0])-color[2]<=15)continue;
        const cv::Point2f delta=cv::Point2f(col,row)-start;
        if(std::abs(delta.x*along.y-delta.y*along.x)>8)continue;
        const double projection=delta.dot(along);low=std::min(low,projection);high=std::max(high,projection);
      }
      assert(std::abs(low)<1.6 && std::abs(high-length)<1.6);
    }
  }
  for(const nlohmann::json invalid:{nlohmann::json{{"lamp_style","unknown"}},
      nlohmann::json{{"lamp_style","uniform_flat"},{"texture_path","not-opened.png"}}}) {
    bool rejected=false;
    try{fixed_sim::Renderer bad(invalid,{1440,1080},27);}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  cv::Mat channels[3];cv::split(image,channels);
  cv::Mat blue;cv::compare(channels[0]-channels[2],50,blue,cv::CMP_GT);
  assert(cv::countNonZero(blue)>30); // actual blue lamp pixels, not just annotation
  const auto reference=renderer.truth(world,camera,0,55*fixed_sim::pi/180);
  assert(reference.size()==1 && reference.front().face==0);
  // Truth endpoints describe the full luminous extent, not cv::line's
  // centre-line endpoints (which would add a round cap outside the label).
  for(const auto ends:{std::pair<int,int>{0,3},std::pair<int,int>{1,2}}) {
    const auto start=independently_projected[ends.first],end=independently_projected[ends.second];
    const double length=cv::norm(end-start);const auto along=(end-start)*(1./length);
    double low=1e9,high=-1e9;
    for(int row=0;row<image.rows;++row)for(int col=0;col<image.cols;++col) {
      const auto color=image.at<cv::Vec3b>(row,col);
      if(color[0]<=100 || static_cast<int>(color[0])-color[2]<=15)continue;
      const cv::Point2f delta=cv::Point2f(col,row)-start;
      if(std::abs(delta.x*along.y-delta.y*along.x)>8)continue;
      const double projection=delta.dot(along);low=std::min(low,projection);high=std::max(high,projection);
    }
    assert(std::abs(low)<1.6 && std::abs(high-length)<1.6);
  }
  fixed_sim::Renderer blank({{"blank_scene",true}},{1440,1080},27);
  assert(blank.truth(world,camera,0,1).empty());
  assert(cv::norm(blank.render(world,camera,0),cv::Mat(image.size(),CV_8UC3,cv::Scalar(65,65,65)),cv::NORM_INF)==0);
  world.center.x()=-3;assert(renderer.truth(world,camera,0,1).empty());
  const auto behind=renderer.render(world,camera,0);
  assert(cv::norm(behind,cv::Mat(image.size(),CV_8UC3,cv::Scalar(65,65,65)),cv::NORM_INF)==0);
  world.center.x()=3;
  fixed_sim::Renderer exposure({{"exposure_s",.006},{"exposure_samples",5}},{1440,1080},27);
  const auto blurred=exposure.render(world,camera,.2);
  assert(cv::norm(blurred,renderer.render(world,camera,.2),cv::NORM_L1)>100);
  std::cout<<"PASS camera projection, lamp pixels, blank/behind-camera rejection and exposure integration\n";
}

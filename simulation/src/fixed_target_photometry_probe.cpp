// File-only photometric metrology. Oracle ROIs are diagnostic, never detections.
#include "fixed_target_renderer.hpp"
#include "tasks/auto_aim/adaptive_corner_refiner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

using Json=nlohmann::json;
static Json points(const std::vector<cv::Point2f>& p) {
  Json result=Json::array();for(const auto& v:p)result.push_back({v.x,v.y});return result;
}
static std::string pixels_checksum(const cv::Mat& image) {
  // Compatibility checksum, not a cryptographic identity or pixel proof.
  uint64_t value=14695981039346656037ULL;
  for(int y=0;y<image.rows;++y)for(size_t i=0;i<image.cols*image.elemSize();++i)
    {value^=image.ptr<unsigned char>(y)[i];value*=1099511628211ULL;}
  std::ostringstream out;out<<std::hex<<std::setfill('0')<<std::setw(16)<<value;return out.str();
}
static double sample(const cv::Mat& image,cv::Point2f p) {
  const int x=std::floor(p.x),y=std::floor(p.y);const double a=p.x-x,b=p.y-y;
  if(x<0||y<0||x+1>=image.cols||y+1>=image.rows)throw std::runtime_error("truncated photometry sample");
  return (1-b)*((1-a)*image.at<uchar>(y,x)+a*image.at<uchar>(y,x+1))+
    b*((1-a)*image.at<uchar>(y+1,x)+a*image.at<uchar>(y+1,x+1));
}
static Json photometry(const cv::Mat& image,const std::vector<cv::Point2f>& truth,
                       const auto_aim::AdaptiveCornerRefiner::Config& config) {
  std::vector<cv::Mat> channels;cv::split(image,channels);cv::Mat gray;
  cv::absdiff(channels[0],channels[2],gray);Json result=Json::array();
  for(auto ends:{std::pair<int,int>{0,3},{1,2}}) {
    const auto top=truth[ends.first],bottom=truth[ends.second],center=(top+bottom)*.5f;
    const double length=cv::norm(top-bottom);
    const double width=std::max(12.,length*config.roi_width_scale);
    const double height=std::max(12.,length*config.roi_length_scale);
    const int x0=std::max(0,int(std::floor(center.x-width*.5))),y0=std::max(0,int(std::floor(center.y-height*.5)));
    const int x1=std::min(image.cols,int(std::ceil(center.x+width*.5))),y1=std::min(image.rows,int(std::ceil(center.y+height*.5)));
    const cv::Rect roi{x0,y0,x1-x0,y1-y0};cv::Mat blurred,binary;
    cv::GaussianBlur(gray(roi),blurred,{3,3},0);
    const double otsu=cv::threshold(blurred,binary,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);
    const double blend=std::clamp(.65*otsu+.35*config.binary_threshold,25.,230.);
    const double selected=config.brightness_invariant_subpixel?std::max(1.,otsu):blend;
    const auto offset=cv::Point2f(x0,y0);
    double peak=0;for(int i=0;i<=40;++i)peak=std::max(peak,sample(blurred,top+(bottom-top)*(i/40.f)-offset));
    result.push_back({{"otsu",otsu},{"blended",blend},{"selected",selected},
      {"axis_peak",peak},{"axis_top",sample(blurred,top-offset)},
      {"axis_bottom",sample(blurred,bottom-offset)},
      {"selected_over_axis_peak",selected/std::max(1.,peak)}});
  }
  return result;
}
int main(int argc,char** argv) {
  if(argc!=3&&argc!=4){std::cerr<<"usage: fixed_target_photometry_probe CONFIG OUTPUT [--compat]\n";return 2;}
  const bool compat=argc==4&&std::string(argv[3])=="--compat";
  if(argc==4&&!compat)return 2;
  const auto yaml=YAML::LoadFile(argv[1]);auto_aim::Solver solver(argv[1]);
  auto_aim::AdaptiveCornerRefiner::Config config;
  config.binary_threshold=yaml["threshold"].as<int>(100);
  config.roi_length_scale=yaml["corner_refine_roi_length_scale"].as<double>(config.roi_length_scale);
  config.roi_width_scale=yaml["corner_refine_roi_width_scale"].as<double>(config.roi_width_scale);
  config.max_center_error_ratio=yaml["corner_refine_max_center_error_ratio"].as<double>(config.max_center_error_ratio);
  config.max_angle_error_deg=yaml["corner_refine_max_angle_error_deg"].as<double>(config.max_angle_error_deg);
  config.max_pair_angle_error_deg=yaml["corner_refine_max_pair_angle_error_deg"].as<double>(config.max_pair_angle_error_deg);
  config.min_pair_length_ratio=yaml["corner_refine_min_pair_length_ratio"].as<double>(config.min_pair_length_ratio);
  Json rows=Json::array();int image_id=0;
  const auto styles=compat?std::vector<std::string>{"original"}:
    std::vector<std::string>{"original","uniform_round","uniform_flat"};
  const auto distances=compat?std::vector<double>{3.}:std::vector<double>{2.,3.,4.};
  const auto angles=compat?std::vector<double>{-45.,0.,45.}:std::vector<double>{-45.,-20.,0.,20.,45.};
  for(const auto& style:styles)for(double distance:distances)for(double angle:angles)
    for(int direction:{-1,1})for(double brightness:{.55,1.,1.45})for(double exposure:{0.,.006})for(double blur:{0.,.6}) {
      fixed_sim::World world;world.center={distance,.07,.03};world.omega=direction*4.56;
      world.initial_phase=angle*fixed_sim::pi/180;world.epoch=1.;
      const auto pose=Eigen::Quaterniond::FromTwoVectors(solver.R_camera2gimbal().col(2),world.center.normalized());
      solver.set_R_gimbal2world_direct(pose);fixed_sim::Camera camera(yaml,pose.toRotationMatrix());
      Json settings={{"lamp_style",style},{"brightness",brightness},{"exposure_s",exposure},
        {"exposure_samples",exposure>0?5:1},{"lens_blur_sigma_px",blur}};
      fixed_sim::Renderer renderer(settings,{yaml["camera_width"].as<int>(),yaml["camera_height"].as<int>()},130003);
      const auto image=renderer.render(world,camera,1.);const auto p=camera.project(world.corners(0,1.,.056));
      const auto true_center=world.plate(0,1.).center;const auto checksum=pixels_checksum(image);
      for(bool candidate:{false,true}) {
        config.brightness_invariant_subpixel=candidate;auto_aim::AdaptiveCornerRefiner refiner(config);
        const auto found=refiner.refine(image,{p[0],p[1],p[2],p[3]},auto_aim::AdaptiveCornerRefiner::ChannelMode::red_blue_contrast);
        Json row={{"image_id",image_id},{"style",style},{"distance",distance},{"angle_deg",angle},
          {"direction",direction},{"brightness",brightness},{"exposure_s",exposure},{"blur_px",blur},
          {"candidate",candidate},{"refined",bool(found)},{"pose_valid",false},
          {"pixel_checksum_fnv1a64",checksum},{"photometry",photometry(image,p,config)},
          {"truth_corners",points(p)},{"truth_xyz",{true_center.x(),true_center.y(),true_center.z()}},
          {"camera_world_quaternion_wxyz",{pose.w(),pose.x(),pose.y(),pose.z()}}};
        if(found) {
          std::vector<cv::Point2f> q(found->points.begin(),found->points.end());
          row["measured_corners"]=points(q);
          auto_aim::Armor a(11,.99f,cv::boundingRect(q),q,auto_aim::YOLOVersion::YOLO26);
          a.pose_observation_valid=true;solver.solve(a);row["pose_valid"]=a.pose_valid;
          if(a.pose_valid)row["estimated_xyz"]={a.xyz_in_world.x(),a.xyz_in_world.y(),a.xyz_in_world.z()};
        }
        rows.push_back(std::move(row));
      }
      ++image_id;
    }
  const Json result={{"scope","offline photometry diagnosis; oracle ROI, no YOLO/tracker/shots"},
    {"physical_device_opened",false},{"seed",130003},{"images",image_id},{"compat",compat},{"rows",rows}};
  std::ofstream out(argv[2]);out<<result.dump(2)<<'\n';
  std::cout<<"images="<<image_id<<" rows="<<rows.size()<<std::endl;
}

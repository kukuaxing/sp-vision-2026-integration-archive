#pragma once
#include "fixed_target_camera.hpp"
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <random>

namespace fixed_sim {
// Procedural appearance is a test fixture, not photorealistic evidence. The
// detector only receives render(). Truth quadrilaterals stay in the evaluator.
class Renderer {
 public:
  struct Truth {int face;std::vector<cv::Point2f> corners;};
  Renderer(const nlohmann::json& settings,cv::Size size,unsigned seed)
    :size_(size),random_(seed^0x1296abb) {
    exposure_=settings.value("exposure_s",0.0);samples_=settings.value("exposure_samples",1);
    brightness_=settings.value("brightness",1.0);blur_=settings.value("lens_blur_sigma_px",0.0);
    noise_=settings.value("sensor_noise_stddev",0.0);number_=settings.value("number",4);
    blank_=settings.value("blank_scene",false);
    if(!(size.width>0 && size.height>0 && samples_>=1 && samples_<=16 && exposure_>=0 && exposure_<=.02 &&
         brightness_>0 && brightness_<=2 && blur_>=0 && blur_<=5 && noise_>=0 && noise_<=30 && number_>=2 && number_<=5))
      throw std::invalid_argument("invalid renderer fixture settings");
    const auto texture_path=settings.value("texture_path",std::string{});
    const auto lamp_style=settings.value("lamp_style",std::string{"original"});
    if(lamp_style!="original" && lamp_style!="uniform_round" && lamp_style!="uniform_flat")
      throw std::invalid_argument("unknown procedural lamp style");
    if(!texture_path.empty() && lamp_style!="original")
      throw std::invalid_argument("procedural lamp style cannot modify a supplied texture");
    if(!texture_path.empty()) {
      texture_=cv::imread(texture_path);
      if(texture_.size()!=cv::Size(512,384))throw std::invalid_argument("rectified texture must be 512x384");
    } else {
      texture_=cv::Mat(384,512,CV_8UC3,cv::Scalar(12,12,12));
      cv::rectangle(texture_,{8,10},{504,374},{38,38,38},3);
      const std::string digit=std::to_string(number_);int baseline=0;
      const auto text_size=cv::getTextSize(digit,cv::FONT_HERSHEY_DUPLEX,12,14,&baseline);
      cv::putText(texture_,digit,{(512-text_size.width)/2,(384+text_size.height)/2},cv::FONT_HERSHEY_DUPLEX,12,
        {230,230,230},14,cv::LINE_AA);
      for(int x:{64,448}) {
        // Appearance-only diagnostic fixtures share the same labelled 160px
        // luminous extent and 30px outer width. They do not change geometry.
        if(lamp_style=="uniform_round") {
          cv::line(texture_,{x,127},{x,257},{255,95,30},30,cv::LINE_AA);
          continue;
        }
        if(lamp_style=="uniform_flat") {
          // Pixel centers on the labelled continuous box boundary have half
          // area coverage. An inclusive integer rectangle would silently add
          // half a source pixel at each end, biasing the metrology fixture.
          const cv::Vec3d foreground{255,95,30},background{12,12,12};
          for(int y=112;y<=272;++y)for(int xx=x-15;xx<=x+15;++xx) {
            const double cover=(y==112||y==272?.5:1.)*(xx==x-15||xx==x+15?.5:1.);
            const auto value=background+(foreground-background)*cover;
            texture_.at<cv::Vec3b>(y,xx)={cv::saturate_cast<uchar>(value[0]),
              cv::saturate_cast<uchar>(value[1]),cv::saturate_cast<uchar>(value[2])};
          }
          continue;
        }
        // cv::line round caps extend half the thickness beyond the centre
        // line. Keep their full luminous extent at the annotated 112..272,
        // otherwise an artificial 3-5px endpoint bias enters the PnP test.
        cv::line(texture_,{x,127},{x,257},{115,30,10},30,cv::LINE_AA);
        cv::line(texture_,{x,122},{x,262},{255,95,30},19,cv::LINE_AA);
        cv::line(texture_,{x,117},{x,267},{255,240,220},9,cv::LINE_AA);
      }
    }
    alpha_=cv::Mat(texture_.size(),CV_8UC1,cv::Scalar(255));
    background_=cv::Mat(size_,CV_8UC3,cv::Scalar(65,65,65));
  }
  std::vector<Truth> truth(const World& world,const Camera& camera,double time,double visible_angle) const {
    std::vector<Truth> result;
    if(blank_)return result;
    for(int face=0;face<4;++face) {
      const auto plate=world.plate(face,time);
      if(plate.normal.dot((camera.origin-plate.center).normalized())<std::cos(visible_angle))continue;
      const auto points=world.corners(face,time,.056);if(!camera.in_front(points))continue;
      auto corners=camera.project(points);
      if(std::any_of(corners.begin(),corners.end(),[&](auto p){return p.x<0||p.y<0||p.x>=size_.width||p.y>=size_.height;}))continue;
      result.push_back({face,std::move(corners)});
    }
    return result;
  }
  cv::Mat render(const World& world,const Camera& camera,double time) {
    if(blank_)return background_.clone();
    // Bound expensive texture warps to the union of the target's projected
    // surfaces. The full camera background is copied only once per frame.
    std::vector<std::vector<Surface>> instants;cv::Rect region;
    for(int sample=0;sample<samples_;++sample) {
      const double dt=samples_==1?0:exposure_*(static_cast<double>(sample)/(samples_-1)-.5);
      auto surfaces=project_surfaces(world,camera,time+dt);
      for(const auto& s:surfaces)region=region.empty()?s.bounds:(region|s.bounds);
      instants.push_back(std::move(surfaces));
    }
    cv::Mat output=background_.clone();region&=cv::Rect({0,0},size_);
    if(region.empty())return output;
    cv::Mat accumulated(region.size(),CV_32FC3,cv::Scalar(0,0,0));
    for(const auto& surfaces:instants) {
      cv::Mat image(region.size(),CV_8UC3,cv::Scalar(65,65,65));
      const cv::Mat shift=(cv::Mat_<double>(3,3)<<1,0,-region.x,0,1,-region.y,0,0,1);
      for(const auto& s:surfaces) {
        cv::Mat warped,mask;const cv::Mat transform=shift*s.transform;
        cv::warpPerspective(texture_,warped,transform,region.size(),cv::INTER_LINEAR);
        cv::warpPerspective(alpha_,mask,transform,region.size(),cv::INTER_NEAREST);
        warped.copyTo(image,mask);
      }
      cv::Mat floating;image.convertTo(floating,CV_32FC3);accumulated+=floating/samples_;
    }
    if(blur_>0)cv::GaussianBlur(accumulated,accumulated,{0,0},blur_);
    accumulated*=brightness_;
    if(noise_>0) {
      std::normal_distribution<float> normal(0,static_cast<float>(noise_));
      for(int row=0;row<accumulated.rows;++row)for(int col=0;col<accumulated.cols;++col)
        for(int channel=0;channel<3;++channel)accumulated.at<cv::Vec3f>(row,col)[channel]+=normal(random_);
    }
    accumulated.convertTo(output(region),CV_8UC3);return output;
  }
 private:
  struct Surface {cv::Mat transform;cv::Rect bounds;double depth;};
  std::vector<Surface> project_surfaces(const World& world,const Camera& camera,double time) const {
    static const std::vector<cv::Point2f> anchors{{64,112},{448,112},{448,272},{64,272}};
    static const std::vector<cv::Point2f> extent{{0,0},{511,0},{511,383},{0,383}};
    std::vector<Surface> result;
    for(int face=0;face<4;++face) {
      const auto plate=world.plate(face,time);
      if(plate.normal.dot(camera.origin-plate.center)<=0)continue;
      const auto points=world.corners(face,time,.056);if(!camera.in_front(points))continue;
      const auto corners=camera.project(points);
      if(std::abs(cv::contourArea(corners))<.5)continue;
      auto transform=cv::getPerspectiveTransform(anchors,corners);std::vector<cv::Point2f> outside;
      cv::perspectiveTransform(extent,outside,transform);
      const auto bounds=cv::boundingRect(outside)&cv::Rect({0,0},size_);
      if(!bounds.empty())result.push_back({transform,bounds,(plate.center-camera.origin).norm()});
    }
    std::sort(result.begin(),result.end(),[](const auto&a,const auto&b){return a.depth>b.depth;});return result;
  }
  cv::Size size_;cv::Mat texture_,alpha_,background_;
  std::mt19937 random_;double exposure_=0,brightness_=1,blur_=0,noise_=0;int samples_=1,number_=4;
  bool blank_=false;
};
}

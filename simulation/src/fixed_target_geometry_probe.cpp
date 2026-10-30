// Offline metrology only. Truth matching and oracle ROI ablations are explicitly
// evaluator diagnostics; their points/face identities never enter robot control.
#include "fixed_target_renderer.hpp"
#include "tasks/auto_aim/adaptive_corner_refiner.hpp"
#include "tasks/auto_aim/fixed_center_model.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolos/yolo26.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <numeric>

using Json=nlohmann::json;
static auto stamp(double t) {
  return std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(t)));
}
static Json vec(const Eigen::Vector3d& p){return {p.x(),p.y(),p.z()};}
static Json points(const std::vector<cv::Point2f>& p) {
  Json result=Json::array();for(const auto& v:p)result.push_back({v.x,v.y});return result;
}
static auto_aim::Armor armor(const std::vector<cv::Point2f>& p,const YAML::Node& yaml) {
  auto_aim::Armor a(11,.99f,cv::boundingRect(p),p,auto_aim::YOLOVersion::YOLO26);
  a.pose_observation_valid=true;a.keypoint_confidence=1;
  a.center_norm={a.center.x/yaml["camera_width"].as<float>(),a.center.y/yaml["camera_height"].as<float>()};
  return a;
}
int main(int argc,char** argv) {
  if(argc!=3){std::cerr<<"usage: fixed_target_geometry_probe CONFIG OUTPUT.json\n";return 2;}
  const std::string config=argv[1];const auto yaml=YAML::LoadFile(config);
  auto_aim::YOLO26 detector(config,false);
  auto_aim::AdaptiveCornerRefiner::Config rc;
  rc.binary_threshold=yaml["threshold"].as<int>(100);
  rc.brightness_invariant_subpixel=yaml["corner_refine_brightness_invariant_subpixel"].as<bool>(false);
  rc.roi_length_scale=yaml["corner_refine_roi_length_scale"].as<double>(rc.roi_length_scale);
  rc.roi_width_scale=yaml["corner_refine_roi_width_scale"].as<double>(rc.roi_width_scale);
  rc.max_center_error_ratio=yaml["corner_refine_max_center_error_ratio"].as<double>(rc.max_center_error_ratio);
  rc.max_angle_error_deg=yaml["corner_refine_max_angle_error_deg"].as<double>(rc.max_angle_error_deg);
  rc.max_pair_angle_error_deg=yaml["corner_refine_max_pair_angle_error_deg"].as<double>(rc.max_pair_angle_error_deg);
  rc.min_pair_length_ratio=yaml["corner_refine_min_pair_length_ratio"].as<double>(rc.min_pair_length_ratio);
  auto_aim::AdaptiveCornerRefiner refiner(rc);
  Json results=Json::array();
  for(const std::string mode:{"ideal","refiner_clean","refiner_dark","refiner_exposure","refiner_blur",
      "refiner_degraded","yolo_clean","yolo_degraded"}) {
    auto_aim::Solver solver(config);fixed_sim::World world;
    const auto pose=Eigen::Quaterniond::FromTwoVectors(solver.R_camera2gimbal().col(2),world.center.normalized());
    solver.set_R_gimbal2world_direct(pose);fixed_sim::Camera camera(yaml,pose.toRotationMatrix());
    auto_aim::Tracker tracker(config,solver);tracker.set_association_diagnostics_enabled(false);
    auto_aim::FixedCenterModel model,oracle_model;
    Json settings=Json::object();
    if(mode=="refiner_dark")settings={{"brightness",.65}};
    if(mode=="refiner_exposure")settings={{"exposure_s",.006},{"exposure_samples",5}};
    if(mode=="refiner_blur")settings={{"lens_blur_sigma_px",.6}};
    if(mode.find("degraded")!=std::string::npos)settings={{"exposure_s",.006},{"exposure_samples",5},
      {"brightness",.65},{"lens_blur_sigma_px",.6},{"sensor_noise_stddev",2.}};
    fixed_sim::Renderer renderer(settings,{yaml["camera_width"].as<int>(),yaml["camera_height"].as<int>()},3011);
    Json rows=Json::array(),fits=Json::array(),selected=Json::array(),oracle_fits=Json::array();
    for(int frame=0;frame<450;++frame) {
      const double time=1+frame*.02;
      const auto truth=renderer.truth(world,camera,time,55*fixed_sim::pi/180);
      const auto all_truth=renderer.truth(world,camera,time,89.9*fixed_sim::pi/180);
      std::list<auto_aim::Armor> observations;
      cv::Mat image;
      if(mode!="ideal") image=renderer.render(world,camera,time);
      if(mode.rfind("yolo",0)==0) {
        cv::Mat rgb;cv::cvtColor(image,rgb,cv::COLOR_BGR2RGB);
        observations=detector.detect(rgb,frame,true);
      }
      else for(const auto& reference:truth) {
        auto p=reference.corners;
        if(mode!="ideal") {
          const auto refined=refiner.refine(image,{p[0],p[1],p[2],p[3]},
            auto_aim::AdaptiveCornerRefiner::ChannelMode::red_blue_contrast);
          if(!refined)continue;
          p.assign(refined->points.begin(),refined->points.end());
        }
        observations.push_back(armor(p,yaml));
      }
      // Associate for evaluation BEFORE Tracker mutates/sorts observations.
      std::vector<bool> used(all_truth.size());
      for(auto a:observations) {
        int best=-1;double closest=15.;
        for(int i=0;i<static_cast<int>(all_truth.size());++i)if(!used[i]) {
          cv::Point2f c;for(const auto& p:all_truth[i].corners)c+=p*.25f;
          const double d=cv::norm(a.center-c);if(d<closest){best=i;closest=d;}
        }
        if(best<0)continue;used[best]=true;
        const auto& ref=all_truth[best];const auto plate=world.plate(ref.face,time);
        solver.solve(a);
        Json row={{"frame",frame},{"t",time},{"truth_face",ref.face},{"phase_rad",plate.phase},
          {"truth_xyz",vec(plate.center)},{"truth_corners",points(ref.corners)},
          {"measured_corners",points(a.points)},{"pose_valid",a.pose_valid},{"confidence",a.confidence},
          {"within_55deg",plate.normal.dot((camera.origin-plate.center).normalized())>=std::cos(55*fixed_sim::pi/180)}};
        if(a.pose_valid) {
          row["estimated_xyz"]=vec(a.xyz_in_world);row["xyz_error"]=vec(a.xyz_in_world-plate.center);
          row["yaw_raw"]=a.yaw_raw;row["yaw_optimized"]=a.ypr_in_world[0];
          row["raw_rms_px"]=a.reprojection_error_px;row["model_rms_px"]=a.model_reprojection_error_px;
        }
        rows.push_back(std::move(row));
      }
      auto targets=tracker.track(observations,stamp(time));
      auto_aim::FixedCenterModel::Input input;
      input.active=!targets.empty()&&targets.front().armor_num()==4;
      input.context_valid=tracker.identity_confirmed()&&input.active;
      input.epoch=tracker.track_id();input.frame_ns=static_cast<int64_t>(std::llround(time*1e9));
      input.rebuilt=tracker.diagnostics().rebuilt;
      if(!targets.empty())input.seed=targets.front().ekf().x;
      auto oracle_input=input;
      if(input.context_valid&&tracker.measurement_updated())for(const auto& association:tracker.last_associations()) {
        if(association.detection_index<0||association.detection_index>=static_cast<int>(observations.size()))continue;
        const auto& a=*std::next(observations.begin(),association.detection_index);
        if(a.pose_valid) {
          input.observations.push_back({association.face_id,a.xyz_in_world});
          int best=-1;double closest=15.;
          for(int i=0;i<static_cast<int>(all_truth.size());++i) {
            cv::Point2f c;for(const auto& p:all_truth[i].corners)c+=p*.25f;
            const double d=cv::norm(a.center-c);if(d<closest){best=i;closest=d;}
          }
          if(best>=0) {
            const auto p=world.plate(all_truth[best].face,time);
            selected.push_back({{"frame",frame},{"internal_face",association.face_id},
              {"truth_face",all_truth[best].face},{"phase_rad",p.phase},
              {"track_id",input.epoch},{"rebuilt",input.rebuilt},
              {"x_error_m",a.xyz_in_world.x()-p.center.x()},{"confidence",a.confidence}});
            // Shadow ablation only: retain actual association/seed/timestamps,
            // replace the 3D observations to isolate estimator error. Never
            // consumed by the measured model, prediction or control path.
            oracle_input.observations.push_back({association.face_id,p.center});
          }
        }
      }
      const auto fit=model.update(input);
      const auto oracle_fit=oracle_model.update(oracle_input);
      if(oracle_fit.ready)oracle_fits.push_back({{"frame",frame},
        {"center",{oracle_fit.state[0],oracle_fit.state[2]}}});
      if(fit.ready)fits.push_back({{"frame",frame},{"center",{fit.state[0],fit.state[2]}},
        {"radii",{fit.state[8],fit.state[8]+fit.state[9]}},{"omega",fit.state[7]},
        {"heldout_error_rad",fit.heldout_yaw_error_rad}});
    }
    std::cout<<mode<<" measurements="<<rows.size()<<" ready="<<fits.size()<<std::endl;
    results.push_back({{"mode",mode},{"oracle_roi_only",mode.rfind("refiner",0)==0},
      {"rows",rows},{"fits",fits},{"tracker_selected",selected},{"oracle_center_fits",oracle_fits}});
    std::ofstream out(argv[2]);out<<Json({{"schema",1},{"scope","offline geometry diagnosis; oracle endpoints/ROI are ablations only"},
      {"physical_device_opened",false},{"shadow_model_uses_truth_only_for_diagnosis",true},
      {"frames_per_mode",450},{"fps",50},{"target_center_m",{3,0,.03}},{"omega_rad_s",4.56},
      {"experiments",results}}).dump(2)<<'\n';
  }
}

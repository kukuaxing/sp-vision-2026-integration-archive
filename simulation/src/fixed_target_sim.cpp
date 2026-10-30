// File-only, synthetic-keypoint experiment. No device I/O, camera, serial, ROS or
// mission bridge. This is NOT the full src/standard.cpp control loop.
#include "fixed_target_plant.hpp"
#include "fixed_target_camera.hpp"
#include "fixed_target_timing.hpp"
#include "fixed_target_cadence.hpp"
#include "fixed_target_pipeline.hpp"
#include "tasks/auto_aim/fire_pose_guard.hpp"
#include "tasks/auto_aim/frame_admission.hpp"
#ifdef SP_FIXED_TARGET_IMAGE_INPUT
#include "fixed_target_renderer.hpp"
#include "tasks/auto_aim/yolos/yolo26.hpp"
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#endif
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/high_speed_vision.hpp"
#include "tasks/auto_aim/rotation_fire_window.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <functional>
#include <memory>

using Json=nlohmann::json;
using Clock=std::chrono::steady_clock;
static Clock::time_point stamp(double seconds) {
  return Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds)));
}
static int64_t ns(double seconds){return std::chrono::duration_cast<std::chrono::nanoseconds>(stamp(seconds).time_since_epoch()).count();}
static double wrap(double x){return std::remainder(x,2*fixed_sim::pi);}
static double radians(const YAML::Node& config,const char* key,double fallback) {
  return (config[key]?config[key].as<double>():fallback)*fixed_sim::pi/180;
}
using Projection=fixed_sim::Camera;
static Json visual_diagnostic(const auto_aim::FacePredictionResidual::Output& q) {
  return {{"bias_rad",q.bias_rad},{"scatter_rad",q.scatter_rad},{"correction_rad",q.correction_rad},
    {"cycles",q.cycles},{"quality_cycles",q.quality_cycles},{"quality_ready",q.quality_ready},
    {"quality_eligible",q.quality_eligible},{"past_error_bound_rad",q.past_error_bound_rad},
    {"generation",q.generation},{"matched",q.matched},{"unmatched",q.unmatched}};
}
struct Pending { double settle;std::optional<fixed_sim::Hit> hit;std::size_t record; };
struct Launch {double time,speed,yaw_scatter,pitch_scatter;std::size_t record;};

static Json run(const std::string& config_path,const Json& scenario,int direction,double initial_phase,unsigned seed,bool keep_frames) {
  const auto yaml=YAML::LoadFile(config_path);
  const auto& target=scenario.at("target");const auto& sampling=scenario.at("sampling");
  const auto& plant=scenario.at("plant");
  const bool fixed_camera=scenario.value("diagnostics",Json::object()).value("fixed_camera_pose",false);
  fixed_sim::TimingPipeline pipeline(scenario.value("timing_pipeline",Json()));
  fixed_sim::World world;
  const auto center=target.at("center_m").get<std::array<double,3>>();
  world.center={center[0],center[1],center[2]};
  world.radii=target.at("radii_m").get<std::array<double,4>>();
  world.omega=direction*target.at("omega_rad_s").get<double>();world.initial_phase=initial_phase;world.epoch=1;
  world.height_delta=target.at("height_delta_m");world.width=target.at("armor_width_m");
  world.height=target.at("armor_hit_height_m");world.tilt=target.at("armor_pitch_deg").get<double>()*fixed_sim::pi/180;
  world.gravity=plant.at("gravity_mps2");
  if(!(world.center.x()>0 && world.width>0 && world.height>0 && std::abs(world.omega)>0.1))
    throw std::invalid_argument("invalid stationary target dimensions or motion");
  const double fps=sampling.at("fps"),duration=sampling.at("duration_s");
  const auto intervals=sampling.value("capture_intervals_s",std::vector<double>{});
  const bool recorded_cadence=!intervals.empty();
  const auto capture_clock=fixed_sim::capture_offsets(duration,fps,intervals,
    sampling.value("capture_interval_offset",std::size_t{0}));
  const double age=sampling.at("processing_age_s"),speed=plant.at("projectile_speed_mps");
  const double predictor_speed=yaml["xuc_projectile_speed_mps"].as<double>();
  const double delay=plant.at("muzzle_delay_s"),jitter=plant.at("muzzle_delay_jitter_s");
  const double noise=sampling.at("keypoint_noise_px"),visibility=sampling.at("visible_half_angle_deg").get<double>()*fixed_sim::pi/180;
  const double step=plant.at("collision_step_s"),flight_limit=plant.at("flight_timeout_s");
  const double dropout=sampling.value("frame_dropout_probability",0.0);
  const double age_jitter=sampling.value("frame_age_jitter_s",0.0),time_bias=sampling.value("timestamp_bias_s",0.0);
  const auto pixel_bias=sampling.value("keypoint_bias_px",std::array<double,2>{0,0});
  const double pose_yaw_bias=sampling.value("pose_yaw_bias_deg",0.0)*fixed_sim::pi/180;
  const double pose_pitch_bias=sampling.value("pose_pitch_bias_deg",0.0)*fixed_sim::pi/180;
  const double speed_std=plant.value("speed_stddev_mps",0.0);
  const double yaw_std=plant.value("yaw_spread_stddev_deg",0.0)*fixed_sim::pi/180;
  const double pitch_std=plant.value("pitch_spread_stddev_deg",0.0)*fixed_sim::pi/180;
  const double yaw_bias=plant.value("barrel_yaw_bias_deg",0.0)*fixed_sim::pi/180;
  const double pitch_bias=plant.value("barrel_pitch_bias_deg",0.0)*fixed_sim::pi/180;
  const double normal_limit=plant.value("min_normal_speed_mps",0.0);
  if(!(fps>=5 && fps<=500 && duration>0 && duration<=300 && age>=0 && age<.2 &&
       speed>5 && speed<35 && delay>=0 && delay<1 && jitter>=0 && jitter<=.2 && noise>=0 && noise<=10 &&
       dropout>=0 && dropout<1 && age_jitter>=0 && age_jitter<=.1 && std::abs(time_bias)<.2 &&
       speed_std>=0 && speed_std<5 && yaw_std>=0 && yaw_std<.1 && pitch_std>=0 && pitch_std<.1 && normal_limit>=0))
    throw std::invalid_argument("invalid bounded experiment settings");
  // Known, fixed fixture pose: point the virtual camera at the configured
  // shooting location. The real camera has a substantial mounting pitch;
  // identity gimbal pose would place this level target outside its image.
  // This pose is constant for the entire run, never driven by target phase.
  auto_aim::Solver solver(config_path);
  const Eigen::Quaterniond sensor_pose=Eigen::Quaterniond::FromTwoVectors(
    solver.R_camera2gimbal().col(2),world.center.normalized());
  solver.set_R_gimbal2world_direct(sensor_pose);
  auto_aim::Tracker tracker(config_path,solver);tracker.set_association_diagnostics_enabled(false);
  auto_aim::Aimer aimer(config_path);auto_aim::HighSpeedVision vision(yaml);
  if(!vision.enabled)throw std::invalid_argument("experiment requires teammate_prediction");
  auto_aim::RotationFireWindow window;
  Projection projection(yaml,sensor_pose.toRotationMatrix());
  const auto observation=scenario.value("observation",Json::object());
  const bool image_input=observation.value("mode",std::string("keypoints"))=="rendered_yolo26";
  if(observation.value("mode",std::string("keypoints"))!="keypoints"&&!image_input)
    throw std::invalid_argument("unknown observation mode");
  Json image_stats={{"detector_frames",0},{"visible_truth_faces",0},{"matched_faces",0},
    {"correct_label_faces",0},{"usable_pose_faces",0},{"unmatched_detections",0}};
  std::vector<double> detector_times,render_times,corner_errors;
  Json previews=Json::array();
#ifdef SP_FIXED_TARGET_IMAGE_INPUT
  std::unique_ptr<auto_aim::YOLO26> detector;
  std::unique_ptr<fixed_sim::Renderer> renderer;
  if(image_input) {
    detector=std::make_unique<auto_aim::YOLO26>(config_path,false);
    renderer=std::make_unique<fixed_sim::Renderer>(observation,
      cv::Size(yaml["camera_width"].as<int>(),yaml["camera_height"].as<int>()),seed);
  }
#else
  if(image_input)throw std::invalid_argument("use fixed_target_image_sim for actual image inference");
#endif
  fixed_sim::Score score{target.at("hp"),target.at("damage_per_hit"),target.at("ammo")};
  if(score.hp<=0 || score.damage<=0 || score.ammo<=0 || score.ammo>1000)
    throw std::invalid_argument("invalid HP/damage/ammo budget");
  std::mt19937 pixels(seed),timing(seed^0x716ab29u),ballistics(seed^0xf7315cu),drop_random(seed^0x399331u),age_random(seed^0x638ab5u);
  std::normal_distribution<double> pixel_noise(0,1),delay_noise(0,1),scatter(0,1),age_noise(0,1);
  std::uniform_real_distribution<double> drop_uniform(0,1);
  Json frames=Json::array(),shots=Json::array(),sampling_events=Json::array(),pipeline_events=Json::array(),admission_events=Json::array();std::vector<Pending> pending;
  struct CapturePacket {double time;uint64_t id;cv::Mat image;std::list<auto_aim::Armor> armors;Json observed;Eigen::Matrix3d rotation;};
  std::deque<CapturePacket> packet_history;std::optional<CapturePacket> held_replay;
  auto_aim::FrameAdmission admission;
  std::vector<Launch> launches;
  fixed_sim::AngleServo yaw_servo,pitch_servo;
  for(auto* servo:{&yaw_servo,&pitch_servo}) {
    servo->tau=plant.value("servo_time_constant_s",0.0);
    servo->max_rate=plant.value("servo_max_rate_deg_s",120.0)*fixed_sim::pi/180;
    servo->max_acceleration=plant.value("servo_max_acceleration_deg_s2",720.0)*fixed_sim::pi/180;
    if(!(servo->tau>=0 && servo->tau<=1 && servo->max_rate>0 && servo->max_acceleration>0))
      throw std::invalid_argument("invalid bounded servo settings");
  }
  double plant_time=1;int geometric_hits=0,valid_hits=0,dropped_frames=0,pitch_frames=0;
  double next_imu_sample=1.005;
  const auto physical_pose=[&]()->Eigen::Quaterniond {
    return fixed_camera?sensor_pose:Eigen::Quaterniond(
      Eigen::AngleAxisd(yaw_servo.angle,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(-pitch_servo.angle,Eigen::Vector3d::UnitY())*sensor_pose);
  };
  if(pipeline.enabled){pipeline.sample(1.,physical_pose());pipeline.deliver(1.);}
  auto advance_servo=[&](double to){
    if(to<plant_time-1e-9)throw std::logic_error("nonmonotonic physical event clock");
    if(pipeline.enabled)while(next_imu_sample<=to+1e-10) {
      const double dt=std::max(0.,next_imu_sample-plant_time);
      yaw_servo.advance(dt);pitch_servo.advance(dt);plant_time=next_imu_sample;
      pipeline.sample(next_imu_sample,physical_pose());pipeline.deliver(next_imu_sample);
      next_imu_sample+=.005;
    }
    yaw_servo.advance(std::max(0.,to-plant_time));pitch_servo.advance(std::max(0.,to-plant_time));plant_time=to;
  };
  auto advance_plant=[&](double to){
    std::stable_sort(launches.begin(),launches.end(),[](const auto& a,const auto& b){return a.time<b.time;});
    while(!launches.empty() && launches.front().time<=to) {
      const auto launch=launches.front();launches.erase(launches.begin());advance_servo(launch.time);
      fixed_sim::MuzzleSample muzzle{launch.time,yaw_servo.angle+yaw_bias+launch.yaw_scatter,
        pitch_servo.angle+pitch_bias+launch.pitch_scatter,launch.speed};
      const auto projectile=muzzle.projectile();const auto hit=fixed_sim::collide(world,projectile,step,flight_limit);
      const bool valid=hit && hit->normal_speed>=normal_limit;
      const double settle=hit?hit->time:launch.time+flight_limit;
      auto& record=shots[launch.record];record["geometric_hit"]=bool(hit);record["valid_hit"]=valid;
      record["settle_s"]=settle-1;record["world_yaw"]=muzzle.yaw;record["world_elevation"]=muzzle.pitch;
      record["physical_speed_mps"]=muzzle.speed;
      record["muzzle_servo_yaw_rad"]=yaw_servo.angle;
      record["muzzle_servo_elevation_rad"]=pitch_servo.angle;
      record["muzzle_target_yaw_rad"]=yaw_servo.target;
      record["muzzle_target_elevation_rad"]=pitch_servo.target;
      record["truth_hit_face"]=hit?Json(hit->face):Json(nullptr);
      record["normal_speed_mps"]=hit?Json(hit->normal_speed):Json(nullptr);
      record["hit_u_m"]=hit?Json(hit->horizontal):Json(nullptr);record["hit_v_m"]=hit?Json(hit->vertical):Json(nullptr);
      pending.push_back({settle,valid?hit:std::nullopt,launch.record});
    }
    advance_servo(to);
  };
  auto settle_hits=[&](double time){
    std::stable_sort(pending.begin(),pending.end(),[](const auto& a,const auto& b){return a.settle<b.settle;});
    for(auto it=pending.begin();it!=pending.end();) {
      if(it->settle>time)break;
      geometric_hits+=shots[it->record]["geometric_hit"].get<bool>();
      if(it->hit){++valid_hits;if(score.hit() && score.hp==0)shots[it->record]["cleared_hp"]=true;}
      it=pending.erase(it);
    }
  };
  int tracking_frames=0,ready_frames=0,quality_frames=0,eligible_frames=0;
  double last_issue=-1e10,first_ready=-1,first_quality=-1,first_shot=-1,kill_time=-1;
  double center_error_sum=0;int center_error_count=0;
  const double fire_interval=yaml["sentinel_fire_min_interval_ms"].as<double>()/1000;
  const double yaw_limit=radians(yaml,"xuc_gyro_predicted_yaw_tolerance_deg",1.2);
  const bool subframe=scenario.value("controller",Json::object()).value("center_crossing_schedule",false);
  const bool prefer_radius=scenario.value("controller",Json::object()).value("prefer_smaller_radius",false);
  const bool apply_visual_correction=scenario.value("controller",Json::object()).value("apply_visual_residual_correction",false);
  const bool reject_verified_replay=scenario.value("controller",Json::object()).value("reject_verified_replay",false);
  const double center_phase_limit=scenario.value("controller",Json::object()).value("center_phase_limit_deg",0.);
  (void)fixed_sim::centered_opportunity(0.,center_phase_limit); // validate even without observations
  int phase_rejected_frames=0;
  const double time_margin=scenario.value("controller",Json::object()).value("horizontal_time_margin_s",0.);
  if(!std::isfinite(time_margin)||time_margin<0||time_margin>.1)
    throw std::invalid_argument("horizontal time margin must be in [0,100]ms");
  int margin_rejected_frames=0,busy_capture_drops=0,processed_frames=0;
  double last_pipeline_observation_time=0;
  bool qualified_fault_injected=false;
  const double elevation_trim_deg=scenario.value("controller",Json::object()).value("elevation_trim_deg",0.);
  if(!std::isfinite(elevation_trim_deg)||std::abs(elevation_trim_deg)>.5)
    throw std::invalid_argument("elevation trim exceeds 0.5 degree calibration bound");
  // Offline calibration candidate; never read the plant's true delay here.
  const double default_offset=scenario.value("controller",Json::object()).value("prediction_time_offset_s",0.);
  const auto direction_offsets=scenario.value("controller",Json::object()).value("directional_prediction_offsets_s",Json::object());
  const double cw_offset=direction_offsets.value("-1",default_offset),ccw_offset=direction_offsets.value("1",default_offset);
  if(!std::isfinite(default_offset)||std::abs(default_offset)>.05 ||
     !std::isfinite(cw_offset)||std::abs(cw_offset)>.05 || !std::isfinite(ccw_offset)||std::abs(ccw_offset)>.05)
    throw std::invalid_argument("prediction time offset exceeds 50ms calibration bound");
  for(int frame=0;frame+1<static_cast<int>(capture_clock.size());++frame) {
    const double scheduled_capture=1+capture_clock[frame];
    if(recorded_cadence && scheduled_capture<plant_time-1e-9) {
      ++busy_capture_drops;
      sampling_events.push_back({{"index",frame},{"scheduled_s",scheduled_capture-1},{"busy_drop",true}});
      continue;
    }
    const double time=recorded_cadence?scheduled_capture:std::max(scheduled_capture,plant_time);
    if(time-1>=duration)break;
    const double decision=time+std::max(0.,age+age_jitter*age_noise(age_random))+pipeline.value("image_delay_s",time);
    if(recorded_cadence)sampling_events.push_back({{"index",frame},{"scheduled_s",scheduled_capture-1},
      {"busy_drop",false},{"decision_s",decision-1}});
    advance_plant(time);settle_hits(time);
    if(!score.may_issue() && pending.empty() && launches.empty())break;
    // Match standard.cpp: sequence counts processed images, not captures that
    // arrived while busy. Timestamp/measurement gates still handle true gaps.
    ++processed_frames;
    const Eigen::Matrix3d true_pose=fixed_camera?sensor_pose.toRotationMatrix():
      (Eigen::AngleAxisd(yaw_servo.angle,Eigen::Vector3d::UnitZ())*
       Eigen::AngleAxisd(-pitch_servo.angle,Eigen::Vector3d::UnitY())*sensor_pose).toRotationMatrix();
    const Eigen::Quaterniond reported_pose=Eigen::AngleAxisd(pose_yaw_bias,Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(pose_pitch_bias,Eigen::Vector3d::UnitY())*Eigen::Quaterniond(true_pose);
    solver.set_R_gimbal2world_direct(reported_pose);projection.pose(true_pose);
    const bool dropped=drop_uniform(drop_random)<dropout||pipeline.flag("drop_images",time);dropped_frames+=dropped;
    std::list<auto_aim::Armor> observations;
    Json observed=Json::array();
    double image_capture_time=time;uint64_t capture_frame_id=static_cast<uint64_t>(frame)+1;
    const auto select_packet=[&](CapturePacket current) {
      const double lag=pipeline.value("replay_packet_lag_frames",time);
      if(lag<0||lag>7||std::floor(lag)!=lag)throw std::invalid_argument("invalid replay packet lag");
      const bool freeze=pipeline.flag("replay_packet_burst",time);
      if(!freeze)held_replay.reset();
      if(freeze&&!held_replay&&!packet_history.empty())held_replay=packet_history.back();
      auto selected=current;
      if(freeze&&held_replay)selected=*held_replay;
      else if(lag>0&&packet_history.size()>=static_cast<std::size_t>(lag))
        selected=packet_history[packet_history.size()-static_cast<std::size_t>(lag)];
      packet_history.push_back(std::move(current));while(packet_history.size()>8)packet_history.pop_front();
      image_capture_time=selected.time;capture_frame_id=selected.id;return selected;
    };
#ifdef SP_FIXED_TARGET_IMAGE_INPUT
    if(image_input) {
      const auto render_start=Clock::now();auto image=renderer->render(world,projection,time);
      render_times.push_back(std::chrono::duration<double,std::milli>(Clock::now()-render_start).count());
      const auto packet=select_packet({time,capture_frame_id,image,{},Json::array(),true_pose});
      image=packet.image;projection.pose(packet.rotation);
      if(!dropped) {
        cv::Mat rgb;cv::cvtColor(image,rgb,cv::COLOR_BGR2RGB);
        const auto detect_start=Clock::now();observations=detector->detect(rgb,processed_frames-1,true);
        detector_times.push_back(std::chrono::duration<double,std::milli>(Clock::now()-detect_start).count());
        image_stats["detector_frames"]=image_stats["detector_frames"].get<int>()+1;
      }
      // Truth is used only here for evaluation/overlays, never to create,
      // repair, select or label a detection passed into Tracker.
      const auto truth=renderer->truth(world,projection,image_capture_time,visibility);
      image_stats["visible_truth_faces"]=image_stats["visible_truth_faces"].get<int>()+truth.size();
      std::vector<const auto_aim::Armor*> found;for(const auto& a:observations)found.push_back(&a);
      std::vector<bool> used(found.size());
      for(const auto& reference:truth) {
        cv::Point2f center;
        for(const auto& p:reference.corners)center+=p*.25F;
        int best=-1;double distance=std::max(12.,.3*cv::norm(reference.corners[0]-reference.corners[2]));
        for(int i=0;i<static_cast<int>(found.size());++i)if(!used[i]) {
          const double d=cv::norm(found[i]->center-center);if(d<distance){best=i;distance=d;}
        }
        if(best<0)continue;used[best]=true;
        const auto& armor=*found[best];image_stats["matched_faces"]=image_stats["matched_faces"].get<int>()+1;
        const bool correct=armor.color==auto_aim::blue && static_cast<int>(armor.name)==observation.value("number",4)-1;
        if(correct)image_stats["correct_label_faces"]=image_stats["correct_label_faces"].get<int>()+1;
        if(correct&&armor.pose_observation_valid)
          image_stats["usable_pose_faces"]=image_stats["usable_pose_faces"].get<int>()+1;
        if(armor.points.size()==4) {
          double squared=0;for(int i=0;i<4;++i){const auto e=armor.points[i]-reference.corners[i];squared+=e.dot(e);}
          corner_errors.push_back(std::sqrt(squared/4));
        }
      }
      image_stats["unmatched_detections"]=image_stats["unmatched_detections"].get<int>()+
        std::count(used.begin(),used.end(),false);
      if(keep_frames && (frame==0 || frame==static_cast<int>(7.2*fps) || frame==static_cast<int>(15*fps))) {
        image=image.clone(); // Never burn evaluator overlays into a cached raw frame.
        const auto directory=observation.value("preview_directory",std::string("logs/fixed-target-images"));
        std::filesystem::create_directories(directory);
        for(const auto& q:truth)for(int i=0;i<4;++i)cv::line(image,q.corners[i],q.corners[(i+1)%4],{40,220,60},1);
        for(const auto& a:observations)for(int i=0;i<4;++i)cv::line(image,a.points[i],a.points[(i+1)%4],{40,170,255},2);
        cv::Mat small;cv::resize(image,small,{720,540});
        const auto file=directory+"/seed"+std::to_string(seed)+"-dir"+std::to_string(direction)+
          "-phase"+std::to_string(initial_phase)+"-frame"+std::to_string(frame)+".jpg";
        if(!cv::imwrite(file,small))throw std::runtime_error("cannot save image preview");
        previews.push_back({{"time_s",time-1},{"path",file}});
      }
      for(const auto& a:observations){Json points=Json::array();for(const auto&p:a.points)points.push_back({p.x,p.y});observed.push_back(points);}
    }
#endif
    if(!image_input)for(int face=0;face<4;++face) {
      if(dropped)break;
      const auto plate=world.plate(face,time);
      const auto toward=plate.center.normalized();
      // Visibility is computed in the sensor world. Face IDs are not attached
      // to detections: Tracker must establish its own identity/association.
      if(plate.normal.dot(-toward)<std::cos(visibility))continue;
      auto corners=projection.project(world.corners(face,time,target.at("lightbar_length_m")));
      if(std::any_of(corners.begin(),corners.end(),[&](const cv::Point2f& p){
          return p.x<0 || p.y<0 || p.x>=yaml["camera_width"].as<int>() || p.y>=yaml["camera_height"].as<int>();}))continue;
      for(auto& p:corners){p.x+=static_cast<float>(pixel_bias[0]+noise*pixel_noise(pixels));p.y+=static_cast<float>(pixel_bias[1]+noise*pixel_noise(pixels));}
      observations.emplace_back(11,.99F,cv::boundingRect(corners),corners,auto_aim::YOLOVersion::YOLO26);
      auto& armor=observations.back();armor.pose_observation_valid=true;armor.keypoint_confidence=1;
      armor.center_norm={armor.center.x/yaml["camera_width"].as<float>(),armor.center.y/yaml["camera_height"].as<float>()};
      Json points=Json::array();for(const auto& p:corners)points.push_back({p.x,p.y});observed.push_back(points);
    }
    if(!image_input) {
      const auto packet=select_packet({time,capture_frame_id,{},observations,observed,true_pose});
      observations=packet.armors;observed=packet.observed;
    }
    double observation_time=image_capture_time+time_bias+pipeline.value("image_stamp_bias_s",time);
    if(pipeline.flag("repeat_image_stamp",time)&&last_pipeline_observation_time>0)
      observation_time=last_pipeline_observation_time;
    last_pipeline_observation_time=observation_time;
    io::sentinel::PoseEstimate capture_pose,decision_pose;
    bool capture_context=true,pipeline_reset=false;
    if(pipeline.enabled) {
      // As in standard.cpp, query capture pose AFTER inference. Populate only
      // causal IMU packets already delivered by this virtual decision time.
      advance_plant(decision);settle_hits(decision);
      capture_pose=pipeline.capture(observation_time,decision);decision_pose=pipeline.latest(decision);
      capture_context=capture_pose.generation==decision_pose.generation&&capture_pose.boot_id==decision_pose.boot_id&&
        capture_pose.epoch==decision_pose.epoch;
      pipeline_reset=pipeline.consume_reset();
      if(pipeline_reset){tracker.reset();window={};vision.reset_passages();}
      if(capture_pose.valid&&decision_pose.valid&&capture_context)
        solver.set_R_gimbal2world_direct(fixed_sim::TimingPipeline::rotation(capture_pose));
      else observations.clear();
      const bool rejected=reject_verified_replay&&admission.reject_replay({capture_frame_id,
        decision_pose.generation,decision_pose.boot_id,decision_pose.epoch,ns(observation_time),ns(decision),
        capture_pose.valid&&decision_pose.valid&&capture_context&&!pipeline_reset});
      admission_events.push_back({{"delivery_s",time-1},{"capture_s",image_capture_time-1},
        {"observation_s",observation_time-1},{"decision_s",decision-1},{"frame_id",capture_frame_id},
        {"generation",decision_pose.generation},{"boot",decision_pose.boot_id},{"epoch",decision_pose.epoch},
        {"reference_valid",capture_pose.valid&&decision_pose.valid&&capture_context&&!pipeline_reset},
        {"candidate",reject_verified_replay},{"rejected",rejected}});
      if(rejected)continue; // No model/window update, command or passage commit.
    }
    auto targets=tracker.track(observations,stamp(observation_time));
    tracking_frames+=tracker.state()=="tracking";
    auto_aim::FixedCenterModel::Input input;
    input.active=!targets.empty()&&targets.front().armor_num()==4;
    input.context_valid=tracker.identity_confirmed()&&input.active;
    input.epoch=tracker.track_id();input.frame_ns=ns(observation_time);input.rebuilt=tracker.diagnostics().rebuilt;
    if(pipeline.enabled) {
      input.context_valid=input.context_valid&&capture_pose.valid&&decision_pose.valid&&capture_context;
      input.epoch=(pipeline.generation()<<32)^tracker.track_id();
      input.rebuilt=input.rebuilt||pipeline_reset;
    }
    if(!targets.empty())input.seed=targets.front().ekf().x;
    if(input.context_valid && tracker.measurement_updated())for(const auto& association:tracker.last_associations()) {
      if(association.detection_index<0 || association.detection_index>=static_cast<int>(observations.size()))continue;
      const auto& armor=*std::next(observations.begin(),association.detection_index);
      if(armor.pose_valid)input.observations.push_back({association.face_id,armor.xyz_in_world});
    }
    const auto fitted=vision.model.update(input);
    const auto owner=vision.ownership.update({input.frame_ns,input.epoch,fitted.generation,true,
      input.context_valid,input.rebuilt,fitted.ready});
    if(owner.changed)vision.reset_passages();
    const auto fixed=owner.available&&!targets.empty()?targets.front().with_fixed_center_prediction(fitted.state):std::nullopt;
    const bool ready=fixed.has_value();ready_frames+=ready;
    if(ready && first_ready<0)first_ready=time-1;
    vision.observe_phase(input.frame_ns,ready?fitted.state[6]:NAN,ready);
    auto_aim::AimPredictionTrace trace;io::Command command{false,false,0,0};
    const double prediction_offset=fixed_sim::directional_time_offset(
      ready?fitted.state[7]:NAN,default_offset,cw_offset,ccw_offset);
    double center_yaw=NAN,yaw_error=NAN,phase=NAN;
    if(fixed) {
      // Aimer's existing deterministic path adds 5ms internally. Shift only
      // its read-only prediction copy so the configured processing age is used
      // exactly, without changing production clocks or its original filter.
      auto aim_input=*fixed;const auto base=stamp(decision-.005+prediction_offset);aim_input.predict(base);
      command=aimer.aim_target(&aim_input,base,predictor_speed,false,&trace,auto_aim::AimGeometry::whole_ekf,true);
      center_yaw=std::atan2(fitted.state[2],fitted.state[0]);
      yaw_error=wrap(command.yaw-center_yaw);
      if(trace.valid)phase=wrap(aimer.debug_aim_point.xyza[3]-center_yaw);
      center_error_sum+=std::hypot(fitted.state[0]-world.center.x(),fitted.state[2]-world.center.y());++center_error_count;
    }
    auto_aim::FacePredictionResidual::Input residual;
    residual.frame_ns=input.frame_ns;residual.epoch=input.epoch;residual.rebuilt=input.rebuilt||owner.changed;
    residual.context_valid=input.context_valid&&ready;residual.high_speed_mode=true;
    residual.observations_valid=tracker.measurement_updated();
    residual.omega_rad_s=ready?fitted.state[7]:NAN;
    residual.center_range_m=ready?std::hypot(fitted.state[0],fitted.state[2]):NAN;
    std::array<int,4> counts{};
    for(const auto& o:input.observations)if(o.face>=0 && o.face<4){
      ++counts[o.face];residual.observations[o.face]=std::array<double,2>{o.xyz[0],o.xyz[1]};}
    for(int face=0;face<4;++face)if(counts[face]!=1)residual.observations[face].reset();
    residual.forecast_valid=ready&&trace.valid&&trace.converged;
    residual.face=trace.face_id;residual.prediction_ns=trace.prediction_ns;
    residual.raw_yaw_rad=command.yaw;residual.delay_s=trace.delay_s+prediction_offset;residual.speed_mps=predictor_speed;
    residual.forecast_alignment_rad=phase;
    const bool retain_context=scenario.value("controller",Json::object()).value("retain_short_loss_context",true);
    const auto quality=retain_context?vision.update_residual(residual,input.context_valid&&owner.fixed):vision.residual.update(residual);
    if(apply_visual_correction) {
      // Match standard.cpp: retain RAW forecasts for learning, reset only the
      // opportunity evidence when the learned correction model changes, then
      // apply the bounded correction to the predicted fire yaw. No impact or
      // future truth is observed here; only already-matured visual passages.
      if(!auto_aim::FacePredictionResidual::same_model(quality,vision.previous_residual))window={};
      vision.previous_residual=quality;
      if(command.control) {
        command.yaw=wrap(command.yaw+quality.correction_rad);
        yaw_error=wrap(command.yaw-center_yaw);
      }
    }
    const bool quality_ok=vision.quality_allows(quality);quality_frames+=quality_ok;
    if(quality_ok && first_quality<0)first_quality=time-1;
    window.update(residual.forecast_valid&&tracker.measurement_updated(),input.rebuilt,
      tracker.track_id(),1,3,1,4,trace.face_id,false,
      aimer.debug_aim_point.valid?aimer.debug_aim_point.xyza[3]:NAN,yaw_error,stamp(time),
      yaml["xuc_gyro_fire_min_consecutive_frames"].as<int>(),
      radians(yaml,"xuc_gyro_candidate_max_phase_step_deg",20),yaw_limit,processed_frames,phase,
      ready?fitted.state[7]:NAN,yaml["comming_angle"].as<double>()/57.3,yaml["leaving_angle"].as<double>()/57.3);
    const auto passage=vision.passage(trace.face_id,ready?fitted.state[7]:NAN,trace.prediction_ns,center_yaw);
    advance_plant(decision);settle_hits(decision);
    const double elevation=-command.pitch-yaml["pitch_offset"].as<double>()/57.3+elevation_trim_deg*fixed_sim::pi/180;
    const double tracking_pitch=trace.valid?pitch_servo.angle-elevation:NAN;
    // The virtual encoder supplies current servo angle, never target truth.
    // This calibrated fixture uses encoder pitch error for the residual too;
    // the real standard.cpp image/ballistic residual chain is not reproduced.
    const auto pitch_gate=vision.pitch.update({input.frame_ns,input.epoch,true,
      ready&&trace.valid&&tracker.measurement_updated(),tracking_pitch,tracking_pitch});
    pitch_frames+=pitch_gate.open;
    // A subframe deadline must precede the next capture. It cannot retain a
    // permission across an unseen new observation or manufacture a past shot.
    const double next_capture=1+capture_clock[frame+1];
    const double horizon=std::clamp(next_capture-decision-1e-6,0.,std::min(.05,1/fps));
    const auto wait=subframe?fixed_sim::center_crossing_wait(phase,ready?fitted.state[7]:NAN,horizon):std::nullopt;
    const bool radius_gate=!prefer_radius||(ready&&fixed_sim::preferred_radius_face(
      trace.face_id,fitted.state[8],fitted.state[8]+fitted.state[9]));
    const bool prior_gate=radius_gate&&ready&&command.control&&trace.valid&&trace.converged&&quality_ok&&pitch_gate.open&&
      std::abs(wrap(yaw_servo.angle-center_yaw))<=yaw_limit&&
      tracker.measurement_updated()&&tracker.identity_confirmed()&&std::abs(yaw_error)<=yaw_limit&&
      window.two_frames_ready&&(wait.has_value()||window.current_frame_is_better||window.exiting_selection_next_step)&&
      vision.passage_available(trace.face_id,passage);
    const bool centered=fixed_sim::centered_opportunity(phase,center_phase_limit);
    phase_rejected_frames+=prior_gate&&!centered;
    const auto headroom=time_margin>0&&ready?fixed_sim::horizontal_time_headroom(phase,fitted.state[7],
      fitted.state[8],fitted.state[8]+fitted.state[9],std::hypot(fitted.state[0],fitted.state[2]),
      wrap(yaw_servo.angle-center_yaw),.135,.2*fixed_sim::pi/180):std::nullopt;
    const bool margin_ok=time_margin==0||(headroom&&*headroom>=time_margin);
    margin_rejected_frames+=prior_gate&&!margin_ok;
    bool gate=prior_gate&&centered&&margin_ok;
    eligible_frames+=gate;
    double issue_time=decision+wait.value_or(0.)+pipeline.value("post_wait_s",time);
    bool qualified_fault=false;
    if(pipeline.enabled&&gate&&!qualified_fault_injected&&
      (pipeline.flag("qualified_reference_change",time)||pipeline.flag("qualified_image_expiry",time))) {
      qualified_fault_injected=qualified_fault=true;
      if(pipeline.flag("qualified_reference_change",time)){pipeline.change_reference();issue_time+=.040;}
      else issue_time+=.120;
    }
    bool pipeline_final_allowed=true,image_fresh=true;
    int final_pose_reason=0;
    io::sentinel::PoseEstimate transmit_pose;
    if(pipeline.enabled) {
      advance_plant(issue_time);settle_hits(issue_time);transmit_pose=pipeline.latest(issue_time);
      const auto relative=(fixed_sim::TimingPipeline::rotation(transmit_pose)*sensor_pose.conjugate()).toRotationMatrix();
      const double measured_yaw=std::atan2(relative(1,0),relative(0,0));
      const double measured_pitch=std::asin(std::clamp(relative(2,0),-1.,1.));
      const auto guard=auto_aim::fire_pose_guard(decision_pose,transmit_pose,measured_yaw,center_yaw,yaw_limit,
        measured_pitch-elevation,radians(yaml,"xuc_fire_pitch_tolerance_deg",.4));
      final_pose_reason=static_cast<int>(guard.reason);
      image_fresh=ns(issue_time)>=ns(observation_time)&&ns(issue_time)-ns(observation_time)<=io::sentinel::PoseHistory::max_age_ns;
      pipeline_final_allowed=capture_context&&capture_pose.valid&&guard.allowed&&image_fresh;
      gate=gate&&pipeline_final_allowed;
    }
    const bool fires=gate&&score.may_issue()&&issue_time-last_issue>=fire_interval;
    if(pipeline.enabled)pipeline_events.push_back({{"capture_s",image_capture_time-1},{"delivery_s",time-1},
      {"frame_id",capture_frame_id},{"observation_s",observation_time-1},
      {"decision_s",decision-1},{"tx_check_s",issue_time-1},{"capture_valid",capture_pose.valid},
      {"capture_status",static_cast<int>(capture_pose.status)},{"capture_context",capture_context},
      {"decision_pose_valid",decision_pose.valid},{"transmit_pose_valid",transmit_pose.valid},
      {"decision_generation",decision_pose.generation},{"transmit_generation",transmit_pose.generation},
      {"reset",pipeline_reset},{"measurement_updated",tracker.measurement_updated()},
      {"track_id",tracker.track_id()},{"fixed_ready",ready},{"residual_generation",quality.generation},
      {"correction_rad",quality.correction_rad},{"residual_cycles",quality.cycles},
      {"prior_gate",prior_gate},{"image_fresh",image_fresh},{"pose_reject",final_pose_reason},
      {"qualified_fault",qualified_fault},{"final_allowed",pipeline_final_allowed},{"fire",fires},{"selected_face",trace.face_id},{"passage",passage}});
    if(fires) {
      // Ideal center-lock servo. Hardware boresight/PID/6deg Aimer trim is not
      // a simulated physical bias: convert its pitch back into world elevation.
      const double actual_delay=std::max(0.,delay+jitter*delay_noise(timing));
      const double muzzle_time=issue_time+actual_delay;
      score.issue();last_issue=issue_time;if(first_shot<0)first_shot=issue_time-1;
      vision.commit_passage(trace.face_id,passage);
      shots.push_back({{"id",score.issued},{"issue_s",issue_time-1},{"muzzle_s",muzzle_time-1},
        {"capture_s",image_capture_time-1},{"decision_s",decision-1},{"subframe_wait_s",wait.value_or(0.)},
        {"scheduled_capture_s",scheduled_capture-1},{"next_capture_s",next_capture-1},
        {"processed_frame_sequence",processed_frames},
        {"horizontal_time_margin_s",time_margin},{"horizontal_time_headroom_s",headroom?Json(*headroom):Json(nullptr)},
        {"estimated_phase_rad",phase},{"estimated_omega_rad_s",ready?fitted.state[7]:NAN},
        {"prediction_time_offset_s",prediction_offset},
        {"center_phase_limit_deg",center_phase_limit},
        {"visual_residual",visual_diagnostic(quality)},
        {"issue_desired_yaw_rad",center_yaw},{"issue_desired_elevation_rad",elevation},
        {"issue_servo_yaw_rad",yaw_servo.angle},{"issue_servo_elevation_rad",pitch_servo.angle},
        {"estimated_center_world_m",{fitted.state[0],fitted.state[2],fitted.state[4]}},
        {"model_aim_yaw_rad",command.yaw},{"model_point_world_m",{aimer.debug_aim_point.xyza[0],
           aimer.debug_aim_point.xyza[1],aimer.debug_aim_point.xyza[2]}},
        {"estimated_radius0_m",fitted.state[8]},{"estimated_radius1_m",fitted.state[8]+fitted.state[9]},
        {"prefer_smaller_radius",prefer_radius},{"elevation_trim_deg",elevation_trim_deg},
        {"visual_correction_applied",apply_visual_correction},
        {"visual_residual_correction_rad",quality.correction_rad},
        {"selected_model_face",trace.face_id},{"selected_passage",passage},{"model_impact_s",trace.prediction_ns*1e-9-1},
        {"geometric_hit",false},{"valid_hit",false},{"cleared_hp",false}});
      if(pipeline.enabled)shots.back()["pipeline_generation"]=transmit_pose.generation;
      launches.push_back({muzzle_time,std::clamp(speed+speed_std*scatter(ballistics),5.01,34.99),
        yaw_std*scatter(ballistics),pitch_std*scatter(ballistics),shots.size()-1});
    }
    if(trace.valid){yaw_servo.command(center_yaw);pitch_servo.command(elevation);}
    if(keep_frames && frame%5==0)frames.push_back({{"t",time-1},{"phase",world.initial_phase+world.omega*(time-world.epoch)},
      {"observed_corners",observed},{"tracker",tracker.state()},{"fixed_ready",ready},{"quality",quality_ok},
      {"visual_residual",visual_diagnostic(quality)},
      {"hp",score.hp},{"issued",score.issued},{"servo_yaw",yaw_servo.angle},{"servo_pitch",pitch_servo.angle},
      {"estimated_center",ready?Json::array({fitted.state[0],fitted.state[2]}):Json(nullptr)}});
  }
  double finish=plant_time;
  for(const auto& launch:launches)finish=std::max(finish,launch.time+flight_limit);
  advance_plant(finish);settle_hits(finish+flight_limit);
  for(const auto& shot:shots)if(shot["cleared_hp"].get<bool>())kill_time=shot["settle_s"];
  const auto percentile=[](std::vector<double> values,double p)->Json{
    if(values.empty())return nullptr;std::sort(values.begin(),values.end());return values[static_cast<size_t>((values.size()-1)*p)];};
  image_stats["detector_p50_ms"]=percentile(detector_times,.5);image_stats["detector_p95_ms"]=percentile(detector_times,.95);
  image_stats["render_p50_ms"]=percentile(render_times,.5);image_stats["corner_rms_p50_px"]=percentile(corner_errors,.5);
  Json result={{"direction",direction},{"omega_rad_s",world.omega},{"initial_phase_rad",initial_phase},
    {"observation_mode",image_input?"rendered_yolo26":"keypoints"},{"image_stats",image_stats},{"previews",previews},
    {"seed",seed},{"geometric_hits",geometric_hits},{"valid_hits",valid_hits},{"ammo_budget",score.ammo},
    {"budget_hit_fraction",static_cast<double>(valid_hits)/score.ammo},{"dropped_frames",dropped_frames},
    {"pitch_ready_frames",pitch_frames},
    {"predictor_speed_mps",predictor_speed},{"physical_speed_mps",speed},
    {"diagnostic_fixed_camera_pose",fixed_camera},
    {"recorded_cadence",recorded_cadence},{"busy_capture_drops",busy_capture_drops},
    {"processed_frames",processed_frames},
    {"sampling_events",sampling_events},{"horizontal_time_margin_s",time_margin},
    {"margin_rejected_frames",margin_rejected_frames},
    {"center_phase_limit_deg",center_phase_limit},{"phase_rejected_frames",phase_rejected_frames},
    {"virtual_sensor_quaternion_wxyz",{sensor_pose.w(),sensor_pose.x(),sensor_pose.y(),sensor_pose.z()}},
    {"issued",score.issued},{"geometric_hits_scored",score.hits},{"hp",score.hp},
    {"first_fixed_ready_s",first_ready},{"first_quality_ready_s",first_quality},{"first_issue_s",first_shot},
    {"kill_s",kill_time},{"tracking_frames",tracking_frames},{"fixed_ready_frames",ready_frames},
    {"quality_ready_frames",quality_frames},{"eligible_frames",eligible_frames},
    {"mean_estimated_center_error_m",center_error_count?Json(center_error_sum/center_error_count):Json(nullptr)},
    {"shots",shots},{"frames",frames}};
  if(pipeline.enabled)result["timing_pipeline"]={{"events",pipeline_events},
    {"admission_events",admission_events},
    {"delivered_imu_packets",pipeline.delivered_packets},{"discarded_old_packets",pipeline.discarded_old_packets},
    {"note","opt-in arrival-clock PoseHistory + TX helper; not full standard.cpp/image reprojection/UART/MCU"}};
  return result;
}
int main(int argc,char**argv) {
 try {
  if(argc!=4)throw std::invalid_argument("usage: fixed_target_sim CONFIG SCENARIO_JSON OUTPUT_JSON");
  cv::setNumThreads(1);Json scenario;std::ifstream(argv[2])>>scenario;
  const bool rendered=scenario.value("observation",Json::object()).value("mode",std::string("keypoints"))=="rendered_yolo26";
  Json result={{"schema",2},{"physical_device_opened",false},{"scenario",scenario},
    {"scope",rendered?"rendered RGB -> actual production YOLO26/refiner -> geometry/tracking/prediction helpers; no truth fallback, not full standard.cpp/MCU":
      "synthetic 2D observations -> production geometry/prediction helpers; optional servo and uncertainty; full ammo budget without referee feedback; not full standard.cpp/YOLO/MCU"},
    {"truth_supplied_to_predictor",false},{"scoring","ideal geometric center-plane hit, not official referee validation"},
    {"runs",Json::array()}};
  const auto seeds=scenario["sampling"].value("seeds",std::vector<unsigned>{scenario["sampling"].at("seed").get<unsigned>()});
  if(seeds.empty())throw std::invalid_argument("empty seeds");
  for(unsigned seed:seeds)for(int direction:scenario.at("target").at("directions"))for(double phase:scenario.at("target").at("initial_phases_rad"))
    result["runs"].push_back(run(argv[1],scenario,direction,phase,seed,seed==seeds.front()));
  std::ofstream output(argv[3]);output<<result.dump(2)<<'\n';if(!output)throw std::runtime_error("cannot save result");
  for(const auto& r:result["runs"])std::cout<<"direction="<<r["direction"]<<" phase="<<r["initial_phase_rad"]
    <<" shots="<<r["issued"]<<" geometric_hits="<<r["geometric_hits"]<<" valid_hits="<<r["valid_hits"]<<" hp="<<r["hp"]
    <<" ready_s="<<r["first_fixed_ready_s"]<<" quality_s="<<r["first_quality_ready_s"]<<'\n';
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}

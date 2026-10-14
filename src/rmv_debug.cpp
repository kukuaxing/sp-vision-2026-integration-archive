#include "rmv_debug.hpp"
#include "io/rmv/debug_link.hpp"
#include "io/camera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/rmv_rotating.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {
using io::rmv::JointGoal;
constexpr double deg=io::rmv::debug_radians_per_degree;
Eigen::Vector3d vector3(const YAML::Node& cfg,const char* key) {
  auto a=cfg[key].as<std::vector<double>>();
  if(a.size()!=3)throw std::runtime_error(std::string(key)+" needs 3 values");
  Eigen::Vector3d v(a[0],a[1],a[2]);
  if(!v.allFinite())throw std::runtime_error(std::string(key)+" is not finite");
  return v;
}
Eigen::Quaterniond quaternion(const io::rmv::Quaternion& q) {return {q[0],q[1],q[2],q[3]};}
std::vector<double> xyz(const Eigen::Vector3d& v){return {v.x(),v.y(),v.z()};}
cv::Point2f keypoint_center(const std::vector<cv::Point2f>& p) {
  if(p.size()!=4)return {};
  const auto a=p[2]-p[0], b=p[3]-p[1], c=p[1]-p[0];
  const double cross=a.x*b.y-a.y*b.x;
  if(std::abs(cross)<1e-6)return (p[0]+p[1]+p[2]+p[3])*.25f;
  return p[0]+a*static_cast<float>((c.x*b.y-c.y*b.x)/cross);
}
}

int run_rmv_shot_test(const std::string& path,unsigned count,double seconds,bool check_only,bool rpm_measurement) {
  try {
    if(!count || count>20 || !std::isfinite(seconds) || seconds<0)
      throw std::runtime_error("Shot test requires 1 to 20 requests and a nonnegative timeout");
    const auto cfg=YAML::LoadFile(path);
    const auto port=cfg["rmv_port"].as<std::string>();
    const auto baud=cfg["rmv_baudrate"].as<unsigned>();
    const auto revision=cfg["rmv_debug_geometry_revision"].as<unsigned>();
    if(!cfg["rmv_debug_enabled"].as<bool>() || port.empty() || baud!=460800 || revision!=1 ||
       cfg["rmv_debug_yaw_axis"].as<std::string>()!="big" ||
       cfg["rmv_debug_max_displacement_deg"].as<double>()!=0)
      throw std::runtime_error("Invalid shot-test commissioning configuration");
    nlohmann::json info={{"run_kind","stationary_bounded_shot_test"},{"request_limit",count},
      {"required_firmware",rpm_measurement?2026100704:2026100703},{"friction_rpm",6500},{"vision_aim",false},
      {"count_basis","MCU feeder cycle completion; not projectile detection"},
      {"wheel_delay_measurement",rpm_measurement},{"wheel_sample_rate_hz",rpm_measurement?200:20},
      {"wheel_measurement_pose_joint_rate_hz",rpm_measurement?25:100},
      {"combined_trace",rpm_measurement},{"measurement_wire_budget_bytes_per_s",rpm_measurement?28564:27004},
      {"drop_evidence","wheel-contact proxy, not projectile exit"},
      {"timeout_s",seconds?seconds:180.}};
    if(check_only){std::cout<<info.dump(2)<<std::endl;return 0;}
    auto root=std::filesystem::absolute(path).parent_path().parent_path();
    if(!std::filesystem::is_regular_file(root/"configs/standard.yaml"))
      throw std::runtime_error("Cannot locate project log directory");
    const auto folder=root/"logs/rmv-shot-test"/std::to_string(io::rmv::debug_now_us());
    std::filesystem::create_directories(folder);
    std::ofstream log(folder/"events.jsonl");
    if(!log)throw std::runtime_error("Cannot open shot test log");
    log<<info.dump()<<'\n';log.flush();
    std::cout<<"Shot test log: "<<(folder/"events.jsonl").string()
      <<"\n保持当前朝向；双上预转，就绪后上位机自动执行本轮单发测试，无需操作摇杆。Ctrl+C 中止。"<<std::endl;
    io::rmv::DebugLink link(port,baud,true,revision,0,true,true,count,rpm_measurement);
    const auto drain_measurement=[&](){
      for(const auto& r:link.take_wheel_measurement()){
        nlohmann::json row={{"host_us",r.host_us},{"mcu_us",r.mcu_us},{"request_id",r.request_id}};
        if(r.kind==io::rmv::WheelMeasurementRecord::wheel){
          row["event"]="wheel_sample";row["frame_seq"]=r.seq;
          row["valid_flags"]=r.valid;row["rpm"]=r.rpm;
          if(r.has_feeder_trace){
            row["source"]="combined_trace_0704";row["feedback_age_us"]=r.feedback_age_us;
            row["feeder"]={{"position_counts",r.feeder_position},{"target_counts",r.feeder_target},
              {"remaining_counts",r.feeder_remaining},{"command_rpm",r.command_rpm},
              {"command_current_native",r.command_current},{"feedback_current_native",r.feedback_current},
              {"state",r.feeder_state},{"motor_mode",r.motor_mode},{"fault",r.feeder_fault},
              {"flags",r.feeder_flags},{"completed",r.feeder_completed}};
          }
        }else if(r.kind==io::rmv::WheelMeasurementRecord::shot){
          row["event"]="shot_event";row["shot_event"]=r.event;row["evidence"]=r.evidence;
          row["reason"]=r.reason;row["aim_seq"]=r.aim_seq;
        }else{
          row["event"]="command_issued";row["flush_done_host_us"]=r.flush_done_host_us;
          row["clock_uncertainty_us"]=r.clock_uncertainty_us;
        }
        log<<row.dump()<<'\n';
      }
    };
    tools::Exiter exiter;
    const auto started=io::rmv::debug_now_us();
    std::uint64_t last_print=0;
    std::optional<JointGoal> hold;
    std::uint32_t event_id=0;std::uint8_t event_kind=255;
    unsigned completed=0;
    std::string result="interrupted";
    bool success=false;
    while(!exiter.exit()) {
      const auto now=io::rmv::debug_now_us();
      auto s=link.snapshot(now);
      if(rpm_measurement)drain_measurement();
      if(!s.error.empty()){result=s.error;break;}
      if(!s.batch.error.empty()){result=s.batch.error;break;}
      if((now-started)*1e-6>=(seconds?seconds:180.)){result="test timeout";break;}
      if(s.batch.issued && (s.faults || s.owner==1 || s.mode!=2)) {
        result="physical control mode withdrawn";break;
      }
      if(!rpm_measurement && s.event_request_id && (s.event_request_id!=event_id || s.shot_event!=event_kind)) {
        nlohmann::json row={{"host_us",now},{"request_id",s.event_request_id},
          {"event",s.shot_event},{"reason",s.shot_reason},{"mcu_us",s.shot_event_mcu_us},
          {"issued",s.batch.issued},{"completed",s.batch.completed}};
        log<<row.dump()<<'\n';log.flush();event_id=s.event_request_id;event_kind=s.shot_event;
      }
      if(s.batch.completed!=completed) {
        completed=s.batch.completed;
        std::cout<<"拨弹完成 "<<completed<<"/"<<count<<std::endl;
      }
      if(s.batch.done()){success=true;result="completed";break;}
      // Send a held target as soon as control is ready. MCU feeder re-arming
      // needs valid AIM; waiting for a quantised speed signal creates a deadlock.
      if(!hold && s.ready){hold=s.joints;hold->trajectory=true;}
      bool submitted=false;
      if(hold && s.ready) {
        submitted=link.submit(*hold);
        if(auto id=link.fire_held_once(*hold)) {
          const nlohmann::json row={{"host_us",now},{"request_id",*id},{"event","request_sent"},
            {"hold_joint_rad",{hold->yaw,hold->pitch}}};
          log<<row.dump()<<'\n';log.flush();
          std::cout<<"单发请求 "<<*id<<"/"<<count<<std::endl;
        }
      }
      if(now-last_print>=2000000) {
        const nlohmann::json row={{"event","waiting_state"},{"host_us",now},
          {"firmware_version",s.firmware},
          {"control_reason",s.ready_reason},{"mode",s.mode},{"owner",s.owner},{"faults",s.faults},
          {"hold_target_established",hold.has_value()},{"aim_submitted",submitted},
          {"received_aim_seq",s.received},{"applied_aim_seq",s.applied},
          {"fire_permission",s.fire_permission},{"shoot_valid_flags",s.shoot_valid},
          {"shoot_state_fresh",s.shoot_state_fresh},{"shooter_rpm",s.shooter_rpm},
          {"joint_velocity_rad_s",s.joint_velocity},{"held_position_settled",s.held_position_settled},
          {"issued",s.batch.issued},{"completed",s.batch.completed}};
        log<<row.dump()<<'\n';log.flush();
        std::cout<<"等待/执行: "<<s.ready_reason<<", 发射许可="<<s.fire_permission
          <<", 摩擦轮="<<s.shooter_rpm[0]<<"/"<<s.shooter_rpm[1]<<" rpm"
          <<", 保持目标="<<hold.has_value()<<", AIM采用="<<s.applied
          <<", 角度停稳="<<s.held_position_settled<<", 反馈有效位="<<s.shoot_valid<<std::endl;
        last_print=now;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const bool restored=!rpm_measurement || link.finish_wheel_measurement();
    if(rpm_measurement)drain_measurement();
    log.flush();log.close();
    const auto s=link.snapshot(io::rmv::debug_now_us());
    nlohmann::json summary={{"result",result},{"request_limit",count},
      {"issued",s.batch.issued},{"feeder_completed",s.batch.completed},{"success",success},
      {"projectiles_detected",nullptr}};
    summary["default_telemetry_restored"]=restored;
    std::ofstream(folder/"summary.json")<<summary.dump(2)<<'\n';
    std::cout<<summary.dump()<<std::endl;
    if(rpm_measurement){
      // Spawn only the offline report, with structured argv and no shell.
      const auto script=(root/"scripts/report_rmv_rpm_delay.py").string();
      const auto input=(folder/"events.jsonl").string();
      const auto output=(folder/"rpm-delay-report.json").string();
      char* args[]={const_cast<char*>("python3"),const_cast<char*>(script.c_str()),
        const_cast<char*>(input.c_str()),const_cast<char*>("--output"),
        const_cast<char*>(output.c_str()),nullptr};
      pid_t child;
      const int spawned=posix_spawnp(&child,"python3",nullptr,nullptr,args,environ);
      int status=0;
      if(spawned || waitpid(child,&status,0)<0 || !WIFEXITED(status) || WEXITSTATUS(status))
        std::cerr<<"轮速报告未成功生成；原始events.jsonl已保留"<<std::endl;
    }
    return success?0:1; // DebugLink destruction sends STOP, never retries a shot.
  }catch(const std::exception& e){std::cerr<<"RMV shot test: "<<e.what()<<std::endl;return 1;}
}

int run_rmv_debug(const std::string& path,bool follow,double seconds,bool check_only,bool rotating,bool fire) {
  try {
    const auto cfg=YAML::LoadFile(path);
    if(fire){rotating=true;follow=true;}
    double projectile_speed=cfg["rmv_projectile_speed_mps"]?cfg["rmv_projectile_speed_mps"].as<double>():0;
    std::string speed_source=projectile_speed>0?"explicit_config_not_measured":"unset";
    if(projectile_speed==0 && cfg["rmv_friction_wheel_diameter_m"] && cfg["rmv_motor_to_wheel_ratio"] &&
       cfg["rmv_friction_wheel_diameter_m"].as<double>()>0 && cfg["rmv_motor_to_wheel_ratio"].as<double>()>0){
      projectile_speed=io::rmv::estimate_projectile_speed(cfg["rmv_friction_wheel_diameter_m"].as<double>(),
        cfg["rmv_friction_motor_rpm"].as<double>(),cfg["rmv_motor_to_wheel_ratio"].as<double>(),
        cfg["rmv_wheel_transfer_efficiency"].as<double>());
      speed_source="wheel_surface_speed_times_assumed_efficiency";
    }
    auto_aim::RmvRotatingAim rotation(cfg,projectile_speed);
    const unsigned required_frames=std::clamp(cfg["rmv_fire_min_consecutive_frames"]?cfg["rmv_fire_min_consecutive_frames"].as<unsigned>():
      (cfg["xuc_gyro_fire_min_consecutive_frames"]?cfg["xuc_gyro_fire_min_consecutive_frames"].as<unsigned>():2),1u,10u);
    if(fire && projectile_speed<=0)throw std::runtime_error("Wheel diameter/gear ratio or projectile speed must be supplied before firing");
    if(!cfg["rmv_debug_enabled"] || !cfg["rmv_debug_enabled"].as<bool>())
      throw std::runtime_error("RMV commissioning is not enabled in standard.yaml");
    if(!std::isfinite(seconds) || seconds<0)throw std::runtime_error("Invalid duration");
    const auto port=cfg["rmv_port"].as<std::string>();
    const auto baud=cfg["rmv_baudrate"].as<unsigned>();
    const auto revision=cfg["rmv_debug_geometry_revision"].as<unsigned>();
    const double limit=cfg["rmv_debug_max_displacement_deg"].as<double>();
    if(!cfg["rmv_debug_yaw_axis"] || cfg["rmv_debug_yaw_axis"].as<std::string>()!="big")
      throw std::runtime_error("Current debug firmware requires rmv_debug_yaw_axis: big");
    io::rmv::debug_limit_radians(limit); // Also checked by --rmv-check, before devices.
    if(port.empty() || baud!=460800 || revision!=1)
      throw std::runtime_error("Invalid commissioning port/baud/geometry/displacement");
    auto_aim::Solver solver(path); // Static geometry only; no camera/serial/model is opened.
    auto intrinsic=cfg["camera_matrix"].as<std::vector<double>>();
    auto distortion=cfg["distort_coeffs"].as<std::vector<double>>();
    const cv::Mat K(3,3,CV_64F,intrinsic.data()),D(1,5,CV_64F,distortion.data());
    const auto project=[&](const Eigen::Vector3d& point)->std::optional<cv::Point2f>{
      if(!point.allFinite() || point.z()<=0)return {};
      std::vector<cv::Point3d> objects{{point.x(),point.y(),point.z()}};
      std::vector<cv::Point2d> pixels;
      cv::projectPoints(objects,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),K,D,pixels);
      if(!std::isfinite(pixels[0].x) || !std::isfinite(pixels[0].y))return {};
      return cv::Point2f(pixels[0]);
    };
    const auto muzzle=vector3(cfg,"t_muzzle2gimbal");
    const auto nominal_bore=vector3(cfg,"gun_forward_in_gimbal");
    const double pitch_trim=cfg["rmv_ballistic_pitch_trim_deg"]?cfg["rmv_ballistic_pitch_trim_deg"].as<double>():0;
    const auto trimmed_bore=io::rmv::bore_for_pitch_trim(nominal_bore,pitch_trim);
    const bool trim_applied=rotating && projectile_speed>0;
    const Eigen::Vector3d bore=trim_applied?trimmed_bore:nominal_bore;
    if(std::abs(nominal_bore.norm()-1)>1e-5 || nominal_bore.x()<=0 ||
       (nominal_bore-solver.R_camera2gimbal().col(2)).norm()>1e-5 ||
       std::abs(solver.R_camera2gimbal().determinant()-1)>1e-5 ||
       (solver.R_camera2gimbal().transpose()*solver.R_camera2gimbal()-Eigen::Matrix3d::Identity()).norm()>1e-5)
      throw std::runtime_error("Invalid camera/gun mounting geometry");
    nlohmann::json info={{"protocol","RMV"},{"run_kind",rotating?"four_armor_rotating_target":"joint_line_of_sight_commissioning"},
      {"follow_requested",follow},{"fire_enabled",fire},{"ballistics_enabled",rotating&&projectile_speed>0},
      {"projectile_speed_mps",projectile_speed},{"projectile_speed_source",speed_source},{"projectile_speed_measured",false},
      {"expected_geometry",revision},{"supported_follow_firmwares",{2026100602,2026100603,2026100604,2026100605,2026100701,2026100702,2026100703,2026100704,2026100705,2026100706,2026100707,2026100708}},{"required_fire_firmware",rotating?2026100701:2026100605},{"yaw_axis","big"},
      {"control_chain",rotating?"synced_pose_gyro_joints_target_range_ballistics_centered_mpc_native_servo":"joint_line_of_sight"},
      {"trajectory_control",rotating},{"trajectory_capability_bit",15},{"fire_delay_s",cfg["rmv_fire_delay_s"].as<double>()},
      {"speed_limits_by_firmware_rad_s",{{"2026100602",{0.1,0.08}},{"2026100603",{0.2,0.16}},{"2026100604",{2.0,1.5}},{"2026100605",{2.0,1.5}},{"2026100701",{2.0,1.5}},{"2026100702",{2.0,1.5}},{"2026100703",{2.0,1.5}},{"2026100704",{2.0,1.5}},{"2026100705",{2.0,1.5}},{"2026100706",{2.0,1.5}},{"2026100707",{2.0,1.5}},{"2026100708",{2.0,1.5}}}},
      {"small_yaw_behavior","hold current joint position"},{"relative_limit_enabled",limit>0},
      {"max_displacement_deg",limit},{"t_camera2gimbal_m",xyz(solver.t_camera2gimbal())},
      {"t_muzzle2gimbal_m",xyz(muzzle)},{"gun_forward_G",xyz(nominal_bore)},
      {"ballistic_model_bore_G",xyz(bore)},
      {"ballistic_pitch_trim_deg",pitch_trim},{"ballistic_pitch_trim_applied",trim_applied},
      {"fire_face_max_alignment_deg",cfg["rmv_fire_face_max_alignment_deg"]?cfg["rmv_fire_face_max_alignment_deg"].as<double>():
        (cfg["xuc_event_face_max_alignment_deg"]?cfg["xuc_event_face_max_alignment_deg"].as<double>():15.)},
      {"fire_min_consecutive_frames",required_frames},
      {"pitch_trim_alignment_measured",false}};
    if(check_only){std::cout<<info.dump(2)<<std::endl;return 0;}
    const bool gui=!(cfg["competition_headless"] && cfg["competition_headless"].as<bool>());
    auto root=std::filesystem::absolute(path).parent_path();
    while (!std::filesystem::is_regular_file(root/"configs/standard.yaml")) {
      const auto parent=root.parent_path();
      if(parent==root)throw std::runtime_error("Cannot locate project log directory");
      root=parent;
    }
    const auto folder=root/"logs/rmv-debug"/std::to_string(io::rmv::debug_now_us());
    std::filesystem::create_directories(folder);
    std::ofstream log(folder/"frames.jsonl");
    if(!log)throw std::runtime_error("Cannot open RMV debug log");
    info["config_path"]=std::filesystem::absolute(path).string();
    info["enemy_color"]=cfg["enemy_color"].as<std::string>();
    log<<info.dump()<<'\n';log.flush();
    std::cout<<"RMV log: "<<(folder/"frames.jsonl").string()<<std::endl;
    auto_aim::Planner planner(path);
    io::rmv::DebugLink link(port,baud,follow,revision,limit,fire,rotating&&follow);
    auto_aim::YOLO detector(path,false);
    auto_aim::Tracker tracker(path,solver);
    io::Camera camera(path);
    tools::Exiter exiter;
    const auto started=io::rmv::debug_now_us();
    std::uint64_t last_print=0; unsigned frame=0,consecutive=0;int last_id=-1;bool synchronized=false;
    std::int64_t last_fire_cycle=-1,qualification_cycle=-1;unsigned qualified_frames=0;std::uint64_t last_fire_us=0;
    int last_low_speed_face=-1,qualification_face=-1;
    const double recovery=cfg["xuc_gyro_post_feed_recovery_ms"]?cfg["xuc_gyro_post_feed_recovery_ms"].as<double>()*.001:.55;
    const auto wanted=cfg["enemy_color"].as<std::string>()=="red"?auto_aim::red:auto_aim::blue;
    while(!exiter.exit() && (!seconds || (io::rmv::debug_now_us()-started)<seconds*1000000)) {
      cv::Mat image;std::chrono::steady_clock::time_point timestamp;
      camera.read(image,timestamp);if(image.empty())continue;
      const auto image_us=std::chrono::duration_cast<std::chrono::microseconds>(timestamp.time_since_epoch()).count();
      auto s=link.snapshot(image_us);
      // Wait briefly for the newer IMU endpoint needed for interpolation.
      const auto wait_started=io::rmv::debug_now_us();
      while(!s.image_q && s.error.empty() && io::rmv::debug_now_us()-wait_started<30000) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));s=link.snapshot(image_us);
      }
      if(!s.error.empty())throw std::runtime_error(s.error);
      if(!synchronized && s.active && s.image_q) {
        std::cout<<"[SYNC][RMV] Mounted pose synchronized; "<<(fire?"double-UP enables host single-shot commands":"fire disabled")<<std::endl;
        synchronized=true;
      }
      auto armors=detector.detect(image,frame++);
      unsigned detected_red=0,detected_blue=0;
      for(const auto& a:armors) {
        if(a.name==auto_aim::not_armor)continue;
        if(a.color==auto_aim::red)++detected_red;
        if(a.color==auto_aim::blue)++detected_blue;
      }
      armors.remove_if([&](const auto& a){return a.color!=wanted || a.name==auto_aim::not_armor;});
      std::optional<JointGoal> goal;Eigen::Vector3d target=Eigen::Vector3d::Zero();
      std::optional<cv::Point2f> selected_center,pnp_center,gun_line;
      nlohmann::json selection=nullptr;
      bool target_valid=false;
      std::list<auto_aim::Target> targets;
      if(s.image_q && solver.set_R_gimbal2world_direct(quaternion(*s.image_q))) {
        targets=tracker.track(armors,timestamp);
        const int selected=tracker.primary_detection_index();
        if(selected>=0 && selected<static_cast<int>(armors.size())) {
          auto it=armors.begin();std::advance(it,selected);
          if(it->xyz_in_world.allFinite() && it->xyz_in_world.norm()>.3 && it->xyz_in_world.norm()<20) {
            target=it->xyz_in_world;target_valid=true;
            selected_center=keypoint_center(it->points);
            selection={{"name",static_cast<int>(it->name)},
              {"type",static_cast<int>(it->type)},{"confidence",it->confidence},
              {"detector_center_px",{it->center.x,it->center.y}},
              {"keypoint_center_px",{selected_center->x,selected_center->y}}};
            selection["keypoints_px"]=nlohmann::json::array();
            for(const auto& p:it->points)selection["keypoints_px"].push_back({p.x,p.y});
            const Eigen::Vector3d camera_point=solver.R_camera2gimbal().transpose()*(it->xyz_in_gimbal-solver.t_camera2gimbal());
            selection["pnp_camera_m"]=xyz(camera_point);
            pnp_center=project(camera_point);
            const Eigen::Vector3d muzzle_C=solver.R_camera2gimbal().transpose()*(muzzle-solver.t_camera2gimbal());
            // Keep the purple gun-line marker on the installation geometry.
            // An empirical command trim does not certify a physical bore tilt.
            const Eigen::Vector3d bore_C=solver.R_camera2gimbal().transpose()*nominal_bore;
            if(bore_C.z()>1e-6){
              const double along=(camera_point.z()-muzzle_C.z())/bore_C.z();
              if(along>0)gun_line=project(muzzle_C+along*bore_C);
            }
            const int id=static_cast<int>(it->name);consecutive=id==last_id?consecutive+1:1;last_id=id;
          }
        }
      }
      if(!target_valid){consecutive=0;last_id=-1;}
      // Inference takes time. Solve against a freshly sampled, matched joint/IMU
      // pair, while keeping the observation's own image-time pose above.
      const auto control=rotating?link.control_snapshot(image_us):link.snapshot(image_us);
      const auto planning_us=io::rmv::debug_now_us();
      auto_aim::RmvRotatingAim::Solution prediction;
      if(rotating){
        if(control.current_q)prediction=rotation.update(targets.empty()?nullptr:&targets.front(),target_valid,
          image_us*1e-6,planning_us*1e-6,quaternion(*control.current_q),control.joints,muzzle,bore,target_valid?&target:nullptr);
        else rotation.reset();
        goal=prediction.goal;
        if(!goal && target_valid && control.current_q){
          if(projectile_speed>0){
            const auto fallback=io::rmv::ballistic_from_muzzle(quaternion(*control.current_q),control.joints,target,muzzle,bore,projectile_speed);
            if(fallback)goal=fallback->joints;
          }else goal=io::rmv::point_from_muzzle(quaternion(*control.current_q),control.joints,target,muzzle,bore,1);
        }
      }else if(target_valid && control.current_q)
        goal=io::rmv::point_from_muzzle(quaternion(*control.current_q),control.joints,target,muzzle,bore,1);
      const auto now=io::rmv::debug_now_us();
      const bool fresh_image=now>=static_cast<std::uint64_t>(image_us) && now-image_us<=80000;
      const bool center_hold=rotating&&prediction.reason=="brief_visual_gap_center_hold";
      const auto nominal_goal=goal;
      auto_aim::Plan motion_plan{};
      if(rotating && goal && control.current_q && control.ready){
        std::optional<auto_aim::Trajectory> reference;
        if(!targets.empty())reference=rotation.trajectory(targets.front(),prediction,(planning_us-image_us)*1e-6,
          quaternion(*control.current_q),control.joints,muzzle,bore);
        if(!reference){
          auto_aim::Trajectory hold;
          for(int i=0;i<auto_aim::HORIZON;++i)hold.col(i)<<goal->yaw,0,goal->pitch,0;
          reference=hold;
        }
        motion_plan=planner.plan_joint_trajectory(*reference,control.joints.yaw);
        if(motion_plan.control){
          goal=JointGoal{motion_plan.yaw,motion_plan.pitch,
            std::clamp(double(motion_plan.yaw_vel),-2.,2.),std::clamp(double(motion_plan.pitch_vel),-1.5,1.5),
            std::clamp(double(motion_plan.yaw_acc),-100.,100.),std::clamp(double(motion_plan.pitch_acc),-100.,100.),true};
          goal=io::rmv::advance_trajectory(*goal,(io::rmv::debug_now_us()-planning_us)*1e-6);
        }else{goal.reset();prediction.qualified=false;prediction.reason="trajectory_planner_invalid";}
      }else if(rotating){goal.reset();prediction.qualified=false;}
      const auto command_us=io::rmv::debug_now_us();
      const bool planning_fresh=command_us>=static_cast<std::uint64_t>(image_us) && command_us-image_us<=80000;
      const bool submitted=link.submit(fresh_image && planning_fresh && (consecutive>=3||center_hold)?goal:std::nullopt);
      auto actual=link.snapshot(image_us);
      std::optional<std::uint32_t> shot_request;
      const bool qualified=rotating&&prediction.qualified&&fresh_image&&planning_fresh&&consecutive>=3&&goal.has_value();
      if(prediction.model.phase_reset){last_fire_cycle=-1;last_low_speed_face=-1;qualified_frames=0;}
      const auto candidate_cycle=prediction.event.valid?prediction.event.hit_cycle:-1;
      if(candidate_cycle!=qualification_cycle||prediction.face_id!=qualification_face)qualified_frames=0;
      qualification_cycle=candidate_cycle;qualification_face=prediction.face_id;
      qualified_frames=qualified?qualified_frames+1:0;
      const bool recovery_ready=!last_fire_us || now-last_fire_us>=recovery*1e6;
      const bool cycle_ready=prediction.event.valid?prediction.event.hit_cycle>last_fire_cycle:
        prediction.model.mode==auto_aim::RotationModelManager::Mode::armor || prediction.face_id!=last_low_speed_face;
      std::string fire_gate_reason=!fire?"disabled":!qualified?prediction.reason:!submitted?"aim_not_submitted":
        qualified_frames<required_frames?"qualification_frames":!actual.fire_permission?"physical_fire_permission":
        !recovery_ready?"feeder_recovery_window":!cycle_ready?"face_or_cycle_already_requested":"awaiting_trigger_window";
      if(fire && submitted && qualified_frames>=required_frames && actual.fire_permission && recovery_ready && cycle_ready){
        const double wait_s=prediction.event.valid?prediction.event.time_until_command_s:0;
        const double trigger_window=(cfg["xuc_event_trigger_window_ms"]?cfg["xuc_event_trigger_window_ms"].as<double>():50)*.001;
        if(wait_s>=0 && wait_s<=trigger_window){
          fire_gate_reason="alignment_or_readiness_after_wait";
          if(wait_s>0)std::this_thread::sleep_until(std::chrono::steady_clock::time_point(
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(prediction.event.command_due_time_s))));
          // Re-evaluate geometry, model and clock after waiting. This also vetoes
          // a target that became too old, moved off the bore or lost RC authority.
          const auto at_shot=link.control_snapshot(image_us);const auto fire_now=io::rmv::debug_now_us();
          if(at_shot.ready && at_shot.current_q){
            // Do not feed the same image to the phase clock twice. Recompute only
            // the future-face ballistic error using the freshly matched pair.
            Eigen::Vector3d impact=prediction.impact_face;
            // In continuous mode account for planning/transport time as well.
            // A phase event already has an absolute impact time; keep that face.
            if(!prediction.event.valid && !targets.empty() && prediction.face_id>=0){
              const auto future=targets.front().armor_xyza_list_at((fire_now-image_us)*1e-6+
                cfg["rmv_fire_delay_s"].as<double>()+prediction.flight_time,prediction.direct_motion);
              if(prediction.face_id<int(future.size()))impact=future[prediction.face_id].head<3>();
            }
            const auto alignment=io::rmv::ballistic_from_muzzle(quaternion(*at_shot.current_q),
              at_shot.joints,impact,muzzle,bore,projectile_speed,1,prediction.range_correction);
            const double distance=alignment?(impact-alignment->muzzle_world).norm():0;
            const double width=cfg["xuc_event_armor_width_m"].as<double>();
            const double height=cfg["rmv_armor_height_m"].as<double>();
            const bool on_armor=alignment && std::abs(std::remainder(alignment->joints.yaw-at_shot.joints.yaw,2*M_PI))<=std::atan2(width*.5,distance) &&
              std::abs(alignment->joints.pitch-at_shot.joints.pitch)<=std::atan2(height*.5,distance);
            const double late=prediction.event.valid?fire_now*1e-6-prediction.event.command_due_time_s:0;
            const double late_limit=(cfg["xuc_event_send_late_tolerance_ms"]?cfg["xuc_event_send_late_tolerance_ms"].as<double>():2)*.001;
            if(on_armor && late>=0 && late<=late_limit){
              shot_request=link.fire_once(io::rmv::advance_trajectory(*goal,(fire_now-command_us)*1e-6),image_us);
              fire_gate_reason=shot_request?"request_submitted":"link_shoot_state_or_pending_request";
            }
          }
          if(shot_request){last_fire_us=fire_now;last_fire_cycle=prediction.event.valid?prediction.event.hit_cycle:-1;
            if(!prediction.event.valid)last_low_speed_face=prediction.face_id;qualified_frames=0;}
        }
      }
      actual=link.snapshot(image_us);
      const auto logged_now=io::rmv::debug_now_us();
      nlohmann::json row={{"host_us",logged_now},{"image_host_us",image_us},{"frame_id",camera.get_last_frame_id()},
        {"image_age_us",logged_now>=static_cast<std::uint64_t>(image_us)?logged_now-image_us:0},
        {"firmware",actual.firmware},{"geometry",actual.geometry},{"mode",actual.mode},{"owner",actual.owner},
        {"ready",actual.ready},{"image_pose_valid",s.image_q.has_value()},{"current_pose_valid",control.current_q.has_value()},
        {"control_ready_reason",actual.ready_reason},{"mcu_vision_permit",actual.mcu_permission},{"mcu_fault_mask",actual.faults},
        {"detections",armors.size()},{"tracker",tracker.state()},{"target_valid",target_valid},
        {"enemy_color",cfg["enemy_color"].as<std::string>()},
        {"detections_by_color",{{"red",detected_red},{"blue",detected_blue}}},
        {"actual_joint_deg",{actual.joints.yaw/deg,actual.joints.pitch/deg}},
        {"yaw_axis","big"},{"small_yaw_joint_deg",actual.held_small_yaw/deg},
        {"goal_valid",goal.has_value()},{"aim_submitted",submitted},{"fire",shot_request.has_value()},
        {"received_seq",actual.received},{"applied_seq",actual.applied},{"constraints",actual.constraints}};
      if(rotating){
        row["trajectory_control"]=motion_plan.control;
        row["control_state_extrapolation_s"]=control.state_extrapolation_s;
        row["body_gyro_valid"]=control.gyro_valid;row["body_gyro_rad_s"]=control.body_gyro;
        row["planning_duration_us"]=command_us-planning_us;
        row["filtered_range_m"]=prediction.filtered_range;row["range_correction_m"]=prediction.range_correction;
        if(nominal_goal)row["nominal_goal_joint_deg"]={nominal_goal->yaw/deg,nominal_goal->pitch/deg};
        if(goal){row["command_velocity_rad_s"]={goal->yaw_rate,goal->pitch_rate};
          row["command_acceleration_rad_s2"]={goal->yaw_accel,goal->pitch_accel};}
        row["fire_gate_reason"]=fire_gate_reason;
        row["rotation"]={{"mode",auto_aim::RotationModelManager::name(prediction.model.mode)},
          {"position_prediction_source",prediction.direct_motion?"physical_face_cv":"rigid_vehicle"},
          {"converged",prediction.converged},{"yaw_trusted",prediction.yaw_trusted},{"omega_rad_s",prediction.omega},
          {"phase_locked",rotation.phase_locked()},{"face_period_s",rotation.face_period_s()},
          {"flight_time_s",prediction.flight_time},{"aim_point_world_m",xyz(prediction.aim_point)},
          {"impact_face_world_m",xyz(prediction.impact_face)},{"face_id",prediction.face_id},{"face_valid",prediction.face_valid},
          {"face_alignment_deg",prediction.face_alignment/deg},
          {"qualified",qualified},{"reason",prediction.reason},{"event_valid",prediction.event.valid},
          {"hit_cycle",prediction.event.hit_cycle},{"event_due_host_s",prediction.event.command_due_time_s},
          {"worst_phase_rad",std::isfinite(prediction.event.worst_interval_phase_rad)?nlohmann::json(prediction.event.worst_interval_phase_rad):nlohmann::json(nullptr)},
          {"yaw_error_deg",prediction.yaw_error/deg},{"pitch_error_deg",prediction.pitch_error/deg}};
        row["fire_permission"]=actual.fire_permission;row["shooter_rpm"]=actual.shooter_rpm;
        row["shoot_state_fresh"]=actual.shoot_state_fresh;row["shoot_valid_flags"]=actual.shoot_valid;
        row["shot_request_id"]=shot_request?nlohmann::json(*shot_request):nlohmann::json(nullptr);
        row["shot_event"]={{"request_id",actual.event_request_id},{"event",actual.shot_event},
          {"reason",actual.shot_reason},{"mcu_us",actual.shot_event_mcu_us}};
        if(!targets.empty()){
          row["rotation"]["observed_face_id"]=targets.front().last_id;
          row["rotation"]["physical_face_motion_trusted"]=targets.front().armor_motion_trusted(prediction.face_id);
          row["rotation"]["physical_face_velocity_world_m_s"]=xyz(targets.front().armor_motion_velocity(prediction.face_id));
          const auto state=targets.front().ekf_x();
          row["rotation"]["model_state"]=std::vector<double>(state.data(),state.data()+state.size());
          row["rotation"]["model_state_order"]={"x","vx","y","vy","z","vz","body_yaw","omega","radius","radius_delta","height_delta"};
          row["rotation"]["yaw_nis"]=targets.front().yaw_eskf_nis();
          row["rotation"]["body_nis"]=targets.front().ekf().last_nis;
        }
      }
      if(target_valid)row["target_world_m"]=xyz(target);
      row["selected_detection"]=selection;
      row["control_joint_sample_mcu_us"]=control.joint_sample_us;
      row["newest_joint_sample_mcu_us"]=control.newest_joint_sample_us;
      row["control_joint_deg"]={control.joints.yaw/deg,control.joints.pitch/deg};
      if(control.current_q)row["control_q_G_to_nav_wxyz"]=*control.current_q;
      row["joint_velocity_deg_s"]=nlohmann::json::array();
      row["mcu_joint_target_deg"]=nlohmann::json::array();
      for(unsigned id=0;id<3;++id){
        row["joint_velocity_deg_s"].push_back(actual.joint_velocity[id]/deg);
        row["mcu_joint_target_deg"].push_back(actual.joint_target_valid[id]?nlohmann::json(actual.joint_target[id]/deg):nlohmann::json(nullptr));
      }
      row["joint_array_order"]={"small_yaw","pitch","big_yaw"};
      if(pnp_center)row["pnp_center_px"]={pnp_center->x,pnp_center->y};
      if(gun_line)row["gun_line_at_target_depth_px"]={gun_line->x,gun_line->y};
      if(gun_line && selected_center)row["selected_minus_gun_line_px"]={selected_center->x-gun_line->x,selected_center->y-gun_line->y};
      row["status_age_us"]=actual.status_age_us?nlohmann::json(*actual.status_age_us):nlohmann::json(nullptr);
      row["joint_age_us"]=actual.joint_age_us?nlohmann::json(*actual.joint_age_us):nlohmann::json(nullptr);
      if(goal){
        row["command_error_deg"]={std::remainder(goal->yaw-actual.joints.yaw,360*deg)/deg,
          (goal->pitch-actual.joints.pitch)/deg};
        row["pitch_goal_clipped"]=goal->pitch<-.72 || goal->pitch>.45;
      }
      if(goal)row["unclipped_goal_joint_deg"]={goal->yaw/deg,goal->pitch/deg};
      if(actual.anchor)row["control_anchor_joint_deg"]={actual.anchor->yaw/deg,actual.anchor->pitch/deg};
      row["sent_goal_valid"]=submitted && actual.sent_goal.has_value();
      if(submitted && actual.sent_goal)row["sent_goal_joint_deg"]={actual.sent_goal->yaw/deg,actual.sent_goal->pitch/deg};
      if(s.image_q)row["image_q_G_to_nav_wxyz"]=*s.image_q;
      log<<row.dump()<<'\n';
      if(now-last_print>=200000){std::cout<<row.dump()<<std::endl;log.flush();last_print=now;}
      if(gui) {
        // HikRobot and the detector use RGB; HighGUI displays BGR. Convert a
        // separate display image so model input and color refinement stay RGB.
        cv::Mat display;
        cv::cvtColor(image,display,cv::COLOR_RGB2BGR);
        for(const auto& a:armors)if(a.points.size()==4)for(int k=0;k<4;++k)
          cv::line(display,a.points[k],a.points[(k+1)%4],cv::Scalar(0,255,0),2);
        for(const auto& a:armors)
          cv::putText(display,a.color==auto_aim::red?"red":"blue",a.center,
            cv::FONT_HERSHEY_SIMPLEX,.6,cv::Scalar(0,255,0),2);
        if(selected_center){
          cv::circle(display,*selected_center,10,{0,255,255},2);
          cv::putText(display,"TARGET",*selected_center+cv::Point2f(12,-12),cv::FONT_HERSHEY_SIMPLEX,.5,{0,255,255},1);
        }
        cv::drawMarker(display,{static_cast<int>(intrinsic[2]),static_cast<int>(intrinsic[5])},{255,255,0},cv::MARKER_CROSS,20,2);
        if(gun_line){
          cv::drawMarker(display,*gun_line,{255,0,255},cv::MARKER_CROSS,24,2);
          cv::putText(display,"GUN LINE",*gun_line+cv::Point2f(12,20),cv::FONT_HERSHEY_SIMPLEX,.5,{255,0,255},1);
        }
        cv::putText(display,fire?"RMV ROTATING / DOUBLE UP / HOST FIRE":follow?(rotating?"RMV ROTATING DRY RUN / NO FIRE":"RMV BIG Y + P / NO FIRE"):"RMV READ ONLY / NO FIRE",
          {15,30},cv::FONT_HERSHEY_SIMPLEX,.7,{0,255,255},2);
        cv::putText(display,"ENEMY: "+cfg["enemy_color"].as<std::string>(),
          {15,60},cv::FONT_HERSHEY_SIMPLEX,.7,{0,255,255},2);
        cv::putText(display,"CONTROL: "+actual.ready_reason,
          {15,90},cv::FONT_HERSHEY_SIMPLEX,.6,{0,255,255},2);
        if(goal){
          const auto e=row["command_error_deg"].get<std::vector<double>>();
          const std::string text=cv::format("CMD ERR Y/P: %+.2f / %+.2f deg",e[0],e[1]);
          cv::putText(display,text,{15,120},cv::FONT_HERSHEY_SIMPLEX,.6,{0,255,255},2);
          if(row["pitch_goal_clipped"].get<bool>())
            cv::putText(display,"P TARGET OUTSIDE LIMIT",{15,150},cv::FONT_HERSHEY_SIMPLEX,.6,{0,165,255},2);
        }
        if(rotating)cv::putText(display,std::string("ROTATION: ")+auto_aim::RotationModelManager::name(prediction.model.mode)+
          " / "+prediction.reason,{15,180},cv::FONT_HERSHEY_SIMPLEX,.55,{0,255,255},2);
        cv::imshow("RMV commissioning",display);if((cv::waitKey(1)&0xff)=='q')break;
      }
    }
    if(gui)cv::destroyWindow("RMV commissioning");
    return 0;
  }catch(const std::exception& e){std::cerr<<"RMV debug: "<<e.what()<<std::endl;return 1;}
}

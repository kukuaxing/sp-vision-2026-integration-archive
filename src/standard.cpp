#include <fmt/core.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>  // for setenv
#include <iterator>
#include <thread>
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

// ROS2 headers (仅在 ROS2 可用时编译，用于可视化)
#ifdef AMENT_CMAKE_FOUND
#include <rclcpp/rclcpp.hpp>
#include "tools/ros2_visualizer.hpp"
#endif

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/xuc.hpp"
#include "rmv_debug.hpp"
#include "standard_settings.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/final_yaw_limiter.hpp"
#include "tasks/auto_aim/final_pitch_limiter.hpp"
#include "tasks/auto_aim/direct_measurement.hpp"
#include "tasks/auto_aim/gyro_face_selector.hpp"
#include "tasks/auto_aim/gyro_phase_scheduler.hpp"
#include "tasks/auto_aim/robust_range_filter.hpp"
#include "tasks/auto_aim/rotation_model_manager.hpp"
#include "tasks/auto_aim/fire_pose_guard.hpp"
#include "tasks/auto_aim/shot_delay_adapter.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/sentinel_geometry.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/detection_search.hpp"
#include "tasks/auto_aim/rotation_fire_window.hpp"
#include "tools/vision_diagnostics.hpp"
#include "tools/exiter.hpp"
#include "tools/build_identity.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/trajectory.hpp"

using namespace std::chrono;

static nlohmann::json effective_aim_settings(const YAML::Node & yaml, const StandardSettings & s)
{
  constexpr double to_deg = 180.0 / M_PI;
  return {{"high_speed_delay_ms", yaml["high_speed_delay_time"].as<double>() * 1000.0},
    {"low_speed_delay_ms", yaml["low_speed_delay_time"].as<double>() * 1000.0},
    {"delay_switch_rad_s", yaml["decision_speed"].as<double>()},
    {"fire_min_interval_ms", yaml["sentinel_fire_min_interval_ms"]
      ? yaml["sentinel_fire_min_interval_ms"].as<int>() : io::XucSender::default_sentinel_fire_interval_ms},
    {"fire_interval_uses_default", !yaml["sentinel_fire_min_interval_ms"].IsDefined()},
    {"stationary_stable_ms", s.xuc_fire_stable_s * 1000.0},
    {"fire_yaw_tolerance_deg", s.xuc_fire_yaw_tolerance_rad * to_deg},
    {"fire_pitch_tolerance_deg", s.xuc_fire_pitch_tolerance_rad * to_deg},
    {"rotating_yaw_tolerance_deg", s.xuc_gyro_predicted_yaw_tolerance_rad * to_deg},
    {"event_tracking_tolerance_deg", s.xuc_event_gimbal_tracking_tolerance_rad * to_deg},
    {"yaw_boresight_deg", s.direct_yaw_boresight_correction_rad * to_deg},
    {"yaw_impact_deg", s.direct_yaw_impact_correction_rad * to_deg},
    {"fire_fixed_yaw_correction_deg", s.direct_yaw_command_correction_rad * to_deg},
    {"rotating_yaw_trim_deg", s.xuc_gyro_predicted_yaw_trim_rad * to_deg},
    {"pitch_trim_deg", s.xuc_ballistic_pitch_trim_rad * to_deg},
    {"projectile_speed_mps", s.xuc_projectile_speed_mps},
    {"event_delay_ms", s.xuc_event_fire_delay_s * 1000.0},
    {"fire_yaw_basis", "prediction_plus_following_correction_minus_trim"},
    {"stationary_yaw_basis", "measured_world_point_plus_following_correction"}};
}

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard.yaml | 位置参数，yaml配置文件路径 }"
  "{sentinel-check | false | 只检查Sentinel固定帧配置，不打开设备 }"
  "{rmv-follow | false | RMV大Y/P跟踪，不发射 }"
  "{rmv-rotate | false | RMV四装甲旋转预测与弹道试算，默认不发射 }"
  "{rmv-fire | false | RMV固定/旋转靶单发，双上转稳后由上位机触发，无摇杆许可 }"
  "{rmv-check | false | 仅核对RMV调试配置，不打开设备 }"
  "{rmv-shot-test | 0 | 保持当前云台角度的连续单发测试，1至20次，无视觉瞄准 }"
  "{rmv-rpm-delay | false | 配合rmv-shot-test，以200Hz轮速估计指令到掉速的延迟 }"
  "{seconds | 0 | RMV调试运行秒数，0为直到退出 }";

int main(int argc, char * argv[])
{
  // 🚀 关键性能优化：全局线程控制
  // vtune分析发现88个线程抢16个核心，导致严重性能下降
  // - cv::cvtColor的Bayer转RGB触发TBB并行：61.660s (10.9%)
  // - 线程调度开销（__sched_yield）：94.800s (16.7%)
  // - Spin Time浪费：99.970s

  // 1. 强制OpenCV单线程（避免cvtColor创建大量线程）
  cv::setNumThreads(1);

  // 2. 限制TBB全局线程池（OpenVINO和OpenCV共用）
  setenv("TBB_NUM_THREADS", "4", 1);  // NUC有16核，4线程足够

  // 3. 限制OpenMP线程数（某些OpenCV编译可能使用）
  setenv("OMP_NUM_THREADS", "4", 1);

  tools::logger()->info("[Performance] Thread limits: OpenCV=1, TBB=4, OMP=4");

  cv::CommandLineParser cli(argc, argv, keys);

  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  const auto integration_yaml = YAML::LoadFile(config_path);
  const bool sentinel_protocol = integration_yaml["board_protocol"] &&
    integration_yaml["board_protocol"].as<std::string>() == "SENTINEL";
  const auto shot_count=cli.get<int>("rmv-shot-test");
  const bool rpm_measurement=cli.get<bool>("rmv-rpm-delay");
  if(!cli.check() || shot_count<0 || shot_count>20) {
    std::cerr<<"Invalid command line / rmv-shot-test must be 1 to 20"<<std::endl;return 2;
  }
  if(rpm_measurement && !shot_count){std::cerr<<"rmv-rpm-delay requires a bounded rmv-shot-test"<<std::endl;return 2;}
  if (cli.get<bool>("sentinel-check")) {
    const StandardSettings check_settings(integration_yaml, sentinel_protocol);
    const auto effective = effective_aim_settings(integration_yaml, check_settings);
    const int fire_interval_ms = effective["fire_min_interval_ms"].get<int>();
    const int friction_hold_ms = integration_yaml["sentinel_friction_lost_timeout_ms"] ?
      integration_yaml["sentinel_friction_lost_timeout_ms"].as<int>() : 5000;
    const bool valid = sentinel_protocol && integration_yaml["xuc_enable"] &&
      integration_yaml["xuc_enable"].as<bool>() && integration_yaml["xuc_serial_baud"] &&
      integration_yaml["xuc_serial_baud"].as<int>() == 921600 && friction_hold_ms>=0 && friction_hold_ms<=60000 &&
      fire_interval_ms>=200 && fire_interval_ms<=5000;
    std::cout << nlohmann::json({{"protocol", "SENTINEL_V2"}, {"config_valid", valid},
      {"rx_frame_bytes", {46, 34, 26}}, {"tx_bytes", 38}, {"baud", 921600}, {"wire_units", "rad, rad/s, us"},
      {"auto_mode_state_bit", 128},
      {"friction_policy", "armor_detection_hold"}, {"friction_only_flags", 5},
      {"friction_lost_timeout_ms", friction_hold_ms},
      {"control_enabled", integration_yaml["xuc_allow_control"].as<bool>()},
      {"shoot_enabled", integration_yaml["xuc_allow_shoot"].as<bool>()},
      {"require_verified_pose", integration_yaml["sentinel_require_verified_pose"] ? integration_yaml["sentinel_require_verified_pose"].as<bool>() : true},
      {"effective_settings", effective},
      {"device_opened", false}, {"live_verified", false}}).dump() << std::endl;
    return valid ? 0 : 2;
  }
  if (sentinel_protocol && (!integration_yaml["xuc_enable"] ||
      !integration_yaml["xuc_enable"].as<bool>())) {
    std::cerr << "SENTINEL requires xuc_enable: true; no device was opened" << std::endl;
    return 2;
  }
  if(shot_count) {
    if(!integration_yaml["board_protocol"] || integration_yaml["board_protocol"].as<std::string>()!="RMV" ||
       cli.get<bool>("rmv-fire") || cli.get<bool>("rmv-follow") || cli.get<bool>("rmv-rotate")) {
      std::cerr<<"Shot test requires RMV and cannot be combined with tracking options"<<std::endl;return 2;
    }
    return run_rmv_shot_test(config_path,shot_count,cli.get<double>("seconds"),cli.get<bool>("rmv-check"),rpm_measurement);
  }
  if (integration_yaml["board_protocol"] &&
      integration_yaml["board_protocol"].as<std::string>()=="RMV")
    return run_rmv_debug(config_path,cli.get<bool>("rmv-follow"),
      cli.get<double>("seconds"),cli.get<bool>("rmv-check"),cli.get<bool>("rmv-rotate"),cli.get<bool>("rmv-fire"));
  if(cli.get<bool>("rmv-check") || cli.get<bool>("rmv-follow") || cli.get<bool>("rmv-rotate") || cli.get<bool>("rmv-fire")) {
    std::cerr<<"RMV options require board_protocol: RMV; no device was opened"<<std::endl;
    return 2;
  }
  tools::logger()->info(
    "[BUILD] revision={} source_sha256={} configured_standard_sha256={} "
    "runtime_config={} runtime_config_fnv1a64={} executable_fnv1a64={}",
    SP_SOURCE_REVISION, SP_SOURCE_SHA256, SP_CONFIGURED_STANDARD_SHA256,
    config_path, tools::file_fnv1a64(config_path), tools::file_fnv1a64("/proc/self/exe"));
  if (integration_yaml["yolo_name"]) {
    const auto model_key = integration_yaml["yolo_name"].as<std::string>() + "_model_path";
    if (integration_yaml[model_key]) {
      const auto model_path = integration_yaml[model_key].as<std::string>();
      const auto extension = model_path.find_last_of('.');
      const auto weights = extension == std::string::npos ? model_path :
        model_path.substr(0, extension) + ".bin";
      tools::logger()->info("[BUILD] model={} model_fnv1a64={} weights_fnv1a64={}",
        model_path, tools::file_fnv1a64(model_path), tools::file_fnv1a64(weights));
    }
  }
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  const StandardSettings settings(integration_yaml, sentinel_protocol);
  if (sentinel_protocol)
    tools::logger()->info("[CONFIG][EFFECTIVE] {}", effective_aim_settings(integration_yaml, settings).dump());
  const auto & competition_headless = settings.competition_headless;
  const auto & direct_yaw_guard_enabled = settings.direct_yaw_guard_enabled;
  const auto & prediction_lead_limit_configured = settings.prediction_lead_limit_configured;
  const auto & max_prediction_lead_rad = settings.max_prediction_lead_rad;
  const auto & direct_yaw_filter_tau_s = settings.direct_yaw_filter_tau_s;
  const auto & direct_yaw_hold_s = settings.direct_yaw_hold_s;
  const auto & direct_yaw_max_rate_rad_s = settings.direct_yaw_max_rate_rad_s;
  const auto & final_yaw_command_max_rate_rad_s = settings.final_yaw_command_max_rate_rad_s;
  const auto & direct_yaw_boresight_correction_rad = settings.direct_yaw_boresight_correction_rad;
  const auto & direct_yaw_impact_correction_rad = settings.direct_yaw_impact_correction_rad;
  const auto & direct_yaw_command_correction_rad = settings.direct_yaw_command_correction_rad;
  const auto & yaw_tracking_pi_enabled = settings.yaw_tracking_pi_enabled;
  const auto & yaw_tracking_kp = settings.yaw_tracking_kp;
  const auto & yaw_tracking_ki_per_s = settings.yaw_tracking_ki_per_s;
  const auto & yaw_tracking_integral_limit_rad = settings.yaw_tracking_integral_limit_rad;
  const auto & yaw_tracking_output_limit_rad = settings.yaw_tracking_output_limit_rad;
  const auto & yaw_tracking_integrate_error_limit_rad = settings.yaw_tracking_integrate_error_limit_rad;
  const auto & yaw_tracking_arm_dwell_s = settings.yaw_tracking_arm_dwell_s;
  const auto & direct_pitch_guard_enabled = settings.direct_pitch_guard_enabled;
  const auto & direct_pitch_filter_tau_s = settings.direct_pitch_filter_tau_s;
  const auto & direct_pitch_hold_s = settings.direct_pitch_hold_s;
  const auto & direct_pitch_max_rate_rad_s = settings.direct_pitch_max_rate_rad_s;
  const auto & direct_pitch_error_gain = settings.direct_pitch_error_gain;
  const auto & xuc_projectile_speed_mps = settings.xuc_projectile_speed_mps;
  const auto & xuc_ballistic_reference_projectile_speed_mps = settings.xuc_ballistic_reference_projectile_speed_mps;
  const auto & xuc_ballistic_pitch_enabled = settings.xuc_ballistic_pitch_enabled;
  const auto & xuc_camera_forward_of_muzzle_m = settings.xuc_camera_forward_of_muzzle_m;
  const auto & xuc_camera_above_muzzle_m = settings.xuc_camera_above_muzzle_m;
  const auto & xuc_ballistic_reference_camera_range_m = settings.xuc_ballistic_reference_camera_range_m;
  const auto & xuc_ballistic_reference_total_drop_m = settings.xuc_ballistic_reference_total_drop_m;
  const auto & xuc_ballistic_max_compensation_rad = settings.xuc_ballistic_max_compensation_rad;
  const auto & xuc_ballistic_pitch_trim_rad = settings.xuc_ballistic_pitch_trim_rad;
  const auto & robust_range_config = settings.robust_range_config;
  const auto & direct_pitch_focal_y_px = settings.direct_pitch_focal_y_px;
  const auto & direct_yaw_focal_x_px = settings.direct_yaw_focal_x_px;
  const auto & xuc_pitch_sign_for_log = settings.xuc_pitch_sign_for_log;
  const auto & xuc_fire_guard_enabled = settings.xuc_fire_guard_enabled;
  const auto & xuc_fire_stable_s = settings.xuc_fire_stable_s;
  const auto & xuc_fire_yaw_tolerance_rad = settings.xuc_fire_yaw_tolerance_rad;
  const auto & xuc_fire_pitch_tolerance_rad = settings.xuc_fire_pitch_tolerance_rad;
  const auto & xuc_gyro_center_follow_enabled = settings.xuc_gyro_center_follow_enabled;
  const auto & xuc_gyro_omega_threshold_rad_s = settings.xuc_gyro_omega_threshold_rad_s;
  const auto & xuc_vehicle_model_enter_rad_s = settings.xuc_vehicle_model_enter_rad_s;
  const auto & xuc_vehicle_model_exit_rad_s = settings.xuc_vehicle_model_exit_rad_s;
  const auto & xuc_high_speed_enter_rad_s = settings.xuc_high_speed_enter_rad_s;
  const auto & xuc_high_speed_exit_rad_s = settings.xuc_high_speed_exit_rad_s;
  const auto & xuc_rotation_mode_dwell_s = settings.xuc_rotation_mode_dwell_s;
  const auto & xuc_rotation_tracker_dropout_hold_s = settings.xuc_rotation_tracker_dropout_hold_s;
  const auto & xuc_rotation_rate_filter_tau_s = settings.xuc_rotation_rate_filter_tau_s;
  const auto & xuc_rotation_rate_disagreement_rad_s = settings.xuc_rotation_rate_disagreement_rad_s;
  const auto & xuc_rotation_speed_step_reset_rad_s = settings.xuc_rotation_speed_step_reset_rad_s;
  const auto & xuc_rotation_rate_agreement_updates = settings.xuc_rotation_rate_agreement_updates;
  const auto & xuc_rotation_rate_disagreement_updates = settings.xuc_rotation_rate_disagreement_updates;
  const auto & xuc_gyro_center_hold_s = settings.xuc_gyro_center_hold_s;
  const auto & xuc_fire_require_tracker_converged = settings.xuc_fire_require_tracker_converged;
  const auto & xuc_one_face_once_fire = settings.xuc_one_face_once_fire;
  const auto & xuc_one_face_limit_rad = settings.xuc_one_face_limit_rad;
  const auto & xuc_one_face_rearm_rad = settings.xuc_one_face_rearm_rad;
  const auto & xuc_gyro_fire_stable_s = settings.xuc_gyro_fire_stable_s;
  const auto & xuc_gyro_predicted_fire_enabled = settings.xuc_gyro_predicted_fire_enabled;
  const auto & xuc_gyro_require_event_lock_for_fire = settings.xuc_gyro_require_event_lock_for_fire;
  const auto & xuc_gyro_predicted_yaw_tolerance_rad = settings.xuc_gyro_predicted_yaw_tolerance_rad;
  const auto & xuc_gyro_predicted_yaw_trim_rad = settings.xuc_gyro_predicted_yaw_trim_rad;
  const auto & xuc_gyro_visibility_yaw_tolerance_rad = settings.xuc_gyro_visibility_yaw_tolerance_rad;
  const auto & xuc_gyro_fire_min_consecutive_frames = settings.xuc_gyro_fire_min_consecutive_frames;
  const auto & xuc_gyro_post_feed_recovery_s = settings.xuc_gyro_post_feed_recovery_s;
  const auto & xuc_event_fire_delay_s = settings.xuc_event_fire_delay_s;
  const auto & xuc_event_fire_delay_min_s = settings.xuc_event_fire_delay_min_s;
  const auto & xuc_event_fire_delay_max_s = settings.xuc_event_fire_delay_max_s;
  const auto & shot_delay_config = settings.shot_delay_config;
  const auto & xuc_event_flight_time_uncertainty_s = settings.xuc_event_flight_time_uncertainty_s;
  const auto & xuc_event_trigger_window_s = settings.xuc_event_trigger_window_s;
  const auto & xuc_event_send_late_tolerance_s = settings.xuc_event_send_late_tolerance_s;
  const auto & xuc_event_crossing_capture_rad = settings.xuc_event_crossing_capture_rad;
  const auto & xuc_event_crossing_rearm_rad = settings.xuc_event_crossing_rearm_rad;
  const auto & xuc_event_min_crossing_separation_ratio = settings.xuc_event_min_crossing_separation_ratio;
  const auto & xuc_event_period_min_s = settings.xuc_event_period_min_s;
  const auto & xuc_event_period_max_s = settings.xuc_event_period_max_s;
  const auto & xuc_event_period_ema_alpha = settings.xuc_event_period_ema_alpha;
  const auto & xuc_event_max_period_relative_error = settings.xuc_event_max_period_relative_error;
  const auto & xuc_event_max_period_step_ratio = settings.xuc_event_max_period_step_ratio;
  const auto & xuc_event_omega_period_weight = settings.xuc_event_omega_period_weight;
  const auto & xuc_event_period_residual_alpha = settings.xuc_event_period_residual_alpha;
  const auto & xuc_event_phase_correction_alpha = settings.xuc_event_phase_correction_alpha;
  const auto & xuc_event_max_phase_correction_s = settings.xuc_event_max_phase_correction_s;
  const auto & xuc_event_max_phase_residual_s = settings.xuc_event_max_phase_residual_s;
  const auto & xuc_event_period_window_size = settings.xuc_event_period_window_size;
  const auto & xuc_event_phase_uncertainty_floor_s = settings.xuc_event_phase_uncertainty_floor_s;
  const auto & xuc_event_phase_residual_window_size = settings.xuc_event_phase_residual_window_size;
  const auto & xuc_event_min_lock_updates = settings.xuc_event_min_lock_updates;
  const auto & xuc_event_relock_rejections = settings.xuc_event_relock_rejections;
  const auto & xuc_event_armor_width_m = settings.xuc_event_armor_width_m;
  const auto & xuc_event_armor_edge_margin_m = settings.xuc_event_armor_edge_margin_m;
  const auto & xuc_event_model_phase_uncertainty_rad = settings.xuc_event_model_phase_uncertainty_rad;
  const auto & xuc_event_phase_grace_rad = settings.xuc_event_phase_grace_rad;
  const auto & xuc_event_require_face_consistency = settings.xuc_event_require_face_consistency;
  const auto & xuc_event_face_max_alignment_rad = settings.xuc_event_face_max_alignment_rad;
  const auto & xuc_event_gimbal_tracking_tolerance_rad = settings.xuc_event_gimbal_tracking_tolerance_rad;
  const auto & xuc_shot_feedback_timeout_s = settings.xuc_shot_feedback_timeout_s;
  const auto & xuc_gyro_candidate_max_phase_step_rad = settings.xuc_gyro_candidate_max_phase_step_rad;
  if (yaw_tracking_pi_enabled) {
    tools::logger()->info(
      "[XUC][YAW-PI] enabled kp={:.2f} ki={:.2f}/s "
      "integral_limit={:.2f}deg output_limit={:.2f}deg "
      "integrate_error_limit={:.2f}deg arm_dwell={:.0f}ms",
      yaw_tracking_kp, yaw_tracking_ki_per_s,
      yaw_tracking_integral_limit_rad * 180.0 / M_PI,
      yaw_tracking_output_limit_rad * 180.0 / M_PI,
      yaw_tracking_integrate_error_limit_rad * 180.0 / M_PI,
      yaw_tracking_arm_dwell_s * 1000.0);
  }

#ifdef AMENT_CMAKE_FOUND
  // 初始化ROS2可视化
  rclcpp::init(argc, argv);
  auto visualizer = std::make_shared<tools::ROS2Visualizer>("standard_node", "standard_markers");
  tools::logger()->info("[ROS2] Visualizer initialized");
#endif

  io::CBoard cboard(config_path);
  io::XucSender xuc(config_path);
  if (sentinel_protocol && !xuc.enabled()) {
    tools::logger()->error("[SENTINEL] serial open failed; camera/control not started");
    return 2;
  }
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);  // 启用调试，显示检测窗口
  auto_aim::Solver solver(config_path);
  const auto_aim::SentinelGeometry sentinel_geometry(
    solver.R_gimbal2imubody(), solver.R_camera2gimbal());
  if (sentinel_protocol)
    tools::logger()->info(
      "[GEOMETRY] world=NAV gravity_up; G_to_world=IMU_to_NAV*G_to_IMU; "
      "commands=sensor_ZYX; regular_armor_top_back_deg=15");
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::DetectionSearch detection_search;
  auto_aim::RotationFireWindow rotation_fire_window;
  tools::VisionDiagnostics vision_diagnostics(integration_yaml);
  std::uint64_t vision_frame_sequence = 0;
  const cv::Rect configured_detection_roi(
    integration_yaml["roi"]["x"].as<int>(), integration_yaml["roi"]["y"].as<int>(),
    integration_yaml["roi"]["width"].as<int>(), integration_yaml["roi"]["height"].as<int>());
  auto_aim::ShotDelayAdapter shot_delay_adapter(shot_delay_config);
  auto_aim::Shooter shooter(config_path);
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter, true);
  auto_aim::GyroPhaseScheduler::Config gyro_phase_config;
  gyro_phase_config.crossing_capture_rad = xuc_event_crossing_capture_rad;
  gyro_phase_config.crossing_rearm_rad = xuc_event_crossing_rearm_rad;
  gyro_phase_config.min_crossing_separation_ratio =
    xuc_event_min_crossing_separation_ratio;
  gyro_phase_config.min_face_period_s = std::min(
    xuc_event_period_min_s, xuc_event_period_max_s);
  gyro_phase_config.max_face_period_s = std::max(
    xuc_event_period_min_s, xuc_event_period_max_s);
  gyro_phase_config.period_ema_alpha = xuc_event_period_ema_alpha;
  gyro_phase_config.max_period_relative_error =
    xuc_event_max_period_relative_error;
  gyro_phase_config.max_period_step_ratio = xuc_event_max_period_step_ratio;
  gyro_phase_config.omega_period_weight = xuc_event_omega_period_weight;
  gyro_phase_config.omega_alias_guard_relative_error = 0.35;
  gyro_phase_config.period_residual_alpha = xuc_event_period_residual_alpha;
  gyro_phase_config.phase_correction_alpha = xuc_event_phase_correction_alpha;
  gyro_phase_config.max_phase_correction_s = xuc_event_max_phase_correction_s;
  gyro_phase_config.max_phase_residual_s = xuc_event_max_phase_residual_s;
  gyro_phase_config.phase_uncertainty_floor_s = xuc_event_phase_uncertainty_floor_s;
  gyro_phase_config.period_window_size = xuc_event_period_window_size;
  gyro_phase_config.phase_residual_window_size = xuc_event_phase_residual_window_size;
  gyro_phase_config.min_lock_updates = xuc_event_min_lock_updates;
  gyro_phase_config.relock_after_rejections = xuc_event_relock_rejections;
  auto_aim::GyroPhaseScheduler gyro_phase_scheduler(gyro_phase_config);
  auto_aim::GyroFaceSelector gyro_face_selector(xuc_event_face_max_alignment_rad);
  auto_aim::RobustRangeFilter robust_range_filter(robust_range_config);
  auto_aim::RotationModelManager::Config rotation_model_config;
  rotation_model_config.vehicle_enter_rad_s = xuc_vehicle_model_enter_rad_s;
  rotation_model_config.vehicle_exit_rad_s = xuc_vehicle_model_exit_rad_s;
  rotation_model_config.high_speed_enter_rad_s = xuc_high_speed_enter_rad_s;
  rotation_model_config.high_speed_exit_rad_s = xuc_high_speed_exit_rad_s;
  rotation_model_config.mode_dwell_s = xuc_rotation_mode_dwell_s;
  rotation_model_config.tracker_dropout_hold_s =
    xuc_rotation_tracker_dropout_hold_s;
  rotation_model_config.rate_filter_tau_s = xuc_rotation_rate_filter_tau_s;
  rotation_model_config.max_rate_disagreement_rad_s =
    xuc_rotation_rate_disagreement_rad_s;
  rotation_model_config.speed_step_reset_rad_s =
    xuc_rotation_speed_step_reset_rad_s;
  rotation_model_config.rate_agreement_updates =
    xuc_rotation_rate_agreement_updates;
  rotation_model_config.rate_disagreement_updates =
    xuc_rotation_rate_disagreement_updates;
  auto_aim::RotationModelManager rotation_model_manager(rotation_model_config);
  // 🎯 所有模块初始化完成，启动相机触发
  tools::logger()->info("=== All modules initialized ===");

#ifdef AMENT_CMAKE_FOUND
  // 发布静态TF: gimbal -> camera（使用标定参数）
  visualizer->publish_static_tf("gimbal", "camera",
    solver.R_camera2gimbal(),
    solver.t_camera2gimbal());
  tools::logger()->info("[ROS2] Published static TF: gimbal -> camera");
#endif

  tools::logger()->info("=== Entering main loop ===");

  cv::Mat img;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  std::chrono::steady_clock::time_point t;

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  // 性能分析计时器
  std::chrono::steady_clock::time_point t_start, t_end;

  // 🆕 同步匹配相关变量（需要在循环外声明，以便后续日志使用）
  uint64_t frame_id = 0;
  uint64_t frame_id_last =0;
  int64_t trigger_imu_count = 0;

  // 📊 相机帧率测量变量
  std::chrono::steady_clock::time_point camera_last_frame_time;
  std::chrono::steady_clock::time_point camera_current_frame_time;
  double camera_fps_instant = 0.0;     // 瞬时帧率
  double camera_fps_avg = 0.0;         // 平均帧率
  int camera_frame_count = 0;          // 帧计数

  // Bounded yaw conditioner.  Detector measurements remain authoritative while
  // a small, validated native prediction lead helps fast lateral tracking.
  // Long visual loss drops control without destroying reacquisition continuity.
  bool direct_yaw_initialized = false;
  auto_aim::DirectControlGate direct_yaw_control_gate;
  double direct_yaw_filtered = 0.0;
  bool direct_yaw_long_loss = false;
  std::chrono::steady_clock::time_point direct_yaw_last_update;
  std::chrono::steady_clock::time_point direct_yaw_last_measurement;
  // Limit the complete yaw reference after model selection and prediction.
  // The measurement conditioner alone cannot bound an abruptly changing
  // native prediction lead or a vehicle-center/armor-model hand-over.
  auto_aim::FinalYawLimiter final_yaw_limiter(final_yaw_command_max_rate_rad_s);
  auto_aim::FinalPitchLimiter final_pitch_limiter(direct_pitch_max_rate_rad_s);
  double yaw_tracking_integral_raw_rad = 0.0;
  std::chrono::steady_clock::time_point yaw_tracking_last_update;
  std::chrono::steady_clock::time_point yaw_tracking_armed_since;
  bool yaw_tracking_initialized = false;
  bool yaw_tracking_lower_armed = false;
  bool direct_pitch_initialized = false;
  double direct_pitch_filtered = 0.0;
  bool direct_pitch_long_loss = false;
  std::chrono::steady_clock::time_point direct_pitch_last_update;
  std::chrono::steady_clock::time_point direct_pitch_last_measurement;
  bool xuc_fire_candidate_active = false;
  bool xuc_fire_ready_logged = false;
  int xuc_fire_candidate_frames = 0;
  double xuc_fire_candidate_face_yaw_rad = INFINITY;
  std::chrono::steady_clock::time_point xuc_fire_candidate_since;
  bool xuc_one_face_armed = true;
  bool xuc_gyro_center_follow_logged = false;
  bool xuc_gyro_center_memory_valid = false;
  double xuc_gyro_center_memory_yaw = 0.0;
  std::chrono::steady_clock::time_point xuc_gyro_center_last_fresh;
  bool xuc_feeder_activity_seen = false;
  bool xuc_feeder_active = false;
  uint64_t xuc_feeder_transition_sequence = 0;
  bool xuc_gyro_cadence_hold_logged = false;
  std::int64_t event_last_fired_cycle = -1;
  std::chrono::steady_clock::time_point xuc_feeder_last_active;
  uint64_t xuc_next_shot_id = 1;
  uint64_t xuc_pending_shot_id = 0;
  uint64_t xuc_active_shot_id = 0;
  bool xuc_pending_shot = false;
  std::chrono::steady_clock::time_point xuc_pending_shot_time;
  std::chrono::steady_clock::time_point xuc_active_shot_start_time;
  std::chrono::steady_clock::time_point fps_measure_start; // 平均帧率测量开始时间
  bool fps_measure_started = false;

  // 初始化默认命令（第一帧使用）
  io::Command default_command;
  default_command.yaw = 0.0;
  default_command.pitch = 0.0;
  default_command.control = false;
  default_command.shoot = false;
  std::uint64_t control_track_id = 0;
  int control_target_name = -1;
  int control_target_type = -1;

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();  // 🔍 性能监控：循环开始

    // 🎯 关键修改：先发送命令，触发相机曝光，然后再读取图像
    // 这样确保电控板收到命令后，才会发出硬触发信号让相机曝光
    static bool first_loop = true;
    if (first_loop) {
      // 第一帧发送默认命令
      if (xuc.enabled()) {
        xuc.stop();
      } else {
        cboard.send(default_command);
      }
      tools::logger()->info("[SYNC] 发送默认命令，等待相机触发");
      first_loop = false;
    }

    camera.read(img, t);

    // 📊 相机帧率测量
    camera_current_frame_time = std::chrono::steady_clock::now();
    if (!fps_measure_started) {
      // 初始化帧率测量
      camera_last_frame_time = camera_current_frame_time;
      fps_measure_start = camera_current_frame_time;
      fps_measure_started = true;
      camera_frame_count = 0;
    } else {
      // 计算瞬时帧率（基于相邻两帧的时间间隔）
      auto frame_interval = std::chrono::duration<double>(camera_current_frame_time - camera_last_frame_time).count();
      if (frame_interval > 0) {
        camera_fps_instant = 1.0 / frame_interval;
      }

      // 计算平均帧率（基于总时间和总帧数）
      camera_frame_count++;
      auto total_time = std::chrono::duration<double>(camera_current_frame_time - fps_measure_start).count();
      if (total_time > 0) {
        camera_fps_avg = camera_frame_count / total_time;
      }

      // 更新上一帧时间
      camera_last_frame_time = camera_current_frame_time;

      // 每50帧打印一次帧率统计
      static int fps_log_counter = 0;
      if (++fps_log_counter % 50 == 0) {
        tools::logger()->info(
          "📊 [CAMERA FPS] 瞬时帧率={:.1f}fps, 平均帧率={:.1f}fps, 总帧数={}, 运行时间={:.1f}s",
          camera_fps_instant, camera_fps_avg, camera_frame_count, total_time);
      }
    }

      // ==================== 基于 count 硬同步（使用环形数组） ====================
      // 核心思想：相机由MCU硬触发,每来一帧图像对应一个IMU计数（0-15循环）
      // 映射关系：frame_id % 16 = imu_count
      // MCU发送的imu_count是4位二进制，范围0-15
      static const int64_t frame_id_to_imu_offset = 0;  // 🔧 手动调试参数（微调偏移）

      static bool first_frame = true;

      frame_id = camera.get_last_frame_id();

      // 新版硬件中IMU连接下位机，姿态由XUC上行包提供。
      if (xuc.enabled()) {
        if (xuc.rx_valid()) {
          const auto imu_angles = xuc.imu_angles_rad();
          const double imu_yaw = imu_angles[2];
          const double imu_pitch = imu_angles[1];
          q = Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(imu_pitch, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(imu_angles[0], Eigen::Vector3d::UnitX());
          q.normalize();
          if (xuc.is_sentinel()) {
            if (!solver.set_R_gimbal2world_absolute_imu(q)) continue;
          } else {
            solver.set_R_gimbal2world(q);
          }
          cboard.mode = static_cast<io::Mode>(xuc.mode());
          if (!xuc.is_sentinel()) {
            const double lower_diagnostic_value = xuc.bullet_speed();
            const auto feeder_feedback = xuc.feeder_feedback();
            const auto feedback_now = std::chrono::steady_clock::now();
            if (feeder_feedback.transition_sequence != xuc_feeder_transition_sequence) {
              xuc_feeder_transition_sequence = feeder_feedback.transition_sequence;
              const auto feeder_edge_time = std::chrono::steady_clock::time_point(
                std::chrono::nanoseconds(feeder_feedback.transition_time_ns));
              xuc_feeder_active = feeder_feedback.active;
              if (feeder_feedback.active) {
                xuc_feeder_activity_seen = true;
                xuc_feeder_last_active = feeder_edge_time;
                if (xuc_pending_shot) {
                  const double command_to_feeder_ms =
                    std::chrono::duration<double, std::milli>(
                      feeder_edge_time - xuc_pending_shot_time).count();
                  if (command_to_feeder_ms >= 0.0 &&
                      command_to_feeder_ms <= xuc_shot_feedback_timeout_s * 1000.0) {
                    xuc_active_shot_id = xuc_pending_shot_id;
                    xuc_active_shot_start_time = feeder_edge_time;
                    const auto adaptation =
                      shot_delay_adapter.update(command_to_feeder_ms / 1000.0);
                    if (adaptation.accepted) {
                      aimer.set_delay_adjustment(shot_delay_adapter.delay_adjustment_s());
                      if (adaptation.regime_change) {
                        tools::logger()->warn(
                          "[SHOTADAPT] persistent feeder-delay shift detected; "
                          "rebuilding robust window");
                      }
                      tools::logger()->info(
                        "[SHOTADAPT] id={} feeder_ms={:.2f} samples={} ready={} "
                        "delay_ms={:.2f} pi95_ms=[{:.2f},{:.2f}]",
                        xuc_active_shot_id, command_to_feeder_ms,
                        adaptation.sample_count, adaptation.ready ? 1 : 0,
                        adaptation.delay_s * 1000.0,
                        adaptation.delay_min_s * 1000.0,
                        adaptation.delay_max_s * 1000.0);
                    } else if (adaptation.rejected_outlier) {
                      tools::logger()->warn(
                        "[SHOTADAPT] id={} rejected feeder outlier {:.2f} ms; "
                        "delay held at {:.2f} ms",
                        xuc_active_shot_id, command_to_feeder_ms,
                        shot_delay_adapter.delay_s() * 1000.0);
                    }
                    tools::logger()->info(
                      "[SHOT] id={} feeder_start command_to_feeder_ms={:.2f} rpm={:.0f}",
                      xuc_active_shot_id, command_to_feeder_ms, feeder_feedback.rpm);
                  } else {
                    tools::logger()->warn(
                      "[SHOT] id={} feeder_start_unmatched age_ms={:.2f} rpm={:.0f}",
                      xuc_pending_shot_id, command_to_feeder_ms, feeder_feedback.rpm);
                  }
                  xuc_pending_shot = false;
                  xuc_pending_shot_id = 0;
                } else {
                  tools::logger()->info(
                    "[SHOT] feeder_start_unpaired rpm={:.0f}", feeder_feedback.rpm);
                }
              } else {
                xuc_feeder_last_active = feeder_edge_time;
                if (xuc_active_shot_id != 0) {
                  const double feeder_motion_ms =
                    std::chrono::duration<double, std::milli>(
                      feeder_edge_time - xuc_active_shot_start_time).count();
                  tools::logger()->info(
                    "[SHOT] id={} feeder_stop feeder_motion_ms={:.1f}",
                    xuc_active_shot_id, feeder_motion_ms);
                  xuc_active_shot_id = 0;
                }
                tools::logger()->info(
                  "[XUC][FIRE] feeder stopped; post-feed hold={:.0f} ms",
                  xuc_gyro_post_feed_recovery_s * 1000.0);
              }
            }
            if (xuc_pending_shot &&
                std::chrono::duration<double>(feedback_now - xuc_pending_shot_time).count() >
                  xuc_shot_feedback_timeout_s) {
              tools::logger()->warn(
                "[SHOT] id={} no_feeder_feedback timeout_ms={:.0f}",
                xuc_pending_shot_id, xuc_shot_feedback_timeout_s * 1000.0);
              xuc_pending_shot = false;
              xuc_pending_shot_id = 0;
            }
            const uint8_t motion_gate_bits = xuc.robot_id();
            const uint16_t packed_wheel_rpm = xuc.bullet_count();
            const unsigned wheel0_rpm =
              static_cast<unsigned>(packed_wheel_rpm & 0xffU) * 40U;
            const unsigned wheel1_rpm =
              static_cast<unsigned>((packed_wheel_rpm >> 8U) & 0xffU) * 40U;


            static int xuc_imu_log_counter = 0;
            const int xuc_log_period_frames = (xuc.mode() == 1) ? 5 : 50;
            if (++xuc_imu_log_counter % xuc_log_period_frames == 0) {
              tools::logger()->info(
                "[SYNC][XUC] mode={} yaw={:.2f} deg pitch={:.2f} deg "
                "feeder_rpm={:.2f} wheel0_rpm={} wheel1_rpm={} wheel_delta={} "
                "projectile_speed={:.2f} gate=0x{:02X} "
                "switch={} rc={} sticks={} imu={} serial={} control={} yaw_armed={} pitch_active={}",
                static_cast<int>(xuc.mode()),
                imu_yaw * 180.0 / M_PI, imu_pitch * 180.0 / M_PI,
                lower_diagnostic_value, wheel0_rpm, wheel1_rpm,
                static_cast<int>(wheel0_rpm) - static_cast<int>(wheel1_rpm),
                cboard.bullet_speed, static_cast<unsigned>(motion_gate_bits),
                (motion_gate_bits >> 0U) & 1U, (motion_gate_bits >> 1U) & 1U,
                (motion_gate_bits >> 2U) & 1U, (motion_gate_bits >> 3U) & 1U,
                (motion_gate_bits >> 4U) & 1U, (motion_gate_bits >> 5U) & 1U,
                (motion_gate_bits >> 6U) & 1U, (motion_gate_bits >> 7U) & 1U);
            }
          } else {
            const auto feedback = xuc.sentinel_feedback();
            static int sentinel_log_counter = 0;
            if (++sentinel_log_counter % 50 == 0)
              tools::logger()->info(
                "[SYNC][SENTINEL V2] yaw={:.2f} pitch={:.2f} roll={:.2f} deg "
                "boot={} p_time_us={} p_seq={} p_flags={} c_time_us={} c_flags={} "
                "aim_seq={} state={} reject={} age_ms={}",
                imu_yaw * 180.0 / M_PI, imu_pitch * 180.0 / M_PI, xuc.imu_roll() * 180.0 / M_PI,
                feedback.boot_id, feedback.gimbal.time_us, feedback.gimbal.seq, feedback.gimbal.flags,
                feedback.chassis.time_us, feedback.chassis.flags, feedback.link.aim_last_seq,
                feedback.link.state, feedback.link.last_reject, feedback.link.aim_age_ms);
          }
          cboard.bullet_speed = xuc_projectile_speed_mps;
          mode = cboard.mode;
        } else {
          cboard.mode = io::Mode::idle;
          mode = io::Mode::idle;
          if (xuc.is_sentinel()) {
            static int invalid_pose_log_counter = 0;
            if (++invalid_pose_log_counter % 50 == 0) {
              const auto feedback = xuc.sentinel_feedback();
              tools::logger()->warn(
                "[SYNC][SENTINEL] link={} pose=0 p_valid={} p_id={} p_time={} frames={} crc_errors={}",
                xuc.feedback_link_valid(), feedback.gimbal.valid(), feedback.gimbal.id,
                feedback.gimbal.time_us, xuc.valid_packets(), xuc.crc_errors());
            }
          }
        }
      // 海康等不提供 frame_id 的相机使用 steady_clock 时间戳插值。
      } else if (frame_id == 0) {
        q = cboard.imu_at(t);
        solver.set_R_gimbal2world(q);

        static int timestamp_sync_log_counter = 0;
        if (++timestamp_sync_log_counter % 50 == 0) {
          auto sync_ypr = tools::eulers(q.toRotationMatrix(), 2, 1, 0);
          tools::logger()->info(
            "[SYNC][Timestamp] yaw={:.2f} deg pitch={:.2f} deg roll={:.2f} deg",
            sync_ypr[0] * 180.0 / M_PI,
            sync_ypr[1] * 180.0 / M_PI,
            sync_ypr[2] * 180.0 / M_PI);
        }

        mode = cboard.mode;
      } else if (frame_id - frame_id_last != 0) {
      // ✅ 修复：根据frame_id计算对应的IMU计数（0-15循环）
      trigger_imu_count = (frame_id + frame_id_to_imu_offset) % 16;
      if (trigger_imu_count < 0) trigger_imu_count += 16;

      // 🔍 检查frame_id是否跳帧
      if (frame_id - frame_id_last > 1) {
        tools::logger()->warn("[SYNC] ⚠️ 相机跳帧！frame_id: {} -> {} (跳过{}帧)",
          frame_id_last, frame_id, frame_id - frame_id_last - 1);
      }

      //使用环形数组O(1)查询IMU数据
      auto imu_result = cboard.get_imu_from_ring_buffer(trigger_imu_count);

      if (imu_result.valid) {
        // 环形数组查询成功
        q = imu_result.q;  // 四元数
        t = imu_result.timestamp;

        // 🔍 计算帧间隔和跳帧情况
        static auto last_t = t;
        static uint64_t last_valid_frame_id = 0;
        auto frame_interval_ms = std::chrono::duration<double, std::milli>(t - last_t).count();
        uint64_t frame_gap = frame_id - last_valid_frame_id;

        // 相机理论帧率 = 1000 / (实际间隔ms / 实际帧数差)
        double camera_theoretical_fps = frame_gap * 1000.0 / frame_interval_ms;

        last_t = t;
        last_valid_frame_id = frame_id;

        // 🔍 同步调试日志（每50帧打印一次）
        static int sync_log_counter = 0;
        if (++sync_log_counter % 50 == 0) {
          tools::logger()->info("[SYNC] ✅ frame_id={}, trigger_imu_count={}, 间隔={:.1f}ms, 跳过{}帧, 相机理论帧率≈{:.0f}fps",
            frame_id, trigger_imu_count, frame_interval_ms, frame_gap - 1, camera_theoretical_fps);
        }

        // ✅ 在获取到有效IMU数据后立即设置solver，确保姿态同步
        solver.set_R_gimbal2world(q);

#ifdef AMENT_CMAKE_FOUND
        // 发布动态TF: world -> gimbal（使用MCU四元数）
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          t.time_since_epoch()).count();
        rclcpp::Time ros_time(ns);
        Eigen::Vector3d zero_trans(0, 0, 0);  // world和gimbal原点重合
        visualizer->publish_dynamic_tf("world", "gimbal", q, zero_trans, ros_time);
#endif
      } else {
        // 🔍 同步调试：IMU数据无效（可能是时序问题或环形缓冲区被覆盖）
        tools::logger()->warn("[SYNC] ❌ frame_id={}, trigger_imu_count={}, IMU数据无效（可能还未到达或已被覆盖）",
          frame_id, trigger_imu_count);
      }
    mode = cboard.mode;
    frame_id_last=frame_id;
     }
    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }
    // recorder.record(img, q, t);
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    Eigen::Vector3d current_ypr = ypr;
    Eigen::Matrix3d frame_current_rotation = solver.R_gimbal2world();
    const auto sentinel_world_rotation = [&](const io::XucSender::PoseSnapshot & pose)
      -> Eigen::Matrix3d {
      return sentinel_geometry.world_rotation(pose.rpy_rad);
    };

    auto t1 = std::chrono::steady_clock::now();  // 🔍 性能监控
    ++vision_frame_sequence;
    const bool search_full_frame = detection_search.full_frame();
    auto armors = detector.detect(img, static_cast<int>(vision_frame_sequence), search_full_frame);
    const auto raw_detection_count = armors.size();
    nlohmann::json detector_observations;
    if (vision_diagnostics.enabled()) {
      detector_observations = nlohmann::json::array();
      for (const auto & detection : armors) {
        if (detector_observations.size() >= 16) break;
        nlohmann::json corners = nlohmann::json::array();
        for (const auto & point : detection.points) corners.push_back({point.x, point.y});
        detector_observations.push_back({{"name", static_cast<int>(detection.name)},
          {"type", static_cast<int>(detection.type)}, {"color", static_cast<int>(detection.color)},
          {"confidence", detection.confidence}, {"keypoint_confidence", detection.keypoint_confidence},
          {"physical_corners_valid", detection.pose_observation_valid}, {"corners_px", corners}});
      }
    }
    const auto display_detections = competition_headless ? decltype(armors){} : armors;
    auto t2 = std::chrono::steady_clock::now();
    io::XucSender::PoseSnapshot frame_pose_now;
    io::XucSender::PoseSnapshot frame_pose_capture;
    bool frame_pose_context_matches = true;
    if (xuc.is_sentinel()) {
      // Inference gives the RX thread time to obtain a later IMU bracket.
      // A missing bracket drops this measurement; it never invents a pose or
      // destroys an otherwise continuous track on a single missing sample.
      frame_pose_capture = xuc.pose_at_capture(t, std::chrono::milliseconds(5));
      frame_pose_now = xuc.pose_now();
      frame_pose_context_matches = frame_pose_capture.generation == frame_pose_now.generation &&
        frame_pose_capture.boot_id == frame_pose_now.boot_id &&
        frame_pose_capture.epoch == frame_pose_now.epoch;
      if (frame_pose_now.valid) {
        frame_current_rotation = sentinel_world_rotation(frame_pose_now);
        current_ypr = tools::eulers(frame_current_rotation, 2, 1, 0);
      }
      if (frame_pose_capture.valid && frame_pose_context_matches) {
        const auto & a = frame_pose_capture.rpy_rad;
        q = Eigen::AngleAxisd(a[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(a[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(a[0], Eigen::Vector3d::UnitX());
        q.normalize();
        if (!solver.set_R_gimbal2world_absolute_imu(q)) frame_pose_capture.valid = false;
        ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      }
      cboard.mode = static_cast<io::Mode>(frame_pose_now.mode);
      mode = cboard.mode;
      // Only an actual boot/reference change destroys the coordinate history.
      bool reset_requested = xuc.consume_tracker_reset();
      if (reset_requested) {
        tracker.reset();
        detection_search.reset();
        rotation_fire_window = {};
        gyro_phase_scheduler.reset();
        gyro_face_selector.reset();
        direct_yaw_initialized = direct_pitch_initialized = false;
        direct_yaw_control_gate.reset();
        xuc_fire_candidate_active = xuc_fire_ready_logged = false;
        armors.clear();
      }
      if (!frame_pose_capture.valid || !frame_pose_now.valid || !frame_pose_context_matches)
        armors.clear();
#ifdef AMENT_CMAKE_FOUND
      if (frame_pose_capture.valid && frame_pose_context_matches) {
        const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          t.time_since_epoch()).count();
        visualizer->publish_dynamic_tf("world", "gimbal",
          Eigen::Quaterniond(solver.R_gimbal2world()), Eigen::Vector3d::Zero(), rclcpp::Time(ns));
      }
#endif
    }
    auto targets = tracker.track(armors, t);
    const auto * fresh_primary = tracker.primary_observation();
    detection_search.observe(tracker.state(), tracker.temp_lost_count(),
      tracker.diagnostics().rebuilt || !frame_pose_context_matches,
      fresh_primary ? &fresh_primary->points : nullptr,
      auto_aim::clipped_detection_roi(configured_detection_roi, img.size()));
    const bool control_model_changed = !targets.empty() &&
      (static_cast<int>(targets.front().name) != control_target_name ||
       static_cast<int>(targets.front().armor_type) != control_target_type);
    if (tracker.track_id() != control_track_id || control_model_changed || tracker.diagnostics().rebuilt) {
      control_track_id = tracker.track_id();
      control_target_name = targets.empty() ? -1 : static_cast<int>(targets.front().name);
      control_target_type = targets.empty() ? -1 : static_cast<int>(targets.front().armor_type);
      direct_yaw_control_gate.reset();
      xuc_fire_candidate_active = xuc_fire_ready_logged = false;
      xuc_fire_candidate_frames = 0;
      xuc_one_face_armed = true;
      event_last_fired_cycle = -1;
      xuc_gyro_center_memory_valid = false;
      rotation_fire_window = {};
      gyro_phase_scheduler.reset();
      gyro_face_selector.reset();
      rotation_model_manager.reset();
      robust_range_filter.reset();
    }
    auto t3 = std::chrono::steady_clock::now();

    static std::string last_logged_track_state;
    static std::uint64_t last_logged_track_id = 0;
    static unsigned track_log_frames = 0;
    if (++track_log_frames % 30 == 0 || tracker.state() != last_logged_track_state ||
        tracker.track_id() != last_logged_track_id) {
      const char * reason = xuc.is_sentinel() && !frame_pose_context_matches
        ? "capture_pose_context_changed" : xuc.is_sentinel() && !frame_pose_capture.valid
        ? "capture_pose_unavailable" : armors.empty() ? "no_enemy_detection" :
          tracker.measurement_updated() ? "measurement_updated" : "association_rejected";
      tools::logger()->info(
        "[TRACK] frame={} id={} state={} raw_detections={} detections={} associations={} updated={} "
        "identity_confirmed={} reason={} capture_pose={} pose_status={} "
        "pose_seq={} pose_time_us={} time_uncertainty_ms={:.2f} synchronized={}",
        camera.get_last_capture_frame_id(), tracker.track_id(), tracker.state(),
        raw_detection_count, armors.size(), tracker.last_associations().size(), tracker.measurement_updated(),
        tracker.identity_confirmed(), reason, frame_pose_capture.valid,
        static_cast<int>(frame_pose_capture.status), frame_pose_capture.source_seq,
        frame_pose_capture.source_time_us, frame_pose_capture.time_uncertainty_ns / 1e6,
        frame_pose_capture.synchronized);
      last_logged_track_state = tracker.state();
      last_logged_track_id = tracker.track_id();
    }

    bool tracker_converged = false;
    bool yaw_motion_model_trusted = false;
    bool whole_vehicle_candidate_trusted = false;
    bool vehicle_model_fresh = false;
    bool gyro_model_fresh = false;
    bool gyro_phase_model_trusted = false;
    bool gyro_center_follow_active = false;
    double gyro_center_yaw = 0.0;
    double gyro_body_phase_rad = NAN;
    double gyro_omega_rad_s = 0.0;
    double gyro_radius_m = 0.0;
    if (!targets.empty()) {
      auto & tracked_target = targets.front();
      tracker_converged = tracked_target.convergened();
      yaw_motion_model_trusted = tracked_target.yaw_eskf_trusted();
      const auto & whole_x = tracked_target.ekf().x;
      // Low/medium-speed whole-state prediction earns trust from that same
      // whole state. Independent-yaw agreement is required only when using
      // the independent yaw / high-speed phase path, never as its entry key.
      whole_vehicle_candidate_trusted = tracked_target.whole_rotation_trusted() &&
        std::abs(whole_x[7]) < xuc_high_speed_enter_rad_s &&
        rotation_model_manager.mode() != auto_aim::RotationModelManager::Mode::high_speed_phase;
      const Eigen::VectorXd tracked_x = whole_vehicle_candidate_trusted
        ? whole_x : tracked_target.ekf_x();
      if (tracked_x.size() >= 9) {
        gyro_center_yaw = std::atan2(tracked_x[2], tracked_x[0]);
        gyro_body_phase_rad = tracked_x[6];
        gyro_omega_rad_s = tracked_x[7];
        gyro_radius_m = tracked_x[8];
      }
      static int yaw_eskf_log_counter = 0;
      if (++yaw_eskf_log_counter % 50 == 0) {
        const Eigen::Vector3d armor_velocity =
          tracked_target.armor_motion_velocity(tracked_target.last_id);
        tools::logger()->info(
          "[YAWESKF] trusted={} angle={:+.3f} omega={:+.3f} alpha={:+.3f} "
          "raw_ekf_omega={:+.3f} nis={:.2f} accepted={} rejected={} "
          "whole_iekf_nis={:.2f} armor_cv_trusted={} armor_v=({:+.2f},{:+.2f},{:+.2f}) "
          "armor_cv_nis={:.2f}",
          tracked_target.yaw_eskf_trusted() ? 1 : 0,
          tracked_target.yaw_eskf_angle(), tracked_target.yaw_eskf_rate(),
          tracked_target.yaw_eskf_acceleration(),
          tracked_target.ekf().x.size() >= 8 ? tracked_target.ekf().x[7] : NAN,
          tracked_target.yaw_eskf_nis(),
          tracked_target.yaw_eskf_accepted_updates(),
          tracked_target.yaw_eskf_rejected_updates(),
          tracked_target.ekf().last_nis,
          tracked_target.armor_motion_trusted(tracked_target.last_id) ? 1 : 0,
          armor_velocity.x(), armor_velocity.y(), armor_velocity.z(),
          tracked_target.armor_motion_nis(tracked_target.last_id));
      }
    }
    const double model_now_s = std::chrono::duration<double>(
      t.time_since_epoch()).count();
    const bool prior_phase_rate_valid = gyro_phase_scheduler.period_valid();
    const double prior_phase_omega_rad_s = prior_phase_rate_valid
      ? (M_PI / 2.0) / gyro_phase_scheduler.face_period_s()
      : NAN;
    const auto rotation_model = rotation_model_manager.update(
      model_now_s, tracker_converged && (yaw_motion_model_trusted || whole_vehicle_candidate_trusted),
      gyro_omega_rad_s,
      prior_phase_rate_valid, prior_phase_omega_rad_s);
    if (rotation_model.phase_reset) {
      gyro_phase_scheduler.reset();
      gyro_face_selector.reset();
      event_last_fired_cycle = -1;
      xuc_one_face_armed = true;
      tools::logger()->warn(
        "[MODEL] phase reset mode={} ekf_omega={:+.3f} pll_omega={:+.3f} "
        "rate_delta={:.3f}",
        auto_aim::RotationModelManager::name(rotation_model.mode),
        gyro_omega_rad_s, prior_phase_omega_rad_s,
        rotation_model.rate_disagreement_rad_s);
    }
    if (rotation_model.mode_changed) {
      tools::logger()->info(
        "[MODEL] switched to {} ekf_omega={:+.3f}rad_s",
        auto_aim::RotationModelManager::name(rotation_model.mode),
        gyro_omega_rad_s);
    }
    vehicle_model_fresh = tracker.state() == "tracking" && tracker.measurement_updated() &&
      fresh_primary != nullptr && fresh_primary->pose_valid &&
      frame_pose_context_matches && (!xuc.is_sentinel() ||
        (frame_pose_capture.valid && frame_pose_now.valid)) && xuc_gyro_center_follow_enabled &&
      rotation_model.tracker_trusted &&
      rotation_model.mode != auto_aim::RotationModelManager::Mode::armor;
    gyro_model_fresh = vehicle_model_fresh &&
      rotation_model.mode ==
        auto_aim::RotationModelManager::Mode::high_speed_phase;
    gyro_phase_model_trusted = gyro_model_fresh &&
      rotation_model.phase_fire_trusted;
    if (vehicle_model_fresh) {
      // Both low/medium rotation and high-speed gyro share the same stable
      // vehicle-centre gimbal reference.  Only their firing clocks differ:
      // continuous future-face prediction below the high-speed threshold and
      // the physical phase-event scheduler above it.
      gyro_center_follow_active = true;
      xuc_gyro_center_memory_valid = true;
      xuc_gyro_center_memory_yaw = gyro_center_yaw;
      xuc_gyro_center_last_fresh = t;
    } else if (xuc_gyro_center_memory_valid) {
      const double center_age_s =
        std::chrono::duration<double>(t - xuc_gyro_center_last_fresh).count();
      if (center_age_s <= xuc_gyro_center_hold_s) {
        // A tracker reset must not make the gimbal chase whichever plate is
        // visible for one frame. Hold the last validated vehicle center, but
        // keep gyro_model_fresh=false so the fire gate remains closed.
        gyro_center_follow_active = true;
        gyro_center_yaw = xuc_gyro_center_memory_yaw;
      } else {
        xuc_gyro_center_memory_valid = false;
      }
    }
    if (gyro_center_follow_active != xuc_gyro_center_follow_logged) {
      tools::logger()->info(
        "[XUC][GYRO] center_follow={} tracker={} omega={:+.3f}rad_s radius={:.3f}m",
        gyro_center_follow_active, tracker.state(), gyro_omega_rad_s, gyro_radius_m);
      xuc_gyro_center_follow_logged = gyro_center_follow_active;
    }

    // 调试：打印 armors 和 targets 数量
    if (!competition_headless) {
      fmt::print("[DEBUG] armors={}, targets={}\n", armors.size(), targets.size());
    }

#ifdef AMENT_CMAKE_FOUND
    // 发布装甲板Marker（可视化检测结果）
    if (!armors.empty()) {
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        t.time_since_epoch()).count();
      rclcpp::Time ros_time(ns);

      visualization_msgs::msg::MarkerArray marker_array;
      int marker_id = 0;
      for (const auto & armor : armors) {
        if (!armor.pose_valid) continue;
        auto marker = visualizer->create_sphere_marker(
          "world", "armors", marker_id++,
          armor.xyz_in_world.x(),
          armor.xyz_in_world.y(),
          armor.xyz_in_world.z(),
          1.0, 0.0, 0.0, 0.8,  // 红色，80%不透明
          0.1,  // 10cm直径
          ros_time
        );
        marker_array.markers.push_back(marker);
      }
      visualizer->publish_marker_array(marker_array);
    }
#endif

    // Keep raw-measurement control associated with the currently followed
    // bearing.  On a fast blue target YOLO can return two armor candidates in
    // one frame; blindly taking list::front() makes the command jump between
    // candidates and breaks both slew and fire qualification.
    const auto_aim::Armor * direct_measurement_armor = !tracker.measurement_updated()
      ? nullptr : auto_aim::select_direct_measurement(
      armors, tracker.primary_detection_index(), true,
      direct_yaw_initialized ? std::optional<double>(xuc.is_sentinel()
        ? sentinel_geometry.world_bearing_for_sensor_yaw(direct_yaw_filtered, frame_pose_now.rpy_rad)
        : direct_yaw_filtered) : std::nullopt,
      targets.empty() ? auto_aim::ArmorName::not_armor : targets.front().name,
      targets.empty() ? auto_aim::ArmorType::small : targets.front().armor_type);
    Eigen::Vector2d frame_image_motion = Eigen::Vector2d::Zero();
    if (xuc.is_sentinel() && direct_measurement_armor != nullptr) {
      const auto delta = frame_pose_context_matches && frame_pose_now.valid
        ? solver.image_angular_delta(direct_measurement_armor->xyz_in_world,
            solver.R_gimbal2world(), frame_current_rotation, img.size())
        : std::nullopt;
      if (delta) frame_image_motion = *delta;
      else direct_measurement_armor = nullptr;
    }

    // Use the rigid-body geometry as the range source for trajectory timing.
    // At a face-centre gyro event the near armor is approximately one radius
    // closer than the vehicle centre.  Raw oblique-PnP depth is only a small,
    // bounded correction; the latest field log ranged from 3.24 to 5.92 m for
    // a nominal 5 m target and otherwise injected more than 30 degrees of
    // time-of-flight phase error.
    double raw_pnp_range_m = NAN;
    if (direct_measurement_armor != nullptr) {
      raw_pnp_range_m = std::hypot(
        direct_measurement_armor->xyz_in_world[0],
        direct_measurement_armor->xyz_in_world[1]);
    }
    double modeled_armor_range_m = NAN;
    if (!targets.empty()) {
      const auto tracked_x = targets.front().ekf_x();
      if (gyro_model_fresh && tracked_x.size() >= 9) {
        const double center_range_m = std::hypot(tracked_x[0], tracked_x[2]);
        if (std::isfinite(center_range_m) && std::isfinite(tracked_x[8])) {
          modeled_armor_range_m = center_range_m - std::abs(tracked_x[8]);
        }
      } else {
        const int primary_face_id = tracker.primary_face_id();
        const auto modeled_faces = targets.front().armor_xyza_list();
        if (primary_face_id >= 0 &&
            primary_face_id < static_cast<int>(modeled_faces.size())) {
          modeled_armor_range_m = std::hypot(
            modeled_faces[primary_face_id][0], modeled_faces[primary_face_id][1]);
        }
      }
    }
    const auto robust_range = robust_range_filter.update(
      model_now_s, modeled_armor_range_m, raw_pnp_range_m);
    const double robust_target_range_m = robust_range.valid
      ? robust_range.range_m
      : raw_pnp_range_m;
    static int range_log_counter = 0;
    if (++range_log_counter % 50 == 0) {
      tools::logger()->info(
        "[RANGE] raw={:.3f}m model={:.3f}m filtered={:.3f}m "
        "model_used={} raw_limited={}",
        raw_pnp_range_m, modeled_armor_range_m, robust_target_range_m,
        robust_range.used_model ? 1 : 0, robust_range.raw_limited ? 1 : 0);
    }

    auto_aim::AimPredictionTrace aim_trace;
    auto command = aimer.aim(targets, t, cboard.bullet_speed, true, &aim_trace,
      vehicle_model_fresh && !gyro_model_fresh
        ? auto_aim::AimGeometry::whole_ekf : auto_aim::AimGeometry::existing);
    const auto native_world_command = command;  // to_now=true，生成当前时刻命令
    // All Sentinel actuator filters, tracking PI and serial errors use sensor
    // Euler angles. Only the target model / ballistics live in NAV coordinates.
    if (xuc.is_sentinel() && command.control) {
      const auto sensor_angles = frame_pose_now.valid
        ? sentinel_geometry.sensor_angles_for_world_ray(
            command.yaw, command.pitch, frame_pose_now.rpy_rad)
        : std::nullopt;
      if (sensor_angles) {
        command.yaw = (*sensor_angles)[0];
        command.pitch = (*sensor_angles)[1];
      } else {
        command.control = command.shoot = false;
      }
    }
    const bool native_prediction_valid =
      command.control && !targets.empty() && std::isfinite(command.yaw) &&
      aim_trace.valid && aim_trace.converged;
    const bool native_prediction_trusted =
      native_prediction_valid && tracker_converged && (yaw_motion_model_trusted ||
        (whole_vehicle_candidate_trusted && aim_trace.whole_ekf_geometry));
    const double native_predicted_yaw = command.yaw;
    const double calibrated_predicted_yaw = auto_aim::calibrated_fire_yaw(
      native_predicted_yaw, xuc.is_sentinel() ? direct_yaw_command_correction_rad : 0.0,
      xuc_gyro_predicted_yaw_trim_rad);
    double gyro_predicted_impact_yaw_error_rad = INFINITY;
    bool gyro_predicted_impact_yaw_valid = false;
    bool gyro_event_schedule_valid = false;
    bool gyro_event_interval_safe = false;
    std::int64_t gyro_event_hit_cycle = -1;
    double gyro_event_lead_error_s = INFINITY;
    double gyro_event_face_period_s = NAN;
    double gyro_event_half_window_rad = NAN;
    double gyro_event_worst_phase_rad = NAN;
    double gyro_event_combined_phase_rad = NAN;
    double gyro_event_phase_uncertainty_rad = NAN;
    bool gyro_event_face_valid = false;
    int gyro_event_face_id = -1;
    double gyro_event_face_alignment_rad = INFINITY;
    double gyro_event_gimbal_tracking_error_rad = INFINITY;
    std::chrono::steady_clock::time_point gyro_event_command_due_time{};

    // Phase 2D.5 integration guard.  The detector pose is much more stable
    // than the current high-dynamic EKF tuning at close range.  Keep the
    // complete Tracker/Aimer prediction path, but prevent a divergent state
    // from commanding a yaw far away from the armor actually in the image.
    // If the tracker briefly rejects convergence while a valid enemy armor is
    // still detected, degrade to measurement-only yaw tracking and never fire.
    bool measurement_fallback = false;
    bool direct_yaw_measurement_fresh = false;
    bool direct_pitch_measurement_fresh = false;
    double direct_yaw_pixel_error_rad = INFINITY;
    double stationary_yaw_fire_error_rad = INFINITY;
    double direct_pitch_pixel_error_rad = INFINITY;
    double direct_pitch_raw_pixel_error_rad = INFINITY;
    double ballistic_pitch_compensation_rad = 0.0;
    double ballistic_camera_range_m = NAN;
    double ballistic_total_holdover_m = 0.0;
    // A newly constructed tracker returns provisional targets before its
    // normal convergence window.  Keep those detections visible, but never let
    // a one-frame pose regain actuator authority after a genuine track loss.
    if (direct_yaw_guard_enabled) {
      bool valid_measurement = false;
      double measured_yaw = 0.0;
      if (direct_measurement_armor != nullptr) {
        const auto & measured_armor = *direct_measurement_armor;
        measured_yaw = gyro_center_follow_active
          ? gyro_center_yaw
          : std::atan2(measured_armor.xyz_in_world.y(), measured_armor.xyz_in_world.x());
        if (xuc.is_sentinel()) {
          measured_yaw = frame_pose_now.valid
            ? sentinel_geometry.sensor_yaw_for_world_bearing(measured_yaw, frame_pose_now.rpy_rad)
            : NAN;
        }
        valid_measurement = std::isfinite(measured_yaw);
        if (valid_measurement && direct_yaw_focal_x_px > 0.0 && !img.empty()) {
          double center_x_px = measured_armor.center.x;
          if (!measured_armor.points.empty()) {
            center_x_px = 0.0;
            for (const auto & point : measured_armor.points) center_x_px += point.x;
            center_x_px /= static_cast<double>(measured_armor.points.size());
          }
          const double raw_yaw_pixel_error_rad = std::atan2(
            center_x_px - 0.5 * static_cast<double>(img.cols), direct_yaw_focal_x_px);
          // On the installed yaw convention, a negative impact correction moves
          // the armor to image-left by the same angle.  Fire on the residual from
          // that requested offset, not on distance from the image centre.
          direct_yaw_pixel_error_rad =
            raw_yaw_pixel_error_rad + frame_image_motion[0] - direct_yaw_impact_correction_rad;
          stationary_yaw_fire_error_rad = direct_yaw_pixel_error_rad;
          if (xuc.is_sentinel()) {
            const double reference = sentinel_geometry.calibrated_yaw_for_world_point(
              measured_armor.xyz_in_world, frame_pose_now.rpy_rad, direct_yaw_command_correction_rad);
            stationary_yaw_fire_error_rad = std::remainder(
              reference - frame_pose_now.rpy_rad[2], 2.0 * M_PI);
          }
          direct_yaw_measurement_fresh = std::isfinite(direct_yaw_pixel_error_rad) &&
            std::isfinite(stationary_yaw_fire_error_rad);
        }
      }

      const bool previously_confirmed = direct_yaw_control_gate.confirmed();
      const double visual_gap_s = direct_yaw_initialized
        ? std::chrono::duration<double>(t - direct_yaw_last_measurement).count()
        : INFINITY;
      const bool direct_control_allowed = direct_yaw_control_gate.update(
        !targets.empty(), valid_measurement, tracker_converged,
        direct_yaw_initialized && visual_gap_s <= direct_yaw_hold_s);
      if (direct_control_allowed && !previously_confirmed) {
        tools::logger()->info("[XUC][YAW] tracker convergence confirmed; actuator reacquired");
      }

      if (valid_measurement) {
        if (!direct_yaw_initialized) {
          direct_yaw_filtered = measured_yaw;
          direct_yaw_initialized = true;
        } else {
          // Cap dt so a camera stall or a long loss cannot turn the rate limit
          // into one large step on the first recovered frame.
          const double dt = std::clamp(
            std::chrono::duration<double>(t - direct_yaw_last_update).count(),
            0.0, 0.05);
          const double yaw_error =
            std::remainder(measured_yaw - direct_yaw_filtered, 2.0 * M_PI);
          const double alpha = direct_yaw_filter_tau_s > 0.0
            ? 1.0 - std::exp(-dt / direct_yaw_filter_tau_s)
            : 1.0;
          double yaw_step = alpha * yaw_error;
          if (direct_yaw_max_rate_rad_s > 0.0) {
            const double max_step = direct_yaw_max_rate_rad_s * dt;
            yaw_step = std::clamp(yaw_step, -max_step, max_step);
          }
          direct_yaw_filtered = std::remainder(
            direct_yaw_filtered + yaw_step, 2.0 * M_PI);
        }
        direct_yaw_last_update = t;
        direct_yaw_last_measurement = t;
        if (direct_yaw_long_loss) {
          tools::logger()->info("[XUC][YAW] measurement reacquired with continuous slew");
        }
        direct_yaw_long_loss = false;

        measurement_fallback = true;
        command.shoot = false;
        command.control = direct_control_allowed;
        if (!direct_control_allowed) {
          command.yaw = std::remainder(
            direct_yaw_filtered + direct_yaw_command_correction_rad, 2.0 * M_PI);
        } else if (gyro_center_follow_active) {
          // In every rotating-target mode the visible plate bearing oscillates
          // around the vehicle. Following the EKF rotation center avoids
          // switching the gimbal at each of the four plate hand-overs.
          command.yaw = std::remainder(
            direct_yaw_filtered + direct_yaw_command_correction_rad,
            2.0 * M_PI);
        } else if (prediction_lead_limit_configured && native_prediction_trusted) {
          const double prediction_lead =
            std::remainder(native_predicted_yaw - measured_yaw, 2.0 * M_PI);
          command.yaw = std::remainder(
            direct_yaw_filtered + std::clamp(
              prediction_lead, -max_prediction_lead_rad, max_prediction_lead_rad) +
              direct_yaw_command_correction_rad,
            2.0 * M_PI);
        } else {
          command.yaw = std::remainder(
            direct_yaw_filtered + direct_yaw_command_correction_rad,
            2.0 * M_PI);
        }
      } else if (direct_yaw_initialized) {
        const double visual_gap =
          std::chrono::duration<double>(t - direct_yaw_last_measurement).count();
        if (visual_gap <= direct_yaw_hold_s) {
          // A short gap can retain previously confirmed control, but cannot
          // grant authority before convergence or after the tracker lost it.
          command.control = direct_control_allowed;
          command.shoot = false;
          command.yaw = std::remainder(
            direct_yaw_filtered + direct_yaw_command_correction_rad,
            2.0 * M_PI);
          if (prediction_lead_limit_configured && native_prediction_trusted) {
            const double prediction_delta =
              std::remainder(native_predicted_yaw - direct_yaw_filtered, 2.0 * M_PI);
            command.yaw = std::remainder(
              direct_yaw_filtered + std::clamp(
                prediction_delta, -max_prediction_lead_rad, max_prediction_lead_rad) +
                direct_yaw_command_correction_rad,
              2.0 * M_PI);
          }
          measurement_fallback = true;
        } else {
          // A genuine loss must stop control, but retaining the filtered yaw and
          // refreshing last_update prevents a full-angle jump on reacquisition.
          if (!direct_yaw_long_loss) {
            tools::logger()->warn("[XUC][YAW] visual loss: control stopped, yaw state retained");
          }
          direct_yaw_long_loss = true;
          direct_yaw_control_gate.reset();
          command.control = false;
          command.shoot = false;
          measurement_fallback = true;
        }
        direct_yaw_last_update = t;
      } else {
        command.control = false;
        command.shoot = false;
        measurement_fallback = true;
      }
    }
    if (direct_pitch_guard_enabled) {
      bool valid_pitch_measurement = false;
      double measured_pitch = 0.0;
      if (direct_measurement_armor != nullptr && direct_pitch_focal_y_px > 0.0 && !img.empty()) {
        const auto & measured_armor = *direct_measurement_armor;
        double center_y_px = measured_armor.center.y;
        if (!measured_armor.points.empty()) {
          center_y_px = 0.0;
          for (const auto & point : measured_armor.points) center_y_px += point.y;
          center_y_px /= static_cast<double>(measured_armor.points.size());
        }
        const double observed_pixel_pitch_error = std::atan2(
          center_y_px - 0.5 * static_cast<double>(img.rows),
          direct_pitch_focal_y_px);
        direct_pitch_raw_pixel_error_rad = observed_pixel_pitch_error;
        const double pixel_pitch_error = observed_pixel_pitch_error + frame_image_motion[1];

        if (xuc_ballistic_pitch_enabled) {
          const double raw_camera_range_m = std::hypot(
            measured_armor.xyz_in_gimbal.x(), measured_armor.xyz_in_gimbal.y());
          ballistic_camera_range_m = std::isfinite(robust_target_range_m)
            ? robust_target_range_m
            : raw_camera_range_m;
          if (std::isfinite(ballistic_camera_range_m) && ballistic_camera_range_m >= 0.10) {
            // Reuse the same closed-form low-arc solver that is exercised by
            // the Gestalt Tongji/SHtech Aimer.  The camera is forward of and
            // above the muzzle, while camera/gimbal Z points down.
            const double range_scale = raw_camera_range_m > 0.10
              ? ballistic_camera_range_m / raw_camera_range_m
              : 1.0;
            const double muzzle_x_m = measured_armor.xyz_in_gimbal.x() * range_scale +
              xuc_camera_forward_of_muzzle_m;
            const double muzzle_y_m = measured_armor.xyz_in_gimbal.y() * range_scale;
            const double muzzle_range_m = std::hypot(muzzle_x_m, muzzle_y_m);
            const double target_height_from_muzzle_m =
              -measured_armor.xyz_in_gimbal.z() + xuc_camera_above_muzzle_m;
            const double camera_los_up_rad = std::atan2(
              -measured_armor.xyz_in_gimbal.z(), ballistic_camera_range_m);
            const double reference_muzzle_range_m =
              xuc_ballistic_reference_camera_range_m + xuc_camera_forward_of_muzzle_m;
            const double reference_measured_compensation_rad = std::atan2(
              xuc_ballistic_reference_total_drop_m, reference_muzzle_range_m);
            const tools::Trajectory reference_trajectory(
              xuc_ballistic_reference_projectile_speed_mps,
              reference_muzzle_range_m,
              xuc_camera_above_muzzle_m);
            const tools::Trajectory current_trajectory(
              xuc_projectile_speed_mps, muzzle_range_m, target_height_from_muzzle_m);
            if (!reference_trajectory.unsolvable && !current_trajectory.unsolvable) {
              // The one real-shot datum calibrates only the fixed mechanical /
              // boresight bias.  Distance, height, gravity and flight time are
              // then solved physically instead of extrapolating a measured miss
              // with distance squared.
              const double fixed_boresight_bias_rad =
                reference_measured_compensation_rad - reference_trajectory.pitch;
              // Sentinel field trim may place the desired impact below the
              // zero-holdover line. Keep that signed correction within the
              // existing absolute bound instead of clipping it to zero.
              ballistic_pitch_compensation_rad = std::clamp(
                fixed_boresight_bias_rad + current_trajectory.pitch - camera_los_up_rad +
                  xuc_ballistic_pitch_trim_rad,
                xuc.is_sentinel() ? -xuc_ballistic_max_compensation_rad : 0.0,
                xuc_ballistic_max_compensation_rad);
              ballistic_total_holdover_m =
                std::tan(ballistic_pitch_compensation_rad) * muzzle_range_m;
            } else {
              ballistic_pitch_compensation_rad = NAN;
              ballistic_total_holdover_m = NAN;
            }
          } else {
            ballistic_pitch_compensation_rad = NAN;
          }
        }
        const double compensated_pitch_error =
          pixel_pitch_error - ballistic_pitch_compensation_rad;
        // Physical acceptance established that a target below image centre
        // requires a larger lower-board IMU pitch target. Closing the loop in
        // image space avoids the currently inconsistent pitch hand-eye offset.
        // Positive ballistic compensation deliberately settles the armor below
        // image centre so the muzzle is elevated above the visual line of sight.
        measured_pitch = (xuc.is_sentinel() ? frame_pose_now.rpy_rad[1] : xuc.imu_pitch()) +
          direct_pitch_error_gain * compensated_pitch_error;
        valid_pitch_measurement = std::isfinite(measured_pitch);
        direct_pitch_pixel_error_rad = compensated_pitch_error;
        direct_pitch_measurement_fresh = valid_pitch_measurement &&
          std::isfinite(direct_pitch_pixel_error_rad);
      }

      if (valid_pitch_measurement) {
        if (!direct_pitch_initialized) {
          direct_pitch_filtered = measured_pitch;
          direct_pitch_initialized = true;
        } else {
          const double dt = std::clamp(
            std::chrono::duration<double>(t - direct_pitch_last_update).count(),
            0.0, 0.05);
          const double pitch_error = measured_pitch - direct_pitch_filtered;
          const double alpha = direct_pitch_filter_tau_s > 0.0
            ? 1.0 - std::exp(-dt / direct_pitch_filter_tau_s)
            : 1.0;
          double pitch_step = alpha * pitch_error;
          if (direct_pitch_max_rate_rad_s > 0.0) {
            const double max_step = direct_pitch_max_rate_rad_s * dt;
            pitch_step = std::clamp(pitch_step, -max_step, max_step);
          }
          direct_pitch_filtered += pitch_step;
        }
        direct_pitch_last_update = t;
        direct_pitch_last_measurement = t;
        if (direct_pitch_long_loss) {
          tools::logger()->info("[XUC][PITCH] measurement reacquired with continuous slew");
        }
        direct_pitch_long_loss = false;
        command.pitch = direct_pitch_filtered;
      } else if (direct_pitch_initialized) {
        const double visual_gap =
          std::chrono::duration<double>(t - direct_pitch_last_measurement).count();
        if (visual_gap <= direct_pitch_hold_s) {
          command.pitch = direct_pitch_filtered;
        } else {
          if (!direct_pitch_long_loss) {
            tools::logger()->warn("[XUC][PITCH] visual loss: control stopped, pitch state retained");
          }
          direct_pitch_long_loss = true;
          command.control = false;
          command.shoot = false;
        }
        direct_pitch_last_update = t;
      } else {
        command.control = false;
        command.shoot = false;
      }
    }

    // Bound every active yaw reference, including the first AUTO/reacquisition
    // frame. A lower-board armed bit must never allow an unconditioned target
    // through while the inner position loop is being enabled.
    const bool lower_yaw_armed_now = xuc.enabled() && (xuc.is_sentinel() ? frame_pose_now.valid : xuc.rx_valid()) &&
      (xuc.is_sentinel() ? frame_pose_now.mode : xuc.mode()) == static_cast<uint8_t>(io::Mode::auto_aim) &&
      (xuc.is_sentinel() ? frame_pose_now.lower_yaw_armed : xuc.lower_yaw_armed());
    if (final_yaw_command_max_rate_rad_s > 0.0 && xuc.enabled()) {
      const auto limiter_now = std::chrono::steady_clock::now();
      const double actual_command_yaw = (xuc.is_sentinel() ? frame_pose_now.valid : xuc.rx_valid())
        ? (xuc.is_sentinel() ? frame_pose_now.rpy_rad[2]
                             : xuc.raw_yaw_correction_to_command(xuc.raw_imu_yaw())) : NAN;
      const bool yaw_control_active = command.control && (xuc.is_sentinel() ? frame_pose_now.valid : xuc.rx_valid()) &&
        (xuc.is_sentinel() ? frame_pose_now.mode : xuc.mode()) == static_cast<uint8_t>(io::Mode::auto_aim);
      const auto limited_yaw = final_yaw_limiter.update(
        command.yaw, actual_command_yaw, yaw_control_active, lower_yaw_armed_now,
        limiter_now);
      if (limited_yaw) command.yaw = *limited_yaw;
      else if (yaw_control_active) command.control = command.shoot = false;
    }
    if (xuc.enabled() && direct_pitch_guard_enabled) {
      // Use the same command basis as the direct-pitch measurement above.
      // Sentinel send_sentinel inverses its IMU sign/offset before transmission;
      // the first target therefore has zero raw pitch error. RMV has already
      // returned through run_rmv_debug and does not execute this path.
      const bool pitch_control_active = command.control && (xuc.is_sentinel() ? frame_pose_now.valid : xuc.rx_valid()) &&
        (xuc.is_sentinel() ? frame_pose_now.mode : xuc.mode()) == static_cast<uint8_t>(io::Mode::auto_aim);
      const auto limited_pitch = final_pitch_limiter.update(
        command.pitch, (xuc.is_sentinel() ? frame_pose_now.valid : xuc.rx_valid()) ? (xuc.is_sentinel() ? frame_pose_now.rpy_rad[1] : xuc.imu_pitch()) : NAN,
        pitch_control_active, lower_yaw_armed_now, std::chrono::steady_clock::now());
      if (limited_pitch) command.pitch = *limited_pitch;
      else if (pitch_control_active) command.control = command.shoot = false;
    }
    const Eigen::Vector3d actuator_ypr = xuc.is_sentinel()
      ? Eigen::Vector3d(frame_pose_now.rpy_rad[2], frame_pose_now.rpy_rad[1], frame_pose_now.rpy_rad[0])
      : current_ypr;
    const bool native_shoot = shooter.shoot(command, aimer, targets, actuator_ypr);
    bool shot_uses_one_face_policy = false;
    nlohmann::json fire_gate_diagnostics;
    if (xuc_fire_guard_enabled) {
      bool face_valid = false;
      double face_error_rad = INFINITY;
      if (!targets.empty() && aimer.debug_aim_point.valid) {
        const auto tracked_x = targets.front().ekf_x();
        if (tracked_x.size() >= 3) {
          const double center_bearing = std::atan2(tracked_x[2], tracked_x[0]);
          face_error_rad = std::abs(std::remainder(
            aimer.debug_aim_point.xyza[3] - center_bearing, 2.0 * M_PI));
          face_valid = std::isfinite(face_error_rad);
        }
      }
      const bool gyro_predicted_fire_active =
        vehicle_model_fresh && gyro_model_fresh &&
        xuc_gyro_predicted_fire_enabled;
      const bool vehicle_predicted_fire_active =
        vehicle_model_fresh && !gyro_model_fresh &&
        xuc_gyro_predicted_fire_enabled;
      // One-shot-per-face is meaningful only for the high-speed phase clock.
      // At low/medium speed, a valid aim window may last much longer than one
      // face passage; let feeder feedback re-arm repeated one-frame pulses.
      const bool one_face_policy_active =
        xuc_one_face_once_fire && gyro_predicted_fire_active;
      shot_uses_one_face_policy = one_face_policy_active;
      const bool predictive_fire_active =
        gyro_predicted_fire_active || vehicle_predicted_fire_active;
      const bool feedback_cadence_gate_open =
        !xuc.enabled() || xuc.is_sentinel() ||
        (!xuc_pending_shot && xuc_active_shot_id == 0 &&
         (!xuc_feeder_activity_seen ||
          (!xuc_feeder_active &&
           std::chrono::duration<double>(t - xuc_feeder_last_active).count() >=
             xuc_gyro_post_feed_recovery_s)));
      if (xuc.enabled() && !feedback_cadence_gate_open) {
        if (!xuc_gyro_cadence_hold_logged) {
          tools::logger()->info(
            "[XUC][FIRE] request inhibited until feeder is available");
          xuc_gyro_cadence_hold_logged = true;
        }
      } else if (xuc_gyro_cadence_hold_logged) {
        tools::logger()->info(
          "[XUC][FIRE] feeder available; fresh qualification required");
        xuc_gyro_cadence_hold_logged = false;
      }
      if (predictive_fire_active && native_prediction_valid) {
        // native_predicted_yaw is produced from the same future AimPoint used by
        // face_error_rad and already includes configured actuation delay plus the
        // iterated projectile flight time. Compare it with actual gimbal attitude,
        // not the centre-follow command, so servo lag is included in qualification.
        // Sentinel inverse geometry has expressed both in sensor yaw. Include
        // the same fixed correction as following, once, before event trim.
        gyro_predicted_impact_yaw_error_rad = std::remainder(
          calibrated_predicted_yaw - actuator_ypr[0],
          2.0 * M_PI);
        gyro_predicted_impact_yaw_valid =
          std::isfinite(gyro_predicted_impact_yaw_error_rad);
      }
      const bool convergence_gate_open =
        !xuc_fire_require_tracker_converged || tracker_converged;
      const bool stationary_yaw_gate_open = direct_yaw_measurement_fresh &&
        std::abs(stationary_yaw_fire_error_rad) <= xuc_fire_yaw_tolerance_rad;
      // A held gyro centre is useful for smooth reacquisition, but it is neither
      // a fresh moving model nor a stationary-target solution. Do not fall through
      // to the stationary fire policy until centre-memory mode has ended.
      const bool motion_transition_gate_open =
        !gyro_center_follow_active || vehicle_model_fresh;
      // ---- Event-clock phase scheduler ----
      // Fold the EKF body yaw modulo one face (pi/2).  Every armor id therefore
      // shares the same physical front-center zero and a detector association
      // switch cannot move the event clock by an entire face.  Raw PnP armor
      // yaw is intentionally not used as the clock because field logs showed
      // paired long/short zero crossings and clustered hit/miss intervals.
      const bool ev_gyro_active = gyro_predicted_fire_active && gyro_model_fresh;
      if (ev_gyro_active && direct_measurement_armor != nullptr &&
          direct_yaw_measurement_fresh && std::isfinite(gyro_center_yaw) &&
          std::isfinite(gyro_body_phase_rad)) {
        const double observed_phase_error = std::remainder(
          gyro_body_phase_rad - gyro_center_yaw, M_PI / 2.0);
        const double observation_time_s = std::chrono::duration<double>(
          t.time_since_epoch()).count();
        const auto event_observation = gyro_phase_scheduler.observe(
          observation_time_s, observed_phase_error, gyro_omega_rad_s);
        if (event_observation.period_updated) {
          tools::logger()->info(
            "[GYROPLL] phase_zero cycle={} sample={:.1f} ms periods={} alias_guard={} "
            "filtered={:.1f} ms residual={:+.1f} ms correction={:+.1f} ms "
            "uncertainty={:.1f} ms "
            "locks={} locked={} reacquired={} rejects={} omega={:.3f} ekf_omega={:+.3f}",
            gyro_phase_scheduler.crossing_cycle(),
            event_observation.period_sample_s * 1000.0,
            event_observation.periods_elapsed,
            event_observation.omega_alias_guarded ? 1 : 0,
            event_observation.face_period_s * 1000.0,
            event_observation.phase_residual_s * 1000.0,
            event_observation.applied_phase_correction_s * 1000.0,
            event_observation.phase_uncertainty_s * 1000.0,
            event_observation.lock_updates,
            event_observation.locked ? 1 : 0,
            event_observation.reacquired ? 1 : 0,
            event_observation.consecutive_rejections,
            (M_PI / 2.0) / event_observation.face_period_s,
            gyro_omega_rad_s);
        } else if (event_observation.crossing && event_observation.rejected) {
          tools::logger()->warn(
            "[GYROPLL] rejected phase crossing sample={:.1f} ms periods={} alias_guard={} "
            "residual={:+.1f} ms rejects={} ekf_omega={:+.3f}",
            event_observation.period_sample_s * 1000.0,
            event_observation.periods_elapsed,
            event_observation.omega_alias_guarded ? 1 : 0,
            event_observation.phase_residual_s * 1000.0,
            event_observation.consecutive_rejections,
            gyro_omega_rad_s);
        }
      } else if (!gyro_center_follow_active) {
        gyro_phase_scheduler.reset();
        gyro_face_selector.reset();
        event_last_fired_cycle = -1;
      }
      if (!ev_gyro_active) gyro_face_selector.reset();

      // The flight-time estimate uses the current measured armor range.  The
      // interval gate below separately accounts for flight-time uncertainty.
      double event_flight_time_s = NAN;
      if (ev_gyro_active && direct_measurement_armor != nullptr) {
        const auto & measured_xyz = direct_measurement_armor->xyz_in_world;
        const double d_face_m = std::isfinite(robust_target_range_m)
          ? robust_target_range_m
          : std::hypot(measured_xyz[0], measured_xyz[1]);
        event_flight_time_s = d_face_m / xuc_projectile_speed_mps;
        const tools::Trajectory trj(
          xuc_projectile_speed_mps, d_face_m, measured_xyz[2]);
        if (!trj.unsolvable && trj.fly_time > 0.0) {
          event_flight_time_s = trj.fly_time;
        }
      }

      auto event_solution = auto_aim::GyroPhaseScheduler::Solution{};
      if (ev_gyro_active && std::isfinite(event_flight_time_s)) {
        const double decision_now_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        event_solution = gyro_phase_scheduler.solve(
          decision_now_s,
          shot_delay_adapter.delay_s(),
          shot_delay_adapter.delay_min_s(),
          shot_delay_adapter.delay_max_s(),
          event_flight_time_s,
          xuc_event_flight_time_uncertainty_s,
          gyro_radius_m,
          xuc_event_armor_width_m,
          xuc_event_armor_edge_margin_m,
          xuc_event_model_phase_uncertainty_rad);
        if (event_solution.valid) {
          gyro_event_schedule_valid = true;
          // The PLL is tied to directly observed physical face crossings.  Use
          // its temporal uncertainty as the primary event gate.  Deployments
          // that trust the EKF face phase may additionally require the harder
          // combined face-consistency check below.
          const double relaxed_half_window_rad =
            event_solution.half_hit_window_rad + xuc_event_phase_grace_rad;
          gyro_event_interval_safe =
            !xuc_event_require_face_consistency &&
            event_solution.worst_interval_phase_rad <= relaxed_half_window_rad;
          gyro_event_hit_cycle = event_solution.hit_cycle;
          gyro_event_lead_error_s = event_solution.time_until_command_s;
          gyro_event_face_period_s = event_solution.face_period_s;
          gyro_event_half_window_rad = event_solution.half_hit_window_rad;
          gyro_event_worst_phase_rad = event_solution.worst_interval_phase_rad;
          gyro_event_phase_uncertainty_rad = event_solution.phase_uncertainty_rad;
          gyro_event_command_due_time = std::chrono::steady_clock::time_point(
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(event_solution.command_due_time_s)));

          // Predict the complete rigid body to this exact impact event, then
          // commit one physical face id.  The commitment is held for repeated
          // frames of the same event cycle and hands over only at the next
          // cycle (or if the prior face becomes geometrically impossible).
          if (!targets.empty()) {
            auto future_target = targets.front();
            const double processing_age_s = std::max(
              0.0, std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t).count());
            future_target.predict(
              processing_age_s + event_solution.time_until_hit_center_s);
            const auto future_x = future_target.ekf_x();
            if (future_x.size() >= 3) {
              const double future_center_yaw = std::atan2(future_x[2], future_x[0]);
              std::vector<auto_aim::GyroFaceSelector::Candidate> face_candidates;
              const auto future_faces = future_target.armor_xyza_list();
              face_candidates.reserve(future_faces.size());
              for (std::size_t face_id = 0; face_id < future_faces.size(); ++face_id) {
                face_candidates.push_back({
                  static_cast<int>(face_id), future_faces[face_id][3]});
              }
              const auto face_selection = gyro_face_selector.select(
                face_candidates, future_center_yaw, gyro_event_hit_cycle);
              gyro_event_face_valid = face_selection.valid;
              gyro_event_face_id = face_selection.face_id;
              gyro_event_face_alignment_rad = face_selection.alignment_rad;
              if (face_selection.valid) {
                // The temporal PLL interval and future-face alignment are
                // partially independent error sources.  Root-sum-square keeps
                // the face check meaningful without the excessive fire-rate
                // loss caused by linearly adding two conservative bounds.
                gyro_event_combined_phase_rad =
                  std::hypot(
                    event_solution.worst_interval_phase_rad,
                    face_selection.alignment_rad);
                if (xuc_event_require_face_consistency) {
                  gyro_event_interval_safe =
                    event_solution.worst_interval_phase_rad <= relaxed_half_window_rad &&
                    gyro_event_combined_phase_rad <= relaxed_half_window_rad;
                }
              }
              if (face_selection.valid && face_selection.changed) {
                tools::logger()->info(
                  "[FACELOCK] handoff cycle={} face={} alignment={:.2f}deg",
                  gyro_event_hit_cycle, gyro_event_face_id,
                  gyro_event_face_alignment_rad * 180.0 / M_PI);
              }
            }
          }

          gyro_event_gimbal_tracking_error_rad = std::abs(std::remainder(
            command.yaw - actuator_ypr[0], 2.0 * M_PI));

          static int event_schedule_log = 0;
          if (++event_schedule_log % 10 == 0) {
            tools::logger()->info(
              "[GYROSCHED] cycle={} due_in={:+.1f} ms hit_in={:.1f} ms "
              "period={:.1f} ms window={:.2f}deg worst={:.2f}deg "
              "combined={:.2f}deg "
              "phase_unc={:.2f}deg face={} face_align={:.2f}deg "
              "gimbal_err={:.2f}deg safe={}",
              gyro_event_hit_cycle, gyro_event_lead_error_s * 1000.0,
              event_solution.time_until_hit_center_s * 1000.0,
              gyro_event_face_period_s * 1000.0,
              gyro_event_half_window_rad * 180.0 / M_PI,
              gyro_event_worst_phase_rad * 180.0 / M_PI,
              gyro_event_combined_phase_rad * 180.0 / M_PI,
              gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
              gyro_event_face_id,
              gyro_event_face_alignment_rad * 180.0 / M_PI,
              gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
              gyro_event_interval_safe);
          }
        }
      }

      // A new physical face-center cycle re-arms a shot.  Outside AUTO the
      // estimator may continue learning the period, but fire state and cooldown
      // are never consumed.
      const bool auto_mode_active = mode == io::Mode::auto_aim;
      if (!auto_mode_active) {
        xuc_one_face_armed = true;
        event_last_fired_cycle = -1;
      } else if (one_face_policy_active) {
        if (gyro_event_schedule_valid) {
          // A PLL correction may move a scheduled event back to an already
          // consumed cycle.  Explicitly disarm in that case; merely rearming
          // on a different future cycle allowed duplicate requests in field
          // log cycles 41 and 146.
          xuc_one_face_armed = gyro_event_hit_cycle > event_last_fired_cycle;
        } else if (!gyro_event_schedule_valid &&
                   (!face_valid || face_error_rad >= xuc_one_face_rearm_rad)) {
          xuc_one_face_armed = true;
        }
      }
      if (!gyro_center_follow_active) xuc_one_face_armed = true;
      const bool one_face_gate_open = !one_face_policy_active ||
        (gyro_predicted_fire_active
           ? (gyro_event_schedule_valid
                ? xuc_one_face_armed
                : (!xuc_gyro_require_event_lock_for_fire && face_valid &&
                   face_error_rad <= xuc_one_face_limit_rad &&
                   xuc_one_face_armed))
           : (face_valid && face_error_rad <= xuc_one_face_limit_rad &&
              xuc_one_face_armed));

      // In event mode, issue only before the due instant and wait up to one
      // frame for a precise steady-clock send.  Never accept a late sample.
      const bool gyro_yaw_gate_native_open = gyro_predicted_impact_yaw_valid &&
        direct_yaw_measurement_fresh &&
        std::abs(gyro_predicted_impact_yaw_error_rad) <=
          xuc_gyro_predicted_yaw_tolerance_rad &&
        std::abs(direct_yaw_pixel_error_rad) <=
          xuc_gyro_visibility_yaw_tolerance_rad;
      const bool gyro_event_gate_open = gyro_event_schedule_valid &&
        gyro_phase_model_trusted && gyro_event_interval_safe &&
        (!xuc_event_require_face_consistency || gyro_event_face_valid) &&
        gyro_event_gimbal_tracking_error_rad <=
          xuc_event_gimbal_tracking_tolerance_rad &&
        gyro_event_lead_error_s >= 0.0 &&
        gyro_event_lead_error_s <= xuc_event_trigger_window_s;
      // Observe every eligible frame, including frames outside the angular
      // fire gate. Transport throttling/rearming never resets this history.
      double selection_phase = NAN, selection_omega = NAN;
      if (aim_trace.whole_ekf_geometry && !targets.empty() && aim_trace.valid && aim_trace.converged) {
        const auto & whole = targets.front().ekf().x;
        if (whole.size() == 11 && whole.allFinite()) {
          const double dt = aim_trace.prediction_ns * 1e-9 - model_now_s;
          const double center_yaw = std::atan2(whole[2] + whole[3] * dt, whole[0] + whole[1] * dt);
          selection_phase = std::remainder(aimer.debug_aim_point.xyza[3] - center_yaw, 2.0 * M_PI);
          selection_omega = whole[7];
        }
      }
      rotation_fire_window.update(
        vehicle_predicted_fire_active && auto_mode_active && tracker.identity_confirmed() &&
          native_prediction_valid && aim_trace.whole_ekf_geometry && direct_yaw_measurement_fresh &&
          direct_pitch_measurement_fresh && convergence_gate_open,
        tracker.diagnostics().rebuilt, tracker.track_id(), frame_pose_capture.generation,
        targets.empty() ? -1 : static_cast<int>(targets.front().name),
        targets.empty() ? -1 : static_cast<int>(targets.front().armor_type),
        targets.empty() ? 0 : targets.front().armor_num(), aim_trace.face_id,
        false, aimer.debug_aim_point.valid ? aimer.debug_aim_point.xyza[3] : NAN,
        gyro_predicted_impact_yaw_error_rad, t, xuc_gyro_fire_min_consecutive_frames,
        xuc_gyro_candidate_max_phase_step_rad, xuc_gyro_predicted_yaw_tolerance_rad,
        vision_frame_sequence, selection_phase, selection_omega,
        integration_yaml["comming_angle"].as<double>() / 57.3,
        integration_yaml["leaving_angle"].as<double>() / 57.3);
      const bool opportunity_window_open = rotation_fire_window.two_frames_ready &&
        (rotation_fire_window.current_frame_is_better || rotation_fire_window.exiting_selection_next_step);
      const bool yaw_gate_open = gyro_predicted_fire_active
        ? (gyro_event_schedule_valid
             ? gyro_event_gate_open
             : (!xuc_gyro_require_event_lock_for_fire &&
                gyro_yaw_gate_native_open))
        : (vehicle_predicted_fire_active
             ? gyro_yaw_gate_native_open
             : stationary_yaw_gate_open);
      const bool fire_candidate = tracker.identity_confirmed() &&
 integration_yaml["auto_fire"].as<bool>() && command.control &&
        auto_mode_active &&
        convergence_gate_open && one_face_gate_open && motion_transition_gate_open &&
        feedback_cadence_gate_open &&
        direct_yaw_measurement_fresh && direct_pitch_measurement_fresh &&
        yaw_gate_open &&
        std::abs(direct_pitch_pixel_error_rad) <= xuc_fire_pitch_tolerance_rad;
      if (vision_diagnostics.enabled()) fire_gate_diagnostics = {
        {"candidate", fire_candidate}, {"auto", auto_mode_active},
        {"converged", convergence_gate_open}, {"one_face", one_face_gate_open},
        {"motion", motion_transition_gate_open}, {"cadence", feedback_cadence_gate_open},
        {"yaw", yaw_gate_open}, {"pitch", std::abs(direct_pitch_pixel_error_rad) <= xuc_fire_pitch_tolerance_rad},
        {"identity", tracker.identity_confirmed()}, {"vehicle_prediction", vehicle_predicted_fire_active},
        {"phase_event", gyro_event_schedule_valid}, {"window_ready", rotation_fire_window.two_frames_ready},
        {"window_better_now", rotation_fire_window.current_frame_is_better},
        {"window_exit_next", rotation_fire_window.exiting_selection_next_step},
        {"window_open", opportunity_window_open}, {"window_frames", rotation_fire_window.frames},
        {"next_error_rad", rotation_fire_window.next_error_rad}, {"selection_phase_rad", selection_phase}};
      // Fire-gate diagnostic for both medium-speed prediction and the high-speed
      // event clock; low-speed dead zones were previously invisible here.
      static int b2_diag_log = 0;
      if (predictive_fire_active && ++b2_diag_log % 50 == 0) {
        tools::logger()->info(
          "[B2DIAG] auto={} control={} conv={} oneface_gate={} armed={} motion={} "
          "cadence={} dyaw_fresh={} dpitch_fresh={} yawgate={} event={} safe={} "
          "face={} face_ok={} face_align_deg={:.2f} phase_unc_deg={:.2f} "
          "gimbal_err_deg={:.2f} due_ms={:+.1f} window_deg={:.2f} "
          "worst_deg={:.2f} combined_deg={:.2f} "
          "yawerr_deg={:.2f} pitcherr_deg={:.2f}",
          auto_mode_active ? 1 : 0, command.control ? 1 : 0,
          convergence_gate_open ? 1 : 0,
          one_face_gate_open ? 1 : 0, xuc_one_face_armed ? 1 : 0,
          motion_transition_gate_open ? 1 : 0,
          feedback_cadence_gate_open ? 1 : 0,
          direct_yaw_measurement_fresh ? 1 : 0,
          direct_pitch_measurement_fresh ? 1 : 0,
          yaw_gate_open ? 1 : 0, gyro_event_schedule_valid ? 1 : 0,
          gyro_event_interval_safe ? 1 : 0,
          gyro_event_face_id, gyro_event_face_valid ? 1 : 0,
          gyro_event_face_alignment_rad * 180.0 / M_PI,
          gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
          gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
          gyro_event_lead_error_s * 1000.0,
          gyro_event_half_window_rad * 180.0 / M_PI,
          gyro_event_worst_phase_rad * 180.0 / M_PI,
          gyro_event_combined_phase_rad * 180.0 / M_PI,
          direct_yaw_pixel_error_rad * 180.0 / M_PI,
          direct_pitch_pixel_error_rad * 180.0 / M_PI);
      }
      if (fire_candidate) {
        const double candidate_face_yaw_rad = gyro_event_face_valid
          ? static_cast<double>(gyro_event_face_id) * M_PI / 2.0
          : aimer.debug_aim_point.xyza[3];
        if (predictive_fire_active && xuc_fire_candidate_active &&
            std::isfinite(xuc_fire_candidate_face_yaw_rad) &&
            std::isfinite(candidate_face_yaw_rad)) {
          const double phase_step_rad = std::abs(std::remainder(
            candidate_face_yaw_rad - xuc_fire_candidate_face_yaw_rad,
            2.0 * M_PI));
          if (phase_step_rad > xuc_gyro_candidate_max_phase_step_rad) {
            tools::logger()->info(
              "[XUC][FIRE] candidate face switched by {:.1f} deg; restarting window",
              phase_step_rad * 180.0 / M_PI);
            xuc_fire_candidate_active = false;
            xuc_fire_ready_logged = false;
            xuc_fire_candidate_frames = 0;
          }
        }
        if (!xuc_fire_candidate_active) {
          xuc_fire_candidate_active = true;
          xuc_fire_ready_logged = false;
          xuc_fire_candidate_frames = 1;
          xuc_fire_candidate_since = t;
          xuc_fire_candidate_face_yaw_rad = candidate_face_yaw_rad;
          tools::logger()->info("[XUC][FIRE] stable-window candidate started");
        } else {
          ++xuc_fire_candidate_frames;
          xuc_fire_candidate_face_yaw_rad = candidate_face_yaw_rad;
        }
        const double stable_time =
          std::chrono::duration<double>(t - xuc_fire_candidate_since).count();
        if (predictive_fire_active) {
          // Predictive fire always uses a one-frame pulse. High-speed phase mode
          // allows one pulse per physical face event; low/medium speed is
          // re-armed by completed feeder feedback, enabling controlled repeated
          // fire without turning a satisfied gate into a long shoot level.
          // Event mode: the dynamic-lead gate is a short time window (~1 frame
          // at high omega), so a single qualifying frame is sufficient to fire.
          // Native fallback (no event omega/tracker) keeps the 2-frame rule.
          const int ev_need_frames = gyro_event_schedule_valid
            ? 1
            : xuc_gyro_fire_min_consecutive_frames;
          command.shoot = (vehicle_predicted_fire_active
            ? opportunity_window_open : (xuc_fire_candidate_frames >= ev_need_frames)) &&
            !xuc_fire_ready_logged;
        } else {
          const double required_stable_s = one_face_policy_active
            ? xuc_gyro_fire_stable_s
            : xuc_fire_stable_s;
          // XUC also receives a one-frame pulse for stationary targets. Once
          // feeder feedback closes this shot, a still-valid gate starts the
          // next pulse immediately; no frame-rate-level command flood and no
          // fixed one-second software divider are involved.
          command.shoot = stable_time >= required_stable_s &&
            (!xuc.enabled() || xuc.is_sentinel() || !xuc_fire_ready_logged);
        }
        if (command.shoot && !xuc_fire_ready_logged) {
          tools::logger()->info(
            "[XUC][FIRE] qualified after {:.0f} ms/{} frames, "
            "current_pixel_error_deg=({:.2f},{:.2f}) "
            "predicted_impact_yaw_error_deg={:.2f} gyro_predict={} "
            "face_error={:.2f}deg committed_face={} face_align={:.2f}deg "
            "phase_unc={:.2f}deg gimbal_err={:.2f}deg omega={:+.3f}rad_s armed={}",
            stable_time * 1000.0,
            xuc_fire_candidate_frames,
            direct_yaw_pixel_error_rad * 180.0 / M_PI,
            direct_pitch_pixel_error_rad * 180.0 / M_PI,
            gyro_predicted_impact_yaw_error_rad * 180.0 / M_PI,
            predictive_fire_active, face_error_rad * 180.0 / M_PI,
            gyro_event_face_id, gyro_event_face_alignment_rad * 180.0 / M_PI,
            gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
            gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI,
            gyro_omega_rad_s, xuc_one_face_armed);
          // Event commitment occurs only after the serial write succeeds.
        }
      } else {
        if (xuc_fire_candidate_active) {
          tools::logger()->info("[XUC][FIRE] stable window reset");
        }
        xuc_fire_candidate_active = false;
        xuc_fire_ready_logged = false;
        xuc_fire_candidate_frames = 0;
        xuc_fire_candidate_face_yaw_rad = INFINITY;
        command.shoot = false;
      }
    } else {
      command.shoot = native_shoot;
      if (measurement_fallback) command.shoot = false;
    }

    // Final actuator tracking layer.  Form the error before adding PI output so
    // the integrator converges to the extra command needed to make actual raw IMU
    // yaw follow the ESKF/direct-guard reference instead of chasing its own
    // boosted output.  Reset on loss/exit to prevent stale wind-up at reacquire.
    double yaw_tracking_error_raw_rad = NAN;
    double yaw_tracking_output_raw_rad = 0.0;
    if (!lower_yaw_armed_now) {
      // The vision pipeline continues producing targets while the RC switch is
      // outside AUTO.  Never integrate that physically untrackable error and
      // then dump it into the gimbal on the AUTO edge.
      yaw_tracking_integral_raw_rad = 0.0;
      yaw_tracking_initialized = false;
      yaw_tracking_lower_armed = false;
    } else if (!yaw_tracking_lower_armed) {
      yaw_tracking_lower_armed = true;
      yaw_tracking_armed_since = std::chrono::steady_clock::now();
      yaw_tracking_last_update = yaw_tracking_armed_since;
      yaw_tracking_integral_raw_rad = 0.0;
      yaw_tracking_initialized = true;
    }

    if (yaw_tracking_pi_enabled && lower_yaw_armed_now &&
        yaw_tracking_lower_armed && (xuc.is_sentinel() ? frame_pose_now.valid : xuc.rx_valid()) && command.control &&
        std::isfinite(command.yaw)) {
      const auto pi_now = std::chrono::steady_clock::now();
      const double armed_age_s = std::chrono::duration<double>(
        pi_now - yaw_tracking_armed_since).count();
      const double dt = yaw_tracking_initialized &&
          armed_age_s >= yaw_tracking_arm_dwell_s
        ? std::clamp(
            std::chrono::duration<double>(pi_now - yaw_tracking_last_update).count(),
            0.0,
            0.05)
        : 0.0;
      yaw_tracking_last_update = pi_now;
      yaw_tracking_initialized = true;

      const double reference_raw_rad = xuc.command_yaw_to_raw(command.yaw);
      const double feedback_raw_rad = xuc.is_sentinel()
        ? xuc.command_yaw_to_raw(frame_pose_now.rpy_rad[2]) : xuc.raw_imu_yaw();
      yaw_tracking_error_raw_rad = std::remainder(
        reference_raw_rad - feedback_raw_rad, 2.0 * M_PI);
      const bool low_dynamic_measurement = direct_yaw_measurement_fresh &&
        tracker_converged &&
        (!gyro_model_fresh || std::abs(gyro_omega_rad_s) <
          xuc_gyro_omega_threshold_rad_s);
      if (armed_age_s >= yaw_tracking_arm_dwell_s &&
          low_dynamic_measurement &&
          std::isfinite(yaw_tracking_error_raw_rad) &&
          std::abs(yaw_tracking_error_raw_rad) <=
            yaw_tracking_integrate_error_limit_rad) {
        yaw_tracking_integral_raw_rad = std::clamp(
          yaw_tracking_integral_raw_rad +
            yaw_tracking_ki_per_s * yaw_tracking_error_raw_rad * dt,
          -yaw_tracking_integral_limit_rad,
          yaw_tracking_integral_limit_rad);
      }
      if (armed_age_s >= yaw_tracking_arm_dwell_s) {
        // Proportional action is permitted only on a fresh, low-dynamic
        // measurement.  During gyro motion retain at most the slowly learned
        // static trim; ESKF/phase prediction remains the sole dynamic lead.
        const double proportional_raw_rad = low_dynamic_measurement
          ? yaw_tracking_kp * yaw_tracking_error_raw_rad
          : 0.0;
        yaw_tracking_output_raw_rad = std::clamp(
          proportional_raw_rad + yaw_tracking_integral_raw_rad,
          -yaw_tracking_output_limit_rad,
          yaw_tracking_output_limit_rad);
      }
      command.yaw = std::remainder(
        command.yaw +
          xuc.raw_yaw_correction_to_command(yaw_tracking_output_raw_rad),
        2.0 * M_PI);
    } else {
      yaw_tracking_integral_raw_rad = 0.0;
      yaw_tracking_initialized = false;
      if (!lower_yaw_armed_now) yaw_tracking_lower_armed = false;
    }

    // Dry-run diagnostics: world-coordinate stability and final command.
    static int dryrun_log_counter = 0;
    if (++dryrun_log_counter % 20 == 0) {
      constexpr double RAD2DEG = 180.0 / M_PI;
      if (!targets.empty()) {
        auto tracked_xyza_list = targets.front().armor_xyza_list();
        if (!tracked_xyza_list.empty() && direct_measurement_armor != nullptr) {
          const auto & measured_gimbal = direct_measurement_armor->xyz_in_gimbal;
          const auto & measured_world = direct_measurement_armor->xyz_in_world;
          int measured_detection_index = 0;
          for (const auto & armor : armors) {
            if (&armor == direct_measurement_armor) break;
            ++measured_detection_index;
          }
          int tracked_face_id = -1;
          for (const auto & association : tracker.last_associations()) {
            if (association.detection_index == measured_detection_index &&
                association.face_id >= 0 &&
                association.face_id < static_cast<int>(tracked_xyza_list.size())) {
              tracked_face_id = association.face_id;
              break;
            }
          }
          const Eigen::Vector4d tracked = tracked_face_id >= 0
            ? tracked_xyza_list[tracked_face_id]
            : Eigen::Vector4d::Constant(NAN);
          double armor_center_y_px = direct_measurement_armor->center.y;
          if (!direct_measurement_armor->points.empty()) {
            armor_center_y_px = 0.0;
            for (const auto & point : direct_measurement_armor->points) armor_center_y_px += point.y;
            armor_center_y_px /= static_cast<double>(direct_measurement_armor->points.size());
          }
          const double pixel_pitch_error_deg = direct_pitch_focal_y_px > 0.0
            ? std::atan2(
                armor_center_y_px - 0.5 * static_cast<double>(img.rows),
                direct_pitch_focal_y_px) * RAD2DEG
            : 0.0;

          const double target_gimbal_yaw_deg =
            std::atan2(measured_gimbal.y(), measured_gimbal.x()) * RAD2DEG;
          const double target_gimbal_pitch_deg =
            std::atan2(
              measured_gimbal.z(),
              std::hypot(measured_gimbal.x(), measured_gimbal.y())) * RAD2DEG;
          const double target_world_yaw_deg =
            std::atan2(measured_world.y(), measured_world.x()) * RAD2DEG;
          const double target_world_pitch_deg =
            std::atan2(
              measured_world.z(),
              std::hypot(measured_world.x(), measured_world.y())) * RAD2DEG;

          tools::logger()->info(
            "[DRYRUN] measured_gimbal=({:.3f},{:.3f},{:.3f}) "
            "measured_world=({:.3f},{:.3f},{:.3f}) "
            "tracked_world=({:.3f},{:.3f},{:.3f}) "
            "target_gimbal_deg=({:.2f},{:.2f}) "
            "target_world_deg=({:.2f},{:.2f}) "
            "gimbal_deg=({:.2f},{:.2f}) "
            "armor_y_px={:.1f} pixel_error_deg=({:.2f},{:.2f}) "
            "ballistic=(range={:.3f}m holdover={:.3f}m comp={:.2f}deg residual={:.2f}deg) "
            "q_wxyz=({:.6f},{:.6f},{:.6f},{:.6f}) "
            "cmd_deg=({:.2f},{:.2f}) packed_pitch_deg={:.2f} "
            "yaw_pi=(error={:.2f}deg integral={:.2f}deg output={:.2f}deg) "
            "control={} shoot={} measurement_detection={} tracked_face={}",
            measured_gimbal.x(), measured_gimbal.y(), measured_gimbal.z(),
            measured_world.x(), measured_world.y(), measured_world.z(),
            tracked.x(), tracked.y(), tracked.z(),
            target_gimbal_yaw_deg, target_gimbal_pitch_deg,
            target_world_yaw_deg, target_world_pitch_deg,
            ypr[0] * RAD2DEG, ypr[1] * RAD2DEG,
            armor_center_y_px, direct_yaw_pixel_error_rad * RAD2DEG, pixel_pitch_error_deg,
            ballistic_camera_range_m, ballistic_total_holdover_m,
            ballistic_pitch_compensation_rad * RAD2DEG,
            direct_pitch_pixel_error_rad * RAD2DEG,
            q.w(), q.x(), q.y(), q.z(),
            command.yaw * RAD2DEG, command.pitch * RAD2DEG,
            xuc_pitch_sign_for_log * command.pitch * RAD2DEG,
            yaw_tracking_error_raw_rad * RAD2DEG,
            yaw_tracking_integral_raw_rad * RAD2DEG,
            yaw_tracking_output_raw_rad * RAD2DEG,
            command.control, command.shoot, measured_detection_index, tracked_face_id);
        }
      } else {
        tools::logger()->info(
          "[DRYRUN] target=none gimbal_deg=({:.2f},{:.2f}) control={} shoot={}",
          ypr[0] * RAD2DEG, ypr[1] * RAD2DEG,
          command.control, command.shoot);
      }
    }

    // Sentinel consumes a fire level and has no feeder ACK. The sender limits
    // successful fire writes and explicitly clears the level during cooldown.
    // For an event-qualified gyro shot, wait for the exact steady-clock due
    // instant instead of quantizing the request to the next camera frame.  A
    // sample that was already late before this deliberate wait is discarded;
    // the small tolerance only covers OS wake-up jitter after sleep_until().
    double gyro_event_send_lateness_ms = NAN;
    if (xuc.enabled() && command.shoot) {
      if (xuc.mode() != static_cast<uint8_t>(io::Mode::auto_aim)) {
        tools::logger()->warn(
          "[SHOT] request suppressed because lower-board mode={} is not AUTO",
          static_cast<int>(xuc.mode()));
        command.shoot = false;
      } else if (gyro_event_schedule_valid) {
        const auto wait_started = std::chrono::steady_clock::now();
        if (wait_started > gyro_event_command_due_time) {
          gyro_event_send_lateness_ms = std::chrono::duration<double, std::milli>(
            wait_started - gyro_event_command_due_time).count();
          tools::logger()->warn(
            "[GYROSCHED] cycle={} missed before send by {:.2f} ms; shot discarded",
            gyro_event_hit_cycle, gyro_event_send_lateness_ms);
          command.shoot = false;
        } else {
          std::this_thread::sleep_until(gyro_event_command_due_time);
          const auto woke_at = std::chrono::steady_clock::now();
          gyro_event_send_lateness_ms = std::chrono::duration<double, std::milli>(
            woke_at - gyro_event_command_due_time).count();
          if (gyro_event_send_lateness_ms >
              xuc_event_send_late_tolerance_s * 1000.0) {
            tools::logger()->warn(
              "[GYROSCHED] cycle={} wake late by {:.2f} ms; shot discarded",
              gyro_event_hit_cycle, gyro_event_send_lateness_ms);
            command.shoot = false;
          }
        }
      }
    }

    // A scheduled request can wait for part of a camera period. Recheck the
    // actual pose after that wait, using the last observed world point and the
    // full calibrated camera transform. This is motion compensation, not a new
    // target observation; an over-age image cannot qualify a shot.
    if (xuc.is_sentinel() && command.shoot) {
      const auto transmit_pose = xuc.pose_now();
      const auto transmit_rotation = sentinel_world_rotation(transmit_pose);
      const auto image_delta = direct_measurement_armor != nullptr && transmit_pose.valid
        ? solver.image_angular_delta(direct_measurement_armor->xyz_in_world,
            solver.R_gimbal2world(), transmit_rotation, img.size())
        : std::nullopt;
      const double latest_pitch_residual = image_delta
        ? direct_pitch_raw_pixel_error_rad + (*image_delta)[1] - ballistic_pitch_compensation_rad
        : INFINITY;
      const double stationary_reference = direct_measurement_armor != nullptr && transmit_pose.valid
        ? sentinel_geometry.calibrated_yaw_for_world_point(
            direct_measurement_armor->xyz_in_world, transmit_pose.rpy_rad,
            direct_yaw_command_correction_rad)
        : NAN;
      const double guarded_yaw = gyro_event_schedule_valid ? command.yaw :
        gyro_predicted_impact_yaw_valid ? calibrated_predicted_yaw :
        stationary_reference;
      const double yaw_tolerance = gyro_event_schedule_valid ? xuc_event_gimbal_tracking_tolerance_rad :
        gyro_predicted_impact_yaw_valid ? xuc_gyro_predicted_yaw_tolerance_rad : xuc_fire_yaw_tolerance_rad;
      const auto pose_guard = auto_aim::fire_pose_guard(frame_pose_now, transmit_pose,
        transmit_pose.rpy_rad[2], guarded_yaw, yaw_tolerance, latest_pitch_residual,
        xuc_fire_pitch_tolerance_rad);
      const auto image_age = std::chrono::steady_clock::now() - t;
      const bool image_fresh = image_age >= std::chrono::steady_clock::duration::zero() &&
        image_age <= std::chrono::nanoseconds(io::sentinel::PoseHistory::max_age_ns);
      if (!pose_guard.allowed || !image_fresh || !transmit_pose.control_ready ||
          transmit_pose.mode != static_cast<uint8_t>(io::Mode::auto_aim)) {
        command.shoot = false;
        tools::logger()->warn("[SHOT] pose_recheck_rejected reason={} image_fresh={} mode={}",
          static_cast<int>(pose_guard.reason), image_fresh, static_cast<int>(transmit_pose.mode));
      }
    }

    // Keep requested values separate from the transport result. A completed
    // serial write is observable here; actuator acceptance is not an ACK.
    nlohmann::json plot_data;
    plot_data["t"] = std::chrono::duration<double>(t - std::chrono::steady_clock::time_point()).count();
    plot_data["cmd_yaw"] = command.yaw * 180.0 / M_PI;
    plot_data["cmd_pitch"] =
      xuc_pitch_sign_for_log * command.pitch * 180.0 / M_PI;
    plot_data["yaw_tracking_error_raw_deg"] = std::isfinite(yaw_tracking_error_raw_rad)
      ? nlohmann::json(yaw_tracking_error_raw_rad * 180.0 / M_PI)
      : nlohmann::json(nullptr);
    plot_data["yaw_tracking_integral_raw_deg"] =
      yaw_tracking_integral_raw_rad * 180.0 / M_PI;
    plot_data["yaw_tracking_output_raw_deg"] =
      yaw_tracking_output_raw_rad * 180.0 / M_PI;
    plot_data["control"] = command.control;
    plot_data["shoot"] = command.shoot;
    plot_data["tx_result_known"] = false;
    plot_data["control_tx"] = false;
    plot_data["fire_tx"] = false;
    plot_data["friction_tx"] = false;
    if (gyro_predicted_impact_yaw_valid) {
      plot_data["gyro_predicted_impact_yaw_error_deg"] =
        gyro_predicted_impact_yaw_error_rad * 180.0 / M_PI;
    } else {
      plot_data["gyro_predicted_impact_yaw_error_deg"] = nullptr;
    }
    if (gyro_event_schedule_valid) {
      plot_data["gyro_event_due_ms"] = gyro_event_lead_error_s * 1000.0;
      plot_data["gyro_event_period_ms"] = gyro_event_face_period_s * 1000.0;
      plot_data["gyro_event_interval_safe"] = gyro_event_interval_safe;
      plot_data["gyro_event_half_window_deg"] =
        gyro_event_half_window_rad * 180.0 / M_PI;
      plot_data["gyro_event_worst_phase_deg"] =
        gyro_event_worst_phase_rad * 180.0 / M_PI;
      plot_data["gyro_event_combined_phase_deg"] =
        gyro_event_combined_phase_rad * 180.0 / M_PI;
      plot_data["gyro_event_phase_uncertainty_deg"] =
        gyro_event_phase_uncertainty_rad * 180.0 / M_PI;
      plot_data["gyro_event_face_id"] = gyro_event_face_id;
      plot_data["gyro_event_face_alignment_deg"] =
        gyro_event_face_alignment_rad * 180.0 / M_PI;
      plot_data["gyro_event_gimbal_tracking_error_deg"] =
        gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI;
    } else {
      plot_data["gyro_event_due_ms"] = nullptr;
      plot_data["gyro_event_period_ms"] = nullptr;
      plot_data["gyro_event_interval_safe"] = false;
      plot_data["gyro_event_half_window_deg"] = nullptr;
      plot_data["gyro_event_worst_phase_deg"] = nullptr;
      plot_data["gyro_event_combined_phase_deg"] = nullptr;
      plot_data["gyro_event_phase_uncertainty_deg"] = nullptr;
      plot_data["gyro_event_face_id"] = nullptr;
      plot_data["gyro_event_face_alignment_deg"] = nullptr;
      plot_data["gyro_event_gimbal_tracking_error_deg"] = nullptr;
    }
    // 电控的欧拉角（从MCU获取的姿态）
    plot_data["mcu_yaw"] = ypr[0] * 180.0 / M_PI;
    plot_data["mcu_pitch"] = ypr[1] * 180.0 / M_PI;
    plot_data["mcu_roll"] = ypr[2] * 180.0 / M_PI;
    // 电控的四元数（从MCU获取）
    plot_data["mcu_q_w"] = q.w();
    plot_data["mcu_q_x"] = q.x();
    plot_data["mcu_q_y"] = q.y();
    plot_data["mcu_q_z"] = q.z();
    if (xuc.is_sentinel()) {
      const auto feedback = xuc.sentinel_feedback();
      const auto gyro = xuc.gimbal_gyro_rad_s();
      plot_data["sentinel_link_valid"] = xuc.feedback_link_valid();
      plot_data["sentinel_pose_valid"] = xuc.rx_valid();
      plot_data["sentinel_boot_id"] = feedback.boot_id;
      plot_data["sentinel_p_time_us"] = feedback.gimbal.time_us;
      plot_data["sentinel_p_id"] = feedback.gimbal.id;
      plot_data["sentinel_p_flags"] = feedback.gimbal.flags;
      plot_data["sentinel_p_seq"] = feedback.gimbal.seq;
      plot_data["sentinel_p_raw_rpy_rad"] = {feedback.gimbal.roll, feedback.gimbal.pitch, feedback.gimbal.yaw};
      plot_data["sentinel_p_gyro_rad_s"] = gyro;
      plot_data["sentinel_c_time_us"] = feedback.chassis.time_us;
      plot_data["sentinel_c_flags"] = feedback.chassis.flags;
      plot_data["sentinel_c_raw_rpy_rad"] = {feedback.chassis.roll, feedback.chassis.pitch, feedback.chassis.yaw};
      plot_data["sentinel_aim_last_seq"] = feedback.link.aim_last_seq;
      plot_data["sentinel_state"] = feedback.link.state;
      plot_data["sentinel_reject"] = feedback.link.last_reject;
      plot_data["sentinel_aim_age_ms"] = feedback.link.aim_age_ms;
      plot_data["capture_frame_id"] = camera.get_last_capture_frame_id();
      plot_data["capture_pose_valid"] = frame_pose_capture.valid;
      plot_data["capture_pose_context_matches"] = frame_pose_context_matches;
      plot_data["capture_pose_status"] = static_cast<int>(frame_pose_capture.status);
      plot_data["capture_pose_seq"] = frame_pose_capture.source_seq;
      plot_data["capture_pose_time_us"] = frame_pose_capture.source_time_us;
      plot_data["capture_pose_interpolated"] = frame_pose_capture.interpolated;
      plot_data["capture_pose_synchronized"] = frame_pose_capture.synchronized;
      plot_data["capture_pose_time_uncertainty_ms"] = frame_pose_capture.time_uncertainty_ns / 1e6;
      plot_data["control_pose_seq"] = frame_pose_now.source_seq;
      plot_data["control_pose_time_us"] = frame_pose_now.source_time_us;
      plot_data["control_pose_world_ypr_rad"] = {current_ypr[0], current_ypr[1], current_ypr[2]};
      plot_data["track_id"] = tracker.track_id();
      plot_data["measurement_updated"] = tracker.measurement_updated();
      plot_data["identity_confirmed"] = tracker.identity_confirmed();
      plot_data["image_motion_yaw_rad"] = frame_image_motion[0];
      plot_data["image_motion_pitch_rad"] = frame_image_motion[1];
    }

    // Commit event state only after a complete write, so a throttled or expired
    // request cannot consume the one-shot opportunity.
    if (xuc.enabled()) {
      if (xuc.is_sentinel() && !frame_pose_context_matches) {
        command.control = false;
        command.shoot = false;
      }
      command.armor_detected = !armors.empty();
      if (xuc.is_sentinel() && std::isfinite(ballistic_camera_range_m))
        command.horizon_distance = ballistic_camera_range_m;
      const auto fire_deadline = command.shoot && gyro_event_schedule_valid
        ? gyro_event_command_due_time + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(xuc_event_send_late_tolerance_s))
        : std::chrono::steady_clock::time_point{};
      const auto send_result = xuc.send(command, fire_deadline,
        xuc.is_sentinel() ? frame_pose_now.generation : 0);
      plot_data["tx_result_known"] = true;
      plot_data["tx_status"] = static_cast<int>(send_result.status);
      plot_data["control_tx"] = send_result.control_sent;
      plot_data["fire_tx"] = send_result.fire_sent;
      plot_data["friction_tx"] = send_result.friction_sent;
      plot_data["fire_expired"] = send_result.fire_expired;
      plot_data["fire_rate_limited"] = send_result.fire_rate_limited;
      plot_data["stationary_yaw_fire_error_deg"] = stationary_yaw_fire_error_rad * 180.0 / M_PI;
      plot_data["pixel_yaw_residual_deg"] = direct_yaw_pixel_error_rad * 180.0 / M_PI;
      plot_data["predicted_fire_sensor_yaw_rad"] =
        gyro_predicted_impact_yaw_valid ? calibrated_predicted_yaw : NAN;
      plot_data["fire_fixed_yaw_correction_rad"] =
        xuc.is_sentinel() ? direct_yaw_command_correction_rad : 0.0;
      plot_data["tx_time_ns"] = send_result.tx_time_ns;
      if (send_result.fire_sent) {
        const auto issue_time = std::chrono::steady_clock::time_point(
          std::chrono::nanoseconds(send_result.tx_time_ns));
        const uint64_t shot_id = xuc_next_shot_id++;
        if (xuc_pending_shot) {
          tools::logger()->warn(
            "[SHOT] id={} superseded_without_feedback by={}",
            xuc_pending_shot_id, shot_id);
        }
        xuc_pending_shot = !xuc.is_sentinel();
        xuc_pending_shot_id = shot_id;
        xuc_pending_shot_time = issue_time;
        const double processing_age_ms =
          std::chrono::duration<double, std::milli>(issue_time - t).count();
        tools::logger()->info(
          "[SHOT] id={} request_tx mode={} gyro={} event={} cycle={} "
          "processing_age_ms={:.1f} due_error_ms={:+.2f} "
          "period_ms={:.1f} window_deg={:.2f} worst_deg={:.2f} combined_deg={:.2f} "
          "face={} face_align_deg={:.2f} phase_unc_deg={:.2f} gimbal_err_deg={:.2f}",
          shot_id, static_cast<int>(xuc.mode()), gyro_model_fresh ? 1 : 0,
          gyro_event_schedule_valid ? 1 : 0, gyro_event_hit_cycle,
          processing_age_ms, gyro_event_send_lateness_ms,
          gyro_event_face_period_s * 1000.0,
          gyro_event_half_window_rad * 180.0 / M_PI,
          gyro_event_worst_phase_rad * 180.0 / M_PI,
          gyro_event_combined_phase_rad * 180.0 / M_PI,
          gyro_event_face_id,
          gyro_event_face_alignment_rad * 180.0 / M_PI,
          gyro_event_phase_uncertainty_rad * 180.0 / M_PI,
          gyro_event_gimbal_tracking_error_rad * 180.0 / M_PI);
      }

      if (send_result.fire_sent) {
        xuc_fire_ready_logged = true;
        if (shot_uses_one_face_policy) {
          xuc_one_face_armed = false;
          if (gyro_event_schedule_valid) event_last_fired_cycle = gyro_event_hit_cycle;
        }
      } else if (command.shoot && !send_result.fire_rate_limited && !(xuc.is_sentinel() && xuc_fire_ready_logged &&
                   !gyro_event_schedule_valid &&
                   send_result.status == io::XucSender::SendStatus::throttled)) {
        tools::logger()->warn("[SHOT] request_not_sent status={} expired={}",
          static_cast<int>(send_result.status), send_result.fire_expired);
      }
    } else {
      cboard.send(command);
      if (command.shoot) {
        xuc_fire_ready_logged = true;
        if (shot_uses_one_face_policy) {
          xuc_one_face_armed = false;
          if (gyro_event_schedule_valid) event_last_fired_cycle = gyro_event_hit_cycle;
        }
      }
    }

    if (vision_diagnostics.enabled()) {
      const auto detection = detector.diagnostics();
      const auto & association = tracker.diagnostics();
      nlohmann::json observation = nullptr, model = nullptr;
      if (fresh_primary) {
        nlohmann::json points = nlohmann::json::array();
        for (const auto & point : fresh_primary->points) points.push_back({point.x, point.y});
        observation = {{"name", static_cast<int>(fresh_primary->name)},
          {"type", static_cast<int>(fresh_primary->type)}, {"confidence", fresh_primary->confidence},
          {"corners_px", points}, {"pose_valid", fresh_primary->pose_valid},
          {"xyz_world_m", {fresh_primary->xyz_in_world[0], fresh_primary->xyz_in_world[1], fresh_primary->xyz_in_world[2]}},
          {"yaw_world_rad", fresh_primary->ypr_in_world[0]},
          {"reprojection_px", fresh_primary->reprojection_error_px}, {"pose_quality", fresh_primary->pose_quality}};
      }
      if (!targets.empty()) {
        const auto & target = targets.front();
        const auto fused = target.ekf_x(); const auto & whole = target.ekf().x;
        model = {{"whole_state", std::vector<double>(whole.data(), whole.data() + whole.size())},
          {"fused_state", std::vector<double>(fused.data(), fused.data() + fused.size())},
          {"yaw_trusted", target.yaw_eskf_trusted()},
          {"whole_rotation_trusted", target.whole_rotation_trusted()}, {"nis", target.ekf().last_nis},
          {"center_acceleration_variance", target.center_acceleration_variance()}};
      }
      vision_diagnostics.write({{"schema", 1}, {"sequence", vision_frame_sequence},
        {"capture_frame", camera.get_last_capture_frame_id()}, {"time_s", model_now_s},
        {"detector", {{"full_frame", search_full_frame},
          {"roi", {detection.roi.x, detection.roi.y, detection.roi.width, detection.roi.height}},
          {"rows", detection.rows}, {"candidates", detection.candidates}, {"nms", detection.nms},
          {"accepted", detection.accepted}, {"keypoint_rejected", detection.rejected_keypoints},
          {"label_shape_rejected", detection.rejected_label_shape}, {"refinement_rejected", detection.rejected_refinement}}},
        {"tracker", {{"id", tracker.track_id()}, {"state", tracker.state()},
          {"updated", tracker.measurement_updated()}, {"rebuilt", association.rebuilt},
          {"batch", association.batch_update}, {"identity", tracker.identity_confirmed()},
          {"primary_face", tracker.primary_face_id()}, {"invalid_pose", association.invalid_pose},
          {"identity_rejected", association.identity_rejected}, {"position_rejected", association.position_rejected},
          {"cost_rejected", association.cost_rejected},
          {"size_correction_rejected", association.size_correction_rejected},
          {"candidate_pairs", association.candidates},
          {"best_cost", association.best_cost}, {"center_error", association.center_error},
          {"corner_error", association.corner_error}, {"angle_error", association.angle_error},
          {"range_error", association.range_error}}},
        {"detections", detector_observations}, {"observation", observation}, {"model", model},
        {"pose", {{"capture_generation", frame_pose_capture.generation},
          {"current_generation", frame_pose_now.generation}, {"context_matches", frame_pose_context_matches},
          {"capture_valid", frame_pose_capture.valid}, {"current_valid", frame_pose_now.valid},
          {"capture_sensor_rpy_rad", {frame_pose_capture.rpy_rad[0], frame_pose_capture.rpy_rad[1], frame_pose_capture.rpy_rad[2]}},
          {"current_sensor_rpy_rad", {frame_pose_now.rpy_rad[0], frame_pose_now.rpy_rad[1], frame_pose_now.rpy_rad[2]}},
          {"final_sensor_command_rad", {command.yaw, command.pitch}},
          {"control", command.control}, {"shoot", command.shoot}}},
        {"center_memory", {{"valid", xuc_gyro_center_memory_valid}, {"fresh", vehicle_model_fresh},
          {"age_s", xuc_gyro_center_memory_valid ? std::chrono::duration<double>(t - xuc_gyro_center_last_fresh).count() : NAN}}},
        {"prediction", {{"valid", aim_trace.valid}, {"converged", aim_trace.converged},
          {"whole_geometry", aim_trace.whole_ekf_geometry}, {"face", aim_trace.face_id},
          {"iterations", aim_trace.iterations}, {"delay_s", aim_trace.delay_s}, {"speed_mps", aim_trace.speed_mps},
          {"delay_branch", aim_trace.high_speed_delay ? "high" : "low"},
          {"delay_motion_rate_rad_s", aim_trace.delay_motion_rate_rad_s},
          {"native_sensor_yaw_rad", native_prediction_valid ? native_predicted_yaw : NAN},
          {"calibrated_fire_sensor_yaw_rad", native_prediction_valid ? calibrated_predicted_yaw : NAN},
          {"aim_point_world_m", {aimer.debug_aim_point.xyza[0], aimer.debug_aim_point.xyza[1], aimer.debug_aim_point.xyza[2]}},
          {"base_time_ns", aim_trace.base_prediction_ns}, {"time_ns", aim_trace.prediction_ns},
          {"flight_time_used_s", aim_trace.flight_time_used_s}, {"flight_time_s", aim_trace.flight_time_s},
          {"world_command_rad", {native_world_command.yaw, native_world_command.pitch}}}},
        {"fire_gates", fire_gate_diagnostics}, {"transport", plot_data},
        {"performance_ms", {{"detection", std::chrono::duration<double, std::milli>(t2 - t1).count()},
          {"tracker", std::chrono::duration<double, std::milli>(t3 - t2).count()}}}});
    }
    plotter.plot(plot_data);

    if (!competition_headless) {
      // Show detector output even when pose or tracking qualification fails.
      // The previous view drew only trusted predictions, hiding valid detections.
      for (const auto & detection : display_detections) {
        tools::draw_points(img, detection.points, {255, 255, 0});
        tools::draw_text(img, fmt::format("{} {:.2f}",
          auto_aim::ARMOR_NAMES[detection.name], detection.confidence),
          detection.center, {255, 255, 0});
      }
      tools::draw_text(img, fmt::format("DETECT {}  TRACK {}  ID {}  FIRE_TX {}",
        raw_detection_count, tracker.state(), tracker.track_id(),
        plot_data["fire_tx"].get<bool>() ? 1 : 0), {10, 30}, {255, 255, 255});
      // 绘制识别与预测结果
      // Display a finite tracked solution independently of rotational trust.
      // Trust still gates motion control / fire above; showing a box grants no permission.
      if (!targets.empty() && tracker_converged && !targets.front().diverged()) {
        auto target = targets.front();
        const bool rotation_trusted = yaw_motion_model_trusted || whole_vehicle_candidate_trusted;
        tools::draw_text(img, whole_vehicle_candidate_trusted ? "WHOLE MODEL TRUSTED" :
          yaw_motion_model_trusted ? "YAW MODEL TRUSTED" : "ROTATION LEARNING - FIRE GATES ACTIVE",
          {10, 58}, rotation_trusted ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 180, 0));

        // 绘制跟踪到的所有装甲板位置（绿色）
        std::vector<Eigen::Vector4d> armor_xyza_list = aim_trace.whole_ekf_geometry
          ? target.whole_ekf_armor_xyza_list() : target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {0, 255, 0});
        }

        // 绘制预测的瞄准位置（红色）
        if (aimer.debug_aim_point.valid && aim_trace.valid && aim_trace.converged &&
            aimer.debug_aim_point.xyza.allFinite()) {
          Eigen::Vector4d aim_xyza = aimer.debug_aim_point.xyza;
          auto image_points =
            solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
          tools::draw_points(img, image_points, {0, 0, 255});
        }
      }

      cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
      // 相机输出为 RGB 格式，imshow 需要 BGR 格式
      cv::Mat img_bgr;
      cv::cvtColor(img, img_bgr, cv::COLOR_RGB2BGR);
      cv::imshow("reprojection", img_bgr);
      auto key = cv::waitKey(1);
      if (key == 'q') break;
    }

    // 🔍 性能监控：打印各环节耗时
    auto loop_end = std::chrono::steady_clock::now();
    static int perf_counter = 0;
    if (++perf_counter % 50 == 0) {
      auto detect_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
      auto track_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
      auto total_ms = std::chrono::duration<double, std::milli>(loop_end - loop_start).count();
      tools::logger()->info("[PERF] 总耗时={:.1f}ms (检测={:.1f}ms, 跟踪={:.1f}ms), FPS={:.1f}",
        total_ms, detect_ms, track_ms, 1000.0 / total_ms);
    }
  }

  // 清理 ROS2
#ifdef AMENT_CMAKE_FOUND
  visualizer.reset();
  rclcpp::shutdown();
  tools::logger()->info("[ROS2] Shutdown complete");
#endif

  return 0;
}

#include <fmt/core.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>  // for setenv
#include <thread>
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
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | 位置参数，yaml配置文件路径 }";

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

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;
  cv::CommandLineParser cli(argc, argv, keys);

  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  const auto integration_yaml = YAML::LoadFile(config_path);
  const bool direct_yaw_guard_enabled =
    integration_yaml["xuc_direct_yaw_guard"] &&
    integration_yaml["xuc_direct_yaw_guard"].as<bool>();
  const bool prediction_lead_limit_configured =
    integration_yaml["xuc_max_prediction_lead_rad"].IsDefined();
  const double max_prediction_lead_rad =
    prediction_lead_limit_configured
      ? std::max(0.0, integration_yaml["xuc_max_prediction_lead_rad"].as<double>())
      : 0.0;
  const double direct_yaw_filter_tau_s =
    integration_yaml["xuc_direct_yaw_filter_tau_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_filter_tau_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_yaw_hold_s =
    integration_yaml["xuc_direct_yaw_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_hold_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_yaw_max_rate_rad_s =
    integration_yaml["xuc_direct_yaw_max_rate_dps"]
      ? std::max(0.0, integration_yaml["xuc_direct_yaw_max_rate_dps"].as<double>()) * M_PI / 180.0
      : 0.0;
  const bool direct_pitch_guard_enabled =
    integration_yaml["xuc_direct_pitch_guard"] &&
    integration_yaml["xuc_direct_pitch_guard"].as<bool>();
  const double direct_pitch_filter_tau_s =
    integration_yaml["xuc_direct_pitch_filter_tau_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_filter_tau_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_pitch_hold_s =
    integration_yaml["xuc_direct_pitch_hold_ms"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_hold_ms"].as<double>()) / 1000.0
      : 0.0;
  const double direct_pitch_max_rate_rad_s =
    integration_yaml["xuc_direct_pitch_max_rate_dps"]
      ? std::max(0.0, integration_yaml["xuc_direct_pitch_max_rate_dps"].as<double>()) * M_PI / 180.0
      : 0.0;
  const double direct_pitch_error_gain =
    integration_yaml["xuc_direct_pitch_error_gain"]
      ? std::clamp(integration_yaml["xuc_direct_pitch_error_gain"].as<double>(), 0.0, 2.0)
      : 1.0;
  const double direct_pitch_focal_y_px =
    integration_yaml["camera_matrix"] && integration_yaml["camera_matrix"].size() >= 5
      ? integration_yaml["camera_matrix"][4].as<double>()
      : 0.0;
  const double xuc_pitch_sign_for_log = integration_yaml["xuc_pitch_sign"]
    ? integration_yaml["xuc_pitch_sign"].as<double>()
    : -1.0;

#ifdef AMENT_CMAKE_FOUND
  // 初始化ROS2可视化
  rclcpp::init(argc, argv);
  auto visualizer = std::make_shared<tools::ROS2Visualizer>("standard_node", "standard_markers");
  tools::logger()->info("[ROS2] Visualizer initialized");
#endif

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);
  io::XucSender xuc(config_path);  // 新版XUC：接收IMU，发送目标角

  auto_aim::YOLO detector(config_path, false);  // 启用调试，显示检测窗口
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter, true);
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
  double direct_yaw_filtered = 0.0;
  bool direct_yaw_long_loss = false;
  std::chrono::steady_clock::time_point direct_yaw_last_update;
  std::chrono::steady_clock::time_point direct_yaw_last_measurement;
  bool direct_pitch_initialized = false;
  double direct_pitch_filtered = 0.0;
  bool direct_pitch_long_loss = false;
  std::chrono::steady_clock::time_point direct_pitch_last_update;
  std::chrono::steady_clock::time_point direct_pitch_last_measurement;
  std::chrono::steady_clock::time_point fps_measure_start; // 平均帧率测量开始时间
  bool fps_measure_started = false;

  // 初始化默认命令（第一帧使用）
  io::Command default_command;
  default_command.yaw = 0.0;
  default_command.pitch = 0.0;
  default_command.control = false;
  default_command.shoot = false;

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();  // 🔍 性能监控：循环开始

    // 🎯 关键修改：先发送命令，触发相机曝光，然后再读取图像
    // 这样确保电控板收到命令后，才会发出硬触发信号让相机曝光
    static bool first_loop = true;
    if (first_loop) {
      // 第一帧发送默认命令
      if (xuc.enabled()) {
        xuc.send(default_command);
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
          const double imu_yaw = xuc.imu_yaw();
          const double imu_pitch = xuc.imu_pitch();
          q = Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(imu_pitch, Eigen::Vector3d::UnitY());
          q.normalize();
          solver.set_R_gimbal2world(q);
          cboard.mode = static_cast<io::Mode>(xuc.mode());
          cboard.bullet_speed = xuc.bullet_speed();
          mode = cboard.mode;

          static int xuc_imu_log_counter = 0;
          if (++xuc_imu_log_counter % 50 == 0) {
            tools::logger()->info(
              "[SYNC][XUC] mode={} disarm_reason={} yaw={:.2f} deg pitch={:.2f} deg "
              "bullet_speed={:.2f}",
              static_cast<int>(xuc.mode()), static_cast<unsigned>(xuc.bullet_count()),
              imu_yaw * 180.0 / M_PI, imu_pitch * 180.0 / M_PI, cboard.bullet_speed);
          }
        } else {
          cboard.mode = io::Mode::idle;
          mode = io::Mode::idle;
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

    auto t1 = std::chrono::steady_clock::now();  // 🔍 性能监控
    auto armors = detector.detect(img);
    auto t2 = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, t);
    auto t3 = std::chrono::steady_clock::now();

    // 调试：打印 armors 和 targets 数量
    fmt::print("[DEBUG] armors={}, targets={}\n", armors.size(), targets.size());

#ifdef AMENT_CMAKE_FOUND
    // 发布装甲板Marker（可视化检测结果）
    if (!armors.empty()) {
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        t.time_since_epoch()).count();
      rclcpp::Time ros_time(ns);

      visualization_msgs::msg::MarkerArray marker_array;
      int marker_id = 0;
      for (const auto & armor : armors) {
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

    auto command = aimer.aim(targets, t, cboard.bullet_speed, true);  // to_now=true，生成当前时刻命令
    const bool native_prediction_valid =
      command.control && !targets.empty() && std::isfinite(command.yaw);
    const double native_predicted_yaw = command.yaw;

    // Phase 2D.5 integration guard.  The detector pose is much more stable
    // than the current high-dynamic EKF tuning at close range.  Keep the
    // complete Tracker/Aimer prediction path, but prevent a divergent state
    // from commanding a yaw far away from the armor actually in the image.
    // If the tracker briefly rejects convergence while a valid enemy armor is
    // still detected, degrade to measurement-only yaw tracking and never fire.
    bool measurement_fallback = false;
    if (direct_yaw_guard_enabled) {
      bool valid_measurement = false;
      double measured_yaw = 0.0;
      if (!armors.empty()) {
        const auto & measured_armor = armors.front();
        measured_yaw =
          std::atan2(measured_armor.xyz_in_world.y(), measured_armor.xyz_in_world.x());
        valid_measurement = std::isfinite(measured_yaw);
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
        command.control = true;
        command.shoot = false;
        if (prediction_lead_limit_configured && native_prediction_valid) {
          const double prediction_lead =
            std::remainder(native_predicted_yaw - measured_yaw, 2.0 * M_PI);
          command.yaw = std::remainder(
            direct_yaw_filtered + std::clamp(
              prediction_lead, -max_prediction_lead_rad, max_prediction_lead_rad),
            2.0 * M_PI);
        } else {
          command.yaw = direct_yaw_filtered;
        }
      } else if (direct_yaw_initialized) {
        const double visual_gap =
          std::chrono::duration<double>(t - direct_yaw_last_measurement).count();
        if (visual_gap <= direct_yaw_hold_s) {
          // During a short detector gap, allow only the already-bounded native
          // prediction.  This keeps fast motion continuous without trusting an
          // unconstrained EKF state.
          command.control = true;
          command.shoot = false;
          command.yaw = direct_yaw_filtered;
          if (prediction_lead_limit_configured && native_prediction_valid) {
            const double prediction_delta =
              std::remainder(native_predicted_yaw - direct_yaw_filtered, 2.0 * M_PI);
            command.yaw = std::remainder(
              direct_yaw_filtered + std::clamp(
                prediction_delta, -max_prediction_lead_rad, max_prediction_lead_rad),
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
      if (!armors.empty() && direct_pitch_focal_y_px > 0.0 && !img.empty()) {
        const auto & measured_armor = armors.front();
        double center_y_px = measured_armor.center.y;
        if (!measured_armor.points.empty()) {
          center_y_px = 0.0;
          for (const auto & point : measured_armor.points) center_y_px += point.y;
          center_y_px /= static_cast<double>(measured_armor.points.size());
        }
        const double pixel_pitch_error = std::atan2(
          center_y_px - 0.5 * static_cast<double>(img.rows),
          direct_pitch_focal_y_px);
        // Physical acceptance established that a target below image centre
        // requires a larger lower-board IMU pitch target.  Closing the loop in
        // image space avoids the currently inconsistent pitch hand-eye offset.
        measured_pitch = xuc.imu_pitch() + direct_pitch_error_gain * pixel_pitch_error;
        valid_pitch_measurement = std::isfinite(measured_pitch);
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
    command.shoot = shooter.shoot(command, aimer, targets, ypr);
    if (measurement_fallback) command.shoot = false;

    // Dry-run diagnostics: world-coordinate stability and final command.
    static int dryrun_log_counter = 0;
    if (++dryrun_log_counter % 20 == 0) {
      constexpr double RAD2DEG = 180.0 / M_PI;
      if (!targets.empty()) {
        auto tracked_xyza_list = targets.front().armor_xyza_list();
        if (!tracked_xyza_list.empty() && !armors.empty()) {
          const auto & measured_gimbal = armors.front().xyz_in_gimbal;
          const auto & measured_world = armors.front().xyz_in_world;
          const auto & tracked = tracked_xyza_list.front();
          double armor_center_y_px = armors.front().center.y;
          if (!armors.front().points.empty()) {
            armor_center_y_px = 0.0;
            for (const auto & point : armors.front().points) armor_center_y_px += point.y;
            armor_center_y_px /= static_cast<double>(armors.front().points.size());
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
            "armor_y_px={:.1f} pixel_pitch_error_deg={:.2f} "
            "q_wxyz=({:.6f},{:.6f},{:.6f},{:.6f}) "
            "cmd_deg=({:.2f},{:.2f}) packed_pitch_deg={:.2f} "
            "control={} shoot={}",
            measured_gimbal.x(), measured_gimbal.y(), measured_gimbal.z(),
            measured_world.x(), measured_world.y(), measured_world.z(),
            tracked.x(), tracked.y(), tracked.z(),
            target_gimbal_yaw_deg, target_gimbal_pitch_deg,
            target_world_yaw_deg, target_world_pitch_deg,
            ypr[0] * RAD2DEG, ypr[1] * RAD2DEG,
            armor_center_y_px, pixel_pitch_error_deg,
            q.w(), q.x(), q.y(), q.z(),
            command.yaw * RAD2DEG, command.pitch * RAD2DEG,
            xuc_pitch_sign_for_log * command.pitch * RAD2DEG,
            command.control, command.shoot);
        }
      } else {
        tools::logger()->info(
          "[DRYRUN] target=none gimbal_deg=({:.2f},{:.2f}) control={} shoot={}",
          ypr[0] * RAD2DEG, ypr[1] * RAD2DEG,
          command.control, command.shoot);
      }
    }

    // 发送命令到 PlotJuggler（显示实际发送的值）
    nlohmann::json plot_data;
    plot_data["t"] = std::chrono::duration<double>(t - std::chrono::steady_clock::time_point()).count();
    plot_data["cmd_yaw"] = command.yaw * 180.0 / M_PI;
    plot_data["cmd_pitch"] =
      xuc_pitch_sign_for_log * command.pitch * 180.0 / M_PI;
    plot_data["control"] = command.control;
    plot_data["shoot"] = command.shoot;
    // 电控的欧拉角（从MCU获取的姿态）
    plot_data["mcu_yaw"] = ypr[0] * 180.0 / M_PI;
    plot_data["mcu_pitch"] = ypr[1] * 180.0 / M_PI;
    plot_data["mcu_roll"] = ypr[2] * 180.0 / M_PI;
    // 电控的四元数（从MCU获取）
    plot_data["mcu_q_w"] = q.w();
    plot_data["mcu_q_x"] = q.x();
    plot_data["mcu_q_y"] = q.y();
    plot_data["mcu_q_z"] = q.z();
    plotter.plot(plot_data);

    // 🎯 关键：将 send 移到循环开头
    // 这样下一次循环时，会先发送这个命令，然后才读取相机
    // 形成闭环：发送命令 → 电控触发相机 → 读取图像 → 处理 → 发送下一条命令
    if (xuc.enabled()) {
      xuc.send(command);
    } else {
      cboard.send(command);
    }

    // 绘制识别与预测结果
    if (!targets.empty()) {
      auto target = targets.front();

      // 绘制跟踪到的所有装甲板位置（绿色）
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      // 绘制预测的瞄准位置（红色）
      if (aimer.debug_aim_point.valid) {
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

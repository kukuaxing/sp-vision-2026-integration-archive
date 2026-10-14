#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <sstream>
#include <nlohmann/json.hpp>

#include "io/camera.hpp"
#include "calibration/imu_pose.hpp"
#include "calibration/rmv_pose.hpp"
#include "calibration/raw_pose_gate.hpp"
#include "io/cboard.hpp"
#include "io/xuc.hpp"
#include "tools/cmdline.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o | data/calibration/sentry_intrinsics | 输出文件夹路径 }"
  "{intrinsics-only |                          | 仅采集内参图像，不读取或保存姿态 }"
  "{gui-check       |                          | 仅检查窗口，不打开相机或串口 }"
  "{gui-info        |                          | 仅报告窗口后端，不创建窗口或打开设备 }"
  "{pose-check      | 0                        | 检查配置的HI91/RMV姿态秒数，不打开相机或发控制指令 }";

std::string capture_gui_backend()
{
  std::istringstream lines(cv::getBuildInformation());
  std::string line, backend;
  while (std::getline(lines, line)) {
    const auto at = line.find("GUI:");
    if (at != std::string::npos) { std::istringstream(line.substr(at + 4)) >> backend; break; }
  }
  if (backend.empty() || backend == "NONE")
    throw std::runtime_error("Linked OpenCV has no GUI backend. Rebuild with the system GUI-enabled OpenCV using scripts/build.sh capture.");
  return backend;
}

void check_capture_gui(bool probe)
{
  const auto backend = capture_gui_backend();
  const std::string window = probe ? "Capture GUI check" : "Chessboard Capture - 's' to save, 'q' to quit";
  cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
  if (probe) {
    cv::Mat image(180, 640, CV_8UC3, cv::Scalar(30, 30, 30));
    cv::putText(image, "GUI OK - no camera or serial opened", {20, 90}, cv::FONT_HERSHEY_SIMPLEX, .7, {0, 220, 0}, 2);
    cv::imshow(window, image);
    cv::waitKey(50);
    cv::destroyWindow(window);
    fmt::print("GUI backend={} window_check=passed camera_opened=false serial_opened=false\n", backend);
  }
}

nlohmann::json rmv_metadata(const CalibrationRmvState::Sample& pose)
{
  return {{"pose_source", "rmv_sensor1_frame1_wxyz"}, {"sensor_id", 1}, {"frame_id", 1},
    {"boot_id", pose.boot_id}, {"session_id", pose.session_id}, {"frame_epoch", pose.epoch},
    {"geometry_revision", pose.geometry_revision}, {"firmware_version", pose.firmware},
    {"source_counter", pose.source_counter}, {"accepted_samples", pose.count},
    {"valid_flags", pose.flags}, {"sample_mcu_us", pose.sample_mcu_us}, {"device_us", pose.device_us},
    {"received_host_us", pose.received_host_us}, {"sample_host_us", pose.sample_host_us},
    {"gyro_deg_s", pose.gyro_deg_s}, {"gyro_rms_100ms_deg_s", pose.gyro_rms_deg_s},
    {"anchor_delta_deg", pose.anchor_delta_deg}, {"stable_since_host_us", pose.stable_since_host_us},
    {"sample_time_basis", (pose.flags & 8) ? "mapped MCU sample time" : "MCU IMU receive time"},
    {"quaternion_wxyz", {pose.q.w(), pose.q.x(), pose.q.y(), pose.q.z()}},
    {"coordinate_meaning", "small-Y sensor body to its navigation frame; not camera-support G"}};
}

void ensure_rmv_context(const std::string& folder, const CalibrationRmvState::Sample& pose)
{
  const nlohmann::json context = {{"pose_source", "rmv_sensor1_frame1_wxyz"},
    {"boot_id", pose.boot_id}, {"frame_epoch", pose.epoch},
    {"geometry_revision", pose.geometry_revision}, {"firmware_version", pose.firmware}};
  const auto marker = std::filesystem::path(folder) / "pose_context.json";
  if (std::filesystem::exists(marker)) {
    std::ifstream in(marker); nlohmann::json previous; in >> previous;
    if (previous != context) throw std::runtime_error("Calibration boot/epoch/geometry changed; use a new dataset");
  } else {
    for (const auto& entry : std::filesystem::directory_iterator(folder))
      if (entry.path().extension() == ".jpg")
        throw std::runtime_error("Existing images have no RMV coordinate context");
    std::ofstream out(marker); out << context.dump(2) << '\n';
    if (!out) throw std::runtime_error("Could not write RMV dataset context");
  }
}

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
  if (!q_file) throw std::runtime_error("Could not write real pose file: " + q_path);
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder,
  int pattern_cols, int pattern_rows, bool use_cboard, bool use_xuc, bool use_hipnuc, bool use_rmv)
{
  // CBoard 构造函数会阻塞等待 C 板 CAN 四元数帧。
  // 没有 C 板时（仅做内参标定）跳过，不保存姿态文件。
  std::unique_ptr<io::CBoard> cboard;
  if (use_cboard) {
    cboard = std::make_unique<io::CBoard>(config_path);
  } else if (!use_xuc && !use_hipnuc && !use_rmv) {
    tools::logger()->warn("require_cboard=false: 未读取C板四元数，仅适用于内参标定");
  }
  std::unique_ptr<io::XucSender> xuc;
  if (use_xuc) {
    xuc = std::make_unique<io::XucSender>(config_path);
    if (!xuc->enabled()) {
      throw std::runtime_error("xuc_enable=true, but the XUC serial port could not be opened");
    }
    tools::logger()->info(
      "使用 XUC 串口姿态采集；capture 不发送任何控制或射击命令");
  }
  std::unique_ptr<CalibrationImu> hipnuc;
  if (use_hipnuc) {
    const auto cfg=YAML::LoadFile(config_path);
    hipnuc=std::make_unique<CalibrationImu>(cfg["xuc_serial_port"].as<std::string>(),
                                          cfg["xuc_serial_baud"].as<uint32_t>());
  }
  std::unique_ptr<CalibrationRmvImu> rmv;
  if (use_rmv) {
    const auto cfg = YAML::LoadFile(config_path);
    rmv = std::make_unique<CalibrationRmvImu>(cfg["xuc_serial_port"].as<std::string>(),
                                             cfg["xuc_serial_baud"].as<uint32_t>());
    tools::logger()->info("RMV 标定接收小 Y 原始姿态；仅发送 HELLO、空闲心跳和时间同步");
  }
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  cv::Size pattern_size(pattern_cols, pattern_rows);
  int count = 0;
  for (const auto & entry : std::filesystem::directory_iterator(output_folder)) {
    if (entry.path().extension() != ".jpg") continue;
    const auto stem = entry.path().stem().string();
    if (!stem.empty() && stem.find_first_not_of("0123456789") == std::string::npos)
      count = std::max(count, std::stoi(stem));
  }
  int failed_count = 0;
  uint64_t last_diagnostic_us = 0;
  std::string last_save_error;
  if (rmv) std::filesystem::create_directories("logs/calibration");
  const auto report_state = [&](const CalibrationRmvState::Sample& pose, uint64_t image_us,
                                bool board_found, uint64_t now) {
    auto status = rmv_metadata(pose);
    status["save_ready"] = pose.valid && board_found;
    status["pose_save_error"] = pose.error;
    status["last_save_error"] = last_save_error;
    status["chessboard_found"] = board_found;
    status["image_age_ms"] = now >= image_us ? double(now - image_us) / 1000 : -1;
    status["stable_ms"] = pose.sample_host_us >= pose.stable_since_host_us ?
      double(pose.sample_host_us - pose.stable_since_host_us) / 1000 : 0;
    status["updated_host_us"] = now;
    status["saved_count"] = count;
    status["output_folder"] = output_folder;
    std::ofstream out("logs/calibration/capture-state.json.tmp"); out << status.dump(2) << '\n';
    out.close();
    if (out) std::filesystem::rename("logs/calibration/capture-state.json.tmp", "logs/calibration/capture-state.json");
    last_diagnostic_us = now;
  };

  tools::logger()->info("=== 棋盘格标定数据采集 ===");
  tools::logger()->info("标定板尺寸: {}x{} 内角点", pattern_cols, pattern_rows);
  tools::logger()->info("按 's' 键保存, 按 'q' 键退出");

  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    const auto image_host_us = std::chrono::duration_cast<std::chrono::microseconds>(
      timestamp.time_since_epoch()).count();
    bool pose_valid = !cboard && !xuc && !hipnuc && !rmv;
    bool pose_stable = true;
    if (rmv) {
      const auto pose = rmv->sample();
      q = pose.q;
      pose_valid = CalibrationRmvState::fresh(pose);
      pose_stable = rmv->for_image(image_host_us).valid;
    } else if (hipnuc) {
      const auto pose=hipnuc->sample();
      q=pose.q;
      pose_valid=CalibrationImu::fresh(pose);
      pose_stable=CalibrationImu::can_save(pose);
    } else if (cboard) {
      q = cboard->imu_at(timestamp);
      pose_valid = true;
    } else if (xuc && xuc->rx_valid()) {
      q = Eigen::AngleAxisd(xuc->imu_yaw(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(xuc->imu_pitch(), Eigen::Vector3d::UnitY());
      q.normalize();
      pose_valid = true;
    }

    // 在图像上显示欧拉角
    auto img_with_info = img.clone();
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
    tools::draw_text(img_with_info, fmt::format("IMU Z(Yaw): {:.1f}", zyx[0]), {10, 30}, {255, 255, 0});
    tools::draw_text(img_with_info, fmt::format("IMU Y(Pitch): {:.1f}", zyx[1]), {10, 60}, {255, 255, 0});
    tools::draw_text(img_with_info, fmt::format("IMU X(Roll): {:.1f}", zyx[2]), {10, 90}, {255, 255, 0});
    if (!pose_valid) {
      tools::draw_text(img_with_info, "IMU INVALID - DO NOT SAVE", {10, 150}, {0, 0, 255}, 1.0, 2);
    }

    if (hipnuc && pose_valid && !pose_stable)
      tools::draw_text(img_with_info, "Hold still for 0.5 s before saving", {10, 180}, {0, 255, 255});

    // HikRobot camera frames are RGB.  Detect on an explicitly converted
    // grayscale image; OpenCV display and image files use BGR ordering.
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

    // 检测棋盘格角点
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(
      gray, pattern_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
      // 亚像素精化（提高精度）
      cv::cornerSubPix(
        gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

      // 显示检测到的角点
      cv::drawChessboardCorners(img_with_info, pattern_size, corners, found);
      tools::draw_text(img_with_info, "Chessboard OK!", {10, 120}, {0, 255, 0}, 1.0, 2);
      failed_count = 0;
    } else {
      failed_count++;
      tools::draw_text(img_with_info, fmt::format("NOT found ({})", failed_count),
                       {10, 120}, {0, 0, 255}, 0.8, 2);
    }

    if (rmv) {
      const auto pose = rmv->for_image(image_host_us);
      const auto stable_ms = pose.sample_host_us >= pose.stable_since_host_us ?
        double(pose.sample_host_us - pose.stable_since_host_us) / 1000 : 0;
      tools::draw_text(img_with_info, pose.valid ? "Pose OK - ready to save" : pose.error,
        {10, 180}, pose.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255), .7, 1);
      tools::draw_text(img_with_info, fmt::format("Gyro {:.2f}, RMS {:.2f}/2 deg/s; stable {:.0f}/500 ms",
        pose.gyro_deg_s, pose.gyro_rms_deg_s, stable_ms), {10, 210}, {255, 255, 0}, .7, 1);
      const auto now = CalibrationRmvState::now_us();
      if (now - last_diagnostic_us >= 1000000) report_state(pose, image_host_us, found, now);
    }

    // 显示已采集数量
    tools::draw_text(img_with_info, fmt::format("Captured: {}", count),
                     {10, img_with_info.rows - 20}, {255, 255, 255}, 1.0, 2);

    cv::Mat preview_bgr;
    cv::cvtColor(img_with_info, preview_bgr, cv::COLOR_RGB2BGR);
    cv::resize(preview_bgr, preview_bgr, {}, 0.5, 0.5);
    cv::imshow("Chessboard Capture - 's' to save, 'q' to quit", preview_bgr);

    auto key = cv::waitKey(1);
    if (key == 'q') {
      break;
    } else if (key == 's') {
      if (!found) {
        tools::logger()->warn("未检测到棋盘格，无法保存！");
        continue;
      }
      CalibrationRmvState::Sample image_pose;
      if (rmv) {
        image_pose = rmv->for_image(image_host_us);
        pose_valid = image_pose.valid;
        pose_stable = image_pose.valid;
        if (pose_valid) {
          q = image_pose.q;
          ensure_rmv_context(output_folder, image_pose);
        }
      } else if (hipnuc) {
        // Recheck after corner detection and GUI work, which can outlive freshness.
        pose_valid = CalibrationImu::fresh(hipnuc->sample());
        pose_stable = CalibrationImu::can_save(hipnuc->sample());
      }
      if (!pose_valid || !pose_stable) {
        if (rmv) {
          last_save_error = image_pose.error;
          const auto now = CalibrationRmvState::now_us();
          report_state(image_pose, image_host_us, found, now);
          tools::logger()->warn("拒绝保存: {}; gyro={:.2f}, RMS100ms={:.2f} deg/s, 图像年龄={:.1f} ms",
            image_pose.error, image_pose.gyro_deg_s, image_pose.gyro_rms_deg_s,
            double(now - image_host_us) / 1000);
        } else tools::logger()->warn("姿态无效或尚未停稳，拒绝保存这一帧");
        continue;
      }

      // 保存图片和四元数
      count++;
      auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
      auto q_path = fmt::format("{}/{}.txt", output_folder, count);
      cv::Mat saved_bgr;
      cv::cvtColor(img, saved_bgr, cv::COLOR_RGB2BGR);
      if (!cv::imwrite(img_path, saved_bgr)) throw std::runtime_error("Could not save image: " + img_path);
      if (cboard || xuc || hipnuc || rmv) write_q(q_path, q); // No synthetic identity poses for intrinsic images.
      if (rmv) {
        auto metadata = rmv_metadata(image_pose);
        metadata["image_host_us"] = image_host_us;
        metadata["image_time_basis"] = "camera host timestamp, not verified hardware exposure time";
        metadata["image_pose_method"] = "nearest sample within 50 ms, inside stationary interval";
        std::ofstream out(fmt::format("{}/{}.pose.json", output_folder, count));
        out << metadata.dump(2) << '\n';
        if (!out) throw std::runtime_error("Could not write RMV sample metadata");
      }
      tools::logger()->info("[{}] Saved to {} (corners: {})", count, output_folder, corners.size());
      if (rmv) { last_save_error.clear(); report_state(image_pose, image_host_us, true, CalibrationRmvState::now_us()); }
    }
  }

  tools::logger()->info("采集完成！共 {} 组数据", count);
  if (count < 10) {
    tools::logger()->warn("建议至少采集10-15组数据");
  }
}

int run(int argc, char * argv[])
{
  // 读取命令行参数（规整 "-o xxx" 这类空格写法，OpenCV CommandLineParser 只认 '=' 形式）
  auto parsed = tools::normalize_argv(argc, argv);
  cv::CommandLineParser cli(parsed.argc(), parsed.argv(), keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  if (cli.has("gui-check")) { check_capture_gui(true); return 0; }
  if (cli.has("gui-info")) {
    const nlohmann::json info = {{"backend", capture_gui_backend()}, {"opencv_version", CV_VERSION},
      {"camera_opened", false}, {"serial_opened", false}, {"window_opened", false}};
    fmt::print("GUI {}\n", info.dump()); return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");


  // 读取标定板尺寸
  auto yaml = YAML::LoadFile(config_path);
  const bool hipnuc_config = yaml["calibration_pose_source"] &&
      yaml["calibration_pose_source"].as<std::string>() == "hipnuc";
  const bool rmv_config = yaml["calibration_pose_source"] &&
      yaml["calibration_pose_source"].as<std::string>() == "rmv";
  if (yaml["calibration_pose_source"] && !hipnuc_config && !rmv_config)
    throw std::runtime_error("Unsupported calibration_pose_source; expected hipnuc or rmv");
  const auto check_seconds=cli.get<double>("pose-check");
  bool check_requested = check_seconds != 0;
  for (int i = 1; i < parsed.argc(); ++i) {
    const std::string arg = parsed.argv()[i];
    if (arg == "--pose-check" || arg == "-pose-check" ||
        arg.rfind("--pose-check=", 0) == 0 || arg.rfind("-pose-check=", 0) == 0)
      check_requested = true;
  }
  if (check_requested) {
    if (!std::isfinite(check_seconds) || check_seconds <= 0 || check_seconds > 30)
      throw std::runtime_error("pose-check duration must be finite and within (0, 30] seconds");
    if (cli.has("intrinsics-only"))
      throw std::runtime_error("pose-check and intrinsics-only are separate operations");
    if (rmv_config) {
      CalibrationRmvImu imu(yaml["xuc_serial_port"].as<std::string>(), yaml["xuc_serial_baud"].as<uint32_t>());
      std::this_thread::sleep_for(std::chrono::duration<double>(check_seconds));
      const auto pose = imu.sample();
      auto report = rmv_metadata(pose);
      report["fresh"] = CalibrationRmvState::fresh(pose);
      report["stationary"] = CalibrationRmvState::can_save(pose);
      report["error"] = pose.error;
      report["allowed_outgoing_commands"] = {"HELLO", "HEARTBEAT(host_state=0)", "TIME_SYNC"};
      fmt::print("RMV {}\n", report.dump());
      return CalibrationRmvState::fresh(pose) ? 0 : 2;
    }
    if (!hipnuc_config) throw std::runtime_error("pose-check requires calibration_pose_source: hipnuc");
    CalibrationImu imu(yaml["xuc_serial_port"].as<std::string>(), yaml["xuc_serial_baud"].as<uint32_t>());
    std::this_thread::sleep_for(std::chrono::duration<double>(check_seconds));
    const auto pose=imu.sample();
    fmt::print("HI91 samples={} fresh={} stationary={} device_ms={} q(wxyz)=[{}, {}, {}, {}]\n",
      pose.count, CalibrationImu::fresh(pose), CalibrationImu::can_save(pose), pose.device_ms,
      pose.q.w(), pose.q.x(), pose.q.y(), pose.q.z());
    return CalibrationImu::fresh(pose) ? 0 : 2;
  }
  int pattern_cols = yaml["pattern_cols"].as<int>();
  int pattern_rows = yaml["pattern_rows"].as<int>();

  // 是否有 C 板/CAN（手眼标定必需；仅内参标定可关掉）
  bool use_cboard = true;
  if (yaml["require_cboard"]) use_cboard = yaml["require_cboard"].as<bool>();
  bool use_xuc = yaml["xuc_enable"] && yaml["xuc_enable"].as<bool>();
  if (use_cboard && use_xuc) {
    tools::logger()->warn("require_cboard 和 xuc_enable 同时为 true；优先使用 XUC 姿态");
    use_cboard = false;
  }

  bool use_hipnuc=hipnuc_config;
  bool use_rmv=rmv_config;
  if (use_hipnuc || use_rmv) { use_cboard=false; use_xuc=false; }
  if (cli.has("intrinsics-only")) {
    use_hipnuc=false;
    use_rmv=false;
    use_cboard = false;
    use_xuc = false;
  }
  const bool mounted_verified = yaml["calibration_pose_verified"] && yaml["calibration_pose_verified"].as<bool>();
  const bool raw_verified = yaml["calibration_raw_pose_verified"] && yaml["calibration_raw_pose_verified"].as<bool>();
  if (!calibration_capture_allowed(use_rmv, use_hipnuc, mounted_verified, raw_verified))
    throw std::runtime_error("Verify small-Y AND pitch rigid raw pose response before RMV capture; verify mounting axes for HI91 capture");
  std::filesystem::create_directories(output_folder);
  const std::string kind = (use_cboard || use_xuc || use_hipnuc || use_rmv) ? "image_and_pose" : "intrinsics_only";
  const auto marker = std::filesystem::path(output_folder) / "dataset_kind.txt";
  if (std::filesystem::exists(marker)) {
    std::ifstream in(marker); std::string existing; in >> existing;
    if (existing != kind) throw std::runtime_error("Dataset mode differs; do not mix intrinsic and pose samples");
  } else {
    if (!std::filesystem::is_empty(output_folder))
      throw std::runtime_error("Unlabelled existing dataset: choose a new empty data directory");
    std::ofstream(marker) << kind << "\n";
  }

  if (use_hipnuc || use_rmv) {
    const std::string expected_source = use_rmv ? "rmv_sensor1_frame1_wxyz" : "hipnuc_uart3_wxyz";
    const auto source=std::filesystem::path(output_folder)/"pose_source.txt";
    if (std::filesystem::exists(source)) {
      std::ifstream in(source); std::string value; in >> value;
      if (value != expected_source) throw std::runtime_error("Different pose source in dataset");
    } else {
      for (const auto& e:std::filesystem::directory_iterator(output_folder))
        if (e.path().extension()==".jpg")
          throw std::runtime_error("Existing images have no verified pose source");
      std::ofstream(source) << expected_source << '\n';
    }
  }

  // 主循环，保存图片和对应四元数
  check_capture_gui(false); // Fail before camera/serial acquisition, not at the first frame.
  capture_loop(config_path, output_folder, pattern_cols, pattern_rows, use_cboard, use_xuc, use_hipnuc, use_rmv);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}

int main(int argc, char * argv[])
{
  try { return run(argc, argv); }
  catch (const std::exception& error) {
    fmt::print(stderr, "capture: {}\n", error.what());
    return 2;
  }
}

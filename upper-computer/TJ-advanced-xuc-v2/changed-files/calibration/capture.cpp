#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdexcept>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "io/xuc.hpp"
#include "tools/cmdline.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder,
  int pattern_cols, int pattern_rows, bool use_cboard, bool use_xuc)
{
  // CBoard 构造函数会阻塞等待 C 板 CAN 四元数帧。
  // 没有 C 板时（仅做内参标定）跳过，四元数以单位四元数代替。
  std::unique_ptr<io::CBoard> cboard;
  if (use_cboard) {
    cboard = std::make_unique<io::CBoard>(config_path);
  } else if (!use_xuc) {
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
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  cv::Size pattern_size(pattern_cols, pattern_rows);
  int count = 0;
  int failed_count = 0;

  tools::logger()->info("=== 棋盘格标定数据采集 ===");
  tools::logger()->info("标定板尺寸: {}x{} 内角点", pattern_cols, pattern_rows);
  tools::logger()->info("按 's' 键保存, 按 'q' 键退出");

  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    bool pose_valid = !cboard && !xuc;
    if (cboard) {
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
      if (!pose_valid) {
        tools::logger()->warn("XUC/IMU 姿态无效，拒绝保存这一帧");
        continue;
      }

      // 保存图片和四元数
      count++;
      auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
      auto q_path = fmt::format("{}/{}.txt", output_folder, count);
      cv::Mat saved_bgr;
      cv::cvtColor(img, saved_bgr, cv::COLOR_RGB2BGR);
      cv::imwrite(img_path, saved_bgr);
      write_q(q_path, q);
      tools::logger()->info("[{}] Saved to {} (corners: {})", count, output_folder, corners.size());
    }
  }

  tools::logger()->info("采集完成！共 {} 组数据", count);
  if (count < 10) {
    tools::logger()->warn("建议至少采集10-15组数据");
  }
}

int main(int argc, char * argv[])
{
  // 读取命令行参数（规整 "-o xxx" 这类空格写法，OpenCV CommandLineParser 只认 '=' 形式）
  auto parsed = tools::normalize_argv(argc, argv);
  cv::CommandLineParser cli(parsed.argc(), parsed.argv(), keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 新建输出文件夹
  std::filesystem::create_directory(output_folder);

  // 读取标定板尺寸
  auto yaml = YAML::LoadFile(config_path);
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

  // 主循环，保存图片和对应四元数
  capture_loop(config_path, output_folder, pattern_cols, pattern_rows, use_cboard, use_xuc);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  return 0;
}

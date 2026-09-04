// 外参验证程序 verify_handeye
//
// 目的: 量化验证当前 R_camera2gimbal / t_camera2gimbal 是否自洽(参考 tests/handeye_test.cpp
//       与 Solver::world2pixel 的世界系->像素投影思路)。
//
// 数据流(与运行时一致):
//   IMU: 运行时 io::CBoard 从 SocketCAN(vcan0/can0, 0x150) 读四元数, solver.set_R_gimbal2world
//        用 R_gimbal2imubody 做相似变换得 R_gimbal2world。这里直接复用 auto_aim::Solver。
//   棋盘: solvePnP 得板在相机系位姿, 经「外参 -> 云台 -> 世界」链投影到世界系。
//
// 核心判据: 棋盘固定在世界上, 每帧板位姿投影到世界系后必须重合。残差统计即外参质量。
//
// 用法:
//   ./build/verify_handeye -c=configs/calibration_hikrobot.yaml \
//                          -e=configs/standard_hikrobot.yaml \
//                          -i=assets/img_with_q
//   --live                 实时模式(相机 + CBoard/虚拟CAN)
//   --compare-roll-flip    额外用 R_c2g*Rz(180) 跑一遍对比(检验 roll 修正方向)
//   --verbose              打印每帧明细
//   -o=out.csv             导出每帧残差
//   --display              显示每帧检测(红)+投影(绿)叠层

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

namespace
{
constexpr double RAD2DEG = 57.29577951308232;

// 角度标准差(度): 先解环绕到首值附近, 再求总体标准差; 避免 ±180 环绕与均值偏置(RMS)污染
double angle_std(std::vector<double> degs)
{
  if (degs.empty()) return 0;
  const double ref = degs[0];
  for (auto & v : degs) {
    while (v - ref > 180) v -= 360;
    while (v - ref < -180) v += 360;
  }
  double mean = 0;
  for (double v : degs) mean += v;
  mean /= degs.size();
  double var = 0;
  for (double v : degs) var += (v - mean) * (v - mean);
  return std::sqrt(var / degs.size());
}

const std::string keys =
  "{help h usage ?      |                                  | 输出命令行参数说明}"
  "{config-path c       | configs/calibration_hikrobot.yaml | 标定配置: 板尺寸/square_size/内参/R_gimbal2imubody}"
  "{extrinsics-path e   | configs/standard_hikrobot.yaml | 运行时配置: 含 R_camera2gimbal / t_camera2gimbal}"
  "{input-folder i      | assets/img_with_q              | 离线输入文件夹(capture 保存的 1.jpg+1.txt 序列)}"
  "{live                |                                | 实时模式: 相机+CBoard(虚拟CAN) 实时采集并验证}"
  "{compare-roll-flip   |                                | 额外对比 R_c2g*Rz(180) (检验 roll 修正方向)}"
  "{verbose             |                                | 打印每帧明细}"
  "{output-csv o        |                                | 导出每帧残差 CSV}"
  "{display d           |                                | 显示每帧检测+投影叠层}";

struct Params {
  int cols = 0;
  int rows = 0;
  double square_mm = 0;
  Eigen::Matrix3d R_gimbal2imubody = Eigen::Matrix3d::Identity();
  cv::Matx33d K{};
  cv::Mat D;
  Eigen::Matrix3d R_c2g = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_c2g = Eigen::Vector3d::Zero();  // 米
};

Params load_params(const std::string & c_path, const std::string & e_path)
{
  Params p;
  auto cyaml = YAML::LoadFile(c_path);
  p.cols = cyaml["pattern_cols"].as<int>();
  p.rows = cyaml["pattern_rows"].as<int>();
  p.square_mm = cyaml["square_size_mm"].as<double>();
  auto R_g2i_data = cyaml["R_gimbal2imubody"].as<std::vector<double>>();
  p.R_gimbal2imubody = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_g2i_data.data());
  auto K_data = cyaml["camera_matrix"].as<std::vector<double>>();
  auto D_data = cyaml["distort_coeffs"].as<std::vector<double>>();
  p.K = cv::Matx33d(K_data.data());
  p.D = cv::Mat(D_data);

  auto eyaml = YAML::LoadFile(e_path);
  auto R_c2g_data = eyaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_c2g_data = eyaml["t_camera2gimbal"].as<std::vector<double>>();
  p.R_c2g = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_c2g_data.data());
  p.t_c2g = Eigen::Vector3d(t_c2g_data.data());  // 米
  return p;
}

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float square)
{
  std::vector<cv::Point3f> pts;
  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      pts.push_back({j * square, i * square, 0});
  return pts;
}

Eigen::Quaterniond read_q(const std::string & q_path)
{
  std::ifstream f(q_path);
  double w, x, y, z;
  f >> w >> x >> y >> z;
  return {w, x, y, z};
}

struct Sample {
  Eigen::Quaterniond q;
  std::vector<cv::Point2f> corners;
  cv::Mat img;
};

bool detect_board(
  const cv::Mat & img, const cv::Size & pattern, std::vector<cv::Point2f> & corners)
{
  if (!cv::findChessboardCorners(
        img, pattern, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE))
    return false;
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  cv::cornerSubPix(
    gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
  return true;
}

std::vector<Sample> collect_offline(const std::string & folder, const cv::Size & pattern)
{
  std::vector<Sample> samples;
  for (int i = 1; true; i++) {
    auto img_path = fmt::format("{}/{}.jpg", folder, i);
    auto q_path = fmt::format("{}/{}.txt", folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;
    std::vector<cv::Point2f> corners;
    if (!detect_board(img, pattern, corners)) {
      fmt::print("[skip] 未检测到棋盘: {}\n", img_path);
      continue;
    }
    samples.push_back({read_q(q_path), std::move(corners), img});
  }
  return samples;
}

std::vector<Sample> collect_live(const std::string & config_path, const cv::Size & pattern)
{
  // CBoard 构造会阻塞等 CAN 四元数帧, 需先起虚拟CAN + imu_to_can 桥
  auto cboard = std::make_unique<io::CBoard>(config_path);
  io::Camera camera(config_path);
  std::vector<Sample> samples;
  while (true) {
    cv::Mat img;
    std::chrono::steady_clock::time_point ts;
    camera.read(img, ts);
    auto q = cboard->imu_at(ts);

    std::vector<cv::Point2f> corners;
    bool ok = detect_board(img, pattern, corners);

    auto disp = img.clone();
    cv::drawChessboardCorners(disp, pattern, corners, ok);
    tools::draw_text(
      disp, fmt::format("Captured: {}  's' save  'q' quit", samples.size()), {10, disp.rows - 20},
      {255, 255, 255});
    cv::resize(disp, disp, {}, 0.5, 0.5);
    cv::imshow("verify_handeye live", disp);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key == 's' && ok) {
      samples.push_back({q, std::move(corners), img});
      fmt::print("[{}] 已保存\n", samples.size());
    }
  }
  return samples;
}

struct FrameData {
  Eigen::Quaterniond q;
  Eigen::Matrix3d R_g2w;   // 生产同款 Solver 相似变换结果
  Eigen::Matrix3d R_t2c;   // 板在相机系(毫米)
  Eigen::Vector3d t_t2c;
  std::vector<cv::Point2f> corners;
  cv::Mat img;
};

std::vector<FrameData> build_frames(
  const std::vector<Sample> & samples, const Params & p, auto_aim::Solver & solver)
{
  std::vector<FrameData> frames;
  auto pts3d = centers_3d(cv::Size(p.cols, p.rows), static_cast<float>(p.square_mm));
  cv::Mat K(p.K), D = p.D;
  for (const auto & s : samples) {
    solver.set_R_gimbal2world(s.q);
    FrameData f;
    f.q = s.q;
    f.R_g2w = solver.R_gimbal2world();
    f.corners = s.corners;
    f.img = s.img;
    cv::Mat rvec, tvec;
    cv::solvePnP(pts3d, s.corners, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::cv2eigen(R, f.R_t2c);
    cv::cv2eigen(tvec, f.t_t2c);
    frames.push_back(std::move(f));
  }
  return frames;
}

struct WorldPose {
  Eigen::Matrix3d R_t2w;
  Eigen::Vector3d t_t2w_mm;
};

std::vector<WorldPose> world_poses(
  const std::vector<FrameData> & frames, const Eigen::Matrix3d & R_c2g,
  const Eigen::Vector3d & t_c2g_mm)
{
  std::vector<WorldPose> wps;
  for (const auto & f : frames) {
    Eigen::Matrix3d R_t2g = R_c2g * f.R_t2c;
    Eigen::Vector3d t_t2g = R_c2g * f.t_t2c + t_c2g_mm;
    wps.push_back({f.R_g2w * R_t2g, f.R_g2w * t_t2g});
  }
  return wps;
}

Eigen::Quaterniond q_average(const std::vector<Eigen::Quaterniond> & qs)
{
  Eigen::Vector4d acc = Eigen::Vector4d::Zero();
  for (auto q : qs) {
    if (q.w() < 0) q.coeffs() *= -1;  // 消除 ±q 歧义
    acc += q.coeffs();
  }
  Eigen::Quaterniond avg(acc.normalized());
  if (avg.w() < 0) avg.coeffs() *= -1;
  return avg;
}

struct Report {
  int n = 0;
  double yaw_min = 0, yaw_max = 0, pitch_min = 0, pitch_max = 0, roll_min = 0, roll_max = 0;
  double pos_mean = 0, pos_std = 0, pos_max = 0, x_std = 0, y_std = 0, z_std = 0;
  double ori_mean = 0, ori_std = 0, ori_max = 0, w_yaw_std = 0, w_pitch_std = 0, w_roll_std = 0;
  double dir_yaw_std = 0, dir_pitch_std = 0;
  double axxb_mean = 0, axxb_max = 0;
  double rep_mean = 0, rep_std = 0, rep_max = 0;
  // 每帧 14 列: i gimbal_ypr(3) world_xyz(3) pos_res ori_res world_ypr(3) dir_ypr(2)
  std::vector<std::vector<double>> per_frame;
};

Report compute_report(
  const std::vector<FrameData> & frames, const Params & p, const Eigen::Matrix3d & R_c2g,
  const Eigen::Vector3d & t_c2g_m, bool display, bool verbose, auto_aim::Solver * solver)
{
  Report r;
  const int n = static_cast<int>(frames.size());
  r.n = n;
  if (n < 3) return r;
  const Eigen::Vector3d t_c2g_mm = t_c2g_m * 1000.0;

  auto wps = world_poses(frames, R_c2g, t_c2g_mm);

  // [0] 云台运动范围(观测性检查); 角度先解环绕, 否则 ±180 环绕会让 min/max 失真
  std::vector<double> yaw_s, pitch_s, roll_s;
  for (const auto & f : frames) {
    Eigen::Vector3d ypr = tools::eulers(f.R_g2w, 2, 1, 0) * RAD2DEG;
    yaw_s.push_back(ypr[0]);
    pitch_s.push_back(ypr[1]);
    roll_s.push_back(ypr[2]);
  }
  auto unwrap = [](std::vector<double> v) {
    for (std::size_t i = 1; i < v.size(); i++) {
      while (v[i] - v[i - 1] > 180) v[i] -= 360;
      while (v[i] - v[i - 1] < -180) v[i] += 360;
    }
    return v;
  };
  auto uy = unwrap(yaw_s), up = unwrap(pitch_s), ur = unwrap(roll_s);
  r.yaw_min = *std::min_element(uy.begin(), uy.end());
  r.yaw_max = *std::max_element(uy.begin(), uy.end());
  r.pitch_min = *std::min_element(up.begin(), up.end());
  r.pitch_max = *std::max_element(up.begin(), up.end());
  r.roll_min = *std::min_element(ur.begin(), ur.end());
  r.roll_max = *std::max_element(ur.begin(), ur.end());

  // 均值世界位姿
  Eigen::Vector3d t_mean = Eigen::Vector3d::Zero();
  for (const auto & wp : wps) t_mean += wp.t_t2w_mm;
  t_mean /= n;
  std::vector<Eigen::Quaterniond> qs;
  for (const auto & wp : wps) qs.emplace_back(wp.R_t2w);
  Eigen::Quaterniond q_mean = q_average(qs);
  Eigen::Matrix3d R_mean = q_mean.toRotationMatrix();

  // [1] 板世界系位置残差
  std::vector<double> pos_res(n);
  double pos_sum = 0, max_pos = 0;
  Eigen::Vector3d t_var = Eigen::Vector3d::Zero();
  for (int i = 0; i < n; i++) {
    pos_res[i] = (wps[i].t_t2w_mm - t_mean).norm();
    pos_sum += pos_res[i];
    max_pos = std::max(max_pos, pos_res[i]);
    t_var += (wps[i].t_t2w_mm - t_mean).array().square().matrix();
  }
  r.pos_mean = pos_sum / n;
  double pos_var = 0;
  for (double v : pos_res) pos_var += (v - r.pos_mean) * (v - r.pos_mean);
  r.pos_std = std::sqrt(pos_var / n);
  r.pos_max = max_pos;
  t_var /= n;
  r.x_std = std::sqrt(t_var[0]);
  r.y_std = std::sqrt(t_var[1]);
  r.z_std = std::sqrt(t_var[2]);

  // [2] 板世界系姿态残差
  std::vector<double> ori_res(n);
  std::vector<Eigen::Vector3d> world_euler(n);
  std::vector<double> w_yaw(n), w_pitch(n), w_roll(n);
  double ori_sum = 0, max_ori = 0;
  for (int i = 0; i < n; i++) {
    ori_res[i] = std::fabs(Eigen::AngleAxisd(R_mean.transpose() * wps[i].R_t2w).angle()) * RAD2DEG;
    ori_sum += ori_res[i];
    max_ori = std::max(max_ori, ori_res[i]);
    world_euler[i] = tools::eulers(wps[i].R_t2w, 2, 1, 0) * RAD2DEG;
    w_yaw[i] = world_euler[i][0];
    w_pitch[i] = world_euler[i][1];
    w_roll[i] = world_euler[i][2];
  }
  r.ori_mean = ori_sum / n;
  double ori_var = 0;
  for (double v : ori_res) ori_var += (v - r.ori_mean) * (v - r.ori_mean);
  r.ori_std = std::sqrt(ori_var / n);
  r.ori_max = max_ori;
  r.w_yaw_std = angle_std(w_yaw);
  r.w_pitch_std = angle_std(w_pitch);
  r.w_roll_std = angle_std(w_roll);

  // [3] 指向板方向角(世界系 yaw/pitch)
  std::vector<double> d_yaw(n), d_pitch(n);
  for (int i = 0; i < n; i++) {
    Eigen::Vector3d dir = wps[i].t_t2w_mm.normalized();
    d_yaw[i] = std::atan2(dir[1], dir[0]) * RAD2DEG;
    d_pitch[i] = std::atan2(dir[2], std::sqrt(dir[0] * dir[0] + dir[1] * dir[1])) * RAD2DEG;
  }
  r.dir_yaw_std = angle_std(d_yaw);
  r.dir_pitch_std = angle_std(d_pitch);

  // [4] 手眼方程残差 AX=XB
  double axxb_sum = 0, max_axxb = 0;
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++) {
      // OpenCV hand-eye convention:
      //   G_i X C_i = G_j X C_j
      //   (G_j^-1 G_i) X = X (C_j C_i^-1)
      // where G is gripper(gimbal)->base(world), X is camera->gimbal,
      // and C is target(board)->camera.
      Eigen::Matrix3d A = frames[j].R_g2w.transpose() * frames[i].R_g2w;
      Eigen::Matrix3d B = frames[j].R_t2c * frames[i].R_t2c.transpose();
      double ang =
        std::fabs(Eigen::AngleAxisd(R_c2g.transpose() * A * R_c2g * B.transpose()).angle()) *
        RAD2DEG;
      axxb_sum += ang;
      max_axxb = std::max(max_axxb, ang);
    }
  r.axxb_mean = axxb_sum / (n * (n - 1) / 2.0);
  r.axxb_max = max_axxb;

  // [5] hand_eye 重投影误差: 板世界位姿均值 -> 逐帧 world2pixel 投影 vs 检测角点
  //     (投影公式与 Solver::world2pixel solver.cpp:311-337 同构)
  auto board3d_mm = centers_3d(cv::Size(p.cols, p.rows), static_cast<float>(p.square_mm));
  std::vector<cv::Point3f> board_world_m;
  for (const auto & c : board3d_mm) {
    Eigen::Vector3d wp_m = (R_mean * Eigen::Vector3d(c.x, c.y, c.z) + t_mean) / 1000.0;
    board_world_m.push_back(
      {static_cast<float>(wp_m[0]), static_cast<float>(wp_m[1]), static_cast<float>(wp_m[2])});
  }
  std::vector<double> rep_all;
  std::vector<std::vector<cv::Point2f>> frame_px(n);
  std::vector<std::vector<int>> frame_valid(n);
  for (int i = 0; i < n; i++) {
    Eigen::Matrix3d R_w2c = R_c2g.transpose() * frames[i].R_g2w.transpose();
    Eigen::Vector3d t_w2c = -R_c2g.transpose() * t_c2g_m;  // 米
    std::vector<cv::Point3f> valid;
    std::vector<int> valid_idx;
    for (int k = 0; k < static_cast<int>(board_world_m.size()); k++) {
      Eigen::Vector3d cw(board_world_m[k].x, board_world_m[k].y, board_world_m[k].z);
      if ((R_w2c * cw + t_w2c)[2] > 0) {
        valid.push_back(board_world_m[k]);
        valid_idx.push_back(k);
      }
    }
    cv::Mat rvec, tvec_cv;
    cv::eigen2cv(R_w2c, rvec);
    cv::eigen2cv(t_w2c, tvec_cv);
    std::vector<cv::Point2f> px;
    cv::projectPoints(valid, rvec, tvec_cv, cv::Mat(p.K), p.D, px);
    frame_px[i] = px;
    frame_valid[i] = valid_idx;
    for (int k = 0; k < static_cast<int>(valid_idx.size()); k++) {
      double err = cv::norm(px[k] - frames[i].corners[valid_idx[k]]);
      rep_all.push_back(err);
    }
  }
  r.rep_mean = 0;
  if (!rep_all.empty()) {
    for (double v : rep_all) r.rep_mean += v;
    r.rep_mean /= rep_all.size();
  }
  double rep_var = 0;
  for (double v : rep_all) rep_var += (v - r.rep_mean) * (v - r.rep_mean);
  r.rep_std = rep_all.empty() ? 0 : std::sqrt(rep_var / rep_all.size());
  r.rep_max = rep_all.empty() ? 0 : *std::max_element(rep_all.begin(), rep_all.end());

  // 交叉校验: 内联投影 vs 生产 Solver::world2pixel(当前外参)
  if (solver && n > 0) {
    solver->set_R_gimbal2world(frames[0].q);
    auto px_prod = solver->world2pixel(board_world_m);
    double max_diff = 0;
    auto m = std::min(px_prod.size(), frame_px[0].size());
    for (std::size_t k = 0; k < m; k++) max_diff = std::max(max_diff, cv::norm(px_prod[k] - frame_px[0][k]));
    fmt::print("[check] Solver::world2pixel vs 内联投影 max diff = {:.4f} px\n", max_diff);
  }

  // per_frame 明细 + display 叠层
  r.per_frame.resize(n);
  for (int i = 0; i < n; i++) {
    Eigen::Vector3d gypr = tools::eulers(frames[i].R_g2w, 2, 1, 0) * RAD2DEG;
    r.per_frame[i] = {
      static_cast<double>(i + 1), gypr[0], gypr[1], gypr[2],
      wps[i].t_t2w_mm[0], wps[i].t_t2w_mm[1], wps[i].t_t2w_mm[2],
      pos_res[i], ori_res[i],
      world_euler[i][0], world_euler[i][1], world_euler[i][2],
      d_yaw[i], d_pitch[i]};
    if (verbose) {
      fmt::print(
        "[{:2d}] gimbal_ypr {:7.2f} {:7.2f} {:7.2f} | pos_res {:6.2f}mm ori_res {:6.2f}deg | "
        "board_world {:7.0f} {:7.0f} {:7.0f}\n",
        i + 1, gypr[0], gypr[1], gypr[2], pos_res[i], ori_res[i], wps[i].t_t2w_mm[0],
        wps[i].t_t2w_mm[1], wps[i].t_t2w_mm[2]);
    }
    if (display) {
      auto disp = frames[i].img.clone();
      for (const auto & c : frames[i].corners) cv::circle(disp, c, 3, cv::Scalar(0, 0, 255), -1);
      for (const auto & px : frame_px[i]) cv::circle(disp, px, 3, cv::Scalar(0, 255, 0), -1);
      tools::draw_text(
        disp, fmt::format("frame {}  red=detected green=projected", i + 1), {10, 30},
        {255, 255, 255});
      cv::imshow("verify_handeye - any key to next", disp);
      cv::waitKey(0);
    }
  }

  return r;
}

void print_report(const std::string & title, const Report & r)
{
  fmt::print("\n===== {} ({} 帧) =====\n", title, r.n);
  fmt::print(
    "[0] 云台运动范围(deg): yaw[{:.1f},{:.1f}] pitch[{:.1f},{:.1f}] roll[{:.1f},{:.1f}]\n",
    r.yaw_min, r.yaw_max, r.pitch_min, r.pitch_max, r.roll_min, r.roll_max);
  if (r.roll_max - r.roll_min < 5.0)
    fmt::print("    ^ roll 未展宽(范围<5°), roll 方向无判别力(退化)\n");
  else
    fmt::print("    ^ roll 已展宽({:.1f}°), 可判别 roll 方向\n", r.roll_max - r.roll_min);
  fmt::print(
    "[1] 板世界系位置残差(mm):  mean={:.2f} std={:.2f} max={:.2f}   xyz_std={:.2f}/{:.2f}/{:.2f}\n",
    r.pos_mean, r.pos_std, r.pos_max, r.x_std, r.y_std, r.z_std);
  fmt::print(
    "[2] 板世界系姿态残差(deg): mean={:.2f} std={:.2f} max={:.2f}   world_ypr_std={:.2f}/{:.2f}/{:.2f}\n",
    r.ori_mean, r.ori_std, r.ori_max, r.w_yaw_std, r.w_pitch_std, r.w_roll_std);
  fmt::print("[3] 指向板方向角(deg):     yaw_std={:.2f} pitch_std={:.2f}\n", r.dir_yaw_std, r.dir_pitch_std);
  fmt::print("[4] 手眼方程残差 AX=XB(deg): mean={:.2f} max={:.2f}\n", r.axxb_mean, r.axxb_max);
  fmt::print("[5] 重投影误差(px):         mean={:.2f} std={:.2f} max={:.2f}\n", r.rep_mean, r.rep_std, r.rep_max);
}

void write_csv(const std::string & path, const Report & r)
{
  std::ofstream f(path);
  f << "frame,gimbal_yaw,gimbal_pitch,gimbal_roll,board_world_x,board_world_y,board_world_z,"
       "pos_res_mm,ori_res_deg,board_yaw,board_pitch,board_roll,dir_yaw,dir_pitch\n";
  for (const auto & row : r.per_frame) {
    for (std::size_t k = 0; k < row.size(); k++) f << (k ? "," : "") << row[k];
    f << "\n";
  }
  fmt::print("[csv] 已写出 {}\n", path);
}

}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto c_path = cli.get<std::string>("config-path");
  auto e_path = cli.get<std::string>("extrinsics-path");
  auto in_folder = cli.get<std::string>("input-folder");
  bool live = cli.has("live");
  bool compare = cli.has("compare-roll-flip");
  bool verbose = cli.has("verbose");
  bool display = cli.has("display");
  auto csv_path = cli.get<std::string>("output-csv");

  Params p = load_params(c_path, e_path);
  cv::Size pattern(p.cols, p.rows);

  // 当前外参相对理想相机系的偏角(同 calibrate_handeye)
  Eigen::Matrix3d R_gimbal2ideal{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
  Eigen::Vector3d ypr = tools::eulers(R_gimbal2ideal * p.R_c2g, 1, 0, 2) * RAD2DEG;
  fmt::print(
    "外参来源: {} (t_camera2gimbal = [{:.4f}, {:.4f}, {:.4f}] m)\n", e_path, p.t_c2g[0], p.t_c2g[1],
    p.t_c2g[2]);
  fmt::print("相对理想偏角: yaw{:.2f} pitch{:.2f} roll{:.2f} deg\n", ypr[0], ypr[1], ypr[2]);

  std::vector<Sample> samples;
  if (live) {
    fmt::print("实时模式: 需要 相机 + 虚拟CAN(imu_to_can) 在跑, 且 -e 配置的 can_interface 正确\n");
    samples = collect_live(e_path, pattern);
  } else {
    fmt::print("离线模式: 读取 {} ...\n", in_folder);
    samples = collect_offline(in_folder, pattern);
  }
  if (samples.size() < 3) {
    fmt::print("有效样本过少({}), 无法验证\n", samples.size());
    return 1;
  }
  fmt::print("有效样本: {} 帧\n", samples.size());

  auto_aim::Solver solver(e_path);
  auto frames = build_frames(samples, p, solver);

  auto report = compute_report(frames, p, p.R_c2g, p.t_c2g, display, verbose, &solver);
  print_report("当前外参 R_camera2gimbal", report);

  if (compare) {
    Eigen::Matrix3d Rz180 = Eigen::Matrix3d::Identity();
    Rz180(0, 0) = -1;
    Rz180(1, 1) = -1;
    Eigen::Matrix3d R_flip = p.R_c2g * Rz180;  // 右乘 = 保留绕光轴 180°(昨天抹掉的那版)
    auto report_flip =
      compute_report(frames, p, R_flip, p.t_c2g, false, false, nullptr);
    print_report("对比: R_c2g * Rz(180)(roll 保留 180)", report_flip);
  }

  if (!csv_path.empty()) write_csv(csv_path, report);
  return 0;
}

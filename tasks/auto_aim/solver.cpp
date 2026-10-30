#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <limits>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);
  if (yaml["solver_max_reprojection_error_px"]) {
    max_reprojection_error_px_ = std::clamp(
      yaml["solver_max_reprojection_error_px"].as<double>(), 0.5, 8.0);
  }

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  if (R_gimbal2imubody_data.size() != 9 || R_camera2gimbal_data.size() != 9 ||
      t_camera2gimbal_data.size() != 3) {
    throw std::runtime_error("extrinsics require two 3x3 rotations and one 3-vector");
  }
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  // 验证数据大小
  if (camera_matrix_data.size() != 9) {
    throw std::runtime_error("camera_matrix must have exactly 9 elements");
  }
  if (distort_coeffs_data.size() != 5) {
    throw std::runtime_error("distort_coeffs must have exactly 5 elements");
  }

  // 将数据复制到成员数组中（确保数据生命周期）
  std::copy(camera_matrix_data.begin(), camera_matrix_data.end(), camera_matrix_data_.begin());
  std::copy(distort_coeffs_data.begin(), distort_coeffs_data.end(), distort_coeffs_data_.begin());

  // 创建 cv::Mat 包装器，指向成员数组数据
  camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_data_.data());
  distort_coeffs_ = cv::Mat(1, 5, CV_64F, distort_coeffs_data_.data());
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

bool Solver::set_R_gimbal2world_absolute_imu(const Eigen::Quaterniond & q)
{
  const auto norm = q.norm();
  if (!q.coeffs().allFinite() || norm < 0.95 || norm > 1.05) return false;
  R_gimbal2world_ = q.normalized().toRotationMatrix() * R_gimbal2imubody_;
  return true;
}

bool Solver::set_R_gimbal2world_direct(const Eigen::Quaterniond & q)
{
  const auto norm = q.norm();
  if (!q.coeffs().allFinite() || norm < 0.95 || norm > 1.05) return false;
  R_gimbal2world_ = q.normalized().toRotationMatrix();
  return true;
}

std::optional<Eigen::Vector2d> Solver::image_angular_delta(
  const Eigen::Vector3d & point_in_world,
  const Eigen::Matrix3d & from_R_gimbal2world,
  const Eigen::Matrix3d & to_R_gimbal2world,
  const cv::Size & image_size) const
{
  const double fx=camera_matrix_.at<double>(0,0), fy=camera_matrix_.at<double>(1,1);
  if (!point_in_world.allFinite() || !from_R_gimbal2world.allFinite() ||
      !to_R_gimbal2world.allFinite() || image_size.width<=0 || image_size.height<=0 ||
      !std::isfinite(fx) || !std::isfinite(fy) || fx<=0 || fy<=0) return std::nullopt;
  std::vector<cv::Point3d> camera_points;
  for (const auto * rotation : {&from_R_gimbal2world, &to_R_gimbal2world}) {
    const Eigen::Vector3d camera_point = R_camera2gimbal_.transpose() *
      (rotation->transpose() * point_in_world - t_camera2gimbal_);
    if (!camera_point.allFinite() || camera_point.z()<=0) return std::nullopt;
    camera_points.emplace_back(camera_point.x(),camera_point.y(),camera_point.z());
  }
  try {
    std::vector<cv::Point2d> pixels;
    cv::projectPoints(camera_points,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),
      camera_matrix_,distort_coeffs_,pixels);
    if (pixels.size()!=2) return std::nullopt;
    for (const auto & pixel : pixels)
      if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y)) return std::nullopt;
    const auto angle = [&](const cv::Point2d & p) {
      return Eigen::Vector2d(std::atan2(p.x-0.5*image_size.width,fx),
        std::atan2(p.y-0.5*image_size.height,fy));
    };
    const Eigen::Vector2d delta = angle(pixels[1])-angle(pixels[0]);
    if (!delta.allFinite()) return std::nullopt;
    return delta;
  } catch (const cv::Exception &) {
    return std::nullopt;
  }
}

std::optional<Eigen::Vector2d> Solver::image_point_delta(
  const Eigen::Vector3d & from_world, const Eigen::Vector3d & to_world,
  const Eigen::Matrix3d & rotation, const cv::Size & size) const
{
  const double fx=camera_matrix_.at<double>(0,0),fy=camera_matrix_.at<double>(1,1);
  if (!from_world.allFinite() || !to_world.allFinite() || !rotation.allFinite() ||
      size.width<=0 || size.height<=0 || fx<=0 || fy<=0) return {};
  std::vector<cv::Point3d> points;
  for(const auto * world : {&from_world,&to_world}) {
    const Eigen::Vector3d camera=R_camera2gimbal_.transpose()*(rotation.transpose()*(*world)-t_camera2gimbal_);
    if(!camera.allFinite() || camera.z()<=0)return {};
    points.emplace_back(camera.x(),camera.y(),camera.z());
  }
  try {
    std::vector<cv::Point2d> pixels;
    cv::projectPoints(points,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),camera_matrix_,distort_coeffs_,pixels);
    const Eigen::Vector2d result(
      std::atan2(pixels[1].x-.5*size.width,fx)-std::atan2(pixels[0].x-.5*size.width,fx),
      std::atan2(pixels[1].y-.5*size.height,fy)-std::atan2(pixels[0].y-.5*size.height,fy));
    return result.allFinite() ? std::optional<Eigen::Vector2d>(result) : std::nullopt;
  }catch(const cv::Exception&){return {};}
}

//solvePnP（获得姿态）
namespace
{
struct PoseGateDiagnostics
{
  Armor & armor;
  double max_reprojection_error_px;
  double width = 0.0, height = 0.0;
  double pnp_error = std::numeric_limits<double>::quiet_NaN();
  double model_error = std::numeric_limits<double>::quiet_NaN();
  void operator()(std::size_t stage, const char * reason) const
  {
    static thread_local std::array<std::uint64_t, 7> outcomes{};
    static thread_local std::uint64_t calls = 0, model_mismatch = 0;
    static thread_local auto last_report = std::chrono::steady_clock::time_point{};
    ++calls;
    ++outcomes[stage];
    if (stage == 6 && (!std::isfinite(model_error) || model_error > max_reprojection_error_px))
      ++model_mismatch;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_report < std::chrono::seconds(1)) return;
    last_report = now;
    tools::logger()->info(
      "[POSE_GATE] calls={} accepted={} reject_points={} reject_geometry={} reject_pnp={} "
      "reject_raw_rms={} reject_nonfinite={} exceptions={} fixed_model_mismatch={} "
      "last={} class={} name={} type={} conf={:.3f} width_px={:.1f} height_px={:.1f} "
      "raw_rms_px={:.2f} fixed_model_rms_px={:.2f} quality={:.3f}",
      calls, outcomes[6], outcomes[0], outcomes[1], outcomes[2], outcomes[3], outcomes[4],
      outcomes[5], model_mismatch, reason, armor.class_id, static_cast<int>(armor.name),
      static_cast<int>(armor.type), armor.confidence, width, height, pnp_error, model_error,
      armor.pose_quality);
  }
};

double reprojection_rms(
  const std::vector<cv::Point2f> & projected, const std::vector<cv::Point2f> & measured)
{
  if (projected.size() != measured.size())
    return std::numeric_limits<double>::infinity();
  double squared_error = 0.0;
  for (std::size_t i = 0; i < projected.size(); ++i) {
    const double error = cv::norm(projected[i] - measured[i]);
    squared_error += error * error;
  }
  return std::sqrt(squared_error / projected.size());
}
}  // namespace

void Solver::solve(Armor & armor) const
{
  armor.invalidate_pose();
  // Keep image-fit rejection separate from the optional fixed-inclination
  // model. The latter also measures mounting/model mismatch in real footage.
  PoseGateDiagnostics report{armor, max_reprojection_error_px_};
  if (!armor.pose_observation_valid) {
    report(0, "unverified_image_geometry");
    return;
  }
  if (armor.points.size() != 4 ||
      !std::all_of(armor.points.begin(), armor.points.end(), [](const cv::Point2f & p) {
        return std::isfinite(p.x) && std::isfinite(p.y);
      })) { report(0, "invalid_points"); return; }

  // A finite planar PnP result is not evidence that the input is an armor.
  // Reject collapsed, crossed and very unbalanced corner sets before IPPE can
  // explain them as an almost edge-on plate with a plausible-looking range.
  const double left = cv::norm(armor.points[3] - armor.points[0]);
  const double right = cv::norm(armor.points[2] - armor.points[1]);
  const double top = cv::norm(armor.points[1] - armor.points[0]);
  const double bottom = cv::norm(armor.points[2] - armor.points[3]);
  report.height = 0.5 * (left + right);
  report.width = 0.5 * (top + bottom);
  const double area = std::abs(cv::contourArea(armor.points));
  if (std::min(left, right) < 2.0 || std::min(top, bottom) < 3.0 ||
      !cv::isContourConvex(armor.points) || area < 12.0 ||
      report.width / report.height < 0.4 || report.width / report.height > 8.0 ||
      std::max(left, right) / std::min(left, right) > 3.0 ||
      std::max(top, bottom) / std::min(top, bottom) > 3.0 ||
      area < 0.25 * report.width * report.height) { report(1, "degenerate_geometry"); return; }

  try {
    const auto & object_points =
      (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    cv::Vec3d rvec, tvec;
    const bool solved = cv::solvePnP(
      object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
      cv::SOLVEPNP_IPPE);

    if (!solved || tvec[2] <= 0) { report(2, "pnp_failed_or_behind_camera"); return; }
    for (int i = 0; i < 3; ++i)
      if (!std::isfinite(rvec[i]) || !std::isfinite(tvec[i])) {
        report(4, "pnp_nonfinite"); return;
      }

    std::vector<cv::Point2f> pnp_projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, pnp_projected);
    report.pnp_error = reprojection_rms(pnp_projected, armor.points);
    if (!std::isfinite(report.pnp_error) || report.pnp_error > max_reprojection_error_px_) {
      report(3, "raw_pnp_rms"); return;
    }

    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
    armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
    if (!armor.xyz_in_gimbal.allFinite() || !armor.xyz_in_world.allFinite() ||
        !armor.ypr_in_gimbal.allFinite() || !armor.ypr_in_world.allFinite() ||
        !armor.ypd_in_world.allFinite()) {
      armor.invalidate_pose();
      report(4, "world_pose_nonfinite");
      return;
    }

    // 平衡不做yaw优化，因为pitch假设不成立
    auto is_balance = (armor.type == ArmorType::big) &&
                      (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                       armor.name == ArmorName::five);
    if (!is_balance) optimize_yaw(armor);
    else armor.yaw_raw = armor.ypr_in_world[0];
    if (!armor.ypr_in_world.allFinite() || !std::isfinite(armor.yaw_raw)) {
      armor.invalidate_pose();
      report(4, "optimized_yaw_nonfinite");
      return;
    }

    // Raw IPPE already passed the image-fit gate. A large residual in the
    // fixed 15-degree model can be a real mounting/inclination mismatch, not
    // an invalid detected plate. Preserve the valid measured pose and express
    // that mismatch as lower model confidence instead of deleting detection.
    report.model_error = is_balance ? report.pnp_error : reprojection_rms(
      reproject_armor(armor.xyz_in_world, armor.ypr_in_world[0], armor.type, armor.name), armor.points);
    armor.reprojection_error_px = report.pnp_error;
    armor.model_reprojection_error_px = report.model_error;
    const double normalized_raw = report.pnp_error / max_reprojection_error_px_;
    const double normalized_model = report.model_error / max_reprojection_error_px_;
    armor.pose_quality = std::isfinite(normalized_model)
      ? 1.0 / (1.0 + normalized_raw * normalized_raw + normalized_model * normalized_model) : 0.0;
    armor.pose_valid = true;
    // Diagnose physical orientation without changing the fitted/control pose.
    // A full PnP round-trip alone cannot validate hand-eye calibration: the
    // same wrong extrinsics can cancel in forward/inverse transformations.
    static thread_local auto last_geometry_report = std::chrono::steady_clock::time_point{};
    const auto geometry_now = std::chrono::steady_clock::now();
    if (armor.confidence >= 0.3 &&
        geometry_now - last_geometry_report >= std::chrono::seconds(1)) {
      last_geometry_report = geometry_now;
      try {
        const Eigen::Vector3d raw_world_ypr = tools::eulers(R_armor2world, 2, 1, 0);
        const Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);
        const Eigen::Vector3d camera_ypr = tools::eulers(R_gimbal2world_ * R_camera2gimbal_, 2, 1, 0);
        const double euler_reconstruction_error =
          (tools::rotation_matrix(raw_world_ypr) - R_armor2world).norm();
        const auto model_points = reproject_armor(
          armor.xyz_in_world, armor.ypr_in_world[0], armor.type, armor.name);
        const auto observed_edge = armor.points[1] - armor.points[0];
        const auto model_edge = model_points[1] - model_points[0];
        const double observed_angle = std::atan2(observed_edge.y, observed_edge.x);
        const double model_angle = std::atan2(model_edge.y, model_edge.x);
        cv::Point2f center_delta(0.f, 0.f);
        for (std::size_t i = 0; i < 4; ++i)
          center_delta += (model_points[i] - armor.points[i]) * 0.25f;
        const double deg = 180.0 / CV_PI;
        tools::logger()->info(
          "[POSE_GEOMETRY] class={} type={} conf={:.3f} "
          "raw_world_ypr_deg=({:.2f},{:.2f},{:.2f}) "
          "gimbal_world_ypr_deg=({:.2f},{:.2f},{:.2f}) "
          "camera_world_ypr_deg=({:.2f},{:.2f},{:.2f}) "
          "optimized_yaw_deg={:.2f} fixed_model_applicable={} fixed_pitch_deg={} "
          "observed_edge_deg={:.2f} model_edge_deg={:.2f} edge_delta_deg={:.2f} "
          "center_delta_px=({:.2f},{:.2f}) raw_rms_px={:.2f} model_rms_px={:.2f} "
          "euler_reconstruction_error={:.3g} "
          "xyz_world_m=({:.4f},{:.4f},{:.4f}) "
          "corners_px=({:.2f},{:.2f};{:.2f},{:.2f};{:.2f},{:.2f};{:.2f},{:.2f})",
          armor.class_id, static_cast<int>(armor.type), armor.confidence,
          raw_world_ypr[0] * deg, raw_world_ypr[1] * deg, raw_world_ypr[2] * deg,
          gimbal_ypr[0] * deg, gimbal_ypr[1] * deg, gimbal_ypr[2] * deg,
          camera_ypr[0] * deg, camera_ypr[1] * deg, camera_ypr[2] * deg,
          armor.ypr_in_world[0] * deg, !is_balance,
          armor.name == ArmorName::outpost ? -15 : 15,
          observed_angle * deg, model_angle * deg,
          std::remainder(model_angle - observed_angle, 2.0 * CV_PI) * deg,
          center_delta.x, center_delta.y, report.pnp_error, report.model_error,
          euler_reconstruction_error,
          armor.xyz_in_world.x(), armor.xyz_in_world.y(), armor.xyz_in_world.z(),
          armor.points[0].x, armor.points[0].y, armor.points[1].x, armor.points[1].y,
          armor.points[2].x, armor.points[2].y, armor.points[3].x, armor.points[3].y);
      } catch (const cv::Exception &) {
        tools::logger()->warn("[POSE_GEOMETRY] diagnostic projection unavailable");
      }
    }
    report(6, "accepted");
  } catch (const cv::Exception &) {
    armor.invalidate_pose();
    report(5, "opencv_exception");
  }
}

std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  // Regular plate +15 degrees: at local yaw zero, its +Z top edge lies
  // farther along +X than the bottom edge (top leans back into the robot).
  // This assumption is defined in gravity-aligned world, never sensor axes.
  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
{
  // solve
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

void Solver::optimize_yaw(Armor & armor) const
{
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // degree
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw = armor.ypr_in_world[0];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  // Refine only inside the coarse winner's 1-degree neighbourhood. A global
  // ternary search would assume an unproven unimodal reprojection objective.
  const double coarse_yaw = best_yaw;
  for (int step = -9; step <= 9; ++step) {
    const double candidate = tools::limit_rad(coarse_yaw + step * 0.1 * CV_PI / 180.0);
    const double error = armor_reprojection_error(armor, candidate, 0.0);
    if (std::isfinite(error) && error < min_error) {
      min_error = error;
      best_yaw = candidate;
    }
  }

  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }
  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;
    // i - p 构成线段。过程：先移动起点，再补长度，再旋转
    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
    Eigen::Vector2d pt_d = pts[p] - pts[i];
    // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
    double pixel_dis =  // dis 是指方差平面内到原点的距离
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();
    double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
    // 平方可能是为了配合 sin 和 cos
    // 弧度差代价（0 度左右占比应该大）
    double cost_i =
      tools::square(pixel_dis * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
    // 重投影像素误差越大，越相信斜率
    cost += std::sqrt(cost_i);
  }
  return cost;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);

  return error;
}

// 世界坐标到像素坐标的转换
// std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
// {
//   Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
//   Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

//   cv::Mat R_world2camera_cv;
//   cv::Mat rvec;
//   cv::Mat tvec;
//   cv::eigen2cv(R_world2camera, R_world2camera_cv);
//   cv::Rodrigues(R_world2camera_cv, rvec);
//   cv::eigen2cv(t_world2camera, tvec);

//   std::vector<cv::Point3f> valid_world_points;
//   for (const auto & world_point : worldPoints) {
//     Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
//     Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

//     if (camera_point.z() > 0) {
//       valid_world_points.push_back(world_point);
//     }
//   }
//   // 如果没有有效点，返回空vector
//   if (valid_world_points.empty()) {
//     return std::vector<cv::Point2f>();
//   }
//   std::vector<cv::Point2f> pixelPoints;
//   cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
//   return pixelPoints;
// }
// }  // namespace auto_aim


// 世界坐标到像素坐标的转换
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec;
  cv::Mat tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }
  // 如果没有有效点，返回空vector
  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }
  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}
}  // namespace auto_aim

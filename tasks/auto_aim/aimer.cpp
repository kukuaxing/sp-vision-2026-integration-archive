#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  const auto directional_delay = [&](const char * key) {
    const double value = yaml[key] ? yaml[key].as<double>() : high_speed_delay_time_;
    if (!std::isfinite(value) || value < 0.0 || value > 1.0)
      throw std::invalid_argument(std::string(key) + " must be finite in [0,1] seconds");
    return value;
  };
  high_speed_delay_time_cw_ = directional_delay("high_speed_delay_time_cw");
  high_speed_delay_time_ccw_ = directional_delay("high_speed_delay_time_ccw");
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

void Aimer::set_delay_adjustment(double adjustment_s)
{
  delay_adjustment_time_ = std::clamp(adjustment_s, -0.050, 0.050);
}

io::Command Aimer::aim(
  const std::list<Target> & targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now, AimPredictionTrace * trace, AimGeometry geometry, bool rigid_high_speed)
{
  return aim_target(targets.empty() ? nullptr : &targets.front(), timestamp, bullet_speed,
    to_now, trace, geometry, rigid_high_speed);
}

io::Command Aimer::aim_target(
  const Target * input, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now, AimPredictionTrace * trace, AimGeometry geometry, bool rigid_high_speed)
{
  debug_aim_point = {false, Eigen::Vector4d::Zero()};
  if (trace) *trace = AimPredictionTrace{};
  if (!input) return {false, false, 0, 0};
  auto target = *input;

  const auto & raw_state = target.ekf().x;
  // Decide once at the frame boundary, so all ballistic iterations use the
  // same model. Stationary/direct-plate and outpost paths retain their model.
  const bool whole_ekf_geometry = geometry == AimGeometry::whole_ekf &&
    target.armor_num() == 4 && target.name != ArmorName::outpost &&
    raw_state.size() == 11 && raw_state.allFinite() &&
    std::abs(raw_state[7]) > decision_speed_ &&
    raw_state[8] > 0.05 && raw_state[8] < 0.5 &&
    raw_state[8] + raw_state[9] > 0.05 && raw_state[8] + raw_state[9] < 0.5;
  if (trace) trace->whole_ekf_geometry = whole_ekf_geometry;
  const double delay_motion_rate = whole_ekf_geometry ? raw_state[7] : target.ekf_x()[7];
  const bool high_speed_delay = std::abs(delay_motion_rate) > decision_speed_;
  double delay_time = (high_speed_delay ? high_speed_delay_time_ : low_speed_delay_time_) +
    delay_adjustment_time_;
  if (rigid_high_speed) {
    const int sign = delay_motion_rate > 0.0 ? 1 : -1;
    delay_time = (sign > 0 ? high_speed_delay_time_ccw_ : high_speed_delay_time_cw_);
    if (trace) trace->high_speed_rotation_sign = sign;
  }

  // Low-speed friction-wheel benches can be well below the competition-speed
  // default.  Preserve the fallback only for missing/invalid telemetry so the
  // trajectory and flight-time predictor can use an explicitly calibrated value.
  if (!std::isfinite(bullet_speed) || bullet_speed < 5 || bullet_speed > 35) {
    bullet_speed = 23;
  }
  if (trace) {
    trace->delay_s = delay_time;
    trace->high_speed_delay = high_speed_delay;
    trace->delay_motion_rate_rad_s = delay_motion_rate;
    trace->speed_mps = bullet_speed;
  }

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  else {
    auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  if (trace) trace->base_prediction_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    future.time_since_epoch()).count();
  if (rigid_high_speed) {
    // Keep one physical face through every flight-time iteration. Apply the
    // selection window only at impact, not at the intermediate launch time.
    if (!whole_ekf_geometry || !target.jumped) return {false, false, 0, 0};
    if (trace) trace->final_point_selection = true;
    const auto initial_faces = target.whole_ekf_armor_xyza_list();
    for (std::size_t face = 0; face < initial_faces.size(); ++face) {
      const auto & initial = initial_faces[face];
      if (!initial.allFinite()) continue;
      tools::Trajectory first(bullet_speed, initial.head<2>().norm(), initial[2]);
      if (first.unsolvable || !std::isfinite(first.fly_time) || first.fly_time <= 0) continue;
      double previous_flight = first.fly_time;
      for (int iteration = 0; iteration < 10; ++iteration) {
        const auto impact = future + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(previous_flight));
        Eigen::Vector4d point;
        double center_yaw;
        if (const auto fixed = target.fixed_center_armor_at(static_cast<int>(face), impact)) {
          point = *fixed;
          center_yaw = std::atan2(target.ekf().x[2], target.ekf().x[0]);
        } else {
          auto candidate = target;
          candidate.predict(impact);
          point = candidate.whole_ekf_armor_xyza_list()[face];
          center_yaw = std::atan2(candidate.ekf().x[2], candidate.ekf().x[0]);
        }
        if (!point.allFinite()) break;
        tools::Trajectory trajectory(bullet_speed, point.head<2>().norm(), point[2]);
        if (trajectory.unsolvable || !std::isfinite(trajectory.fly_time) || trajectory.fly_time <= 0) break;
        if (std::abs(trajectory.fly_time - previous_flight) < 0.00001) {
          const auto & state = target.ekf().x;
          const double phase = tools::limit_rad(point[3] - center_yaw);
          const bool selected = std::abs(phase) <= comming_angle_ &&
            (state[7] > 0 ? phase < leaving_angle_ : phase > -leaving_angle_);
          if (!selected) break;
          const double yaw = std::atan2(point[1], point[0]) + yaw_offset_;
          const double pitch = -(trajectory.pitch + pitch_offset_);
          if (!std::isfinite(yaw) || !std::isfinite(pitch)) break;
          debug_aim_point = {true, point};
          if (trace) {
            trace->valid = trace->converged = true;
            trace->face_id = static_cast<int>(face);
            trace->prediction_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(impact.time_since_epoch()).count();
            trace->flight_time_used_s = previous_flight;
            trace->flight_time_s = trajectory.fly_time;
            trace->iterations = iteration + 1;
          }
          return {true, false, yaw, pitch};
        }
        previous_flight = trajectory.fly_time;
      }
    }
    return {false, false, 0, 0};
  }
  auto aim_point0 = choose_aim_point(target, whole_ekf_geometry);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  for (int iter = 0; iter < 10; ++iter) {
    // Copy only the iteration that is actually evaluated; always restart from
    // the same launch state so the physical prediction is unchanged.
    auto iteration_target = target;
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target.predict(predict_time);

    // 计算瞄准点
    auto aim_point = choose_aim_point(
      iteration_target, whole_ekf_geometry, trace ? &trace->face_id : nullptr);
    if (trace) {
      trace->prediction_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        predict_time.time_since_epoch()).count();
      trace->flight_time_used_s = prev_fly_time;
      trace->iterations = iter + 1;
    }
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 计算最终角度
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负
  if (trace) {
    trace->valid = std::isfinite(yaw) && std::isfinite(pitch);
    trace->converged = converged;
    trace->flight_time_s = current_traj.fly_time;
  }
  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  const std::list<Target> & targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now, AimPredictionTrace * trace, AimGeometry geometry)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now, trace, geometry);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target, bool whole_ekf_geometry, int * face_id)
{
  Eigen::VectorXd ekf_x = whole_ekf_geometry ? target.ekf().x : target.ekf_x();
  const bool prefer_direct_motion =
    !whole_ekf_geometry && std::abs(ekf_x[7]) <= decision_speed_ &&
    target.name != ArmorName::outpost;
  std::vector<Eigen::Vector4d> armor_xyza_list =
    whole_ekf_geometry ? target.whole_ekf_armor_xyza_list()
                       : target.armor_xyza_list(prefer_direct_motion);
  auto armor_num = armor_xyza_list.size();
  const auto select = [&](bool valid, int id) -> AimPoint {
    if (face_id) *face_id = valid ? id : -1;
    return {valid, armor_xyza_list[id]};
  };
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return select(true, 0);

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  if (prefer_direct_motion) {
    // At low rotation rate the currently observed physical plate is the most
    // reliable target. Prefer its fresh CV track instead of re-selecting among
    // several model faces at every ballistic iteration; the latter produced
    // red-box jumps and corresponding gimbal reversals near a face hand-over.
    if (target.last_id >= 0 && target.last_id < static_cast<int>(armor_num) &&
        target.armor_motion_trusted(target.last_id) &&
        std::abs(delta_angle_list[target.last_id]) <= 75.0 / 57.3) {
      lock_id_ = target.last_id;
      return select(true, target.last_id);
    }

    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return select(false, 0);
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return select(true, static_cast<int>(lock_id_));
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return select(true, id_list[0]);
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return select(true, i);
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return select(true, i);
  }

  return select(false, 0);
}

}  // namespace auto_aim

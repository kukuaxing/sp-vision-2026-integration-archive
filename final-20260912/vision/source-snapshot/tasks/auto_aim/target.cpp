#include "target.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <utility>

#include "measurement_noise.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig, const YawESKF::Config & yaw_eskf_config)
: Target(
    armor, t, radius, armor_num, std::move(P0_dig), yaw_eskf_config,
    IteratedUpdateConfig{})
{
}

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
  Eigen::VectorXd P0_dig, const YawESKF::Config & yaw_eskf_config,
  const IteratedUpdateConfig & iterated_update_config)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  armor_num_(armor_num),
  switch_count_(0),
  update_count_(0),
  is_switch_(false),
  is_converged_(false),
  yaw_eskf_(yaw_eskf_config),
  iterated_update_config_(iterated_update_config),
  t_(t)
{
  auto r = radius;
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // l: r2 - r1
  // h: z2 - z1
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
  yaw_eskf_.reset(ypr[0]);
  armor_motion_filters_[0].reset(xyz.head<3>());
}

Target::Target(double x, double vyaw, double radius, double h) : armor_num_(4)
{
  Eigen::VectorXd x0{{x, 0, 0, 0, 0, 0, 0, vyaw, radius, 0, h}};
  Eigen::VectorXd P0_dig{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
  yaw_eskf_.reset(0.0, vyaw);
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  yaw_eskf_.predict(dt);
  for (auto & filter : armor_motion_filters_) filter.predict(dt);

  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  double v1, v2;
  if (name == ArmorName::outpost) {
    v1 = 10;   // 前哨站加速度方差
    v2 = 0.1;  // 前哨站角加速度方差
  } else {
    v1 = 100;  // 加速度方差
    v2 = 64;  // 角加速度方差
  }
  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  // 前哨站转速特判
  if (this->convergened() && this->name == ArmorName::outpost && std::abs(this->ekf_.x[7]) > 2)
    this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

  ekf_.predict(F, Q, f);
}

void Target::update(const Armor & armor)
{
  // 装甲板匹配
  int id;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  // 取前3个distance最小的装甲板
  for (int i = 0; i < std::min(3, armor_num_); i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  update(armor, id);
}

void Target::update(const Armor & armor, int id)
{
  if (id < 0 || id >= armor_num_) return;

  if (id != 0) jumped = true;

  if (id != last_id) {
    is_switch_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;

  update_ypda(armor, id);
  update_yaw_eskf(armor, id);
  if (id < static_cast<int>(armor_motion_filters_.size()))
    armor_motion_filters_[id].update(armor.xyz_in_world.head<3>());
}

void Target::update_batch(
  const std::vector<std::pair<Armor, int>> & observations, int primary_armor_id)
{
  std::vector<std::pair<Armor, int>> valid;
  valid.reserve(observations.size());
  for (const auto & observation : observations) {
    if (observation.second >= 0 && observation.second < armor_num_) {
      valid.push_back(observation);
    }
  }
  if (valid.empty()) return;

  if (primary_armor_id < 0 || primary_armor_id >= armor_num_) {
    primary_armor_id = valid.front().second;
  }
  jumped = jumped || std::any_of(
    valid.begin(), valid.end(), [](const auto & observation) {
      return observation.second != 0;
    });
  is_switch_ = primary_armor_id != last_id;
  if (is_switch_) ++switch_count_;
  last_id = primary_armor_id;
  ++update_count_;

  const int observation_count = static_cast<int>(valid.size());
  Eigen::VectorXd z(4 * observation_count);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(
    4 * observation_count, 4 * observation_count);

  for (int i = 0; i < observation_count; ++i) {
    const auto & armor = valid[i].first;
    const int id = valid[i].second;
    const double center_yaw = std::atan2(
      armor.xyz_in_world[1], armor.xyz_in_world[0]);
    const double delta_angle = tools::limit_rad(
      armor.ypr_in_world[0] - center_yaw);
    const auto variances = armor_measurement_variances(
      armor.ypd_in_world[2], delta_angle, armor.confidence);
    const Eigen::Vector4d R_diag{
      variances[0], variances[1], variances[2], variances[3]};
    R.block<4, 4>(i * 4, i * 4) = R_diag.asDiagonal();

    z.segment<4>(i * 4) << armor.ypd_in_world[0], armor.ypd_in_world[1],
      armor.ypd_in_world[2], armor.ypr_in_world[0];
  }

  const auto h = [this, valid](const Eigen::VectorXd & x) {
    Eigen::VectorXd predicted(4 * static_cast<int>(valid.size()));
    for (std::size_t i = 0; i < valid.size(); ++i) {
      const int id = valid[i].second;
      const Eigen::Vector3d xyz = h_armor_xyz(x, id);
      const Eigen::Vector3d ypd = tools::xyz2ypd(xyz);
      predicted.segment<4>(static_cast<int>(i) * 4) <<
        ypd[0], ypd[1], ypd[2],
        tools::limit_rad(x[6] + id * 2.0 * CV_PI / armor_num_);
    }
    return predicted;
  };
  const auto calculate_H = [this, valid](const Eigen::VectorXd & x) {
    Eigen::MatrixXd H(4 * static_cast<int>(valid.size()), x.size());
    for (std::size_t i = 0; i < valid.size(); ++i) {
      H.block(static_cast<int>(i) * 4, 0, 4, x.size()) =
        h_jacobian(x, valid[i].second);
    }
    return H;
  };
  const auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd residual = a - b;
    for (Eigen::Index i = 0; i < residual.size(); i += 4) {
      residual[i] = tools::limit_rad(residual[i]);
      residual[i + 1] = tools::limit_rad(residual[i + 1]);
      residual[i + 3] = tools::limit_rad(residual[i + 3]);
    }
    return residual;
  };
  const auto x_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd residual = a - b;
    if (residual.size() > 6) residual[6] = tools::limit_rad(residual[6]);
    return residual;
  };
  if (iterated_update_config_.enabled) {
    ekf_.update_iterated(
      z, R, h, calculate_H, z_subtract, x_subtract,
      iterated_update_config_.max_iterations,
      iterated_update_config_.stop_threshold,
      iterated_update_config_.huber_threshold);
  } else {
    ekf_.update(z, calculate_H(ekf_.x), R, h, z_subtract);
  }

  // Every associated face measures the same underlying body phase after its
  // fixed face offset is removed.  Sequential scalar ESKF updates are cheap
  // and retain information from both visible faces at hand-over.
  for (const auto & observation : valid) {
    update_yaw_eskf(observation.first, observation.second);
    if (observation.second < static_cast<int>(armor_motion_filters_.size())) {
      armor_motion_filters_[observation.second].update(
        observation.first.xyz_in_world.head<3>());
    }
  }

  // The legacy diagnostic compares NIS against a per-four-dimensional
  // threshold.  A stacked 4N observation naturally has a larger raw NIS; keep
  // tracker reset behaviour comparable by recording NIS per observed face.
  if (observation_count > 1 && std::isfinite(ekf_.last_nis)) {
    const double normalized_nis = ekf_.last_nis / observation_count;
    ekf_.last_nis = normalized_nis;
    ekf_.data["nis"] = normalized_nis;
    constexpr double chi_square_4d_95 = 9.487729;
    ekf_.data["nis_fail"] = normalized_nis > chi_square_4d_95 ? 1.0 : 0.0;
    if (!ekf_.recent_nis_failures.empty()) {
      ekf_.recent_nis_failures.back() = normalized_nis > chi_square_4d_95 ? 1 : 0;
      const int recent_failures = std::accumulate(
        ekf_.recent_nis_failures.begin(), ekf_.recent_nis_failures.end(), 0);
      ekf_.data["recent_nis_failures"] =
        static_cast<double>(recent_failures) / ekf_.recent_nis_failures.size();
    }
  }
}

void Target::update_ypda(const Armor & armor, int id)
{
  // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  const auto variances = armor_measurement_variances(
    armor.ypd_in_world[2], delta_angle, armor.confidence);
  Eigen::VectorXd R_dig{
    {variances[0], variances[1], variances[2], variances[3]}};

  //测量过程噪声偏差的方差
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  const auto calculate_H = [this, id](const Eigen::VectorXd & x) {
    return h_jacobian(x, id);
  };
  const auto x_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    Eigen::VectorXd residual = a - b;
    if (residual.size() > 6) residual[6] = tools::limit_rad(residual[6]);
    return residual;
  };
  if (iterated_update_config_.enabled) {
    ekf_.update_iterated(
      z, R, h, calculate_H, z_subtract, x_subtract,
      iterated_update_config_.max_iterations,
      iterated_update_config_.stop_threshold,
      iterated_update_config_.huber_threshold);
  } else {
    ekf_.update(z, calculate_H(ekf_.x), R, h, z_subtract);
  }
}

void Target::update_yaw_eskf(const Armor & armor, int id)
{
  if (id < 0 || id >= armor_num_ || armor.ypr_in_world.size() < 1) return;
  const double body_yaw_measurement = tools::limit_rad(
    armor.ypr_in_world[0] - id * 2.0 * CV_PI / armor_num_);
  const double center_yaw = std::atan2(
    armor.xyz_in_world[1], armor.xyz_in_world[0]);
  const double incidence = tools::limit_rad(
    armor.ypr_in_world[0] - center_yaw);
  const auto variances = armor_measurement_variances(
    armor.ypd_in_world[2], incidence, armor.confidence);
  const double equivalent_face_period = 2.0 * CV_PI / std::max(1, armor_num_);
  yaw_eskf_.update_equivalent(
    body_yaw_measurement, variances[3], equivalent_face_period);
}

Eigen::VectorXd Target::ekf_x() const
{
  Eigen::VectorXd state = ekf_.x;
  if (yaw_eskf_.trusted() && state.size() >= 8) {
    state[6] = yaw_eskf_.angle();
    state[7] = yaw_eskf_.rate();
  }
  return state;
}

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

bool Target::yaw_eskf_trusted() const { return yaw_eskf_.trusted(); }

double Target::yaw_eskf_angle() const { return yaw_eskf_.angle(); }

double Target::yaw_eskf_rate() const { return yaw_eskf_.rate(); }

double Target::yaw_eskf_acceleration() const { return yaw_eskf_.acceleration(); }

double Target::yaw_eskf_nis() const { return yaw_eskf_.last_nis(); }

int Target::yaw_eskf_accepted_updates() const { return yaw_eskf_.accepted_updates(); }

int Target::yaw_eskf_rejected_updates() const { return yaw_eskf_.rejected_updates(); }

bool Target::armor_motion_trusted(int id) const
{
  return id >= 0 && id < armor_num_ && id < static_cast<int>(armor_motion_filters_.size()) &&
    armor_motion_filters_[id].trusted();
}

Eigen::Vector3d Target::armor_motion_velocity(int id) const
{
  if (id < 0 || id >= armor_num_ || id >= static_cast<int>(armor_motion_filters_.size()))
    return Eigen::Vector3d::Zero();
  return armor_motion_filters_[id].velocity();
}

double Target::armor_motion_nis(int id) const
{
  if (id < 0 || id >= armor_num_ || id >= static_cast<int>(armor_motion_filters_.size()))
    return std::numeric_limits<double>::quiet_NaN();
  return armor_motion_filters_[id].last_nis();
}

std::vector<Eigen::Vector4d> Target::armor_xyza_list(bool prefer_direct_motion) const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;
  Eigen::VectorXd state = ekf_x();

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(state[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(state, i);
    if (prefer_direct_motion && i < static_cast<int>(armor_motion_filters_.size()) &&
        armor_motion_filters_[i].trusted()) {
      xyz = armor_motion_filters_[i].position();
    }
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

int Target::armor_num() const { return armor_num_; }

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

bool Target::convergened()
{
  if (this->name != ArmorName::outpost && update_count_ > 3 && !this->diverged()) {
    is_converged_ = true;
  }

  //前哨站特殊判断
  if (this->name == ArmorName::outpost && update_count_ > 10 && !this->diverged()) {
    is_converged_ = true;
  }

  return is_converged_;
}

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim

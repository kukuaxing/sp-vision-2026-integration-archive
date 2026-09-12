#include "extended_kalman_filter.hpp"

#include <algorithm>
#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  const Eigen::VectorXd x_prior = x;
  const Eigen::MatrixXd P_prior = P;
  const Eigen::VectorXd innovation = z_subtract(z, h(x_prior));
  const Eigen::MatrixXd S = H * P_prior * H.transpose() + R;
  const auto S_ldlt = S.ldlt();
  const Eigen::MatrixXd K =
    P_prior * H.transpose() * S_ldlt.solve(
      Eigen::MatrixXd::Identity(S.rows(), S.cols()));

  // NIS must use the prior innovation and prior innovation covariance.  The
  // previous implementation evaluated a posterior residual with posterior P,
  // so its value did not have the chi-square distribution used by the gate.
  const double nis = innovation.dot(S_ldlt.solve(innovation));

  x = x_add(x_prior, K * innovation);

  // Stable computation of the posterior covariance (Joseph form).
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P_prior * (I - K * H).transpose() + K * R * K.transpose();

  /// 卡方检验
  const Eigen::VectorXd residual = z_subtract(z, h(x));
  const Eigen::VectorXd state_delta = x - x_prior;
  const double nees = state_delta.dot(P.ldlt().solve(state_delta));

  // 95% chi-square threshold for each four-dimensional armor observation.
  // Stacked batch updates are normalized per armor below and in Target.
  constexpr double chi_square_4d_95 = 9.487729;
  const int observation_blocks = std::max<int>(1, z.size() / 4);
  const double nis_threshold = chi_square_4d_95 * observation_blocks;
  // NEES is diagnostic only; 19.675 is chi-square(11) at 95%.
  constexpr double nees_threshold = 19.675138;

  data["nis_fail"] = nis > nis_threshold ? 1.0 : 0.0;
  data["nees_fail"] = nees > nees_threshold ? 1.0 : 0.0;
  if (nis > nis_threshold) ++nis_count_;
  if (nees > nees_threshold) ++nees_count_;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update_iterated(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::MatrixXd(const Eigen::VectorXd &)> calculate_H,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_subtract,
  int max_iterations, double stop_threshold, double huber_threshold)
{
  const Eigen::VectorXd x_prior = x;
  const Eigen::MatrixXd P_prior = P;
  const Eigen::MatrixXd H_prior = calculate_H(x_prior);
  const Eigen::VectorXd innovation = z_subtract(z, h(x_prior));
  const Eigen::MatrixXd S_prior = H_prior * P_prior * H_prior.transpose() + R;
  const auto S_prior_ldlt = S_prior.ldlt();
  const double nis = innovation.dot(S_prior_ldlt.solve(innovation));

  Eigen::VectorXd x_current = x_prior;
  Eigen::MatrixXd H_final = H_prior;
  Eigen::MatrixXd R_final = R;
  Eigen::MatrixXd K_final = Eigen::MatrixXd::Zero(x.size(), z.size());
  max_iterations = std::clamp(max_iterations, 1, 10);
  stop_threshold = std::max(1e-9, stop_threshold);
  huber_threshold = std::max(0.5, huber_threshold);

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    H_final = calculate_H(x_current);
    const Eigen::VectorXd residual = z_subtract(z, h(x_current));
    R_final = R;
    for (Eigen::Index i = 0; i < residual.size(); ++i) {
      const double sigma = std::sqrt(std::max(1e-12, R(i, i)));
      const double normalized = std::abs(residual[i]) / sigma;
      if (normalized > huber_threshold) {
        // IRLS Huber weight w=k/|r| is equivalent to R/w.
        R_final(i, i) *= normalized / huber_threshold;
      }
    }

    const Eigen::MatrixXd S = H_final * P_prior * H_final.transpose() + R_final;
    const auto S_ldlt = S.ldlt();
    K_final = P_prior * H_final.transpose() * S_ldlt.solve(
      Eigen::MatrixXd::Identity(S.rows(), S.cols()));
    const Eigen::VectorXd prior_delta = x_subtract(x_prior, x_current);
    // Iterated EKF Gauss-Newton correction.  prior_delta is x_prior-x_i,
    // therefore the relinearization term enters with a minus sign.
    const Eigen::VectorXd correction =
      K_final * (residual - H_final * prior_delta);
    const Eigen::VectorXd x_next = x_add(x_prior, correction);
    const Eigen::VectorXd step = x_subtract(x_next, x_current);
    x_current = x_next;
    if (step.norm() < stop_threshold) break;
  }

  x = x_current;
  const Eigen::MatrixXd IKH = I - K_final * H_final;
  P = IKH * P_prior * IKH.transpose() + K_final * R_final * K_final.transpose();
  P = 0.5 * (P + P.transpose());

  const Eigen::VectorXd residual = z_subtract(z, h(x));
  const Eigen::VectorXd state_delta = x_subtract(x, x_prior);
  const double nees = state_delta.dot(P.ldlt().solve(state_delta));
  constexpr double chi_square_4d_95 = 9.487729;
  const int observation_blocks = std::max<int>(1, z.size() / 4);
  const double nis_threshold = chi_square_4d_95 * observation_blocks;
  constexpr double nees_threshold = 19.675138;

  data["nis_fail"] = nis > nis_threshold ? 1.0 : 0.0;
  data["nees_fail"] = nees > nees_threshold ? 1.0 : 0.0;
  if (nis > nis_threshold) ++nis_count_;
  if (nees > nees_threshold) ++nees_count_;
  ++total_count_;
  last_nis = nis;
  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);
  if (recent_nis_failures.size() > window_size) recent_nis_failures.pop_front();
  const int recent_failures =
    std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  data["residual_yaw"] = residual.size() > 0 ? residual[0] : 0.0;
  data["residual_pitch"] = residual.size() > 1 ? residual[1] : 0.0;
  data["residual_distance"] = residual.size() > 2 ? residual[2] : 0.0;
  data["residual_angle"] = residual.size() > 3 ? residual[3] : 0.0;
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = static_cast<double>(recent_failures) /
    std::max<std::size_t>(1, recent_nis_failures.size());
  data["iekf_iterations_enabled"] = 1.0;
  return x;
}

}  // namespace tools

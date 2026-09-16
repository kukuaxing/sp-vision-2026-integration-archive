#include <Eigen/Dense>

#include <cassert>
#include <iostream>

#include "tools/extended_kalman_filter.hpp"

int main()
{
  const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
  const Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(4, 4);
  const Eigen::MatrixXd H = Eigen::MatrixXd::Identity(4, 4);
  const Eigen::MatrixXd R = 0.1 * Eigen::MatrixXd::Identity(4, 4);

  tools::ExtendedKalmanFilter nominal(x0, P0);
  nominal.update(Eigen::VectorXd::Zero(4), H, R);
  assert(nominal.data.at("nis") < 1e-12);
  assert(nominal.data.at("nis_fail") == 0.0);

  tools::ExtendedKalmanFilter outlier(x0, P0);
  outlier.update(10.0 * Eigen::VectorXd::Ones(4), H, R);
  assert(outlier.data.at("nis") > 9.487729);
  assert(outlier.data.at("nis_fail") == 1.0);

  std::cout << "ekf_consistency_test: PASS\n";
  return 0;
}

#include <cassert>
#include <cmath>
#include <iostream>

#include "tools/extended_kalman_filter.hpp"

int main()
{
  Eigen::VectorXd x0(1);
  x0 << 1.2;
  Eigen::MatrixXd P0(1, 1);
  P0 << 1.0;
  tools::ExtendedKalmanFilter filter(x0, P0);

  Eigen::VectorXd z(1);
  z << 9.0;
  Eigen::MatrixXd R(1, 1);
  R << 0.01;
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd y(1);
    y << x[0] * x[0];
    return y;
  };
  auto H = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd jacobian(1, 1);
    jacobian << 2.0 * x[0];
    return jacobian;
  };
  const auto subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) {
    return a - b;
  };
  filter.update_iterated(z, R, h, H, subtract, subtract, 8, 1e-8, 100.0);
  std::cout << "iterated estimate=" << filter.x[0] << std::endl;
  assert(std::abs(filter.x[0] - 3.0) < 0.05);
  assert(std::isfinite(filter.P(0, 0)) && filter.P(0, 0) > 0.0);

  // A large outlier must remain finite and be downweighted by the robust IRLS
  // path rather than making the covariance non-positive.
  z << 100.0;
  filter.update_iterated(z, R, h, H, subtract, subtract, 4, 1e-6, 2.5);
  assert(filter.x.allFinite());
  assert(filter.P.allFinite() && filter.P(0, 0) > 0.0);
  std::cout << "iterated_ekf_test: PASS x=" << filter.x[0]
            << " P=" << filter.P(0, 0) << std::endl;
  return 0;
}

#include "tools/math_tools.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
int main() {
  const double pi = std::acos(-1.);
  assert(tools::limit_rad(-pi) == pi);
  assert(tools::limit_rad(pi) == pi);
  assert(std::isnan(tools::limit_rad(std::numeric_limits<double>::infinity())));
  assert(std::isnan(tools::limit_rad(-std::numeric_limits<double>::infinity())));
  assert(std::isnan(tools::limit_rad(std::numeric_limits<double>::quiet_NaN())));
  assert(std::isfinite(tools::limit_rad(1e308)));
  for (int i = -1000; i <= 1000; ++i) {
    const double input = i * 100.1234;
    const double output = tools::limit_rad(input);
    assert(output > -pi && output <= pi);
    assert(std::abs(std::sin(input) - std::sin(output)) < 1e-10);
    assert(std::abs(std::cos(input) - std::cos(output)) < 1e-10);
  }
  std::cout << "math_tools_test: PASS; angle range/sign and non-finite termination\n";
}

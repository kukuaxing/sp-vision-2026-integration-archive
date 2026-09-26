#include "tools/trajectory.hpp"
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
int main() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (auto values : {std::array<double,3>{0., 3., 0.}, {-1.,3.,0.}, {20.,0.,0.},
                     {20.,-1.,0.}, {nan,3.,0.}, {20.,nan,0.}, {20.,3.,nan}, {5.,100.,10.}}) {
    tools::Trajectory t(values[0], values[1], values[2]);
    assert(t.unsolvable);
    assert(t.fly_time == 0 && t.pitch == 0);
  }
  for (double d : {.001, 1., 4., 8.}) {
    for (double h : {-.5, 0., .5}) {
      tools::Trajectory t(20., d, h);
      assert(!t.unsolvable && std::isfinite(t.fly_time) && t.fly_time > 0);
      const double x = 20*std::cos(t.pitch)*t.fly_time;
      const double z = 20*std::sin(t.pitch)*t.fly_time - .5*9.7833*t.fly_time*t.fly_time;
      assert(std::abs(x-d)<1e-9 && std::abs(z-h)<1e-9);
    }
  }
  std::cout << "trajectory_test: PASS; invalid inputs and ballistic round-trip\n";
}

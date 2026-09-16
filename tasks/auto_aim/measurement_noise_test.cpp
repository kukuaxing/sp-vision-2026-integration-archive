#include "measurement_noise.hpp"

#include <cassert>
#include <iostream>

int main()
{
  const auto near = auto_aim::armor_measurement_variances(1.0, 0.0, 1.0);
  const auto far = auto_aim::armor_measurement_variances(5.0, 0.0, 1.0);
  const auto oblique = auto_aim::armor_measurement_variances(5.0, 1.0, 1.0);
  const auto uncertain = auto_aim::armor_measurement_variances(5.0, 0.0, 0.25);
  assert(far[2] > near[2]);
  assert(oblique[3] > far[3]);
  assert(uncertain[0] > far[0]);
  assert(uncertain[2] > far[2]);
  std::cout << "measurement_noise_test: PASS\n";
  return 0;
}

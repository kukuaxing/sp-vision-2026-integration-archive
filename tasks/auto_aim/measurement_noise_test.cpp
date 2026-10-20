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
  // A 7 m armor only eight pixels high cannot have the same depth/normal
  // precision as a large image with the same range and detector score.
  const auto tiny = auto_aim::armor_measurement_variances(7.0, 0.0, 1.0, 8.0, 19.0, 1.0, 1.0);
  const auto large = auto_aim::armor_measurement_variances(7.0, 0.0, 1.0, 32.0, 76.0, 1.0, 1.0);
  const auto bad_fit = auto_aim::armor_measurement_variances(7.0, 0.0, 1.0, 8.0, 19.0, 2.0, 0.5);
  assert(std::sqrt(tiny[2]) >= 0.7); // pixel scale alone leaves a substantial depth interval
  assert(std::sqrt(tiny[3]) > 20.0 * M_PI / 180.0);
  assert(tiny[2] > large[2] && tiny[3] > large[3]);
  assert(bad_fit[2] > tiny[2] && bad_fit[3] > tiny[3]);
  std::cout << "measurement_noise_test: PASS\n";
  return 0;
}

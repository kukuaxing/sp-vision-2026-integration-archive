#include "robust_range_filter.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  auto_aim::RobustRangeFilter filter;
  double t = 1.0;
  auto result = filter.update(t, 4.70, 4.72);
  assert(result.valid && result.used_model);
  assert(std::abs(result.range_m - 4.701) < 1e-6);

  // Oblique-PnP depth spikes must contribute at most 5% of a bounded 0.40 m
  // residual, rather than moving the ballistic range by metres.
  for (int i = 0; i < 30; ++i) {
    t += 0.018;
    const double raw = i % 2 == 0 ? 3.20 : 5.95;
    result = filter.update(t, 4.70, raw);
    assert(result.valid && result.raw_limited);
    assert(std::abs(result.range_m - 4.70) < 0.025);
  }

  // A physically moving model is followed smoothly without accepting the raw
  // depth oscillation.
  for (int i = 0; i < 60; ++i) {
    t += 0.018;
    const double model = 4.70 + 0.018 * static_cast<double>(i);
    result = filter.update(t, model, i % 2 == 0 ? 3.10 : 6.10);
  }
  assert(result.range_m > 5.55 && result.range_m < 5.80);

  // A different target after a long gap is allowed to reset immediately.
  result = filter.update(t + 1.0, 8.0, 8.2);
  assert(std::abs(result.range_m - 8.01) < 1e-6);

  std::cout << "robust_range_filter_test: PASS\n";
  return 0;
}

#include "shot_delay_adapter.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  auto_aim::ShotDelayAdapter::Config config;
  config.enabled = true;
  config.nominal_delay_s = 0.085;
  config.min_delay_s = 0.065;
  config.max_delay_s = 0.105;
  config.min_samples = 5;
  config.window_size = 7;
  config.max_step_s = 0.002;
  auto_aim::ShotDelayAdapter adapter(config);

  // Warm-up learns the receive-thread baseline without changing the proven
  // 85 ms total calibration.
  for (double sample : {0.019, 0.020, 0.021, 0.020, 0.020}) {
    const auto result = adapter.update(sample);
    assert(result.accepted);
  }
  assert(adapter.ready());
  assert(std::abs(adapter.delay_s() - 0.085) < 1e-9);
  assert(adapter.delay_min_s() > 0.065);
  assert(adapter.delay_max_s() < 0.105);

  // A sustained 10 ms feeder slowdown is followed gradually, with no single
  // shot able to move the prediction by more than the configured 2 ms step.
  double previous_delay_s = adapter.delay_s();
  for (int delay_ms = 21; delay_ms <= 30; ++delay_ms) {
    for (int repeat = 0; repeat < 2; ++repeat) {
      const auto result = adapter.update(static_cast<double>(delay_ms) / 1000.0);
      if (result.accepted) {
        assert(std::abs(adapter.delay_s() - previous_delay_s) <= 0.0020001);
        previous_delay_s = adapter.delay_s();
      }
    }
  }
  assert(adapter.delay_s() > 0.089);
  assert(adapter.delay_s() <= 0.0951);

  // A single timeout-like sample is rejected and cannot perturb the estimate.
  const double before_outlier_s = adapter.delay_s();
  const auto outlier = adapter.update(0.090);
  assert(!outlier.accepted && outlier.rejected_outlier);
  assert(std::abs(adapter.delay_s() - before_outlier_s) < 1e-12);

  // A persistent new cluster is accepted as a regime change after three
  // consistent outliers, then requires a fresh robust warm-up.
  adapter.reset();
  for (int i = 0; i < 5; ++i) adapter.update(0.020);
  assert(adapter.update(0.035).rejected_outlier);
  assert(adapter.update(0.035).rejected_outlier);
  const auto regime = adapter.update(0.035);
  assert(regime.accepted && regime.regime_change);
  for (int i = 0; i < 4; ++i) adapter.update(0.035);
  assert(adapter.delay_s() > 0.085);

  std::cout << "shot_delay_adapter_test: PASS\n";
  return 0;
}

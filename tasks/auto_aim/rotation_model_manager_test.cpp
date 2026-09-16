#include "rotation_model_manager.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  using Manager = auto_aim::RotationModelManager;
  Manager manager;

  auto result = manager.update(0.00, true, 2.0);
  assert(result.mode == Manager::Mode::armor);
  result = manager.update(0.20, true, 2.0);
  assert(result.mode == Manager::Mode::vehicle);
  assert(result.mode_changed);
  assert(!result.phase_reset);

  result = manager.update(0.30, true, 4.5);
  assert(result.mode == Manager::Mode::vehicle);
  result = manager.update(0.50, true, 4.5);
  assert(result.mode == Manager::Mode::high_speed_phase);
  assert(result.phase_reset);

  result = manager.update(0.51, true, 4.5, true, 4.4);
  assert(!result.phase_fire_trusted);
  result = manager.update(0.52, true, 4.5, true, 4.4);
  assert(result.rate_consistent);
  assert(result.phase_fire_trusted);

  result = manager.update(0.53, true, 4.5, true, 6.2);
  assert(!result.phase_fire_trusted);
  manager.update(0.54, true, 4.5, true, 6.2);
  result = manager.update(0.55, true, 4.5, true, 6.2);
  assert(result.phase_reset);

  manager.update(0.60, true, 2.4);
  result = manager.update(0.80, true, 2.4);
  assert(result.mode == Manager::Mode::vehicle);
  assert(result.phase_reset);

  result = manager.update(1.00, false, 0.0);
  assert(result.mode == Manager::Mode::armor);
  assert(!result.phase_fire_trusted);

  // Filtered rate plus separate enter/exit thresholds must prevent mode
  // chatter when a slow target hovers around the vehicle-model boundary.
  Manager::Config smooth_config;
  smooth_config.vehicle_enter_rad_s = 0.9;
  smooth_config.vehicle_exit_rad_s = 0.45;
  smooth_config.mode_dwell_s = 0.20;
  smooth_config.rate_filter_tau_s = 0.10;
  smooth_config.rate_disagreement_updates = 18;
  Manager smooth(smooth_config);
  smooth.update(0.00, true, 1.2);
  result = smooth.update(0.25, true, 1.2);
  assert(result.mode == Manager::Mode::vehicle);
  int changes = result.mode_changed ? 1 : 0;
  for (int i = 1; i <= 80; ++i) {
    const double noisy_rate = i % 2 == 0 ? 0.65 : 1.0;
    result = smooth.update(0.25 + 0.02 * i, true, noisy_rate);
    changes += result.mode_changed ? 1 : 0;
    assert(result.mode == Manager::Mode::vehicle);
  }
  assert(changes == 1);

  // Short tracker convergence dropouts must inhibit trust/fire without
  // destroying the selected motion model.  A sustained dropout still resets.
  Manager::Config dropout_config;
  dropout_config.mode_dwell_s = 0.10;
  dropout_config.tracker_dropout_hold_s = 0.25;
  Manager dropout(dropout_config);
  dropout.update(0.00, true, 4.5);
  result = dropout.update(0.11, true, 4.5);
  assert(result.mode == Manager::Mode::high_speed_phase);
  result = dropout.update(0.20, false, 0.0);
  assert(result.mode == Manager::Mode::high_speed_phase);
  assert(!result.tracker_trusted);
  assert(!result.phase_reset);
  result = dropout.update(0.40, false, 0.0);
  assert(result.mode == Manager::Mode::armor);
  assert(result.phase_reset);

  std::cout << "rotation_model_manager_test: PASS\n";
  return 0;
}

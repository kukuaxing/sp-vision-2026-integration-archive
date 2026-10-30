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

  // A live target can briefly fail geometry covariance checks at face
  // hand-over. Keep earned clock/mode agreement, but require currently trusted
  // geometry/rate before fire trust returns. An expired gap destroys the clock.
  Manager::Config geometry_config = dropout_config;
  geometry_config.rate_agreement_updates = 3;
  Manager geometry(geometry_config);
  geometry.update(0.00, true, 4.5);
  geometry.update(0.11, true, 4.5);
  for (int i = 0; i < 3; ++i)
    result = geometry.update(.12 + .01*i, true, 4.5, true, 4.5, true);
  assert(result.phase_fire_trusted);
  for (double t : {.20, .35, .50}) {
    result = geometry.update(t, false, 4.5, true, 4.5, true);
    assert(result.mode == Manager::Mode::high_speed_phase);
    assert(result.geometry_held);
    assert(!result.phase_reset && !result.tracker_trusted && !result.phase_fire_trusted);
  }
  result = geometry.update(.52, true, 4.5, true, 4.5, true);
  assert(result.phase_fire_trusted);
  result = geometry.update(.53, true, 4.5, true, 7.0, true);
  assert(!result.phase_fire_trusted);  // Old agreement cannot hide a new mismatch.
  for (int i = 0; i < 3; ++i) {
    result = geometry.update(.54 + .01*i, true, 4.5, true, 4.5, true);
    assert(result.phase_fire_trusted == (i == 2));
  }
  result = geometry.update(1.30, false, 4.5, true, 4.5, true);
  assert(result.phase_reset);
  assert(result.mode == Manager::Mode::armor && !result.phase_fire_trusted);

  // A missing observation keeps the original 250ms limit, even though the
  // caller supplies the longer geometry-only retention policy.
  Manager missing(geometry_config);
  missing.update(0., true, 4.5);
  missing.update(.11, true, 4.5);
  result = missing.update(.40, false, 4.5, true, 4.5, false);
  assert(result.phase_reset && result.mode == Manager::Mode::armor);

  // Absolute speeds can agree during reversal. A sign change must still
  // invalidate the old phase clock instead of borrowing its lock.
  Manager reverse(geometry_config);
  reverse.update(0., true, 4.5);
  reverse.update(.11, true, 4.5);
  for (int i = 0; i < 3; ++i)
    reverse.update(.12 + .01*i, true, 4.5, true, 4.5, true);
  result = reverse.update(.20, true, -4.5, true, 4.5, true);
  assert(result.phase_reset && !result.phase_fire_trusted);
  reverse.update(.21, true, -.2, true, 4.5, true);
  reverse.update(.22, true, +.2, true, 4.5, true);
  result = reverse.update(.23, true, +4.5, true, 4.5, true);
  assert(result.phase_reset && !result.phase_fire_trusted);

  std::cout << "rotation_model_manager_test: PASS\n";
  return 0;
}

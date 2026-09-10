#include "gyro_phase_scheduler.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{
void cross(auto_aim::GyroPhaseScheduler & scheduler, double t, double omega)
{
  // The folded phase wraps at +/-45 degrees between faces.  This large jump
  // must not be mistaken for the physical zero crossing.
  scheduler.observe(t - 0.012, -0.70, omega);
  if (omega > 0.0) {
    scheduler.observe(t - 0.004, -0.02, omega);
    scheduler.observe(t + 0.004, +0.02, omega);
  } else {
    scheduler.observe(t - 0.004, +0.02, omega);
    scheduler.observe(t + 0.004, -0.02, omega);
  }
}

void acquire(
  auto_aim::GyroPhaseScheduler & scheduler, double first_t,
  double period, double omega)
{
  for (int i = 0; i < 6; ++i) cross(scheduler, first_t + i * period, omega);
}
}  // namespace

int main()
{
  constexpr double omega = 5.0;
  const double period = (M_PI / 2.0) / omega;

  auto_aim::GyroPhaseScheduler scheduler;
  acquire(scheduler, 1.0, period, omega);
  assert(scheduler.period_valid());
  assert(scheduler.locked());
  assert(std::abs(scheduler.face_period_s() - period) < 1e-6);

  // A missed crossing is normalized to two face periods instead of doubling
  // the estimated period.
  cross(scheduler, 1.0 + 7.0 * period, omega);
  assert(std::abs(scheduler.face_period_s() - period) < 1e-6);

  const double now = 1.0 + 7.0 * period + 0.010;
  const auto safe = scheduler.solve(
    now, 0.154, 0.134, 0.174, 0.155, 0.005,
    0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(safe.valid);
  assert(safe.interval_safe);
  assert(safe.time_until_command_s >= 0.0);
  assert(safe.time_until_command_s < period);

  const auto unsafe = scheduler.solve(
    now, 0.154, 0.034, 0.254, 0.155, 0.020,
    0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(unsafe.valid);
  assert(!unsafe.interval_safe);

  const auto stale = scheduler.solve(
    now + 4.0 * period, 0.154, 0.134, 0.174, 0.155, 0.005,
    0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(!stale.valid);

  const auto invalid_radius = scheduler.solve(
    now, 0.154, 0.134, 0.174, 0.155, 0.005,
    0.0, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(!invalid_radius.valid);

  // Reverse rotation uses the opposite physical zero-crossing edge.
  auto_aim::GyroPhaseScheduler reverse_scheduler;
  acquire(reverse_scheduler, 2.0, period, -omega);
  assert(reverse_scheduler.locked());
  assert(std::abs(reverse_scheduler.face_period_s() - period) < 1e-6);

  // Paired long/short raw crossings reproduce the field-log failure mode.
  // The PLL phase anchor may move only a few milliseconds instead of following
  // the full +/-25 ms measurement jitter.
  auto_aim::GyroPhaseScheduler jitter_scheduler;
  const std::vector<double> jitter_s{0.0, 0.025, -0.025, 0.022, -0.020, 0.018, -0.018, 0.0};
  for (std::size_t i = 0; i < jitter_s.size(); ++i) {
    cross(jitter_scheduler, 4.0 + i * period + jitter_s[i], omega);
  }
  assert(jitter_scheduler.locked());
  assert(std::abs(jitter_scheduler.face_period_s() - period) < 0.010);
  const double ideal_last_crossing = 4.0 + (jitter_s.size() - 1) * period;
  const double anchor_error_s =
    jitter_scheduler.phase_anchor_time_s() - ideal_last_crossing;
  std::cout << "paired-jitter anchor error: " << anchor_error_s * 1000.0 << " ms\n";
  assert(std::abs(anchor_error_s) < 0.020);

  // Field replay regression: the measured face period is about 327 ms while
  // EKF omega implies about 320 ms.  The event median must remove that small
  // persistent frequency bias instead of accumulating a 90 ms phase error and
  // permanently dropping into the native fallback path.
  auto_aim::GyroPhaseScheduler biased_omega_scheduler;
  constexpr double field_period = 0.327;
  constexpr double ekf_period = 0.320;
  const double biased_omega = (M_PI / 2.0) / ekf_period;
  for (int i = 0; i < 120; ++i) {
    cross(biased_omega_scheduler, 8.0 + i * field_period, biased_omega);
  }
  assert(biased_omega_scheduler.locked());
  assert(std::abs(biased_omega_scheduler.face_period_s() - field_period) < 0.004);
  const double biased_anchor_error_s =
    biased_omega_scheduler.phase_anchor_time_s() - (8.0 + 119.0 * field_period);
  std::cout << "biased-omega anchor error: " << biased_anchor_error_s * 1000.0 << " ms\n";
  assert(std::abs(biased_anchor_error_s) < 0.020);
  assert(biased_omega_scheduler.phase_uncertainty_s() < 0.010);

  const auto field_solution = biased_omega_scheduler.solve(
    8.0 + 119.0 * field_period + 0.010,
    0.154, 0.134, 0.174, 0.155, 0.005,
    0.28, 0.135, 0.005, 1.0 * M_PI / 180.0);
  assert(field_solution.valid);
  assert(std::isfinite(field_solution.phase_uncertainty_rad));

  // A genuine phase discontinuity must temporarily unlock, establish a fresh
  // anchor from several consistent crossings, and then reacquire without a
  // mode reset.
  auto_aim::GyroPhaseScheduler relock_scheduler;
  acquire(relock_scheduler, 50.0, period, omega);
  bool saw_reacquire = false;
  for (int i = 6; i < 16; ++i) {
    const auto observation = [&]() {
      relock_scheduler.observe(50.0 + i * period + 0.100 - 0.012, -0.70, omega);
      relock_scheduler.observe(50.0 + i * period + 0.100 - 0.004, -0.02, omega);
      return relock_scheduler.observe(
        50.0 + i * period + 0.100 + 0.004, +0.02, omega);
    }();
    saw_reacquire = saw_reacquire || observation.reacquired;
  }
  assert(saw_reacquire);
  assert(relock_scheduler.locked());

  std::cout << "gyro_phase_scheduler_test: PASS\n";
  return 0;
}

#include "final_yaw_limiter.hpp"
#include "final_pitch_limiter.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace
{
constexpr double rad = 3.14159265358979323846 / 180.0;
using Limiter = auto_aim::FinalYawLimiter;

Limiter::TimePoint at_ms(int milliseconds)
{
  return Limiter::TimePoint{} + std::chrono::milliseconds(milliseconds);
}

void expect(const std::optional<double> & actual, double degrees)
{
  assert(actual && std::abs(*actual - degrees * rad) < 1e-10);
}
}  // namespace

int main()
{
  Limiter limiter(30.0 * rad);

  // A large target present in idle cannot become the first AUTO command. While
  // the MCU has not armed, every emitted reference follows the measured angle.
  assert(!limiter.update(170 * rad, 10 * rad, false, false, at_ms(1000)));
  expect(limiter.update(170 * rad, 10 * rad, true, false, at_ms(1020)), 10);
  expect(limiter.update(-100 * rad, 10.5 * rad, true, false, at_ms(1040)), 10.5);
  expect(limiter.update(170 * rad, 11 * rad, true, true, at_ms(1060)), 11);
  expect(limiter.update(170 * rad, 11 * rad, true, true, at_ms(1080)), 11.6);

  // A short visual hold keeps control active and preserves bounded continuity.
  expect(limiter.update(170 * rad, 11 * rad, true, true, at_ms(1100)), 12.2);
  // An older timestamp must neither move the command nor lower the time anchor.
  expect(limiter.update(170 * rad, 11 * rad, true, true, at_ms(1090)), 12.2);
  expect(limiter.update(170 * rad, 11 * rad, true, true, at_ms(1110)), 12.5);
  // A stalled camera cannot spend the whole gap as one large command step.
  expect(limiter.update(170 * rad, 11 * rad, true, true, at_ms(2110)), 14.0);

  // Revoked lower-board control re-anchors even if AUTO remains selected.
  expect(limiter.update(-100 * rad, 13 * rad, true, false, at_ms(2130)), 13);
  expect(limiter.update(-100 * rad, 14 * rad, true, true, at_ms(2150)), 14);
  expect(limiter.update(-100 * rad, 14 * rad, true, true, at_ms(2170)), 13.4);

  // Genuine visual loss and mode exit both discard the previous reference.
  assert(!limiter.update(170 * rad, 60 * rad, false, true, at_ms(2190)));
  expect(limiter.update(-100 * rad, 61 * rad, true, true, at_ms(2210)), 61);
  assert(!limiter.update(170 * rad, 62 * rad, false, false, at_ms(2230)));
  expect(limiter.update(-100 * rad, 63 * rad, true, false, at_ms(2250)), 63);

  // Re-entry also anchors correctly when no disarmed frame was observed.
  Limiter already_armed(30.0 * rad);
  expect(already_armed.update(170 * rad, 20 * rad, true, true, at_ms(1000)), 20);
  expect(already_armed.update(170 * rad, 20 * rad, true, true, at_ms(1020)), 20.6);

  // Follow the shortest path across +/-pi rather than reversing for 358 deg.
  Limiter wrapping(30.0 * rad);
  expect(wrapping.update(-179 * rad, 179 * rad, true, true, at_ms(1000)), 179);
  expect(wrapping.update(-179 * rad, 179 * rad, true, true, at_ms(1020)), 179.6);
  expect(wrapping.update(-179 * rad, 179 * rad, true, true, at_ms(1040)), -179.8);

  // Invalid references revoke output and require a fresh measured-angle entry.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  assert(!already_armed.update(nan, 21 * rad, true, true, at_ms(1040)));
  expect(already_armed.update(170 * rad, 22 * rad, true, true, at_ms(1060)), 22);
  assert(!already_armed.update(170 * rad, nan, true, true, at_ms(1080)));
  expect(already_armed.update(170 * rad, 23 * rad, true, true, at_ms(1100)), 23);

  // Inputs are absolute command-basis angles, including any IMU sign/offset.
  // The first Sentinel wire target must therefore have exactly zero raw error.
  Limiter transformed(30.0 * rad);
  const double raw_yaw = 12 * rad, sign = -1.0, offset = 23 * rad;
  const auto first = transformed.update(
    100 * rad, sign * raw_yaw + offset, true, false, at_ms(1000));
  assert(first && std::abs((*first - offset) / sign - raw_yaw) < 1e-10);

  auto_aim::FinalPitchLimiter pitch(45.0 * rad);
  assert(!pitch.update(20 * rad, -2 * rad, false, false, at_ms(1000)));
  expect(pitch.update(20 * rad, -2 * rad, true, false, at_ms(1020)), -2);
  expect(pitch.update(20 * rad, -1.5 * rad, true, false, at_ms(1040)), -1.5);
  expect(pitch.update(20 * rad, -1 * rad, true, true, at_ms(1060)), -1);
  expect(pitch.update(20 * rad, -1 * rad, true, true, at_ms(1065)), -.775);
  expect(pitch.update(20 * rad, -1 * rad, true, true, at_ms(1064)), -.775);
  expect(pitch.update(20 * rad, -1 * rad, true, true, at_ms(1070)), -.55);
  expect(pitch.update(20 * rad, -1 * rad, true, true, at_ms(2070)), 1.7); // cap stall at 50ms
  expect(pitch.update(-20 * rad, 3 * rad, true, false, at_ms(2090)), 3); // lease revoked
  expect(pitch.update(-20 * rad, 4 * rad, true, true, at_ms(2110)), 4); // armed edge
  expect(pitch.update(-20 * rad, 4 * rad, true, true, at_ms(2130)), 3.1);
  assert(!pitch.update(-20 * rad, 4 * rad, false, true, at_ms(2150)));
  expect(pitch.update(-20 * rad, 5 * rad, true, true, at_ms(2170)), 5); // visual reacquisition
  assert(!pitch.update(nan, 5 * rad, true, true, at_ms(2190)));
  expect(pitch.update(-20 * rad, 6 * rad, true, true, at_ms(2210)), 6);
  assert(!pitch.update(-20 * rad, nan, true, true, at_ms(2230)));
  expect(pitch.update(-20 * rad, 7 * rad, true, true, at_ms(2250)), 7);

  // Unlike yaw, even +/-179 degrees must follow the linear pitch direction.
  auto_aim::FinalPitchLimiter linear_pitch(45.0 * rad);
  expect(linear_pitch.update(-179 * rad, 179 * rad, true, true, at_ms(1000)), 179);
  expect(linear_pitch.update(-179 * rad, 179 * rad, true, true, at_ms(1020)), 178.1);
  auto_aim::FinalPitchLimiter transformed_pitch(45.0 * rad);
  const double raw_pitch = -12 * rad, pitch_sign = -1.0, pitch_offset = 3 * rad;
  const auto first_pitch = transformed_pitch.update(
    25 * rad, pitch_sign * raw_pitch + pitch_offset, true, false, at_ms(1000));
  assert(first_pitch && std::abs((*first_pitch - pitch_offset) / pitch_sign - raw_pitch) < 1e-10);
  auto_aim::FinalPitchLimiter zero_rate(0.0), invalid_rate(nan);
  assert(!zero_rate.update(0, 0, true, true, at_ms(1000)));
  assert(!invalid_rate.update(0, 0, true, true, at_ms(1000)));

  std::cout << "final_yaw_limiter_test: PASS yaw + linear pitch AUTO/lease entry, time, loss and raw-zero anchor\n";
  return 0;
}

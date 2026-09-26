#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  if (!std::isfinite(v0) || !std::isfinite(d) || !std::isfinite(h) || v0 <= 0 || d <= 0)
    return;
  const double a = 0.5 * g * (d / v0) * (d / v0);
  const double c = a + h;
  const double delta = d * d - 4 * a * c;
  if (!std::isfinite(a) || a <= 0 || !std::isfinite(delta) || delta < 0) return;

  // Rationalized low-arc root avoids cancellation for nearby/level targets.
  // This root also gives the shorter flight time.
  const double angle = std::atan(2 * c / (d + std::sqrt(delta)));
  const double time = d / (v0 * std::cos(angle));
  if (!std::isfinite(angle) || !std::isfinite(time) || time <= 0) return;
  pitch = angle;
  fly_time = time;
  unsolvable = false;
}
}  // namespace tools

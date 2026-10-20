#pragma once

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace io {
// USB buffer availability is later than exposure. This is an estimate until
// transport latency and the device clock have been measured independently.
inline std::chrono::steady_clock::time_point estimated_exposure_midpoint(
  std::chrono::steady_clock::time_point available,
  double exposure_us, double transport_delay_us = 0.0)
{
  if (!std::isfinite(exposure_us) || !std::isfinite(transport_delay_us) ||
      exposure_us < 0.0 || exposure_us > 1000000.0 ||
      transport_delay_us < 0.0 || transport_delay_us > 1000000.0)
    throw std::invalid_argument("Invalid camera timestamp exposure/transport delay");
  const auto delay = std::chrono::duration<double, std::micro>(
    0.5 * exposure_us + transport_delay_us);
  return available - std::chrono::duration_cast<std::chrono::steady_clock::duration>(delay);
}
}  // namespace io

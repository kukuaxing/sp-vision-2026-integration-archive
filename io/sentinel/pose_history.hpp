#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>

#include "io/sentinel/protocol.hpp"

namespace io::sentinel {

enum class PoseStatus { valid, no_history, warming_up, stale, bracket_unavailable, gap, clock_unstable };

// This is an arrival-clock estimate, never a claim of exposure/IMU hardware sync.
// time_us currently timestamps arrival of an IMU frame at the MCU. The unknown
// sensor processing delay is NOT included in the finite transport budget below.
struct PoseEstimate {
  bool valid = false;
  PoseStatus status = PoseStatus::no_history;
  bool interpolated = false;
  bool synchronized = false;
  bool sensor_latency_known = false;
  bool sample_certified = false;
  bool axes_verified = false;
  uint32_t boot_id = 0;
  uint16_t epoch = 0;
  uint16_t source_seq = 0; // Right bracket/current sample, never an invented wire sequence.
  uint64_t generation = 0;
  uint32_t source_time_us = 0;
  int64_t host_time_ns = 0;
  int64_t rx_time_ns = 0;
  // Assumed transport budget + observed RX jitter only. Sensor latency is
  // unknown; this is not a certified end-to-end error bound.
  int64_t time_uncertainty_ns = 0;
  int64_t assumed_transport_budget_ns = 30000000;
  std::array<double, 3> rpy_rad{};
  std::array<double, 3> gyro_rad_s{};
};

class PoseHistory {
public:
  static constexpr int64_t max_age_ns = 100000000;
  static constexpr int64_t max_bracket_ns = 50000000;

  void reset() { samples_.clear(); }

  void push(const Imu & imu, uint32_t boot, uint64_t generation, int64_t rx_ns) {
    if (!imu.valid() || imu.id != 1 || rx_ns <= 0) { reset(); return; }
    if (!samples_.empty() && (samples_.back().boot != boot ||
        samples_.back().imu.epoch != imu.epoch || samples_.back().generation != generation)) reset();
    int64_t source_ns = int64_t(imu.time_us) * 1000;
    if (!samples_.empty()) {
      const auto & last = samples_.back();
      if (!newer_time(imu.time_us, last.imu.time_us) || rx_ns < last.rx_ns) return;
      const uint32_t delta_us = imu.time_us - last.imu.time_us;
      // A gap cannot form an interpolation bridge. Re-learn this clock locally.
      if (delta_us > 500000 || rx_ns - last.rx_ns > 500000000) reset();
      else source_ns = last.source_ns + int64_t(delta_us) * 1000;
    }
    samples_.push_back({imu, boot, generation, rx_ns, source_ns});
    while (samples_.size() > 64) samples_.pop_front();
  }

  PoseEstimate latest(int64_t now_ns) const {
    if (samples_.empty()) return {};
    const auto mapping = clock_mapping();
    const auto & last = samples_.back();
    auto out = describe(last, mapping);
    out.host_time_ns = last.source_ns + mapping.offset_ns;
    out.valid = now_ns >= last.rx_ns && now_ns - last.rx_ns <= max_age_ns &&
      now_ns >= out.host_time_ns && now_ns - out.host_time_ns <= max_age_ns;
    out.status = out.valid ? PoseStatus::valid : PoseStatus::stale;
    return out;
  }

  PoseEstimate at(int64_t query_ns, int64_t now_ns) const {
    if (samples_.empty()) return {};
    PoseEstimate out = latest(now_ns);
    const bool latest_valid = out.valid;
    out.valid = false;
    if (samples_.size() < 3) { out.status = PoseStatus::warming_up; return out; }
    if (query_ns <= 0 || now_ns < query_ns || now_ns - query_ns > max_age_ns || !latest_valid) {
      out.status = PoseStatus::stale; return out;
    }
    const auto mapping = clock_mapping();
    // Variable delivery lag beyond this budget cannot masquerade as alignment.
    if (mapping.jitter_ns > 50000000) { out.status = PoseStatus::clock_unstable; return out; }
    out.status = PoseStatus::bracket_unavailable;
    for (std::size_t i = 1; i < samples_.size(); ++i) {
      const auto & a = samples_[i - 1];
      const auto & b = samples_[i];
      const int64_t ta = a.source_ns + mapping.offset_ns;
      const int64_t tb = b.source_ns + mapping.offset_ns;
      if (query_ns < ta || query_ns > tb) continue;
      if (tb <= ta || tb - ta > max_bracket_ns) { out.status = PoseStatus::gap; return out; }
      const double f = double(query_ns - ta) / double(tb - ta);
      out = describe(b, mapping);
      out.host_time_ns = query_ns;
      out.source_time_us = a.imu.time_us + static_cast<uint32_t>(
        double(uint32_t(b.imu.time_us - a.imu.time_us)) * f);
      out.rpy_rad = to_rpy(slerp(from_rpy(a.imu), from_rpy(b.imu), f));
      const std::array<double, 3> ga{a.imu.gx, a.imu.gy, a.imu.gz};
      const std::array<double, 3> gb{b.imu.gx, b.imu.gy, b.imu.gz};
      const bool gyro_ok = (a.imu.flags & gyro_valid) && (b.imu.flags & gyro_valid);
      for (int axis = 0; axis < 3; ++axis)
        out.gyro_rad_s[axis] = gyro_ok ? ga[axis] + f * (gb[axis] - ga[axis])
          : std::numeric_limits<double>::quiet_NaN();
      out.sample_certified = (a.imu.flags & sample_valid) && (b.imu.flags & sample_valid);
      out.axes_verified = (a.imu.flags & axes_verified) && (b.imu.flags & axes_verified);
      out.interpolated = f > 0.0 && f < 1.0;
      out.valid = true;
      out.status = PoseStatus::valid;
      return out;
    }
    return out; // No extrapolation, no substitution of the newest sample.
  }

private:
  struct Sample { Imu imu; uint32_t boot; uint64_t generation; int64_t rx_ns, source_ns; };
  struct Mapping { int64_t offset_ns = 0, jitter_ns = 0; };
  std::deque<Sample> samples_;
  Mapping clock_mapping() const {
    Mapping m;
    m.offset_ns = std::numeric_limits<int64_t>::max();
    int64_t max_offset = std::numeric_limits<int64_t>::min();
    for (const auto & s : samples_) {
      const auto offset = s.rx_ns - s.source_ns;
      m.offset_ns = std::min(m.offset_ns, offset);
      max_offset = std::max(max_offset, offset);
    }
    m.jitter_ns = max_offset - m.offset_ns;
    return m;
  }
  static PoseEstimate describe(const Sample & s, const Mapping & mapping) {
    PoseEstimate out;
    out.boot_id = s.boot; out.epoch = s.imu.epoch; out.source_seq = s.imu.seq;
    out.generation = s.generation;
    out.source_time_us = s.imu.time_us; out.rx_time_ns = s.rx_ns;
    out.time_uncertainty_ns = out.assumed_transport_budget_ns + mapping.jitter_ns;
    out.sample_certified = (s.imu.flags & sample_valid) != 0;
    out.axes_verified = (s.imu.flags & axes_verified) != 0;
    out.rpy_rad = {s.imu.roll, s.imu.pitch, s.imu.yaw};
    const double bad = std::numeric_limits<double>::quiet_NaN();
    out.gyro_rad_s = (s.imu.flags & gyro_valid)
      ? std::array<double, 3>{s.imu.gx, s.imu.gy, s.imu.gz}
      : std::array<double, 3>{bad, bad, bad};
    return out;
  }
  using Quaternion = std::array<double, 4>; // WXYZ
  static Quaternion from_rpy(const Imu & v) {
    const double cr = std::cos(v.roll / 2), sr = std::sin(v.roll / 2);
    const double cp = std::cos(v.pitch / 2), sp = std::sin(v.pitch / 2);
    const double cy = std::cos(v.yaw / 2), sy = std::sin(v.yaw / 2);
    return {cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
      cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy};
  }
  static Quaternion slerp(Quaternion a, Quaternion b, double f) {
    double dot = 0;
    for (int i = 0; i < 4; ++i) dot += a[i] * b[i];
    if (dot < 0) { for (auto & x : b) x = -x; dot = -dot; }
    double wa = 1 - f, wb = f;
    if (dot < 0.9995) {
      const double angle = std::acos(std::clamp(dot, -1.0, 1.0));
      wa = std::sin((1 - f) * angle) / std::sin(angle);
      wb = std::sin(f * angle) / std::sin(angle);
    }
    Quaternion q{}; double norm2 = 0;
    for (int i = 0; i < 4; ++i) { q[i] = wa * a[i] + wb * b[i]; norm2 += q[i] * q[i]; }
    for (auto & x : q) x /= std::sqrt(norm2);
    return q;
  }
  static std::array<double, 3> to_rpy(const Quaternion & q) {
    const auto w=q[0], x=q[1], y=q[2], z=q[3];
    return {std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
      std::asin(std::clamp(2*(w*y-z*x), -1.0, 1.0)),
      std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z))};
  }
};

} // namespace io::sentinel

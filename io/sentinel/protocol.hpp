#pragma once

// Hohokamk/Sentinel_robot b6f4697756eb8b7c61a6868e450f8d0cda2752ae.
// Wire values are little endian, degrees and degrees/second. No device I/O.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace io::sentinel {
constexpr std::size_t command_size = 40, feedback_size = 64;
constexpr double pi = 3.14159265358979323846;
constexpr double deg_to_rad = pi / 180.0;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);

inline uint16_t crc16(const uint8_t* bytes, std::size_t size) {
  uint16_t crc = 0xffff;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1) ? 0x8408 : 0);
  }
  return crc;
}
inline uint16_t u16(const uint8_t* p) { return p[0] | (uint16_t(p[1]) << 8); }
inline uint32_t u32(const uint8_t* p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
         (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
inline float f32(const uint8_t* p) {
  const uint32_t bits = u32(p);
  float value; std::memcpy(&value, &bits, 4); return value;
}
inline void put_float(uint8_t* p, float value) {
  uint32_t bits; std::memcpy(&bits, &value, 4);
  for (int i = 0; i < 4; ++i) p[i] = uint8_t(bits >> (8 * i));
}
template<std::size_t N> inline void append_crc(std::array<uint8_t, N>& frame) {
  const auto crc = crc16(frame.data(), N - 2);
  frame[N - 2] = uint8_t(crc); frame[N - 1] = uint8_t(crc >> 8);
}

struct Imu {
  // Preserve firmware field names. Its HI91 parser exchanges roll/pitch and gx/gy.
  float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
  float gx_dps = 0, gy_dps = 0, gz_dps = 0;
  uint32_t time_ms = 0;
  uint8_t id = 0;
  bool valid = false;
};
struct Feedback {
  uint8_t detect_color = 0;
  bool reset_tracker = false;
  Imu gimbal, chassis;
  uint16_t rx_total = 0, rx_count = 0;
};
inline bool decode(const uint8_t* p, Feedback& out) {
  if (p[0] != 0x5a || crc16(p, 62) != u16(p + 62)) return false;
  Feedback next;
  next.detect_color = p[1] & 1; next.reset_tracker = (p[1] & 2) != 0;
  next.gimbal = {f32(p+4), f32(p+8), f32(p+12), f32(p+16), f32(p+20),
                 f32(p+24), u32(p+28), p[32], p[33] == 1};
  next.chassis = {f32(p+36), f32(p+40), f32(p+44), 0, 0, 0,
                  u32(p+48), p[52], p[53] == 1};
  next.rx_total = u16(p+56); next.rx_count = u16(p+58);
  // Invalid IMU blocks may contain garbage; keep link and per-sensor validity separate.
  const auto finite_imu = [](const Imu& imu) {
    return std::isfinite(imu.roll_deg) && std::isfinite(imu.pitch_deg) &&
           std::isfinite(imu.yaw_deg) && std::isfinite(imu.gx_dps) &&
           std::isfinite(imu.gy_dps) && std::isfinite(imu.gz_dps);
  };
  if (p[33] > 1 || p[53] > 1 || (next.gimbal.valid && !finite_imu(next.gimbal)) ||
      (next.chassis.valid && !finite_imu(next.chassis))) return false;
  out = next; return true;
}

// Absolute Command is converted by the adapter to the incremental errors consumed
// by RC::OnPC. Current pitch/yaw go in offsets 1/5, NOT the target angles.
struct Control {
  float pitch_deg = 0, yaw_deg = 0, yaw_error_deg = 0, pitch_error_deg = 0;
  float distance_m = 0, lateral_velocity = 0, speed_x_mps = 0, speed_y_mps = 0;
  bool fire = false;
};
inline std::array<uint8_t, command_size> encode(const Control& command) {
  std::array<uint8_t, command_size> frame{};
  frame[0] = 0xa5;
  put_float(frame.data()+1, command.pitch_deg); put_float(frame.data()+5, command.yaw_deg);
  put_float(frame.data()+9, command.yaw_error_deg); put_float(frame.data()+13, command.pitch_error_deg);
  put_float(frame.data()+17, command.distance_m); frame[21] = command.fire ? 1 : 0;
  put_float(frame.data()+25, command.lateral_velocity);
  put_float(frame.data()+30, command.speed_x_mps); put_float(frame.data()+34, command.speed_y_mps);
  append_crc(frame); return frame;
}

class Decoder {
public:
  uint64_t headers = 0, crc_errors = 0, field_errors = 0;
  template<class Callback> void feed(const uint8_t* data, std::size_t size, Callback accept) {
    // Incremental processing bounds retained memory even for a huge noise chunk.
    for (std::size_t i = 0; i < size; ++i) {
      pending_.push_back(data[i]);
      while (!pending_.empty() && pending_.front() != 0x5a) pending_.erase(pending_.begin());
      if (pending_.size() < feedback_size) continue;
      ++headers;
      Feedback packet;
      if (crc16(pending_.data(), 62) != u16(pending_.data()+62)) {
        ++crc_errors; pending_.erase(pending_.begin());
      } else if (!decode(pending_.data(), packet)) {
        ++field_errors; pending_.erase(pending_.begin());
      } else {
        accept(packet); pending_.clear();
      }
    }
  }
private:
  std::vector<uint8_t> pending_;
};
} // namespace io::sentinel

#pragma once
// Portable RMV 1.0 framing + telemetry/time adapter. No device I/O or actuation.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace io::rmv {
using Bytes = std::vector<std::uint8_t>;
using Quaternion = std::array<double, 4>; // wxyz; transforms body to world
inline constexpr std::size_t HEADER_SIZE = 28, MAX_PAYLOAD = 256;

inline std::uint16_t crc16(const std::uint8_t* p, std::size_t n) {
  std::uint16_t crc = 0xffff;
  for (std::size_t i = 0; i < n; ++i) {
    crc ^= static_cast<std::uint16_t>(p[i]) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = static_cast<std::uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
  }
  return crc;
}
inline std::uint64_t integer(const Bytes& bytes, std::size_t offset, std::size_t width) {
  if (width > 8 || offset + width > bytes.size()) throw std::invalid_argument("short field");
  std::uint64_t result = 0;
  for (std::size_t i = 0; i < width; ++i) result |= std::uint64_t(bytes[offset + i]) << (8 * i);
  return result;
}
inline void append(Bytes& bytes, std::uint64_t value, std::size_t width) {
  for (std::size_t i = 0; i < width; ++i) bytes.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
}
inline float real(const Bytes& bytes, std::size_t offset) {
  static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
  auto bits = static_cast<std::uint32_t>(integer(bytes, offset, 4));
  float result;
  std::memcpy(&result, &bits, 4);
  return result;
}
inline bool newer(std::uint32_t a, std::uint32_t b) {
  const std::uint32_t d = a - b;
  return d != 0 && d < 0x80000000u;
}
struct Frame {
  std::uint16_t type = 0, flags = 0;
  std::uint32_t seq = 0, session = 0;
  std::uint64_t sender_time_us = 0;
  Bytes payload;
};
inline Bytes encode(const Frame& frame) {
  if (frame.payload.size() > MAX_PAYLOAD || (frame.flags & ~3u))
    throw std::invalid_argument("invalid envelope");
  Bytes bytes{'R', 'M', 'V', '1', 1, 0};
  append(bytes, frame.type, 2); append(bytes, frame.payload.size(), 2);
  append(bytes, frame.flags, 2); append(bytes, frame.seq, 4);
  append(bytes, frame.session, 4); append(bytes, frame.sender_time_us, 8);
  bytes.insert(bytes.end(), frame.payload.begin(), frame.payload.end());
  append(bytes, crc16(bytes.data(), bytes.size()), 2);
  return bytes;
}
class Decoder {
  Bytes buffer_;
  std::optional<std::uint64_t> pending_;
public:
  std::uint64_t crc_errors = 0, format_errors = 0, timeouts = 0;
  std::size_t buffered() const { return buffer_.size(); }
  std::vector<Frame> feed(const Bytes& bytes, std::uint64_t host_now_us) {
    if (pending_ && host_now_us >= *pending_ && host_now_us - *pending_ >= 50000) {
      buffer_.erase(buffer_.begin()); pending_.reset(); ++timeouts;
    }
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
    const Bytes magic{'R', 'M', 'V', '1'};
    std::vector<Frame> frames;
    while (true) {
      auto start = std::search(buffer_.begin(), buffer_.end(), magic.begin(), magic.end());
      if (start == buffer_.end()) {
        if (buffer_.size() > 3) buffer_.erase(buffer_.begin(), buffer_.end() - 3);
        pending_.reset(); break;
      }
      if (start != buffer_.begin()) { buffer_.erase(buffer_.begin(), start); pending_.reset(); }
      if (buffer_.size() < HEADER_SIZE) {
        if (!pending_) pending_ = host_now_us;
        break;
      }
      const auto length = integer(buffer_, 8, 2), flags = integer(buffer_, 10, 2);
      if (buffer_[4] != 1 || buffer_[5] != 0 || length > MAX_PAYLOAD || (flags & ~3u)) {
        ++format_errors; buffer_.erase(buffer_.begin()); pending_.reset(); continue;
      }
      const auto total = HEADER_SIZE + length + 2;
      if (buffer_.size() < total) {
        if (!pending_) pending_ = host_now_us;
        break;
      }
      if (crc16(buffer_.data(), total - 2) != integer(buffer_, total - 2, 2)) {
        ++crc_errors; buffer_.erase(buffer_.begin()); pending_.reset(); continue;
      }
      Frame frame;
      frame.type = static_cast<std::uint16_t>(integer(buffer_, 6, 2));
      frame.flags = static_cast<std::uint16_t>(flags);
      frame.seq = static_cast<std::uint32_t>(integer(buffer_, 12, 4));
      frame.session = static_cast<std::uint32_t>(integer(buffer_, 16, 4));
      frame.sender_time_us = integer(buffer_, 20, 8);
      frame.payload.assign(buffer_.begin() + HEADER_SIZE, buffer_.begin() + total - 2);
      frames.push_back(frame);
      buffer_.erase(buffer_.begin(), buffer_.begin() + total); pending_.reset();
    }
    return frames;
  }
};

inline std::optional<Quaternion> normalize(Quaternion q) {
  double norm = 0;
  for (double v : q) { if (!std::isfinite(v)) return {}; norm += v * v; }
  norm = std::sqrt(norm);
  if (norm < .95 || norm > 1.05) return {};
  for (auto& v : q) v /= norm;
  return q;
}
inline Quaternion slerp(Quaternion a, Quaternion b, double fraction) {
  double dot = 0;
  for (std::size_t i = 0; i < 4; ++i) dot += a[i] * b[i];
  if (dot < 0) { for (auto& v : b) v = -v; dot = -dot; }
  dot = std::clamp(dot, -1.0, 1.0);
  if (dot > .9995) {
    for (std::size_t i = 0; i < 4; ++i) a[i] += fraction * (b[i] - a[i]);
    return *normalize(a);
  }
  const auto angle = std::acos(dot);
  const auto left = std::sin((1 - fraction) * angle) / std::sin(angle);
  const auto right = std::sin(fraction * angle) / std::sin(angle);
  for (std::size_t i = 0; i < 4; ++i) a[i] = left * a[i] + right * b[i];
  return a;
}
struct Pose {
  std::uint64_t sample_us;
  std::uint32_t counter, epoch;
  Quaternion q;
  std::array<double,3> gyro{};
  bool gyro_valid=false;
};
class PoseHistory {
  std::deque<Pose> poses_;
public:
  void clear() { poses_.clear(); }
  std::size_t size() const { return poses_.size(); }
  bool fresh(std::uint64_t now_mcu) const {
    return !poses_.empty() && now_mcu >= poses_.back().sample_us &&
           now_mcu - poses_.back().sample_us <= 100000;
  }
  bool add(Pose pose, std::uint64_t now_mcu) {
    auto q = normalize(pose.q);
    const auto age = static_cast<double>(now_mcu) - static_cast<double>(pose.sample_us);
    if (!q || age < -5000 || age > 100000) return false;
    if (!poses_.empty()) {
      if (pose.sample_us <= poses_.back().sample_us) return false;
      if (pose.epoch != poses_.back().epoch) clear();
      else if (!newer(pose.counter, poses_.back().counter)) return false;
    }
    pose.q = *q; poses_.push_back(pose);
    while (poses_.size() > 256) poses_.pop_front();
    return true;
  }
  std::optional<Quaternion> at(std::uint64_t sample_us, std::uint64_t now_mcu) const {
    if (poses_.empty() || now_mcu < poses_.back().sample_us ||
        now_mcu - poses_.back().sample_us > 100000 || sample_us < poses_.front().sample_us ||
        sample_us > poses_.back().sample_us) return {};
    for (std::size_t i = 0; i < poses_.size(); ++i) {
      if (sample_us == poses_[i].sample_us) return poses_[i].q;
      if (i && sample_us < poses_[i].sample_us) {
        const auto gap = poses_[i].sample_us - poses_[i-1].sample_us;
        if (gap > 50000) return {};
        return slerp(poses_[i-1].q, poses_[i].q,
                     double(sample_us - poses_[i-1].sample_us) / double(gap));
      }
    }
    return {};
  }
  std::optional<std::array<double,3>> gyro_at(std::uint64_t stamp,std::uint64_t now)const{
    if(!at(stamp,now))return {};
    for(std::size_t i=0;i<poses_.size();++i){
      if(stamp==poses_[i].sample_us)return poses_[i].gyro_valid?
        std::optional<std::array<double,3>>(poses_[i].gyro):std::nullopt;
      if(i && stamp<poses_[i].sample_us){
        if(!poses_[i-1].gyro_valid || !poses_[i].gyro_valid)return {};
        const double f=double(stamp-poses_[i-1].sample_us)/double(poses_[i].sample_us-poses_[i-1].sample_us);
        auto g=poses_[i-1].gyro;
        for(unsigned k=0;k<3;++k)g[k]+=f*(poses_[i].gyro[k]-g[k]);
        return g;
      }
    }
    return {};
  }
};
class ClockSync {
  struct Sample { std::uint64_t time; double offset, uncertainty; };
  std::deque<Sample> samples_;
public:
  void clear() { samples_.clear(); }
  bool observe(std::uint64_t t1, std::uint64_t t2, std::uint64_t t3, std::uint64_t t4) {
    if (t4 < t1 || t3 < t2 || t4-t1 < t3-t2) return false;
    const auto offset = ((double(t2) - double(t1)) + (double(t3) - double(t4))) / 2;
    samples_.push_back({t4, offset, double((t4-t1) - (t3-t2)) / 2});
    while (samples_.size() > 8) samples_.pop_front();
    return true;
  }
  std::optional<double> offset(std::uint64_t now) const {
    if (samples_.size() != 8) return {};
    const Sample* best = &samples_.front();
    for (const auto& s : samples_) {
      if (now < s.time || now - s.time > 3000000) return {};
      if (s.uncertainty < best->uncertainty) best = &s;
    }
    double uncertainty = best->uncertainty;
    for (const auto& s : samples_) uncertainty = std::max(uncertainty, std::abs(s.offset - best->offset));
    return uncertainty <= 5000 ? std::optional<double>(best->offset) : std::nullopt;
  }
  std::optional<std::uint64_t> to_mcu(std::uint64_t stamp, std::uint64_t now) const {
    auto delta = offset(now);
    if (!delta || double(stamp) + *delta < 0 ||
        double(stamp) + *delta >= double(std::numeric_limits<std::uint64_t>::max())) return {};
    return static_cast<std::uint64_t>(std::round(double(stamp) + *delta));
  }
  std::optional<double> uncertainty_us(std::uint64_t now) const {
    const auto delta=offset(now);if(!delta)return {};
    const Sample* best=&samples_.front();
    for(const auto& s:samples_)if(s.uncertainty<best->uncertainty)best=&s;
    double error=best->uncertainty;
    for(const auto& s:samples_)error=std::max(error,std::abs(s.offset-*delta));
    return error;
  }
};
struct Status {
  std::uint8_t mode = 0, owner = 0;
  bool control_permitted = false, fire_permitted=false;
  std::uint32_t faults = 0, received_seq = 0, applied_seq = 0, parameter_revision = 0, geometry_revision = 0;
  std::uint16_t result = 0, constraints = 0;
};
// Call establish() only AFTER checking HELLO nonce, boot ID and capability bits.
// Own this adapter on one thread, or protect all accesses with the same mutex.
class TelemetryAdapter {
  std::uint32_t session_ = 0;
  std::uint64_t boot_ = 0, status_time_ = 0;
  std::optional<std::uint32_t> status_seq_, pose_seq_;
public:
  Decoder decoder;
  ClockSync clock;
  PoseHistory poses;
  std::optional<Status> status;
  std::optional<std::uint32_t> epoch;
  void establish(std::uint32_t session, std::uint64_t boot) {
    if (!session) throw std::invalid_argument("zero session");
    session_ = session; boot_ = boot; status.reset(); epoch.reset();
    status_seq_.reset(); pose_seq_.reset(); clock.clear(); poses.clear(); decoder = Decoder{};
  }
  bool consume(const Frame& frame, std::uint64_t host_now) {
    if (!session_ || frame.session != session_ || frame.flags) return false;
    const auto& p = frame.payload;
    if (frame.type == 0x0101) {
      if (p.size() != 32 || p[0] > 4 || p[1] > 2 || (p[2] & ~7u) || (p[3] & ~7u) ||
          (p[4] & ~7u) || p[5] > 1 || p[6] > 1 || p[7] > 1 ||
          (status_seq_ && !newer(frame.seq, *status_seq_))) return false;
      Status s;
      s.mode = p[0]; s.owner = p[1]; s.control_permitted = p[6] != 0;
      s.fire_permitted=p[7]!=0;
      s.faults = static_cast<std::uint32_t>(integer(p, 8, 4));
      s.received_seq = static_cast<std::uint32_t>(integer(p, 12, 4));
      s.applied_seq = static_cast<std::uint32_t>(integer(p, 16, 4));
      s.result = static_cast<std::uint16_t>(integer(p, 20, 2));
      s.constraints = static_cast<std::uint16_t>(integer(p, 22, 2));
      s.parameter_revision = static_cast<std::uint32_t>(integer(p, 24, 4));
      s.geometry_revision = static_cast<std::uint32_t>(integer(p, 28, 4));
      if (status && status->geometry_revision != s.geometry_revision) { poses.clear(); epoch.reset(); }
      status = s; status_time_ = host_now; status_seq_ = frame.seq; return true;
    }
    if (frame.type != 0x0102 || p.size() != 68 || p[0] != 0 || p[1] != 0 ||
        (pose_seq_ && !newer(frame.seq, *pose_seq_))) return false;
    const auto valid = integer(p, 2, 2);
    pose_seq_ = frame.seq;
    if ((valid & ~0x3fu) || !(valid & 1) || !(valid & (8 | 32)) || ((valid & 8) && (valid & 32))) {
      poses.clear(); epoch.reset(); return false;
    }
    const auto now = clock.to_mcu(host_now, host_now);
    if (!now) return false;
    Pose pose{integer(p, 8, 8), static_cast<std::uint32_t>(integer(p, 4, 4)),
              static_cast<std::uint32_t>(integer(p, 24, 4)),
              {real(p, 28), real(p, 32), real(p, 36), real(p, 40)}};
    pose.gyro_valid=(valid&2)!=0;
    for(unsigned k=0;k<3;++k){pose.gyro[k]=real(p,44+4*k);pose.gyro_valid&=std::isfinite(pose.gyro[k]);}
    if (!poses.add(pose, *now)) return false;
    if (epoch && *epoch != pose.epoch) status.reset();
    epoch = pose.epoch; return true;
  }
  std::optional<Quaternion> camera_pose_at(std::uint64_t image_host_us, std::uint64_t host_now) const {
    const auto stamp = clock.to_mcu(image_host_us, host_now), now = clock.to_mcu(host_now, host_now);
    return stamp && now ? poses.at(*stamp, *now) : std::nullopt;
  }
  bool control_ready(std::uint64_t host_now) const {
    auto now_mcu = clock.to_mcu(host_now, host_now);
    return status && epoch && host_now >= status_time_ && host_now-status_time_ <= 100000 &&
           status->owner == 2 && status->control_permitted && !status->faults &&
           (status->mode == 2 || status->mode == 3) && now_mcu && poses.fresh(*now_mcu);
  }
  std::optional<std::uint64_t> status_age_us(std::uint64_t host_now) const {
    if(!status || host_now<status_time_)return {};
    return host_now-status_time_;
  }
  std::string control_ready_reason(std::uint64_t host_now) const {
    if(!status)return "waiting_status";
    if(!epoch)return "waiting_pose_epoch";
    auto age=status_age_us(host_now);
    if(!age || *age>100000)return "stale_status";
    if(status->owner!=2)return "local_rc_control";
    if(!status->control_permitted)return "mcu_permission_missing";
    if(status->faults)return "mcu_fault";
    if(status->mode!=2 && status->mode!=3)return "wrong_mode";
    auto now=clock.to_mcu(host_now,host_now);
    if(!now)return "waiting_clock_sync";
    if(!poses.fresh(*now))return "stale_mounted_pose";
    return "ready";
  }
};
} // namespace io::rmv

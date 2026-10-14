#pragma once
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <Eigen/Geometry>
#include "io/rmv/serial.hpp"
#include "io/rmv/session.hpp"

// Calibration uses small-Y sensor coordinates S -> its navigation frame.
// This is NOT the installed camera-support G -> W pose used for aiming.
class CalibrationRmvState {
 public:
  struct Sample {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    uint64_t count = 0, received_host_us = 0, sample_host_us = 0, stable_since_host_us = 0;
    uint64_t sample_mcu_us = 0, device_us = 0, boot_id = 0;
    uint32_t source_counter = 0, epoch = 0, session_id = 0, geometry_revision = 0, firmware = 0;
    uint16_t flags = 0;
    bool valid = false, stationary = false;
    double gyro_deg_s = 0, gyro_rms_deg_s = 0, anchor_delta_deg = 0;
    std::string stability_reason = "Waiting for motion window";
    std::string error = "Waiting for RMV handshake and time sync";
  };
  static constexpr uint32_t required_capabilities = 1u << 10;
  static uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  static bool fresh(const Sample& s, uint64_t now = now_us()) {
    return s.valid && s.count && now >= s.received_host_us &&
      now - s.received_host_us < 100000 && now >= s.sample_host_us &&
      now - s.sample_host_us < 100000;
  }
  static bool can_save(const Sample& s, uint64_t now = now_us()) {
    return fresh(s, now) && s.stationary && s.sample_host_us >= s.stable_since_host_us &&
      s.sample_host_us - s.stable_since_host_us >= 500000;
  }
  bool begin(uint64_t nonce, uint64_t now) {
    return session_.begin(nonce, now, required_capabilities);
  }
  void ingest(const io::rmv::Bytes& bytes, uint64_t now) {
    session_.ingest(bytes, now);
    const auto frames = decoder_.feed(bytes, now);
    if (!session_.active()) {
      invalidate(session_error());
      return;
    }
    if (geometry_ && *geometry_ != session_.geometry_revision()) {
      fail("Geometry revision changed; use a new calibration dataset");
      return;
    }
    geometry_ = session_.geometry_revision();
    for (const auto& frame : frames) consume(frame, now);
  }
  void tick(uint64_t now) {
    session_.tick(now);
    decoder_.feed({}, now); // Bound partial-frame lifetime during idle reads.
    if (!session_.active()) invalidate(session_error());
    else if (latest_.valid && !fresh(latest_, now)) invalidate("No fresh, synchronized small-Y pose");
  }
  std::vector<io::rmv::Bytes> take_outgoing(uint64_t now) {
    auto frames = session_.take_outgoing(now);
    if (!session_.active() && session_.state() == io::rmv::SessionState::disconnected)
      invalidate(session_error());
    // Enforce diagnostic-only output even if the session API changes later.
    for (const auto& bytes : frames) {
      const auto type = io::rmv::integer(bytes, 6, 2);
      if (type != 1 && type != 2 && type != 3)
        throw std::logic_error("Calibration cannot send control commands");
      if (type == 3 && bytes[io::rmv::HEADER_SIZE] != 0)
        throw std::logic_error("Calibration heartbeat must remain idle");
    }
    return frames;
  }
  bool disconnected() const {
    return failed_ || session_.state() == io::rmv::SessionState::disconnected;
  }
  Sample sample() const { return latest_; }
  Sample current_sample(uint64_t now) const {
    auto result = latest_;
    if (disconnected() || !session_.telemetry().clock.to_mcu(now, now)) {
      result.valid = false; result.stationary = false;
      if (result.error.empty()) result.error = "Session or clock sync is no longer valid";
    }
    return result;
  }
  Sample for_image(uint64_t image_host_us, uint64_t now = now_us()) const {
    Sample rejected = current_sample(now);
    const auto reject = [&](const std::string& reason) {
      auto result = rejected; result.valid = false; result.error = reason; return result;
    };
    if (!rejected.valid) return reject(rejected.error);
    if (!fresh(rejected, now)) return reject("Latest pose is older than 100 ms");
    if (image_host_us > now) return reject("Image timestamp is in the future");
    rejected.valid = false;
    const Sample* closest = nullptr;
    uint64_t distance = 50001;
    for (const auto& pose : history_) {
      const auto delta = pose.sample_host_us > image_host_us ?
        pose.sample_host_us - image_host_us : image_host_us - pose.sample_host_us;
      if (pose.valid && delta < distance) {
        closest = &pose; distance = delta;
      }
    }
    if (!closest) return reject("No pose within 50 ms of image timestamp");
    rejected = *closest;
    if (!closest->stationary) return reject(closest->stability_reason);
    if (image_host_us < closest->stable_since_host_us ||
        image_host_us - closest->stable_since_host_us < 500000 ||
        !can_save(*closest, closest->received_host_us))
      return reject("Image has not completed the 500 ms stable interval");
    auto result = *closest;
    result.error.clear();
    return result;
  }
  void fail(const std::string& message) { failed_ = true; invalidate(message); }

 private:
  std::string session_error() const {
    using io::rmv::Completion;
    const auto& result = session_.last_result();
    if (!result) return "Waiting for RMV HELLO reply";
    const auto request = result->type == 1 ? "HELLO" :
                         result->type == 2 ? "TIME_SYNC" : "request";
    const auto prefix = std::string("RMV ") + request;
    switch (result->completion) {
      case Completion::rejected: {
        static const char* names[] = {"OK", "UNSUPPORTED", "BAD_VERSION", "BAD_LENGTH",
          "BAD_VALUE", "BAD_SESSION", "WRONG_MODE", "NOT_READY", "STALE", "OUT_OF_RANGE",
          "BUSY", "CONFLICT", "HARDWARE_FAULT", "RESOURCE_LIMIT"};
        const auto name = result->result < 14 ? names[result->result] : "UNKNOWN";
        return prefix + " rejected: " + name + " (" + std::to_string(result->result) +
          "), detail=" + std::to_string(result->detail);
      }
      case Completion::timeout: return prefix + " timed out";
      case Completion::protocol_error: return prefix + " protocol error";
      case Completion::disconnected: return "RMV link disconnected or timed out";
      case Completion::accepted: return "RMV session is not active";
    }
    return "RMV session is not active";
  }
  void invalidate(const std::string& message) {
    latest_.valid = false; latest_.stationary = false;
    latest_.error = message; history_.clear(); gyro_window_.clear();
  }
  void consume(const io::rmv::Frame& frame, uint64_t now) {
    using namespace io::rmv;
    if (failed_ || frame.type != 0x0102 || frame.flags ||
        frame.session != session_.session_id()) return;
    const auto& p = frame.payload;
    // A different sensor must never substitute for small-Y, even if it is valid.
    if (p.empty() || p[0] != 1) return;
    if (p.size() != 68 || p[1] != 1) { invalidate("Invalid small-Y pose layout/frame"); return; }
    if (sequence_ && !newer(frame.seq, *sequence_)) return;
    sequence_ = frame.seq;
    const auto flags = static_cast<uint16_t>(integer(p, 2, 2));
    if ((flags & ~0x3fu) || (flags & 3) != 3 ||
        !(flags & (8 | 32)) || ((flags & 8) && (flags & 32)) ||
        (!(flags & 16) && integer(p, 16, 8))) {
      invalidate("Quaternion, gyro or sample-time flags are invalid"); return;
    }
    const auto q = normalize({real(p, 28), real(p, 32), real(p, 36), real(p, 40)});
    double gyro2 = 0;
    for (size_t offset : {44u, 48u, 52u}) {
      const auto value = real(p, offset);
      if (!std::isfinite(value)) { invalidate("Non-finite gyro"); return; }
      gyro2 += double(value) * value;
    }
    if (!q) { invalidate("Invalid quaternion norm"); return; }
    const auto mapped_now = session_.telemetry().clock.to_mcu(now, now);
    if (!mapped_now) { invalidate("Waiting for valid RMV clock sync"); return; }
    const auto stamp = integer(p, 8, 8);
    const auto pose_age = double(*mapped_now) - double(stamp);
    const auto packet_age = double(*mapped_now) - double(frame.sender_time_us);
    if (pose_age < -5000 || pose_age >= 100000 || packet_age < -5000 || packet_age >= 100000) {
      invalidate("Pose or packet timestamp is stale/future"); return;
    }
    const auto epoch = static_cast<uint32_t>(integer(p, 24, 4));
    if (epoch_ && *epoch_ != epoch) { fail("Frame epoch changed; use a new calibration dataset"); return; }
    epoch_ = epoch;
    const auto counter = static_cast<uint32_t>(integer(p, 4, 4));
    if (counter_ && !newer(counter, *counter_)) return; // Repeats never renew freshness.
    if (last_stamp_ && stamp <= *last_stamp_) {
      invalidate("Sample timestamp did not advance"); return;
    }
    const bool continuous = latest_.valid && fresh(latest_, now) && last_stamp_ && stamp - *last_stamp_ < 100000;
    counter_ = counter; last_stamp_ = stamp;
    const Eigen::Quaterniond orientation((*q)[0], (*q)[1], (*q)[2], (*q)[3]);
    constexpr double rad = 3.141592653589793 / 180.0;
    const auto age_us = static_cast<uint64_t>(std::max(0.0, pose_age));
    const auto host_stamp = now >= age_us ? now - age_us : 0;
    // Average energy over 100 ms: isolated small sensor spikes must not restart
    // settling. Sustained motion and a single >5 deg/s event still block saving.
    if (!continuous) gyro_window_.clear();
    gyro_window_.emplace_back(stamp, gyro2);
    while (gyro_window_.size() > 1 && stamp - gyro_window_.front().first > 100000)
      gyro_window_.pop_front();
    double energy = 0;
    for (const auto& entry : gyro_window_) energy += entry.second;
    const bool window_ready = stamp - gyro_window_.front().first >= 80000;
    latest_.gyro_deg_s = std::sqrt(gyro2) / rad;
    latest_.gyro_rms_deg_s = std::sqrt(energy / gyro_window_.size()) / rad;
    latest_.anchor_delta_deg = anchor_.angularDistance(orientation) / rad;
    const bool quiet = window_ready && latest_.gyro_deg_s < 5 && latest_.gyro_rms_deg_s < 2;
    latest_.stability_reason = !window_ready ? "Waiting for 100 ms gyro window" :
      latest_.gyro_deg_s >= 5 ? "Gyro spike exceeds 5 deg/s" :
      latest_.gyro_rms_deg_s >= 2 ? "100 ms gyro RMS exceeds 2 deg/s" : "";
    const bool stable = continuous && latest_.stationary && quiet &&
      anchor_.angularDistance(orientation) < .5 * rad;
    if (!stable) { latest_.stable_since_host_us = host_stamp; anchor_ = orientation; }
    latest_.q = orientation; ++latest_.count;
    latest_.received_host_us = now; latest_.sample_host_us = host_stamp;
    latest_.sample_mcu_us = stamp; latest_.device_us = integer(p, 16, 8);
    latest_.source_counter = counter; latest_.epoch = epoch; latest_.flags = flags;
    latest_.boot_id = session_.boot_id(); latest_.session_id = session_.session_id();
    latest_.geometry_revision = session_.geometry_revision(); latest_.firmware = session_.firmware_version();
    latest_.valid = true; latest_.stationary = quiet; latest_.error.clear();
    history_.push_back(latest_);
    while (history_.size() > 256) history_.pop_front();
  }
  io::rmv::Session session_;
  // Observe raw sensor frames independently; the aiming adapter accepts G only.
  io::rmv::Decoder decoder_;
  std::optional<uint32_t> sequence_, counter_, epoch_, geometry_;
  std::optional<uint64_t> last_stamp_;
  Eigen::Quaterniond anchor_ = Eigen::Quaterniond::Identity();
  std::deque<Sample> history_;
  std::deque<std::pair<uint64_t, double>> gyro_window_;
  Sample latest_;
  bool failed_ = false;
};

class CalibrationRmvImu {
 public:
  using Sample = CalibrationRmvState::Sample;
  CalibrationRmvImu(const std::string& port, uint32_t baud) {
    std::random_device random;
    nonce_ = (uint64_t(random()) << 32) ^ random();
    if (!nonce_) nonce_ = 1;
    transport_.open(port, baud);
    try { worker_ = std::thread([this] { receive(); }); }
    catch (...) { transport_.close(); throw; }
  }
  ~CalibrationRmvImu() {
    quit_ = true;
    if (worker_.joinable()) worker_.join();
    try { transport_.close(); } catch (...) {}
  }
  Sample sample() {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.current_sample(CalibrationRmvState::now_us());
  }
  Sample for_image(uint64_t image_host_us) {
    std::lock_guard<std::mutex> lock(mutex_); return state_.for_image(image_host_us);
  }
 private:
  void receive() {
    try {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!state_.begin(nonce_, CalibrationRmvState::now_us()))
          throw std::runtime_error("Cannot begin diagnostic RMV session");
      }
      while (!quit_) {
        std::vector<io::rmv::Bytes> outgoing;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const auto now = CalibrationRmvState::now_us();
          state_.tick(now); outgoing = state_.take_outgoing(now);
          if (state_.disconnected()) break;
        }
        for (const auto& bytes : outgoing) transport_.send(bytes);
        const auto bytes = transport_.receive();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          state_.ingest(bytes, CalibrationRmvState::now_us());
          if (state_.disconnected()) break;
        }
        if (bytes.empty()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    } catch (const std::exception& error) {
      std::lock_guard<std::mutex> lock(mutex_); state_.fail(error.what());
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_); state_.fail("RMV reader failed");
    }
    // A failed session is terminal. Restart explicitly; never auto-reconnect.
  }
  io::rmv::SerialTransport transport_;
  CalibrationRmvState state_;
  uint64_t nonce_ = 0;
  std::atomic<bool> quit_{false};
  std::thread worker_;
  std::mutex mutex_;
};

#pragma once
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include <Eigen/Geometry>
#include <serial/serial.h>
#include "firmware/sentry/STM32F405/imu_packet.h"

// Receive only. The MCU forwards checked small-gimbal HI91 frames unchanged.
// Pose is sensor-body to its navigation frame, not an assumed gimbal frame.
class CalibrationImu {
 public:
  struct Sample {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    std::chrono::steady_clock::time_point received{}, stable_since{};
    uint32_t device_ms = 0;
    uint64_t count = 0;
    bool stationary = false;
  };
  CalibrationImu(const std::string& port, uint32_t baud) {
    serial_.setPort(port); serial_.setBaudrate(baud);
    auto timeout=serial::Timeout::simpleTimeout(10);
    serial_.setTimeout(timeout);
    serial_.open(); serial_.flushInput();
    worker_ = std::thread([this] { receive(); });
  }
  ~CalibrationImu() {
    quit_ = true;
    if (worker_.joinable()) worker_.join();
  }
  Sample sample() {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
  }
  static bool fresh(const Sample& s) {
    return s.count && std::chrono::steady_clock::now()-s.received < std::chrono::milliseconds(100);
  }
  static bool can_save(const Sample& s) {
    return fresh(s) && s.stationary && s.received-s.stable_since >= std::chrono::milliseconds(500);
  }
 private:
  void receive() {
    std::vector<uint8_t> pending;
    Eigen::Quaterniond anchor = Eigen::Quaterniond::Identity();
    try {
      while (!quit_) {
        uint8_t bytes[512];
        const auto n = serial_.read(bytes, sizeof(bytes));
        pending.insert(pending.end(), bytes, bytes+n);
        while (pending.size() >= 82) {
          ImuPacket decoded{};
          if (!ParseImuPacket(pending.data(), 82, decoded)) {
            pending.erase(pending.begin());
            continue;
          }
          pending.erase(pending.begin(), pending.begin()+82);
          const auto now=std::chrono::steady_clock::now();
          const Eigen::Quaterniond q(decoded.quaternion[0], decoded.quaternion[1],
                                     decoded.quaternion[2], decoded.quaternion[3]);
          double gyro2=0;
          for (auto v : decoded.gyro) gyro2+=v*v;
          std::lock_guard<std::mutex> lock(mutex_);
          if (latest_.count && decoded.sensor_time_ms==latest_.device_ms) continue;
          const bool continuous = latest_.count && now-latest_.received<std::chrono::milliseconds(100) &&
              uint32_t(decoded.sensor_time_ms-latest_.device_ms)<100;
          const bool quiet = gyro2<4.0; // Below 2 degrees/s on all axes combined.
          const bool stable = continuous && latest_.stationary && quiet &&
              anchor.angularDistance(q.normalized()) < 0.5*3.141592653589793/180.0;
          if (!stable) { latest_.stable_since=now; anchor=q.normalized(); }
          latest_.q=q.normalized(); latest_.received=now;
          latest_.device_ms=decoded.sensor_time_ms; ++latest_.count;
          latest_.stationary=quiet;
        }
      }
    } catch (...) {
      // A failed reader stops; freshness expires. Never reconnect indefinitely.
      std::lock_guard<std::mutex> lock(mutex_);
      latest_.stationary=false;
    }
  }
  serial::Serial serial_;
  std::atomic<bool> quit_{false};
  std::thread worker_;
  std::mutex mutex_;
  Sample latest_;
};

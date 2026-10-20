#ifndef IO__XUC_HPP
#define IO__XUC_HPP

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <serial/serial.h>

#include "io/command.hpp"
#include "io/sentinel/protocol.hpp"
#include "io/sentinel/pose_history.hpp"

namespace io
{
// XUC 上位机→下位机：与新版下位机 RxPacket_TJ 完全一致（14 字节）
#pragma pack(push, 1)
struct XucRxPacket
{
  uint8_t head[2];   // 'S' 'P'
  uint8_t control;   // 是否控制云台
  uint8_t shoot;     // 是否开火
  float yaw;         // 目标 yaw 角
  float pitch;       // 目标 pitch 角
  uint16_t crc16;    // CRC16（低字节在前）
};
#pragma pack(pop)
static_assert(sizeof(XucRxPacket) == 14, "XucRxPacket must be 14 bytes");
static_assert(offsetof(XucRxPacket, crc16) == 12, "XucRxPacket CRC offset must be 12");

// XUC 下位机→上位机：与新版下位机 TxPacket_TJ 完全一致（20 字节）
#pragma pack(push, 1)
struct XucTxPacket
{
  uint8_t head[2];        // 'S' 'P'
  uint8_t mode;           // 0 空闲, 1 自瞄, 2 小符, 3 大符
  uint8_t robot_id;       // 3 红方, 103 蓝方
  float bullet_speed;     // 弹速 (m/s)
  uint16_t bullet_count;  // 子弹计数
  float imu_pitch;        // IMU 俯仰角 (rad)
  float imu_yaw;          // IMU 航向角 (rad)
  uint16_t crc16;         // CRC16（低字节在前）
};
#pragma pack(pop)
static_assert(sizeof(XucTxPacket) == 20, "XucTxPacket must be 20 bytes");
static_assert(offsetof(XucTxPacket, crc16) == 18, "XucTxPacket CRC offset must be 18");

// 向下位机发送目标角；接收 mode/robot_id/弹速/弹数/IMU。
class XucSender
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr int default_sentinel_fire_interval_ms = 1000;
  struct PoseSnapshot : sentinel::PoseEstimate {
    uint8_t mode = 0;
    bool lower_yaw_armed = false;
    bool control_ready = false;
  };
  enum class SendStatus { sent, throttled, rejected, io_failed, disabled };
  struct SendResult {
    SendStatus status = SendStatus::rejected;
    bool control_sent = false;
    bool friction_sent = false;
    bool fire_sent = false;
    bool fire_expired = false;
    bool fire_rate_limited = false;
    int64_t tx_time_ns = 0;
    uint16_t seq = 0;
  };
  struct FeederFeedback
  {
    bool active = false;
    uint64_t transition_sequence = 0;
    int64_t transition_time_ns = 0;
    double rpm = 0.0;
  };

  explicit XucSender(const std::string & config_path);
  ~XucSender();

  bool enabled() const { return enabled_; }
  bool is_sentinel() const { return sentinel_; }
  int64_t fire_min_interval_ms() const { return sentinel_fire_min_interval_ns_ / 1000000; }
  // A valid wire packet is not proof of a valid/fresh gimbal IMU.
  bool feedback_link_valid() const;
  sentinel::Feedback sentinel_feedback() const;
  bool consume_tracker_reset() { return tracker_reset_.exchange(false); }
  double imu_roll() const;
  double raw_imu_pitch() const;
  std::array<double, 3> imu_angles_rad() const;
  std::array<double, 3> gimbal_gyro_rad_s() const;
  bool lower_yaw_armed() const;
  // Capture lookup never extrapolates. A bounded wait may obtain the next IMU
  // bracket; failure must cause a missed measurement, never an invented pose.
  PoseSnapshot pose_now() const;
  PoseSnapshot pose_at_capture(Clock::time_point capture,
    std::chrono::milliseconds wait = std::chrono::milliseconds(0)) const;
  // A qualified fire request bypasses normal TX throttling. A nonzero deadline
  // suppresses an expired fire while still sending an ordinary safe update.
  // sent means complete serial write, NOT an MCU ACK or an actual projectile.
  SendResult send(const Command & cmd, Clock::time_point fire_deadline = Clock::time_point{},
    uint64_t expected_generation = 0);
  SendResult stop(); // Explicit shutdown/fault cancellation; bypasses the detection-loss hold.
  bool control_ready() const;

  // 下位机回传数据（接收线程更新；rx_valid() 为 false 时这些值不可信）
  bool rx_valid() const;
  uint8_t mode() const { return mode_.load(std::memory_order_relaxed); }
  uint8_t robot_id() const { return robot_id_.load(std::memory_order_relaxed); }
  double bullet_speed() const
  {
    return static_cast<double>(bullet_speed_.load(std::memory_order_relaxed));
  }
  uint16_t bullet_count() const { return bullet_count_.load(std::memory_order_relaxed); }
  // The current lower-board diagnostic firmware places feeder RPM in the
  // bullet_speed field.  Timestamp threshold crossings in the serial receive
  // thread so command-to-feeder latency is not quantized by the camera loop.
  FeederFeedback feeder_feedback() const;
  int64_t last_shoot_tx_ns() const
  {
    return last_shoot_tx_ns_.load(std::memory_order_acquire);
  }
  double imu_pitch() const
  {
    return imu_pitch_sign_ * static_cast<double>(imu_pitch_.load(std::memory_order_relaxed)) +
           imu_pitch_offset_rad_;
  }
  double imu_yaw() const
  {
    return imu_yaw_sign_ * static_cast<double>(imu_yaw_.load(std::memory_order_relaxed)) +
           imu_yaw_offset_rad_;
  }
  // Raw lower-board yaw is the only coordinate shared by the command and IMU
  // fields.  Expose it for the upper yaw tracking loop so that actuator error is
  // never formed by subtracting two differently signed display/world angles.
  double raw_imu_yaw() const
  {
    return static_cast<double>(imu_yaw_.load(std::memory_order_relaxed));
  }
  double command_yaw_to_raw(double command_yaw_rad) const
  {
    if (sentinel_) return (command_yaw_rad - imu_yaw_offset_rad_) / imu_yaw_sign_;
    return yaw_sign_ * command_yaw_rad;
  }
  double raw_yaw_correction_to_command(double raw_correction_rad) const
  {
    if (sentinel_) return imu_yaw_sign_ * raw_correction_rad;
    return std::abs(yaw_sign_) > 1e-9 ? raw_correction_rad / yaw_sign_ : 0.0;
  }
  uint64_t rx_bytes() const { return rx_bytes_.load(std::memory_order_relaxed); }
  uint64_t header_candidates() const
  {
    return header_candidates_.load(std::memory_order_relaxed);
  }
  uint64_t crc_errors() const { return crc_errors_.load(std::memory_order_relaxed); }
  uint64_t field_errors() const { return field_errors_.load(std::memory_order_relaxed); }
  uint64_t valid_packets() const
  {
    return valid_packets_.load(std::memory_order_relaxed);
  }

private:
  void recv_loop();
  void recv_sentinel_loop();
  SendResult send_sentinel(const Command & cmd, bool stop = false,
    Clock::time_point fire_deadline = Clock::time_point{}, uint64_t expected_generation = 0);
  PoseSnapshot pose_snapshot_locked(int64_t query_ns, int64_t now_ns, bool capture) const;

  serial::Serial serial_;
  bool enabled_ = false;
  bool sentinel_ = false;
  bool sentinel_allow_friction_ = false;
  bool sentinel_require_verified_pose_ = true;
  bool sentinel_tx_failed_ = false;
  uint16_t sentinel_valid_for_ms_ = 100;
  uint64_t sentinel_output_generation_ = 0;
  int64_t sentinel_tx_interval_ns_ = 10000000; // 100 Hz; MCU queue/DecodeTask max 200 Hz.
  int64_t sentinel_last_tx_ns_ = 0;
  int64_t sentinel_fire_min_interval_ns_ = int64_t(default_sentinel_fire_interval_ms) * 1000000;
  int64_t sentinel_sensor_timeout_ns_ = 100000000;
  bool sentinel_output_active_ = false;
  uint8_t sentinel_last_sent_flags_ = 0;
  mutable std::mutex sentinel_mutex_;
  sentinel::State sentinel_state_;
  sentinel::PoseHistory sentinel_pose_history_;
  sentinel::DetectionFrictionHold sentinel_friction_hold_;
  std::atomic<bool> tracker_reset_{false};
  double yaw_sign_ = 1.0;     // 目标 yaw 符号
  double pitch_sign_ = -1.0;  // 目标 pitch 符号（与现有 CAN 发送 -command.pitch 一致）
  double imu_yaw_sign_ = 1.0;
  double imu_pitch_sign_ = 1.0;
  double imu_yaw_offset_rad_ = 0.0;
  double imu_pitch_offset_rad_ = 0.0;
  bool allow_control_ = false;
  bool allow_shoot_ = false;
  bool require_feedback_for_control_ = true;
  bool require_auto_mode_for_control_ = false;
  bool lock_pitch_for_control_ = false;
  double max_yaw_excursion_rad_ = 0.0;
  bool control_session_active_ = false;
  double control_yaw_anchor_raw_ = 0.0;
  double feeder_active_rpm_threshold_ = 500.0;

  std::thread recv_thread_;
  std::atomic<bool> recv_quit_{false};
  std::atomic<bool> rx_valid_{false};
  std::atomic<int64_t> last_rx_ns_{0};
  static constexpr int64_t RX_TIMEOUT_NS = 500000000;
  std::atomic<uint8_t> mode_{0};
  std::atomic<uint8_t> robot_id_{0};
  std::atomic<float> bullet_speed_{0.0f};
  std::atomic<uint16_t> bullet_count_{0};
  std::atomic<float> imu_pitch_{0.0f};
  std::atomic<float> imu_yaw_{0.0f};
  std::atomic<int64_t> last_shoot_tx_ns_{0};
  mutable std::mutex feeder_feedback_mutex_;
  FeederFeedback feeder_feedback_{};
  std::atomic<uint64_t> rx_bytes_{0};
  std::atomic<uint64_t> header_candidates_{0};
  std::atomic<uint64_t> crc_errors_{0};
  std::atomic<uint64_t> field_errors_{0};
  std::atomic<uint64_t> valid_packets_{0};
};

}  // namespace io

#endif  // IO__XUC_HPP

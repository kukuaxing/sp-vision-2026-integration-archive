#include "io/xuc.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <vector>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
XucSender::XucSender(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  sentinel_ = yaml["board_protocol"] && yaml["board_protocol"].as<std::string>() == "SENTINEL";

  if (yaml["xuc_enable"]) enabled_ = yaml["xuc_enable"].as<bool>();
  if (!enabled_) {
    tools::logger()->info("[XUC] disabled (xuc_enable=false)");
    return;
  }

  std::string port = "/dev/ttyUSB0";
  uint32_t baud = sentinel_ ? 921600 : 460800;
  if (yaml["xuc_serial_port"]) port = yaml["xuc_serial_port"].as<std::string>();
  if (yaml["xuc_serial_baud"]) baud = yaml["xuc_serial_baud"].as<uint32_t>();
  if (yaml["xuc_yaw_sign"]) yaw_sign_ = yaml["xuc_yaw_sign"].as<double>();
  if (yaml["xuc_pitch_sign"]) pitch_sign_ = yaml["xuc_pitch_sign"].as<double>();
  if (yaml["xuc_imu_yaw_sign"]) imu_yaw_sign_ = yaml["xuc_imu_yaw_sign"].as<double>();
  if (yaml["xuc_imu_pitch_sign"]) imu_pitch_sign_ = yaml["xuc_imu_pitch_sign"].as<double>();
  if (yaml["xuc_imu_yaw_offset_rad"]) {
    imu_yaw_offset_rad_ = yaml["xuc_imu_yaw_offset_rad"].as<double>();
  }
  if (yaml["xuc_imu_pitch_offset_rad"]) {
    imu_pitch_offset_rad_ = yaml["xuc_imu_pitch_offset_rad"].as<double>();
  }
  if (yaml["xuc_allow_control"]) allow_control_ = yaml["xuc_allow_control"].as<bool>();
  if (yaml["xuc_allow_shoot"]) allow_shoot_ = yaml["xuc_allow_shoot"].as<bool>();
  if (yaml["xuc_require_feedback_for_control"]) {
    require_feedback_for_control_ = yaml["xuc_require_feedback_for_control"].as<bool>();
  }
  if (yaml["xuc_require_auto_mode_for_control"]) {
    require_auto_mode_for_control_ = yaml["xuc_require_auto_mode_for_control"].as<bool>();
  }
  if (yaml["xuc_lock_pitch_for_control"]) {
    lock_pitch_for_control_ = yaml["xuc_lock_pitch_for_control"].as<bool>();
  }
  if (yaml["xuc_max_yaw_excursion_rad"]) {
    max_yaw_excursion_rad_ =
      std::max(0.0, yaml["xuc_max_yaw_excursion_rad"].as<double>());
  }
  if (yaml["xuc_feeder_active_rpm_threshold"]) {
    feeder_active_rpm_threshold_ = std::clamp(
      yaml["xuc_feeder_active_rpm_threshold"].as<double>(), 50.0, 3000.0);
  }
  if (sentinel_) {
    if (baud != 921600) throw std::invalid_argument("SENTINEL requires UART4 921600 baud");
    if (!std::isfinite(imu_yaw_sign_) || !std::isfinite(imu_pitch_sign_) ||
        std::abs(imu_yaw_sign_) < 1e-9 || std::abs(imu_pitch_sign_) < 1e-9 ||
        !std::isfinite(imu_yaw_offset_rad_) || !std::isfinite(imu_pitch_offset_rad_))
      throw std::invalid_argument("Invalid SENTINEL IMU transform");
    if (yaml["sentinel_swap_roll_pitch"] && yaml["sentinel_swap_roll_pitch"].as<bool>())
      throw std::invalid_argument("Sentinel V2 has correct R/P ordering; remove old axis swap");
    if (yaml["sentinel_allow_friction"]) sentinel_allow_friction_ = yaml["sentinel_allow_friction"].as<bool>();
    const int hold_ms = yaml["sentinel_friction_lost_timeout_ms"] ?
      yaml["sentinel_friction_lost_timeout_ms"].as<int>() : 5000;
    if (hold_ms < 0 || hold_ms > 60000)
      throw std::invalid_argument("Sentinel friction lost timeout must be 0..60000 ms");
    sentinel_friction_hold_.timeout_ns = int64_t(hold_ms)*1000000;
    if (yaml["sentinel_require_verified_pose"])
      sentinel_require_verified_pose_ = yaml["sentinel_require_verified_pose"].as<bool>();
    const int valid_ms = yaml["sentinel_valid_for_ms"] ? yaml["sentinel_valid_for_ms"].as<int>() : 100;
    if (valid_ms < 1 || valid_ms > 150) throw std::invalid_argument("Sentinel validity must be 1..150 ms");
    sentinel_valid_for_ms_ = static_cast<uint16_t>(valid_ms);
    const int fire_interval_ms = yaml["sentinel_fire_min_interval_ms"]
      ? yaml["sentinel_fire_min_interval_ms"].as<int>() : default_sentinel_fire_interval_ms;
    // Exceed the maximum 150 ms command lease, so a sustained host request
    // cannot keep the MCU's fire level high through its 300 ms repeat arm time.
    if (fire_interval_ms < 200 || fire_interval_ms > 5000)
      throw std::invalid_argument("Sentinel fire interval must be 200..5000 ms");
    sentinel_fire_min_interval_ns_ = int64_t(fire_interval_ms) * 1000000;
    const double tx_hz = yaml["sentinel_tx_hz"] ? yaml["sentinel_tx_hz"].as<double>() : 100.0;
    if (!std::isfinite(tx_hz) || tx_hz < 10 || tx_hz > 100)
      throw std::invalid_argument("SENTINEL tx_hz must be 10..100 (MCU max 200 Hz)");
    sentinel_tx_interval_ns_ = static_cast<int64_t>(1e9 / tx_hz);
    require_feedback_for_control_ = true;
    require_auto_mode_for_control_ = true;
  }

  try {
    serial_.setPort(port);
    serial_.setBaudrate(baud);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
    serial_.setTimeout(timeout);
    serial_.open();
    if (sentinel_) serial_.flushInput(); // Drop queued startup bytes; State also guards stale-sequence recovery.
    // 启动接收线程：解析新版 TxPacket_TJ
    recv_thread_ = std::thread([this] {
      if (sentinel_) recv_sentinel_loop(); else recv_loop();
    });
    if (sentinel_) {
      tools::logger()->info("[SENTINEL] {} @ {} baud: V2 A6/6A RX=46+34+26 TX=38; control={} friction={} shoot={} verified_pose_required={}",
        port, baud, allow_control_, sentinel_allow_friction_, allow_shoot_, sentinel_require_verified_pose_);
      tools::logger()->info("[SENTINEL] friction follows armor detections; lost hold={} ms; faults/exit cancel immediately",
        sentinel_friction_hold_.timeout_ns/1000000);
      tools::logger()->info("[SENTINEL] fire requests separated by at least {} ms; cooldown sends fire=0; no queued requests",
        sentinel_fire_min_interval_ns_/1000000);
      return;
    }
    tools::logger()->info(
      "[XUC] serial opened {} @ {} baud, angles=rad, yaw_sign={}, pitch_sign={}, "
      "imu_yaw_sign={}, imu_pitch_sign={}, allow_control={}, allow_shoot={}, "
      "require_feedback={}, require_auto_mode={}, max_yaw_excursion_deg={:.2f}, "
      "lock_pitch={}",
      port, baud, yaw_sign_, pitch_sign_, imu_yaw_sign_, imu_pitch_sign_, allow_control_,
      allow_shoot_, require_feedback_for_control_, require_auto_mode_for_control_,
      max_yaw_excursion_rad_ * 180.0 / M_PI, lock_pitch_for_control_);
    if (!allow_control_) {
      tools::logger()->warn("[XUC][SAFE] control output is forced to zero");
    }
    if (!allow_shoot_) {
      tools::logger()->warn("[XUC][SAFE] shoot output is forced to zero");
    }
  } catch (const serial::IOException & e) {
    enabled_ = false;
    tools::logger()->warn("[XUC] failed to open serial port {}: {}", port, e.what());
  }
}

XucSender::~XucSender()
{
  if (sentinel_ && sentinel_output_active_ && serial_.isOpen()) {
    // Explicit V2 cancellation bypasses ordinary TX throttling.
    send_sentinel(Command{}, true);
  }
  recv_quit_.store(true, std::memory_order_relaxed);
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

namespace {
int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
bool XucSender::rx_valid() const
{
  if (!sentinel_) return feedback_link_valid();
  if (!feedback_link_valid()) return false;
  std::lock_guard<std::mutex> lock(sentinel_mutex_);
  return sentinel_state_.pose_valid(steady_ns());
}
bool XucSender::control_ready() const
{
  if (!allow_control_) return false;
  if (sentinel_) {
    std::lock_guard<std::mutex> lock(sentinel_mutex_);
    return rx_valid_.load(std::memory_order_relaxed) && !sentinel_tx_failed_ &&
      sentinel_state_.reference_ready(steady_ns(),sentinel_require_verified_pose_);
  }
  return (!require_feedback_for_control_ || rx_valid()) &&
    (!require_auto_mode_for_control_ || mode() == 1);
}
bool XucSender::lower_yaw_armed() const
{
  if (!sentinel_) return rx_valid() && mode() == 1 && (robot_id() & 0x40);
  std::lock_guard<std::mutex> lock(sentinel_mutex_);
  return sentinel_state_.status_valid(steady_ns()) && (sentinel_state_.feedback.link.state & 0x04);
}

bool XucSender::feedback_link_valid() const
{
  if (!rx_valid_.load(std::memory_order_relaxed)) return false;

  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto last_ns = last_rx_ns_.load(std::memory_order_relaxed);

  return last_ns > 0 && now_ns - last_ns <=
    (sentinel_ ? sentinel_sensor_timeout_ns_ : RX_TIMEOUT_NS);
}

XucSender::FeederFeedback XucSender::feeder_feedback() const
{
  std::lock_guard<std::mutex> lock(feeder_feedback_mutex_);
  return feeder_feedback_;
}

sentinel::Feedback XucSender::sentinel_feedback() const
{
  std::lock_guard<std::mutex> lock(sentinel_mutex_);
  return sentinel_state_.feedback;
}

double XucSender::imu_roll() const { return sentinel_ ? sentinel_feedback().gimbal.roll : 0.0; }
double XucSender::raw_imu_pitch() const { return imu_pitch_.load(std::memory_order_relaxed); }
std::array<double, 3> XucSender::imu_angles_rad() const
{
  if (!sentinel_) return {0, imu_pitch(), imu_yaw()};
  const auto imu = sentinel_feedback().gimbal;
  return {imu.roll, imu_pitch_sign_*imu.pitch+imu_pitch_offset_rad_,
          imu_yaw_sign_*imu.yaw+imu_yaw_offset_rad_};
}
std::array<double, 3> XucSender::gimbal_gyro_rad_s() const
{
  const auto imu = sentinel_feedback().gimbal;
  if (!(imu.flags & sentinel::gyro_valid)) return {NAN,NAN,NAN};
  return {imu.gx,imu_pitch_sign_*imu.gy,imu_yaw_sign_*imu.gz};
}

XucSender::PoseSnapshot XucSender::pose_snapshot_locked(
  int64_t query_ns, int64_t now_ns, bool capture) const
{
  PoseSnapshot out;
  static_cast<sentinel::PoseEstimate &>(out) = capture
    ? sentinel_pose_history_.at(query_ns, now_ns) : sentinel_pose_history_.latest(now_ns);
  if (!rx_valid_.load(std::memory_order_relaxed) || !sentinel_state_.pose_valid(now_ns)) {
    out.valid = false;
    out.status = sentinel::PoseStatus::stale;
  }
  const bool status_fresh = sentinel_state_.status_valid(now_ns);
  out.mode = status_fresh && (sentinel_state_.feedback.link.state & sentinel::auto_mode) ? 1 : 0;
  out.lower_yaw_armed = status_fresh && (sentinel_state_.feedback.link.state & 0x04);
  out.control_ready = allow_control_ && rx_valid_.load(std::memory_order_relaxed) && !sentinel_tx_failed_ &&
    sentinel_pose_history_.latest(now_ns).valid &&
    sentinel_state_.reference_ready(now_ns, sentinel_require_verified_pose_);
  out.rpy_rad[1] = imu_pitch_sign_ * out.rpy_rad[1] + imu_pitch_offset_rad_;
  out.rpy_rad[2] = imu_yaw_sign_ * out.rpy_rad[2] + imu_yaw_offset_rad_;
  out.gyro_rad_s[1] *= imu_pitch_sign_;
  out.gyro_rad_s[2] *= imu_yaw_sign_;
  return out;
}

XucSender::PoseSnapshot XucSender::pose_now() const
{
  if (!enabled_ || !sentinel_) return {};
  std::lock_guard<std::mutex> lock(sentinel_mutex_);
  return pose_snapshot_locked(0, steady_ns(), false);
}

XucSender::PoseSnapshot XucSender::pose_at_capture(
  Clock::time_point capture, std::chrono::milliseconds wait) const
{
  if (!enabled_ || !sentinel_) return {};
  const int64_t query_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    capture.time_since_epoch()).count();
  const auto until = Clock::now() + std::clamp(wait,
    std::chrono::milliseconds(0), std::chrono::milliseconds(10));
  while (true) {
    PoseSnapshot out;
    {
      std::lock_guard<std::mutex> lock(sentinel_mutex_);
      out = pose_snapshot_locked(query_ns, steady_ns(), true);
    }
    if (out.valid || out.status != sentinel::PoseStatus::bracket_unavailable ||
        Clock::now() >= until) return out;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

XucSender::SendResult XucSender::stop()
{
  if (!enabled_ || !serial_.isOpen()) { SendResult r; r.status=SendStatus::disabled; return r; }
  return sentinel_ ? send_sentinel(Command{}, true) : send(Command{});
}

XucSender::SendResult XucSender::send_sentinel(
  const Command & cmd, bool stop, Clock::time_point fire_deadline, uint64_t expected_generation)
{
  const auto now_ns = steady_ns();
  SendResult result;
  sentinel::Bytes bytes;
  bool requested = false;
  uint8_t transmitted_flags = 0;
  {
    std::lock_guard<std::mutex> lock(sentinel_mutex_);
    if (sentinel_tx_failed_) { result.status=SendStatus::io_failed; return result; }
    // The caller solved this target in one reference generation. Never wrap
    // that old target in a newer MCU epoch/boot merely because it changed
    // while inference or the precise shot deadline was being processed.
    if (!stop && expected_generation != 0 && expected_generation != sentinel_state_.generation)
      return result;
    if (sentinel_output_generation_ != sentinel_state_.generation) {
      sentinel_output_active_ = false;
      sentinel_last_sent_flags_ = 0;
      sentinel_friction_hold_.reset();
      sentinel_output_generation_ = sentinel_state_.generation;
    }
    const bool ready = !stop && allow_control_ && rx_valid_.load(std::memory_order_relaxed) &&
      sentinel_pose_history_.latest(now_ns).valid &&
      sentinel_state_.reference_ready(now_ns,sentinel_require_verified_pose_);
    const bool angular = ready && cmd.control &&
      std::isfinite(cmd.yaw) && std::isfinite(cmd.pitch);
    const bool friction = sentinel_friction_hold_.update(ready && sentinel_allow_friction_,
      cmd.armor_detected, now_ns);
    const bool fire_expired = cmd.shoot && fire_deadline != Clock::time_point{} &&
      Clock::now() > fire_deadline;
    result.fire_expired = fire_expired;
    const bool fire_requested = angular && friction && allow_shoot_ && cmd.shoot && !fire_expired;
    const auto last_fire_ns = last_shoot_tx_ns_.load(std::memory_order_acquire);
    result.fire_rate_limited = fire_requested && last_fire_ns > 0 &&
      now_ns - last_fire_ns < sentinel_fire_min_interval_ns_;
    const bool fire = fire_requested && !result.fire_rate_limited;
    const bool fire_edge = fire && (!(sentinel_last_sent_flags_ & sentinel::fire_request) ||
      fire_deadline != Clock::time_point{});
    const bool clear_fire = !fire && (sentinel_last_sent_flags_ & sentinel::fire_request);
    requested = angular || friction;
    if (!requested && !sentinel_output_active_) return result; // receive-only sends zero bytes.
    // A one-frame qualified fire event must not vanish behind the state-update
    // rate limiter. Expired fire also bypasses it so a prior fire level clears.
    if (requested && !fire_edge && !clear_fire && !fire_expired && sentinel_last_tx_ns_ &&
        now_ns-sentinel_last_tx_ns_<sentinel_tx_interval_ns_) {
      result.status=SendStatus::throttled; return result;
    }
    if (!sentinel_state_.can_stop()) return result;
    sentinel::Aim aim;
    if (requested) {
      const auto& ref=sentinel_state_.feedback.gimbal;
      aim.ref_epoch=ref.epoch; aim.ref_seq=ref.seq; aim.ref_time_us=ref.time_us;
      aim.valid_for_ms=sentinel_valid_for_ms_;
      aim.flags=sentinel::control_enable;
      if (angular) {
        aim.flags|=sentinel::target_valid;
        const double target_pitch=(cmd.pitch-imu_pitch_offset_rad_)/imu_pitch_sign_;
        aim.yaw_error=static_cast<float>(std::remainder(command_yaw_to_raw(cmd.yaw)-ref.yaw,2*sentinel::pi));
        aim.pitch_error=static_cast<float>(target_pitch-ref.pitch);
      }
      if (friction) aim.flags|=sentinel::friction_enable;
      if (fire) aim.flags|=sentinel::fire_request;
      if (!std::isfinite(aim.yaw_error) || !std::isfinite(aim.pitch_error) ||
          std::abs(aim.pitch_error) > sentinel::pi) {
        aim={}; requested=false; // Invalid transformed targets revoke, never retain the last fire request.
        sentinel_friction_hold_.reset();
      }
    }
    result.seq=sentinel_state_.take_sequence();
    bytes=sentinel::encode_aim(aim,result.seq,sentinel_state_.feedback.boot_id);
    transmitted_flags=aim.flags;
  }
  try {
    // Re-check after acquiring/encoding the reference, immediately before I/O.
    if ((transmitted_flags & sentinel::fire_request) &&
        fire_deadline != Clock::time_point{} && Clock::now() > fire_deadline) {
      transmitted_flags &= ~sentinel::fire_request;
      bytes[sentinel::header_size+14]=transmitted_flags;
      sentinel::put16(bytes.data()+bytes.size()-2, sentinel::crc16(bytes.data(),bytes.size()-2));
      result.fire_expired=true;
    }
    if (serial_.write(bytes.data(),bytes.size())!=bytes.size())
      throw std::runtime_error("Incomplete Sentinel V2 serial frame");
    result.tx_time_ns=steady_ns();
    sentinel_last_tx_ns_=result.tx_time_ns; sentinel_output_active_=requested;
    sentinel_last_sent_flags_=transmitted_flags;
    result.status=SendStatus::sent;
    result.control_sent=(transmitted_flags&sentinel::target_valid)!=0;
    result.friction_sent=(transmitted_flags&sentinel::friction_enable)!=0;
    result.fire_sent=(transmitted_flags&sentinel::fire_request)!=0;
    if (result.fire_sent)
      last_shoot_tx_ns_.store(result.tx_time_ns,std::memory_order_release);
  } catch (const std::exception& error) {
    { std::lock_guard<std::mutex> lock(sentinel_mutex_); sentinel_tx_failed_=true; }
    sentinel_output_active_=false;
    result.status=SendStatus::io_failed;
    tools::logger()->warn("[SENTINEL V2] write failed; command output latched off: {}",error.what());
  }
  return result;
}

void XucSender::recv_sentinel_loop()
{
  sentinel::Decoder decoder;
  uint8_t scratch[512];
  while (!recv_quit_.load(std::memory_order_relaxed)) {
    std::size_t count=0;
    try { count=serial_.read(scratch,sizeof(scratch)); }
    catch (const std::exception& error) {
      rx_valid_.store(false,std::memory_order_relaxed);
      tools::logger()->warn("[SENTINEL V2] read stopped: {}",error.what()); return;
    }
    rx_bytes_.fetch_add(count,std::memory_order_relaxed);
    const auto now_ns=steady_ns();
    decoder.feed(scratch,count,now_ns,[this,now_ns](const sentinel::Frame& frame) {
      {
        std::lock_guard<std::mutex> lock(sentinel_mutex_);
        const auto generation=sentinel_state_.generation;
        if (!sentinel_state_.apply(frame,now_ns)) return;
        if (generation!=sentinel_state_.generation) {
          tracker_reset_.store(true);
          sentinel_pose_history_.reset();
        }
        const auto& imu=sentinel_state_.feedback.gimbal;
        if (frame.type==sentinel::gimbal_type)
          sentinel_pose_history_.push(imu, sentinel_state_.feedback.boot_id,
            sentinel_state_.generation, now_ns);
        imu_yaw_.store(imu.yaw,std::memory_order_relaxed);
        imu_pitch_.store(imu.pitch,std::memory_order_relaxed);
      }
      // Paired firmware emits status in every mode. Never infer AUTO from incoming bytes.
      mode_.store((sentinel_feedback().link.state & sentinel::auto_mode) ? 1 : 0,
                  std::memory_order_relaxed);
      valid_packets_.fetch_add(1,std::memory_order_relaxed);
      last_rx_ns_.store(now_ns,std::memory_order_relaxed);
      rx_valid_.store(true,std::memory_order_relaxed);
    });
    header_candidates_.store(decoder.headers,std::memory_order_relaxed);
    crc_errors_.store(decoder.crc_errors,std::memory_order_relaxed);
    field_errors_.store(decoder.field_errors,std::memory_order_relaxed);
  }
}

XucSender::SendResult XucSender::send(
  const Command & cmd, Clock::time_point fire_deadline, uint64_t expected_generation)
{
  SendResult result;
  if (!enabled_ || !serial_.isOpen()) { result.status=SendStatus::disabled; return result; }
  if (sentinel_) return send_sentinel(cmd, false, fire_deadline, expected_generation);

  XucRxPacket pkt{};
  pkt.head[0] = 'S';
  pkt.head[1] = 'P';
  const bool link_ready = control_ready();
  const bool control_requested = link_ready && cmd.control;
  pkt.control = control_requested ? 1 : 0;
  result.fire_expired=cmd.shoot && fire_deadline!=Clock::time_point{} && Clock::now()>fire_deadline;
  pkt.shoot = (control_requested && allow_shoot_ && cmd.shoot && !result.fire_expired) ? 1 : 0;

  double target_yaw_raw = yaw_sign_ * cmd.yaw;
  double target_pitch_raw = pitch_sign_ * cmd.pitch;
  const bool bounded_session =
    max_yaw_excursion_rad_ > 0.0 || lock_pitch_for_control_;

  // This diagnostic profile intentionally permits only one yaw origin per
  // process.  Detector dropouts, link recovery, or a chattering feedback mode
  // must never move that origin.  A new physical test requires stopping and
  // restarting the process, which is also the safest explicit re-arm boundary.
  if (control_requested && bounded_session) {
    if (!control_session_active_) {
      control_yaw_anchor_raw_ =
        static_cast<double>(imu_yaw_.load(std::memory_order_relaxed));
      control_session_active_ = true;
      tools::logger()->warn(
        "[XUC][BOUNDED] control session anchored at raw yaw {:.2f} deg",
        control_yaw_anchor_raw_ * 180.0 / M_PI);
    }

    if (max_yaw_excursion_rad_ > 0.0) {
      double delta = target_yaw_raw - control_yaw_anchor_raw_;
      delta = std::remainder(delta, 2.0 * M_PI);
      delta = std::clamp(delta, -max_yaw_excursion_rad_, max_yaw_excursion_rad_);
      target_yaw_raw = control_yaw_anchor_raw_ + delta;
    }
    if (lock_pitch_for_control_) {
      target_pitch_raw =
        static_cast<double>(imu_pitch_.load(std::memory_order_relaxed));
    }
  } else if (!bounded_session) {
    control_session_active_ = false;
  }

  pkt.yaw = static_cast<float>(target_yaw_raw);
  pkt.pitch = static_cast<float>(target_pitch_raw);

  // CRC16 对前 12 字节计算，低字节在前
  uint16_t crc = tools::get_crc16(reinterpret_cast<const uint8_t *>(&pkt),
                                  offsetof(XucRxPacket, crc16));
  uint8_t buf[sizeof(XucRxPacket)]{};
  std::memcpy(buf, &pkt, offsetof(XucRxPacket, crc16));
  buf[offsetof(XucRxPacket, crc16)] = static_cast<uint8_t>(crc & 0xff);
  buf[offsetof(XucRxPacket, crc16) + 1] = static_cast<uint8_t>((crc >> 8) & 0xff);

  try {
    if (serial_.write(buf, sizeof(buf)) != sizeof(buf))
      throw std::runtime_error("Incomplete XUC serial frame");
    result.status=SendStatus::sent;
    result.tx_time_ns=steady_ns();
    result.control_sent=pkt.control!=0;
    result.fire_sent=pkt.shoot!=0;
    if (result.fire_sent) last_shoot_tx_ns_.store(result.tx_time_ns, std::memory_order_release);
  } catch (const std::exception & e) {
    result.status=SendStatus::io_failed;
    tools::logger()->warn("[XUC] write failed: {}", e.what());
  }
  return result;
}

// 接收线程：读串口字节 → 扫描 'S''P' + CRC16 校验 → 解析 TxPacket_TJ（20 字节）
void XucSender::recv_loop()
{
  uint8_t scratch[64];
  std::vector<uint8_t> buf;
  buf.reserve(sizeof(XucTxPacket) * 2);

  while (!recv_quit_.load(std::memory_order_relaxed)) {
    size_t n = 0;
    try {
      n = serial_.read(scratch, sizeof(scratch));
    } catch (const std::exception &) {
      break;
    }
    if (n == 0) continue;  // 超时无数据
    rx_bytes_.fetch_add(static_cast<uint64_t>(n), std::memory_order_relaxed);
    buf.insert(buf.end(), scratch, scratch + n);

    size_t i = 0;
    while (i + sizeof(XucTxPacket) <= buf.size()) {
      if (buf[i] == 'S' && buf[i + 1] == 'P') {
        header_candidates_.fetch_add(1, std::memory_order_relaxed);
        uint16_t crc = tools::get_crc16(&buf[i], offsetof(XucTxPacket, crc16));
        if (static_cast<uint8_t>(crc & 0xff) == buf[i + offsetof(XucTxPacket, crc16)] &&
            static_cast<uint8_t>((crc >> 8) & 0xff) == buf[i + offsetof(XucTxPacket, crc16) + 1]) {
          XucTxPacket pkt{};
          std::memcpy(&pkt, &buf[i], sizeof(XucTxPacket));

          const bool fields_valid =
            pkt.mode <= 3 &&
            std::isfinite(pkt.bullet_speed) &&
            std::isfinite(pkt.imu_pitch) &&
            std::isfinite(pkt.imu_yaw);

          if (fields_valid) {
            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
            const bool feeder_active =
              std::abs(static_cast<double>(pkt.bullet_speed)) >=
              feeder_active_rpm_threshold_;
            {
              std::lock_guard<std::mutex> lock(feeder_feedback_mutex_);
              if (feeder_active != feeder_feedback_.active) {
                feeder_feedback_.active = feeder_active;
                feeder_feedback_.transition_time_ns = now_ns;
                feeder_feedback_.rpm = static_cast<double>(pkt.bullet_speed);
                ++feeder_feedback_.transition_sequence;
              }
            }
            mode_.store(pkt.mode, std::memory_order_relaxed);
            robot_id_.store(pkt.robot_id, std::memory_order_relaxed);
            bullet_speed_.store(pkt.bullet_speed, std::memory_order_relaxed);
            bullet_count_.store(pkt.bullet_count, std::memory_order_relaxed);
            imu_pitch_.store(pkt.imu_pitch, std::memory_order_relaxed);
            imu_yaw_.store(pkt.imu_yaw, std::memory_order_relaxed);

            last_rx_ns_.store(now_ns, std::memory_order_relaxed);
            valid_packets_.fetch_add(1, std::memory_order_relaxed);
            rx_valid_.store(true, std::memory_order_relaxed);
          } else {
            field_errors_.fetch_add(1, std::memory_order_relaxed);
          }

          i += sizeof(XucTxPacket);
          continue;
        }
        crc_errors_.fetch_add(1, std::memory_order_relaxed);
      }
      ++i;
    }
    buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(i));
    // 防止残留缓冲无限增长（最多保留一个半包，供后续拼包）
    if (buf.size() > sizeof(XucTxPacket) + 2) {
      buf.erase(buf.begin(),
                buf.begin() + static_cast<std::ptrdiff_t>(buf.size() - sizeof(XucTxPacket) - 2));
    }
  }

  rx_valid_.store(false, std::memory_order_relaxed);
}

}  // namespace io

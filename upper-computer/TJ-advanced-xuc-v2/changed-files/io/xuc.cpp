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

  if (yaml["xuc_enable"]) enabled_ = yaml["xuc_enable"].as<bool>();
  if (!enabled_) {
    tools::logger()->info("[XUC] disabled (xuc_enable=false)");
    return;
  }

  std::string port = "/dev/ttyUSB0";
  uint32_t baud = 460800;
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
    // 启动接收线程：解析新版 TxPacket_TJ
    recv_thread_ = std::thread([this] { recv_loop(); });
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
  recv_quit_.store(true, std::memory_order_relaxed);
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

bool XucSender::rx_valid() const
{
  if (!rx_valid_.load(std::memory_order_relaxed)) return false;

  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto last_ns = last_rx_ns_.load(std::memory_order_relaxed);

  return last_ns > 0 && now_ns - last_ns <= RX_TIMEOUT_NS;
}

void XucSender::send(const Command & cmd)
{
  if (!enabled_ || !serial_.isOpen()) return;

  XucRxPacket pkt{};
  pkt.head[0] = 'S';
  pkt.head[1] = 'P';
  const bool link_ready = control_ready();
  const bool control_requested = link_ready && cmd.control;
  pkt.control = control_requested ? 1 : 0;
  pkt.shoot = (control_requested && allow_shoot_ && cmd.shoot) ? 1 : 0;

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
    serial_.write(buf, sizeof(buf));
  } catch (const std::exception & e) {
    tools::logger()->warn("[XUC] write failed: {}", e.what());
  }
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
            mode_.store(pkt.mode, std::memory_order_relaxed);
            robot_id_.store(pkt.robot_id, std::memory_order_relaxed);
            bullet_speed_.store(pkt.bullet_speed, std::memory_order_relaxed);
            bullet_count_.store(pkt.bullet_count, std::memory_order_relaxed);
            imu_pitch_.store(pkt.imu_pitch, std::memory_order_relaxed);
            imu_yaw_.store(pkt.imu_yaw, std::memory_order_relaxed);

            const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
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

// 新版XUC真实下位机探测工具：默认只接收，可选发送全零安全心跳。
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

#include "io/command.hpp"
#include "io/xuc.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::printf(
      "用法: %s <serial_port> [baud=460800] [seconds=10] [--sentinel|--safe-heartbeat]\n",
      argv[0]);
    return 2;
  }

  const std::string port = argv[1];
  uint32_t baud = 460800;
  double duration_seconds = 10.0;
  bool safe_heartbeat = false;
  const bool sentinel = argc >= 5 && std::string(argv[4]) == "--sentinel";

  try {
    if (argc >= 3) baud = static_cast<uint32_t>(std::stoul(argv[2]));
    if (argc >= 4) duration_seconds = std::stod(argv[3]);
    if (argc >= 5) safe_heartbeat = std::string(argv[4]) == "--safe-heartbeat";
  } catch (const std::exception & e) {
    std::printf("[FAIL] 参数错误: %s\n", e.what());
    return 2;
  }

  if (duration_seconds <= 0.0 || duration_seconds > 300.0) {
    std::printf("[FAIL] seconds必须位于(0, 300]\n");
    return 2;
  }

  const std::string config_path = "/tmp/xuc_board_probe.yaml";
  {
    std::ofstream config(config_path);
    config
      << "board_protocol: " << (sentinel ? "SENTINEL" : "SP") << "\n"
      << "xuc_enable: true\n"
      << "xuc_serial_port: \"" << port << "\"\n"
      << "xuc_serial_baud: " << baud << "\n"
      << "xuc_yaw_sign: 1\n"
      << "xuc_pitch_sign: -1\n"
      << "xuc_imu_yaw_sign: 1\n"
      << "xuc_imu_pitch_sign: 1\n"
      << "xuc_allow_control: false\n"
      << "xuc_allow_shoot: false\n"
      << "xuc_require_feedback_for_control: true\n";
  }

  std::printf("XUC_PROBE_PORT=%s\n", port.c_str());
  std::printf("XUC_PROBE_BAUD=%u\n", baud);
  if (sentinel) std::printf("PROTOCOL=SENTINEL RX=64 TX=40; receive-only, sends 0 bytes\n");
  std::printf(
    "XUC_PROBE_MODE=%s\n",
    safe_heartbeat ? "SAFE_HEARTBEAT" : "RECEIVE_ONLY");

  io::XucSender xuc(config_path);
  if (!xuc.enabled()) {
    std::printf("XUC_PROBE_RESULT=SERIAL_OPEN_FAILED\n");
    std::remove(config_path.c_str());
    return 3;
  }

  io::Command safe_command{};
  safe_command.control = false;
  safe_command.shoot = false;
  safe_command.yaw = 0.0;
  safe_command.pitch = 0.0;

  int valid_samples = 0;
  int invalid_samples = 0;
  bool was_valid = false;
  const auto start = std::chrono::steady_clock::now();
  auto next_log = start;

  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
         duration_seconds) {
    if (safe_heartbeat) xuc.send(safe_command);

    const bool valid = sentinel ? xuc.feedback_link_valid() : xuc.rx_valid();
    if (valid) {
      ++valid_samples;
    } else {
      ++invalid_samples;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_log || valid != was_valid) {
      if (valid) {
        if (sentinel) {
          const auto f = xuc.sentinel_feedback();
          std::printf("[SENTINEL] link=1 pose=%u color=%u reset=%u rx_total=%u rx_count=%u "
                      "p_deg=(%.3f,%.3f,%.3f) p_dps=(%.3f,%.3f,%.3f) p_time=%u p_id=%u p_valid=%u "
                      "c_deg=(%.3f,%.3f,%.3f) c_time=%u c_id=%u c_valid=%u\n",
            xuc.rx_valid() ? 1U : 0U, unsigned(f.detect_color), unsigned(f.reset_tracker),
            unsigned(f.rx_total), unsigned(f.rx_count),
            f.gimbal.roll_deg, f.gimbal.pitch_deg, f.gimbal.yaw_deg,
            f.gimbal.gx_dps, f.gimbal.gy_dps, f.gimbal.gz_dps,
            f.gimbal.time_ms, unsigned(f.gimbal.id), unsigned(f.gimbal.valid),
            f.chassis.roll_deg, f.chassis.pitch_deg, f.chassis.yaw_deg,
            f.chassis.time_ms, unsigned(f.chassis.id), unsigned(f.chassis.valid));
        } else {
        std::printf(
          "[RX] valid=1 mode=%u robot_id=%u bullet_speed=%.3f bullet_count=%u "
          "imu_pitch=%.6f imu_yaw=%.6f\n",
          static_cast<unsigned>(xuc.mode()), static_cast<unsigned>(xuc.robot_id()),
          xuc.bullet_speed(), static_cast<unsigned>(xuc.bullet_count()),
          xuc.imu_pitch(), xuc.imu_yaw());
        }
      } else {
        std::printf("[RX] valid=0 waiting_or_timeout\n");
      }
      next_log = now + std::chrono::seconds(1);
    }
    was_valid = valid;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::remove(config_path.c_str());
  std::printf("VALID_SAMPLES=%d\n", valid_samples);
  std::printf("INVALID_SAMPLES=%d\n", invalid_samples);
  std::printf("RX_BYTES=%llu\n", static_cast<unsigned long long>(xuc.rx_bytes()));
  std::printf("HEADER_CANDIDATES=%llu\n", static_cast<unsigned long long>(xuc.header_candidates()));
  std::printf("CRC_ERRORS=%llu\n", static_cast<unsigned long long>(xuc.crc_errors()));
  std::printf("FIELD_ERRORS=%llu\n", static_cast<unsigned long long>(xuc.field_errors()));
  std::printf("VALID_PACKETS=%llu\n", static_cast<unsigned long long>(xuc.valid_packets()));
  std::printf(
    "XUC_PROBE_RESULT=%s\n",
    valid_samples > 0 ? "PASS" : "NO_VALID_PACKET");
  return valid_samples > 0 ? 0 : 4;
}

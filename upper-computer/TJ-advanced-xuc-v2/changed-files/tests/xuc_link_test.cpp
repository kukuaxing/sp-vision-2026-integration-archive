// XUC虚拟下位机双向链路测试
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>

#include "io/command.hpp"
#include "io/xuc.hpp"

static bool near(double value, double expected, double tolerance)
{
  return std::fabs(value - expected) <= tolerance;
}

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::printf("用法: %s <serial_port> [--safety]\n", argv[0]);
    return 1;
  }

  const std::string port = argv[1];
  const bool safety_profile = argc >= 3 && std::string(argv[2]) == "--safety";
  const std::string cfg_path = "/tmp/xuc_link_test.yaml";

  {
    std::ofstream config(cfg_path);
    config
      << "xuc_enable: true\n"
      << "xuc_serial_port: \"" << port << "\"\n"
      << "xuc_serial_baud: 460800\n"
      << "xuc_yaw_sign: 1\n"
      << "xuc_pitch_sign: -1\n"
      << "xuc_allow_control: true\n"
      << "xuc_allow_shoot: " << (safety_profile ? "false" : "true") << "\n"
      << "xuc_require_feedback_for_control: "
      << (safety_profile ? "true" : "false") << "\n"
      << "xuc_require_auto_mode_for_control: "
      << (safety_profile ? "true" : "false") << "\n"
      << "xuc_max_yaw_excursion_rad: "
      << (safety_profile ? "0.05" : "0.0") << "\n"
      << "xuc_lock_pitch_for_control: "
      << (safety_profile ? "true" : "false") << "\n";
  }
  std::printf("TEST_PROFILE=%s\n", safety_profile ? "SAFETY" : "PROTOCOL");

  io::XucSender xuc(cfg_path);

  if (!xuc.enabled()) {
    std::printf("[FAIL] XUC串口无法打开\n");
    return 2;
  }

  io::Command command{};
  command.control = true;
  command.shoot = true;
  command.yaw = 0.30;
  command.pitch = -0.15;

  bool accepted_bad_crc = false;
  bool saw_initial_valid = false;
  bool saw_timeout_invalid = false;
  bool saw_recovery_valid = false;
  bool fields_valid = true;

  const auto start = std::chrono::steady_clock::now();

  while (true) {
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        now - start).count();

    if (elapsed_ms >= 4000) break;

    // Exercise a brief detector/tracker dropout while AUTO feedback remains
    // active.  A bounded session must keep its original yaw anchor.
    command.control =
      !(safety_profile && elapsed_ms >= 1150 && elapsed_ms < 1300);
    xuc.send(command);

    const bool valid = xuc.rx_valid();

    if (elapsed_ms < 600 && valid) {
      accepted_bad_crc = true;
    }

    if (elapsed_ms >= 900 &&
        elapsed_ms < 1450 &&
        valid) {
      saw_initial_valid = true;
    }

    if (elapsed_ms >= 2100 &&
        elapsed_ms < 2350 &&
        !valid) {
      saw_timeout_invalid = true;
    }

    if (elapsed_ms >= 2700 && valid) {
      saw_recovery_valid = true;
    }

    if (valid) {
      fields_valid =
        fields_valid &&
        (safety_profile ? xuc.mode() <= 1 : xuc.mode() == 1) &&
        xuc.robot_id() == 3 &&
        near(xuc.bullet_speed(), 14.5, 0.01) &&
        xuc.bullet_count() > 0 &&
        near(xuc.imu_pitch(), 0.10, 0.001) &&
        (near(xuc.imu_yaw(), 0.50, 0.001) ||
         near(xuc.imu_yaw(), 0.55, 0.001));
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(50));
  }

  auto result = [](bool ok, const char * name) {
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
  };

  result(!accepted_bad_crc, "错误CRC反馈未被接受");
  result(saw_initial_valid, "正常反馈能够被解析");
  result(fields_valid, "反馈字段和IMU数值正确");
  result(saw_timeout_invalid, "反馈中断500ms后状态失效");
  result(saw_recovery_valid, "反馈恢复后链路重新有效");
  result(xuc.rx_bytes() > 0, "接收字节计数有效");
  result(xuc.crc_errors() > 0, "错误CRC已计数");
  result(xuc.valid_packets() > 0, "合法反馈包已计数");

  const bool passed =
    !accepted_bad_crc &&
    saw_initial_valid &&
    fields_valid &&
    saw_timeout_invalid &&
    saw_recovery_valid &&
    xuc.rx_bytes() > 0 &&
    xuc.crc_errors() > 0 &&
    xuc.valid_packets() > 0;

  std::printf(
    "XUC_LINK_RESULT=%s\n",
    passed ? "PASS" : "FAIL");

  return passed ? 0 : 3;
}

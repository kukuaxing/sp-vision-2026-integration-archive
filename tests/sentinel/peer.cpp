// Private PTY only. Exercise the actual production sender and receiver.
#include "io/xuc.hpp"
#include <cstdio>
#include <thread>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  io::XucSender link(argv[1]);
  if (!link.enabled() || !link.is_sentinel()) return 3;
  const auto start = std::chrono::steady_clock::now();
  bool saw_pose = false, saw_invalid_pose_with_link = false, saw_reset = false;
  while (true) {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    if (elapsed > 3.2) break;
    saw_pose |= link.rx_valid();
    saw_invalid_pose_with_link |= link.feedback_link_valid() && !link.rx_valid();
    saw_reset |= link.consume_tracker_reset();
    io::Command command{};
    command.control = !(elapsed > 2.4 && elapsed < 2.6);
    command.shoot = elapsed > 0.4 && elapsed < 0.46;
    command.yaw = 179 * io::sentinel::deg_to_rad; // sign=-1 -> raw -179; shortest error +2 deg
    command.pitch = 12.5 * io::sentinel::deg_to_rad;
    command.horizon_distance = 3;
    link.send(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const auto f = link.sentinel_feedback();
  const auto gyro = link.gimbal_gyro_rad_s();
  const bool normalized = std::abs(link.imu_roll() / io::sentinel::deg_to_rad - 22) < .001 &&
    std::abs(link.imu_pitch() / io::sentinel::deg_to_rad - 11) < .001 &&
    std::abs(link.imu_yaw() / io::sentinel::deg_to_rad + 179) < .001 &&
    std::abs(gyro[0] / io::sentinel::deg_to_rad - 2) < .001 &&
    std::abs(gyro[1] / io::sentinel::deg_to_rad - 1) < .001 &&
    std::abs(gyro[2] / io::sentinel::deg_to_rad + 3) < .001;
  std::printf("SUMMARY pose=%d invalid_with_link=%d reset=%d normalized=%d frames=%llu crc=%llu id=%u count=%u\n",
    saw_pose, saw_invalid_pose_with_link, saw_reset, normalized,
    (unsigned long long)link.valid_packets(), (unsigned long long)link.crc_errors(),
    unsigned(f.gimbal.id), unsigned(f.rx_count));
  return saw_pose && saw_invalid_pose_with_link && saw_reset && normalized && link.crc_errors() ? 0 : 4;
}

// 新版XUC真实下位机探测工具：默认只接收，可选发送全零安全心跳。
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "io/command.hpp"
#include "io/xuc.hpp"

namespace
{
volatile std::sig_atomic_t record_stop = 0;

std::string json_number(double value)
{
  if (!std::isfinite(value)) return "null";
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}

std::string rpy_degrees(const io::sentinel::Imu & imu)
{
  constexpr double degrees = 180.0 / io::sentinel::pi;
  return "[" + json_number(imu.roll * degrees) + "," +
    json_number(imu.pitch * degrees) + "," + json_number(imu.yaw * degrees) + "]";
}

void write_line(std::FILE * output, const std::string & line)
{
  if (std::fwrite(line.data(), 1, line.size(), output) != line.size() ||
      std::fputc('\n', output) == EOF || std::fflush(output) != 0)
    throw std::runtime_error("JSONL write failed");
}

// Deliberately separate from XucSender: recording has no transmitter object,
// no Command, no serial.write call, and no destructor cancellation packet.
int record_sentinel(
  const std::string & port, uint32_t baud, double duration_seconds,
  const std::string & output_path)
{
  std::unique_ptr<std::FILE, decltype(&std::fclose)> output(
    std::fopen(output_path.c_str(), "wx"), &std::fclose);
  if (!output) {
    std::fprintf(stderr, "[FAIL] Cannot exclusively create unused JSONL path: %s\n",
      output_path.c_str());
    return 2;
  }
  std::signal(SIGINT, [](int) { record_stop = 1; });
  std::signal(SIGTERM, [](int) { record_stop = 1; });
  const auto start = std::chrono::steady_clock::now();
  io::sentinel::Decoder decoder;
  io::sentinel::State state;
  uint64_t rx_bytes = 0, valid_packets = 0, sample_count = 0;
  uint32_t last_boot = 0, last_source_time = 0;
  uint16_t last_epoch = 0;
  bool have_sample = false, completed = false;
  int result = 3;
  try {
    serial::Serial receiver;
    receiver.setPort(port);
    receiver.setBaudrate(baud);
    receiver.setFlowcontrol(serial::flowcontrol_none);
    receiver.setParity(serial::parity_none);
    receiver.setStopbits(serial::stopbits_one);
    receiver.setBytesize(serial::eightbits);
    auto timeout = serial::Timeout::simpleTimeout(2);
    receiver.setTimeout(timeout);
    receiver.open();
    std::printf("XUC_PROBE_MODE=RECORD_RECEIVE_ONLY TX_BYTES=0\n");
    uint8_t scratch[512];
    while (!record_stop) {
      const auto before_read = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(before_read - start).count() >= duration_seconds) break;
      const auto count = receiver.read(scratch, sizeof(scratch));
      rx_bytes += count;
      const auto now = std::chrono::steady_clock::now();
      const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      decoder.feed(scratch, count, ns, [&](const io::sentinel::Frame & frame) {
        if (!state.apply(frame, ns)) return;
        ++valid_packets;
        if (frame.type != io::sentinel::gimbal_type || !state.pose_valid(ns)) return;
        const auto & f = state.feedback;
        const auto & p = f.gimbal;
        // A new packet sequence with a frozen source timestamp is not a new
        // physical attitude sample. Handle epoch/boot and counter wraps too.
        if (have_sample && last_boot == f.boot_id && last_epoch == p.epoch &&
            last_source_time == p.time_us) return;
        have_sample = true;
        last_boot = f.boot_id; last_epoch = p.epoch; last_source_time = p.time_us;
        std::ostringstream row;
        row << "{\"type\":\"gimbal\",\"elapsed_s\":" << json_number(elapsed)
            << ",\"boot_id\":" << f.boot_id << ",\"seq\":" << p.seq
            << ",\"epoch\":" << p.epoch << ",\"source_time_us\":" << p.time_us
            << ",\"gimbal_id\":" << unsigned(p.id) << ",\"gimbal_flags\":" << unsigned(p.flags)
            << ",\"rpy_order\":\"roll_pitch_yaw\",\"raw_rpy_deg\":" << rpy_degrees(p)
            << ",\"raw_gyro_rad_s\":[" << json_number(p.gx) << ","
            << json_number(p.gy) << "," << json_number(p.gz) << "]"
            << ",\"gyro_valid\":" << ((p.flags & io::sentinel::gyro_valid) ? "true" : "false")
            << ",\"chassis\":{\"received\":" << (f.chassis.received ? "true" : "false")
            << ",\"seq\":" << f.chassis.seq << ",\"epoch\":" << f.chassis.epoch
            << ",\"source_time_us\":" << f.chassis.time_us << ",\"id\":" << unsigned(f.chassis.id)
            << ",\"flags\":" << unsigned(f.chassis.flags) << ",\"raw_rpy_deg\":" << rpy_degrees(f.chassis)
            << "},\"mcu_status\":{\"received\":" << (f.link.received ? "true" : "false")
            << ",\"seq\":" << f.link.seq << ",\"state\":" << unsigned(f.link.state)
            << ",\"aim_last_seq\":" << f.link.aim_last_seq
            << ",\"last_reject\":" << unsigned(f.link.last_reject)
            << ",\"aim_age_ms\":" << f.link.aim_age_ms << "}}";
        write_line(output.get(), row.str());
        ++sample_count;
      });
    }
    receiver.close();
    completed = true;
    result = sample_count ? 0 : 4;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "[FAIL] Recording: %s\n", error.what());
  }
  std::ostringstream summary;
  summary << "{\"type\":\"summary\",\"sample_count\":" << sample_count
          << ",\"rx_bytes\":" << rx_bytes << ",\"valid_packets\":" << valid_packets
          << ",\"header_candidates\":" << decoder.headers
          << ",\"crc_errors\":" << decoder.crc_errors
          << ",\"field_errors\":" << decoder.field_errors
          << ",\"unknown_types\":" << decoder.unknown_types
          << ",\"tx_bytes\":0,\"transmitter_constructed\":false,\"completed\":"
          << (completed ? "true" : "false") << ",\"interrupted\":"
          << (record_stop ? "true" : "false") << "}";
  try { write_line(output.get(), summary.str()); }
  catch (const std::exception & error) {
    std::fprintf(stderr, "[FAIL] Summary: %s\n", error.what());
    return 3;
  }
  std::printf("RECORDED_SAMPLES=%llu\nTX_BYTES=0\nXUC_PROBE_RESULT=%s\n",
    static_cast<unsigned long long>(sample_count),
    result == 0 ? "PASS" : (result == 4 ? "NO_VALID_PACKET" : "RECORD_FAILED"));
  return result;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::printf(
      "用法: %s <serial_port> [baud=460800] [seconds=10] [--sentinel] "
      "[--safe-heartbeat|--record-jsonl <unused_path>]\n",
      argv[0]);
    return 2;
  }

  const std::string port = argv[1];
  uint32_t baud = 460800;
  double duration_seconds = 10.0;
  bool safe_heartbeat = false;
  bool sentinel = false;
  std::string record_path;

  try {
    if (argc >= 3) baud = static_cast<uint32_t>(std::stoul(argv[2]));
    if (argc >= 4) duration_seconds = std::stod(argv[3]);
    for (int index = 4; index < argc; ++index) {
      const std::string flag = argv[index];
      if (flag == "--sentinel") sentinel = true;
      else if (flag == "--safe-heartbeat") safe_heartbeat = true;
      else if (flag == "--record-jsonl" && index + 1 < argc && record_path.empty()) {
        record_path = argv[++index];
        if (record_path.empty() || record_path.rfind("--", 0) == 0)
          throw std::invalid_argument("--record-jsonl needs an unused output path");
      }
      else throw std::invalid_argument("Unknown or incomplete option: " + flag);
    }
  } catch (const std::exception & e) {
    std::printf("[FAIL] 参数错误: %s\n", e.what());
    return 2;
  }

  if (!std::isfinite(duration_seconds) || duration_seconds <= 0.0 || duration_seconds > 300.0) {
    std::printf("[FAIL] seconds必须位于(0, 300]\n");
    return 2;
  }

  if (!record_path.empty()) {
    if (!sentinel || safe_heartbeat || baud != 921600) {
      std::fprintf(stderr, "[FAIL] --record-jsonl requires --sentinel at 921600 and forbids --safe-heartbeat\n");
      return 2;
    }
    return record_sentinel(port, baud, duration_seconds, record_path);
  }

  // Keep the ordinary probe's temporary config unique too. Exclusive creation
  // works on Linux and keeps the existing Windows offline target compilable.
  std::error_code temporary_error;
  const auto temporary_root = std::filesystem::temp_directory_path(temporary_error);
  std::string config_path;
  std::unique_ptr<std::FILE, decltype(&std::fclose)> temporary(nullptr, &std::fclose);
  const auto unique_tick = std::chrono::steady_clock::now().time_since_epoch().count();
  for (int attempt = 0; !temporary_error && attempt < 16; ++attempt) {
    config_path = (temporary_root / ("xuc_board_probe_" + std::to_string(unique_tick) +
      "_" + std::to_string(attempt) + ".yaml")).string();
    temporary.reset(std::fopen(config_path.c_str(), "wx"));
    if (temporary) break;
  }
  if (!temporary) {
    std::fprintf(stderr, "[FAIL] Cannot create temporary probe config\n");
    return 2;
  }
  {
    std::ostringstream config;
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
      << "sentinel_allow_friction: false\n"
      << "xuc_require_feedback_for_control: true\n";
    try { write_line(temporary.get(), config.str()); }
    catch (const std::exception & error) {
      temporary.reset();
      std::remove(config_path.c_str());
      std::fprintf(stderr, "[FAIL] Temporary config: %s\n", error.what());
      return 2;
    }
    temporary.reset();
  }

  std::printf("XUC_PROBE_PORT=%s\n", port.c_str());
  std::printf("XUC_PROBE_BAUD=%u\n", baud);
  if (sentinel) std::printf("PROTOCOL=SENTINEL_V2 RX=46+34+26 TX=38; receive-only, sends 0 bytes\n");
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
          std::printf("[SENTINEL V2] link=1 pose=%u boot=%u "
                      "p_rad=(%.6f,%.6f,%.6f) gyro_rad_s=(%.6f,%.6f,%.6f) "
                      "p_time_us=%u p_seq=%u p_epoch=%u p_id=%u p_flags=0x%02x "
                      "c_time_us=%u c_id=%u c_flags=0x%02x aim_seq=%u state=0x%02x reject=%u age_ms=%u\n",
            xuc.rx_valid()?1U:0U,f.boot_id,f.gimbal.roll,f.gimbal.pitch,f.gimbal.yaw,
            f.gimbal.gx,f.gimbal.gy,f.gimbal.gz,f.gimbal.time_us,unsigned(f.gimbal.seq),
            unsigned(f.gimbal.epoch),unsigned(f.gimbal.id),unsigned(f.gimbal.flags),
            f.chassis.time_us,unsigned(f.chassis.id),unsigned(f.chassis.flags),
            unsigned(f.link.aim_last_seq),unsigned(f.link.state),unsigned(f.link.last_reject),
            unsigned(f.link.aim_age_ms));
          const auto feed=xuc.sentinel_feed_diagnostic();
          std::printf("[FEED DIAGNOSTIC] received=%u flags=0x%02x sample_us=%u "
                      "rpm=%d issue_count=%u issue_seq=%u motion_count=%u motion_seq=%u "
                      "muzzle=UNMEASURED\n",unsigned(feed.received),unsigned(feed.flags),
            feed.sample_us,int(feed.feeder_rpm),feed.issue_count,unsigned(feed.issue_seq),
            feed.motion_count,unsigned(feed.motion_seq));
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

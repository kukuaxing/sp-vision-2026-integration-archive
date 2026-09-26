// Linux PTY-only integration exercise. Never accepts a physical serial path.
#include "io/rmv/session.hpp"
#include "io/rmv/serial.hpp"
#include <chrono>
#include <cctype>
#include <iostream>
#include <string>
#include <thread>

namespace {
std::uint64_t now_us() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}
bool allowed_pty(const std::string& path) {
  const std::string prefix = "/dev/pts/";
  return path.compare(0, prefix.size(), prefix) == 0 && path.size() > prefix.size() &&
    std::all_of(path.begin() + prefix.size(), path.end(),
                [](unsigned char value) { return std::isdigit(value) != 0; });
}
}
int main(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) != "--pty-offline" || !allowed_pty(argv[2])) {
    std::cerr << "Only --pty-offline /dev/pts/<number> is allowed; no physical serial device.\n";
    return 2;
  }
  io::rmv::Session session;
  io::rmv::SerialTransport transport;
  try {
    transport.open(argv[2]);
    const auto begin = now_us();
    if (!session.begin(0x5054590000000001ULL, begin)) return 3;
    unsigned stage = 0, targets = 0;
    std::uint64_t motion_begin = 0, previous_target = 0;
    bool stop_acked = false;
    while (now_us() - begin < 5000000) {
      session.ingest(transport.receive(), now_us());
      const auto now = now_us();
      if (stage > 0 && !session.active()) throw std::runtime_error("session lost during PTY test");
      const auto& result = session.last_result();
      if (result && result->type == 0x0202 && result->completion == io::rmv::Completion::accepted)
        stop_acked = true;
      if (stage == 0 && session.active() && !session.pending() && session.telemetry().epoch &&
          session.telemetry().clock.to_mcu(now, now) && session.request_mode(3, 3, now)) stage = 1;
      if (stage == 1 && session.control_ready(now)) {
        stage = 2; motion_begin = now; previous_target = now - 20000;
      }
      if (stage == 2 && now - motion_begin >= 300000) {
        if (!session.stop(now)) throw std::runtime_error("STOP could not be queued");
        stage = 3;
      }
      if (stage == 2 && now - previous_target >= 20000) {
        if (!session.aim(.1f, .05f, now, 2)) throw std::runtime_error("fresh PTY target was rejected locally");
        ++targets; previous_target = now;
      }
      if (stage == 3 && stop_acked && session.telemetry().status &&
          session.telemetry().status->mode == 0 && !session.control_ready(now)) {
        transport.close(); session.disconnect();
        std::cout << "{\"success\":true,\"hardware\":false,\"transport\":\"linux-pty\","
                  << "\"targets\":" << targets << ",\"stop_acked\":true}\n";
        return targets >= 5 ? 0 : 4;
      }
      session.tick(now);
      for (const auto& bytes : session.take_outgoing(now_us())) transport.send(bytes);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error("PTY handshake/control/STOP did not complete within 5 seconds");
  } catch (const std::exception& error) {
    session.disconnect();
    try { transport.close(); } catch (...) {}
    std::cerr << "PTY offline failure: " << error.what() << '\n'; return 1;
  }
}

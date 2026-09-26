// Deterministic process boundary for Python Board <-> production C++ Session.
// It never opens devices and refuses to start without the explicit offline flag.
#include "io/rmv/session.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {
int digit(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  return -1;
}
io::rmv::Bytes decode(const std::string& value) {
  if (value == "-") return {};
  if (value.size() % 2 || value.size() > 131072) throw std::invalid_argument("invalid hex input length");
  io::rmv::Bytes result;
  for (std::size_t i = 0; i < value.size(); i += 2) {
    const auto a = digit(value[i]), b = digit(value[i + 1]);
    if (a < 0 || b < 0) throw std::invalid_argument("non-hex input");
    result.push_back(static_cast<std::uint8_t>(a * 16 + b));
  }
  return result;
}
const char* completion(io::rmv::Completion value) {
  using io::rmv::Completion;
  switch (value) {
    case Completion::accepted: return "accepted";
    case Completion::rejected: return "rejected";
    case Completion::timeout: return "timeout";
    case Completion::disconnected: return "disconnected";
    case Completion::protocol_error: return "protocol_error";
  }
  return "unknown";
}
}
int main(int argc, char** argv) {
  if (argc != 2 || std::string(argv[1]) != "--stdio-offline") {
    std::cerr << "Only --stdio-offline is supported. No serial devices are opened.\n";
    return 2;
  }
  try {
    io::rmv::Session session;
    std::uint64_t nonce = 0x524d560000000000ULL;
    bool first = true;
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.size() > 131200) throw std::invalid_argument("input line too large");
      std::istringstream input(line);
      std::string stamp, action, raw, extra;
      if (!(input >> stamp >> action >> raw) || (input >> extra) ||
          stamp.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("expected: host_now_us action hex-or-dash");
      const auto now = std::stoull(stamp);
      if (first) {
        if (action != "none" || raw != "-") throw std::invalid_argument("first line must be: time none -");
        if (!session.begin(++nonce, now)) throw std::runtime_error("initial handshake did not start");
        first = false;
      }
      session.ingest(decode(raw), now);
      bool accepted = true;
      if (action == "mode") accepted = session.request_mode(3, 3, now);
      else if (action == "aim") accepted = session.aim(.1f, .05f, now, 2);
      else if (action == "stop") accepted = session.stop(now);
      else if (action == "disconnect") session.disconnect();
      else if (action == "connect") accepted = session.begin(++nonce, now);
      else if (action == "retry") accepted = session.retry(now);
      else if (action == "sync") accepted = session.request_time_sync(now);
      else if (action != "none") throw std::invalid_argument("unknown offline action");
      session.tick(now);
      const auto outgoing = session.take_outgoing(now);
      const auto& result = session.last_result();
      const auto& status = session.telemetry().status;
      std::cout << std::boolalpha << "{\"active\":" << session.active()
                << ",\"session\":" << session.session_id()
                << ",\"ready\":" << session.control_ready(now)
                << ",\"mode\":" << (status ? static_cast<int>(status->mode) : -1)
                << ",\"result\":" << (result ? static_cast<int>(result->result) : -1)
                << ",\"completion\":\"" << (result ? completion(result->completion) : "none")
                << "\",\"action_accepted\":" << accepted
                << ",\"pending\":" << session.pending() << ",\"wires\":[";
      for (std::size_t i = 0; i < outgoing.size(); ++i) {
        if (i) std::cout << ',';
        std::cout << '"' << std::hex << std::setfill('0');
        for (auto byte : outgoing[i]) std::cout << std::setw(2) << static_cast<unsigned>(byte);
        std::cout << '"' << std::dec;
      }
      std::cout << "]}" << std::endl;
    }
    session.disconnect();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Offline peer error: " << error.what() << '\n'; return 2;
  }
}

#pragma once
// Reuse the repository's cross-platform serial driver instead of introducing a
// second termios/Win32 implementation. Construction never opens a device.
#include "io/rmv/transport.hpp"
#include <serial/serial.h>
#include <string>

namespace io::rmv {
class SerialTransport {
  serial::Serial port_;
  FrameTransport<serial::Serial> framing_{port_};
public:
  SerialTransport() = default;
  SerialTransport(const SerialTransport&) = delete;
  SerialTransport& operator=(const SerialTransport&) = delete;
  void open(const std::string& path, std::uint32_t baud = 460800) {
    if (path.empty() || !baud) throw std::invalid_argument("explicit serial path/baud required");
    if (port_.isOpen()) throw std::logic_error("close the old port and invalidate the session first");
    port_.setPort(path);
    port_.setBaudrate(baud);
    port_.setBytesize(serial::eightbits);
    port_.setParity(serial::parity_none);
    port_.setStopbits(serial::stopbits_one);
    port_.setFlowcontrol(serial::flowcontrol_none);
    // Each read/write is bounded, including a device unplug during the call.
    auto timeout = serial::Timeout::simpleTimeout(5);
    port_.setTimeout(timeout);
    try {
      port_.open();
      port_.flushInput();
    } catch (...) {
      try { port_.close(); } catch (...) {}
      throw;
    }
  }
  bool is_open() const { return port_.isOpen(); }
  void close() { port_.close(); }
  void send(const Bytes& bytes) { framing_.send(bytes); }
  Bytes receive() { return framing_.receive(); }
};
} // namespace io::rmv

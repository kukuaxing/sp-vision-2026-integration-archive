#pragma once
// One owner/thread per transport. No background reconnection or command replay.
#include "io/rmv/core.hpp"
#include <algorithm>
#include <stdexcept>

namespace io::rmv {
// Port is the existing serial::Serial API, or an offline test double.
// A short write may already have reached the device. Close immediately instead
// of appending another frame to an unknown partial frame. The caller must also
// invalidate its Session and establish a new session after reopening the port.
template <class Port> class FrameTransport {
  Port& port_;
public:
  explicit FrameTransport(Port& port) : port_(port) {}
  void send(const Bytes& bytes) {
    if (bytes.empty()) return;
    if (bytes.size() > HEADER_SIZE + MAX_PAYLOAD + 2)
      throw std::invalid_argument("send one RMV frame at a time");
    try {
      if (!port_.isOpen() || port_.write(bytes) != bytes.size())
        throw std::runtime_error("RMV serial write incomplete; session must be invalidated");
    } catch (...) {
      try { port_.close(); } catch (...) {}
      throw;
    }
  }
  Bytes receive() {
    try {
      if (!port_.isOpen()) throw std::runtime_error("RMV serial port is closed");
      // Bound work and allocation per pump, even if a noisy device floods us.
      const auto count = std::min<std::size_t>(port_.available(), 4096);
      Bytes bytes;
      if (count) port_.read(bytes, count);
      return bytes;
    } catch (...) {
      try { port_.close(); } catch (...) {}
      throw;
    }
  }
};
} // namespace io::rmv

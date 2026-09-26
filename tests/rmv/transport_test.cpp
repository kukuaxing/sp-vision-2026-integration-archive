#include "io/rmv/transport.hpp"
#include <cassert>
#include <iostream>

struct Port {
  bool open = true, fail_read = false;
  std::size_t write_limit = 1000, available_count = 0, read_count = 0;
  io::rmv::Bytes written;
  bool isOpen() const { return open; }
  void close() { open = false; }
  std::size_t available() const { return available_count; }
  std::size_t write(const io::rmv::Bytes& bytes) {
    const auto n = std::min(write_limit, bytes.size());
    written.insert(written.end(), bytes.begin(), bytes.begin() + n); return n;
  }
  void read(io::rmv::Bytes& bytes, std::size_t count) {
    if (fail_read) throw std::runtime_error("unplugged");
    read_count = count; bytes.resize(count);
  }
};
int main() {
  using namespace io::rmv;
  Port port;
  FrameTransport<Port> transport(port);
  const auto wire = encode(Frame{});
  transport.send(wire);
  assert(port.written == wire);
  assert(transport.receive().empty());
  port.available_count = 100000;
  assert(transport.receive().size() == 4096 && port.read_count == 4096);
  port.write_limit = 4;
  bool rejected = false;
  try { transport.send(wire); } catch (const std::runtime_error&) { rejected = true; }
  assert(rejected && !port.open);
  const auto size = port.written.size();
  try { transport.send(wire); } catch (const std::runtime_error&) {}
  assert(port.written.size() == size); // no automatic re-send on closed port
  port.open = true; port.fail_read = true;
  rejected = false;
  try { transport.receive(); } catch (const std::runtime_error&) { rejected = true; }
  assert(rejected && !port.open);
  std::cout << "rmv_transport_test: PASS\n";
}

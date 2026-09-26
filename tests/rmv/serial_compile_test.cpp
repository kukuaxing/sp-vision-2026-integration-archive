#include "io/rmv/serial.hpp"
#include <cassert>
#include <iostream>
int main() {
  io::rmv::SerialTransport transport;
  assert(!transport.is_open());
  bool rejected = false;
  try { transport.open(""); } catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected && !transport.is_open());
  std::cout << "rmv_serial_compile_test: PASS (no device opened)\n";
}

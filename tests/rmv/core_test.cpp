#include "io/rmv/core.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

using namespace io::rmv;
int main(int argc, char** argv) {
  const std::string check = "123456789";
  assert(crc16(reinterpret_cast<const std::uint8_t*>(check.data()), check.size()) == 0x29b1);
  assert(newer(1, 0xffffffffu) && !newer(1, 1) && !newer(0x80000000u, 0));
  Frame f{0x0102, 0, 1, 3, 1000, Bytes{1, 2, 3}};
  auto raw = encode(f);
  for (std::size_t split = 0; split <= raw.size(); ++split) {
    Decoder d;
    auto a = d.feed(Bytes(raw.begin(), raw.begin() + split), 0);
    auto b = d.feed(Bytes(raw.begin() + split, raw.end()), 1000);
    a.insert(a.end(), b.begin(), b.end());
    assert(a.size() == 1 && encode(a.front()) == raw);
  }
  auto bad = raw; bad.back() ^= 1; bad.insert(bad.end(), raw.begin(), raw.end());
  Decoder d;
  assert(d.feed(bad, 0).size() == 1 && d.crc_errors == 1);
  auto stalled = raw; stalled[8] = 0; stalled[9] = 1; stalled.resize(28);
  stalled.insert(stalled.end(), raw.begin(), raw.end());
  Decoder timed;
  assert(timed.feed(stalled, 0).empty());
  assert(timed.feed({}, 50001).size() == 1 && timed.timeouts == 1);
  ClockSync sync;
  for (std::uint64_t i = 0; i < 8; ++i)
    assert(sync.observe(i*10000, i*10000+4001000, i*10000+4001000, i*10000+2000));
  assert(sync.to_mcu(80000, 80000) == 4080000);
  assert(!sync.offset(4000000));
  PoseHistory history;
  assert(history.add({100000, 1, 1, {1,0,0,0}}, 100000));
  assert(history.add({110000, 2, 1, {-1,0,0,0}}, 110000));
  assert(std::abs(history.at(105000, 115000)->at(0)) == 1);
  assert(!history.at(99000, 115000) && !history.at(111000, 115000));
  assert(!history.at(105000, 211000));
  assert(!history.add({120000, 2, 1, {1,0,0,0}}, 120000));
  assert(history.add({130000, 1, 2, {1,0,0,0}}, 130000) && history.size() == 1);
  assert(!history.add({120000, 3, 1, {1,0,0,0}}, 135000));
  assert(!history.add({140000, 2, 2, {0,0,0,0}}, 140000));
  TelemetryAdapter adapter;
  adapter.establish(7, 1);
  for (std::uint64_t i = 0; i < 8; ++i)
    assert(adapter.clock.observe(i*10000, i*10000+1000, i*10000+1000, i*10000+2000));
  Frame status{0x0101, 0, 1, 7, 100000, Bytes(32, 0)};
  status.payload[0] = 3; status.payload[1] = 2; status.payload[6] = 1;
  assert(adapter.consume(status, 100000));
  assert(!adapter.control_ready(100000)); // Permission alone is insufficient.
  Frame pose{0x0102, 0, 2, 7, 100000, {}};
  append(pose.payload, 0, 1); append(pose.payload, 0, 1); append(pose.payload, 9, 2);
  append(pose.payload, 1, 4); append(pose.payload, 100000, 8);
  append(pose.payload, 0, 8); append(pose.payload, 1, 4);
  append(pose.payload, 0x3f800000, 4); // float32 1.0
  pose.payload.resize(68, 0);
  assert(adapter.consume(pose, 100000));
  assert(adapter.control_ready(100000));
  assert(adapter.camera_pose_at(100000, 100000));
  assert(!adapter.consume(pose, 100000)); // Replayed packet.
  auto foreign = pose; foreign.seq = 3; foreign.session = 8;
  assert(!adapter.consume(foreign, 100000));
  status.seq = 4;
  assert(adapter.consume(status, 205000));
  assert(!adapter.control_ready(205000)); // Fresh STATUS cannot freshen old POSE.
  pose.seq = 5; pose.payload[2] = 0;
  assert(!adapter.consume(pose, 205000));
  assert(!adapter.epoch && adapter.poses.size() == 0);
  adapter.establish(8, 2);
  assert(!adapter.status && !adapter.clock.offset(205000));
  std::size_t golden = 0;
  if (argc == 2) {
    std::ifstream input(argv[1]); assert(input.good());
    std::string line;
    while (std::getline(input, line)) {
      Bytes bytes;
      std::istringstream row(line);
      unsigned value;
      while (row >> std::hex >> value) bytes.push_back(static_cast<std::uint8_t>(value));
      Decoder parser;
      auto frames = parser.feed(bytes, 0);
      assert(frames.size() == 1 && encode(frames[0]) == bytes);
      ++golden;
    }
    assert(golden >= 28);
  }
  std::cout << "PASS: C++ RMV framing, clock, pose history, telemetry permission; Python golden frames=" << golden << '\n';
}

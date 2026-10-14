// Compile against the unmodified rmv_core.h from the supplied firmware ZIP.
// Everything stays in memory: no UART, camera, motor or flash is accessed.
#include "calibration/rmv_pose.hpp"
#include "rmv_core.h"
#include <cassert>
#include <iostream>

namespace {
using namespace io::rmv;
struct Backend : rmv::Backend {
  Bytes replies;
  bool send(const uint8_t* p, size_t n) override {
    replies.insert(replies.end(), p, p + n); return true;
  }
  bool save(const float*, size_t) override { assert(false && "unexpected flash write"); return false; }
};
struct Fixture {
  Backend backend;
  rmv::Device board{backend};
  CalibrationRmvState host;
  uint64_t now = 1000000;
  static constexpr uint64_t offset = 5000000;
  Fixture(uint32_t caps = 0xc12) { board.init(99, 2026100402, caps); }
  void exchange(bool strip_hello_flag = false) {
    for (auto wire : host.take_outgoing(now)) {
      Decoder decoder; auto frames = decoder.feed(wire, now); assert(frames.size() == 1);
      auto frame = frames.front();
      assert(frame.type == 1 || frame.type == 2 || frame.type == 3);
      if (frame.type == 1) {
        assert(frame.flags == 1 && frame.session == 0);
        if (strip_hello_flag) { frame.flags = 0; wire = encode(frame); }
      }
      if (frame.type == 3) assert(frame.payload == Bytes({0, 0, 0, 0}));
      board.feed(wire.data(), wire.size(), now + offset + 100);
    }
    // Give the actual firmware a new stationary raw small-Y IMU sample.
    auto& pose = board.input.poses[1];
    pose.flags = 55; ++pose.counter; pose.time = now + offset + 100;
    pose.device = pose.counter * 10000ull; pose.acc[2] = 9.81f;
    board.telemetry(now + offset + 100);
    now += 200;
    host.ingest(backend.replies, now); backend.replies.clear();
    assert(board.mode == 0 && !board.target.valid && board.revision == 0);
  }
  void step() { now += 9800; host.tick(now); exchange(); }
};
}

int main() {
  Fixture good; assert(good.host.begin(42, good.now)); good.exchange();
  for (int i = 0; i < 150; ++i) good.step();
  auto sample = good.host.current_sample(good.now);
  assert(CalibrationRmvState::can_save(sample, good.now));
  assert(sample.count > 100 && sample.session_id == good.board.session && sample.boot_id == 99);
  assert(sample.firmware == 2026100402 && sample.geometry_revision == 0 && sample.flags == 55);
  assert(sample.q.w() == 1 && sample.error.empty());

  Fixture bad; assert(bad.host.begin(43, bad.now)); bad.exchange(true);
  assert(bad.board.session == 0 && bad.host.disconnected());
  const auto rejection = bad.host.sample().error;
  assert(rejection == "RMV HELLO rejected: BAD_VALUE (4), detail=0");
  bad.host.tick(bad.now); bad.host.take_outgoing(bad.now); bad.host.ingest({}, bad.now);
  assert(bad.host.current_sample(bad.now).error == rejection);

  Fixture unsupported(0x12); assert(unsupported.host.begin(44, unsupported.now)); unsupported.exchange();
  assert(unsupported.host.disconnected() && unsupported.board.session == 0);
  assert(unsupported.host.sample().error == "RMV HELLO rejected: UNSUPPORTED (1), detail=0");
  std::cout << "PASS supplied firmware: HELLO ACK_REQUEST, stationary sensor1, BAD_VALUE/UNSUPPORTED diagnostics; diagnostic commands only\n";
}

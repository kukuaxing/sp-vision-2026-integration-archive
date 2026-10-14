#include "calibration/rmv_pose.hpp"
#include <cassert>
#include <iostream>

using namespace io::rmv;
namespace {
void put_float(Bytes& p, float value) {
  uint32_t bits; std::memcpy(&bits, &value, 4); append(p, bits, 4);
}
struct Fixture {
  CalibrationRmvState state;
  uint64_t now = 1000000, offset = 5000000;
  uint32_t seq = 0, counter = 0, epoch = 1, geometry = 0;
  uint32_t caps = 0xc12;
  Fixture() { assert(state.begin(42, now)); pump(); }
  void send(uint16_t type, Bytes p, uint16_t flags = 0, uint32_t session = 7) {
    state.ingest(encode({type, flags, ++seq, session, now + offset, std::move(p)}), now);
  }
  void pump() {
    for (auto bytes : state.take_outgoing(now)) {
      Decoder decoder; const auto frames = decoder.feed(bytes, now); assert(frames.size() == 1);
      const auto& frame = frames.front();
      assert(frame.type == 1 || frame.type == 2 || frame.type == 3);
      if (frame.type == 1) {
        assert(frame.flags == 1 && frame.session == 0);
        assert(integer(frame.payload, 8, 4) == (1u << 10));
        Bytes p; append(p, 42, 8); append(p, 99, 8); append(p, 2026100402, 4);
        append(p, caps, 4); append(p, 0, 4); append(p, geometry, 4);
        append(p, 256, 2); p.insert(p.end(), {2, 3}); send(0x8001, p, 2);
      } else if (frame.type == 2) {
        Bytes p = frame.payload; append(p, now + offset, 8); append(p, now + offset, 8);
        send(0x8002, p, 2);
      } else assert(frame.payload == Bytes({0, 0, 0, 0}));
    }
  }
  void status() {
    Bytes p{0, 1, 7, 7, 0, 1, 0, 0}; p.resize(24, 0);
    append(p, 0, 4); append(p, geometry, 4); send(0x0101, p);
  }
  void pose(uint16_t flags = 55, float gyro = 0, float angle = 0,
            uint8_t sensor = 1, int64_t time_shift = 0, bool repeat = false,
            uint32_t session = 7, float qw_override = 2) {
    if (!repeat) ++counter;
    const uint64_t stamp = static_cast<uint64_t>(now + offset + time_shift);
    Bytes p{sensor, sensor}; append(p, flags, 2); append(p, counter, 4);
    append(p, stamp, 8); append(p, flags & 16 ? now : 0, 8); append(p, epoch, 4);
    put_float(p, qw_override == 2 ? std::cos(angle / 2) : qw_override);
    put_float(p, 0); put_float(p, 0); put_float(p, std::sin(angle / 2));
    put_float(p, gyro); put_float(p, 0); put_float(p, 0);
    put_float(p, 0); put_float(p, 0); put_float(p, 9.81f);
    send(0x0102, p, 0, session);
  }
  void step() { now += 10000; status(); state.tick(now); pump(); }
  void settle(int count = 75) { for (int i = 0; i < count; ++i) { step(); pose(); } }
};
}
int main() {
  Fixture quiet; quiet.settle();
  assert(CalibrationRmvState::can_save(quiet.state.sample(), quiet.now));
  auto image = quiet.state.for_image(quiet.now - 10000, quiet.now);
  assert(image.valid && image.source_counter == quiet.counter - 1);
  assert(image.received_host_us == quiet.now - 10000);
  assert(image.q.w() == 1 && image.flags == 55 && image.boot_id == 99 && image.epoch == 1);
  assert(!quiet.state.for_image(quiet.now + 1, quiet.now).valid);
  assert(!quiet.state.for_image(quiet.now - 600000, quiet.now).valid);
  quiet.step(); quiet.pose(55, .1f);
  assert(CalibrationRmvState::fresh(quiet.state.sample(), quiet.now));
  assert(!CalibrationRmvState::can_save(quiet.state.sample(), quiet.now));
  // Motion after exposure must not revoke an already settled image.
  assert(quiet.state.for_image(quiet.now - 10000, quiet.now).valid);
  assert(!quiet.state.for_image(quiet.now, quiet.now).valid);
  assert(quiet.state.for_image(quiet.now, quiet.now).error.find("5 deg/s") != std::string::npos);

  Fixture spike; spike.settle(); spike.step(); spike.pose(55, float(3 * M_PI / 180));
  assert(CalibrationRmvState::can_save(spike.state.sample(), spike.now));
  assert(spike.state.for_image(spike.now, spike.now).valid);
  assert(spike.state.sample().gyro_deg_s > 2.9 && spike.state.sample().gyro_rms_deg_s < 2);
  Fixture sustained; sustained.settle();
  for (int i = 0; i < 12; ++i) { sustained.step(); sustained.pose(55, float(3 * M_PI / 180)); }
  assert(!sustained.state.for_image(sustained.now, sustained.now).valid);
  assert(sustained.state.for_image(sustained.now, sustained.now).error.find("RMS") != std::string::npos);
  Fixture noisy; noisy.settle();
  for (int i = 0; i < 100; ++i) {
    noisy.step(); noisy.pose(55, float((i % 15 == 0 ? 2.85 : 1.2) * M_PI / 180));
    assert(CalibrationRmvState::can_save(noisy.state.sample(), noisy.now));
  }

  Fixture mapped; mapped.settle(); mapped.step(); mapped.pose(31);
  assert(CalibrationRmvState::can_save(mapped.state.sample(), mapped.now));
  for (auto flags : {uint16_t(54), uint16_t(53), uint16_t(23), uint16_t(63), uint16_t(0x80 | 55)}) {
    Fixture bad; bad.settle(); bad.step(); bad.pose(flags);
    assert(!CalibrationRmvState::fresh(bad.state.sample(), bad.now));
    assert(!bad.state.for_image(bad.now - 10000, bad.now).valid);
    assert(bad.state.for_image(bad.now, bad.now).error.find("flags") != std::string::npos);
  }
  for (auto shift : {-100000, 6000}) {
    Fixture bad; bad.settle(); bad.step(); bad.pose(55, 0, 0, 1, shift);
    assert(!CalibrationRmvState::fresh(bad.state.sample(), bad.now));
  }
  for (auto q : {0.f, .8f, std::numeric_limits<float>::quiet_NaN()}) {
    Fixture bad; bad.settle(); bad.step(); bad.pose(55, 0, 0, 1, 0, false, 7, q);
    assert(!CalibrationRmvState::fresh(bad.state.sample(), bad.now));
  }
  Fixture repeated; repeated.settle(); const auto previous = repeated.state.sample().count;
  for (int i = 0; i < 12; ++i) { repeated.step(); repeated.pose(55, 0, 0, 1, 0, true); }
  assert(repeated.state.sample().count == previous);
  assert(!CalibrationRmvState::fresh(repeated.state.sample(), repeated.now));
  Fixture wrong; wrong.settle(); const auto accepted = wrong.state.sample().count;
  for (int i = 0; i < 12; ++i) { wrong.step(); wrong.pose(55, 0, 0, 2); wrong.pose(55, 0, 0, 1, 0, false, 8); }
  assert(wrong.state.sample().count == accepted);
  assert(!CalibrationRmvState::fresh(wrong.state.sample(), wrong.now));

  Fixture changed; changed.settle(); changed.epoch++; changed.step(); changed.pose();
  assert(changed.state.disconnected() && !changed.state.sample().valid);
  assert(changed.state.sample().error.find("epoch") != std::string::npos);
  Fixture geometry; geometry.settle(); geometry.geometry++; geometry.step(); geometry.pose();
  assert(geometry.state.disconnected() && !geometry.state.sample().valid);
  Fixture lost; lost.settle(); lost.now += 500001; lost.state.tick(lost.now);
  assert(lost.state.disconnected() && !lost.state.sample().valid);
  lost.pose(); assert(!lost.state.sample().valid); // No automatic reconnection/resurrection.
  Fixture drift; drift.settle();
  for (int i = 1; i <= 70; ++i) { drift.step(); drift.pose(55, 0, i * .004f); }
  assert(!CalibrationRmvState::can_save(drift.state.sample(), drift.now));
  Fixture wrap; wrap.counter = 0xfffffff0u; wrap.settle();
  assert(CalibrationRmvState::can_save(wrap.state.sample(), wrap.now));
  Fixture clock; clock.settle(); bool clock_expired_before_pose = false;
  for (int i = 0; i < 400; ++i) {
    clock.now += 10000; clock.status(); clock.state.tick(clock.now);
    clock.state.take_outgoing(clock.now); // Drop time replies, keep diagnostic heartbeats flowing.
    clock.pose();
    if (CalibrationRmvState::fresh(clock.state.sample(), clock.now + 90000) &&
        !clock.state.current_sample(clock.now + 90000).valid) {
      clock_expired_before_pose = true;
      assert(!clock.state.for_image(clock.now, clock.now + 90000).valid);
      break;
    }
  }
  assert(clock_expired_before_pose);
  std::cout << "PASS: raw RMV calibration, clocks, flags, source identity, repeats, stationary images, epoch/geometry discontinuities and terminal link loss\n";
}

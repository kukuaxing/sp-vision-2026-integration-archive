#include "io/rmv/session.hpp"
#include <cassert>
#include <iostream>

using namespace io::rmv;
namespace {
SessionOptions manual_options() { SessionOptions options; options.automatic_sync = false; return options; }
struct Fixture {
  Session session{manual_options()};
  std::uint64_t now = 1000000, offset = 4000000;
  std::uint32_t wire_seq = 0, source = 0;
  Frame outgoing() {
    auto packets = session.take_outgoing(now); assert(packets.size() == 1);
    Decoder decoder; auto frames = decoder.feed(packets[0], now); assert(frames.size() == 1);
    return frames[0];
  }
  void receive(std::uint16_t type, Bytes payload, std::uint16_t flags = 0,
               std::uint32_t sid = 0, std::optional<std::uint64_t> stamp = {}) {
    session.ingest(encode(Frame{type, flags, ++wire_seq, sid ? sid : session.session_id(),
                               stamp ? *stamp : now + offset, std::move(payload)}), now);
  }
  Frame begin(std::uint64_t nonce = 1234) {
    assert(session.begin(nonce, now)); auto request = outgoing();
    assert(request.type == 1 && request.session == 0 && integer(request.payload, 0, 8) == nonce);
    return request;
  }
  void hello(std::uint64_t nonce = 1234, std::uint64_t boot = 1, std::uint32_t sid = 10,
             std::uint32_t caps = 0x1f) {
    Bytes p; append(p, nonce, 8); append(p, boot, 8); append(p, 42, 4); append(p, caps, 4);
    append(p, 3, 4); append(p, 7, 4); append(p, 256, 2); p.push_back(1); p.push_back(2);
    receive(0x8001, p, 2, sid);
  }
  void connect() { begin(); now += 2000; hello(); assert(session.active()); }
  void time_reply(const Frame& request, std::uint64_t extra_delay = 2000) {
    const auto t1 = integer(request.payload, 0, 8);
    Bytes p; append(p, t1, 8); append(p, t1 + offset + 1000, 8);
    append(p, t1 + offset + 1000, 8); now += extra_delay; receive(0x8002, p, 2);
  }
  void sync() {
    for (unsigned i = 0; i < 8; ++i) {
      assert(session.request_time_sync(now)); auto request = outgoing(); time_reply(request);
    }
    assert(session.telemetry().clock.offset(now));
  }
  void pose(std::uint32_t epoch = 1, std::uint16_t valid = 9) {
    Bytes p{0, 0}; append(p, valid, 2); append(p, ++source, 4);
    append(p, now + offset, 8); append(p, 0, 8); append(p, epoch, 4);
    append(p, 0x3f800000, 4); for (unsigned i = 0; i < 9; ++i) append(p, 0, 4);
    receive(0x0102, p);
  }
  void status(std::uint8_t mode = 3, bool permitted = true,
              std::optional<std::uint64_t> stamp = {}) {
    Bytes p{mode, 2, 3, 3, 3, 1, static_cast<std::uint8_t>(permitted), 0};
    for (unsigned i = 0; i < 3; ++i) append(p, 0, 4);
    append(p, 0, 2); append(p, 0, 2); append(p, 3, 4); append(p, 7, 4);
    receive(0x0101, p, 0, 0, stamp);
  }
  void ack(const Frame& request, std::uint16_t code = 0, std::optional<std::uint32_t> sid = {}) {
    Bytes p; append(p, request.seq, 4); append(p, request.type, 2); append(p, code, 2);
    append(p, 0, 4); append(p, 3, 4);
    session.ingest(encode(Frame{0x00ff, 2, ++wire_seq, sid ? *sid : session.session_id(),
                               now + offset, p}), now);
  }
  void ready() {
    connect(); sync(); pose(); status(0, false);
    assert(session.request_mode(3, 3, now)); auto request = outgoing(); ack(request);
    assert(!session.control_ready(now)); status(); assert(session.control_ready(now));
  }
};

void handshake_tests() {
  Fixture f; f.begin(); f.hello(999); assert(!f.session.active() && f.session.pending());
  f.hello(); assert(f.session.active() && f.session.boot_id() == 1 && f.session.session_id() == 10);
  assert(f.session.capabilities() == 0x1f && f.session.parameter_revision() == 3);
  assert(f.session.geometry_revision() == 7 && f.session.firmware_version() == 42);
  f.session.disconnect(); assert(!f.session.begin(1234, f.now));
  f.begin(1235); f.hello(1235); assert(!f.session.active()); // Same boot/session was reused.
  assert(f.session.last_result()->completion == Completion::protocol_error);
  f.begin(1236); f.hello(1236, 2); assert(f.session.active()); // New boot may reuse numeric session.
  Fixture missing; missing.begin(); missing.hello(1234, 1, 1, 0xf); assert(!missing.session.active());
  Fixture future; future.begin(); future.hello(1234, 1, 1, 0x401f); assert(!future.session.active());
  Fixture refusal; auto req = refusal.begin(); refusal.ack(req, 10, 0);
  assert(!refusal.session.pending() && !refusal.session.active());
  assert(refusal.session.last_result()->result == 10);
  Fixture ack_only; auto hello_req = ack_only.begin(); ack_only.ack(hello_req, 0, 0);
  assert(ack_only.session.pending());
}
void timing_tests() {
  Fixture f; f.connect(); assert(!f.session.aim(0, 0, f.now));
  assert(f.session.request_time_sync(f.now)); auto request = f.outgoing();
  Bytes wrong; append(wrong, f.now - 1, 8); append(wrong, f.now + f.offset, 8);
  append(wrong, f.now + f.offset, 8); f.receive(0x8002, wrong, 2); assert(f.session.pending());
  f.time_reply(request); assert(!f.session.pending());
  f.sync(); f.pose(); assert(f.session.telemetry().poses.size() == 1);
  f.now += 1000; assert(f.session.request_time_sync(f.now)); request = f.outgoing();
  f.now += 100000; f.session.tick(f.now); assert(f.session.request_timed_out());
  assert(!f.session.retry(f.now)); // TIME retries cannot bias the offset with an old t1.
  f.time_reply(request); assert(f.session.request_timed_out());
  assert(f.session.request_time_sync(f.now));
  Fixture slow; slow.connect();
  for (unsigned i = 0; i < 8; ++i) {
    assert(slow.session.request_time_sync(slow.now)); const auto time = slow.outgoing();
    slow.time_reply(time, 20000);
    if (i == 4) { assert(slow.session.heartbeat(slow.now)); slow.outgoing(); }
  }
  assert(!slow.session.telemetry().clock.offset(slow.now)); // 10 ms uncertainty rejected.
}
void control_tests() {
  Fixture f; f.ready();
  assert(f.session.aim(.1f, .2f, f.now, 2));
  assert(f.session.aim(.3f, .4f, f.now, 2)); // Only the latest unsent target survives.
  auto aim = f.outgoing(); assert(aim.type == 0x0201 && real(aim.payload, 24) == .3f);
  assert(integer(aim.payload, 12, 8) - integer(aim.payload, 4, 8) == 80000);
  assert(!f.session.retry(f.now));
  assert(!f.session.aim(std::numeric_limits<float>::quiet_NaN(), 0, f.now));
  assert(!f.session.aim(0, 0, f.now, 1)); assert(!f.session.aim(0, 0, f.now, 0, 100001));
  assert(f.session.aim(.3f, .4f, f.now, 2, 80000, false));
  aim = f.outgoing(); assert(integer(aim.payload, 22, 2) == 0 && real(aim.payload, 24) == 0);
  assert(f.session.aim(0, 0, f.now)); assert(f.session.stop(f.now));
  auto stop = f.outgoing(); assert(stop.type == 0x0202 && !f.session.control_ready(f.now));
  f.ack(stop); f.status(); assert(!f.session.aim(0, 0, f.now)); // STOP remains latched locally.
  assert(f.session.request_mode(2, 3, f.now)); const auto mode = f.outgoing();
  f.status(2); assert(!f.session.control_ready(f.now));
  f.ack(mode); assert(f.session.control_ready(f.now));
  assert(!f.session.aim(0, 0, f.now, 2)); // Joint targets only in calibration.
  f.now += 100001; f.status(2); assert(!f.session.aim(0, 0, f.now)); // Fresh STATUS, stale pose.
  f.pose(); assert(f.session.aim(0, 0, f.now)); f.outgoing();
  f.now += 1000; f.pose(2); assert(!f.session.control_ready(f.now)); // Epoch invalidates status.
  f.status(2); assert(!f.session.control_ready(f.now));
  assert(f.session.request_mode(2, 3, f.now)); const auto new_epoch_mode = f.outgoing();
  f.ack(new_epoch_mode); f.status(2); assert(f.session.control_ready(f.now));
  f.now += 1000; f.pose(2, 8); assert(!f.session.control_ready(f.now));
}
void retry_tests() {
  Fixture handshake; const auto hello = handshake.begin();
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    handshake.now += 100000; handshake.session.tick(handshake.now);
    assert(handshake.session.retry(handshake.now));
    assert(encode(handshake.outgoing()) == encode(hello));
  }
  handshake.now += 100000; handshake.session.tick(handshake.now);
  assert(!handshake.session.retry(handshake.now));
  handshake.hello(); assert(handshake.session.active()); // Late matching HELLO can still resolve uncertainty.
  Fixture f; f.ready();
  assert(f.session.request_mode(2, 3, f.now)); const auto mode = f.outgoing();
  f.ack(Frame{0x0202, 1, mode.seq, mode.session, mode.sender_time_us, {} });
  assert(f.session.pending()); // Matching seq alone is insufficient.
  f.ack(mode, 0, 999); assert(f.session.pending());
  f.now += 100000; f.session.tick(f.now); assert(f.session.request_timed_out());
  assert(f.session.retry(f.now));
  auto batch = f.session.take_outgoing(f.now); assert(batch.size() == 2); // Heartbeat plus retry.
  Decoder decoder; auto frames = decoder.feed(batch[1], f.now);
  assert(frames.size() == 1 && encode(frames[0]) == encode(mode));
  f.ack(mode); assert(!f.session.pending());
  assert(f.session.stop(f.now)); const auto stop = f.outgoing();
  f.ack(mode); assert(f.session.pending()); // An old mode ACK cannot complete STOP.
  f.ack(stop, 5, 0); assert(!f.session.active() && f.session.session_id() == 0);
  assert(f.session.telemetry().poses.size() == 0 && !f.session.retry(f.now));
  Fixture preempt; preempt.ready();
  assert(preempt.session.request_mode(2, 3, preempt.now)); const auto pending_mode = preempt.outgoing();
  assert(preempt.session.stop(preempt.now)); const auto pending_stop = preempt.outgoing();
  preempt.ack(pending_mode); assert(preempt.session.pending()); preempt.ack(pending_stop);
  preempt.status(2); assert(!preempt.session.control_ready(preempt.now));

  Fixture changed_frame; changed_frame.ready();
  assert(changed_frame.session.request_mode(2, 3, changed_frame.now));
  const auto old_context_mode = changed_frame.outgoing();
  changed_frame.now += 1000; changed_frame.pose(2); changed_frame.status(2);
  changed_frame.ack(old_context_mode);
  assert(!changed_frame.session.control_ready(changed_frame.now));
  assert(changed_frame.session.request_mode(2, 3, changed_frame.now));
  const auto new_context_mode = changed_frame.outgoing();
  changed_frame.ack(new_context_mode); changed_frame.status(2);
  assert(changed_frame.session.control_ready(changed_frame.now));
}
void expiry_tests() {
  Fixture queued; queued.begin(); queued.session.disconnect();
  assert(queued.session.begin(2000, queued.now));
  assert(queued.session.take_outgoing(queued.now + 5001).empty());
  assert(queued.session.state() == SessionState::disconnected);
  Fixture silent; silent.ready(); silent.now += 500000;
  silent.status(); assert(!silent.session.active()); // Late valid STATUS cannot revive a session.
  Fixture rollback; rollback.ready(); rollback.session.tick(rollback.now - 1);
  assert(!rollback.session.active());
  Fixture stale; stale.ready(); stale.now += 100001;
  stale.pose(); stale.status(3, true, stale.now + stale.offset - 200000);
  assert(!stale.session.control_ready(stale.now));
  Fixture other_session; other_session.ready();
  other_session.receive(0x0101, Bytes(32, 0), 0, 999);
  assert(other_session.session.control_ready(other_session.now));
  Fixture stalled_sender; stalled_sender.ready();
  for (int i = 0; i < 4; ++i) { stalled_sender.now += 100000; stalled_sender.pose(); stalled_sender.status(); }
  stalled_sender.now += 100000; stalled_sender.pose(); assert(!stalled_sender.session.active());
}
void parser_tests() {
  Fixture f; f.begin();
  Bytes p; append(p, 1234, 8); append(p, 1, 8); append(p, 1, 4); append(p, 0x1f, 4);
  append(p, 1, 4); append(p, 1, 4); append(p, 256, 2); p.push_back(1); p.push_back(2);
  auto wire = encode(Frame{0x8001, 2, 1, 10, f.now + f.offset, p});
  auto corrupt = wire; corrupt.back() ^= 1; f.session.ingest(corrupt, f.now);
  assert(!f.session.active());
  for (auto byte : wire) f.session.ingest(Bytes{byte}, f.now);
  assert(f.session.active());
}
void automatic_tests() {
  Fixture f; f.session = Session{}; f.connect();
  for (int i = 0; i < 8; ++i) {
    f.session.tick(f.now); auto req = f.outgoing(); assert(req.type == 2); f.time_reply(req);
  }
  assert(f.session.telemetry().clock.offset(f.now));
  for (int i = 0; i < 160; ++i) {
    f.now += 50000; f.pose(); f.status(0, false); f.session.tick(f.now);
    for (const auto& raw : f.session.take_outgoing(f.now)) {
      Decoder decoder; const auto frames = decoder.feed(raw, f.now); assert(frames.size() == 1);
      assert(frames[0].type == 2 || frames[0].type == 3);
      if (frames[0].type == 2) f.time_reply(frames[0]);
    }
    assert(f.session.active() && f.session.telemetry().clock.offset(f.now));
    assert(!f.session.control_ready(f.now)); // Maintenance never requests mode/target.
  }
}
}
int main() {
  handshake_tests(); timing_tests(); control_tests(); retry_tests(); expiry_tests();
  parser_tests(); automatic_tests();
  std::cout << "PASS: RMV active-session correlation, retries, timeouts, lifecycle and automatic maintenance\n";
}

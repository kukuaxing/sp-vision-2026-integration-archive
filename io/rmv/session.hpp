#pragma once
// RMV 1.0 event-driven host session. No serial port, worker thread or automatic mode change.
#include "core.hpp"

namespace io::rmv {

enum class SessionState { disconnected, connecting, active };
enum class Completion { accepted, rejected, timeout, disconnected, protocol_error };
struct RequestResult {
  std::uint32_t seq = 0;
  std::uint16_t type = 0, result = 0;
  std::uint32_t detail = 0, revision = 0;
  Completion completion = Completion::accepted;
};
struct SessionOptions {
  bool automatic_sync = true;
  std::uint64_t request_timeout_us = 100000;
  std::uint64_t link_timeout_us = 500000;
  std::uint64_t maximum_queue_age_us = 5000;
};

// All methods must run on one thread (or under the same caller-owned mutex).
// take_outgoing() transfers ownership: write the bytes immediately, in order. If
// transport cannot do so, disconnect(); never replay a partially written batch.
class Session {
  struct Pending {
    Frame frame;
    std::optional<std::uint64_t> sent;
    bool queued = true, timed_out = false;
    unsigned retries = 0;
    std::uint32_t geometry_revision = 0;
  };
  struct Queued { Frame frame; std::uint64_t queued_us; };
  SessionOptions options_;
  SessionState state_ = SessionState::disconnected;
  Decoder decoder_;
  TelemetryAdapter telemetry_;
  std::optional<Pending> pending_;
  std::optional<RequestResult> result_;
  std::deque<Queued> outgoing_;
  std::deque<std::uint64_t> used_nonces_;
  std::deque<std::pair<std::uint64_t, std::uint32_t>> used_sessions_;
  std::optional<std::uint64_t> last_now_, last_sync_sent_;
  std::uint64_t nonce_ = 0, boot_ = 0, last_rx_ = 0, last_heartbeat_ = 0;
  std::uint32_t seq_ = 0, session_ = 0, capabilities_ = 0, required_caps_ = 0;
  std::uint32_t parameter_revision_ = 0, geometry_revision_ = 0, firmware_ = 0;
  unsigned sync_samples_ = 0;
  bool control_blocked_ = true;

  void reset_session() {
    state_ = SessionState::disconnected; session_ = 0; boot_ = 0;
    capabilities_ = 0; parameter_revision_ = 0; geometry_revision_ = 0; firmware_ = 0;
    telemetry_ = TelemetryAdapter{}; decoder_ = Decoder{};
    pending_.reset(); outgoing_.clear(); last_sync_sent_.reset(); sync_samples_ = 0;
    control_blocked_ = true;
  }
  void fail(Completion reason) {
    RequestResult result;
    if (pending_) { result.seq = pending_->frame.seq; result.type = pending_->frame.type; }
    result.completion = reason;
    reset_session(); result_ = result;
  }
  bool observe_now(std::uint64_t now) {
    if (last_now_ && now < *last_now_) { fail(Completion::protocol_error); return false; }
    last_now_ = now; return true;
  }
  Frame frame(std::uint16_t type, Bytes payload, std::uint64_t now, std::uint16_t flags = 0) {
    if (++seq_ == 0) ++seq_;
    return Frame{type, flags, seq_, session_, now, std::move(payload)};
  }
  bool queue(Frame value, std::uint64_t now) {
    // Latest unsent setpoint and heartbeat replace older unsent instances.
    if (value.type == 0x0201 || value.type == 3) {
      outgoing_.erase(std::remove_if(outgoing_.begin(), outgoing_.end(),
        [&](const Queued& old) { return old.frame.type == value.type; }), outgoing_.end());
    }
    if (outgoing_.size() >= 8) { fail(Completion::protocol_error); return false; }
    outgoing_.push_back({std::move(value), now}); return true;
  }
  bool request(std::uint16_t type, Bytes payload, std::uint64_t now, bool ack) {
    if (pending_) return false;
    auto value = frame(type, std::move(payload), now, ack ? 1 : 0);
    pending_ = Pending{value, {}, true, false, 0, geometry_revision_};
    return queue(std::move(value), now);
  }
  bool fresh_pose(std::uint64_t now) const {
    const auto mcu = telemetry_.clock.to_mcu(now, now);
    return mcu && telemetry_.epoch && telemetry_.poses.fresh(*mcu);
  }
  void completed(const Frame& reply, Completion completion, std::uint16_t code = 0,
                 std::uint32_t detail = 0, std::uint32_t revision = 0) {
    const auto& request = pending_->frame;
    result_ = RequestResult{request.seq, request.type, code, detail, revision, completion};
    if (request.type == 0x0200 && completion == Completion::accepted)
      control_blocked_ = request.payload[0] == 0 || !telemetry_.epoch ||
                         integer(request.payload, 4, 4) != *telemetry_.epoch ||
                         pending_->geometry_revision != geometry_revision_;
    pending_.reset();
    (void)reply;
  }
  void consume(const Frame& reply, std::uint64_t now) {
    if (reply.flags == 2 && reply.type == 0x00ff && reply.payload.size() == 16 &&
        pending_ && pending_->sent && !pending_->queued) {
      const auto& p = reply.payload;
      if (integer(p, 0, 4) != pending_->frame.seq || integer(p, 4, 2) != pending_->frame.type)
        return;
      const auto code = static_cast<std::uint16_t>(integer(p, 6, 2));
      if (code > 13 || (reply.session != session_ && !(code == 5 && reply.session == 0))) return;
      // Dedicated replies, never a success ACK, complete HELLO/TIME.
      if (!code && (pending_->frame.type == 1 || pending_->frame.type == 2)) return;
      const auto detail = static_cast<std::uint32_t>(integer(p, 8, 4));
      const auto revision = static_cast<std::uint32_t>(integer(p, 12, 4));
      completed(reply, code ? Completion::rejected : Completion::accepted, code, detail, revision);
      last_rx_ = now; parameter_revision_ = revision;
      if (code == 5 || state_ == SessionState::connecting) {
        const auto result = result_; reset_session(); result_ = result;
      }
      return;
    }
    if (state_ == SessionState::connecting) {
      if (!pending_ || !pending_->sent || pending_->queued || reply.type != 0x8001 ||
          reply.flags != 2 || reply.payload.size() != 36 || !reply.session) return;
      const auto& p = reply.payload;
      const auto caps = static_cast<std::uint32_t>(integer(p, 20, 4));
      const auto boot = integer(p, 8, 8);
      if (integer(p, 0, 8) != nonce_) return;
      if ((caps & required_caps_) != required_caps_ || (caps & ~0x3fffu) ||
          integer(p, 32, 2) != MAX_PAYLOAD || ((caps & 1) && p[34] == 0) ||
          p[35] < 2 || p[35] > 3 ||
          std::find(used_sessions_.begin(), used_sessions_.end(),
                    std::make_pair(boot, reply.session)) != used_sessions_.end()) {
        fail(Completion::protocol_error); return;
      }
      session_ = reply.session; boot_ = boot; capabilities_ = caps;
      firmware_ = static_cast<std::uint32_t>(integer(p, 16, 4));
      parameter_revision_ = static_cast<std::uint32_t>(integer(p, 24, 4));
      geometry_revision_ = static_cast<std::uint32_t>(integer(p, 28, 4));
      used_sessions_.push_back({boot_, session_});
      while (used_sessions_.size() > 64) used_sessions_.pop_front();
      telemetry_.establish(session_, boot_); state_ = SessionState::active;
      last_rx_ = now; last_heartbeat_ = now;
      completed(reply, Completion::accepted); return;
    }
    if (!active() || reply.session != session_) return;
    if (reply.type == 0x8002 && reply.flags == 2 && reply.payload.size() == 24 &&
        pending_ && pending_->frame.type == 2 && pending_->sent && !pending_->queued) {
      const auto& p = reply.payload;
      const auto t1 = integer(p, 0, 8);
      if (t1 != integer(pending_->frame.payload, 0, 8)) return;
      // A timeout makes this timing sample unusable even if the response arrives later.
      if (pending_->timed_out || now - *pending_->sent >= options_.request_timeout_us) return;
      if (!telemetry_.clock.observe(t1, integer(p, 8, 8), integer(p, 16, 8), now)) return;
      sync_samples_ = std::min(sync_samples_ + 1, 8u); last_rx_ = now;
      completed(reply, Completion::accepted); return;
    }
    if (reply.flags) return;
    if (auto mapped = telemetry_.clock.to_mcu(now, now)) {
      const auto age = double(*mapped) - double(reply.sender_time_us);
      if (age < -5000 || age > 100000) return;
    }
    const auto old_epoch = telemetry_.epoch;
    if (telemetry_.consume(reply, now)) {
      last_rx_ = now;
      if (old_epoch && telemetry_.epoch && old_epoch != telemetry_.epoch) control_blocked_ = true;
      if (telemetry_.status) {
        parameter_revision_ = telemetry_.status->parameter_revision;
        if (geometry_revision_ != telemetry_.status->geometry_revision) {
          control_blocked_ = true;
          telemetry_.poses.clear(); telemetry_.epoch.reset();
        }
        geometry_revision_ = telemetry_.status->geometry_revision;
      }
    }
  }
public:
  explicit Session(SessionOptions options = {}) : options_(options) {
    if (!options.request_timeout_us || options.request_timeout_us >= 500000 ||
        !options.link_timeout_us || options.link_timeout_us > 500000 ||
        !options.maximum_queue_age_us || options.maximum_queue_age_us > 5000)
      throw std::invalid_argument("invalid RMV session timing");
  }
  SessionState state() const { return state_; }
  bool active() const { return state_ == SessionState::active; }
  bool pending() const { return pending_.has_value(); }
  bool request_timed_out() const { return pending_ && pending_->timed_out; }
  std::uint32_t session_id() const { return session_; }
  std::uint64_t boot_id() const { return boot_; }
  std::uint32_t capabilities() const { return capabilities_; }
  std::uint32_t parameter_revision() const { return parameter_revision_; }
  std::uint32_t geometry_revision() const { return geometry_revision_; }
  std::uint32_t firmware_version() const { return firmware_; }
  const TelemetryAdapter& telemetry() const { return telemetry_; }
  const std::optional<RequestResult>& last_result() const { return result_; }
  bool control_ready(std::uint64_t now) const {
    return active() && !control_blocked_ && telemetry_.control_ready(now);
  }
  void disconnect() { fail(Completion::disconnected); }
  bool begin(std::uint64_t nonce, std::uint64_t now, std::uint32_t required_caps = 0x1f) {
    if (!observe_now(now) || !nonce || (required_caps & ~0x3fffu) ||
        std::find(used_nonces_.begin(), used_nonces_.end(), nonce) != used_nonces_.end()) return false;
    reset_session(); result_.reset(); nonce_ = nonce; required_caps_ = required_caps;
    used_nonces_.push_back(nonce);
    while (used_nonces_.size() > 64) used_nonces_.pop_front();
    state_ = SessionState::connecting; last_rx_ = now;
    Bytes p; append(p, nonce, 8); append(p, required_caps, 4);
    p.insert(p.end(), {1, 1, 0, 0});
    return request(1, std::move(p), now, false);
  }
  void ingest(const Bytes& bytes, std::uint64_t now) {
    if (!observe_now(now)) return;
    // Expire before accepting a late packet; a stale link cannot resurrect itself.
    expire(now);
    if (state_ == SessionState::disconnected) return;
    for (const auto& value : decoder_.feed(bytes, now)) consume(value, now);
  }
private:
  void expire(std::uint64_t now) {
    if (state_ != SessionState::disconnected &&
        (now - last_rx_ >= options_.link_timeout_us ||
         (active() && now - last_heartbeat_ >= 500000))) {
      fail(Completion::disconnected); return;
    }
    if (pending_ && pending_->sent && !pending_->queued && !pending_->timed_out &&
        now - *pending_->sent >= options_.request_timeout_us) {
      pending_->timed_out = true;
      result_ = RequestResult{pending_->frame.seq, pending_->frame.type, 0, 0, 0, Completion::timeout};
    }
  }
public:
  void tick(std::uint64_t now) {
    if (!observe_now(now)) return;
    expire(now);
    if (!active()) return;
    // Idle feed is needed for the 50 ms partial-frame parser timeout.
    for (const auto& value : decoder_.feed({}, now)) consume(value, now);
    if (now - last_heartbeat_ >= 100000) heartbeat(now);
    if (options_.automatic_sync) {
      if (pending_ && pending_->frame.type == 2 && pending_->timed_out) pending_.reset();
      if (!pending_ && (!last_sync_sent_ ||
          now - *last_sync_sent_ >= (sync_samples_ < 8 ? 1000u : 250000u))) request_time_sync(now);
    }
  }
  std::vector<Bytes> take_outgoing(std::uint64_t now) {
    if (!observe_now(now)) return {};
    expire(now);
    std::vector<Bytes> result;
    for (const auto& item : outgoing_) {
      if (now - item.queued_us > options_.maximum_queue_age_us) {
        fail(Completion::disconnected); return {};
      }
    }
    for (const auto& item : outgoing_) {
      result.push_back(encode(item.frame));
      if (pending_ && pending_->frame.seq == item.frame.seq) {
        pending_->queued = false; pending_->sent = now;
        if (item.frame.type == 2) last_sync_sent_ = now;
      }
      if (item.frame.type == 3) last_heartbeat_ = now;
    }
    outgoing_.clear(); return result;
  }
  bool request_time_sync(std::uint64_t now) {
    if (!observe_now(now)) return false;
    expire(now);
    if (!active() || (last_sync_sent_ && now <= *last_sync_sent_)) return false;
    if (pending_ && pending_->frame.type == 2 && pending_->timed_out) pending_.reset();
    Bytes p; append(p, now, 8); return request(2, std::move(p), now, false);
  }
  bool heartbeat(std::uint64_t now, std::uint8_t host_state = 0) {
    if (!observe_now(now)) return false;
    expire(now);
    if (!active() || host_state > 2) return false;
    return queue(frame(3, Bytes{host_state, 0, 0, 0}, now), now);
  }
  bool request_mode(std::uint8_t mode, std::uint8_t axis_mask, std::uint64_t now) {
    if (!observe_now(now)) return false;
    expire(now);
    if (!active() || pending_ || (mode != 0 && mode != 2 && mode != 3) ||
        (axis_mask & ~7u) || (mode && (axis_mask & 3u) != 3) || !fresh_pose(now) ||
        (mode == 2 && !(capabilities_ & 4)) || (mode == 3 && !(capabilities_ & 8)) ||
        ((axis_mask & 4u) && !(capabilities_ & (1u << 13)))) return false;
    Bytes p{mode, axis_mask, 0, 0}; append(p, *telemetry_.epoch, 4);
    // Require a new STATUS after a mode request and its successful ACK.
    control_blocked_ = true; telemetry_.status.reset();
    outgoing_.erase(std::remove_if(outgoing_.begin(), outgoing_.end(),
      [](const Queued& item) { return item.frame.type == 0x0201; }), outgoing_.end());
    return request(0x0200, std::move(p), now, true);
  }
  bool aim(float yaw, float pitch, std::uint64_t now, std::uint8_t reference = 0,
           std::uint64_t lifetime_us = 80000, bool valid = true) {
    if (!observe_now(now)) return false;
    expire(now);
    if (!control_ready(now) || !std::isfinite(yaw) || !std::isfinite(pitch) ||
        (reference != 0 && reference != 2) || !lifetime_us || lifetime_us > 100000 ||
        (reference == 0 && !(capabilities_ & 4)) ||
        (reference == 2 && (!(capabilities_ & 8) || telemetry_.status->mode != 3))) return false;
    const auto stamp = telemetry_.clock.to_mcu(now, now);
    if (!stamp || *stamp > std::numeric_limits<std::uint64_t>::max() - lifetime_us) return false;
    Bytes p; append(p, *telemetry_.epoch, 4); append(p, *stamp, 8);
    append(p, *stamp + lifetime_us, 8); p.push_back(reference); p.push_back(0);
    append(p, valid ? 1 : 0, 2);
    for (float number : {valid ? yaw : 0.f, valid ? pitch : 0.f, 0.f, 0.f, 0.f, 0.f}) {
      std::uint32_t bits; std::memcpy(&bits, &number, sizeof(bits)); append(p, bits, 4);
    }
    return queue(frame(0x0201, std::move(p), now), now);
  }
  bool stop(std::uint64_t now, std::uint16_t reason = 0) {
    if (!observe_now(now)) return false;
    expire(now);
    if (!active()) return false;
    // STOP supersedes an uncertain mode change and every unsent motion target.
    pending_.reset(); outgoing_.clear(); control_blocked_ = true; telemetry_.status.reset();
    Bytes p; append(p, reason, 2); append(p, 0, 2);
    return request(0x0202, std::move(p), now, true);
  }
  bool retry(std::uint64_t now) {
    if (!observe_now(now)) return false;
    expire(now);
    if (!pending_ || !pending_->timed_out || pending_->queued || pending_->retries >= 2 ||
        (pending_->frame.type != 1 && pending_->frame.type != 0x0200 && pending_->frame.type != 0x0202))
      return false;
    ++pending_->retries; pending_->timed_out = false; pending_->queued = true;
    // Same seq, sender timestamp, nonce and payload: never reconstruct an action.
    return queue(pending_->frame, now);
  }
};
} // namespace io::rmv

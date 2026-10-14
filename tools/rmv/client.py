"""Transport-independent RMV client and deterministic in-process transport."""
import json
from collections import deque
from pathlib import Path
import struct

from .protocol import Frame, StreamDecoder, encode, fixed_payload, LAYOUTS, ACK_REQUEST, RESPONSE
from .pose import ClockSync, PoseHistory, Pose
from .simulator import Result


class Rejected(RuntimeError):
    def __init__(self, result, detail=0):
        self.result, self.detail = Result(result), detail
        super().__init__(f"{self.result.name}, detail={detail}")


class InMemoryTransport:
    """Each exchange has a deterministic 2 ms round trip; no actual I/O."""
    def __init__(self, board):
        self.board = board
        self.host_now = 1_000_000

    def advance(self, microseconds):
        self.board.advance(microseconds)
        self.host_now += microseconds

    def exchange(self, wire):
        self.advance(1000)
        reply = self.board.receive(wire) + self.board.telemetry()
        self.advance(1000)
        return reply


class Journal:
    """Append raw framed traffic with host times; one JSON object per line."""
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w", encoding="utf-8")
        self.file.write(json.dumps({"schema": "rmv-wire-journal-1", "protocol": "RMV 1.0",
                                    "environment": "offline-simulator", "hardware": False}) + "\n")

    def write(self, direction, host_time, data):
        self.file.write(json.dumps({"direction": direction, "host_time_us": host_time,
                                    "wire_hex": data.hex()}) + "\n")
        self.file.flush()

    def close(self):
        self.file.close()


class Client:
    SIDE_EFFECTS = frozenset({0x0004, 0x0200, 0x0202, 0x0203, 0x0205,
                              0x0302, 0x0303, 0x0304})

    def __init__(self, transport, journal=None):
        self.transport = transport
        self.journal = journal
        self.seq = 0
        self.session = 0
        self.boot = None
        self.capabilities = 0
        self.param_revision = 0
        self.geometry_revision = None
        self.status = None
        self.status_time = None
        self.status_seq = None
        self.pose_seq = None
        self.epoch = None
        self.decoder = StreamDecoder()
        self.clock = ClockSync()
        self.poses = PoseHistory()
        self.transaction = 0
        self.rejected_poses = 0
        self.last_request = None
        self._pending_operation = None
        self.last_operation_result = None
        self._control_confirmed = False
        self._mode_context = None
        self._used_nonces = deque(maxlen=64)
        self._used_sessions = deque(maxlen=64)

    @property
    def now(self):
        return self.transport.host_now

    def frame(self, msg, payload=b"", flags=0):
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        if self.seq == 0:
            self.seq = 1
        return Frame(msg, self.seq, self.session, self.now, payload, flags)

    def exchange_frame(self, frame):
        wire = encode(frame)
        if self.journal:
            self.journal.write("tx", self.now, wire)
        raw = self.transport.exchange(wire)
        if self.journal:
            self.journal.write("rx", self.now, raw)
        frames = self.decoder.feed(raw, now=self.now / 1e6)
        for received in frames:
            self._operation_reply(received)
            self._telemetry(received)
        return frames

    def _telemetry(self, frame):
        from .protocol import seq_newer
        if not self.session or frame.session_id != self.session or frame.flags:
            return
        if frame.msg_type == 0x0101 and len(frame.payload) == 32:
            if self.status_seq is not None and not seq_newer(frame.seq, self.status_seq):
                return
            values = struct.unpack(LAYOUTS["STATUS"][1], frame.payload)
            if (values[0] > 4 or values[1] > 2 or any(v & ~7 for v in values[2:5])
                    or any(v > 1 for v in values[5:8])):
                return
            self.status_seq = frame.seq
            self.status = values
            self.status_time = self.now
            self.param_revision = self.status[-2]
            if self.geometry_revision is not None and self.geometry_revision != self.status[-1]:
                self.poses.reset()
                self.epoch = None
                self._control_confirmed = False
            self.geometry_revision = self.status[-1]
        if frame.msg_type == 0x0102 and len(frame.payload) == 68:
            sensor, coordinate, flags, counter, stamp, device, epoch, *values = struct.unpack(LAYOUTS["POSE"][1], frame.payload)
            if sensor != 0 or coordinate != 0:
                return
            if self.pose_seq is not None and not seq_newer(frame.seq, self.pose_seq):
                return
            self.pose_seq = frame.seq
            if flags & ~0x3F or not flags & 1 or not flags & (8 | 32) or flags & 8 and flags & 32:
                self.poses.reset()
                self.epoch = None
                return
            try:
                mapped_now = self.clock.to_mcu(self.now, self.now)
                self.poses.add(Pose(stamp, counter, epoch, tuple(values[:4])), mapped_now)
                if self.epoch is not None and self.epoch != epoch:
                    self.status = None
                    self._control_confirmed = False
                self.epoch = epoch
            except ValueError:
                self.rejected_poses += 1

    def _ack_values(self, received, frame):
        if received.flags != RESPONSE or received.msg_type != 0x00FF or len(received.payload) != 16:
            return None
        values = struct.unpack("<IHHII", received.payload)
        seq, kind, result, _, _ = values
        if seq != frame.seq or kind != frame.msg_type or not 0 <= result <= 13:
            return None
        if received.session_id != self.session and not (result == Result.BAD_SESSION and received.session_id == 0):
            return None
        return values

    def _operation_reply(self, received):
        if not self._pending_operation:
            return
        frame, _ = self._pending_operation
        values = self._ack_values(received, frame)
        if values is None:
            return
        _, kind, result, _, revision = values
        self.last_operation_result = values
        self._pending_operation = None
        self.param_revision = revision
        if kind == 0x0200:
            self._control_confirmed = (result == Result.OK and len(frame.payload) == 8
                                       and frame.payload[0] in (2, 3)
                                       and self._mode_context == (self.geometry_revision, self.epoch))
        if result == Result.BAD_SESSION:
            self.invalidate()

    def request(self, msg, payload, expect=0x00FF, ack=True):
        if not self.session:
            raise ValueError("an established session is required")
        if self._pending_operation and msg in self.SIDE_EFFECTS and msg != 0x0202:
            raise RuntimeError("resolve the previous operation with its original request before another operation")
        if msg in (0x0200, 0x0202):
            self._control_confirmed = False
            self.status = None
        if msg == 0x0200:
            self._mode_context = (self.geometry_revision, self.epoch)
        if msg == 0x0202:
            self._pending_operation = None  # STOP may supersede an uncertain operation.
        frame = self.frame(msg, payload, ACK_REQUEST if ack else 0)
        if msg in self.SIDE_EFFECTS:
            self._pending_operation = (frame, expect)
            self.last_operation_result = None
        # Read-only queries and heartbeats cannot erase a lost operation's retry record.
        self.last_request = self._pending_operation or (frame, expect)
        return self._request_frame(frame, expect)

    def retry_last_request(self):
        request = self._pending_operation or self.last_request
        if not request or request[0].msg_type in (0x0002, 0x0201, 0x0204):
            raise ValueError("no retryable request; motion setpoints must never be replayed")
        return self._request_frame(*request)

    def _request_frame(self, frame, expect):
        msg = frame.msg_type
        responses = self.exchange_frame(frame)
        for received in responses:
            if received.flags != RESPONSE:
                continue
            if received.msg_type == 0x00FF:
                values = self._ack_values(received, frame)
                if values is None:
                    continue
                _, _, result, detail, revision = values
                self.param_revision = revision
                if result:
                    if result == Result.BAD_SESSION:
                        self.invalidate()
                    raise Rejected(result, detail)
                if expect == 0x00FF:
                    return received.payload
            elif (received.msg_type == expect and received.session_id == self.session
                  and self._dedicated_matches(frame, received)):
                return received.payload
        raise TimeoutError(f"no matching response for 0x{msg:04X}; never retry with a new action ID")

    @staticmethod
    def _dedicated_matches(request, received):
        if request.msg_type == 2:
            return len(received.payload) == 24 and received.payload[:8] == request.payload
        if request.msg_type == 0x0300:
            return len(received.payload) >= 8 and received.payload[:2] == request.payload[:2]
        if request.msg_type == 0x0301:
            return len(received.payload) == 72 and received.payload[:2] == request.payload[:2]
        return False

    def invalidate(self):
        self.session = 0
        self.boot = None
        self.capabilities = 0
        self.param_revision = 0
        self.geometry_revision = None
        self.transaction = 0
        self.status = None
        self.status_time = None
        self.status_seq = None
        self.pose_seq = None
        self.epoch = None
        self.clock.reset()
        self.poses.reset()
        self.last_request = None
        self._pending_operation = None
        self._control_confirmed = False
        self._mode_context = None

    def connect(self, nonce):
        if not 0 < nonce <= 0xFFFFFFFFFFFFFFFF or nonce in self._used_nonces:
            raise ValueError("new connections require a fresh nonzero nonce")
        self.invalidate()
        self.decoder = StreamDecoder()
        frame = self.frame(1, fixed_payload("HELLO", nonce, 0x1F, 1, 1, 0), ACK_REQUEST)
        for reply in self.exchange_frame(frame):
            if (reply.msg_type == 0x00FF and reply.flags == RESPONSE and reply.session_id == 0
                    and len(reply.payload) == 16):
                seq, kind, code, detail, _ = struct.unpack("<IHHII", reply.payload)
                if seq == frame.seq and kind == 1 and 1 <= code <= 13:
                    raise Rejected(code, detail)
            if reply.msg_type != 0x8001 or reply.flags != RESPONSE or len(reply.payload) != 36:
                continue
            values = struct.unpack(LAYOUTS["HELLO_REPLY"][1], reply.payload)
            if values[0] != nonce or not reply.session_id:
                continue
            if (values[3] & 0x1F != 0x1F or values[3] & ~0xBFFF or values[6] != 256
                    or not values[7] or values[8] not in (2, 3)
                    or (values[1], reply.session_id) in self._used_sessions):
                raise ValueError("invalid/reused HELLO session or capabilities")
            self.session, self.boot, self.capabilities = reply.session_id, values[1], values[3]
            self.param_revision, self.geometry_revision = values[4:6]
            self.transaction = 0
            self._used_nonces.append(nonce)
            self._used_sessions.append((self.boot, self.session))
            return
        raise TimeoutError("HELLO failed")

    def synchronize(self):
        for _ in range(8):
            t1 = self.now
            raw = self.request(2, fixed_payload("TIME_SYNC", t1), 0x8002, False)
            echo, t2, t3 = struct.unpack("<QQQ", raw)
            if echo != t1:
                raise ValueError("time reply does not echo this request")
            self.clock.observe(t1, t2, t3, self.now)
        return self.clock.estimate(self.now)

    def heartbeat(self):
        self.request(3, fixed_payload("HEARTBEAT", 0, b"\0" * 3))

    def poll(self):
        raw = self.transport.exchange(b"")
        if self.journal:
            self.journal.write("rx", self.now, raw)
        for frame in self.decoder.feed(raw, now=self.now / 1e6):
            self._operation_reply(frame)
            self._telemetry(frame)

    def mode(self, mode):
        if self.epoch is None:
            raise ValueError("fresh synchronized camera-support pose required")
        now_mcu = self.clock.to_mcu(self.now, self.now)
        if not self.poses.samples:
            raise ValueError("fresh synchronized camera-support pose required")
        self.poses.at(self.poses.samples[-1].time_us, now_mcu)
        return self.request(0x0200, fixed_payload("MODE_REQUEST", mode, 3, 0, self.epoch))

    def aim(self, yaw, pitch, valid=True, reference=2):
        if (not self._control_confirmed or self.epoch is None or self.status is None or not self.status[6]
                or self.status[1] != 2 or self.status[0] not in (2, 3) or self.status[8]
                or self.status_time is None or self.now - self.status_time > 100_000):
            raise ValueError("no confirmed remote control permission")
        now_mcu = self.clock.to_mcu(self.now, self.now)
        # Freshness is required even if the caller only needs the epoch.
        if not self.poses.samples:
            raise ValueError("no pose history")
        self.poses.at(self.poses.samples[-1].time_us, now_mcu)
        payload = fixed_payload("AIM_SETPOINT", self.epoch, now_mcu, now_mcu + 80_000,
                                reference, 0, int(valid), yaw, pitch, 0., 0., 0., 0.)
        return self.request(0x0201, payload)

    def stop(self):
        return self.request(0x0202, fixed_payload("STOP_REMOTE", 0, 0))

    def list_parameters(self):
        keys, start = [], 0
        while True:
            raw = self.request(0x0300, struct.pack("<HH", start, 64), 0x8300, False)
            echoed, following, count, reserved = struct.unpack_from("<4H", raw)
            if echoed != start or reserved or count > 64 or len(raw) != 8 + 2 * count:
                raise ValueError("invalid parameter list")
            page = list(struct.unpack_from(f"<{count}H", raw, 8))
            if page != sorted(set(page)) or any(k < start or k == 0xFFFF for k in page):
                raise ValueError("invalid parameter key order")
            keys.extend(page)
            if following == 0xFFFF:
                return keys
            if not page or following <= page[-1]:
                raise ValueError("parameter pagination did not advance")
            start = following

    def parameter(self, key):
        raw = self.request(0x0301, struct.pack("<HH", key, 0), 0x8301, False)
        values = struct.unpack(LAYOUTS["PARAM_DESC_REPLY"][1], raw)
        if values[0] != key:
            raise ValueError("wrong parameter description key")
        fmt = {1: "<B", 2: "<I", 3: "<i", 4: "<f"}[values[1]]
        return {"key": key, "type": values[1], "access": values[2], "unit": values[3],
                "conditions": values[5], "minimum": struct.unpack_from(fmt, values[6])[0],
                "maximum": struct.unpack_from(fmt, values[7])[0],
                "default": struct.unpack_from(fmt, values[8])[0],
                "current": struct.unpack_from(fmt, values[9])[0],
                "name": values[10].split(b"\0", 1)[0].decode("ascii")}

    def set_parameters(self, changes, persist=False):
        from .protocol import value_slot
        if self._pending_operation:
            raise RuntimeError("resolve the previous operation before starting a parameter transaction")
        if not 1 <= len(changes) <= 16:
            raise ValueError("one to sixteen parameters per transaction")
        self.transaction += 1
        revision = self.param_revision
        payload = struct.pack("<IIHH", self.transaction, revision, len(changes), 0)
        for key, value in changes.items():
            descriptor = self.parameter(key)
            payload += struct.pack("<HBB8s", key, descriptor["type"], 0,
                                   value_slot(descriptor["type"], value))
        self.request(0x0302, payload)
        self.request(0x0303, fixed_payload("PARAM_COMMIT", self.transaction, revision,
                                          int(persist), b"\0" * 3))
        return {key: self.parameter(key)["current"] for key in changes}

    def abort_parameters(self):
        return self.request(0x0304, fixed_payload("PARAM_ABORT", self.transaction))


def inspect_journal(path):
    parsers = {"tx": StreamDecoder(), "rx": StreamDecoder()}
    counts, records = {}, 0
    with Path(path).open(encoding="utf-8") as handle:
        metadata = json.loads(next(handle))
        if metadata.get("schema") != "rmv-wire-journal-1":
            raise ValueError("unknown journal schema")
        last_time = {"tx": -1, "rx": -1}
        for line in handle:
            entry = json.loads(line)
            direction, stamp = entry["direction"], entry["host_time_us"]
            if direction not in parsers or stamp < last_time[direction]:
                raise ValueError("invalid journal direction/time")
            last_time[direction] = stamp
            for frame in parsers[direction].feed(bytes.fromhex(entry["wire_hex"]), stamp / 1e6):
                key = f"{direction}:0x{frame.msg_type:04X}"
                counts[key] = counts.get(key, 0) + 1
            records += 1
    return {"records": records, "messages": counts,
            "crc_errors": sum(p.crc_errors for p in parsers.values()),
            "format_errors": sum(p.format_errors for p in parsers.values())}

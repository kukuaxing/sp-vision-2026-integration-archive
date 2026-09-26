"""Deterministic two-axis RMV reference peer, not a real motor/flash model.

No hardware, sockets or threads. Advance virtual time explicitly. Only the base
profile is implemented; optional firing/chassis/feedforward return UNSUPPORTED.
"""
from dataclasses import dataclass
from enum import IntEnum
import math
import struct

from .protocol import (Frame, StreamDecoder, encode, fixed_payload, LAYOUTS,
                       ACK_REQUEST, RESPONSE, seq_newer, value_slot,
                       JOINT_HEADER, JOINT_ENTRY, PARAM_STAGE_HEADER, PARAM_ENTRY)


class Result(IntEnum):
    OK = 0
    UNSUPPORTED = 1
    BAD_VERSION = 2
    BAD_LENGTH = 3
    BAD_VALUE = 4
    BAD_SESSION = 5
    WRONG_MODE = 6
    NOT_READY = 7
    STALE = 8
    OUT_OF_RANGE = 9
    BUSY = 10
    CONFLICT = 11
    HARDWARE_FAULT = 12
    RESOURCE_LIMIT = 13


@dataclass
class Parameter:
    key: int
    name: str
    minimum: float
    maximum: float
    default: float
    current: float
    unit: int = 2
    group: int = 0
    access: int = 7
    type_id: int = 4

    def describe(self):
        return fixed_payload("PARAM_DESC_REPLY", self.key, self.type_id, self.access,
                             self.unit, self.group, 1 if self.access & 2 else 0,
                             *(value_slot(self.type_id, v) for v in
                               (self.minimum, self.maximum, self.default, self.current)),
                             self.name.encode("ascii"))


class Board:
    CAPS = 0x1F
    SIDE_EFFECTS = {0x0004, 0x0200, 0x0202, 0x0203, 0x0205, 0x0302, 0x0303, 0x0304}

    def __init__(self):
        self.now = 5_000_000
        self.boot = 1
        self.session_counter = 0
        self.tx_seq = 0
        self.parameters = {}
        for axis, prefix, low, high in ((0, "sy", -1.5, 1.5), (1, "p", -0.6, 0.6)):
            base = 0x1000 + axis * 0x100
            for offset, name, lo, hi, default, unit, access in (
                (0, "hard_angle_min", low, low, low, 1, 1),
                (1, "hard_angle_max", high, high, high, 1, 1),
                (3, "follow_speed_max", 0.01, 1.0, 0.4, 2, 7),
                (4, "follow_accel_max", 0.1, 3.0, 1.0, 3, 7),
            ):
                key = base + offset
                self.parameters[key] = Parameter(key, f"{prefix}_{name}", lo, hi,
                                                 default, default, unit, axis + 1, access)
        self.persisted = {key: p.current for key, p in self.parameters.items()}
        self.last_save = -10_000_000
        self.fail_save = False
        self.physical_permit = True  # Virtual fixture, never inferred from hardware.
        self.sensor_online = True
        self.fault = 0
        self.epoch = 1
        self.geometry_rev = 1
        self.param_rev = 1
        self.angle = [0.0, 0.0]
        self.velocity = [0.0, 0.0]
        self.source_counter = 0
        self.last_sample = self.now
        self._reset_session()
        self.decoder = StreamDecoder()

    def _reset_session(self):
        self.session = 0
        self.nonce = None
        self.mode = 0
        self.target = None
        self.last_heartbeat = self.now
        self.last_received = 0
        self.last_applied = 0
        self.last_accepted = 0
        self.last_result = Result.OK
        self.constraints = 0
        self.op_seq = None
        self.op_request = None
        self.op_response = None
        self.staged = None
        self.tx_high = 0
        self.completed_tx = None
        self.service_high = 0
        self.service_result = (0, 0, 0, 0, self.now, self.epoch, 0, b"\0" * 3)
        self.subscriptions = {(0x0101, 0): 20, (0x0102, 1): 100,
                              (0x0103, 0): 100, (0x0106, 0): 2, (0x0108, 0): 1}
        self.last_emit = {}

    def reboot(self):
        self.boot += 1
        self.now = 0
        self.epoch += 1
        self.param_rev = 1
        self.velocity = [0.0, 0.0]
        self.last_sample = 0
        self.source_counter = 0
        self.last_save = -10_000_000
        for key, value in self.persisted.items():
            self.parameters[key].current = value
        self._reset_session()
        self.decoder = StreamDecoder()

    def advance(self, delta_us):
        if delta_us < 0:
            raise ValueError("virtual time cannot run backwards")
        end = self.now + delta_us
        # Expired controls cannot move a virtual joint across a long advance.
        motion_end = end
        if self.target:
            motion_end = min(end, self.target[2], self.last_heartbeat + 500_000)
        dt = max(0, motion_end - self.now) / 1e6
        if self.target and self.physical_permit and self.sensor_online and not self.fault:
            self.last_applied = self.last_accepted
            self.constraints &= ~7
            for axis in range(2):
                wanted = self.target[6 + axis]
                speed = self.parameters[0x1003 + axis * 0x100].current
                acceleration = self.parameters[0x1004 + axis * 0x100].current
                error = wanted - self.angle[axis]
                desired = max(-speed, min(speed, error / dt)) if dt else 0
                if dt and abs(error / dt) > speed:
                    self.constraints |= 2
                velocity = max(self.velocity[axis] - acceleration * dt,
                               min(self.velocity[axis] + acceleration * dt, desired))
                if abs(velocity - desired) > 1e-9:
                    self.constraints |= 4
                move = velocity * dt
                if abs(move) > abs(error) and move * error >= 0:
                    move, velocity = error, 0
                self.angle[axis] += move
                self.velocity[axis] = velocity
        self.now = end
        if self.target and self.now >= self.target[2]:
            self.target = None
            self.velocity = [0.0, 0.0]
            self.constraints |= 8
        if not self.physical_permit or not self.sensor_online or self.fault:
            self.target = None
            self.velocity = [0.0, 0.0]
            self.mode = 4 if self.fault else (1 if not self.physical_permit else 0)
        if self.session and self.now - self.last_heartbeat >= 500_000:
            self._reset_session()
            self.velocity = [0.0, 0.0]
        if self.staged and self.now >= self.staged[3]:
            self.staged = None
        if self.sensor_online and delta_us:
            self.source_counter = (self.source_counter + 1) & 0xFFFFFFFF
            self.last_sample = self.now

    def _frame(self, msg, payload, flags=0, session=None):
        self.tx_seq = (self.tx_seq + 1) & 0xFFFFFFFF
        return Frame(msg, self.tx_seq, self.session if session is None else session,
                     self.now, payload, flags)

    def _ack(self, request, result, detail=0):
        return self._frame(0x00FF, fixed_payload("ACK", request.seq, request.msg_type,
                                               result, detail, self.param_rev), RESPONSE,
                           0 if request.msg_type == 1 else None)

    def receive(self, data):
        self.advance(0)
        replies = []
        for request in self.decoder.feed(data, now=self.now / 1e6):
            replies.extend(self.handle(request))
        return b"".join(encode(frame) for frame in replies)

    def handle(self, req):
        if req.flags & RESPONSE:
            return [self._ack(req, Result.BAD_VALUE)]
        if req.msg_type == 1:
            return [self._hello(req)]
        if not self.session or req.session_id != self.session:
            return [self._ack(req, Result.BAD_SESSION)]
        side_effect = req.msg_type in self.SIDE_EFFECTS
        if side_effect:
            if not req.flags & ACK_REQUEST:
                return [self._ack(req, Result.BAD_VALUE)]
            if self.op_seq is not None:
                if req.seq == self.op_seq:
                    if (req.msg_type, req.payload) == self.op_request:
                        return [self.op_response]
                    return [self._ack(req, Result.CONFLICT)]
                if not seq_newer(req.seq, self.op_seq):
                    return [self._ack(req, Result.STALE)]
        try:
            result = self._dispatch(req)
        except struct.error:
            result = self._ack(req, Result.BAD_LENGTH)
        except (ValueError, OverflowError):
            result = self._ack(req, Result.BAD_VALUE)
        if side_effect:
            self.op_seq, self.op_request, self.op_response = req.seq, (req.msg_type, req.payload), result
        return [] if result is None else [result]

    def _hello(self, req):
        if req.session_id != 0:
            return self._ack(req, Result.BAD_SESSION)
        try:
            nonce, required, low, high, reserved = struct.unpack("<QIBBH", req.payload)
        except struct.error:
            return self._ack(req, Result.BAD_LENGTH)
        if reserved or low > high:
            return self._ack(req, Result.BAD_VALUE)
        if not low <= 1 <= high:
            return self._ack(req, Result.BAD_VERSION)
        if required & ~self.CAPS:
            return self._ack(req, Result.UNSUPPORTED)
        if nonce != self.nonce:
            if self.session and self.mode in (2, 3):
                return self._ack(req, Result.BUSY)
            self._reset_session()
            self.session_counter += 1
            self.session = self.session_counter
            self.nonce = nonce
        payload = fixed_payload("HELLO_REPLY", nonce, self.boot, 1, self.CAPS,
                                self.param_rev, self.geometry_rev, 256, 1, 2)
        return self._frame(0x8001, payload, RESPONSE)

    def _dispatch(self, req):
        msg = req.msg_type
        if msg == 2:
            t1, = struct.unpack("<Q", req.payload)
            return self._frame(0x8002, fixed_payload("TIME_REPLY", t1, self.now, self.now), RESPONSE)
        if msg == 3:
            state, reserved = struct.unpack("<B3s", req.payload)
            if state > 2 or any(reserved):
                return self._ack(req, Result.BAD_VALUE)
            self.last_heartbeat = self.now
            return self._ack(req, Result.OK) if req.flags & ACK_REQUEST else None
        if msg == 0x0200:
            mode, axes, reserved, epoch = struct.unpack("<BBHI", req.payload)
            if reserved or mode not in (0, 2, 3) or axes & ~3:
                return self._ack(req, Result.BAD_VALUE)
            if epoch != self.epoch:
                return self._ack(req, Result.CONFLICT)
            if mode and (axes != 3 or not self.sensor_online):
                return self._ack(req, Result.NOT_READY)
            if mode and not self.physical_permit:
                return self._ack(req, Result.WRONG_MODE)
            if self.fault:
                return self._ack(req, Result.HARDWARE_FAULT)
            self.mode, self.target, self.velocity = mode, None, [0., 0.]
            return self._ack(req, Result.OK)
        if msg == 0x0201:
            return self._aim(req)
        if msg == 0x0202:
            _, reserved = struct.unpack("<HH", req.payload)
            if reserved:
                return self._ack(req, Result.BAD_VALUE)
            self.target, self.velocity = None, [0., 0.]
            self.mode = 0 if self.physical_permit else 1
            return self._ack(req, Result.OK)
        if msg in (0x0203, 0x0204):
            return self._ack(req, Result.UNSUPPORTED)
        if msg == 0x0205:
            operation, action, axes, reserved, epoch = struct.unpack("<IBBHI", req.payload)
            if not operation or action > 2 or axes & ~3 or reserved:
                return self._ack(req, Result.BAD_VALUE)
            if epoch != self.epoch:
                return self._ack(req, Result.CONFLICT)
            if operation <= self.service_high:
                return self._ack(req, Result.STALE)
            if action == 1:
                return self._ack(req, Result.UNSUPPORTED)  # No pretend motor homing.
            if self.fault or not self.physical_permit or self.mode not in (0, 4):
                return self._ack(req, Result.NOT_READY)
            self.service_high = operation
            self.service_result = (operation, action, 3, 0, self.now, self.epoch, axes, b"\0" * 3)
            return self._ack(req, Result.OK)
        if msg == 4:
            return self._subscribe(req)
        if 0x0300 <= msg <= 0x0304:
            return self._parameter(req)
        return self._ack(req, Result.UNSUPPORTED) if req.flags & ACK_REQUEST else None

    def _aim(self, req):
        values = struct.unpack(LAYOUTS["AIM_SETPOINT"][1], req.payload)
        epoch, produced, deadline, reference, policy, valid, *angles = values
        self.last_received = req.seq
        result = Result.OK
        if not req.seq or valid & ~7 or not all(math.isfinite(v) for v in angles):
            result = Result.BAD_VALUE
        elif reference not in (0, 2) or policy or valid & 6:
            result = Result.UNSUPPORTED
        elif any(angles[2:]):
            result = Result.BAD_VALUE
        elif epoch != self.epoch:
            result = Result.CONFLICT
        elif self.last_accepted and not seq_newer(req.seq, self.last_accepted):
            result = Result.STALE
        elif self.mode not in (2, 3) or not self.physical_permit or (reference == 2 and self.mode != 3):
            result = Result.WRONG_MODE
        elif not self.sensor_online or self.fault:
            result = Result.NOT_READY
        elif not (produced <= self.now + 5_000 and self.now - produced <= 50_000
                  and produced < deadline <= produced + 100_000 and self.now < deadline):
            result = Result.STALE
        elif valid & 1 and any(not self.parameters[0x1000 + a * 0x100].current <= angles[a]
                               <= self.parameters[0x1001 + a * 0x100].current for a in range(2)):
            result = Result.OUT_OF_RANGE
        self.last_result = result
        if result == Result.OK:
            self.last_accepted = req.seq
            # Virtual base is fixed: world LOS and joint angles coincide.
            self.target = values if valid & 1 else None
            if not valid & 1:
                self.velocity = [0., 0.]
                self.last_applied = req.seq
        return self._ack(req, result) if req.flags & ACK_REQUEST else None

    def _subscribe(self, req):
        count, reserved = struct.unpack_from("<B3s", req.payload)
        if len(req.payload) != 4 + count * 8:
            return self._ack(req, Result.BAD_LENGTH)
        if not 1 <= count <= 16 or any(reserved):
            return self._ack(req, Result.BAD_VALUE)
        proposed = dict(self.subscriptions)
        seen = set()
        sizes = {0x0101: 62, 0x0102: 98, 0x0103: 110, 0x0106: 54, 0x0108: 70}
        for index in range(count):
            msg, rate, source = struct.unpack_from("<HHI", req.payload, 4 + index * 8)
            if msg not in sizes or source != (1 if msg == 0x0102 else 0):
                return self._ack(req, Result.UNSUPPORTED, msg)
            if msg in seen or rate > 200 or (msg == 0x0101 and rate < 10) or (msg in (0x0106, 0x0108) and rate < 1):
                return self._ack(req, Result.BAD_VALUE, msg)
            seen.add(msg)
            proposed[(msg, source)] = rate
        if sum(sizes[msg] * rate for (msg, _), rate in proposed.items()) > 46080 * .7:
            return self._ack(req, Result.RESOURCE_LIMIT)
        self.subscriptions = proposed
        return self._ack(req, Result.OK)

    def _parameter(self, req):
        if req.msg_type == 0x0300:
            start, limit = struct.unpack("<HH", req.payload)
            if not 1 <= limit <= 64:
                return self._ack(req, Result.BAD_VALUE)
            remaining = [key for key in sorted(self.parameters) if key >= start]
            keys = remaining[:limit]
            next_key = remaining[limit] if len(remaining) > limit else 0xFFFF
            return self._frame(0x8300, struct.pack("<4H", start, next_key, len(keys), 0) +
                               struct.pack(f"<{len(keys)}H", *keys), RESPONSE)
        if req.msg_type == 0x0301:
            key, reserved = struct.unpack("<HH", req.payload)
            if reserved:
                return self._ack(req, Result.BAD_VALUE)
            if key not in self.parameters:
                return self._ack(req, Result.UNSUPPORTED, key)
            return self._frame(0x8301, self.parameters[key].describe(), RESPONSE)
        if req.msg_type == 0x0304:
            transaction, = struct.unpack("<I", req.payload)
            if not self.staged or self.staged[0] != transaction:
                return self._ack(req, Result.NOT_READY)
            self.staged = None
            return self._ack(req, Result.OK)
        if self.mode != 0 or self.target is not None or self.fault:
            return self._ack(req, Result.WRONG_MODE)
        if req.msg_type == 0x0302:
            transaction, revision, count, reserved = PARAM_STAGE_HEADER.unpack_from(req.payload)
            if len(req.payload) != 12 + count * 12:
                return self._ack(req, Result.BAD_LENGTH)
            if not transaction or reserved or not 1 <= count <= 16:
                return self._ack(req, Result.BAD_VALUE)
            if revision != self.param_rev:
                return self._ack(req, Result.CONFLICT)
            if self.staged:
                if transaction == self.staged[0]:
                    return self._ack(req, Result.OK if req.payload == self.staged[4] else Result.CONFLICT)
                return self._ack(req, Result.BUSY)
            if transaction <= self.tx_high:
                return self._ack(req, Result.STALE)
            changes = {}
            for index in range(count):
                key, type_id, padding, slot = PARAM_ENTRY.unpack_from(req.payload, 12 + index * 12)
                if key not in self.parameters:
                    return self._ack(req, Result.UNSUPPORTED, key)
                param = self.parameters[key]
                if type_id != param.type_id or padding or any(slot[4:]) or key in changes:
                    return self._ack(req, Result.BAD_VALUE, key)
                if not param.access & 2:
                    return self._ack(req, Result.UNSUPPORTED, key)
                value, = struct.unpack("<f", slot[:4])
                if not math.isfinite(value):
                    return self._ack(req, Result.BAD_VALUE, key)
                wire_min = struct.unpack("<f", struct.pack("<f", param.minimum))[0]
                wire_max = struct.unpack("<f", struct.pack("<f", param.maximum))[0]
                if not wire_min <= value <= wire_max:
                    return self._ack(req, Result.OUT_OF_RANGE, key)
                changes[key] = value
            self.tx_high = transaction
            self.staged = (transaction, revision, changes, self.now + 2_000_000, req.payload)
            return self._ack(req, Result.OK)
        transaction, revision, persist, reserved = struct.unpack("<IIB3s", req.payload)
        if persist > 1 or any(reserved):
            return self._ack(req, Result.BAD_VALUE)
        if self.completed_tx and transaction == self.completed_tx[0]:
            if req.payload != self.completed_tx[1]:
                return self._ack(req, Result.CONFLICT)
            return self._ack(req, Result.OK)
        if revision != self.param_rev:
            return self._ack(req, Result.CONFLICT)
        if not self.staged or (transaction, revision) != self.staged[:2]:
            return self._ack(req, Result.NOT_READY)
        if persist and self.now - self.last_save < 10_000_000:
            return self._ack(req, Result.BUSY)
        if persist and self.fail_save:
            return self._ack(req, Result.HARDWARE_FAULT)
        changes = self.staged[2]
        if persist:
            self.persisted = self.persisted | changes
            self.last_save = self.now
        for key, value in changes.items():
            self.parameters[key].current = value
        self.param_rev += 1
        self.completed_tx = (transaction, req.payload)
        self.staged = None
        return self._ack(req, Result.OK)

    def telemetry(self, force=False):
        if not self.session:
            return b""
        frames = []
        for (msg, _), rate in self.subscriptions.items():
            if not rate or (not force and self.now - self.last_emit.get(msg, -10_000_000) < 1e6 / rate):
                continue
            self.last_emit[msg] = self.now
            if msg == 0x0101:
                payload = fixed_payload("STATUS", self.mode, 2 if self.mode in (2, 3) else (0 if self.physical_permit else 1),
                                        3, 3 if self.mode in (2, 3) else 0, 3, 1,
                                        int(self.mode in (2, 3) and self.physical_permit), 0,
                                        self.fault, self.last_received, self.last_applied,
                                        self.last_result, self.constraints, self.param_rev, self.geometry_rev)
            elif msg == 0x0102:
                yaw, pitch = self.angle
                cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
                # q_W_G = Rz(yaw) Ry(-pitch): +pitch raises the forward axis.
                q = (cy * cp, sy * sp, -cy * sp, sy * cp)
                payload = fixed_payload("POSE", 0, 0, 9 if self.sensor_online else 0,
                                        self.source_counter, self.last_sample, 0, self.epoch,
                                        *q, 0., 0., 0., 0., 0., 0.)
            elif msg == 0x0103:
                payload = JOINT_HEADER.pack(2, 0, 1, self.now, self.epoch)
                for axis in range(2):
                    target = self.target[6 + axis] if self.target else self.angle[axis]
                    state_bits = 0x3D | (2 if self.mode in (2, 3) else 0)
                    payload += JOINT_ENTRY.pack(axis, state_bits, 0, self.angle[axis], self.velocity[axis],
                                                target, self.velocity[axis], 0., 0., 0)
            elif msg == 0x0106:
                payload = fixed_payload("SERVICE_STATE", *self.service_result)
            else:
                payload = fixed_payload("DIAGNOSTICS", self.decoder.crc_errors,
                                        self.decoder.format_errors, 0, 0, 0, 0, 0, 10000, 0, 0)
            frames.append(self._frame(msg, payload))
        return b"".join(encode(frame) for frame in frames)

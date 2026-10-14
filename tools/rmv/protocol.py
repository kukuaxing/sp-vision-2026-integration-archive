"""RMV 1.0 wire-format reference. Offline only: never opens a serial port.

Run: python serial_protocol.py --self-test
     python serial_protocol.py --vectors serial_test_vectors.json

This implements framing, CRC, incremental parsing and payload layouts. Control
permission, timestamp validity, idempotency, safety and parameter transactions
belong to the application; this module does not implement them.
"""
from __future__ import annotations

import argparse
import binascii
from dataclasses import dataclass
import json
from pathlib import Path
import struct
import time

MAGIC = b"RMV1"
MAJOR, MINOR = 1, 0
MAX_PAYLOAD = 256
ACK_REQUEST, RESPONSE = 1, 2
HEADER = struct.Struct("<4sBBHHHIIQ")
CRC_FIELD = struct.Struct("<H")
FRAME_TIMEOUT_S = 0.050

# Wire layouts, independent of any C compiler alignment.
LAYOUTS = {
    "HELLO": (0x0001, "<QIBBH", 16),
    "HELLO_REPLY": (0x8001, "<QQIIIIHBB", 36),
    "TIME_SYNC": (0x0002, "<Q", 8),
    "TIME_REPLY": (0x8002, "<QQQ", 24),
    "HEARTBEAT": (0x0003, "<B3s", 4),
    "ACK": (0x00FF, "<IHHII", 16),
    "STATUS": (0x0101, "<8BIIIHHII", 32),
    "POSE": (0x0102, "<BBHIQQIffffffffff", 68),
    "SHOOT_STATE": (0x0104, "<QIIffffIffIHH", 52),
    "SHOT_EVENT": (0x0105, "<IBBHQI", 20),
    "SERVICE_STATE": (0x0106, "<IBBHQIB3s", 24),
    "CHASSIS_STATE": (0x0107, "<QIIffffff", 40),
    "DIAGNOSTICS": (0x0108, "<10I", 40),
    "MODE_REQUEST": (0x0200, "<BBHI", 8),
    "AIM_SETPOINT": (0x0201, "<IQQBBHffffff", 48),
    "STOP_REMOTE": (0x0202, "<HH", 4),
    "SHOOT_REQUEST": (0x0203, "<IBBHIQ", 20),
    "CHASSIS_SETPOINT": (0x0204, "<QQIBBHfff", 36),
    "SERVICE_REQUEST": (0x0205, "<IBBHI", 12),
    "PARAM_LIST_REQ": (0x0300, "<HH", 4),
    "PARAM_DESC_REQ": (0x0301, "<HH", 4),
    "PARAM_DESC_REPLY": (0x8301, "<HBBBBH8s8s8s8s32s", 72),
    "PARAM_COMMIT": (0x0303, "<IIB3s", 12),
    "PARAM_ABORT": (0x0304, "<I", 4),
}
JOINT_HEADER = struct.Struct("<BBHQI")
JOINT_ENTRY = struct.Struct("<BBHffffffI")
SUBSCRIBE_HEADER = struct.Struct("<B3s")
SUBSCRIBE_ENTRY = struct.Struct("<HHI")
PARAM_LIST_HEADER = struct.Struct("<HHHH")
PARAM_STAGE_HEADER = struct.Struct("<IIHH")
PARAM_ENTRY = struct.Struct("<HBB8s")


def crc16(data: bytes | bytearray) -> int:
    """CRC-16/CCITT-FALSE; check(b'123456789') == 0x29B1."""
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xFFFF
    return value


def seq_newer(candidate: int, previous: int) -> bool:
    return 0 < ((candidate - previous) & 0xFFFFFFFF) < 0x80000000


@dataclass(frozen=True)
class Frame:
    msg_type: int
    seq: int
    session_id: int
    sender_time_us: int
    payload: bytes = b""
    flags: int = 0


def encode(frame: Frame) -> bytes:
    if len(frame.payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 256 bytes")
    if frame.flags & ~3 or frame.flags < 0:
        raise ValueError("reserved header flags must be zero")
    header = HEADER.pack(MAGIC, MAJOR, MINOR, frame.msg_type,
                         len(frame.payload), frame.flags, frame.seq,
                         frame.session_id, frame.sender_time_us)
    body = header + frame.payload
    return body + CRC_FIELD.pack(crc16(body))


def fixed_payload(name: str, *values) -> bytes:
    return struct.pack(LAYOUTS[name][1], *values)


def unpack_fixed(name: str, payload: bytes) -> tuple:
    """Requires exact length. Callers must also validate field semantics."""
    return struct.unpack(LAYOUTS[name][1], payload)


def value_slot(type_id: int, value) -> bytes:
    formats = {1: "<B", 2: "<I", 3: "<i", 4: "<f"}
    if type_id not in formats:
        raise ValueError("unknown parameter type")
    if type_id == 1 and value not in (0, 1):
        raise ValueError("bool must be 0 or 1")
    return struct.pack(formats[type_id], value).ljust(8, b"\0")


class StreamDecoder:
    """Envelope parser; semantic validators MUST run before any actuation.

    Call feed(b'') periodically to expire an incomplete frame during idle.
    A malformed but plausible length can defer resynchronization up to 50 ms.
    Buffered bytes after each call are bounded by one maximum-sized frame.
    """

    def __init__(self):
        self.buffer = bytearray()
        self.pending_since = None
        self.crc_errors = 0
        self.format_errors = 0
        self.timeouts = 0
        self.noise_bytes = 0

    def feed(self, data: bytes, now: float | None = None) -> list[Frame]:
        now = time.monotonic() if now is None else now
        if (self.pending_since is not None
                and now - self.pending_since >= FRAME_TIMEOUT_S):
            del self.buffer[:1]
            self.pending_since = None
            self.timeouts += 1
        self.buffer.extend(data)
        frames = []
        while True:
            offset = self.buffer.find(MAGIC)
            if offset < 0:
                # Preserve a possible split magic suffix.
                trim = max(0, len(self.buffer) - len(MAGIC) + 1)
                self.noise_bytes += trim
                del self.buffer[:trim]
                self.pending_since = None
                break
            if offset:
                self.noise_bytes += offset
                del self.buffer[:offset]
                self.pending_since = None
            if len(self.buffer) < HEADER.size:
                if self.pending_since is None:
                    self.pending_since = now
                break
            _, major, minor, msg, length, flags, seq, session, stamp = HEADER.unpack_from(self.buffer)
            if (major, minor) != (MAJOR, MINOR) or length > MAX_PAYLOAD or flags & ~3:
                self.format_errors += 1
                del self.buffer[:1]
                self.pending_since = None
                continue
            total = HEADER.size + length + CRC_FIELD.size
            if len(self.buffer) < total:
                if self.pending_since is None:
                    self.pending_since = now
                break
            expected = CRC_FIELD.unpack_from(self.buffer, total - 2)[0]
            if crc16(self.buffer[:total - 2]) != expected:
                self.crc_errors += 1
                del self.buffer[:1]
                self.pending_since = None
                continue
            payload = bytes(self.buffer[HEADER.size:total - 2])
            frames.append(Frame(msg, seq, session, stamp, payload, flags))
            del self.buffer[:total]
            self.pending_since = None
        return frames


def examples() -> list[tuple[str, Frame]]:
    """Deterministic OFFLINE fixtures, not instructions to a live robot."""
    session = 0x10203040
    nonce = 0x0102030405060708
    definitions = [
        ("HELLO", (nonce, 0x1F, 1, 1, 0), 0, ACK_REQUEST),
        ("HELLO_REPLY", (nonce, 0x1112131415161718, 1, 0x1F, 7, 3, 256, 2, 2), session, RESPONSE),
        ("TIME_SYNC", (1000000,), session, 0),
        ("TIME_REPLY", (1000000, 5000100, 5000200), session, RESPONSE),
        ("HEARTBEAT", (0, b"\0" * 3), session, 0),
        ("STATUS", (0, 0, 3, 0, 3, 1, 0, 0, 0, 0, 0, 0, 0, 7, 3), session, 0),
        ("POSE", (0, 0, 0x27, 100, 5000000, 0, 1,
                   1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.80665), session, 0),
        ("MODE_REQUEST", (3, 3, 0, 1), session, ACK_REQUEST),
        ("AIM_SETPOINT", (1, 5000000, 5080000, 2, 0, 1,
                           0.1, 0.05, 0.0, 0.0, 0.0, 0.0), session, 0),
        ("ACK", (8, 0x0200, 0, 0, 7), session, RESPONSE),
        ("STOP_REMOTE", (0, 0), session, ACK_REQUEST),
        ("PARAM_LIST_REQ", (0, 64), session, 0),
        ("PARAM_DESC_REQ", (0x1103, 0), session, 0),
        ("PARAM_DESC_REPLY", (0x1103, 4, 7, 2, 2, 1,
                              value_slot(4, 0.01), value_slot(4, 1.0),
                              value_slot(4, 0.1), value_slot(4, 0.1),
                              b"p_follow_speed_max"), session, RESPONSE),
        ("PARAM_COMMIT", (1, 7, 0, b"\0" * 3), session, ACK_REQUEST),
        ("PARAM_ABORT", (1,), session, ACK_REQUEST),
        ("DIAGNOSTICS", (0, 0, 0, 0, 0, 0, 100, 1000, 30, 0), session, 0),
        ("SHOOT_STATE", (5000000, 0, 0, 0., 0., 0., 0., 0, 0., 0., 0, 0, 0), session, 0),
        ("SHOT_EVENT", (1, 4, 0, 6, 5000000, 0), session, 0),
        ("SHOOT_REQUEST", (1, 0, 0, 0, 0, 5080000), session, ACK_REQUEST),
        ("CHASSIS_SETPOINT", (5000000, 5080000, 1, 0, 0, 0, 0., 0., 0.), session, ACK_REQUEST),
        ("CHASSIS_STATE", (5000000, 1, 0, 0., 0., 0., 0., 0., 0.), session, 0),
        ("SERVICE_REQUEST", (1, 0, 3, 0, 1), session, ACK_REQUEST),
        ("SERVICE_STATE", (1, 0, 3, 0, 5000000, 1, 3, b"\0" * 3), session, 0),
    ]
    result = []
    for index, (name, values, sid, flags) in enumerate(definitions, 1):
        result.append((name, Frame(LAYOUTS[name][0], index, sid, 1000000,
                                  fixed_payload(name, *values), flags)))
    joints = JOINT_HEADER.pack(2, 0, 1, 5000000, 1)
    for axis in (0, 1):
        joints += JOINT_ENTRY.pack(axis, 0x1D, 0, 0., 0., 0., 0., 0., 0., 1000)
    subscriptions = SUBSCRIBE_HEADER.pack(3, b"\0" * 3)
    for values in ((0x0101, 20, 0), (0x0102, 100, 1), (0x0103, 100, 0)):
        subscriptions += SUBSCRIBE_ENTRY.pack(*values)
    stage = PARAM_STAGE_HEADER.pack(1, 7, 1, 0)
    stage += PARAM_ENTRY.pack(0x1103, 4, 0, value_slot(4, 0.1))
    listing = PARAM_LIST_HEADER.pack(0, 0xFFFF, 2, 0) + struct.pack("<HH", 0x1103, 0x1104)
    for name, msg, payload, flags in (
        ("JOINT_STATE", 0x0103, joints, 0),
        ("SUBSCRIBE", 0x0004, subscriptions, ACK_REQUEST),
        ("PARAM_STAGE", 0x0302, stage, ACK_REQUEST),
        ("PARAM_LIST_REPLY", 0x8300, listing, RESPONSE),
    ):
        result.append((name, Frame(msg, len(result) + 1, session, 1000000, payload, flags)))
    return result


def self_test() -> None:
    assert HEADER.size == 28
    assert JOINT_HEADER.size == 16 and JOINT_ENTRY.size == 32
    assert PARAM_STAGE_HEADER.size == 12 and PARAM_ENTRY.size == 12
    assert crc16(b"123456789") == 0x29B1
    assert seq_newer(1, 0xFFFFFFFF) and not seq_newer(5, 5)
    assert not seq_newer(4, 5) and not seq_newer(0x80000000, 0)
    for name, (_, fmt, size) in LAYOUTS.items():
        assert struct.calcsize(fmt) == size, name
    fixtures = examples()
    split_count = 0
    for name, frame in fixtures:
        raw = encode(frame)
        # Independent stdlib implementation verifies CRC parameter choice.
        assert crc16(raw[:-2]) == binascii.crc_hqx(raw[:-2], 0xFFFF), name
        assert len(raw) == len(frame.payload) + 30
        assert StreamDecoder().feed(raw, now=0) == [frame]
        for split in range(len(raw) + 1):
            decoder = StreamDecoder()
            actual = decoder.feed(raw[:split], now=0)
            actual += decoder.feed(raw[split:], now=0.001)
            assert actual == [frame], (name, split)
            split_count += 1
        if name in LAYOUTS:
            values = unpack_fixed(name, frame.payload)
            assert fixed_payload(name, *values) == frame.payload
    stream = b"noise" + b"junk".join(encode(frame) for _, frame in fixtures)
    assert StreamDecoder().feed(stream, now=0) == [f for _, f in fixtures]
    decoder = StreamDecoder()
    decoded = []
    for byte in stream:
        decoded += decoder.feed(bytes([byte]), now=0)
    assert decoded == [f for _, f in fixtures]
    raw = encode(fixtures[0][1])
    bad = bytearray(raw)
    bad[30] ^= 1
    decoder = StreamDecoder()
    assert decoder.feed(bytes(bad) + raw, now=0) == [fixtures[0][1]]
    assert decoder.crc_errors == 1
    for field_offset, replacement in ((8, b"\xff\xff"), (4, b"\x02"), (10, b"\x04\x00")):
        malformed = bytearray(raw)
        malformed[field_offset:field_offset + len(replacement)] = replacement
        decoder = StreamDecoder()
        assert decoder.feed(bytes(malformed) + raw, now=0) == [fixtures[0][1]]
        assert decoder.format_errors >= 1
    plausible_bad_length = HEADER.pack(MAGIC, 1, 0, 1, 256, 0, 1, 0, 0)
    decoder = StreamDecoder()
    assert decoder.feed(plausible_bad_length + raw, now=0) == []
    assert decoder.feed(b"", now=0.051) == [fixtures[0][1]]
    assert decoder.timeouts == 1
    decoder = StreamDecoder()
    decoder.feed(b"x" * 10000, now=0)
    assert len(decoder.buffer) <= 3
    unknown = Frame(0x7777, 1, 1, 1, b"x")
    assert StreamDecoder().feed(encode(unknown), now=0) == [unknown]
    maximum = Frame(1, 1, 1, 1, b"x" * 256)
    assert StreamDecoder().feed(encode(maximum), now=0) == [maximum]
    for invalid in (Frame(1, 1, 1, 1, b"x" * 257), Frame(1, 1, 1, 1, flags=4)):
        try:
            encode(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid frame accepted")
    print(f"PASS: {len(fixtures)} message fixtures, {split_count} split positions; "
          "CRC cross-check, concatenation, noise, corruption, malformed headers, "
          "timeout recovery, bounded buffer, sequence wrap. Offline framing only.")


def write_vectors(path: Path) -> None:
    vectors = []
    for name, frame in examples():
        raw = encode(frame)
        vectors.append({
            "name": name, "msg_type": f"0x{frame.msg_type:04X}",
            "seq": frame.seq, "session_id": frame.session_id,
            "sender_time_us": frame.sender_time_us, "flags": frame.flags,
            "payload_bytes": len(frame.payload), "frame_bytes": len(raw),
            "payload_hex": frame.payload.hex(" "),
            "crc16": f"0x{crc16(raw[:-2]):04X}", "frame_hex": raw.hex(" "),
        })
    bad = bytearray(encode(examples()[0][1]))
    bad[-1] ^= 0x80
    output = {
        "protocol": "RMV 1.0 proposal",
        "warning": "Independent offline wire fixtures; not a valid live control session or hardware test.",
        "crc_check": {"ascii": "123456789", "crc16": "0x29B1"},
        "valid_frames": vectors,
        "invalid_frames": [{"name": "HELLO_BAD_CRC", "frame_hex": bad.hex(" "),
                            "expected": "Discard; no ACK; increment CRC error count"}],
    }
    path.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {len(vectors)} valid fixtures and 1 invalid fixture: {path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--vectors", type=Path)
    arguments = parser.parse_args()
    if arguments.self_test:
        self_test()
    if arguments.vectors is not None:
        write_vectors(arguments.vectors)
    if not arguments.self_test and arguments.vectors is None:
        parser.print_help()

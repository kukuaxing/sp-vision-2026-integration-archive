#!/usr/bin/env python3
"""Host-side wire-format checks for the NUC protocol V3."""

from __future__ import annotations

import math
import random
import struct


def crc16(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if crc & 1 else crc >> 1
    return crc & 0xFFFF


def with_crc(payload: bytes) -> bytes:
    return payload + struct.pack("<H", crc16(payload))


FORMATS = {
    "aim": "<2sBBffH",
    "feedback": "<2sBBfHffH",
    "navigation": "<2sBBBBHIHbBffffH",
    "tactical": "<2sBBBBHIBBBBBBBBHBBBBBBH",
    "fast_state": "<2sBBBBHIfffffffffH",
    "robot_status": "<2sBBBBHIBBBBHHHHHHHHffffffIIH",
    "legacy_navigation": "<4sfffbH",
}

EXPECTED_SIZES = {
    "aim": 14,
    "feedback": 20,
    "navigation": 34,
    "tactical": 30,
    "fast_state": 50,
    "robot_status": 66,
    "legacy_navigation": 19,
}


def make_aim(control: int = 1, shoot: int = 1) -> bytes:
    body = struct.pack("<2sBBff", b"SP", control, shoot, 1.25, -0.15)
    return with_crc(body)


def stream_extract_aim(stream: bytes) -> list[bytes]:
    """Reference resynchronizer used to verify fragmentation/noise behavior."""
    buffer = bytearray()
    frames: list[bytes] = []
    random.seed(20260914)
    cursor = 0
    while cursor < len(stream):
        width = random.randint(1, 9)
        buffer.extend(stream[cursor : cursor + width])
        cursor += width
        while len(buffer) >= 2:
            if buffer[:2] != b"SP":
                del buffer[0]
                continue
            if len(buffer) < EXPECTED_SIZES["aim"]:
                break
            candidate = bytes(buffer[: EXPECTED_SIZES["aim"]])
            expected = struct.unpack_from("<H", candidate, len(candidate) - 2)[0]
            if crc16(candidate[:-2]) != expected:
                del buffer[0]
                continue
            frames.append(candidate)
            del buffer[: EXPECTED_SIZES["aim"]]
    return frames


def main() -> None:
    for name, fmt in FORMATS.items():
        actual = struct.calcsize(fmt)
        assert actual == EXPECTED_SIZES[name], (name, actual, EXPECTED_SIZES[name])

    assert crc16(b"123456789") == 0x6F91
    first = make_aim(1, 0)
    second = make_aim(1, 1)
    broken = bytearray(make_aim())
    broken[7] ^= 0x80
    recovered = stream_extract_aim(b"noise" + bytes(broken) + b"junk" + first + second)
    assert recovered == [first, second]

    _, control, shoot, yaw, pitch, received_crc = struct.unpack(FORMATS["aim"], second)
    assert control == 1 and shoot == 1
    assert math.isclose(yaw, 1.25) and math.isclose(pitch, -0.15, abs_tol=1e-6)
    assert received_crc == crc16(second[:-2])
    print("protocol_v3_selftest: PASS (sizes, CRC, corruption rejection, stream resync)")


if __name__ == "__main__":
    main()

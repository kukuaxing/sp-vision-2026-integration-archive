#!/usr/bin/env python3
"""Host-side reference tests for the XUC V2 wire format."""

import math
import struct
import unittest


RX_FORMAT = "<2sBBffH"
TX_FORMAT = "<2sBBfHffH"
RX_SIZE = struct.calcsize(RX_FORMAT)
TX_SIZE = struct.calcsize(TX_FORMAT)


def crc16(data: bytes, initial: int = 0xFFFF) -> int:
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x8408 if value & 1 else 0)
    return value & 0xFFFF


def with_crc(payload_without_crc: bytes) -> bytes:
    return payload_without_crc + struct.pack("<H", crc16(payload_without_crc))


def valid_crc(packet: bytes) -> bool:
    if len(packet) < 3:
        return False
    return struct.unpack_from("<H", packet, len(packet) - 2)[0] == crc16(packet[:-2])


class ReferenceStreamParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.valid_packets = []
        self.crc_errors = 0
        self.field_errors = 0

    def feed(self, data: bytes) -> None:
        self.buffer.extend(data)
        while len(self.buffer) >= 2:
            header = self.buffer.find(b"SP")
            if header < 0:
                self.buffer[:] = b"S" if self.buffer[-1:] == b"S" else b""
                return
            del self.buffer[:header]
            if len(self.buffer) < RX_SIZE:
                return
            candidate = bytes(self.buffer[:RX_SIZE])
            if not valid_crc(candidate):
                self.crc_errors += 1
                del self.buffer[0]
                continue
            _, control, shoot, yaw, pitch, _ = struct.unpack(RX_FORMAT, candidate)
            if (
                control > 1
                or shoot > 1
                or not math.isfinite(yaw)
                or not math.isfinite(pitch)
                or abs(yaw) > 2 * math.pi
                or abs(pitch) > 2 * math.pi
            ):
                self.field_errors += 1
                del self.buffer[:RX_SIZE]
                continue
            self.valid_packets.append((control, shoot, yaw, pitch))
            del self.buffer[:RX_SIZE]


class XucV2ProtocolTest(unittest.TestCase):
    def test_packet_sizes(self) -> None:
        self.assertEqual(RX_SIZE, 14)
        self.assertEqual(TX_SIZE, 20)

    def test_safe_command_crc_vector(self) -> None:
        payload = bytes.fromhex("535000000000000000000000")
        self.assertEqual(crc16(payload), 0xE637)
        self.assertEqual(with_crc(payload).hex(), "53500000000000000000000037e6")

    def test_feedback_crc_vector(self) -> None:
        payload = bytes.fromhex("53500103000068410100cdcccc3d0000003f")
        self.assertEqual(crc16(payload), 0xEC87)
        self.assertEqual(with_crc(payload)[-2:].hex(), "87ec")

    def test_split_noise_corruption_and_recovery(self) -> None:
        first = with_crc(struct.pack("<2sBBff", b"SP", 0, 0, 0.25, -0.5))
        second = with_crc(struct.pack("<2sBBff", b"SP", 1, 0, -1.0, 0.75))
        corrupt = bytearray(first)
        corrupt[-1] ^= 0x5A

        parser = ReferenceStreamParser()
        parser.feed(b"noiseS")
        parser.feed(first[:3])
        parser.feed(first[3:] + bytes(corrupt) + second)

        self.assertEqual(len(parser.valid_packets), 2)
        self.assertEqual(parser.crc_errors, 1)
        self.assertEqual(parser.field_errors, 0)
        self.assertAlmostEqual(parser.valid_packets[0][2], 0.25)
        self.assertAlmostEqual(parser.valid_packets[1][3], 0.75)

    def test_nonfinite_command_is_rejected(self) -> None:
        packet = with_crc(struct.pack("<2sBBff", b"SP", 1, 0, float("nan"), 0.0))
        parser = ReferenceStreamParser()
        parser.feed(packet)
        self.assertEqual(parser.valid_packets, [])
        self.assertEqual(parser.field_errors, 1)

    def test_implausible_angle_is_rejected(self) -> None:
        packet = with_crc(struct.pack("<2sBBff", b"SP", 1, 0, 1000.0, 0.0))
        parser = ReferenceStreamParser()
        parser.feed(packet)
        self.assertEqual(parser.valid_packets, [])
        self.assertEqual(parser.field_errors, 1)


if __name__ == "__main__":
    result = unittest.main(exit=False)
    if result.result.wasSuccessful():
        print("XUC_V2_PROTOCOL_TEST=PASS")
    raise SystemExit(0 if result.result.wasSuccessful() else 1)

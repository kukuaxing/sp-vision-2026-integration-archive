#!/usr/bin/env python3
"""Read-only smoke test for lower-board SP/SV telemetry."""

from __future__ import annotations

import argparse
import collections
import time

import serial


def crc16(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if crc & 1 else crc >> 1
    return crc & 0xFFFF


def extract_frames(data: bytes) -> tuple[collections.Counter[str], int, int]:
    counts: collections.Counter[str] = collections.Counter()
    valid = invalid = 0
    cursor = 0
    while cursor + 2 <= len(data):
        if data[cursor : cursor + 2] == b"SP":
            length, name = 20, "SP-feedback"
        elif data[cursor : cursor + 2] == b"SV" and cursor + 5 <= len(data):
            length = data[cursor + 4]
            name = f"SV-type-{data[cursor + 3]}"
            if length not in (50, 66):
                cursor += 1
                continue
        else:
            cursor += 1
            continue
        if cursor + length > len(data):
            break
        frame = data[cursor : cursor + length]
        received = int.from_bytes(frame[-2:], "little")
        if crc16(frame[:-2]) == received:
            valid += 1
            counts[name] += 1
            cursor += length
        else:
            invalid += 1
            cursor += 1
    return counts, valid, invalid


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()

    captured = bytearray()
    deadline = time.monotonic() + args.seconds
    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        port.reset_input_buffer()
        while time.monotonic() < deadline:
            captured.extend(port.read(4096))
    counts, valid, invalid = extract_frames(bytes(captured))
    print(f"captured_bytes={len(captured)} valid_frames={valid} invalid_candidates={invalid}")
    for name, count in sorted(counts.items()):
        print(f"{name}={count}")
    if valid == 0:
        raise SystemExit("No valid SP/SV telemetry found; verify the debugger TX/RX wiring to UART4.")


if __name__ == "__main__":
    main()

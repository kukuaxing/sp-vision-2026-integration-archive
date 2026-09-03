#!/usr/bin/env python3
import os
import pty
import struct
import subprocess
import sys
import time
import tty
from pathlib import Path


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def make_feedback(counter, corrupt=False):
    payload = struct.pack(
        "<2sBBfHff",
        b"SP",
        1,
        3,
        14.5,
        counter & 0xFFFF,
        0.10,
        0.50,
    )
    packet = payload + struct.pack("<H", crc16(payload))
    if corrupt:
        packet = packet[:-1] + bytes([packet[-1] ^ 0x80])
    return packet


def main():
    repo = Path(__file__).resolve().parents[1]
    executable = repo / "build/xuc_board_probe"
    if not executable.exists():
        print("[FAIL] build/xuc_board_probe不存在")
        return 2

    master_fd, slave_fd = pty.openpty()
    tty.setraw(master_fd)
    tty.setraw(slave_fd)
    slave_name = os.ttyname(slave_fd)
    print(f"[PTY] virtual serial: {slave_name}")

    process = subprocess.Popen(
        [str(executable), slave_name, "460800", "2.0"],
        cwd=repo,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    start = time.monotonic()
    counter = 0
    valid_sent = 0
    corrupt_sent = 0
    noise_sent = False

    while process.poll() is None:
        elapsed = time.monotonic() - start
        if elapsed > 4.0:
            process.kill()
            print("[FAIL] 探测程序超时")
            break

        if not noise_sent:
            try:
                os.write(master_fd, b"\x00\xff\x53")
                os.write(master_fd, make_feedback(0, corrupt=True))
                corrupt_sent += 1
                noise_sent = True
            except OSError:
                time.sleep(0.01)
                continue

        counter += 1
        packet = make_feedback(counter)
        try:
            os.write(master_fd, packet[:7])
            time.sleep(0.002)
            os.write(master_fd, packet[7:])
            valid_sent += 1
        except OSError:
            if process.poll() is None:
                time.sleep(0.01)
                continue
            break
        time.sleep(0.018)

    output = process.stdout.read() if process.stdout is not None else ""
    return_code = process.wait()
    os.close(slave_fd)
    os.close(master_fd)

    print("===== 探测程序输出 =====")
    print(output, end="")
    print("===== 虚拟反馈统计 =====")
    print(f"VALID_SENT={valid_sent}")
    print(f"CORRUPT_SENT={corrupt_sent}")
    print(f"PROBE_EXIT={return_code}")

    passed = (
        return_code == 0
        and valid_sent >= 20
        and "XUC_PROBE_MODE=RECEIVE_ONLY" in output
        and "XUC_PROBE_RESULT=PASS" in output
        and "mode=1" in output
        and "robot_id=3" in output
        and "RX_BYTES=" in output
        and "CRC_ERRORS=1" in output
        and "VALID_PACKETS=" in output
    )
    print("XUC_VIRTUAL_PROBE_RESULT=" + ("PASS" if passed else "FAIL"))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())

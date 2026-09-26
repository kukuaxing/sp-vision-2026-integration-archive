#!/usr/bin/env python3
"""Linux-only, low-priority RMV serial acceptance using a newly created private PTY.

No camera, real serial path, ROS process, service, navigation state or network.
One small g++ build (-O0) at a time; artifacts stay in build/offline-linux.
The kernel PTY exercises byte-stream I/O, not USB hardware timing or motor behavior.
"""
import errno
import json
import os
from pathlib import Path
import select
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    if sys.platform != "linux":
        raise SystemExit("Run this optional PTY acceptance on Linux; no hardware fallback exists.")
    import pty
    from tools.rmv.protocol import StreamDecoder
    from tools.rmv.simulator import Board

    for tool in ("g++", "nice", "ionice"):
        if not shutil.which(tool):
            raise SystemExit(f"Missing {tool}; no package installation is performed.")
    current_nice = os.getpriority(os.PRIO_PROCESS, 0)
    if current_nice < 19:
        os.nice(19 - current_nice)
    subprocess.run(["ionice", "-c", "3", "-p", str(os.getpid())], check=True)
    build = ROOT / "build/offline-linux"
    build.mkdir(parents=True, exist_ok=True)
    binary = build / "rmv_pty_peer"
    command = ["nice", "-n", "19", "ionice", "-c", "3", "g++", "-std=c++17", "-O0",
               "-UNDEBUG", "-Wall", "-Wextra", "-I.", "-Iio/serial/include",
               "tests/rmv/pty_peer.cpp", "io/serial/src/serial.cc", "io/serial/src/impl/unix.cc",
               "-pthread", "-lrt", "-o", str(binary)]
    print("+ " + " ".join(command), flush=True)
    subprocess.run(command, cwd=ROOT, check=True, timeout=120)
    refused = subprocess.run([str(binary), "--pty-offline", "/dev/ttyUSB0"], capture_output=True)
    if refused.returncode != 2:
        raise AssertionError("PTY executable did not refuse a physical serial path")

    master, slave = pty.openpty()
    child = None
    try:
        path = os.ttyname(slave)
        if not path.startswith("/dev/pts/") or not path[9:].isdigit():
            raise AssertionError("Unexpected PTY path")
        os.set_blocking(master, False)
        board = Board()
        start = time.monotonic()
        board_elapsed = 0
        queued = bytearray()
        child = subprocess.Popen([str(binary), "--pty-offline", path], cwd=ROOT,
                                 text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        decoder = StreamDecoder()
        injected = False
        fragments = reads = targets = 0
        sizes = (1, 7, 31, 127, 511)
        while child.poll() is None and time.monotonic() - start < 8:
            elapsed = int((time.monotonic() - start) * 1e6)
            board.advance(max(0, elapsed - board_elapsed))
            board_elapsed = elapsed
            readable, _, _ = select.select([master], [], [], 0.001)
            if readable:
                try:
                    data = os.read(master, 4096)
                except BlockingIOError:
                    data = b""
                except OSError as error:
                    if error.errno != errno.EIO:
                        raise
                    data = b""
                if data:
                    reads += 1
                    frames = decoder.feed(data, now=time.monotonic())
                    targets += sum(frame.msg_type == 0x0201 for frame in frames)
                    response = board.receive(data)
                    if response and not injected:
                        corrupt = bytearray(response)
                        corrupt[-1] ^= 1
                        queued.extend(b"\x00offline-noise\x01" + corrupt)
                        injected = True
                    queued.extend(response)
            queued.extend(board.telemetry())
            if len(queued) > 65536:
                raise AssertionError("PTY test receive queue grew without bound")
            if queued:
                try:
                    count = os.write(master, queued[:sizes[fragments % len(sizes)]])
                except BlockingIOError:
                    count = 0
                if count:
                    del queued[:count]
                    fragments += 1
        if child.poll() is None:
            child.kill()
            raise TimeoutError("PTY peer did not complete")
        output, errors = child.communicate(timeout=2)
        if child.returncode:
            raise RuntimeError(f"PTY peer exit {child.returncode}: {errors.strip()}")
        result = json.loads(output)
        assert result["success"] and result["stop_acked"] and result["targets"] >= 5
        assert targets == result["targets"] and board.mode == 0 and board.target is None
        assert injected and fragments > 10 and reads > 5
        result.update({"host_packet_reads": reads, "fragment_writes": fragments,
                       "corrupt_reply_and_noise_injected": injected, "targets_received": targets,
                       "simulated_stop_confirmed": board.mode == 0 and board.target is None,
                       "physical_timing_or_motor_validation": False})
        report = ROOT / "logs/rmv/pty_acceptance.json"
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2))
        print("PASS: private PTY, C++ Session + repository serial driver + Python Board; no hardware.")
    finally:
        if child is not None and child.poll() is None:
            child.kill()
            child.communicate(timeout=2)
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()

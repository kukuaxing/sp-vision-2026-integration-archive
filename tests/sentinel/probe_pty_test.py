"""Record with the real probe and a private PTY; never opens a robot port."""
import errno
import json
import math
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
import tempfile
import time
import tty

from pty_test import crc16, frame


def test_record(binary):
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    captured = bytearray()

    def drain():
        while select.select([master], [], [], 0)[0]:
            try:
                block = os.read(master, 4096)
            except OSError as error:
                if error.errno == errno.EIO:
                    break
                raise
            if not block:
                break
            captured.extend(block)

    try:
        with tempfile.TemporaryDirectory(prefix="sentinel-probe-record-") as folder:
            output_path = Path(folder) / "attitude.jsonl"
            command = [binary, os.ttyname(slave), "921600", "0.9", "--sentinel",
                       "--record-jsonl", str(output_path)]
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            start = time.monotonic()
            index = 0
            next_tx = start + 0.15
            boot = 0x11223344
            sent = {}
            try:
                while process.poll() is None:
                    now = time.monotonic()
                    assert now - start < 4, "recording probe hung"
                    if index < 20 and now >= next_tx:
                        stamp = 1_000_000 + index * 20_000
                        p = struct.pack("<BBHI6f", 1, 3, 7, stamp,
                                        math.radians(10 + index), math.radians(-2),
                                        math.radians(35), 0.1, 0.2, 0.3)
                        c = frame(2, index, boot, struct.pack("<BBHI3f", 2, 1, 8, stamp,
                                                             0.01, 0.02, 0.03))
                        st = frame(0x81, index, boot,
                                   struct.pack("<IHBBHH", 0, 123, 0x8E, 0, 22, 0))
                        g = frame(1, index * 2, boot, p)
                        # Duplicate packet and a newer sequence with a frozen
                        # source time must both be excluded from the record.
                        duplicate_time = frame(1, index * 2 + 1, boot, p)
                        wire = c + st + g + g + duplicate_time
                        if index == 0:
                            bad = bytearray(g)
                            bad[-1] ^= 1
                            wire = bytes(bad) + wire
                        os.write(master, wire[:11])
                        time.sleep(0.001)
                        os.write(master, wire[11:])
                        sent[stamp] = index
                        index += 1
                        next_tx += 0.02
                    drain()
                    time.sleep(0.001)
                output = process.communicate(timeout=2)[0].decode(errors="replace")
                drain()  # Also capture anything written during destruction/exit.
                assert process.returncode == 0, output
                assert captured == b"", f"receive-only wrote {bytes(captured)!r}"
                rows = [json.loads(line) for line in output_path.read_text().splitlines()]
                samples = [row for row in rows if row["type"] == "gimbal"]
                assert len(samples) == len(sent) == 20, (len(samples), len(sent), output)
                assert [row["source_time_us"] for row in samples] == list(sent)
                for row in samples:
                    index = sent[row["source_time_us"]]
                    assert (row["boot_id"], row["seq"], row["epoch"], row["gimbal_flags"]) == (
                        boot, index * 2, 7, 3)
                    assert math.isclose(row["raw_rpy_deg"][0], 10 + index, abs_tol=1e-4)
                    assert math.isclose(row["raw_rpy_deg"][1], -2, abs_tol=1e-4)
                    assert math.isclose(row["raw_rpy_deg"][2], 35, abs_tol=1e-4)
                    assert all(math.isclose(a, b, abs_tol=1e-6)
                               for a, b in zip(row["raw_gyro_rad_s"], [0.1, 0.2, 0.3]))
                    assert row["chassis"]["flags"] == 1
                    assert row["chassis"]["source_time_us"] == row["source_time_us"]
                    assert row["mcu_status"]["state"] == 0x8E
                    assert row["mcu_status"]["aim_last_seq"] == 123
                assert all(a["elapsed_s"] <= b["elapsed_s"] for a, b in zip(samples, samples[1:]))
                summary = rows[-1]
                assert summary["type"] == "summary" and summary["sample_count"] == 20
                assert summary["completed"] and summary["crc_errors"] >= 1
                assert summary["tx_bytes"] == 0 and not summary["transmitter_constructed"]
                assert "TX_BYTES=0" in output
                original = output_path.read_bytes()
                rejected = subprocess.run(command, capture_output=True, timeout=2)
                assert rejected.returncode == 2 and output_path.read_bytes() == original
                drain()
                assert not captured
                conflicting = subprocess.run(command + ["--safe-heartbeat"], capture_output=True, timeout=2)
                assert conflicting.returncode == 2 and output_path.read_bytes() == original
                drain()
                assert not captured
                print("PASS probe PTY: every fresh sample, duplicate suppression, raw units, "
                      "AUTO feedback, CRC statistics, no overwrite, exclusive mode, 0 TX including exit")
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
    finally:
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    test_record(str(Path(sys.argv[1]).resolve()))

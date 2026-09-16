#!/usr/bin/env python3
"""Exercise the real receive-only capture binary through a pseudo-terminal."""
import os
from pathlib import Path
import pty
import struct
import subprocess
import tempfile
import time


def crc16(data):
    value = 0
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xffff
    return value


def frame(sequence, moving=False, corrupt=False):
    data = bytearray(82)
    data[:4] = bytes.fromhex('5aa54c00')
    data[6] = 0x91
    struct.pack_into('<I', data, 14, sequence)
    struct.pack_into('<f', data, 30, 10.0 if moving else 0.0)
    struct.pack_into('<ffff', data, 66, 1.0, 0.0, 0.0, 0.0)
    struct.pack_into('<H', data, 4, crc16(data[:4]+data[6:]))
    if corrupt:
        data[20] ^= 1
    return data


def check(mode, fresh, stationary):
    master, slave = pty.openpty()
    with tempfile.TemporaryDirectory(prefix='calibration-pose-test-') as tmp:
        cfg = Path(tmp)/'test.yaml'
        cfg.write_text('calibration_pose_source: hipnuc\nxuc_serial_baud: 460800\n'
                       f'xuc_serial_port: {os.ttyname(slave)}\n')
        proc = subprocess.Popen(['./build/capture', str(cfg), '--pose-check=1.5'],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        start = time.monotonic()
        sequence = 100
        try:
            while proc.poll() is None and time.monotonic()-start < 4:
                elapsed = time.monotonic()-start
                if elapsed > .15 and (mode != 'stale' or elapsed < .4):
                    data = frame(100 if mode == 'repeated' else sequence,
                                 moving=mode == 'moving', corrupt=mode == 'bad_crc')
                    # Noise and arbitrary read boundaries must not lose framing.
                    os.write(master, b'noise'+data[:17])
                    os.write(master, data[17:])
                    sequence += 10
                time.sleep(.01)
            out, err = proc.communicate(timeout=2)
            assert proc.returncode == (0 if fresh else 2), (mode, out, err)
            assert f'fresh={str(fresh).lower()}' in out, (mode, out)
            assert f'stationary={str(stationary).lower()}' in out, (mode, out)
            print('PASS', mode, out.strip())
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
            os.close(master)
            os.close(slave)


for scenario in [('valid', True, True), ('bad_crc', False, False),
                 ('stale', False, False), ('moving', True, False),
                 ('repeated', False, False)]:
    check(*scenario)

#!/usr/bin/env python3
"""Exercise the real receive-only capture binary through a pseudo-terminal."""
import os
import json
from pathlib import Path
import pty
import struct
import subprocess
import tempfile
import time
import select
import sys
import argparse

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--rmv-case', help='Run one named RMV scenario while debugging')
args = parser.parse_args()

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.rmv.protocol import Frame, StreamDecoder, encode, fixed_payload, unpack_fixed


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
    if not args.rmv_case:
        check(*scenario)


def check_rmv(mode, fresh, stationary, seconds=1.6):
    """Independent MCU wire fixture; open only this newly allocated PTY."""
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    proc = None
    with tempfile.TemporaryDirectory(prefix='rmv-calibration-test-') as tmp:
        cfg = Path(tmp) / 'test.yaml'
        cfg.write_text('calibration_pose_source: rmv\nxuc_serial_baud: 460800\n'
                       f'xuc_serial_port: {os.ttyname(slave)}\n')
        try:
            proc = subprocess.Popen([str(ROOT / 'build/capture'), str(cfg), f'--pose-check={seconds}'],
                                    cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            decoder = StreamDecoder()
            start = time.monotonic()
            seq = counter = hello_count = 0
            session = 0
            pending = bytearray()
            packets = []
            frozen_stamp = None
            fragment = 0
            status_at = pose_at = 0

            def transmit(kind, payload, flags=0, corrupt=False, wrong_session=False):
                nonlocal seq
                seq += 1
                stamp = int((time.monotonic() - start) * 1e6) + 5_000_000
                data = bytearray(encode(Frame(kind, seq, session + int(wrong_session), stamp, payload, flags)))
                if corrupt:
                    data[-1] ^= 1
                pending.extend(data)

            while proc.poll() is None and time.monotonic() - start < seconds + 3:
                elapsed = time.monotonic() - start
                stamp = int(elapsed * 1e6) + 5_000_000
                ready, _, _ = select.select([master], [], [], .001)
                if ready:
                    data = os.read(master, 65536)
                    for packet in decoder.feed(data):
                        packets.append(packet.msg_type)
                        assert packet.msg_type in (1, 2, 3), ('non-diagnostic command', mode, packet)
                        if packet.msg_type == 1:
                            hello_count += 1
                            assert packet.flags == 1 and packet.session_id == 0, ('invalid HELLO', packet)
                            nonce, caps, *_ = unpack_fixed('HELLO', packet.payload)
                            assert caps == 1 << 10
                            if mode == 'hello_rejected':
                                transmit(0x00FF, fixed_payload('ACK', packet.seq, 1, 4, 0, 0), 2)
                                continue
                            session = 7
                            payload = fixed_payload('HELLO_REPLY', nonce, 99, 2026100402,
                                                    0x12 if mode == 'missing_cap' else 0xC12,
                                                    0, 0, 256, 2, 3)
                            # CRC damage and arbitrary fragments precede a valid handshake.
                            pending.extend(b'noise')
                            transmit(0x8001, payload, 2, corrupt=True)
                            transmit(0x8001, payload, 2)
                        elif packet.msg_type == 2:
                            if mode != 'clock_stale' or elapsed < .5:
                                t1, = unpack_fixed('TIME_SYNC', packet.payload)
                                reply_stamp = int((time.monotonic() - start) * 1e6) + 5_000_000
                                transmit(0x8002, fixed_payload('TIME_REPLY', t1, reply_stamp, reply_stamp), 2)
                        else:
                            assert packet.payload == b'\0\0\0\0'
                stopped = mode == 'link_loss' and elapsed > .65
                if session and not stopped and mode != 'missing_cap':
                    geometry = int(mode == 'geometry_change' and elapsed > .9)
                    if elapsed >= status_at:
                        transmit(0x0101, fixed_payload('STATUS', 0, 1, 7, 7, 0, 1, 0, 0,
                                                     0, 0, 0, 0, 0, 0, geometry))
                        status_at = elapsed + .05
                    if elapsed >= pose_at and (mode != 'stale' or elapsed < .65):
                        pose_at = elapsed + .01
                        counter += 1
                        sensor = 2 if mode == 'wrong_sensor' else 1
                        flags = 54 if mode == 'invalid_flags' else 55
                        if mode == 'time_flags_conflict':
                            flags |= 8
                        epoch = 2 if mode == 'epoch_change' and elapsed > .9 else 1
                        sample_stamp = stamp
                        if mode == 'stale_time': sample_stamp -= 1_000_000
                        if mode == 'future_time': sample_stamp += 1_000_000
                        if mode == 'repeated_time':
                            if frozen_stamp is None and elapsed > .3: frozen_stamp = stamp
                            if frozen_stamp is not None: sample_stamp = frozen_stamp
                        q = [0.9238795325, 0, 0.3826834324, 0]
                        if mode == 'zero_quaternion': q = [0, 0, 0, 0]
                        if mode == 'slow_drift':
                            import math
                            q = [math.cos(elapsed / 2), 0, 0, math.sin(elapsed / 2)]
                        payload = fixed_payload('POSE', sensor, sensor, flags,
                                                1 if mode == 'repeated_counter' else counter,
                                                sample_stamp, counter * 10000, epoch, *q,
                                                .1 if mode == 'moving' else 0, 0, 0, 0, 0, 9.81)
                        transmit(0x0102, payload, corrupt=mode == 'bad_crc',
                                 wrong_session=mode == 'wrong_session')
                if stopped:
                    pending.clear()
                if pending:
                    # Keep fragmentation bounded while avoiding a growing fixture backlog.
                    size = (3, 17, 511, 4096)[fragment % 4]
                    count = os.write(master, pending[:size])
                    del pending[:count]
                    fragment += 1
                assert len(pending) < 65536, (mode, 'fixture backlog')
            out, err = proc.communicate(timeout=2)
            reports = [json.loads(line[4:]) for line in out.splitlines() if line.startswith('RMV ')]
            assert len(reports) == 1, (mode, out, err)
            report = reports[0]
            assert proc.returncode == (0 if fresh else 2), (mode, report, err)
            assert report['fresh'] == fresh and report['stationary'] == stationary, (mode, report)
            assert hello_count == 1, (mode, 'unexpected reconnect')
            if mode == 'hello_rejected':
                assert report['error'] == 'RMV HELLO rejected: BAD_VALUE (4), detail=0', report
            if fresh:
                assert report['sensor_id'] == report['frame_id'] == 1
                assert report['boot_id'] == 99 and report['valid_flags'] == 55
                if mode != 'slow_drift':
                    assert abs(report['quaternion_wxyz'][0] - .9238795325) < 1e-6
                    assert abs(report['quaternion_wxyz'][2] - .3826834324) < 1e-6
            print('PASS RMV', mode, f"samples={report['accepted_samples']} fresh={fresh} stationary={stationary}", flush=True)
        finally:
            if proc is not None and proc.poll() is None:
                proc.kill(); proc.communicate(timeout=2)
            os.close(master); os.close(slave)


for mode, fresh, stationary in [
    ('valid', True, True), ('moving', True, False), ('slow_drift', True, False),
    ('bad_crc', False, False), ('invalid_flags', False, False), ('zero_quaternion', False, False),
    ('stale', False, False), ('stale_time', False, False), ('future_time', False, False),
    ('repeated_counter', False, False), ('repeated_time', False, False),
    ('wrong_sensor', False, False), ('wrong_session', False, False),
    ('time_flags_conflict', False, False), ('epoch_change', False, False),
    ('geometry_change', False, False), ('link_loss', False, False), ('missing_cap', False, False),
    ('hello_rejected', False, False),
]:
    if not args.rmv_case or args.rmv_case == mode:
        check_rmv(mode, fresh, stationary)
if not args.rmv_case or args.rmv_case == 'clock_stale':
    check_rmv('clock_stale', False, False, seconds=4)


def check_capture_gates():
    # Every case must fail before camera/serial construction. The port is fictitious.
    with tempfile.TemporaryDirectory(prefix='capture-gate-test-') as tmp:
        folder = Path(tmp)
        cfg = folder / 'config.yaml'
        dataset = folder / 'dataset'; dataset.mkdir()
        base = ('pattern_cols: 11\npattern_rows: 8\nrequire_cboard: false\nxuc_enable: false\n'
                'xuc_serial_port: /nonexistent-private-test-device\nxuc_serial_baud: 460800\n')

        def invoke(arguments, message):
            result = subprocess.run([str(ROOT/'build/capture'), str(cfg), *arguments], cwd=ROOT,
                                    capture_output=True, text=True, timeout=5)
            assert result.returncode == 2 and message in result.stderr, (arguments, result.stdout, result.stderr)

        for source in ('rmv', 'hipnuc'):
            cfg.write_text(base+f'calibration_pose_source: {source}\ncalibration_pose_verified: false\n')
            invoke([], 'Verify small-Y AND pitch')
        cfg.write_text(base+'calibration_pose_source: rmv\ncalibration_pose_verified: true\n')
        invoke([], 'Verify small-Y AND pitch')
        cfg.write_text(base+'calibration_pose_source: rmv\ncalibration_pose_verified: false\ncalibration_raw_pose_verified: true\n')
        (dataset/'dataset_kind.txt').write_text('image_and_pose\n')
        (dataset/'pose_source.txt').write_text('hipnuc_uart3_wxyz\n')
        invoke([f'--output-folder={dataset}'], 'Different pose source')
        invoke(['--intrinsics-only', f'--output-folder={dataset}'], 'Dataset mode differs')
        for duration in ('0', '-1', 'nan', '31'):
            invoke([f'--pose-check={duration}'], 'duration must be finite')
        invoke(['--pose-check=1', '--intrinsics-only'], 'separate operations')
    print('PASS: unverified mounting, mixed datasets and invalid CLI durations fail before device IO')


if not args.rmv_case:
    check_capture_gates()
print('All calibration PTY checks passed. No physical camera or serial opened.')

#!/usr/bin/env python3
"""Bounded diagnostic serial session: HELLO, idle heartbeat, time sync only.

Does not send mode/aim/shoot/parameter commands or start vision. Run explicitly
on the robot computer with the operator ready for communication diagnostics.
"""
import argparse
import array
import collections
import fcntl
import json
import os
from pathlib import Path
import secrets
import select
import sys
import termios
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.rmv.protocol import Frame, StreamDecoder, encode, fixed_payload, unpack_fixed, JOINT_HEADER, JOINT_ENTRY
from report_rmv_health import build_report

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=8, help='Capture duration, 1 to 120 seconds')
args = parser.parse_args()
if not 1 <= args.seconds <= 120:
    parser.error('--seconds must be between 1 and 120')

port = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
folder = ROOT / 'logs/serial-probe' / time.strftime('%Y%m%d_%H%M%S_diagnostic')
folder.mkdir(parents=True, exist_ok=False)
fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
old = termios.tcgetattr(fd)
nonce = secrets.randbits(64)
decoder = StreamDecoder()
counts = collections.Counter()
samples = collections.defaultdict(list)
session = 0
seq = 0
tx_bytes = 0
rx_bytes = 0
status_keys = ['mode', 'owner', 'available_axis_mask', 'enabled_axis_mask', 'homed_axis_mask', 'remote_online', 'vision_permit', 'fire_permit', 'fault_mask', 'last_received_aim_seq', 'last_applied_aim_seq', 'last_aim_result', 'constraint_flags', 'param_rev', 'geometry_rev']
summary = {'port': port, 'baud': 460800, 'sent_commands': [], 'hello': None, 'last_status': None, 'last_joints': None, 'last_pose': {}, 'time_reply': None}
summary.update(requested_seconds=args.seconds, read_calls=0, max_read_interval_ms=0,
               max_decode_ms=0, receive_errors=[], diagnostics_first=None)


def serial_counters():
    """Linux serial_icounter_struct; driver support varies, zero is not proof of no loss."""
    counters = array.array('i', [0] * 20)
    try:
        fcntl.ioctl(fd, getattr(termios, 'TIOCGICOUNT', 0x545D), counters, True)
    except OSError as error:
        return {'unavailable': str(error)}
    keys = ['cts', 'dsr', 'rng', 'dcd', 'rx', 'tx', 'frame', 'overrun', 'parity', 'brk', 'buf_overrun']
    return dict(zip(keys, counters))


def send(kind, payload, flags=0):
    global seq, tx_bytes
    seq += 1
    data = encode(Frame(kind, seq, session, time.monotonic_ns()//1000, payload, flags))
    sent = os.write(fd, data)
    tx_bytes += sent
    if sent != len(data):
        raise RuntimeError('Partial serial write; not retried')
    name = {1: 'HELLO', 2: 'TIME_SYNC', 3: 'HEARTBEAT(host_state=0)'}[kind]
    if name not in summary['sent_commands']:
        summary['sent_commands'].append(name)


try:
    fcntl.ioctl(fd, termios.TIOCEXCL)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[4] = a[5] = termios.B460800
    a[6][termios.VMIN] = a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    # Discard bytes left by an earlier session before starting this diagnostic.
    # Errors received after HELLO are still counted; no retry or suppression.
    termios.tcflush(fd, termios.TCIFLUSH)
    summary['serial_counters_before'] = serial_counters()
    send(1, fixed_payload('HELLO', nonce, 0, 1, 1, 0), flags=1)
    start = time.monotonic()
    heartbeat_at = 0
    synced = False
    last_read_at = start
    with (folder / 'rx.bin').open('wb') as raw:
        while time.monotonic() - start < args.seconds:
            if session and time.monotonic() >= heartbeat_at:
                send(3, fixed_payload('HEARTBEAT', 0, b'\0'*3))
                heartbeat_at = time.monotonic() + .1
                if not synced:
                    send(2, fixed_payload('TIME_SYNC', time.monotonic_ns()//1000))
                    synced = True
            ready, _, _ = select.select([fd], [], [], .02)
            data = os.read(fd, 65536) if ready else b''
            if ready and not data:
                raise RuntimeError('Serial device disconnected')
            raw.write(data)
            rx_bytes += len(data)
            read_at = time.monotonic()
            if data:
                summary['read_calls'] += 1
                summary['max_read_interval_ms'] = max(summary['max_read_interval_ms'], (read_at-last_read_at)*1000)
                last_read_at = read_at
            errors_before = decoder.crc_errors
            decoded = decoder.feed(data)
            summary['max_decode_ms'] = max(summary['max_decode_ms'], (time.monotonic()-read_at)*1000)
            if decoder.crc_errors != errors_before:
                summary['receive_errors'].append({'elapsed_s': read_at-start, 'crc_errors': decoder.crc_errors-errors_before, 'read_bytes': len(data)})
            for frame in decoded:
                kind = frame.msg_type
                counts[hex(kind)] += 1
                if kind == 0x8001:
                    values = unpack_fixed('HELLO_REPLY', frame.payload)
                    if values[0] != nonce or frame.session_id == 0:
                        continue
                    session = frame.session_id
                    summary['hello'] = dict(zip(['nonce', 'boot_id', 'firmware_version', 'capabilities', 'param_rev', 'geometry_rev', 'max_payload', 'imu_count', 'axis_count'], values))
                elif not session or frame.session_id != session:
                    continue
                elif kind == 0x101:
                    summary['last_status'] = dict(zip(status_keys, unpack_fixed('STATUS', frame.payload)))
                elif kind == 0x102:
                    v = unpack_fixed('POSE', frame.payload)
                    samples[v[0]].append((v[2], v[3], v[4]))
                    summary['last_pose'][v[0]] = {'flags': v[2], 'source_counter': v[3], 'sample_time_us': v[4], 'frame_epoch': v[6], 'quaternion_wxyz': v[7:11], 'gyro_rad_s': v[11:14], 'acceleration_m_s2': v[14:17]}
                elif kind == 0x103:
                    count, _, flags, timestamp, epoch = JOINT_HEADER.unpack_from(frame.payload)
                    assert len(frame.payload) == JOINT_HEADER.size + count * JOINT_ENTRY.size
                    keys = ['axis_id', 'state_bits', 'fault_code', 'angle_rad', 'velocity_rad_s', 'target_rad', 'target_velocity_rad_s', 'output_fraction', 'temperature_c', 'feedback_age_us']
                    summary['last_joints'] = [dict(zip(keys, JOINT_ENTRY.unpack_from(frame.payload, JOINT_HEADER.size+i*JOINT_ENTRY.size))) for i in range(count)]
                elif kind == 0x108:
                    summary['diagnostics'] = unpack_fixed('DIAGNOSTICS', frame.payload)
                    if summary['diagnostics_first'] is None:
                        summary['diagnostics_first'] = summary['diagnostics']
                elif kind == 0x8002:
                    summary['time_reply'] = unpack_fixed('TIME_REPLY', frame.payload)
except Exception as error:
    summary['capture_error'] = str(error)
finally:
    try:
        summary['serial_counters_after'] = serial_counters()
        try:
            termios.tcsetattr(fd, termios.TCSANOW, old)
        except (OSError, termios.error) as error:
            summary['restore_error'] = str(error)
    finally:
        os.close(fd)
    summary.update(bytes_received=rx_bytes, bytes_transmitted=tx_bytes, frames=dict(counts), crc_errors=decoder.crc_errors, format_errors=decoder.format_errors, session_id=session)
    summary.update(parser_timeouts=decoder.timeouts, parser_noise_bytes=decoder.noise_bytes)
    before, after = summary.get('serial_counters_before', {}), summary['serial_counters_after']
    if before and 'unavailable' not in before and 'unavailable' not in after:
        summary['serial_counter_deltas'] = {key: (after[key]-before[key]) & 0xFFFFFFFF for key in before}
    summary['pose_statistics'] = {k: {'frames': len(v), 'nonzero_flags': sum(bool(x[0]) for x in v), 'distinct_source_counters': len({x[1] for x in v}), 'first_counter': v[0][1], 'last_counter': v[-1][1]} for k, v in samples.items()}
    summary['exit_code'] = (1 if summary.get('capture_error') or summary.get('restore_error') or not (summary['hello'] and summary['last_status']) else
                            2 if decoder.crc_errors or decoder.format_errors or decoder.timeouts else 0)
    summary['telemetry_health'] = build_report(summary)
    (folder / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print('Report:', folder / 'summary.json')
    print(json.dumps(summary, indent=2))
raise SystemExit(summary['exit_code'])

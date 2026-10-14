"""Private Linux PTYs, never opens a physical serial port or starts a robot."""
import errno
import math
import os
import pty
import select
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def feedback(elapsed, index):
    packet = bytearray(64)
    packet[:2] = bytes((0x5a, 0x3f))  # blue, reset, firmware reserved=15
    stamp = (0xffffff00 + int(elapsed * 1000)) & 0xffffffff
    if 1.2 <= elapsed < 1.7:
        stamp = 12345  # repeated timestamps must eventually invalidate pose
    valid = 0 if .75 <= elapsed < 1.0 else 1
    identity = 2 if 2.7 <= elapsed < 3.0 else 1
    struct.pack_into('<6fIBB', packet, 4, 11, 22, 179, 1, 2, 3, stamp, identity, valid)
    struct.pack_into('<3fIBB', packet, 36, 44, 55, 66, stamp, 2, 1)
    struct.pack_into('<HH', packet, 56, index + 1, index)
    struct.pack_into('<H', packet, 62, crc16(packet[:62]))
    return packet


def run(binary, control, shoot):
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    with tempfile.TemporaryDirectory(prefix='sentinel-pty-') as folder:
        cfg = Path(folder) / 'peer.yaml'
        cfg.write_text(f'''board_protocol: SENTINEL
xuc_enable: true
xuc_serial_port: "{os.ttyname(slave)}"
xuc_serial_baud: 921600
xuc_allow_control: {str(control).lower()}
xuc_allow_shoot: {str(shoot).lower()}
xuc_imu_yaw_sign: -1
xuc_imu_pitch_sign: 1
sentinel_swap_roll_pitch: true
sentinel_tx_hz: 100
''')
        process = subprocess.Popen([binary, str(cfg)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        start = time.monotonic()
        next_feedback = .12
        rx = bytearray()
        outgoing = []
        index = 0
        try:
            while process.poll() is None:
                elapsed = time.monotonic() - start
                if elapsed > 8:
                    raise AssertionError('production sender/receiver hung')
                if elapsed >= next_feedback:
                    frame = feedback(elapsed, index)
                    if index == 0:
                        broken = bytearray(frame)
                        broken[12] ^= 1
                        os.write(master, b'\x00\xff\x5a' + broken)
                    # Deliberately split feedback mid-float, followed by remainder.
                    os.write(master, frame[:7])
                    time.sleep(.001)
                    os.write(master, frame[7:])
                    index += 1
                    next_feedback = elapsed + .05
                if select.select([master], [], [], .002)[0]:
                    try:
                        rx.extend(os.read(master, 4096))
                    except OSError as error:
                        if error.errno != errno.EIO:
                            raise
                    while len(rx) >= 40:
                        frame = bytes(rx[:40])
                        del rx[:40]
                        assert frame[0] == 0xa5 and crc16(frame[:38]) == struct.unpack_from('<H', frame, 38)[0]
                        outgoing.append((time.monotonic() - start, frame))
            output = process.communicate(timeout=3)[0].decode(errors='replace')
            assert process.returncode == 0, output
            assert 'normalized=1' in output and 'invalid_with_link=1' in output, output
            if not control:
                assert not outgoing, 'read-only mode sent a frame that could start friction wheels'
            else:
                assert 40 < len(outgoing) <= 325, 'TX exceeds configured 100Hz'
                assert all(b[22:25] == bytes(3) and b[29:38] == bytes(9) for _, b in outgoing)
                nonzero = [(t, b) for t, b in outgoing if struct.unpack_from('<f', b, 9)[0] != 0]
                for _, frame in nonzero:
                    pitch, yaw, dy, dp, distance = struct.unpack_from('<5f', frame, 1)
                    assert math.isclose(pitch, 11, abs_tol=.001) and math.isclose(yaw, 179, abs_tol=.001)
                    assert math.isclose(dy, 2, abs_tol=.001) and math.isclose(dp, 1.5, abs_tol=.001)
                    assert distance == 3
                assert not any(.86 < t < .98 or 1.55 < t < 1.68 or 2.46 < t < 2.57 or 2.81 < t < 2.98 for t, _ in nonzero)
                assert any(.25 < t < .7 for t, _ in nonzero) and any(1.9 < t < 2.35 for t, _ in nonzero)
                assert any(b[21] for _, b in outgoing) == shoot
                # Cancellation frames carry zero motion and fire; no legacy SP/RMV packets.
                assert any(b[1:38] == bytes(37) for _, b in outgoing)
            print(f'PASS production PTY control={control} shoot={shoot}: frames={len(outgoing)}; timeout, sensor validity, wrong ID, timestamp wrap/freeze, recovery, units, signs, fire gate')
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    for allow_control, allow_shoot in ((False, False), (True, False), (True, True)):
        run(str(Path(sys.argv[1]).resolve()), allow_control, allow_shoot)

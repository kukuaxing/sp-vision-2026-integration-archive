#!/usr/bin/env python3
import fcntl
import math
import os
import pty
import select
import struct
import subprocess
import sys
import time
import tty
from pathlib import Path

RX_SIZE = 14
TX_SIZE = 20

def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF

def make_feedback(counter, corrupt=False, imu_yaw=0.50, mode=1):
    payload = struct.pack(
        "<2sBBfHff",
        b"SP",
        mode,
        3,
        14.5,
        counter & 0xFFFF,
        0.10,
        imu_yaw,
    )
    packet = payload + struct.pack("<H", crc16(payload))
    if corrupt:
        packet = packet[:-1] + bytes([packet[-1] ^ 0x80])
    return packet

def close_enough(value, expected, tolerance=1e-4):
    return math.isfinite(value) and abs(value - expected) <= tolerance

def main():
    safety_profile = "--safety" in sys.argv[1:]
    repo = Path(__file__).resolve().parents[1]
    executable = repo / "build/xuc_link_test"

    if not executable.exists():
        print("[FAIL] build/xuc_link_test不存在")
        return 2

    master_fd, slave_fd = pty.openpty()
    tty.setraw(master_fd)
    tty.setraw(slave_fd)

    slave_name = os.ttyname(slave_fd)
    print(f"[PTY] virtual serial: {slave_name}")

    command_line = [str(executable), slave_name]
    if safety_profile:
        command_line.append("--safety")
    print("[PROFILE] " + ("SAFETY" if safety_profile else "PROTOCOL"))

    process = subprocess.Popen(
        command_line,
        cwd=repo,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    # 保持父进程的slave描述符打开，避免测试程序打开串口前
    # master端因暂时没有slave而收到EIO。
    flags = fcntl.fcntl(master_fd, fcntl.F_GETFL)
    fcntl.fcntl(master_fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

    start = time.monotonic()
    buffer = bytearray()

    command_count = 0
    outbound_crc_errors = 0
    field_errors = []
    safety_errors = []
    pre_feedback_checks = 0
    active_control_checks = 0
    stale_link_checks = 0
    recovery_control_checks = 0
    transient_dropout_checks = 0
    transient_recovery_checks = 0
    transient_dropout_started = False
    corrupt_feedback = 0
    valid_feedback = 0
    silence_commands = 0
    recovery_feedback = 0
    feedback_counter = 0
    noise_sent = False

    while process.poll() is None:
        elapsed = time.monotonic() - start

        if elapsed > 7.0:
            process.kill()
            print("[FAIL] 测试进程超时")
            break

        readable, _, _ = select.select(
            [master_fd], [], [], 0.05)

        if not readable:
            continue

        try:
            chunk = os.read(master_fd, 4096)
        except BlockingIOError:
            continue
        except OSError:
            if process.poll() is None:
                time.sleep(0.01)
                continue
            break

        if not chunk:
            continue

        buffer.extend(chunk)

        while True:
            start_index = buffer.find(b"SP")

            if start_index < 0:
                if len(buffer) > 1:
                    del buffer[:-1]
                break

            if start_index > 0:
                del buffer[:start_index]

            if len(buffer) < RX_SIZE:
                break

            packet = bytes(buffer[:RX_SIZE])
            del buffer[:RX_SIZE]

            expected_crc = crc16(packet[:12])
            received_crc = struct.unpack_from("<H", packet, 12)[0]

            if expected_crc != received_crc:
                outbound_crc_errors += 1
                continue

            (
                head,
                control,
                shoot,
                yaw,
                pitch,
                _,
            ) = struct.unpack("<2sBBffH", packet)

            command_count += 1

            if head != b"SP":
                field_errors.append("包头错误")

            if safety_profile:
                if shoot != 0:
                    safety_errors.append(f"{elapsed:.3f}s shoot={shoot}")

                if elapsed < 0.60:
                    pre_feedback_checks += 1
                    if control != 0:
                        safety_errors.append(f"{elapsed:.3f}s pre_feedback control={control}")
                elif 0.85 <= elapsed < 1.50:
                    # The C++ test and this process have independent monotonic
                    # clock origins.  Follow the observed 1 -> 0 -> 1 sequence
                    # instead of assigning packets to narrow absolute windows.
                    if control == 0:
                        transient_dropout_started = True
                        transient_dropout_checks += 1
                    elif transient_dropout_started:
                        transient_recovery_checks += 1
                    else:
                        active_control_checks += 1
                elif 2.10 <= elapsed < 2.35:
                    stale_link_checks += 1
                    if control != 0:
                        safety_errors.append(f"{elapsed:.3f}s stale control={control}")
                elif elapsed >= 2.75:
                    recovery_control_checks += 1
                    if control != 1:
                        safety_errors.append(f"{elapsed:.3f}s recovery control={control}")
            else:
                if control != 1:
                    field_errors.append(f"control={control}")
                if shoot != 1:
                    field_errors.append(f"shoot={shoot}")

            if safety_profile and control == 1:
                # The session initially anchors at feedback raw yaw=0.50.  After
                # a short command dropout feedback moves to 0.55, but the fixed
                # session bound must still produce 0.45 rather than reanchor.
                if not close_enough(yaw, 0.45):
                    field_errors.append(f"bounded yaw={yaw}")
                if not close_enough(pitch, 0.10):
                    field_errors.append(f"locked pitch={pitch}")
            else:
                if not close_enough(yaw, 0.30):
                    field_errors.append(f"yaw={yaw}")
                if not close_enough(pitch, 0.15):
                    field_errors.append(f"pitch={pitch}")

            elapsed = time.monotonic() - start
            feedback_counter += 1

            if elapsed < 0.70:
                os.write(
                    master_fd,
                    make_feedback(
                        feedback_counter,
                        corrupt=True))
                corrupt_feedback += 1
            elif elapsed < 1.50:
                feedback_yaw = 0.50 if elapsed < 0.95 else 0.55
                # Simulate the real-car mode acknowledgement chattering low
                # while the operator remains in the same physical AUTO session.
                feedback_mode = (
                    0 if safety_profile and 0.95 <= elapsed < 1.05 else 1)
                packet_out = make_feedback(
                    feedback_counter,
                    imu_yaw=feedback_yaw,
                    mode=feedback_mode)

                if not noise_sent:
                    os.write(master_fd, b"\x00\xff\x53")
                    noise_sent = True

                os.write(master_fd, packet_out[:5])
                time.sleep(0.005)
                os.write(master_fd, packet_out[5:])
                valid_feedback += 1
            elif elapsed < 2.40:
                silence_commands += 1
            else:
                packet_out = make_feedback(
                    feedback_counter, imu_yaw=0.55)
                os.write(master_fd, packet_out[:3])
                time.sleep(0.003)
                os.write(master_fd, packet_out[3:])
                valid_feedback += 1
                recovery_feedback += 1

    output = ""

    if process.stdout is not None:
        output = process.stdout.read()

    return_code = process.wait()
    os.close(slave_fd)
    os.close(master_fd)

    print("===== C++测试输出 =====")
    print(output, end="")

    print("===== 虚拟下位机统计 =====")
    print(f"COMMAND_COUNT={command_count}")
    print(f"OUTBOUND_CRC_ERRORS={outbound_crc_errors}")
    print(f"FIELD_ERRORS={len(field_errors)}")
    print(f"CORRUPT_FEEDBACK={corrupt_feedback}")
    print(f"VALID_FEEDBACK={valid_feedback}")
    print(f"SILENCE_COMMANDS={silence_commands}")
    print(f"RECOVERY_FEEDBACK={recovery_feedback}")
    print(f"PRE_FEEDBACK_CHECKS={pre_feedback_checks}")
    print(f"ACTIVE_CONTROL_CHECKS={active_control_checks}")
    print(f"STALE_LINK_CHECKS={stale_link_checks}")
    print(f"RECOVERY_CONTROL_CHECKS={recovery_control_checks}")
    print(f"TRANSIENT_DROPOUT_CHECKS={transient_dropout_checks}")
    print(f"TRANSIENT_RECOVERY_CHECKS={transient_recovery_checks}")
    print(f"SAFETY_ERRORS={len(safety_errors)}")
    print(f"CPP_EXIT={return_code}")

    if field_errors:
        for error in field_errors[:10]:
            print(f"FIELD_ERROR_DETAIL={error}")

    if safety_errors:
        for error in safety_errors[:10]:
            print(f"SAFETY_ERROR_DETAIL={error}")

    safety_passed = (
        not safety_profile
        or (
            not safety_errors
            and pre_feedback_checks >= 5
            and active_control_checks >= 2
            and transient_dropout_checks >= 2
            and transient_recovery_checks >= 2
            and stale_link_checks >= 2
            and recovery_control_checks >= 5
        )
    )

    passed = (
        return_code == 0
        and command_count >= 50
        and outbound_crc_errors == 0
        and not field_errors
        and corrupt_feedback >= 5
        and valid_feedback >= 10
        and silence_commands >= 5
        and recovery_feedback >= 5
        and safety_passed
    )

    print(
        "VIRTUAL_LOWER_RESULT="
        + ("PASS" if passed else "FAIL")
    )
    if safety_profile:
        print(
            "XUC_SAFETY_RESULT="
            + ("PASS" if safety_passed else "FAIL")
        )

    return 0 if passed else 1

if __name__ == "__main__":
    sys.exit(main())

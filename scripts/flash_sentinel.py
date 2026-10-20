#!/usr/bin/env python3
"""Explicit stopped-robot update of the identified Sentinel STM32F405.

Use the local pyOCD environment. Saves the original preservation scope, programs
application sectors only, verifies that entire scope, then resets. The default
scope is all 1 MiB; --application-sectors-only covers the full first 128 KiB.
Any failed verification leaves the target halted. No mass erase,
protection unlock, remote commands, actuator tests, or automatic retries.
"""
import argparse
import hashlib
import json
import logging
from pathlib import Path
import platform
import re
import struct
from datetime import datetime

from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target
from pyocd.flash.file_programmer import FileProgrammer

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_UID = "1b0037000647353330303436"


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--confirmed-stopped", action="store_true", required=True)
    cli.add_argument("--expected-sha256", required=True)
    cli.add_argument("--control-enabled", action="store_true",
                     help="Flash the explicitly authorized operational build instead of read-only commissioning")
    cli.add_argument("--running-layout", type=Path,
                     help="Read-only STOP/RESET check against the identified currently running build")
    cli.add_argument("--application-sectors-only", action="store_true",
                     help="Back up and verify the complete 128 KiB application erase region; never erase other sectors")
    cli.add_argument("--verify-snapshot", type=Path,
                     help="Verify an already programmed target against this saved original; never erase/write again")
    args = cli.parse_args()
    preservation_bytes = 0x20000 if args.application_sectors_only else 0x100000
    image = ROOT / "firmware/sentry/build/STM32F405.bin"
    firmware = image.read_bytes()
    digest = hashlib.sha256(firmware).hexdigest()
    assert digest == args.expected_sha256.lower(), "Image differs from reviewed build"
    assert 8 < len(firmware) <= 0x20000
    sp, pc = struct.unpack_from("<II", firmware)
    assert 0x20000000 <= sp <= 0x20020000 and 0x08000000 <= (pc & ~1) < 0x08020000 and pc & 1
    header = (ROOT / "firmware/sentry/STM32F405/xuc.h").read_text(encoding="utf-8")
    for name in ("XUC_SAMPLE_TIME_VERIFIED", "XUC_AXES_VERIFIED"):
        assert re.search(r"#define\s+" + name + r"\s+0\b", header), name
    assert re.search(r"#define\s+XUC_ALLOW_CONTROL\s+" + str(int(args.control_enabled)) + r"\b", header)
    reviewed = json.loads((ROOT / "logs/sentinel-v2-validation.json").read_text(encoding="utf-8"))
    assert reviewed["binary_sha256"] == digest and reviewed["control_enabled"] == args.control_enabled
    saved_original = None
    if args.verify_snapshot:
        snapshot = args.verify_snapshot.resolve()
        assert snapshot.is_relative_to((ROOT / "data/flash").resolve())
        previous = json.loads((snapshot.parent / "result.json").read_text(encoding="utf-8"))
        saved_original = snapshot.read_bytes()
        assert len(saved_original) == preservation_bytes
        assert hashlib.sha256(saved_original).hexdigest() == previous["original_sha256"]
        assert previous["image_sha256"] == digest and previous["uid"] == EXPECTED_UID
    # Avoid unbounded HID waiting on the wireless debug probe.
    if platform.system() == "Windows":
        from pyocd.probe.pydapaccess.interface.hidapi_backend import HidApiUSB
        from pyocd.probe.pydapaccess.dap_access_api import DAPAccessIntf

        def bounded_read(self):
            data = bytes(self.device.read(self.packet_size, self.DEFAULT_USB_TIMEOUT_MS))
            if not data:
                raise DAPAccessIntf.DeviceError("Probe HID reply timeout; no automatic reconnect")
            return data

        HidApiUSB.read = bounded_read
    out = ROOT / "data/flash" / datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=False)
    logging.basicConfig(level=logging.INFO, handlers=[logging.FileHandler(out / "pyocd.log", encoding="utf-8")])
    report = dict(image=str(image), image_bytes=len(firmware), image_sha256=digest,
                  probe="ATK 20190528", operator_confirmed_stopped=True,
                  control_enabled=args.control_enabled, physical_tests_started=False,
                  preservation_scope_bytes=preservation_bytes, erase_mode="application_sectors_only",
                  never_erase_at_or_above="0x08020000")
    if args.verify_snapshot:
        report.update(original_snapshot=str(args.verify_snapshot.resolve()), verification_only=True)

    def record(status):
        report["status"] = status
        (out / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(status, flush=True)

    def read_flash(target, label):
        data = bytearray()
        block_size = 256
        for offset in range(0, preservation_bytes, block_size):
            block = bytes(target.read_memory_block8(0x08000000 + offset, block_size))
            assert len(block) == block_size
            data.extend(block)
            if (offset + block_size) % 0x20000 == 0:
                print(f"{label}: {offset + block_size}/{preservation_bytes}", flush=True)
        return bytes(data)

    record("connecting")
    probe = ConnectHelper.choose_probe(unique_id="cmsisdap:ATK 20190528", blocking=False)
    assert probe is not None, "Expected probe not connected"
    # The exact MCU type and UID are checked below. Skip optional board metadata:
    # a failed temporary DAP_Info open can leave this wireless probe half-open.
    # Open it once through Session, with bounded HID reads and no retry loop.
    probe.create_associated_board = lambda: None
    session = Session(probe,
        options={"target_override": "stm32f405rg", "frequency": 250000,
                 "connect_mode": "attach" if args.running_layout else "halt",
                 "auto_unlock": False, "reset_type": "sysresetreq", "resume_on_disconnect": False,
                 "no_config": True, "cmsis_dap.limit_packets": True, "cmsis_dap.deferred_transfers": False,
                 "cache.enable_memory": False, "cache.enable_register": False})
    assert session is not None, "Expected probe not connected"
    print("Recovery evidence:", out, flush=True)
    try:
        with session:
            target = session.target
            chip, capacity = target.read32(0xE0042000), target.read16(0x1FFF7A22)
            uid = bytes(target.read_memory_block8(0x1FFF7A10, 12)).hex()
            report.update(chip_id=hex(chip), flash_kib=capacity, uid=uid)
            assert chip & 0xfff == 0x413 and capacity == 1024, "Unsupported target"
            assert uid == EXPECTED_UID, "Not the previously identified robot controller; no programming"
            if args.running_layout:
                running = json.loads(args.running_layout.read_text(encoding="utf-8"))
                assert target.get_state() == Target.State.RUNNING
                for _ in range(3):
                    mode_size = running["fields"]["mode"]["size_bytes"]
                    assert mode_size in (1, 2, 4), "Unsupported mode field width"
                    mode = target.read_memory(running["addresses"]["mode"], transfer_size=mode_size * 8)
                    assert mode in (running["mode_enum"]["STOP"], running["mode_enum"]["RESET"]), \
                        "Physical STOP/RESET is not active; no halt, erase or reset requested"
                report["preflash_mode"] = mode
            target.halt()
            report["flash_optcr"] = hex(target.read32(0x40023C14))
            if saved_original is None:
                record("saving_original_flash")
                original = read_flash(target, "Saved")
                if args.running_layout:
                    running_bytes = running.get("binary_size")
                    running_hash = running.get("binary_sha256")
                    assert running_bytes and running_hash and running_bytes <= preservation_bytes
                    assert hashlib.sha256(original[:running_bytes]).hexdigest() == running_hash, \
                        "Running application differs from the STOP layout; no erase/write"
                (out / "original_flash.bin").write_bytes(original)
                report["original_sha256"] = hashlib.sha256(original).hexdigest()
                target.reset_and_halt(reset_type=Target.ResetType.SYSRESETREQ)
                record("programming_application_sectors")
                FileProgrammer(session, chip_erase="sector", smart_flash=False,
                    trust_crc=False, keep_unwritten=True).program(str(image), base_address=0x08000000)
            else:
                original = saved_original
                report["original_sha256"] = hashlib.sha256(original).hexdigest()
            target.halt()
            record("verifying_application_erase_region" if args.application_sectors_only else "verifying_entire_flash")
            readback = read_flash(target, "Verified")
            expected = firmware + original[len(firmware):]
            assert readback == expected, "Flash mismatch; target left halted"
            report.update(readback_sha256=hashlib.sha256(readback[:len(firmware)]).hexdigest(),
                          untouched_start=hex(0x08000000 + len(firmware)),
                          preserved_sha256=hashlib.sha256(readback[len(firmware):]).hexdigest())
            record("verified_resetting")
            target.reset(reset_type=Target.ResetType.SYSRESETREQ)
            record("flashed_verified_reset")
    except BaseException as exc:
        report["error"] = repr(exc)
        record("failed_do_not_assume_running")
        raise
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()

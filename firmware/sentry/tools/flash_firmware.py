#!/usr/bin/env python3
"""Program the connected sentry through its ATK CMSIS-DAP; leave the CPU halted."""
import argparse
import faulthandler
import hashlib
import json
from pathlib import Path
import struct
import subprocess

from intelhex import IntelHex
from pyocd.core.session import Session
from pyocd.flash.file_programmer import FileProgrammer
from pyocd.probe.cmsis_dap_probe import CMSISDAPProbe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--program', action='store_true', help='Erase/program after checking device identity')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    image = root / 'build/sentry_uc.hex'
    ihex = IntelHex(str(image))
    segments = ihex.segments()
    assert segments and all(0x08000000 <= start < end <= 0x08100000 for start, end in segments)
    sp, entry = struct.unpack('<II', bytes(ihex.tobinarray(start=0x08000000, size=8)))
    assert 0x20000000 < sp <= 0x20020000 and entry & 1 and 0x08000000 <= entry < 0x08100000
    probes = [p for p in CMSISDAPProbe.get_all_connected_probes() if p.unique_id == 'ATK 20190528']
    assert len(probes) == 1, 'Expected exactly one ATK 20190528 probe'
    faulthandler.dump_traceback_later(20, exit=True)
    session = Session(probes[0], options={'target_override': 'stm32f405rg',
                      'frequency': 100000, 'connect_mode': 'attach', 'cmsis_dap.limit_packets': True, 'cmsis_dap.deferred_transfers': False,
                      'resume_on_disconnect': False, 'hide_programming_progress': True})
    try:
        session.open()
        faulthandler.cancel_dump_traceback_later()
        target = session.target
        identity = target.read32(0xE0042000)
        flash_kib = target.read16(0x1FFF7A22)
        option_control = target.read32(0x40023C14)
        print(f'Device ID={identity:#x}, Flash={flash_kib} KiB, options={option_control:#x}', flush=True)
        assert identity & 0xFFF == 0x413 and flash_kib == 1024, 'Unexpected MCU'
        assert (option_control >> 8) & 0xFF == 0xAA, 'Unexpected read protection'
        if not args.program:
            print('Identity check complete; no flash written.', flush=True)
            return
        target.reset_and_halt()
        FileProgrammer(session, chip_erase='sector', smart_flash=False, trust_crc=False,
                       keep_unwritten=False).program(str(image))
        target.reset_and_halt()
        observed = hashlib.sha256()
        count = 0
        for start, end in segments:
            for address in range(start, end, 1024):
                length = min(1024, end - address)
                actual = bytes(target.read_memory_block8(address, length))
                wanted = bytes(ihex.tobinarray(start=address, size=length))
                assert actual == wanted, f'Flash readback mismatch at {address:#x}'
                observed.update(actual)
                count += length
        assert target.read32(0x40023C14) == option_control, 'Option bytes changed'
        state = str(target.get_state())
        assert 'HALTED' in state
        result = {'upstream_source_commit': '6a107ca08ab944c43043a4d861032b03101c71fc',
                  'repository_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                  'worktree_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=root, text=True)),
                  'hex_sha256': hashlib.sha256(image.read_bytes()).hexdigest(),
                  'verified_bytes': count, 'readback_sha256': observed.hexdigest(),
                  'device_id': hex(identity), 'flash_kib': flash_kib,
                  'option_control': hex(option_control), 'state': state}
        (root / 'build/flash_result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
    finally:
        faulthandler.cancel_dump_traceback_later()
        session.close()


if __name__ == '__main__':
    main()

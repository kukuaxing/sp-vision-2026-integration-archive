#!/usr/bin/env python3
"""Offline client regression using the unmodified supplied firmware protocol headers."""
import argparse
import hashlib
from pathlib import Path
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_SHA256 = 'c407c2d9891fa5074b6f7b4a4e24856b0db1efeb0f5cac9ed4ee03b1b303317b'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware-zip', type=Path,
                        default=ROOT / 'data/firmware-delivery/RCS_code_rmv_20261004.zip')
    parser.add_argument('--cxx', default='g++')
    args = parser.parse_args()
    digest = hashlib.sha256(args.firmware_zip.read_bytes()).hexdigest()
    if digest != EXPECTED_SHA256:
        raise ValueError('Firmware ZIP differs from the supplied 20261004 fixture')
    # Extract only two protocol headers to a temporary directory, never a source copy.
    with tempfile.TemporaryDirectory(prefix='rmv-firmware-regression-') as tmp:
        directory = Path(tmp)
        with zipfile.ZipFile(args.firmware_zip) as archive:
            for name in ('rmv_core.h', 'rmv_board_config.h'):
                matches = [entry for entry in archive.namelist()
                           if entry.replace('\\', '/').endswith('/STM32F405/' + name)]
                if len(matches) != 1:
                    raise ValueError(f'Expected exactly one firmware header: {name}')
                (directory / name).write_bytes(archive.read(matches[0]))
        binary = directory / 'firmware_handshake_test'
        subprocess.run([args.cxx, '-std=c++17', '-O0', '-UNDEBUG', '-pthread',
                        '-I', str(ROOT), '-I', str(ROOT / 'librm/libs/eigen'),
                        '-I', str(ROOT / 'io/serial/include'), '-I', str(directory),
                        str(ROOT / 'tests/rmv/firmware_handshake_test.cpp'),
                        '-o', str(binary)], check=True, timeout=120)
        subprocess.run([str(binary)], check=True, timeout=10)
    print(f'Firmware SHA256: {digest}; no physical devices accessed')


if __name__ == '__main__':
    main()

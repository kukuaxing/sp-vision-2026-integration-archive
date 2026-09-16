#!/usr/bin/env python3
"""Check the Git index, not untracked local runtime resources."""
from pathlib import Path, PurePosixPath
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN = {'.bin', '.hex', '.elf', '.axf', '.exe', '.dll', '.so', '.dylib', '.a', '.lib',
             '.o', '.obj', '.pdb', '.pyc', '.pyo', '.onnx', '.pt', '.pth', '.engine',
             '.avi', '.mp4', '.zip', '.tar', '.gz', '.tgz', '.7z', '.log'}


def git(*arguments, **kwargs):
    return subprocess.run(['git', *arguments], cwd=ROOT, capture_output=True, **kwargs)


def main():
    result = git('ls-files', '--stage', '-z')
    result.check_returncode()
    errors = []
    count = 0
    for record in result.stdout.split(b'\0'):
        if not record:
            continue
        metadata, raw_name = record.split(b'\t', 1)
        if metadata.startswith(b'160000 '):
            continue  # Upstream source dependencies have their own pinned repositories.
        name = raw_name.decode('utf-8')
        path = PurePosixPath(name)
        count += 1
        if (path.suffix.lower() in FORBIDDEN or '.so.' in path.name or
                (name.startswith('assets/') and name != 'assets/README.md')):
            errors.append(f'Non-source resource is tracked: {name}')
        with (ROOT / name).open('rb') as handle:
            magic = handle.read(8)
        if magic.startswith((b'\x7fELF', b'MZ', b'!<arch>\n', b'\x89PNG', b'\xff\xd8\xff')):
            errors.append(f'Binary artifact is tracked: {name}')
    ignored = git('ls-files', '-ci', '--exclude-standard', '-z')
    ignored.check_returncode()
    errors.extend('Tracked file matches ignore rules: ' + p.decode() for p in ignored.stdout.split(b'\0') if p)
    must_ignore = ['build/test', 'firmware/sentry/build/test', 'firmware/infantry/build-arm/a.o',
                   'CMakeCache.txt', 'compile_commands.json', 'temp.elf', 'temp.hex', 'temp.bin',
                   'libtest.so.1', 'assets/yolo26.xml', 'assets/yolo26.bin', 'logs/run.log',
                   '__pycache__/x.pyc', '.env', 'id_ed25519', 'io/hikrobot/lib/amd64/libMvCameraControl.so']
    must_keep = ['CMakeLists.txt', 'configs/standard.yaml', 'configs/calibration.yaml',
                 'src/standard.cpp', 'firmware/sentry/STM32F405.cpp', 'scripts/build.sh',
                 '.github/workflows/check.yml', '.env.example', 'assets/README.md', 'docs/artifacts.json']
    for name in must_ignore + must_keep:
        result = git('check-ignore', '--no-index', '-q', '--', name)
        expected = 0 if name in must_ignore else 1
        if result.returncode != expected:
            errors.append(f'Unexpected ignore rule for {name}: exit={result.returncode}')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print(f'PASS: {count} tracked source/config/document files; no tracked artifacts or ignored files; ignore-rule contract passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())

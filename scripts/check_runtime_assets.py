#!/usr/bin/env python3
"""Verify current external runtime dependencies without modifying files."""
import hashlib
import json
from pathlib import Path
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / 'docs/artifacts.json').read_text(encoding='utf-8'))
    failures = []
    for item in manifest['files']:
        path = (root / item['path']).resolve()
        if not path.is_relative_to(root):
            raise ValueError('Resource path is outside this repository')
        if not path.is_file():
            failures.append(f"Missing: {item['path']}; see docs/dependencies.md")
            continue
        if path.stat().st_size != item['size'] or hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            failures.append(f"Content differs: {item['path']}")
    if failures:
        print('\n'.join(failures), file=sys.stderr)
        return 1
    print(f"Verified {len(manifest['files'])} current runtime resources; no files changed or application started.")
    return 0


if __name__ == '__main__':
    sys.exit(main())

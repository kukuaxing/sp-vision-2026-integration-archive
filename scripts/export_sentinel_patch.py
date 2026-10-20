#!/usr/bin/env python3
"""Export only MCU application changes against the exact user-supplied firmware ZIP.

No hardware, network or flash access. Patch paths are relative to Sentinel_robot-main.
Text is normalized to LF, as in a Git source checkout.
"""
import argparse
import difflib
import hashlib
from pathlib import Path
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
BASE_SHA256 = "c93e6e85afa33fbc012e00c91d255332c33f03d02258b0348b6a510e4ff283a0"


def normalized(data):
    return data.decode("utf-8-sig").replace("\r\n", "\n")


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("firmware_zip", type=Path)
    args = cli.parse_args()
    if hashlib.sha256(args.firmware_zip.read_bytes()).hexdigest() != BASE_SHA256:
        cli.error("ZIP differs from the reviewed source; do not apply this patch to another firmware")
    output = ROOT / "build/sentinel-v2-firmware.patch"
    output.parent.mkdir(parents=True, exist_ok=True)
    pieces, originals = [], {}
    with zipfile.ZipFile(args.firmware_zip) as archive:
        for name in archive.namelist():
            if not name.startswith("Sentinel_robot-main/STM32F405/"):
                continue
            relative = Path(name).relative_to("Sentinel_robot-main")
            if relative.suffix not in (".c", ".cpp", ".h", ".hpp"):
                continue
            current = ROOT / "firmware/sentry" / relative
            before = normalized(archive.read(name))
            after = normalized(current.read_bytes())
            if before == after:
                continue
            originals[relative] = before
            difference = difflib.unified_diff(before.splitlines(keepends=True),
                after.splitlines(keepends=True), fromfile="a/" + relative.as_posix(),
                tofile="b/" + relative.as_posix())
            for line in difference:
                pieces.append(line if line.endswith("\n") else line + "\n\\ No newline at end of file\n")
    output.write_text("".join(pieces), encoding="utf-8", newline="\n")
    # Apply the delivered patch to a pristine normalized source fixture, then compare.
    with tempfile.TemporaryDirectory(prefix="sentinel-patch-check-") as folder:
        fixture = Path(folder)
        for name, before in originals.items():
            path = fixture / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(before, encoding="utf-8", newline="\n")
        subprocess.run(["git", "apply", "--check", str(output)], cwd=fixture, check=True)
        subprocess.run(["git", "apply", str(output)], cwd=fixture, check=True)
        for name in originals:
            assert normalized((fixture / name).read_bytes()) == normalized(
                (ROOT / "firmware/sentry" / name).read_bytes()), name
    print(f"PASS patch applies to pristine ZIP source and exactly reproduces {len(originals)} modified application files")
    for name in originals:
        print(name.as_posix())
    print(output)


if __name__ == "__main__":
    main()

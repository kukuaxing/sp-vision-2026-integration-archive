#!/usr/bin/env python3
"""One offline gate for Windows/Linux. No camera SDK, model, robot or network.

python scripts/check_offline.py [--python-only]
CXX can select g++/clang++; Windows additionally discovers installed MSVC.
Build output stays inside the repository build/ directory.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def run(arguments, **kwargs):
    print("+ " + " ".join(map(str, arguments)), flush=True)
    subprocess.run(list(map(str, arguments)), cwd=ROOT, check=True, **kwargs)


def compiler_environment():
    candidate = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
    if candidate:
        return candidate, os.environ.copy(), False
    cl = shutil.which("cl")
    if cl:
        return cl, os.environ.copy(), True
    if os.name != "nt":
        raise RuntimeError("Install a C++17 compiler or run --python-only")
    vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
    if not vswhere.is_file():
        raise RuntimeError("No compiler found; install Visual Studio C++ Build Tools")
    installations = json.loads(subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires",
                                "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-format", "json", "-utf8"],
                                text=True, encoding="utf-8"))
    if not installations:
        raise RuntimeError("No MSVC C++ toolchain installed")
    setup = Path(installations[0]["installationPath"]) / "VC/Auxiliary/Build/vcvars64.bat"
    # Trusted installed batch file; only obtaining its build environment.
    if any(char in str(setup) for char in '"%!?&|<>\r\n'):
        raise RuntimeError("Unsupported shell metacharacter in MSVC installation path")
    command = f'cmd.exe /d /s /c ""{setup}" >nul && set"'
    output = subprocess.check_output(command, text=True, errors="replace")
    environment = os.environ.copy()
    for line in output.splitlines():
        key, separator, value = line.partition("=")
        if separator and key:
            # Windows variable names are case insensitive.
            for existing in list(environment):
                if existing.lower() == key.lower():
                    del environment[existing]
            environment[key] = value
    compiler = shutil.which("cl.exe", path=next(v for k, v in environment.items() if k.lower() == "path"))
    if not compiler:
        raise RuntimeError("MSVC environment did not contain cl.exe")
    return compiler, environment, True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python-only", action="store_true")
    args = parser.parse_args()
    run([sys.executable, "-m", "unittest", "discover", "-s", "tests/rmv", "-v"])
    run([sys.executable, "-m", "tools.rmv", "demo"])
    run([sys.executable, "-m", "tools.rmv", "inspect", "logs/rmv/offline_demo.jsonl"])
    if args.python_only:
        print("Python checks passed; C++ checks explicitly skipped.")
        return
    compiler, environment, msvc = compiler_environment()
    build = ROOT / "build/offline"
    build.mkdir(parents=True, exist_ok=True)
    from tools.rmv.protocol import examples, encode
    golden = build / "rmv_vectors.txt"
    golden.write_text("\n".join(encode(f).hex(" ") for _, f in examples()) + "\n", encoding="ascii")
    targets = [("rmv_core_test", "tests/rmv/core_test.cpp")]
    # These are existing header-only estimator regressions, not replacement implementations.
    for name in ("measurement_noise", "yaw_eskf", "armor_motion_filter", "robust_range_filter",
                 "shot_delay_adapter", "rotation_model_manager", "gyro_phase_scheduler", "gyro_face_selector"):
        targets.append((name + "_test", "tasks/auto_aim/" + name + "_test.cpp"))
    for name, source in targets:
        binary = build / (name + (".exe" if os.name == "nt" else ""))
        if msvc:
            arguments = [compiler, "/nologo", "/utf-8", "/std:c++17", "/EHsc", "/O2", "/W3",
                         "/D_USE_MATH_DEFINES", "/UNDEBUG", "/I.", "/Ilibrm/libs/eigen", source,
                         "/Fe:" + str(binary), "/Fo:" + str(build / (name + ".obj"))]
        else:
            arguments = [compiler, "-std=c++17", "-O2", "-UNDEBUG", "-I.", "-Ilibrm/libs/eigen",
                         source, "-o", binary]
        run(arguments, env=environment)
        run([binary] + ([golden] if name == "rmv_core_test" else []), env=environment)
    print(f"PASS: offline acceptance and {len(targets)} C++ executables; no device I/O.")


if __name__ == "__main__":
    main()

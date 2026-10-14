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
    subprocess.run(list(map(str, arguments)), cwd=ROOT, check=True, timeout=kwargs.pop("timeout", 600), **kwargs)


def compiler_environment():
    candidate = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
    if candidate:
        return candidate, os.environ.copy(), Path(candidate).stem.lower() in ("cl", "clang-cl")
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


def cmake_environment(compiler, environment, requested=None):
    search_path = next((v for k, v in environment.items() if k.lower() == "path"), "")
    cmake = requested or shutil.which("cmake", path=search_path)
    if not cmake:
        for parent in Path(compiler).parents:
            candidate = parent / "Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"
            if candidate.is_file():
                cmake = str(candidate)
                break
    if not cmake:
        raise RuntimeError("CMake not found; supply --cmake PATH")
    return str(cmake)


def native_checks(args, compiler, environment):
    cmake = cmake_environment(compiler, environment, args.cmake)
    ctest = str(Path(cmake).with_name("ctest.exe" if os.name == "nt" else "ctest"))
    if not Path(ctest).is_file():
        ctest = shutil.which("ctest") or ctest
    build = ROOT / "build/native-offline"
    arguments = [cmake, "-S", ROOT, "-B", build, "-DSP_BUILD_OFFLINE_ONLY=ON",
                 "-DSP_OFFLINE_WITH_VISION=ON", "-DPython3_EXECUTABLE=" + sys.executable,
                 "-DSP_OFFLINE_WITH_INFERENCE=" + ("ON" if args.with_inference else "OFF")]
    if args.vcpkg_root:
        arguments += ["-DCMAKE_TOOLCHAIN_FILE=" + str(args.vcpkg_root.resolve() / "scripts/buildsystems/vcpkg.cmake")]
    if args.openvino_dir:
        arguments += ["-DOpenVINO_DIR=" + str(args.openvino_dir.resolve())]
    run(arguments, env=environment)
    run([cmake, "--build", build, "--config", "Release", "--parallel", "1"], env=environment)
    run([ctest, "--test-dir", build, "-C", "Release", "--output-on-failure", "-j", "1"], env=environment)
    print("PASS: native geometry/estimator/transport regressions; no device I/O. Detector replay is explicit.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python-only", action="store_true")
    parser.add_argument("--with-vision", action="store_true", help="also build real OpenCV Solver/Target/Aimer/Planner")
    parser.add_argument("--with-inference", action="store_true", help="also build explicit YOLO26 file replay (does not auto-run)")
    parser.add_argument("--cmake", help="CMake executable if not on PATH")
    parser.add_argument("--vcpkg-root", type=Path, help="optional Windows vcpkg installation")
    parser.add_argument("--openvino-dir", type=Path, help="OpenVINO CMake package directory")
    args = parser.parse_args()
    if args.python_only and (args.with_vision or args.with_inference):
        parser.error("--python-only conflicts with native vision/inference checks")
    # The native CTest gate includes these same Python checks.
    if not (args.with_vision or args.with_inference):
        for suite in ("rmv", "replay", "benchmark"):
            run([sys.executable, "-m", "unittest", "discover", "-s", "tests/" + suite, "-v"])
    run([sys.executable, "-m", "tools.rmv", "demo"])
    run([sys.executable, "-m", "tools.rmv", "inspect", "logs/rmv/offline_demo.jsonl"])
    if args.python_only:
        print("Python checks passed; C++ checks explicitly skipped.")
        return
    compiler, environment, msvc = compiler_environment()
    if args.with_vision or args.with_inference:
        native_checks(args, compiler, environment)
        return
    build = ROOT / "build/offline"
    build.mkdir(parents=True, exist_ok=True)
    from tools.rmv.protocol import examples, encode
    golden = build / "rmv_vectors.txt"
    golden.write_text("\n".join(encode(f).hex(" ") for _, f in examples()) + "\n", encoding="ascii")
    targets = [("sentinel_protocol_test", "tests/sentinel/protocol_test.cpp"),
               ("rmv_core_test", "tests/rmv/core_test.cpp"),
               ("rmv_session_test", "tests/rmv/session_test.cpp"),
               ("rmv_transport_test", "tests/rmv/transport_test.cpp"),
               ("rmv_session_peer", "tests/rmv/session_peer.cpp")]
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
        if name == "rmv_session_peer":
            run([sys.executable, "-m", "tools.rmv.cpp_interop", binary], env=environment)
        else:
            run([binary] + ([golden] if name == "rmv_core_test" else []), env=environment)
    print(f"PASS: offline acceptance and {len(targets)} C++ executables, including cross-language scenarios; no device I/O.")


if __name__ == "__main__":
    main()

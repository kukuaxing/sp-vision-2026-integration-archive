#!/usr/bin/env python3
"""Run the real C++ YOLO26 on explicit files, record events, and optionally compare two runs.

No cameras, serial devices, networks, target tracking, motion commands or firing outputs.
Video decoding uses an already-installed cv2 FFmpeg backend with one decoder thread;
the actual inference and detection postprocessing remain in the repository C++ sources.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import platform
import statistics
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.replay import RecordingError, Writer, inspect, stamp
from tools.replay.recording import HOST, relative_file, sha256_file

FRAMES = {
    "world": "Not observed in file detector replay; no world pose or extrinsic transform is assumed.",
    "gimbal": "Not observed in file detector replay; no gimbal or IMU data was synthesized.",
    "camera": "Decoded image pixel coordinates: x right, y down. No calibrated 3D frame is inferred.",
    "chassis": "Not observed in file detector replay; no chassis measurement is available.",
}


def environment(dll_dirs: list[Path]) -> dict:
    result = os.environ.copy()
    for key in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "OPENCV_FOR_THREADS_NUM",
                "OPENCV_FFMPEG_THREADS"):
        result[key] = "1"
    result["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "threads;1"
    defaults = [Path("D:/Dev/vcpkg/installed/x64-windows/bin")]
    spec = importlib.util.find_spec("openvino")
    if spec is not None and spec.origin:
        defaults.append(Path(spec.origin).parent / "libs")
    paths = [str(path.resolve()) for path in dll_dirs + defaults if path.is_dir()]
    result["PATH"] = os.pathsep.join(paths + [result.get("PATH", "")])
    return result


def revision() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown-local-revision"


def reject_constant(value: str):
    raise RecordingError(f"non-finite raw detector JSON: {value}")


def read_raw(path: Path) -> tuple[dict, list[dict]]:
    with path.open(encoding="utf-8") as handle:
        records = [json.loads(line, parse_constant=reject_constant) for line in handle]
    if len(records) < 3 or records[0].get("schema") != "sp-yolo26-file-replay-1":
        raise RecordingError("C++ detector did not produce the expected raw manifest and frames")
    footer = records[-1]
    if footer.get("kind") != "end" or footer.get("complete") is not True:
        raise RecordingError("C++ detector output is incomplete")
    frames = records[1:-1]
    if footer.get("frame_count") != len(frames) or any(row.get("kind") != "frame" for row in frames):
        raise RecordingError("C++ detector frame count/kind mismatch")
    if records[0].get("inference_device") != "CPU" or records[0].get("inference_threads") != 1:
        raise RecordingError("C++ detector did not report required CPU single-thread configuration")
    return records[0], frames


def prepare_input(source: Path, directory: Path, max_frames: int, stride: int) -> tuple[list[str], dict]:
    """Use stdlib for images; decode videos with the existing Python OpenCV/FFmpeg wheel."""
    image_extensions = {".png", ".jpg", ".jpeg", ".bmp", ".pgm", ".ppm", ".tif", ".tiff", ".webp"}
    if source.suffix.lower() in image_extensions:
        return ["--input", str(source)], {"kind": "image", "decoder": "C++ OpenCV", "hardware_exposure_time": None}
    for key, value in environment([]).items():
        if key != "PATH" and key.startswith(("OMP_", "MKL_", "OPENBLAS_", "OPENCV_")):
            os.environ[key] = value
    try:
        import cv2
    except ImportError as error:
        raise RecordingError("Video input requires the already-installed Python cv2 or direct C++ FFmpeg support") from error
    if not hasattr(cv2, "CAP_PROP_N_THREADS"):
        raise RecordingError("This Python OpenCV lacks decoder thread control; use a local image or a newer cv2 build")
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    capture = cv2.VideoCapture(str(source), cv2.CAP_FFMPEG, [cv2.CAP_PROP_N_THREADS, 1])
    if not capture.isOpened():
        raise RecordingError("Cannot open explicit video with FFmpeg and one decoder thread")
    directory.mkdir()
    frames = []
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        index = 0
        while len(frames) < max_frames:
            ok, bgr = capture.read()
            if not ok:
                break
            if index % stride == 0:
                path = directory / f"frame-{index:06d}.png"
                if not cv2.imwrite(str(path), bgr):
                    raise RecordingError("Cannot store decoded input frame")
                pts_ms = capture.get(cv2.CAP_PROP_POS_MSEC)
                pts_us = round(pts_ms * 1000) if math.isfinite(pts_ms) and pts_ms >= 0 else None
                frames.append({"path": str(path.resolve()), "source_index": index, "video_pts_us": pts_us})
            index += 1
    finally:
        capture.release()
    if not frames:
        raise RecordingError("Video contained no decodable frames")
    list_path = directory / "frames.json"
    list_path.write_text(json.dumps(frames, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return ["--image-list", str(list_path.resolve())], {
        "kind": "video", "decoder": f"Python cv2 {cv2.__version__}, FFmpeg, threads=1",
        "fps_metadata": fps if math.isfinite(fps) and fps > 0 else None,
        "selected_source_indices": [row["source_index"] for row in frames],
        "pts_meaning": "Container presentation time; unpaired with hardware exposure or MCU time",
        "hardware_exposure_time": None,
    }


def invoke(executable: Path, config: Path, output: Path, input_options: list[str],
           max_frames: int, stride: int, log: Path, dll_dirs: list[Path]) -> tuple[dict, list[dict]]:
    arguments = [str(executable), "--config", str(config), "--repo-root", str(ROOT),
                 "--output", str(output), "--max-frames", str(max_frames), "--stride", str(stride)] + input_options
    with log.open("xb") as handle:
        result = subprocess.run(arguments, cwd=ROOT, env=environment(dll_dirs), stdout=handle,
                                stderr=subprocess.STDOUT, timeout=600,
                                creationflags=subprocess.BELOW_NORMAL_PRIORITY_CLASS if os.name == "nt" else 0)
    if result.returncode != 0:
        raise RecordingError(f"C++ detector exited {result.returncode}; log: {log}")
    return read_raw(output / "raw.jsonl")


def compare(first: list[dict], second: list[dict], first_root: Path, second_root: Path,
            tolerance: float = 1e-5) -> dict:
    if len(first) != len(second):
        raise RecordingError("repeat mismatch: frame count")
    max_confidence = max_keypoint = 0.0
    object_count = 0
    for left, right in zip(first, second):
        for field in ("source_index", "width", "height", "video_pts_us"):
            if left[field] != right[field]:
                raise RecordingError(f"repeat mismatch: {field}")
        if sha256_file(relative_file(first_root, left["image_path"])) != sha256_file(relative_file(second_root, right["image_path"])):
            raise RecordingError("repeat mismatch: decoded pixels/image file")
        key = lambda value: (value["class_id"], value["color"], value["armor_type"],
                             value["corners_px"][0][0], value["corners_px"][0][1])
        left_objects = sorted(left["objects"], key=key)
        right_objects = sorted(right["objects"], key=key)
        if len(left_objects) != len(right_objects):
            raise RecordingError("repeat mismatch: detection count")
        for a, b in zip(left_objects, right_objects):
            object_count += 1
            for field in ("label", "color", "armor_type", "class_id", "color_source"):
                if a[field] != b[field]:
                    raise RecordingError(f"repeat mismatch: {field}")
            for field in ("confidence", "keypoint_confidence", "color_score"):
                if not math.isclose(a[field], b[field], rel_tol=tolerance, abs_tol=tolerance):
                    raise RecordingError(f"repeat mismatch: {field}")
            max_confidence = max(max_confidence, abs(a["confidence"] - b["confidence"]))
            for ac, bc in zip(a["corners_px"], b["corners_px"]):
                for x, y in zip(ac, bc):
                    max_keypoint = max(max_keypoint, abs(x-y))
                    if not math.isclose(x, y, rel_tol=tolerance, abs_tol=tolerance):
                        raise RecordingError("repeat mismatch: corner coordinate")
    return {"matched": True, "frames": len(first), "detections": object_count,
            "absolute_and_relative_tolerance": tolerance, "max_confidence_difference": max_confidence,
            "max_keypoint_difference_px": max_keypoint, "compared_timing": False,
            "meaning": "same decoded files produce matching production detector output; not accuracy"}


def normalize(output: Path, raw_root: Path, manifest: dict, frames: list[dict], config: Path,
              original_config: str, metadata: dict) -> dict:
    configs = {str(config.relative_to(ROOT)) if config.is_relative_to(ROOT) else str(config): original_config,
               "derived_detector_runtime.yaml": (raw_root / "runtime.yaml").read_text(encoding="utf-8"),
               "detector_provenance.json": json.dumps(metadata, ensure_ascii=False, sort_keys=True, indent=2)}
    events_path = output / "events.jsonl"
    host_epoch = "cpp-replay-host-1"
    with Writer(events_path, run_id=output.name,
                source="synthetic" if manifest["input_kind"] == "synthetic_blank" else "imported",
                runtime={"application": "production-YOLO26-file-replay", "revision": metadata["revision"],
                         "build": metadata["executable"]["sha256"], "platform": platform.platform()},
                clocks={HOST: host_epoch, "media_pts": "file-1"}, frames=FRAMES, configs=configs) as writer:
        for frame in frames:
            identity = f"frame-{frame['source_index']:06d}"
            input_image = relative_file(raw_root, frame["image_path"])
            image_path = input_image.relative_to(output).as_posix()
            is_video = manifest["input_kind"] in {"video", "decoded_sequence"}
            if is_video and frame["video_pts_us"] is not None:
                measurement = stamp(frame["video_pts_us"], domain="media_pts", epoch="file-1")
            else:
                # Explicit decode-completion time, not a fabricated exposure timestamp.
                measurement = stamp(frame["decoded_host_us"], epoch=host_epoch)
            writer.append("image", stream="file/images", host_us=frame["decoded_host_us"], timestamp=measurement,
                          data={"image_id": identity, "path": image_path, "sha256": sha256_file(input_image),
                                "width": frame["width"], "height": frame["height"], "encoding": "png"})
            objects = [{"label": item["label"], "color": item["color"] if item["color"] in {"red", "blue"} else "unknown",
                        "confidence": item["confidence"], "corners_px": item["corners_px"]} for item in frame["objects"]]
            writer.append("detections", stream="detector/yolo26", host_us=frame["finished_host_us"],
                          timestamp=measurement, data={"image_id": identity, "objects": objects})
            detail = json.dumps({"image_id": identity, "detector_wall_us": frame["detector_wall_us"],
                                 "video_pts_us": frame["video_pts_us"], "objects": len(objects),
                                 "measurement_time": "container_pts" if is_video and frame["video_pts_us"] is not None else "decode_completion",
                                 "raw_details": (raw_root / "raw.jsonl").relative_to(output).as_posix()},
                                separators=(",", ":"))
            writer.append("health", stream="detector/timing", host_us=frame["finished_host_us"],
                          timestamp=stamp(frame["finished_host_us"], epoch=host_epoch),
                          data={"component": "production_yolo26", "status": "ok", "detail": detail})
    return inspect(events_path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--input", type=Path, help="Explicit existing video/image file, never a camera ID or URL")
    inputs.add_argument("--blank", action="store_true", help="Real-model empty-image and 640x640 black-image regression")
    parser.add_argument("--executable", type=Path, default=ROOT / "build/native-offline/bin/Release/detector_replay.exe")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/standard.yaml")
    parser.add_argument("--output", type=Path, required=True, help="New capture directory under data/replay/")
    parser.add_argument("--report", type=Path, help="New JSON report under logs/replay/")
    parser.add_argument("--max-frames", type=int, default=8)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--repeat-check", action="store_true", help="Run a fresh detector twice on the identical decoded input")
    parser.add_argument("--dll-dir", type=Path, action="append", default=[])
    args = parser.parse_args(argv)
    try:
        if not 1 <= args.max_frames <= 1000 or not 1 <= args.stride <= 10000:
            raise RecordingError("max-frames must be 1..1000 and stride 1..10000")
        output, executable, config = args.output.resolve(), args.executable.resolve(), args.config.resolve()
        source = args.input.resolve() if args.input else None
        data_root = (ROOT / "data/replay").resolve()
        log_root = (ROOT / "logs/replay").resolve()
        if output == data_root or not output.is_relative_to(data_root):
            raise RecordingError("output must be a new child directory under this repository's data/replay/")
        report = args.report.resolve() if args.report else log_root / f"{output.name}_detector_report.json"
        if not report.is_relative_to(log_root) or report == log_root:
            raise RecordingError("report must be a file under this repository's logs/replay/")
        if output.exists() or report.exists():
            raise RecordingError("refusing to overwrite existing capture or report")
        for path in (executable, config, source):
            if path is not None and not path.is_file():
                raise RecordingError(f"required local file missing: {path}")
        logs = [log_root / f"{output.name}_detector_pass{index}.log" for index in (1, 2)]
        if any(log.exists() for log in logs[:2 if args.repeat_check else 1]):
            raise RecordingError("refusing to overwrite existing detector logs")
        config_bytes = config.read_bytes()
        before = {str(path): sha256_file(path) for path in (executable, config, source) if path is not None}
        output.mkdir(parents=True)
        log_root.mkdir(parents=True, exist_ok=True)
        if args.blank:
            input_options, input_metadata = ["--blank"], {"kind": "synthetic_blank", "hardware_exposure_time": None}
        else:
            input_options, input_metadata = prepare_input(source, output / "decoded_input", args.max_frames, args.stride)
        first_root = output / "pass1"
        manifest, first = invoke(executable, config, first_root, input_options, args.max_frames, args.stride,
                                 logs[0], args.dll_dir)
        consistency = None
        if args.repeat_check:
            second_root = output / "pass2"
            second_manifest, second = invoke(executable, config, second_root, input_options, args.max_frames, args.stride,
                                             logs[1], args.dll_dir)
            if manifest != second_manifest:
                raise RecordingError("repeat run environment/configuration manifest changed")
            consistency = compare(first, second, first_root, second_root)
        for path, expected in before.items():
            if sha256_file(Path(path)) != expected:
                raise RecordingError(f"input/config/executable changed during replay: {path}")
        sources = ["tests/offline/detector_replay.cpp", "tasks/auto_aim/yolos/yolo26.cpp",
                   "tasks/auto_aim/armor.cpp", "tasks/auto_aim/detector.cpp", "tasks/auto_aim/classifier.cpp",
                   "tasks/auto_aim/adaptive_corner_refiner.cpp"]
        metadata = {"revision": revision(), "executable": {"path": str(executable), "sha256": before[str(executable)]},
                    "input": None if source is None else {"path": str(source), "sha256": before[str(source)]},
                    "input_metadata": input_metadata, "cpp_manifest": manifest,
                    "model_hashes_after_execution": [{"path": path, "sha256": sha256_file(Path(path))} for path in manifest["models"]],
                    "source_hashes_after_execution": {name: sha256_file(ROOT / name) for name in sources},
                    "coordinate_order": "top-left, top-right, bottom-right, bottom-left; pixels",
                    "full_detection_details": "pass1/raw.jsonl preserves class_id, armor_type, keypoint_confidence and color evidence"}
        integrity = normalize(output, first_root, manifest, first, config, config_bytes.decode("utf-8"), metadata)
        times = sorted(frame["detector_wall_us"] for frame in first)
        result = {"schema": "sp-detector-replay-report-1", "recording": str(output / "events.jsonl"),
                  "detector": "actual repository YOLO26 C++ and OpenVINO CPU", "frames": len(first),
                  "detections": sum(len(frame["objects"]) for frame in first),
                  "empty_and_black_image_regression": "passed" if args.blank else "not requested",
                  "detector_wall_us": {"min": times[0], "median": statistics.median(times), "max": times[-1],
                                       "meaning": "preprocessing + inference + postprocessing; this Windows machine only"},
                  "repeat_consistency": consistency, "recording_integrity": integrity, "provenance": metadata,
                  "limits": ["No annotations: this is not a detection accuracy or robustness measurement.",
                             "No pose/IMU, target tracking, ballistic calculation, commands or physical feedback was fabricated.",
                             "Video PTS is not hardware exposure time and is not mapped to a robot clock.",
                             "Thread limits apply to OpenVINO, OpenCV and decoder work; OS/runtime bookkeeping threads may exist.",
                             "Model/source hashes are recorded after execution; executable/config/input stability is checked before and after."]}
        report.parent.mkdir(parents=True, exist_ok=True)
        with report.open("x", encoding="utf-8") as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
        print(json.dumps({"report": str(report), "recording": result["recording"], "frames": result["frames"],
                          "detections": result["detections"], "repeat_consistency": consistency,
                          "detector_wall_us": result["detector_wall_us"], "integrity": integrity["integrity"]},
                         ensure_ascii=False, indent=2))
        return 0
    except (RecordingError, OSError, ValueError, KeyError, subprocess.TimeoutExpired) as error:
        print(f"DETECTOR REPLAY REJECTED: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

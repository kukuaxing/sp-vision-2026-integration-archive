"""Validated JSONL recordings with explicit clocks and immutable image references."""

from __future__ import annotations

import hashlib
import json
import math
import re
import statistics
from collections import Counter, defaultdict
from pathlib import Path, PurePosixPath
from typing import Any, Iterable, Iterator

SCHEMA = "sp-vision-events-1"
MAX_LINE_BYTES = 8 * 1024 * 1024
HOST = "host_monotonic"
KINDS = {"image", "detections", "target", "pose", "command", "feedback", "clock_epoch", "health"}
FRAMES = {"world", "gimbal", "camera", "chassis"}
REFERENCES = {"world_los", "gimbal_joint"}
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class RecordingError(ValueError):
    """A recording cannot be interpreted without guessing or ignoring damage."""


def canonical(value: Any) -> bytes:
    try:
        return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
                          allow_nan=False).encode("utf-8")
    except (ValueError, TypeError, UnicodeError) as exc:
        raise RecordingError(f"not finite UTF-8 JSON: {exc}") from exc


def _keys(value: Any, required: set[str], label: str) -> None:
    if not isinstance(value, dict) or set(value) != required:
        actual = set(value) if isinstance(value, dict) else set()
        raise RecordingError(f"{label}: fields missing={sorted(required - actual)}, "
                             f"unknown={sorted(actual - required)}")


def _text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or len(value) > 1024 or any(ord(c) < 32 for c in value):
        raise RecordingError(f"{label}: expected nonempty text without control characters")
    return value


def _uint(value: Any, label: str, maximum: int = (1 << 64) - 1) -> int:
    if type(value) is not int or not 0 <= value <= maximum:
        raise RecordingError(f"{label}: expected unsigned integer <= {maximum}")
    return value


def _number(value: Any, label: str) -> float:
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise RecordingError(f"{label}: expected finite number")
    return float(value)


def _boolean(value: Any, label: str) -> bool:
    if type(value) is not bool:
        raise RecordingError(f"{label}: expected bool")
    return value


def _vector(value: Any, length: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise RecordingError(f"{label}: expected {length} numbers")
    return [_number(x, label) for x in value]


def _choice(value: Any, choices: set[str], label: str) -> str:
    if not isinstance(value, str) or value not in choices:
        raise RecordingError(f"{label}: expected one of {sorted(choices)}")
    return value


def _confidence(value: Any) -> None:
    if not 0 <= _number(value, "confidence") <= 1:
        raise RecordingError("confidence: outside [0,1]")


def _digest(value: Any) -> None:
    if not isinstance(value, str) or not _SHA256.fullmatch(value):
        raise RecordingError("sha256: expected 64 lowercase hexadecimal characters")


def relative_file(root: Path, relative: str) -> Path:
    """Reject absolute, traversal, Windows-drive, alternate-stream and escaping symlink paths."""
    _text(relative, "relative file path")
    parts = relative.split("/")
    if "\\" in relative or ":" in relative or any(p in ("", ".", "..") for p in parts):
        raise RecordingError(f"unsafe relative path: {relative!r}")
    reserved = {"CON", "PRN", "AUX", "NUL", "CONIN$", "CONOUT$"}
    reserved.update(f"{prefix}{number}" for prefix in ("COM", "LPT") for number in "123456789¹²³")
    if any(p[-1] in ". " or any(c in p for c in '<>"|?*') or
           p.split(".", 1)[0].rstrip(" ").upper() in reserved for p in parts):
        raise RecordingError(f"non-portable or reserved device path: {relative!r}")
    if PurePosixPath(relative).is_absolute():
        raise RecordingError(f"absolute path: {relative!r}")
    root = root.resolve()
    destination = root.joinpath(*parts).resolve()
    if not destination.is_relative_to(root) or destination == root:
        raise RecordingError(f"path escapes recording directory: {relative!r}")
    return destination


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise RecordingError(f"cannot read artifact {path}: {exc}") from exc
    return digest.hexdigest()


def stamp(time_us: int, *, domain: str = HOST, epoch: str = "host-1",
          mapped_host_us: int | None = None, uncertainty_us: int | None = None) -> dict:
    """Construct a timestamp; non-host clocks have no implicit conversion."""
    if domain == HOST:
        if mapped_host_us not in (None, time_us) or uncertainty_us not in (None, 0):
            raise RecordingError("host stamp helper requires identity mapping and zero uncertainty")
        mapped_host_us, uncertainty_us = time_us, 0
    return {"domain": domain, "epoch": epoch, "time_us": time_us,
            "mapped_host_us": mapped_host_us, "uncertainty_us": uncertainty_us}


def _parse_json(raw: bytes, line: int) -> dict:
    def reject_pairs(pairs: list[tuple[str, Any]]) -> dict:
        result: dict = {}
        for key, value in pairs:
            if key in result:
                raise RecordingError(f"duplicate JSON key {key!r}")
            result[key] = value
        return result

    def reject_constant(value: str) -> None:
        raise RecordingError(f"non-finite JSON constant {value}")

    try:
        value = json.loads(raw.decode("utf-8"), object_pairs_hook=reject_pairs,
                           parse_constant=reject_constant)
    except (ValueError, UnicodeError, RecursionError) as exc:
        raise RecordingError(f"line {line}: {exc}") from exc
    if not isinstance(value, dict):
        raise RecordingError(f"line {line}: expected JSON object")
    return value


class _Validator:
    def __init__(self, manifest: dict, root: Path):
        _keys(manifest, {"schema", "kind", "run_id", "source", "time_unit", "runtime",
                         "clocks", "frames", "configs"}, "manifest")
        if manifest["schema"] != SCHEMA or manifest["kind"] != "manifest" or manifest["time_unit"] != "us":
            raise RecordingError("unsupported schema, manifest kind, or time unit")
        _text(manifest["run_id"], "run_id")
        _choice(manifest["source"], {"synthetic", "captured", "imported"}, "source")
        _keys(manifest["runtime"], {"application", "revision", "build", "platform"}, "runtime")
        for key, value in manifest["runtime"].items():
            _text(value, f"runtime.{key}")
        clocks = manifest["clocks"]
        if not isinstance(clocks, dict) or HOST not in clocks:
            raise RecordingError("clocks must declare host_monotonic and each source epoch")
        for key, value in clocks.items():
            _text(key, "clock domain")
            _text(value, "clock epoch")
        _keys(manifest["frames"], FRAMES, "frames")
        for value in manifest["frames"].values():
            _text(value, "frame convention")
        if not isinstance(manifest["configs"], list):
            raise RecordingError("configs: expected list")
        config_names = set()
        for config in manifest["configs"]:
            _keys(config, {"name", "content_utf8", "sha256"}, "config")
            name = _text(config["name"], "config name")
            if name in config_names:
                raise RecordingError(f"duplicate config name: {name}")
            config_names.add(name)
            if not isinstance(config["content_utf8"], str):
                raise RecordingError("config content must be UTF-8 text")
            _digest(config["sha256"])
            if hashlib.sha256(config["content_utf8"].encode("utf-8")).hexdigest() != config["sha256"]:
                raise RecordingError(f"config snapshot hash mismatch: {name}")
        self.root = root.resolve()
        self.clocks = dict(clocks)
        self.epochs = {domain: {epoch} for domain, epoch in clocks.items()}
        self.seq = 0
        self.host_us = 0
        self.stream_times: dict[tuple, tuple[int, int | None]] = {}
        self.images: dict[str, tuple[int, int]] = {}
        self.image_stamps: dict[str, dict] = {}
        self.commands: dict[str, dict] = {}

    def consume(self, event: dict) -> None:
        # State is changed only after all validation succeeds: a rejected append is recoverable.
        _keys(event, {"kind", "seq", "stream", "host_us", "stamp", "data"}, "event")
        kind = _choice(event["kind"], KINDS, "kind")
        if _uint(event["seq"], "seq") != self.seq + 1:
            raise RecordingError(f"seq must be contiguous: expected {self.seq + 1}")
        stream = _text(event["stream"], "stream")
        host_us = _uint(event["host_us"], "host_us")
        if host_us < self.host_us:
            raise RecordingError("host monotonic time moved backwards")
        timestamp = event["stamp"]
        _keys(timestamp, {"domain", "epoch", "time_us", "mapped_host_us", "uncertainty_us"}, "stamp")
        domain = _text(timestamp["domain"], "stamp.domain")
        epoch = _text(timestamp["epoch"], "stamp.epoch")
        if self.clocks.get(domain) != epoch:
            raise RecordingError("unknown or retired clock epoch; clock_epoch event required")
        sample = _uint(timestamp["time_us"], "stamp.time_us")
        mapped = timestamp["mapped_host_us"]
        uncertainty = timestamp["uncertainty_us"]
        if (mapped is None) != (uncertainty is None):
            raise RecordingError("mapped_host_us and uncertainty_us must be present together")
        if mapped is not None:
            _uint(mapped, "mapped_host_us")
            _uint(uncertainty, "uncertainty_us")
            if mapped > host_us + uncertainty:
                raise RecordingError("mapped sample is later than recording time beyond uncertainty")
        if domain == HOST and (mapped != sample or uncertainty != 0 or sample > host_us):
            raise RecordingError("host timestamp requires identity mapping, zero uncertainty, sample <= host_us")
        key = (stream, domain, epoch)
        previous = self.stream_times.get(key)
        if previous and (sample < previous[0] or
                         (mapped is not None and previous[1] is not None and mapped < previous[1])):
            raise RecordingError("stream source/mapped time moved backwards within one epoch")
        self._data(kind, event["data"], timestamp, host_us)
        self.seq, self.host_us = event["seq"], host_us
        self.stream_times[key] = (sample, mapped)
        data = event["data"]
        if kind == "image":
            self.images[data["image_id"]] = (data["width"], data["height"])
            self.image_stamps[data["image_id"]] = dict(timestamp)
        elif kind == "command":
            self.commands[data["command_id"]] = dict(data)
        elif kind == "clock_epoch":
            self.clocks[data["domain"]] = data["epoch"]
            self.epochs[data["domain"]].add(data["epoch"])

    def _data(self, kind: str, data: dict, timestamp: dict, host_us: int) -> None:
        if kind == "image":
            _keys(data, {"image_id", "path", "sha256", "width", "height", "encoding"}, "image")
            identity = _text(data["image_id"], "image_id")
            if identity in self.images:
                raise RecordingError("duplicate image_id")
            width, height = _uint(data["width"], "width", 65535), _uint(data["height"], "height", 65535)
            if not width or not height:
                raise RecordingError("image dimensions must be positive")
            encoding = _choice(data["encoding"], {"pgm", "ppm", "png", "jpeg"}, "encoding")
            _digest(data["sha256"])
            path = relative_file(self.root, data["path"])
            if not path.is_file() or sha256_file(path) != data["sha256"]:
                raise RecordingError(f"missing image or image hash mismatch: {data['path']}")
            # The portable fixture format is fully checked. PNG/JPEG are integrity-checked
            # opaque payloads; their pixel decoding belongs to the consuming image backend.
            if encoding in {"pgm", "ppm"}:
                _check_pnm(path, encoding, width, height)
        elif kind == "detections":
            _keys(data, {"image_id", "objects"}, "detections")
            width, height = self._image(data["image_id"])
            image_stamp = self.image_stamps[data["image_id"]]
            if (image_stamp["domain"], image_stamp["epoch"]) == (timestamp["domain"], timestamp["epoch"]):
                if image_stamp["time_us"] != timestamp["time_us"]:
                    raise RecordingError("detection stamp must refer to its image measurement time")
            elif image_stamp["mapped_host_us"] is None or timestamp["mapped_host_us"] is None:
                raise RecordingError("cannot relate image/detection clocks without explicit mapping")
            elif abs(image_stamp["mapped_host_us"] - timestamp["mapped_host_us"]) > (
                    image_stamp["uncertainty_us"] + timestamp["uncertainty_us"]):
                raise RecordingError("detection/image mapped measurement times disagree")
            if not isinstance(data["objects"], list):
                raise RecordingError("detection objects must be a list")
            for obj in data["objects"]:
                _keys(obj, {"label", "color", "confidence", "corners_px"}, "detection")
                _text(obj["label"], "detection label")
                _choice(obj["color"], {"red", "blue", "unknown"}, "color")
                _confidence(obj["confidence"])
                if not isinstance(obj["corners_px"], list) or len(obj["corners_px"]) != 4:
                    raise RecordingError("detection must have four image corners")
                for corner in obj["corners_px"]:
                    x, y = _vector(corner, 2, "corner")
                    if not 0 <= x < width or not 0 <= y < height:
                        raise RecordingError("corner outside referenced image")
        elif kind == "target":
            _keys(data, {"target_id", "image_id", "frame", "position_m", "velocity_mps",
                         "yaw_rad", "yaw_rate_rad_s", "confidence"}, "target")
            _text(data["target_id"], "target_id")
            self._image(data["image_id"])
            _choice(data["frame"], FRAMES, "target frame")
            _vector(data["position_m"], 3, "position_m")
            _vector(data["velocity_mps"], 3, "velocity_mps")
            _number(data["yaw_rad"], "yaw_rad")
            _number(data["yaw_rate_rad_s"], "yaw_rate_rad_s")
            _confidence(data["confidence"])
        elif kind == "pose":
            _keys(data, {"sensor_id", "from_frame", "to_frame", "quaternion_wxyz",
                         "angular_velocity_rad_s"}, "pose")
            _text(data["sensor_id"], "sensor_id")
            _choice(data["from_frame"], FRAMES, "pose from_frame")
            _choice(data["to_frame"], FRAMES, "pose to_frame")
            if data["from_frame"] == data["to_frame"]:
                raise RecordingError("pose source and destination frames must differ")
            q = _vector(data["quaternion_wxyz"], 4, "quaternion_wxyz")
            if abs(math.sqrt(sum(x*x for x in q)) - 1) > 1e-3:
                raise RecordingError("pose quaternion must already be normalized")
            if data["angular_velocity_rad_s"] is not None:
                _vector(data["angular_velocity_rad_s"], 3, "angular_velocity_rad_s")
        elif kind == "command":
            _keys(data, {"command_id", "session_id", "reference", "yaw_rad", "pitch_rad",
                         "valid_until_host_us", "control", "fire_requested"}, "command")
            identity = _text(data["command_id"], "command_id")
            if identity in self.commands:
                raise RecordingError("duplicate command_id; raw retransmissions belong in wire journal")
            _uint(data["session_id"], "session_id", (1 << 32) - 1)
            _choice(data["reference"], REFERENCES, "command reference")
            _number(data["yaw_rad"], "yaw_rad")
            _number(data["pitch_rad"], "pitch_rad")
            deadline = _uint(data["valid_until_host_us"], "valid_until_host_us")
            if timestamp["domain"] != HOST or deadline < timestamp["time_us"]:
                raise RecordingError("command generation and deadline must use host clock in order")
            _boolean(data["control"], "control")
            _boolean(data["fire_requested"], "fire_requested")
        elif kind == "feedback":
            _keys(data, {"session_id", "reference", "accepted_command_id", "applied_command_id",
                         "yaw_rad", "pitch_rad", "mode", "fault_bits", "fire_permit",
                         "shots_fired_total"}, "feedback")
            session = _uint(data["session_id"], "session_id", (1 << 32) - 1)
            _choice(data["reference"], REFERENCES, "feedback reference")
            _number(data["yaw_rad"], "yaw_rad")
            _number(data["pitch_rad"], "pitch_rad")
            _choice(data["mode"], {"safe", "manual", "vision", "calibration", "fault"}, "mode")
            _uint(data["fault_bits"], "fault_bits", (1 << 32) - 1)
            _boolean(data["fire_permit"], "fire_permit")
            if data["shots_fired_total"] is not None:
                _uint(data["shots_fired_total"], "shots_fired_total", (1 << 32) - 1)
            for field in ("accepted_command_id", "applied_command_id"):
                identity = data[field]
                if identity is not None:
                    _text(identity, field)
                    if identity not in self.commands or self.commands[identity]["session_id"] != session:
                        raise RecordingError(f"{field}: missing prior command or session mismatch")
        elif kind == "clock_epoch":
            _keys(data, {"domain", "previous_epoch", "epoch", "reason"}, "clock_epoch")
            domain = _text(data["domain"], "clock domain")
            epoch = _text(data["epoch"], "clock epoch")
            _text(data["reason"], "clock transition reason")
            if timestamp["domain"] != HOST or domain == HOST or domain not in self.clocks:
                raise RecordingError("epoch transitions require a known non-host domain and host timestamp")
            if self.clocks[domain] != data["previous_epoch"] or epoch in self.epochs[domain]:
                raise RecordingError("epoch transition must retire current epoch and introduce a new epoch")
        elif kind == "health":
            _keys(data, {"component", "status", "detail"}, "health")
            _text(data["component"], "component")
            _choice(data["status"], {"ok", "degraded", "failed"}, "health status")
            _text(data["detail"], "health detail")

    def _image(self, identity: Any) -> tuple[int, int]:
        _text(identity, "image_id")
        if identity not in self.images:
            raise RecordingError("image reference must point to an earlier image event")
        return self.images[identity]


def _check_pnm(path: Path, encoding: str, width: int, height: int) -> None:
    """Accept the deliberately small portable subset P5/P6, 8 bit, no comments."""
    channels = 1 if encoding == "pgm" else 3
    expected = f"{'P5' if channels == 1 else 'P6'}\n{width} {height}\n255\n".encode("ascii")
    try:
        with path.open("rb") as handle:
            if handle.read(len(expected)) != expected:
                raise RecordingError("PNM header disagrees with image dimensions/encoding")
        if path.stat().st_size != len(expected) + width * height * channels:
            raise RecordingError("PNM pixel byte count disagrees with dimensions")
    except OSError as exc:
        raise RecordingError(f"cannot read PNM: {exc}") from exc


class Writer:
    """Single-writer recorder. Existing JSONL/images are never overwritten."""

    def __init__(self, path: Path | str, *, run_id: str, source: str, runtime: dict,
                 clocks: dict[str, str], frames: dict[str, str], configs: dict[str, str]):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.manifest = {"schema": SCHEMA, "kind": "manifest", "run_id": run_id,
                         "source": source, "time_unit": "us", "runtime": runtime,
                         "clocks": clocks, "frames": frames,
                         "configs": [{"name": name, "content_utf8": content,
                                      "sha256": hashlib.sha256(content.encode("utf-8")).hexdigest()}
                                     for name, content in sorted(configs.items())]}
        # Round-trip before storing to detach callers' mutable dictionaries.
        self.manifest = json.loads(canonical(self.manifest))
        self._validator = _Validator(self.manifest, self.path.parent)
        encoded = canonical(self.manifest) + b"\n"
        if len(encoded) > MAX_LINE_BYTES:
            raise RecordingError("manifest exceeds line byte limit")
        self._handle = self.path.open("xb")
        self._handle.write(encoded)
        self._handle.flush()
        self._recording_digest = hashlib.sha256(encoded)

    def append(self, kind: str, *, stream: str, host_us: int, timestamp: dict, data: dict) -> dict:
        if self._handle.closed:
            raise RecordingError("writer is closed")
        event = {"kind": kind, "seq": self._validator.seq + 1, "stream": stream,
                 "host_us": host_us, "stamp": timestamp, "data": data}
        encoded = canonical(event) + b"\n"
        if len(encoded) > MAX_LINE_BYTES:
            raise RecordingError("event exceeds line byte limit")
        event = json.loads(encoded)
        self._validator.consume(event)
        try:
            self._handle.write(encoded)
            self._handle.flush()
            self._recording_digest.update(encoded)
        except OSError:
            # The last write could be partial: do not let later events hide damage.
            self._handle.close()
            raise
        return json.loads(encoded)

    def write_image(self, relative: str, pixels: bytes) -> str:
        """Store an immutable encoded image; returns SHA256 for its image event."""
        if self._handle.closed:
            raise RecordingError("writer is closed")
        if not isinstance(pixels, (bytes, bytearray, memoryview)):
            raise RecordingError("encoded image must be bytes")
        path = relative_file(self.path.parent, relative)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("xb") as handle:
            handle.write(pixels)
        return hashlib.sha256(pixels).hexdigest()

    def close(self, *, complete: bool = True) -> None:
        if not self._handle.closed:
            footer = {"kind": "end", "event_count": self._validator.seq,
                      "recording_sha256": self._recording_digest.hexdigest(),
                      "complete": complete}
            try:
                self._handle.write(canonical(footer) + b"\n")
                self._handle.flush()
            finally:
                self._handle.close()

    def __enter__(self) -> Writer:
        return self

    def __exit__(self, exc_type: Any, *_: Any) -> None:
        self.close(complete=exc_type is None)


class Reader:
    """Re-iterable validating reader. No clock sleeps, transports or control callbacks."""

    def __init__(self, path: Path | str):
        self.path = Path(path)
        self.manifest: dict | None = None
        self.recording_sha256: str | None = None

    def __iter__(self) -> Iterator[dict]:
        self.manifest = None
        self.recording_sha256 = None
        try:
            handle = self.path.open("rb")
        except OSError as exc:
            raise RecordingError(f"cannot open recording: {exc}") from exc
        with handle:
            validator = None
            line = 0
            digest = hashlib.sha256()
            ended = False
            while True:
                raw = handle.readline(MAX_LINE_BYTES + 1)
                if not raw:
                    break
                line += 1
                if len(raw) > MAX_LINE_BYTES:
                    raise RecordingError(f"line {line}: exceeds {MAX_LINE_BYTES} bytes")
                if not raw.endswith(b"\n"):
                    raise RecordingError(f"line {line}: unterminated line; recording may be truncated")
                value = _parse_json(raw, line)
                try:
                    if ended:
                        raise RecordingError("content after end footer")
                    if validator is None:
                        validator = _Validator(value, self.path.parent)
                        self.manifest = value
                        digest.update(raw)
                        continue
                    if value.get("kind") == "end":
                        _keys(value, {"kind", "event_count", "recording_sha256", "complete"}, "end footer")
                        if _uint(value["event_count"], "event_count") != validator.seq:
                            raise RecordingError("footer event count mismatch")
                        _digest(value["recording_sha256"])
                        if value["recording_sha256"] != digest.hexdigest():
                            raise RecordingError("recording hash mismatch")
                        if not _boolean(value["complete"], "complete"):
                            raise RecordingError("writer marked recording incomplete")
                        ended = True
                        continue
                    validator.consume(value)
                    digest.update(raw)
                except (RecordingError, UnicodeError, OverflowError) as exc:
                    raise RecordingError(f"line {line}: {exc}") from exc
                yield value
            if validator is None:
                raise RecordingError("recording is empty: missing manifest")
            if not ended:
                raise RecordingError("missing end footer; recording was not closed or was truncated")
            self.recording_sha256 = digest.hexdigest()


def replay_digest(events: Iterable[dict]) -> dict:
    """Replay every event in recorded order and fingerprint its canonical content."""
    digest = hashlib.sha256()
    count = 0
    for event in events:
        digest.update(canonical(event) + b"\n")
        count += 1
    return {"event_count": count, "event_sha256": digest.hexdigest(),
            "execution": "read-only event replay; no vision algorithm or control output"}


def _summary(values: list[float]) -> dict | None:
    if not values:
        return None
    values = sorted(values)
    return {"count": len(values), "min": values[0], "median": statistics.median(values),
            "p95": values[max(0, math.ceil(.95 * len(values)) - 1)], "max": values[-1]}


def inspect(path: Path | str) -> dict:
    """Integrity and timing report; no detection accuracy or hit-rate inference."""
    reader = Reader(path)
    counts: Counter = Counter()
    source_domains: Counter = Counter()
    latencies: dict[str, list[float]] = defaultdict(list)
    uncertainties: dict[str, list[float]] = defaultdict(list)
    commands: dict[str, dict] = {}
    accepted, applied = set(), set()
    yaw_errors, pitch_errors, feedback_ages = [], [], []
    duplicate_times = 0
    previous_times = {}
    uncertain = unmapped = fault_samples = reference_mismatches = expired_applies = 0
    first = last = None
    image_mapped_times: list[int] = []
    pose_mapped_times: dict[tuple, list[int]] = defaultdict(list)
    digest = hashlib.sha256()
    for event in reader:
        digest.update(canonical(event) + b"\n")
        counts[event["kind"]] += 1
        host_us = event["host_us"]
        first = host_us if first is None else first
        last = host_us
        ts, data = event["stamp"], event["data"]
        source_domains[ts["domain"]] += 1
        key = (event["stream"], ts["domain"], ts["epoch"])
        duplicate_times += previous_times.get(key) == ts["time_us"]
        previous_times[key] = ts["time_us"]
        mapped = ts["mapped_host_us"]
        if mapped is None:
            unmapped += 1
        else:
            latencies[event["kind"]].append(host_us - mapped)
            uncertainties[ts["domain"]].append(ts["uncertainty_us"])
            uncertain += ts["uncertainty_us"] > 5000
            if ts["uncertainty_us"] <= 5000:
                if event["kind"] == "image":
                    image_mapped_times.append(mapped)
                if event["kind"] == "pose":
                    pose_key = (data["sensor_id"], data["from_frame"], data["to_frame"], ts["domain"], ts["epoch"])
                    pose_mapped_times[pose_key].append(mapped)
        if event["kind"] == "command":
            commands[data["command_id"]] = {"data": data, "time_us": ts["time_us"]}
        elif event["kind"] == "feedback":
            fault_samples += data["fault_bits"] != 0
            if data["accepted_command_id"] is not None:
                accepted.add(data["accepted_command_id"])
            identity = data["applied_command_id"]
            if identity is not None:
                applied.add(identity)
                command = commands[identity]
                intended = command["data"]
                # Header receipt time is not the control loop's execution time.
                if mapped is not None and ts["uncertainty_us"] <= 5000:
                    feedback_ages.append(mapped - command["time_us"])
                    expired_applies += mapped - ts["uncertainty_us"] > intended["valid_until_host_us"]
                if data["reference"] == intended["reference"]:
                    yaw_errors.append(math.remainder(data["yaw_rad"] - intended["yaw_rad"], 2 * math.pi))
                    pitch_errors.append(data["pitch_rad"] - intended["pitch_rad"])
                else:
                    reference_mismatches += 1
    assert reader.manifest is not None
    # Availability is about recorded sample brackets, never proof of calibration validity.
    from bisect import bisect_left
    brackets = []
    for key, samples in sorted(pose_mapped_times.items()):
        samples.sort()
        bracketed = 0
        for timestamp in image_mapped_times:
            index = bisect_left(samples, timestamp)
            exact = index < len(samples) and samples[index] == timestamp
            if exact or (0 < index < len(samples) and samples[index] - samples[index - 1] <= 50000):
                bracketed += 1
        brackets.append({"sensor_id": key[0], "from_frame": key[1], "to_frame": key[2],
                         "domain": key[3], "epoch": key[4],
                         "images_with_pose_bracket_at_most_50ms": bracketed})
    return {"schema": "sp-vision-replay-report-1", "run_id": reader.manifest["run_id"],
            "source": reader.manifest["source"], "integrity": "passed",
            "recording_sha256": reader.recording_sha256,
            "event_count": sum(counts.values()), "event_sha256": digest.hexdigest(),
            "duration_us": 0 if first is None else last - first,
            "event_counts": dict(sorted(counts.items())), "clock_domains": dict(sorted(source_domains.items())),
            "unmapped_events": unmapped, "mapping_uncertainty_over_5ms_events": uncertain,
            "duplicate_source_timestamps": duplicate_times,
            "ingress_delay_us": {key: _summary(value) for key, value in sorted(latencies.items())},
            "mapping_uncertainty_us": {key: _summary(value) for key, value in sorted(uncertainties.items())},
            "image_pose_brackets": {"images_with_mapping_at_most_5ms": len(image_mapped_times),
                                    "by_sensor_frames_and_epoch": brackets},
            "commands": {"recorded": len(commands), "accepted_by_feedback": len(accepted),
                         "applied_by_feedback": len(applied), "without_applied_feedback": len(set(commands) - applied),
                         "reference_mismatch_feedback_samples": reference_mismatches,
                         "feedback_samples_reporting_old_command_after_deadline": expired_applies,
                         "feedback_sample_minus_generation_us": _summary(feedback_ages),
                         "yaw_feedback_minus_command_rad": _summary(yaw_errors),
                         "pitch_feedback_minus_command_rad": _summary(pitch_errors)},
            "fault_feedback_samples": fault_samples,
            "limitations": ["Event replay does not execute detector, estimator, planner or control.",
                            "Recorded command intent is not measured motion, applied feedback, or a shot.",
                            "Latency uses declared clock mappings; it does not measure USB or exposure timing.",
                            "Pose brackets are grouped by sensor, frames and epoch; they do not validate mounting or extrinsics.",
                            "PNG/JPEG hashes are checked; pixel decoding and declared dimensions need an image backend.",
                            "Synthetic reports do not measure real detection accuracy, stability or hit rate."]}

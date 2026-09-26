"""Deterministic synthetic images/events; no model and no physical device is involved."""

from __future__ import annotations

import math
import platform
from pathlib import Path

from .recording import HOST, Writer, stamp

FRAMES = {
    "world": "Synthetic fixed right-handed frame: x forward, y left, z up; metres.",
    "gimbal": "Synthetic rigid camera support: x forward, y left, z up.",
    "camera": "Synthetic optical frame: x right, y down, z forward.",
    "chassis": "Synthetic fixed chassis, coincident with world; no real mounting claim.",
}


def make_demo(path: Path | str, config_path: Path | str | None = None) -> Path:
    """Create a reproducible 12-frame PGM recording, refusing to replace existing data."""
    path = Path(path)
    config = (Path(config_path).read_text(encoding="utf-8") if config_path is not None
              else "# Synthetic replay fixture, not a robot operating configuration.\nframes: 12\n")
    configs = {"configs/standard.yaml" if config_path else "synthetic_fixture": config}
    with Writer(path, run_id="synthetic-replay-demo", source="synthetic",
                runtime={"application": "tools.replay.demo", "revision": "synthetic-fixture-v1",
                         "build": "python-stdlib", "platform": platform.system()},
                clocks={HOST: "host-1", "mcu_monotonic": "mcu-boot-1", "camera_device": "camera-1"},
                frames=FRAMES, configs=configs) as writer:
        for index in range(12):
            t = 1_000_000 + index * 20_000
            yaw = index * .01
            image_id = f"image-{index:04d}"
            command_id = f"aim-{index:04d}"
            # These are generated samples of known synthetic truth, not detector results.
            writer.append("pose", stream="pose/small_y", host_us=t,
                          timestamp=stamp(t + 4_000_000, domain="mcu_monotonic", epoch="mcu-boot-1",
                                          mapped_host_us=t, uncertainty_us=100),
                          data={"sensor_id": "synthetic_camera_support", "from_frame": "gimbal",
                                "to_frame": "world", "quaternion_wxyz": [math.cos(yaw/2), 0, 0, math.sin(yaw/2)],
                                "angular_velocity_rad_s": [0, 0, .5]})
            width, height = 32, 24
            left, right, top, bottom = 4 + index, 13 + index, 7, 16
            pixels = bytearray(width * height)
            for y in range(top, bottom + 1):
                for x in range(left, right + 1):
                    pixels[y * width + x] = 220 if x in (left, right) else 80
            encoded = f"P5\n{width} {height}\n255\n".encode("ascii") + pixels
            relative = f"images/{image_id}.pgm"
            digest = writer.write_image(relative, encoded)
            writer.append("image", stream="camera/main", host_us=t + 2_000,
                          timestamp=stamp(30_000 + index * 20_000, domain="camera_device", epoch="camera-1",
                                          mapped_host_us=t + 1_000, uncertainty_us=100),
                          data={"image_id": image_id, "path": relative, "sha256": digest,
                                "width": width, "height": height, "encoding": "pgm"})
            writer.append("detections", stream="detector/synthetic", host_us=t + 3_000,
                          timestamp=stamp(t + 1_000), data={"image_id": image_id, "objects": [
                              {"label": "synthetic-armor", "color": "unknown", "confidence": 1.0,
                               "corners_px": [[left, top], [right, top], [right, bottom], [left, bottom]]}]})
            writer.append("target", stream="target/synthetic", host_us=t + 4_000,
                          timestamp=stamp(t + 1_000), data={"target_id": "synthetic-target", "image_id": image_id,
                              "frame": "world", "position_m": [5, .1 * index, 0], "velocity_mps": [0, 5, 0],
                              "yaw_rad": yaw, "yaw_rate_rad_s": .5, "confidence": 1.0})
            writer.append("command", stream="control/intended", host_us=t + 5_000,
                          timestamp=stamp(t + 5_000), data={"command_id": command_id, "session_id": 1,
                              "reference": "gimbal_joint", "yaw_rad": yaw, "pitch_rad": .1,
                              "valid_until_host_us": t + 85_000, "control": True, "fire_requested": False})
            # Accepted and applied are independent values; the third intent has no applied feedback.
            applied = None if index == 2 else command_id
            writer.append("feedback", stream="control/feedback", host_us=t + 8_000,
                          timestamp=stamp(t + 4_007_000, domain="mcu_monotonic", epoch="mcu-boot-1",
                                          mapped_host_us=t + 7_000, uncertainty_us=100),
                          data={"session_id": 1, "reference": "gimbal_joint", "accepted_command_id": command_id,
                                "applied_command_id": applied, "yaw_rad": yaw - .005, "pitch_rad": .099,
                                "mode": "calibration", "fault_bits": 0, "fire_permit": False,
                                "shots_fired_total": 0})
        writer.append("health", stream="recorder/health", host_us=t + 9_000, timestamp=stamp(t + 9_000),
                      data={"component": "synthetic_fixture", "status": "ok",
                            "detail": "No camera, neural model, estimator, motor or firing hardware was used."})
    return path

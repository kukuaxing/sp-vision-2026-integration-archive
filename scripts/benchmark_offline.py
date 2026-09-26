#!/usr/bin/env python3
"""Build actual estimator headers and report deterministic synthetic comparisons.

No camera, device I/O, ROS, WSL or network. Serial compilation uses the same
compiler discovery as check_offline.py. Generated outputs are ignored by Git.
"""
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
from collections import defaultdict
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.check_offline import compiler_environment


def finite_metrics(values):
    values = [float(value) for value in values if math.isfinite(float(value))]
    if not values:
        return {"count": 0, "rmse": None, "p95_absolute": None, "max_absolute": None}
    ordered = sorted(map(abs, values))
    index = 0.95 * (len(ordered) - 1)
    lo, hi = math.floor(index), math.ceil(index)
    return {"count": len(values), "rmse": math.sqrt(sum(v * v for v in values) / len(values)),
            "p95_absolute": ordered[lo] + (index - lo) * (ordered[hi] - ordered[lo]),
            "max_absolute": ordered[-1]}


def settling_time(rows, event, column, threshold, dwell=20):
    """First 20 consecutive valid samples within a diagnostic tolerance.

    Returns elapsed time to the START of the successful 0.20 s window. The
    complete window must follow the event; no interpolation across missing rows.
    """
    count = 0
    previous = None
    for row in rows:
        time = float(row["time_s"])
        if time < event:
            continue
        contiguous = previous is None or abs(time - previous - 0.01) < 1e-8
        previous = time
        value = float(row[column])
        if contiguous and row["valid"] == "1" and math.isfinite(value) and abs(value) <= threshold:
            count += 1
        else:
            count = 1 if (row["valid"] == "1" and math.isfinite(value) and abs(value) <= threshold) else 0
        if count >= dwell:
            return max(0.0, round(time - (dwell - 1) * 0.01 - event, 8))
    return None


def configuration_header():
    """Read only named scalar numeric keys; refuse duplicate/missing keys.

    This is deliberately not a replacement YAML parser. The canonical config
    remains the source of these scalar values, and unsupported forms fail loudly.
    """
    source = (ROOT / "configs/standard.yaml").read_text(encoding="utf-8-sig")
    values = {}

    def number(name):
        matches = re.findall(r"^" + re.escape(name) + r":\s*([^#\r\n]+)", source, re.MULTILINE)
        if len(matches) != 1:
            raise ValueError(f"Expected one scalar value for {name}")
        value = float(matches[0].strip())
        if not math.isfinite(value):
            raise ValueError(f"Nonfinite value for {name}")
        values[name] = value
        return format(value, ".17g")

    yaw_map = {
        "angular_jerk_variance": "yaw_eskf_angular_jerk_variance",
        "nis_reject_threshold": "yaw_eskf_nis_reject_threshold",
        "min_trusted_updates": "yaw_eskf_min_trusted_updates",
        "max_trusted_abs_rate_rad_s": "yaw_eskf_max_trusted_abs_rate_rad_s",
        "max_trusted_abs_acceleration_rad_s2": "yaw_eskf_max_trusted_abs_acceleration_rad_s2",
    }
    lines = ["inline auto_aim::YawESKF::Config benchmark_yaw_config() {", "auto_aim::YawESKF::Config c;"]
    for field, key in yaw_map.items():
        lines.append(f"c.{field} = {number(key)};")
    lines.append(f"c.measurement_variance_floor = std::pow({number('yaw_eskf_measurement_floor_deg')} * 3.14159265358979323846 / 180.0, 2);")
    lines.append(f"c.trusted_rate_variance = std::pow({number('yaw_eskf_trusted_rate_std_rad_s')}, 2);")
    lines.extend(["return c;", "}", "inline auto_aim::RobustRangeFilter::Config benchmark_range_config() {",
                  "auto_aim::RobustRangeFilter::Config c;"])
    lines.append(f"c.time_constant_s = {number('xuc_range_filter_tau_ms')} / 1000.0;")
    for field, key in (("max_rate_mps", "xuc_range_filter_max_rate_mps"),
                       ("raw_blend", "xuc_range_filter_raw_blend"),
                       ("max_raw_model_residual_m", "xuc_range_filter_max_raw_residual_m")):
        lines.append(f"c.{field} = {number(key)};")
    lines.extend(["return c;", "}"])
    return "\n".join(lines) + "\n", values


def summarize(rows):
    grouped = defaultdict(list)
    for row in rows:
        if row["valid"] == "1" and not math.isfinite(float(row["error"])):
            raise ValueError(f"Valid output has nonfinite error: {row}")
        if float(row["time_s"]) >= 1.0:  # same declared warm-up exclusion for every method
            grouped[(row["family"], row["scenario"], row["method"])].append(row)
    summaries = []
    for (family, scenario, method), group in sorted(grouped.items()):
        trusted = [row for row in group if row["valid"] == "1"]
        summary = {"family": family, "scenario": scenario, "method": method, "rows": len(group),
                   "observed": sum(row["observed"] == "1" for row in group),
                   "accepted": sum(row["accepted"] == "1" for row in group),
                   "valid_outputs": len(trusted), "valid_fraction": len(trusted) / len(group)}
        for column in ("error", "rate_error", "forecast_error", "equivalent_error", "equivalent_forecast_error"):
            summary[column] = finite_metrics(row[column] for row in group)
            summary["valid_" + column] = finite_metrics(row[column] for row in trusted)
        summary["phase_error_rmse"] = {
            phase: finite_metrics(row["error"] for row in group if row["phase"] == phase)["rmse"]
            for phase in ("before", "stress", "recovery", "late")}
        event = None
        if family == "yaw" and scenario == "rate_ramp":
            event = (3.0, "rate_error", 0.25)
        elif family == "armor" and scenario == "velocity_ramp":
            event = (3.0, "rate_error", 0.15)
        elif family == "armor" and scenario == "target_switch":
            event = (2.5, "error", 0.10)
        elif family == "range" and scenario == "model_bias":
            event = (6.0, "error", 0.05)
        if event:
            summary["settling"] = {"event_s": event[0], "metric": event[1], "threshold": event[2],
                                   "dwell_samples": 20, "seconds_by_seed": {
                seed: settling_time([row for row in group if row["seed"] == seed], *event)
                for seed in sorted({row["seed"] for row in group})}}
        summaries.append(summary)
    return summaries


def markdown_report(summaries, digest):
    lines = ["# Synthetic estimator benchmark", "",
             "Actual production headers; deterministic synthetic inputs. No device I/O. These numbers are not robot accuracy or hit rate.", "",
             "All methods receive the same observations. 100 Hz, 8 s, seeds 17/2026/98173; statistics exclude the first 1 s. Forecast horizon is 0.40 s.",
             "Angles/errors use rad and rad/s; armor and range errors use m and m/s. Armor error is a 3D norm. RMSE includes untrusted outputs, while validity is reported separately.", "",
             "`valid` means estimator trust for yaw/armor (baseline has no trust gate), and a fresh valid observation for range. A higher valid count alone does not mean better safety or accuracy.", "",
             "| Family | Scenario | Method | Error RMSE | Rate RMSE | Forecast RMSE | Valid / total | Accepted / observed |",
             "|---|---|---|---:|---:|---:|---:|---:|"]

    def fmt(value):
        return "—" if value is None else f"{value:.5f}"

    for row in summaries:
        lines.append(f"| {row['family']} | {row['scenario']} | {row['method']} | "
                     f"{fmt(row['error']['rmse'])} | {fmt(row['rate_error']['rmse'])} | "
                     f"{fmt(row['forecast_error']['rmse'])} | {row['valid_outputs']} / {row['rows']} | "
                     f"{row['accepted']} / {row['observed']} |")
    lines.extend(["", "## Yaw equivalent-face errors", "",
                  "Absolute body yaw is not fully observable when all four faces are interchangeable. The main table uses modulo 2π error; this table also uses modulo π/2 error to distinguish face-branch ambiguity from geometric phase error. Neither metric alone validates production association for unequal faces.", "",
                  "| Scenario | Method | Face-period RMSE | Face-period forecast RMSE |",
                  "|---|---|---:|---:|"])
    for row in summaries:
        if row["family"] == "yaw":
            lines.append(f"| {row['scenario']} | {row['method']} | {fmt(row['equivalent_error']['rmse'])} | "
                         f"{fmt(row['equivalent_forecast_error']['rmse'])} |")
    lines.extend(["", "## Response diagnostics", "",
                  "Tolerances below describe settling time; they are not acceptance thresholds. Values are seconds after the event to the start of 20 consecutive valid samples in tolerance. null means never settled in this recording.", ""])
    for row in summaries:
        if "settling" in row:
            s = row["settling"]
            lines.append(f"- {row['family']}/{row['scenario']}/{row['method']}: {s['metric']} ≤ {s['threshold']}; "
                         f"event {s['event_s']} s; {json.dumps(s['seconds_by_seed'])}")
    lines.extend(["", "## Limits", "",
                  "The range filter receives a synthetic model observation, not the vehicle EKF output. Its input noise and model bias are stipulated, not measured. Alpha-beta gains (0.35, 0.04) and EMA tau (0.08 s) are fixed illustrative baselines, not tuned competitors.",
                  "Yaw measurements assume correct body-yaw association modulo a face pitch. Armor measurements assume an identified 3D physical face. Detector failures, PnP correlations, clock skew, image/IMU alignment, ballistic error and physical control are absent.",
                  "The target_switch scene intentionally supplies a new physical position to the old filter to reveal reacquisition behavior; it does not test production target association. Future truth discontinuities make some forecast error unavoidable.",
                  "Delayed yaw observations are deliberately passed as current observations, exposing timestamp misuse; this harness supplies no latency compensator.",
                  "No performance pass/fail threshold is asserted and no production tuning is changed from these synthetic results.", "",
                  f"Two consecutive executions produced identical CSV bytes: SHA-256 `{digest}`. Source hashes, configuration and compiler are recorded in summary.json.", ""])
    return "\n".join(lines)


def main():
    build = ROOT / "build/offline"
    output = ROOT / "logs/benchmark"
    build.mkdir(parents=True, exist_ok=True)
    output.mkdir(parents=True, exist_ok=True)
    generated, config = configuration_header()
    (build / "benchmark_config.hpp").write_text(generated, encoding="utf-8")
    compiler, environment, msvc = compiler_environment()
    binary = build / ("estimators_benchmark.exe" if os.name == "nt" else "estimators_benchmark")
    source = "tests/benchmark/estimators.cpp"
    if msvc:
        command = [compiler, "/nologo", "/utf-8", "/std:c++17", "/EHsc", "/O2", "/W3", "/D_USE_MATH_DEFINES",
                   "/I.", "/Ilibrm/libs/eigen", "/Ibuild/offline", source,
                   "/Fe:" + str(binary), "/Fo:" + str(build / "estimators_benchmark.obj")]
    else:
        command = [compiler, "-std=c++17", "-O2", "-I.", "-Ilibrm/libs/eigen", "-Ibuild/offline", source, "-o", str(binary)]
    print("Building synthetic estimator benchmark (one compiler process)...", flush=True)
    subprocess.run(command, cwd=ROOT, env=environment, check=True)
    first = subprocess.check_output([str(binary)], cwd=ROOT, env=environment, timeout=60)
    second = subprocess.check_output([str(binary)], cwd=ROOT, env=environment, timeout=60)
    if first != second:
        raise RuntimeError("Benchmark is not byte-for-byte repeatable")
    rows = list(csv.DictReader(io.StringIO(first.decode("ascii"))))
    if len(rows) != 88800:
        raise ValueError(f"Expected 88800 rows, got {len(rows)}")
    summaries = summarize(rows)
    digest = hashlib.sha256(first).hexdigest()
    tracked = [source, "scripts/benchmark_offline.py", "configs/standard.yaml",
               "tasks/auto_aim/yaw_eskf.hpp", "tasks/auto_aim/armor_motion_filter.hpp", "tasks/auto_aim/robust_range_filter.hpp"]
    report = {"schema": "synthetic-estimator-benchmark-1", "generated_utc": datetime.now(timezone.utc).isoformat(),
              "hardware": False, "compiler": compiler, "command": command, "seed_list": [17, 2026, 98173],
              "sample_period_s": 0.01, "duration_s": 8.0, "warmup_excluded_s": 1.0, "forecast_s": 0.40,
              "configuration": config, "rows": len(rows), "csv_sha256": digest, "repeatable": True,
              "source_sha256": {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in tracked},
              "summaries": summaries}
    (output / "samples.csv").write_bytes(first)
    (output / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (output / "report.md").write_text(markdown_report(summaries, digest), encoding="utf-8")
    print(f"PASS: {len(rows)} synthetic rows; {len(summaries)} comparisons; exact repeatability verified.")
    print(output / "report.md")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import re
import statistics
import sys
from datetime import datetime


path = sys.argv[1]
text = open(path, encoding="utf-8", errors="replace").read()


def ts(line):
    match = re.search(r"\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]", line)
    return datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S.%f") if match else None


active = None
qualified_in_window = False
reset_durations = []
qualified = []
starts = 0
resets = 0

for line in text.splitlines():
    if "stable-window candidate started" in line:
        active = ts(line)
        qualified_in_window = False
        starts += 1
    elif "[XUC][FIRE] qualified" in line:
        match = re.search(
            r"qualified after ([0-9.]+) ms.*current_pixel_error_deg=\(([-+0-9.]+),([-+0-9.]+)\) "
            r"predicted_impact_yaw_error_deg=([-+0-9.]+).*omega=([-+0-9.]+)rad_s",
            line,
        )
        if match:
            qualified.append(tuple(float(value) for value in match.groups()))
        qualified_in_window = True
    elif "stable window reset" in line:
        resets += 1
        current = ts(line)
        if active and current and not qualified_in_window:
            reset_durations.append((current - active).total_seconds() * 1000.0)
        active = None
        qualified_in_window = False

fps = [float(value) for value in re.findall(r"平均帧率=([0-9.]+)fps", text)]
armor_counts = [int(value) for value in re.findall(r"\[DEBUG\] armors=(\d+)", text)]

print(f"candidate_starts={starts}")
print(f"qualified={len(qualified)}")
print(f"resets={resets}")
if reset_durations:
    buckets = {
        "<20ms": sum(value < 20 for value in reset_durations),
        "20-30ms": sum(20 <= value < 30 for value in reset_durations),
        "30-45ms": sum(30 <= value < 45 for value in reset_durations),
        ">=45ms": sum(value >= 45 for value in reset_durations),
    }
    print("unqualified_window_ms=" + ", ".join(f"{key}:{value}" for key, value in buckets.items()))
    print(f"unqualified_median_ms={statistics.median(reset_durations):.1f}")
if qualified:
    print(f"qualified_stable_ms={[row[0] for row in qualified]}")
    print(f"qualified_current_yaw_deg={[row[1] for row in qualified]}")
    print(f"qualified_predicted_yaw_deg={[row[3] for row in qualified]}")
    print(f"qualified_omega_rad_s={[row[4] for row in qualified]}")
if fps:
    print(f"fps_avg_range={min(fps):.1f}-{max(fps):.1f}")
if armor_counts:
    detected = sum(value > 0 for value in armor_counts)
    print(f"armor_detected_frames={detected}/{len(armor_counts)} ({100*detected/len(armor_counts):.1f}%)")

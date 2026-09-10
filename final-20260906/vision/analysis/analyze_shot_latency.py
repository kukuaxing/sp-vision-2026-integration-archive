#!/usr/bin/env python3
import re
import statistics
import sys
from datetime import datetime


def parse_ts(line):
    match = re.search(r"\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]", line)
    return datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S.%f") if match else None


lines = open(sys.argv[1], encoding="utf-8", errors="replace").read().splitlines()
qualified = []
feeder = []
last_rpm = 0.0
for line in lines:
    stamp = parse_ts(line)
    if not stamp:
        continue
    if "[XUC][FIRE] qualified" in line and "gyro_predict=true" in line:
        qualified.append(stamp)
    rpm_match = re.search(r"feeder_rpm=([-+0-9.]+)", line)
    if rpm_match:
        rpm = float(rpm_match.group(1))
        if rpm > 500.0 and last_rpm <= 500.0:
            feeder.append(stamp)
        last_rpm = rpm

delays = []
for shot in qualified:
    later = [(event - shot).total_seconds() * 1000 for event in feeder if event >= shot]
    if later and later[0] <= 250:
        delays.append(later[0])

print(f"software_qualifications={len(qualified)}")
print(f"feeder_rising_edges={len(feeder)}")
print(f"matched_delays={len(delays)}")
if delays:
    print(f"qualification_to_feeder_ms_min_median_max={min(delays):.0f}/{statistics.median(delays):.0f}/{max(delays):.0f}")
    print("delays_ms=" + ",".join(f"{value:.0f}" for value in delays))

#!/usr/bin/env bash
set -euo pipefail
NAV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! ip -4 addr show dev enp3s0 | grep -q 'inet 192.168.1.2/24'; then
  printf '%s\n' 'Missing radar interface address 192.168.1.2/24 on enp3s0. Restore the configured Ethernet profile first.' >&2
  exit 2
fi
exec 9>"$NAV_ROOT/logs/lidar_driver.lock"
if ! flock -n 9; then printf '%s\n' 'MID-360 driver is already starting/running.' >&2; exit 2; fi
if [[ -r "$NAV_ROOT/logs/lidar_driver.pid" ]]; then
  old_pid="$(cat "$NAV_ROOT/logs/lidar_driver.pid")"
  if [[ "$old_pid" =~ ^[0-9]+$ && "$old_pid" != "$$" ]] && kill -0 "$old_pid" 2>/dev/null && grep -aq 'mid360_observe.launch.py' "/proc/$old_pid/cmdline"; then
    printf '%s\n' "MID-360 driver already running: $old_pid" >&2
    exit 2
  fi
fi
printf '%s\n' "$$" > "$NAV_ROOT/logs/lidar_driver.pid"
set +u
source /opt/ros/humble/setup.bash
source "$NAV_ROOT/install/livox_driver/local_setup.bash"
source "$NAV_ROOT/install/localization/local_setup.bash"
set -u
export LD_LIBRARY_PATH="$NAV_ROOT/install/livox_sdk/lib:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR="$NAV_ROOT/logs/lidar_ros"
cd "$NAV_ROOT/real_bringup"
exec ros2 launch ./mid360_observe.launch.py user_config_path:="$NAV_ROOT/real_bringup/MID360.vehicle.json" frame_id:=livox_frame xfer_format:=1
